#!/bin/bash
# Build MicroPython with gps_logger frozen into the firmware, and optionally
# flash it. Frozen, the logger's bytecode runs from flash instead of taking
# ~60 KB of the Python heap: the heap starts at 64 KB with ~38 KB live, so
# far fewer allocations fail to fit and grow it into the ESP-IDF heap that
# TLS uploads need (it still can: MicroPython's split heap stays on).
#
# Needs ESP-IDF (v5.5.2 for MicroPython 1.29) and a MicroPython v1.29.0 tree:
#   IDF_DIR=~/esp/esp-idf  MPY_DIR=~/esp/micropython  (defaults)
# Usage:
#   tools/build_firmware.sh                    build only
#   PORT=/dev/ttyACM0 tools/build_firmware.sh --flash
# --flash checks that the board's partition table matches the build's,
# backs up the board's app partition to firmware/, writes only the app
# (0x10000; bootloader, partition table and the file system are kept) and
# resets. Nothing on the file system is touched before or by the write: a
# failed check or write leaves the board as it was. Then run ./deploy.sh: it
# removes the old /gps_logger.mpy (until then it shadows the frozen logger
# and runs as before) and any /gps_logger.py, clears the boot-fail counter
# and puts main.py and the page up. Right after esptool's reset the raw
# REPL may not come for a while: if deploy.sh cannot reach it, run it again.
# firmware/gps_frozen.py records the SHA-256 of the frozen gps_logger.py
# (deploy.sh stops when it differs from the one in the repo) and the
# commit, "-dirty" when the tree had uncommitted changes.
set -euo pipefail
cd "$(dirname "$0")/.."

IDF_DIR=${IDF_DIR:-$HOME/esp/esp-idf}
MPY_DIR=${MPY_DIR:-$HOME/esp/micropython}
BOARD=${BOARD:-ESP32_GENERIC_C6}
CHIP=${CHIP:-esp32c6}
PORT=${PORT:-/dev/ttyUSB0}
OUT=$PWD/firmware

mkdir -p "$OUT"
# the frozen files: the logger and a marker deploy.sh looks for
cp gps_logger.py "$OUT/gps_logger.py"
COMMIT=$(git rev-parse --short HEAD 2>/dev/null || echo unknown)
if [ "$COMMIT" != unknown ] && [ -n "$(git status --porcelain --untracked-files=no 2>/dev/null)" ]; then
    COMMIT=$COMMIT-dirty
fi
SHA=$(sha256sum "$OUT/gps_logger.py" | cut -d" " -f1)
printf 'COMMIT = "%s"\nSHA = "%s"\n' "$COMMIT" "$SHA" > "$OUT/gps_frozen.py"
cat > "$OUT/manifest.py" <<EOF
include("\$(PORT_DIR)/boards/manifest.py")
module("gps_logger.py", base_path="$OUT")
module("gps_frozen.py", base_path="$OUT")
EOF

# shellcheck disable=SC1091
source "$IDF_DIR/export.sh" >/dev/null
make -C "$MPY_DIR/mpy-cross" >/dev/null
make -C "$MPY_DIR/ports/esp32" BOARD="$BOARD" FROZEN_MANIFEST="$OUT/manifest.py" -j"$(nproc)" >/dev/null
BIN=$MPY_DIR/ports/esp32/build-$BOARD/micropython.bin
cp "$BIN" "$OUT/micropython.bin"
echo "Built $OUT/micropython.bin ($(wc -c < "$BIN") bytes, logger $COMMIT, sha256 $SHA)"

[ "${1:-}" = "--flash" ] || exit 0

# the board's partition table must match the build's (only the app is written)
esptool.py --chip "$CHIP" --port "$PORT" -b 460800 read_flash 0x8000 0xc00 "$OUT/pt_board.bin" >/dev/null
if ! cmp -s "$OUT/pt_board.bin" "$MPY_DIR/ports/esp32/build-$BOARD/partition_table/partition-table.bin"; then
    echo "The board's partition table differs from the build's: not flashing" >&2
    exit 1
fi
BACKUP=$OUT/app-backup-$(date +%Y%m%d-%H%M%S).bin
esptool.py --chip "$CHIP" --port "$PORT" -b 460800 read_flash 0x10000 0x1F0000 "$BACKUP" >/dev/null
echo "App partition backed up to $BACKUP"
esptool.py --chip "$CHIP" --port "$PORT" -b 460800 --after hard_reset write_flash \
    --flash_mode dio --flash_freq 80m 0x10000 "$OUT/micropython.bin"
echo "Flashed $PORT. Now run ./deploy.sh (again if the board's REPL does not answer yet):"
echo "it removes the old gps_logger.mpy that still shadows the frozen logger."
echo "Roll back with: esptool.py --chip $CHIP --port $PORT write_flash 0x10000 $BACKUP, then ./deploy.sh"
