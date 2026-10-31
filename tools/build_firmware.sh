#!/bin/bash
# Build MicroPython with gps_logger frozen into the firmware, and optionally
# flash it. Frozen, the logger's bytecode runs from flash instead of taking
# ~60 KB of the Python heap: the heap then stays at its 64 KB start and never
# grows into the ESP-IDF heap that TLS uploads need.
#
# Needs ESP-IDF (v5.5.2 for MicroPython 1.29) and a MicroPython v1.29.0 tree:
#   IDF_DIR=~/esp/esp-idf  MPY_DIR=~/esp/micropython  (defaults)
# Usage:
#   tools/build_firmware.sh                    build only
#   PORT=/dev/ttyACM0 tools/build_firmware.sh --flash
# --flash backs up the board's app partition to firmware/, removes the
# gps_logger.mpy that would shadow the frozen module, writes only the app
# (0x10000; bootloader, partition table and the file system are kept) and
# resets. Run ./deploy.sh afterwards for main.py and web.html.
set -euo pipefail
cd "$(dirname "$0")/.."

IDF_DIR=${IDF_DIR:-$HOME/esp/esp-idf}
MPY_DIR=${MPY_DIR:-$HOME/esp/micropython}
BOARD=${BOARD:-ESP32_GENERIC_C6}
CHIP=${CHIP:-esp32c6}
PORT=${PORT:-/dev/ttyUSB0}
OUT=$PWD/firmware
REPO=$PWD

mkdir -p "$OUT"
# the frozen files: the logger and a marker deploy.sh looks for
cp gps_logger.py "$OUT/gps_logger.py"
printf 'COMMIT = "%s"\n' "$(git rev-parse --short HEAD 2>/dev/null || echo unknown)" > "$OUT/gps_frozen.py"
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
echo "Built $OUT/micropython.bin ($(wc -c < "$BIN") bytes, logger $(cat "$OUT/gps_frozen.py"))"

[ "${1:-}" = "--flash" ] || exit 0

# First, while the logger runs undisturbed (right after an esptool reset the
# raw REPL does not come): remove the .mpy that would shadow the frozen
# module and clear the boot-fail counter. resume: no soft reset.
for try in 1 2 3; do
    if timeout 60 "$REPO/.venv/bin/mpremote" connect "$PORT" resume exec "
import os
'gps_logger.mpy' in os.listdir() and os.remove('gps_logger.mpy')
from machine import RTC
RTC().memory(b'0')
print('board ready')" | grep -q "board ready"; then
        break
    fi
    [ "$try" = 3 ] && { echo "Could not reach the board's REPL: not flashing" >&2; exit 1; }
    sleep 15
done
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
echo "Flashed $PORT. Roll back with: esptool.py --chip $CHIP --port $PORT write_flash 0x10000 $BACKUP"
