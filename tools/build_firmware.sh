#!/bin/bash
# Build MicroPython with gps_logger frozen into the firmware, and optionally
# flash or publish it. Frozen, the logger's bytecode runs from flash instead
# of taking ~60 KB of the Python heap: the heap starts at 64 KB with ~38 KB
# live, so far fewer allocations fail to fit and grow it into the ESP-IDF
# heap that TLS uploads need (it still can: MicroPython's split heap stays on).
#
# Needs ESP-IDF (v5.5.2 for MicroPython 1.29) and a MicroPython v1.29.0 tree:
#   IDF_DIR=~/esp/esp-idf  MPY_DIR=~/esp/micropython  (defaults)
# The board is boards/GPS_LOGGER_C6: ESP32_GENERIC_C6 with two OTA app slots
# (partitions-8MiBplus-ota.csv, 16 MB flash). BOARD_DIR picks another.
# Usage:
#   tools/build_firmware.sh                    build only
#   PORT=/dev/ttyACM0 tools/build_firmware.sh --flash
#   PORT=/dev/ttyACM0 tools/build_firmware.sh --migrate
#   OTA_DEST=host:/srv/ota/gps tools/build_firmware.sh --publish
# --flash checks that the board's partition table matches the build's,
# backs up the board's first app slot to firmware/, writes only the app
# (0x10000; bootloader, partition table and the file system are kept),
# erases otadata so that slot boots (after an OTA the other one would) and
# resets. Nothing on the file system is touched before or by the write: a
# failed check or write leaves the board as it was. Then run ./deploy.sh: it
# removes the old /gps_logger.mpy (until then it shadows the frozen logger
# and runs as before) and any /gps_logger.py, clears the boot-fail counter
# and puts main.py and the page up. Right after esptool's reset the raw
# REPL may not come for a while: if deploy.sh cannot reach it, run it again.
# --migrate moves a board to the OTA partition table, once: it needs 16 MB
# of flash, backs up the first 4 MB, erases the whole flash (the file
# system too: copy /.env and /hist off first) and writes bootloader,
# partition table, otadata and the app. Then put the files back and run
# ./deploy.sh (see the README, "Firmware updates over WiFi").
# --publish writes firmware/ota/ (manifest.json, micropython.bin, the page)
# and copies it to OTA_DEST with scp. The manifest is signed with the key in
# OTA_KEY_FILE (default ~/.config/gps-logger/ota.key, the board's .env
# OTA_KEY); only a clean tree is published (BUILD is the commit time).
# firmware/gps_frozen.py records the SHA-256 of the frozen gps_logger.py
# (deploy.sh stops when it differs from the one in the repo), the commit,
# "-dirty" when the tree had uncommitted changes, and BUILD, the commit
# time the board's OTA compares.
set -euo pipefail
cd "$(dirname "$0")/.."

IDF_DIR=${IDF_DIR:-$HOME/esp/esp-idf}
MPY_DIR=${MPY_DIR:-$HOME/esp/micropython}
BOARD_DIR=${BOARD_DIR:-$PWD/boards/GPS_LOGGER_C6}
BOARD=$(basename "$BOARD_DIR")
CHIP=${CHIP:-esp32c6}
PORT=${PORT:-/dev/ttyUSB0}
APP_SIZE=0x270000                         # one OTA app slot
OUT=$PWD/firmware
MODE=${1:-}
case "$MODE" in
    ""|--flash|--migrate|--publish) ;;
    *) echo "usage: $0 [--flash|--migrate|--publish]" >&2; exit 2 ;;
esac

mkdir -p "$OUT"
# the frozen files: the logger and a marker deploy.sh looks for
cp gps_logger.py "$OUT/gps_logger.py"
COMMIT=$(git rev-parse --short HEAD 2>/dev/null || echo unknown)
BUILD=$(git log -1 --format=%ct 2>/dev/null || echo 0)
DIRTY=
if [ "$COMMIT" != unknown ] && [ -n "$(git status --porcelain --untracked-files=no 2>/dev/null)" ]; then
    COMMIT=$COMMIT-dirty
    DIRTY=1
fi
SHA=$(sha256sum "$OUT/gps_logger.py" | cut -d" " -f1)
printf 'COMMIT = "%s"\nSHA = "%s"\nBUILD = %s\n' "$COMMIT" "$SHA" "$BUILD" > "$OUT/gps_frozen.py"
cat > "$OUT/manifest.py" <<EOF
include("\$(PORT_DIR)/boards/manifest.py")
module("gps_logger.py", base_path="$OUT")
module("gps_frozen.py", base_path="$OUT")
EOF

# shellcheck disable=SC1091
source "$IDF_DIR/export.sh" >/dev/null
make -C "$MPY_DIR/mpy-cross" >/dev/null
make -C "$MPY_DIR/ports/esp32" BOARD="$BOARD" BOARD_DIR="$BOARD_DIR" \
    FROZEN_MANIFEST="$OUT/manifest.py" -j"$(nproc)" >/dev/null
BDIR=$MPY_DIR/ports/esp32/build-$BOARD
BIN=$BDIR/micropython.bin
cp "$BIN" "$OUT/micropython.bin"
SIZE=$(wc -c < "$BIN")
echo "Built $OUT/micropython.bin ($SIZE bytes, logger $COMMIT, sha256 $SHA)"
if [ "$SIZE" -gt $((APP_SIZE)) ]; then
    echo "The app ($SIZE bytes) does not fit an OTA slot ($((APP_SIZE)))" >&2
    exit 1
fi

case "$MODE" in
"")
    exit 0
    ;;
--flash)
    # the board's partition table must match the build's (only the app is written)
    esptool.py --chip "$CHIP" --port "$PORT" -b 460800 read_flash 0x8000 0xc00 "$OUT/pt_board.bin" >/dev/null
    if ! cmp -s "$OUT/pt_board.bin" "$BDIR/partition_table/partition-table.bin"; then
        echo "The board's partition table differs from the build's: not flashing" >&2
        echo "(a board without the OTA slots: --migrate, once)" >&2
        exit 1
    fi
    BACKUP=$OUT/app-backup-$(date +%Y%m%d-%H%M%S).bin
    esptool.py --chip "$CHIP" --port "$PORT" -b 460800 read_flash 0x10000 $APP_SIZE "$BACKUP" >/dev/null
    echo "App slot backed up to $BACKUP"
    esptool.py --chip "$CHIP" --port "$PORT" -b 460800 --after no_reset write_flash \
        --flash_mode dio --flash_freq 80m 0x10000 "$OUT/micropython.bin"
    # otadata gone: the bootloader boots the first slot, the one just written
    esptool.py --chip "$CHIP" --port "$PORT" -b 460800 --after hard_reset erase_region 0xd000 0x2000 >/dev/null
    echo "Flashed $PORT. Now run ./deploy.sh (again if the board's REPL does not answer yet):"
    echo "it removes the old gps_logger.mpy that still shadows the frozen logger."
    echo "Roll back with: esptool.py --chip $CHIP --port $PORT write_flash 0x10000 $BACKUP, then ./deploy.sh"
    ;;
--migrate)
    if ! esptool.py --chip "$CHIP" --port "$PORT" flash_id 2>/dev/null | grep -q "flash size: 16MB"; then
        echo "--migrate needs a board with 16 MB of flash: not touching it" >&2
        exit 1
    fi
    BACKUP=$OUT/full-backup-$(date +%Y%m%d-%H%M%S).bin
    esptool.py --chip "$CHIP" --port "$PORT" -b 460800 read_flash 0 0x400000 "$BACKUP" >/dev/null
    echo "First 4 MB (old bootloader, table, app, start of the file system) backed up to $BACKUP"
    esptool.py --chip "$CHIP" --port "$PORT" erase_flash >/dev/null
    (cd "$BDIR" && esptool.py --chip "$CHIP" --port "$PORT" -b 460800 --after hard_reset write_flash @flash_args)
    echo "Migrated $PORT to the OTA table. The file system is new and empty: put /.env"
    echo "(with OTA_URL and OTA_KEY) and /hist back, then run ./deploy.sh."
    echo "Back to the old layout: esptool.py --chip $CHIP --port $PORT write_flash 0 $BACKUP"
    ;;
--publish)
    if [ -n "$DIRTY" ] || [ "$COMMIT" = unknown ]; then
        echo "Only a clean, committed tree is published ($COMMIT)" >&2
        exit 1
    fi
    KEY_FILE=${OTA_KEY_FILE:-$HOME/.config/gps-logger/ota.key}
    if [ ! -r "$KEY_FILE" ]; then
        echo "No OTA key in $KEY_FILE (the board's .env OTA_KEY)" >&2
        exit 1
    fi
    : "${OTA_DEST:?OTA_DEST: where to copy firmware/ota (an scp target)}"
    rm -rf "$OUT/ota"
    mkdir -p "$OUT/ota"
    cp "$OUT/micropython.bin" web.html "$OUT/ota/"
    gzip -9 -n -c web.html > "$OUT/ota/web.html.gz"
    python3 - "$OUT/ota" "$KEY_FILE" "$BUILD" "$COMMIT" <<'EOF'
import hashlib, hmac, json, os, sys
d, key_file, build, commit = sys.argv[1], sys.argv[2], int(sys.argv[3]), sys.argv[4]
key = open(key_file, "rb").read().strip()
def meta(name):
    b = open(os.path.join(d, name), "rb").read()
    return len(b), hashlib.sha256(b).hexdigest()
size, sha = meta("micropython.bin")
m = {"build": build, "commit": commit, "size": size, "sha256": sha,
     "url": "micropython.bin", "files": []}
for name in ("web.html.gz", "web.html"):
    n, h = meta(name)
    m["files"].append({"path": "/" + name, "size": n, "sha256": h, "url": name})
# the bytes gps_logger._ota_signed covers
t = "%d|%s|%d|%s|%s" % (m["build"], m["commit"], m["size"], m["sha256"], m["url"])
for f in m["files"]:
    t += "|%s|%d|%s|%s" % (f["path"], f["size"], f["sha256"], f["url"])
m["hmac"] = hmac.new(key, t.encode(), hashlib.sha256).hexdigest()
with open(os.path.join(d, "manifest.json"), "w") as f:
    json.dump(m, f)
print("manifest: build", build, "commit", commit, "size", size)
EOF
    scp -q "$OUT/ota/micropython.bin" "$OUT/ota/web.html" "$OUT/ota/web.html.gz" "$OUT/ota/manifest.json" "$OTA_DEST/"
    echo "Published $COMMIT (build $BUILD) to $OTA_DEST"
    ;;
esac
