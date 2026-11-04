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
# (partitions-ota.csv, 16 MB flash). BOARD_DIR picks another, its chip and
# flash size read from it: boards/GPS_LOGGER_S3 is the Heltec Wireless Tracker
# (ESP32-S3, 8 MB).
# Usage:
#   tools/build_firmware.sh                    build only
#   PORT=/dev/ttyACM0 tools/build_firmware.sh --flash
#   PORT=/dev/ttyACM0 tools/build_firmware.sh --migrate
#   OTA_DEST=host:/srv/ota/gps tools/build_firmware.sh --publish
# --flash checks that the board's partition table matches the build's,
# backs up both app slots to firmware/ (after an OTA the second one runs),
# writes only the app
# (0x10000; bootloader, partition table and the file system are kept),
# erases otadata so that slot boots (after an OTA the other one would) and
# resets. Nothing on the file system is touched before or by the write: a
# failed check leaves the board as it was. A failed write does not: it
# always goes to the first slot (ota_0), the running one unless an OTA
# switched to ota_1, so the board may not boot an app until --flash runs
# again or the backup is written back (the ROM download mode over USB is
# never touched; the commands are printed). With ota_1 running and otadata
# not yet erased, the board still boots ota_1. Then run ./deploy.sh: it
# removes the old /gps_logger.mpy (until then it shadows the frozen logger
# and runs as before) and any /gps_logger.py, clears the boot-fail counter
# and puts main.py and the page up. Right after esptool's reset the raw
# REPL may not come for a while: if deploy.sh cannot reach it, run it again.
# --migrate moves a board to the OTA partition table, once: it needs the
# flash size of the board's sdkconfig.board, backs up the first 4 MB (mode 600: it holds the old file
# system's start, /.env with its keys too), erases the whole flash (the file
# system too: copy /.env and /hist off first) and writes bootloader,
# partition table, otadata and the app. Then put the files back and run
# ./deploy.sh (see the README, "Firmware updates over WiFi").
# --publish writes firmware/ota/ (micropython-<commit>.bin, the page as
# web-<commit>.html.gz, manifest.json naming them: the board keeps the page
# only gzipped, made with its WEB_GZ_WBITS window, and inflates it for a
# client without gzip; an older firmware takes such a manifest too, and the
# new one removes the old plain /web.html once valid) and copies it to
# OTA_DEST (host:/path) with scp, each file as <name>.tmp moved over the
# live one with ssh (a board reading a half-copied file got a mix), the
# manifest last: a board reading the old manifest meanwhile still finds
# the files it names. Each board has its own OTA_DEST and .env OTA_URL: a
# manifest names its board and a board refuses another's. A build not newer than the published manifest's is
# refused (FORCE=1 publishes it anyway). The manifest is signed with the
# key in OTA_KEY_FILE (default ~/.config/gps-logger/ota.key, the board's
# .env OTA_KEY: printable ASCII, e.g. openssl rand -hex 32); only a clean
# tree is published (BUILD is the commit time).
# firmware/gps_frozen.py records the SHA-256 of the frozen gps_logger.py
# (deploy.sh stops when it differs from the one in the repo), the commit,
# "-dirty" when the tree had uncommitted changes, and BUILD, which the
# board's OTA compares: the commit time; for --flash and a dirty tree the
# build time, so an OTA does not replace what was just flashed by hand
# with an older published build.
set -euo pipefail
cd "$(dirname "$0")/.."

IDF_DIR=${IDF_DIR:-$HOME/esp/esp-idf}
MPY_DIR=${MPY_DIR:-$HOME/esp/micropython}
BOARD_DIR=${BOARD_DIR:-$PWD/boards/GPS_LOGGER_C6}
BOARD=$(basename "$BOARD_DIR")
CHIP=${CHIP:-$(sed -n 's/^set(IDF_TARGET \(.*\))$/\1/p' "$BOARD_DIR/mpconfigboard.cmake" 2>/dev/null || true)}
CHIP=${CHIP:-esp32c6}
FLASH_MB=$(sed -n 's/^CONFIG_ESPTOOLPY_FLASHSIZE="\([0-9]*\)MB"$/\1/p' "$BOARD_DIR/sdkconfig.board" 2>/dev/null || true)
FLASH_MB=${FLASH_MB:-16}
PORT=${PORT:-/dev/ttyUSB0}
APP_SIZE=0x1c0000                         # one OTA app slot (partitions-ota.csv)
APP1=$(printf '0x%x' $((0x10000 + APP_SIZE)))  # the second slot's offset
OUT=$PWD/firmware
MODE=${1:-}
case "$MODE" in
    ""|--flash|--migrate|--publish) ;;
    *) echo "usage: $0 [--flash|--migrate|--publish]" >&2; exit 2 ;;
esac

mkdir -p "$OUT"
# the frozen files: the logger, main.py's boot helpers (gps_boot.py: OTA
# and the boot-fail retry, out of the heap) and a marker deploy.sh looks for
cp gps_logger.py "$OUT/gps_logger.py"
cp gps_boot.py "$OUT/gps_boot.py"
COMMIT=$(git rev-parse --short HEAD 2>/dev/null || echo unknown)
BUILD=$(git log -1 --format=%ct 2>/dev/null || echo 0)
DIRTY=
if [ "$COMMIT" != unknown ] && [ -n "$(git status --porcelain --untracked-files=no 2>/dev/null)" ]; then
    COMMIT=$COMMIT-dirty
    DIRTY=1
fi
if [ -n "$DIRTY" ] || [ "$MODE" = --flash ]; then
    BUILD=$(date +%s)
fi
SHA=$(sha256sum "$OUT/gps_logger.py" | cut -d" " -f1)
printf 'COMMIT = "%s"\nSHA = "%s"\nBUILD = %s\n' "$COMMIT" "$SHA" "$BUILD" > "$OUT/gps_frozen.py"
# only the port's own modules (_boot.py mounts the file system) with the
# logger's: not boards/manifest.py's asyncio, umqtt, webrepl, mip,
# requests... (~25 KB the logger never imports)
cat > "$OUT/manifest.py" <<EOF
freeze("\$(PORT_DIR)/modules")
module("gps_logger.py", base_path="$OUT")
module("gps_frozen.py", base_path="$OUT")
module("gps_boot.py", base_path="$OUT")
EOF

# shellcheck disable=SC1091
source "$IDF_DIR/export.sh" >/dev/null
make -C "$MPY_DIR/mpy-cross" >/dev/null
BDIR=$MPY_DIR/ports/esp32/build-$BOARD
# a changed board (sdkconfig, partition table, macros): a clean build, as
# ESP-IDF keeps an existing build's sdkconfig over new defaults (BLE stayed
# on after it was turned off)
BSTAMP=$( (cat "$BOARD_DIR"/* 2>/dev/null || true) | sha256sum | cut -c1-16)
if [ -d "$BDIR" ] && [ "$(cat "$BDIR/.gps_board" 2>/dev/null)" != "$BSTAMP" ]; then
    rm -rf "$BDIR"
fi
make -C "$MPY_DIR/ports/esp32" BOARD="$BOARD" BOARD_DIR="$BOARD_DIR" \
    FROZEN_MANIFEST="$OUT/manifest.py" -j"$(nproc)" >/dev/null
mkdir -p "$BDIR"
echo "$BSTAMP" > "$BDIR/.gps_board"
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
    STAMP=$(date +%Y%m%d-%H%M%S)
    BACKUP=$OUT/app-backup-$STAMP.bin
    BACKUP1=$OUT/app1-backup-$STAMP.bin
    esptool.py --chip "$CHIP" --port "$PORT" -b 460800 read_flash 0x10000 $APP_SIZE "$BACKUP" >/dev/null
    esptool.py --chip "$CHIP" --port "$PORT" -b 460800 read_flash "$APP1" $APP_SIZE "$BACKUP1" >/dev/null
    echo "App slots backed up to $BACKUP and $BACKUP1 (the second one runs after an OTA)"
    # always the first slot: when it is the running one, a failed write
    # leaves no bootable app there (see the header)
    if ! esptool.py --chip "$CHIP" --port "$PORT" -b 460800 --after no_reset write_flash \
            --flash_mode dio --flash_freq 80m 0x10000 "$OUT/micropython.bin"; then
        echo "The write failed: the first slot (0x10000) may not boot now. The ROM" >&2
        echo "download mode is untouched: run --flash again, or write the backup:" >&2
        echo "  esptool.py --chip $CHIP --port $PORT write_flash 0x10000 $BACKUP" >&2
        echo "(if the board ran the second slot it still boots it until otadata is erased)" >&2
        exit 1
    fi
    # otadata gone: the bootloader boots the first slot, the one just written
    esptool.py --chip "$CHIP" --port "$PORT" -b 460800 --after hard_reset erase_region 0xd000 0x2000 >/dev/null
    echo "Flashed $PORT. Now run ./deploy.sh (again if the board's REPL does not answer yet):"
    echo "it removes the old gps_logger.mpy that still shadows the frozen logger."
    echo "Roll back with: esptool.py --chip $CHIP --port $PORT write_flash 0x10000 $BACKUP, then ./deploy.sh"
    echo "(if the board ran the second slot: write_flash 0x10000 $BACKUP1)"
    ;;
--migrate)
    # (not "| grep -q" under pipefail: grep stops at the match, esptool's
    # next write gets EPIPE and the pipe fails: a 16 MB board was refused)
    FID=$(esptool.py --chip "$CHIP" --port "$PORT" flash_id 2>/dev/null || true)
    if ! grep -q "flash size: ${FLASH_MB}MB" <<< "$FID"; then
        echo "--migrate needs a board with $FLASH_MB MB of flash: not touching it" >&2
        exit 1
    fi
    BACKUP=$OUT/full-backup-$(date +%Y%m%d-%H%M%S).bin
    # the old file system's start, /.env (WiFi, API, OTA keys) too: 600
    (umask 077 && esptool.py --chip "$CHIP" --port "$PORT" -b 460800 read_flash 0 0x400000 "$BACKUP" >/dev/null)
    chmod 600 "$BACKUP"
    echo "First 4 MB (old bootloader, table, app, start of the file system) backed up to $BACKUP"
    echo "It holds the old /.env and its secrets (mode 600): keep it off shared places, delete it when done"
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
    # the board reads it from .env: printable ASCII, no quotes or spaces
    if ! tr -d '\n' < "$KEY_FILE" | LC_ALL=C grep -qE "^[!#-&(-~]{16,}$"; then
        echo "The OTA key in $KEY_FILE must be 16+ printable ASCII characters" >&2
        echo "without quotes or spaces (e.g. openssl rand -hex 32)" >&2
        exit 1
    fi
    : "${OTA_DEST:?OTA_DEST: where to copy firmware/ota (host:/path)}"
    case "$OTA_DEST" in
        *:*) DHOST=${OTA_DEST%%:*}; DDIR=${OTA_DEST#*:}; DDIR=${DDIR:-.} ;;
        *) echo "OTA_DEST must be host:/path (scp and ssh)" >&2; exit 1 ;;
    esac
    QDIR=$(printf %q "$DDIR")
    # the published build: only a newer one goes up (a board takes a
    # manifest's build when it is newer than its own, so an older or equal
    # one would not reach boards already updated, and republishing a commit
    # rewrote the .bin boards may be downloading)
    # (expanded here on purpose: QDIR is quoted for the remote shell)
    # shellcheck disable=SC2029
    if ! LIVE=$(ssh "$DHOST" "if [ -e $QDIR/manifest.json ]; then cat $QDIR/manifest.json; fi"); then
        echo "Could not read the published manifest in $OTA_DEST" >&2
        exit 1
    fi
    LIVE_BUILD=0
    # the manifest's "board": the C6's has none (boards in the field read it
    # as ever), any other board's is signed and refused by a different board
    MBOARD=$BOARD
    [ "$BOARD" = GPS_LOGGER_C6 ] && MBOARD=
    if [ -n "$LIVE" ]; then
        LIVE_BOARD=$(python3 -c 'import json, sys; print(json.loads(sys.stdin.read()).get("board") or "")' <<< "$LIVE" 2>/dev/null || true)
        if [ "$LIVE_BOARD" != "$MBOARD" ]; then
            echo "$OTA_DEST holds the firmware of another board (${LIVE_BOARD:-GPS_LOGGER_C6}): each board has its own folder" >&2
            exit 1
        fi
        LIVE_BUILD=$(python3 -c 'import json, sys; print(int(json.loads(sys.stdin.read())["build"]))' <<< "$LIVE" 2>/dev/null || echo x)
    fi
    if [ "$LIVE_BUILD" = x ]; then
        echo "The published manifest in $OTA_DEST has no readable build" >&2
        [ "${FORCE:-0}" = 1 ] || exit 1
    elif [ "$BUILD" -le "$LIVE_BUILD" ]; then
        echo "Build $BUILD ($COMMIT) is not newer than the published build $LIVE_BUILD" >&2
        [ "${FORCE:-0}" = 1 ] || { echo "(FORCE=1 publishes it anyway)" >&2; exit 1; }
        echo "FORCE=1: publishing it anyway" >&2
    fi
    rm -rf "$OUT/ota"
    mkdir -p "$OUT/ota"
    BINF=micropython-$COMMIT.bin
    cp "$OUT/micropython.bin" "$OUT/ota/$BINF"
    # not gzip -9: its 32 KB window is what inflating it takes on the board,
    # more than the heap has free; the firmware's own WEB_GZ_WBITS window
    WBITS=$(sed -n 's/^WEB_GZ_WBITS = \([0-9][0-9]*\).*/\1/p' gps_logger.py)
    case "$WBITS" in
        9|1[0-5]) ;;
        *) echo "No WEB_GZ_WBITS in gps_logger.py: '$WBITS'" >&2; exit 1 ;;
    esac
    python3 - "$WBITS" web.html "$OUT/ota/web-$COMMIT.html.gz" <<'EOF'
import sys, zlib
w, src, dst = int(sys.argv[1]), sys.argv[2], sys.argv[3]
z = zlib.compressobj(9, zlib.DEFLATED, 16 + w)
with open(src, "rb") as f:
    gz = z.compress(f.read()) + z.flush()
with open(dst, "wb") as f:
    f.write(gz)
EOF
    python3 - "$OUT/ota" "$KEY_FILE" "$BUILD" "$COMMIT" "$MBOARD" <<'EOF'
import hashlib, hmac, json, os, sys
d, key_file, build, commit, board = sys.argv[1], sys.argv[2], int(sys.argv[3]), sys.argv[4], sys.argv[5]
key = open(key_file, "rb").read().strip()
def meta(name):
    b = open(os.path.join(d, name), "rb").read()
    return len(b), hashlib.sha256(b).hexdigest()
binf = "micropython-%s.bin" % commit
size, sha = meta(binf)
m = {"build": build, "commit": commit, "size": size, "sha256": sha,
     "url": binf, "files": []}
# the page only gzipped (an older firmware after a rollback takes this too)
for path, name in (("/web.html.gz", "web-%s.html.gz" % commit),):
    n, h = meta(name)
    m["files"].append({"path": path, "size": n, "sha256": h, "url": name})
# the bytes gps_logger._ota_signed covers
t = "%d|%s|%d|%s|%s" % (m["build"], m["commit"], m["size"], m["sha256"], m["url"])
for f in m["files"]:
    t += "|%s|%d|%s|%s" % (f["path"], f["size"], f["sha256"], f["url"])
if board:
    m["board"] = board
    t += "|board=" + board
m["hmac"] = hmac.new(key, t.encode(), hashlib.sha256).hexdigest()
with open(os.path.join(d, "manifest.json"), "w") as f:
    json.dump(m, f)
print("manifest: build", build, "commit", commit, "size", size)
EOF
    # each as <name>.tmp, then moved over the live one (one rename: a board
    # downloading it keeps reading the old file), the named files first,
    # the manifest naming them last
    MV=""
    for f in "$BINF" "web-$COMMIT.html.gz" manifest.json; do
        scp -q "$OUT/ota/$f" "$DHOST:$DDIR/$f.tmp"
        MV+="mv -f $QDIR/$f.tmp $QDIR/$f && "
    done
    # (MV built here, its paths %q-quoted)
    # shellcheck disable=SC2029
    ssh "$DHOST" "${MV}true"
    echo "Published $COMMIT (build $BUILD) to $OTA_DEST"
    ;;
esac
