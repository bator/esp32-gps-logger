#!/bin/bash
# Build MicroPython with gps_logger frozen into the firmware, and optionally
# flash or publish it. Frozen, the logger's bytecode runs from flash instead
# of taking ~100 KB of the Python heap: the heap starts at 96 KB, so far fewer
# allocations fail to fit and grow it into the ESP-IDF heap that TLS uploads need
# (it still can: MicroPython's split heap stays on).
#
# Needs ESP-IDF (v5.5.2 for MicroPython 1.29) and a MicroPython v1.29.0 tree:
#   IDF_DIR=~/esp/esp-idf  MPY_DIR=~/esp/micropython  (defaults)
# The board: boards/GPS_LOGGER_S3, the Heltec Wireless Tracker (ESP32-S3, 8 MB,
# two OTA app slots: partitions-ota.csv); BOARD_DIR names another, its chip and
# flash size read from it.
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
# ./deploy.sh (see docs/setup.md, "Firmware updates over WiFi").
# --publish writes firmware/ota/ (micropython-<commit>.bin, the page as
# web-<commit>.html.gz, manifest.json naming them: the board keeps the page
# only gzipped, made with its WEB_GZ_WBITS window, and inflates it for a
# client without gzip; an older firmware takes such a manifest too, and the
# new one removes the old plain /web.html once valid) and copies it to
# OTA_DEST (host:/path) with scp, each file as <name>.tmp moved over the
# live one with ssh (a board reading a half-copied file got a mix), the
# manifest last: a board reading the old manifest meanwhile still finds
# the files it names; then the files only the replaced manifest named are
# removed (an older signed build could be served again: the HMAC binds no
# freshness). Each board has its own OTA_DEST and .env OTA_URL: a
# manifest names its board and a board refuses another's. A build not newer than the published manifest's is
# refused (FORCE=1 publishes it anyway). The manifest is signed with the
# key in OTA_KEY_FILE (default ~/.config/gps-logger/ota.key, the board's
# .env OTA_KEY: printable ASCII, e.g. openssl rand -hex 32); only a clean
# tree is published (BUILD is the commit time); a key file readable by
# everyone is refused, one readable by its group warned about (chmod 600).
# The OTA_DEST path goes to scp as it is and to ssh shell-quoted: only
# letters, digits and _ . / - in it (no ~, spaces or shell specials), checked
# before the build.
# firmware/gps_frozen.py records the SHA-256 of the frozen gps_logger.py
# (SHA), of gps_tft.py + gps_tft_font.py (TFT), of gps_boot.py (BOOT) and
# of gps_clock.py (CLOCK) and of gps_receiver.py (RX; deploy.sh stops when
# any differs from the repo's), the commit, "-dirty"
# when the tree had uncommitted changes, and BUILD, which the board's OTA
# compares: the commit time; for --flash and a dirty tree the build time,
# so an OTA does not replace what was just flashed by hand with an older
# published build.
set -euo pipefail
cd "$(dirname "$0")/.."

IDF_DIR=${IDF_DIR:-$HOME/esp/esp-idf}
MPY_DIR=${MPY_DIR:-$HOME/esp/micropython}
BOARD_DIR=${BOARD_DIR:-$PWD/boards/GPS_LOGGER_S3}
BOARD=$(basename "$BOARD_DIR")
CHIP=$(sed -n 's/^set(IDF_TARGET \(.*\))$/\1/p' "$BOARD_DIR/mpconfigboard.cmake")
FLASH_MB=$(sed -n 's/^CONFIG_ESPTOOLPY_FLASHSIZE="\([0-9]*\)MB"$/\1/p' "$BOARD_DIR/sdkconfig.board")
PORT=${PORT:-/dev/ttyACM0}               # the Heltec's USB Serial/JTAG (as deploy.sh)
APP_SIZE=0x1c0000                         # one OTA app slot (partitions-ota.csv)
APP1=$(printf '0x%x' $((0x10000 + APP_SIZE)))  # the second slot's offset
OUT=$PWD/firmware
MODE=${1:-}
case "$MODE" in
    ""|--flash|--migrate|--publish) ;;
    *) echo "usage: $0 [--flash|--migrate|--publish]" >&2; exit 2 ;;
esac

# A published destination is checked before the build (and long before any
# upload): the path goes to scp as it is and to ssh shell-quoted, which agree
# only on a plain one ("~/x" would be a directory called "~" for the ssh half)
# The key file too: a missing OTA_DEST or key failed only after the
# ESP-IDF build
if [ "$MODE" = --publish ]; then
    : "${OTA_DEST:?OTA_DEST: where to copy firmware/ota (host:/path)}"
    case "$OTA_DEST" in
        *:*) _dhost=${OTA_DEST%%:*}; _ddir=${OTA_DEST#*:} ;;
        *) echo "OTA_DEST must be host:/path (scp and ssh)" >&2; exit 1 ;;
    esac
    case "$_dhost" in
        -*|"") echo "OTA_DEST: the host part must not be empty or start with a dash: '$OTA_DEST'" >&2; exit 1 ;;
    esac
    case "$_ddir" in
        "~"*|-*|*[!A-Za-z0-9_./-]*)
            echo "OTA_DEST: the directory '$_ddir' must be a plain path (letters, digits, _ . / -; no ~," >&2
            echo "no leading dash: mv and rm on the server would take it for an option," >&2
            echo "spaces or shell specials: scp and the ssh rename would read it differently)" >&2
            exit 1 ;;
    esac
    # the manifest's board is the board directory's name; the firmware takes
    # only its own OTA_BOARD: from another directory every manifest was
    # "bad manifest" for good, the board never updated over the air
    _ota_board=$(sed -n 's/^OTA_BOARD = "\([^"]*\)".*/\1/p' gps_logger.py)
    if [ "$BOARD" != "$_ota_board" ]; then
        echo "BOARD_DIR is $BOARD, the firmware's OTA_BOARD is '$_ota_board': its boards would refuse the manifest" >&2
        exit 1
    fi
    KEY_FILE=${OTA_KEY_FILE:-$HOME/.config/gps-logger/ota.key}
    if [ ! -r "$KEY_FILE" ]; then
        echo "No OTA key in $KEY_FILE (the board's .env OTA_KEY)" >&2
        exit 1
    fi
    # the key is a secret: readable by everyone is refused, by the group warned
    # (its content is never printed here); -L: the file a symlink points to
    # (the link's own mode is 777: a 600 key behind one was refused)
    KMODE=$(stat -L -c %a "$KEY_FILE" 2>/dev/null || stat -L -f %Lp "$KEY_FILE" 2>/dev/null || true)
    case "$KMODE" in
        ""|*[!0-7]*) echo "Could not read the mode of $KEY_FILE (chmod 600 it)" >&2 ;;
        *)
            if [ $(( 8#$KMODE & 4 )) -ne 0 ]; then
                echo "The OTA key file $KEY_FILE is readable by everyone (mode $KMODE): chmod 600 it" >&2
                exit 1
            elif [ $(( 8#$KMODE & 32 )) -ne 0 ]; then
                echo "WARNING: the OTA key file $KEY_FILE is readable by its group (mode $KMODE): chmod 600 it" >&2
            fi ;;
    esac
    # the board reads it from .env: one line of printable ASCII, no quotes
    # or spaces; checked as firmware.py signs with it (the file stripped: a
    # check that dropped every newline passed a two-line key)
    if ! python3 tools/firmware.py key "$KEY_FILE"; then
        exit 1
    fi
fi

mkdir -p "$OUT"
# the frozen files: the logger, its Clock (gps_clock.py) and Receiver
# (gps_receiver.py), main.py's boot helpers (gps_boot.py: OTA and the boot-fail
# retry, out of the heap) and a marker deploy.sh looks for
# what goes into the image (and the page): a change or an untracked one of
# these is "-dirty" (--untracked-files=no let an untracked gps_*.py through
# under a clean commit's name)
SRC_FILES="gps_logger.py gps_clock.py gps_receiver.py gps_boot.py gps_tft.py gps_tft_font.py web.html"
# shellcheck disable=SC2086
src_sum() { cat $SRC_FILES | sha256sum | cut -d" " -f1; }
SRC_SUM=$(src_sum)                        # (before the copies below)
cp gps_logger.py "$OUT/gps_logger.py"
cp gps_clock.py "$OUT/gps_clock.py"
cp gps_receiver.py "$OUT/gps_receiver.py"
cp gps_boot.py "$OUT/gps_boot.py"
COMMIT=$(git rev-parse --short HEAD 2>/dev/null || echo unknown)
BUILD=$(git log -1 --format=%ct 2>/dev/null || echo 0)
DIRTY=
# shellcheck disable=SC2086
if [ "$COMMIT" != unknown ] && { [ -n "$(git status --porcelain --untracked-files=no 2>/dev/null)" ] \
        || [ -n "$(git status --porcelain -- $SRC_FILES 2>/dev/null)" ]; }; then
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
# requests... (~25 KB the logger never imports). opt=3 leaves the line numbers
# out of the bytecode (~10 KB of the logger's): a traceback on the USB console
# says line 1 (README); asserts and __debug__ would go too: there are none
cat > "$OUT/manifest.py" <<EOF
freeze("\$(PORT_DIR)/modules")
module("gps_logger.py", base_path="$OUT", opt=3)
module("gps_clock.py", base_path="$OUT", opt=3)
module("gps_receiver.py", base_path="$OUT", opt=3)
module("gps_frozen.py", base_path="$OUT")
module("gps_boot.py", base_path="$OUT", opt=3)
module("gps_tft.py", base_path="$OUT", opt=3)
module("gps_tft_font.py", base_path="$OUT", opt=3)
EOF
# the display's code, gps_boot.py, gps_clock.py and gps_receiver.py, in the
# marker too (tools/firmware.py: what each field hashes; deploy.sh compares
# them with it; an older marker without TFT, BOOT, CLOCK or RX still passes)
python3 tools/firmware.py marker >> "$OUT/gps_frozen.py"
cp gps_tft.py gps_tft_font.py "$OUT/"

# the toolchain the .mpy ABI and deploy.sh's mpy-cross pin are for: another
# checkout made a signed image whose frozen code no deploy matched
# (FORCE_TOOLS=1 builds anyway; a tree that is no git checkout is not checked)
# (a function: a path with a space split into dir and tag, the check
# skipped without a word)
tool_tag() {
    local d=$1 want=$2 got
    [ -e "$d/.git" ] || return 0
    if ! got=$(git -C "$d" describe --tags --exact-match 2>&1); then
        got="no release tag ($got)"
    fi
    if [ "$got" != "$want" ]; then
        echo "$d is at $got, not $want" >&2
        [ "${FORCE_TOOLS:-0}" = 1 ] || { echo "(FORCE_TOOLS=1 builds anyway)" >&2; exit 1; }
    fi
}
tool_tag "$MPY_DIR" v1.29.0
tool_tag "$IDF_DIR" v5.5.2

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
    if ! esptool.py --chip "$CHIP" --port "$PORT" -b 460800 --after hard_reset erase_region 0xd000 0x2000 >/dev/null; then
        echo "The app is written but otadata (0xd000) is not erased: a board that ran the" >&2
        echo "second slot still boots it. Run:" >&2
        echo "  esptool.py --chip $CHIP --port $PORT erase_region 0xd000 0x2000" >&2
        exit 1
    fi
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
    : "${OTA_DEST:?OTA_DEST: where to copy firmware/ota (host:/path)}"
    # (the path: no spaces or shell specials; scp takes it as it is, the
    # ssh mv below %q-quoted, and the two agree only on a plain path)
    case "$OTA_DEST" in
        *:*) DHOST=${OTA_DEST%%:*}; DDIR=${OTA_DEST#*:}; DDIR=${DDIR:-.} ;;
        *) echo "OTA_DEST must be host:/path (scp and ssh)" >&2; exit 1 ;;
    esac
    QDIR=$(printf %q "$DDIR")
    # no password prompt, no stdin, no 2-minute hang on a dead host
    SSHO=(-o BatchMode=yes -o ConnectTimeout=10 -o ServerAliveInterval=10
          -o ServerAliveCountMax=3)      # (and no hang on a link that dies)
    # the published build: only a newer one goes up (a board takes a
    # manifest's build when it is newer than its own, so an older or equal
    # one would not reach boards already updated, and republishing a commit
    # rewrote the .bin boards may be downloading)
    # (expanded here on purpose: QDIR is quoted for the remote shell)
    # shellcheck disable=SC2029
    if ! LIVE=$(ssh "${SSHO[@]}" -n "$DHOST" "if [ -d $QDIR ]; then echo dir; fi; if [ -e $QDIR/manifest.json ]; then cat $QDIR/manifest.json; fi"); then
        echo "Could not read the published manifest in $OTA_DEST" >&2
        exit 1
    fi
    # (a folder that is not there: a typo in OTA_DEST, not a first publish
    # whose checks below would all pass)
    if [ "${LIVE%%$'\n'*}" != dir ]; then
        echo "$OTA_DEST is no directory on $DHOST (mkdir it for a first publish)" >&2
        exit 1
    fi
    LIVE=${LIVE#dir}
    LIVE=${LIVE#$'\n'}
    if [ -z "$LIVE" ]; then
        echo "No manifest in $OTA_DEST yet: a first publish there"
    fi
    LIVE_BUILD=0
    LIVE_COMMIT=
    # the manifest's "board" is signed, and a board refuses another's
    if [ -n "$LIVE" ]; then
        # ("?": unreadable, not an empty name, an older board's manifest:
        # LIVE_BUILD below says so)
        LIVE_BOARD=$(python3 -c 'import json, sys; print(json.loads(sys.stdin.read()).get("board") or "")' <<< "$LIVE" 2>/dev/null || echo "?")
        if [ "$LIVE_BOARD" != "?" ] && [ "$LIVE_BOARD" != "$BOARD" ]; then
            echo "$OTA_DEST holds the firmware of another board (${LIVE_BOARD:-no name: the older board}): each board has its own folder" >&2
            exit 1
        fi
        LIVE_BUILD=$(python3 -c 'import json, sys; print(int(json.loads(sys.stdin.read())["build"]))' <<< "$LIVE" 2>/dev/null || echo x)
        LIVE_COMMIT=$(python3 -c 'import json, sys; print(json.loads(sys.stdin.read()).get("commit") or "")' <<< "$LIVE" 2>/dev/null || true)
    fi
    RESUME=
    BINF=micropython-$COMMIT.bin
    # (expanded here on purpose: QDIR is quoted for the remote shell)
    # shellcheck disable=SC2029
    if ! LS=$(ssh "${SSHO[@]}" -n "$DHOST" "ls -1A $QDIR"); then
        echo "WARNING: could not list $OTA_DEST: leftovers are not removed" >&2
        LS=
        LS_OK=
    else
        LS_OK=1
    fi
    # the live manifest is this very build (same build and commit): only a
    # cleanup when its files are there and it does not end within a week;
    # else it is published again (the same names, each moved in)
    SAME=
    if [ "$LIVE_BUILD" != x ] && [ "$BUILD" = "$LIVE_BUILD" ] && [ "$COMMIT" = "$LIVE_COMMIT" ]; then
        SAME=1
        LIVE_LEFT=$(python3 -c 'import json, sys, time; m = json.loads(sys.stdin.read()); print(int(m.get("expires", 0)) + 1577836800 - int(time.time()))' <<< "$LIVE" 2>/dev/null || echo 0)
        WHY=
        if [ -z "$LS_OK" ]; then
            WHY="its folder could not be listed"
        elif ! grep -qx -- "$BINF" <<< "$LS" || ! grep -qx -- "web-$COMMIT.html.gz" <<< "$LS"; then
            WHY="a file it names is missing"
        elif [ "$LIVE_LEFT" -lt $((7 * 86400)) ]; then
            WHY="it ends in under a week (expires)"
        else
            # the files' sums, the manifest's: a republish of this commit
            # (FORCE=1: the same names, a rebuild not byte for byte the same)
            # whose ssh broke between the moves left a new image under the
            # old manifest; a cleanup alone kept every board failing its sum
            WANT=$(python3 -c 'import json, re, sys
m = json.loads(sys.stdin.read())
for x in [m] + list(m.get("files", [])):
    assert re.match(r"^[0-9a-f]{64}$", x["sha256"]) and re.match(r"^[A-Za-z0-9._-]+$", x["url"])
    print(x["sha256"] + "  " + x["url"])' <<< "$LIVE" 2>/dev/null | sort || true)
            HAVE=
            if [ -n "$WANT" ]; then
                NAMES=""
                while read -r _ f; do NAMES+=" $QDIR/$(printf %q "$f")"; done <<< "$WANT"
                # (expanded here on purpose: the paths are %q-quoted)
                # shellcheck disable=SC2029
                HAVE=$(ssh "${SSHO[@]}" -n "$DHOST" "sha256sum$NAMES" 2>/dev/null \
                       | sed -E 's#  .*/#  #' | sort || true)
            fi
            if [ -z "$WANT" ]; then
                WHY="its manifest names no sums"
            elif [ -z "$HAVE" ]; then
                WHY="its files' sums could not be read (sha256sum on $DHOST?)"
            elif [ "$HAVE" != "$WANT" ]; then
                WHY="its files are not the ones its manifest signs"
            fi
        fi
    fi
    if [ "$LIVE_BUILD" = x ]; then
        echo "The published manifest in $OTA_DEST has no readable build" >&2
        [ "${FORCE:-0}" = 1 ] || exit 1
    elif [ -n "$SAME" ] && [ "${FORCE:-0}" != 1 ] && [ -z "$WHY" ]; then
        # this very build is the live one (a publish whose ssh broke after
        # the renames, or whose cleanup failed): only the cleanup below
        echo "Build $BUILD ($COMMIT) is already published: removing what is left over only (FORCE=1 publishes it again)"
        RESUME=1
    elif [ -n "$SAME" ]; then
        echo "Build $BUILD ($COMMIT) is the published one, ${WHY:-FORCE=1}: publishing it again"
    elif [ "$BUILD" -le "$LIVE_BUILD" ]; then
        echo "Build $BUILD ($COMMIT) is not newer than the published build $LIVE_BUILD" >&2
        [ "${FORCE:-0}" = 1 ] || { echo "(FORCE=1 publishes it anyway)" >&2; exit 1; }
        echo "FORCE=1: publishing it anyway" >&2
        echo "WARNING: no board takes build $BUILD over the air if it runs build $LIVE_BUILD" >&2
        echo "WARNING: (not newer than its own: --flash over USB), or saw a newer one signed" >&2
        echo "WARNING: (ota state superseded: OTA_SEEN_RESET=1 ./deploy.sh over USB)" >&2
    fi
    if [ -z "$RESUME" ]; then
    rm -rf "$OUT/ota"
    mkdir -p "$OUT/ota"
    cp "$OUT/micropython.bin" "$OUT/ota/$BINF"
    # stripped and gzipped as deploy.sh does it, the firmware's own window
    python3 tools/web_min.py web.html "$OUT/ota/web-$COMMIT.html.gz"
    # the manifest and its HMAC (tools/firmware.py: the string the board checks)
    python3 tools/firmware.py manifest "$OUT/ota" "$KEY_FILE" "$BUILD" "$COMMIT" "$BOARD"
    # each as <name>.tmp, then moved over the live one (one rename: a board
    # downloading it keeps reading the old file), the named files first,
    # the manifest naming them last
    # the sources once more: one edited during the build (web.html is
    # minified just above) went up under the clean commit's name
    if [ "$(src_sum)" != "$SRC_SUM" ]; then
        echo "The sources changed during the build: nothing published (build again)" >&2
        exit 1
    fi
    MV=""
    for f in "$BINF" "web-$COMMIT.html.gz" manifest.json; do
        scp "${SSHO[@]}" -q "$OUT/ota/$f" "$DHOST:$DDIR/$f.tmp"
        MV+="mv -f $QDIR/$f.tmp $QDIR/$f && "
    done
    # (MV built here, its paths %q-quoted)
    # shellcheck disable=SC2029
    ssh "${SSHO[@]}" -n "$DHOST" "${MV}true"
    echo "Published $COMMIT (build $BUILD) to $OTA_DEST"
    fi
    # the files only the replaced manifest named go (an older signed
    # manifest and its image stayed on the server to be served again; the
    # HMAC binds no freshness): after the new manifest is up, never one the
    # new manifest names, plain names only (tools/firmware.py old)
    # And any build's image, page or .tmp in the folder the new manifest
    # does not name: one a failed cleanup or upload left was named by no
    # later manifest and stayed for good (served again, a rollback target)
    RM=""
    OLD=()
    if [ -n "$LIVE" ] && [ -z "$RESUME" ]; then
        read -r -a OLD <<< "$(python3 tools/firmware.py old "$BINF" "web-$COMMIT.html.gz" <<< "$LIVE" 2>/dev/null | tr '\n' ' ' || true)"
    fi
    # (expanded here on purpose: QDIR is quoted for the remote shell)
    # shellcheck disable=SC2029
    if [ -z "$RESUME" ] && ! LS=$(ssh "${SSHO[@]}" -n "$DHOST" "ls -1A $QDIR"); then
        echo "WARNING: could not list $OTA_DEST: leftovers are not removed" >&2
        LS=
    fi
    while IFS= read -r f; do
        case "$f" in
            "$BINF"|"web-$COMMIT.html.gz"|manifest.json) ;;
            micropython-*.bin|web-*.html.gz) OLD+=("$f") ;;
        esac
    done <<< "$LS"
    SEEN=" "
    for f in "${OLD[@]+"${OLD[@]}"}"; do
        [[ $f =~ ^[A-Za-z0-9._-]+$ ]] || continue
        case "$SEEN" in *" $f "*) continue ;; esac
        SEEN+="$f "
        RM+=" $QDIR/$(printf %q "$f")"
    done
    # a .tmp only once an hour old: one younger is another publish's
    # upload in progress (its move would fail); one older a cut upload's
    # shellcheck disable=SC2029
    if [ -n "$LS_OK" ] && grep -q '\.tmp$' <<< "$LS"; then
        ssh "${SSHO[@]}" -n "$DHOST" "find -H $QDIR -maxdepth 1 -type f -name '*.tmp' -mmin +60 -delete" \
            || echo "WARNING: could not remove the old .tmp files in $OTA_DEST" >&2
    fi
    if [ -n "$RM" ]; then
        # (RM built here, its paths %q-quoted)
        # shellcheck disable=SC2029
        if ssh "${SSHO[@]}" -n "$DHOST" "rm -f$RM"; then
            echo "Removed the previous build's files:$RM"
        else
            echo "WARNING: could not remove the previous build's files:$RM" >&2
        fi
    fi
    ;;
esac
