#!/bin/bash
# Compile gps_logger.py to .mpy and install it (with main.py and the status
# page, only gzipped: web.html.gz) on the ESP32, then reboot.
# It must run precompiled: compiling on the device grows the Python heap,
# which leaves too little IDF heap for TLS uploads (-O3: no line numbers in
# tracebacks, ~10 KB less, as in the frozen firmware). main.py imports
# gps_logger at boot (hold the boot-bypass pin, GPIO7, during reset to skip it;
# main.py's SKIP_PIN).
# On a firmware with the logger frozen in (tools/build_firmware.sh) only
# main.py and the page go up, and only if the frozen logger is this
# gps_logger.py (same SHA-256; FORCE=1 deploys them anyway). FORCE_MPY=1
# installs gps_logger.mpy even there (an emergency: it shadows the frozen
# logger and brings its ~100 KB back into the Python heap).
#
# The files are copied next to the old ones (<name>.dpl), checked (size and
# SHA-256 on the board) and only then renamed over them: a copy cut short
# on a nearly full flash used to leave an unimportable gps_logger.mpy and
# the board in the REPL. Any failure stops here, old files untouched.
# (Not <name>.new: an OTA stages the page it installs as /web.html.gz.new.)
# While an OTA install is being validated (/ota.pending) it stops: its
# reset would abort the pending firmware (the bootloader rolls back and the
# build is blamed). Wait the ~2 minutes, or FORCE_PENDING=1 (not FORCE=1:
# that one is for a stale frozen logger only, and one variable meant for
# that walked past this guard too). That is the first
# question to the board: each mpremote session stops the logger (raw REPL),
# so every exit restarts it, by a reset, or with an OTA pending by running
# the logger again without one (a soft reset does not run main.py from the
# raw REPL; left stopped, the WDT reset the board: a boot failure, and an
# aborted OTA).
#
# Everything made here (the .mpy files, web.html.gz) is made first, in a
# temporary directory, before the first session: a local failure (mpy-cross,
# tools/web_min.py) used to come after the logger was stopped, and its exit
# reset a healthy board for nothing.
#
# Usage: ./deploy.sh            (the board's USB Serial/JTAG, /dev/ttyACM0;
#                                PORT=/dev/ttyACM1 ./deploy.sh to override;
#                                VENV=path for a venv other than ./.venv,
#                                relative to where you run it)
set -euo pipefail
# (a relative VENV is the caller's: resolved before the cd below)
case "${VENV:-.venv}" in
    /*) ;;
    *) [ -z "${VENV:-}" ] || VENV=$PWD/$VENV ;;
esac
# (CA_DER too: a relative one was looked for in the script's folder, not
# found, and skipped without a word)
case "${CA_DER:-}" in
    ""|/*) ;;
    *) CA_DER=$PWD/$CA_DER ;;
esac
cd "$(dirname "$0")"

PORT=${PORT:-/dev/ttyACM0}               # the Heltec's USB Serial/JTAG
VENV=${VENV:-$PWD/.venv}                 # python, pip and mpremote come from it
PY=$VENV/bin/python
MPREMOTE=$VENV/bin/mpremote
# (the display's two modules too: without them a plain firmware's TFT stayed
# dark, "tft ImportError")
# (main.py last: the swap below puts it in after the rest, a cut swap
# then starts the old main.py over the files it knows)
FILES="gps_logger.mpy gps_clock.mpy gps_receiver.mpy gps_tft.mpy gps_tft_font.mpy web.html.gz main.py"
ARCH=xtensawin                           # the Heltec's ESP32-S3 (checked below)

# A device path that is not there: say so, before anything else (mpremote's
# own error, and the restart below, said less)
case "$PORT" in
    /*) if [ ! -e "$PORT" ]; then
            echo "No such port: $PORT (the board's USB Serial/JTAG is usually /dev/ttyACM0; PORT=... to choose)" >&2
            exit 1
        fi ;;
esac

# mpy-cross must match the firmware's .mpy version (MicroPython 1.29): an
# other one installed made .mpy files the board refused at import (5 failed
# boots, the REPL) while this said "Deployed"
if [ ! -x "$PY" ]; then
    echo "No Python in $VENV (VENV=... to choose; see docs/setup.md)" >&2
    exit 1
fi
if ! "$PY" -c "import mpy_cross" 2>/dev/null; then
    "$VENV/bin/pip" install -q "mpy-cross==1.29.*"
fi
MPYV=$("$PY" -c 'import importlib.metadata as m; print(m.version("mpy-cross"))' 2>/dev/null || true)
case "$MPYV" in
    1.29.*) ;;
    *) echo "mpy-cross $MPYV is not 1.29.x: $VENV/bin/pip install 'mpy-cross==1.29.*'" >&2
       exit 1 ;;
esac

# What goes up, made here before the board is asked anything: the logger's
# modules (sent unless the firmware has them frozen in) and main.py, and the
# status page only gzipped: sent as is to browsers (Accept-Encoding: gzip),
# inflated on the board for other clients (curl). tools/web_min.py strips
# it and makes the gzip, with the firmware's WEB_GZ_WBITS window (see
# there); web.html itself keeps its comments.
BUILD=$(mktemp -d "${TMPDIR:-/tmp}/deploy.XXXXXX")
trap 'rm -rf "$BUILD"' EXIT
for m in gps_logger gps_clock gps_receiver gps_tft gps_tft_font; do
    "$PY" -m mpy_cross -O3 -march="$ARCH" -o "$BUILD/$m.mpy" "$m.py"
done
cp main.py "$BUILD/main.py"
"$PY" tools/web_min.py web.html "$BUILD/web.html.gz"

# The swap (LittleFS renames over an existing file in one step; other file
# systems need the old one removed first), main.py last. $1 True: every .dpl
# must be there (the first try: the copy check just saw them); False: what is
# left of a cut one, a file already swapped skipped (it must be there). The
# old plain web.html goes in the same step: the board inflates web.html.gz
# for a client without gzip now (a staged OTA page, /web.html.gz.new, is
# left alone). A leftover gps_logger.py would be imported instead of the
# .mpy (or the frozen module: '' comes first in sys.path); the display's
# .py/.mpy stay on a frozen firmware without them (KEEP_TFT). The RTC
# boot-fail counter cleared so a previous crash lock (REPL after 5 failures)
# does not survive this deploy reset (power-on also clears it).
swap_py() {
    printf '%s' "
import os
F = '$FILES'.split()
if $1:
    for n in F:
        os.stat(n + '.dpl')
for n in F:
    if n + '.dpl' not in os.listdir():
        os.stat(n)
        continue
    try:
        os.rename(n + '.dpl', n)
    except OSError:
        os.stat(n + '.dpl')
        print('removing', n)
        os.remove(n)
        os.rename(n + '.dpl', n)
    print('swapped', n)
D = ['web.html', 'gps_logger.py', 'gps_clock.py', 'gps_receiver.py']
if not $KEEP_TFT:
    D += ['gps_tft.py', 'gps_tft_font.py']
if $RM_MPY:
    D += ['gps_logger.mpy', 'gps_clock.mpy', 'gps_receiver.mpy']
    if not $KEEP_TFT:
        D += ['gps_tft.mpy', 'gps_tft_font.mpy']
if $SEEN_RESET:
    D += ['ota.seen']
for n in D:
    n in os.listdir() and os.remove(n)
from machine import RTC
RTC().memory(b'0')"
}

# The logger stopped by the sessions below runs again on every exit (the
# last step's own reset clears this). Unknown whether an OTA is pending:
# no reset either.
restart_logger() {
    local rc=$?
    # (a second Ctrl-C now killed the roll forward below with the script:
    # ignored from here, by the mpremote it starts too)
    trap '' INT TERM HUP
    # cut in the swap (a Ctrl-C, a lost link): a reset now booted a mixed
    # set (the old main.py over new modules), and left stopped the WDT
    # did the same a little later; the rest of it goes in first
    if [ "${SWAPPING:-0}" = 1 ]; then
        SWAPPING=0
        if ! "$MPREMOTE" connect "$PORT" exec "$(swap_py False)" >/dev/null 2>&1; then
            echo "Swap cut part way and not finished (the files not swapped are <name>.dpl):" >&2
            echo "run ./deploy.sh again now (the WDT resets a stopped logger)" >&2
            rm -rf "$BUILD"
            exit 1
        fi
        echo "Swap cut part way: the rest swapped in" >&2
        [ "$rc" != 0 ] || rc=1
    fi
    if [ "${OTAPEND:-}" = False ]; then
        if ! "$MPREMOTE" connect "$PORT" reset >/dev/null 2>&1; then
            echo "WARNING: the reset failed: power the board off and on (or 'mpremote reset')" >&2
            [ "$rc" != 0 ] || rc=1
        fi
    else
        echo "Running the logger again without a reset (OTA pending or unknown)" >&2
        "$MPREMOTE" connect "$PORT" exec --no-follow \
            "import gps_logger; gps_logger.run()" >/dev/null 2>&1 || true
    fi
    rm -rf "$BUILD"
    exit "$rc"
}
trap restart_logger EXIT

# An OTA being validated? And is it the board: a Heltec Wireless Tracker (an
# ESP32-S3: Xtensa); the logger runs on nothing else
BOARD=$("$MPREMOTE" connect "$PORT" exec "
import os, sys
print('ota.pending', 'ota.pending' in os.listdir())
print(sys.implementation._machine)" | tr -d '\r' || true)
OTAPEND=$(grep -E '^ota.pending (True|False)$' <<< "$BOARD" | tail -n 1 | cut -d' ' -f2 || true)
if [ "$OTAPEND" != False ]; then
    if [ "$OTAPEND" = True ]; then
        echo "An OTA install is being validated on the board (/ota.pending): its reset" >&2
        echo "now would roll it back. Wait ~2 minutes (status page: ota ok), then deploy." >&2
    else
        echo "Could not tell whether an OTA install is pending: '$BOARD'" >&2
    fi
    if [ "${FORCE_PENDING:-0}" != 1 ]; then
        echo "(FORCE_PENDING=1 deploys anyway, without a reset; FORCE=1 does not)" >&2
        exit 1
    fi
    echo "FORCE_PENDING=1: deploying during the OTA validation" >&2
fi
MACHINE=$(tail -n 1 <<< "$BOARD")
case "$MACHINE" in
    *Heltec*) ;;
    *) echo "Unsupported board: $MACHINE" >&2; exit 1 ;;
esac
echo "Board: $MACHINE ($ARCH)"

# A firmware built by tools/build_firmware.sh has the logger frozen in (and
# a gps_frozen marker with the SHA-256 of the gps_logger.py it was built
# from, TFT for the display's modules, BOOT for gps_boot.py, main.py's
# OTA/boot-retry helpers, CLOCK for gps_clock.py and RX for gps_receiver.py): a gps_logger.mpy on
# flash would shadow it and bring its ~100 KB back into the Python heap, so
# only main.py and the page go up. Only an explicit answer counts: anything else (a garbled or missing
# reply) stops. An older marker has no TFT, BOOT or CLOCK ('-'): those are
# not compared then.
FROZEN=$("$MPREMOTE" connect "$PORT" exec "
try:
    import gps_frozen
    print('frozen', gps_frozen.COMMIT, getattr(gps_frozen, 'SHA', '-'), getattr(gps_frozen, 'TFT', '-'),
          getattr(gps_frozen, 'BOOT', '-'), getattr(gps_frozen, 'CLOCK', '-'),
          getattr(gps_frozen, 'RX', '-'))
except ImportError:
    print('plain')" | tr -d '\r' | grep -E '^(frozen [A-Za-z0-9._-]+ [A-Za-z0-9._-]+( [A-Za-z0-9._-]+){0,4}|plain)$' | tail -n 1 || true)
RM_MPY=0                                 # (a frozen logger: drop a gps_logger.mpy that shadows it)
KEEP_TFT=0                               # (a frozen firmware without the display's code: its .py/.mpy stay)
# OTA_SEEN_RESET=1: the board forgets the newest signed build it saw
# (/ota.seen): an older build published on purpose is taken again
SEEN_RESET=0
[ "${OTA_SEEN_RESET:-0}" = 1 ] && SEEN_RESET=1
case "$FROZEN" in
    frozen*)
        read -r _ FCOMMIT FSHA FTFT FBOOT FCLOCK FRX <<< "$FROZEN"
        if [ "${FORCE_MPY:-0}" = 1 ]; then
            echo "WARNING: FORCE_MPY=1: installing gps_logger.mpy over the frozen logger ($FCOMMIT)."
            echo "WARNING: it shadows the frozen one and takes ~100 KB of the Python heap again;"
            echo "WARNING: rebuild with tools/build_firmware.sh --flash and deploy again soon."
            # (not the display's modules: the frozen ones stay, ~8 KB of
            # heap and a shadow of their frozen code for nothing)
            FILES="gps_logger.mpy gps_clock.mpy gps_receiver.mpy web.html.gz main.py"
        else
            # (the display's code, gps_boot.py, gps_clock.py and
            # gps_receiver.py on the board that has them in the marker: an older
            # firmware has no TFT, BOOT, CLOCK or RX, the logger's SHA alone then)
            # (tools/firmware.py: what each field hashes, the first that differs)
            STALE=$("$PY" tools/firmware.py stale "$FSHA" "${FTFT:--}" "${FBOOT:--}" \
                    "${FCLOCK:--}" "${FRX:--}")
            if [ -n "$STALE" ]; then
                echo "The frozen logger ($FCOMMIT) is not this $STALE: run" >&2
                echo "tools/build_firmware.sh --flash first (FORCE=1 deploys main.py and the page anyway)" >&2
                [ "${FORCE:-0}" = 1 ] || exit 1
                echo "FORCE=1: deploying main.py and the page next to a stale frozen logger" >&2
            fi
            FILES="web.html.gz main.py"
            RM_MPY=1
            [ -n "$FTFT" ] && [ "$FTFT" != - ] || KEEP_TFT=1
            echo "Firmware has the logger frozen in ($FCOMMIT): main.py and the page only;"
            echo "tools/build_firmware.sh --flash updates the logger itself"
        fi ;;
    plain) ;;
    *) echo "Could not tell whether the firmware has the logger frozen in: '$FROZEN'" >&2
       exit 1 ;;
esac
# The upload server's root certificate (DER, at most 4 KB) goes up as
# /ca.der when there is one: CA_DER=path, or ca.der next to this script
# (gitignored). Without it the logger posts no unverified TLS (see
# UPLOAD_UNVERIFIED in docs/setup.md).
CA_SRC=${CA_DER:-ca.der}
if [ -n "${CA_DER:-}" ] && [ ! -f "$CA_SRC" ]; then
    echo "CA_DER=$CA_DER: no such file" >&2
    exit 1
fi
if [ -f "$CA_SRC" ]; then
    CA_N=$(wc -c < "$CA_SRC")
    # (a DER SEQUENCE: 0x30, then a long length, 0x81 or 0x82, as every
    # certificate over 127 bytes has)
    CA_HEAD=$(head -c 2 "$CA_SRC" | od -An -tx1 | tr -d ' \n')
    if [ "$CA_N" -gt 4096 ] || { [ "$CA_HEAD" != 3081 ] && [ "$CA_HEAD" != 3082 ]; }; then
        echo "$CA_SRC is not a DER certificate of at most 4096 bytes" >&2
        exit 1
    fi
    cp "$CA_SRC" "$BUILD/ca.der"
    FILES="ca.der $FILES"
    echo "Root certificate: $CA_SRC ($CA_N B) -> /ca.der"
else
    echo "No ca.der (CA_DER=...): the board's /ca.der is left as it is"
fi

# Free flash, after dropping the .dpl files of an earlier failed deploy:
# each new file needs its size rounded up to whole blocks, plus one block
# of metadata, while the old one still exists
FREE=$("$MPREMOTE" connect "$PORT" exec "
import os
for n in os.listdir():
    if n.endswith('.dpl'):
        os.remove(n)
s = os.statvfs('/')
print('free', s[0], s[3])" | tr -d '\r' | grep '^free ' | tail -n 1 || true)
read -r _ BSIZE BFREE <<< "$FREE" || true
case "${BSIZE:-x}${BFREE:-x}" in
    *[!0-9]*) echo "Could not read the free flash of the board: $FREE" >&2; exit 1 ;;
esac
NEED=0
for f in $FILES; do
    NEED=$(( NEED + ($(wc -c < "$BUILD/$f") + BSIZE - 1) / BSIZE + 1 ))
done
echo "Flash: $(( BFREE * BSIZE / 1024 )) KB free, $(( NEED * BSIZE / 1024 )) KB needed"
if [ "$BFREE" -lt "$NEED" ]; then
    echo "Not enough free flash on the board: free some (old tracks in /gpx/, see docs/usage.md) and deploy again" >&2
    exit 1
fi

# Copy to <name>.dpl, then size + SHA-256 of each on the board
CP=()
EXPECT=""
for f in $FILES; do
    CP+=(cp "$f" ":$f.dpl" +)
    EXPECT+="$f $(( $(wc -c < "$BUILD/$f") )) $(sha256sum "$BUILD/$f" | cut -d' ' -f1)"$'\n'
done
# (from the build directory: the board's names are the files' own)
GOT=$(cd "$BUILD" || exit 1; "$MPREMOTE" connect "$PORT" "${CP[@]}" exec "
import os, hashlib, binascii
b = bytearray(512)
for n in '$FILES'.split():
    h = hashlib.sha256()
    f = open(n + '.dpl', 'rb')
    while True:
        k = f.readinto(b)
        if not k:
            break
        h.update(memoryview(b)[:k])
    f.close()
    print('sum', n, os.stat(n + '.dpl')[6], binascii.hexlify(h.digest()).decode())" \
    | tr -d '\r' | grep '^sum ' | cut -d' ' -f2- || true)
if [ "$GOT"$'\n' != "$EXPECT" ]; then
    echo "Copy check FAILED, the old files are kept. Expected:" >&2
    printf '%s' "$EXPECT" >&2
    echo "Got:" >&2
    printf '%s\n' "$GOT" >&2
    "$MPREMOTE" connect "$PORT" exec "
import os
for n in '$FILES'.split():
    try:
        os.remove(n + '.dpl')
    except OSError:
        pass" >/dev/null || true
    exit 1
fi
echo "Copied and checked: $FILES"

# Swap them in (swap_py). With an OTA pending (FORCE_PENDING=1) no reset:
# it would abort the pending image; the EXIT trap runs the logger again and
# the new files count from the next boot.
SWAPPING=1
if ! SWAPPED=$("$MPREMOTE" connect "$PORT" exec "$(swap_py True)"); then
    printf '%s\n' "$SWAPPED" | tr -d '\r'
    SWAPPED_OUT=$SWAPPED
    SWAPPED=
    if ! grep -qE "^(swapped|removing) " <<< "$(tr -d '\r' <<< "$SWAPPED_OUT")"; then
        # nothing changed (a .dpl gone since the copy check): the old
        # install is whole, the rest of it is not to be swapped in
        SWAPPING=0
        echo "Swap refused: a copied file is gone, nothing changed (the old install stays)" >&2
        exit 1
    fi
    # (the "swapped" lines above say which went in: the rest goes in now, the
    # same way, rather than a reset onto a mixed set)
    echo "Swap failed part way: swapping the rest in" >&2
    if ! "$MPREMOTE" connect "$PORT" exec "$(swap_py False)"; then
        SWAPPING=0
        trap 'rm -rf "$BUILD"' EXIT
        echo "Swap FAILED part way: the files not swapped are still <name>.dpl." >&2
        echo "No reset onto a mixed set: run ./deploy.sh again now (the WDT resets a stopped logger)" >&2
        exit 1
    fi
fi
printf '%s\n' "$SWAPPED" | tr -d '\r'
SWAPPING=0
# The reset in a session of its own: one that fails (the USB link gone with
# it) is no failed swap, every file is in
if [ "$OTAPEND" = False ]; then
    if "$MPREMOTE" connect "$PORT" ls : + reset; then
        trap 'rm -rf "$BUILD"' EXIT      # (the reset done: only the build dir)
    else
        echo "WARNING: every file is in, the reset failed: trying it once more" >&2
    fi
else
    "$MPREMOTE" connect "$PORT" ls : || true
    echo "No reset (OTA pending or unknown): the new files count from the next boot" >&2
fi

echo "Deployed to $PORT"
