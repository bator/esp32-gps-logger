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
# Usage: ./deploy.sh            (PORT=/dev/ttyUSB1 ./deploy.sh to override;
#                                VENV=path for a venv other than ./.venv,
#                                relative to where you run it)
set -euo pipefail
# (a relative VENV is the caller's: resolved before the cd below)
case "${VENV:-.venv}" in
    /*) ;;
    *) [ -z "${VENV:-}" ] || VENV=$PWD/$VENV ;;
esac
cd "$(dirname "$0")"

PORT=${PORT:-/dev/ttyUSB0}
VENV=${VENV:-.venv}                      # python, pip and mpremote come from it
PY=$VENV/bin/python
MPREMOTE=$VENV/bin/mpremote
FILES="gps_logger.mpy gps_clock.mpy gps_receiver.mpy main.py web.html.gz"

# mpy-cross must match the firmware's .mpy version (MicroPython 1.29)
if ! "$PY" -c "import mpy_cross" 2>/dev/null; then
    "$VENV/bin/pip" install -q "mpy-cross==1.29.*"
fi

# The logger stopped by the sessions below runs again on every exit (the
# last step's own reset clears this). Unknown whether an OTA is pending:
# no reset either.
restart_logger() {
    local rc=$?
    if [ "${OTAPEND:-}" = False ]; then
        "$MPREMOTE" connect "$PORT" reset >/dev/null 2>&1 || true
    else
        echo "Running the logger again without a reset (OTA pending or unknown)" >&2
        "$MPREMOTE" connect "$PORT" exec --no-follow \
            "import gps_logger; gps_logger.run()" >/dev/null 2>&1 || true
    fi
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
    *Heltec*) ARCH=xtensawin ;;
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
case "$FROZEN" in
    frozen*)
        read -r _ FCOMMIT FSHA FTFT FBOOT FCLOCK FRX <<< "$FROZEN"
        if [ "${FORCE_MPY:-0}" = 1 ]; then
            echo "WARNING: FORCE_MPY=1: installing gps_logger.mpy over the frozen logger ($FCOMMIT)."
            echo "WARNING: it shadows the frozen one and takes ~100 KB of the Python heap again;"
            echo "WARNING: rebuild with tools/build_firmware.sh --flash and deploy again soon."
            "$PY" -m mpy_cross -O3 -march="$ARCH" -o gps_logger.mpy gps_logger.py
            "$PY" -m mpy_cross -O3 -march="$ARCH" -o gps_clock.mpy gps_clock.py
            "$PY" -m mpy_cross -O3 -march="$ARCH" -o gps_receiver.mpy gps_receiver.py
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
            FILES="main.py web.html.gz"
            RM_MPY=1
            echo "Firmware has the logger frozen in ($FCOMMIT): main.py and the page only;"
            echo "tools/build_firmware.sh --flash updates the logger itself"
        fi ;;
    plain) "$PY" -m mpy_cross -O3 -march="$ARCH" -o gps_logger.mpy gps_logger.py
           "$PY" -m mpy_cross -O3 -march="$ARCH" -o gps_clock.mpy gps_clock.py
           "$PY" -m mpy_cross -O3 -march="$ARCH" -o gps_receiver.mpy gps_receiver.py ;;
    *) echo "Could not tell whether the firmware has the logger frozen in: '$FROZEN'" >&2
       exit 1 ;;
esac
# The status page goes up only gzipped: sent as is to browsers
# (Accept-Encoding: gzip), inflated on the board for other clients (curl).
# tools/web_min.py strips it and makes the gzip, with the firmware's
# WEB_GZ_WBITS window (see there); web.html itself keeps its comments.
"$PY" tools/web_min.py web.html web.html.gz

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
    NEED=$(( NEED + ($(wc -c < "$f") + BSIZE - 1) / BSIZE + 1 ))
done
echo "Flash: $(( BFREE * BSIZE / 1024 )) KB free, $(( NEED * BSIZE / 1024 )) KB needed"
if [ "$BFREE" -lt "$NEED" ]; then
    echo "Not enough free flash on the board: free some (old tracks in /gpx/, see README) and deploy again" >&2
    exit 1
fi

# Copy to <name>.dpl, then size + SHA-256 of each on the board
CP=()
EXPECT=""
for f in $FILES; do
    CP+=(cp "$f" ":$f.dpl" +)
    EXPECT+="$f $(( $(wc -c < "$f") )) $(sha256sum "$f" | cut -d' ' -f1)"$'\n'
done
GOT=$("$MPREMOTE" connect "$PORT" "${CP[@]}" exec "
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

# Swap them in (LittleFS renames over an existing file in one step; other
# file systems need the old one removed first). The old plain web.html goes
# in the same step: the board inflates web.html.gz for a client without
# gzip now (a staged OTA page, /web.html.gz.new, is left alone).
# A leftover gps_logger.py would be imported instead of the .mpy (or the
# frozen module: '' comes first in sys.path).
# Clear the RTC boot-fail counter so a previous crash lock (REPL after 5
# failures) does not survive this deploy reset. Power-on also clears it.
# With an OTA pending (FORCE_PENDING=1) no reset: it would abort the
# pending image; the EXIT trap runs the logger again and the new files
# count from the next boot.
RESET=(+ reset)
[ "$OTAPEND" = False ] || RESET=()
"$MPREMOTE" connect "$PORT" \
    exec "
import os
for n in '$FILES'.split():
    try:
        os.rename(n + '.dpl', n)
    except OSError:
        os.remove(n)
        os.rename(n + '.dpl', n)
'web.html' in os.listdir() and os.remove('web.html')
'gps_logger.py' in os.listdir() and os.remove('gps_logger.py')
'gps_clock.py' in os.listdir() and os.remove('gps_clock.py')
'gps_receiver.py' in os.listdir() and os.remove('gps_receiver.py')
$RM_MPY and 'gps_logger.mpy' in os.listdir() and os.remove('gps_logger.mpy')
$RM_MPY and 'gps_clock.mpy' in os.listdir() and os.remove('gps_clock.mpy')
$RM_MPY and 'gps_receiver.mpy' in os.listdir() and os.remove('gps_receiver.mpy')" \
    + exec "from machine import RTC; RTC().memory(b'0')" \
    + ls : \
    ${RESET[@]+"${RESET[@]}"}
if [ "$OTAPEND" = False ]; then
    trap - EXIT
else
    echo "No reset (OTA pending or unknown): the new files count from the next boot" >&2
fi

echo "Deployed to $PORT"
