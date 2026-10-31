#!/bin/bash
# Compile gps_logger.py to .mpy and install it (with main.py and the status
# page web.html, plain and gzipped) on the ESP32, then reboot.
# It must run precompiled: compiling on the device grows the Python heap,
# which leaves too little IDF heap for TLS uploads. main.py imports
# gps_logger at boot (hold GPIO25 during reset to skip).
# On a firmware with the logger frozen in (tools/build_firmware.sh) only
# main.py and the page go up, and only if the frozen logger is this
# gps_logger.py (same SHA-256; FORCE=1 deploys them anyway). FORCE_MPY=1
# installs gps_logger.mpy even there (an emergency: it shadows the frozen
# logger and brings its ~60 KB back into the Python heap).
#
# The files are copied next to the old ones (<name>.new), checked (size and
# SHA-256 on the board) and only then renamed over them: a copy cut short
# on a nearly full flash used to leave an unimportable gps_logger.mpy and
# the board in the REPL. Any failure stops here, old files untouched.
#
# Usage: ./deploy.sh            (PORT=/dev/ttyUSB1 ./deploy.sh to override)
set -euo pipefail
cd "$(dirname "$0")"

PORT=${PORT:-/dev/ttyUSB0}
PY=.venv/bin/python
MPREMOTE=.venv/bin/mpremote
FILES="gps_logger.mpy main.py web.html web.html.gz"

# mpy-cross must match the firmware's .mpy version (MicroPython 1.29)
if ! $PY -c "import mpy_cross" 2>/dev/null; then
    .venv/bin/pip install -q "mpy-cross==1.29.*"
fi

# ESP32 is Xtensa, ESP32-C6 is RISC-V: ask the board which one it is
MACHINE=$($MPREMOTE connect "$PORT" exec "import sys; print(sys.implementation._machine)" | tail -n 1)
case "$MACHINE" in
    *ESP32C6*) ARCH=rv32imc ;;
    *ESP32*)   ARCH=xtensawin ;;
    *) echo "Unsupported board: $MACHINE" >&2; exit 1 ;;
esac
echo "Board: $MACHINE ($ARCH)"

# A firmware built by tools/build_firmware.sh has the logger frozen in (and
# a gps_frozen marker with the SHA-256 of the gps_logger.py it was built
# from): a gps_logger.mpy on flash would shadow it and bring its ~60 KB back
# into the Python heap, so only main.py and the page go up. Only an
# explicit answer counts: anything else (a garbled or missing reply) stops.
FROZEN=$($MPREMOTE connect "$PORT" exec "
try:
    import gps_frozen
    print('frozen', gps_frozen.COMMIT, getattr(gps_frozen, 'SHA', '-'))
except ImportError:
    print('plain')" | tr -d '\r' | grep -E '^(frozen [^ ]+ [^ ]+|plain)$' | tail -n 1 || true)
KEEP_MPY=0
case "$FROZEN" in
    frozen*)
        read -r _ FCOMMIT FSHA <<< "$FROZEN"
        if [ "${FORCE_MPY:-0}" = 1 ]; then
            KEEP_MPY=1
            echo "WARNING: FORCE_MPY=1: installing gps_logger.mpy over the frozen logger ($FCOMMIT)."
            echo "WARNING: it shadows the frozen one and takes ~60 KB of the Python heap again;"
            echo "WARNING: rebuild with tools/build_firmware.sh --flash and deploy again soon."
            $PY -m mpy_cross -march="$ARCH" -o gps_logger.mpy gps_logger.py
        else
            if [ "$FSHA" != "$(sha256sum gps_logger.py | cut -d' ' -f1)" ]; then
                echo "The frozen logger ($FCOMMIT) is not this gps_logger.py: run" >&2
                echo "tools/build_firmware.sh --flash first (FORCE=1 deploys main.py and the page anyway)" >&2
                [ "${FORCE:-0}" = 1 ] || exit 1
                echo "FORCE=1: deploying main.py and the page next to a stale frozen logger" >&2
            fi
            FILES="main.py web.html web.html.gz"
            echo "Firmware has the logger frozen in ($FCOMMIT): main.py and the page only;"
            echo "tools/build_firmware.sh --flash updates the logger itself"
        fi ;;
    plain) $PY -m mpy_cross -march="$ARCH" -o gps_logger.mpy gps_logger.py ;;
    *) echo "Could not tell whether the firmware has the logger frozen in: '$FROZEN'" >&2
       exit 1 ;;
esac
# the status page also gzipped: sent to browsers (Accept-Encoding: gzip),
# a third of the bytes; web.html stays for other clients (curl)
gzip -9 -n -c web.html > web.html.gz

# Free flash, after dropping the .new files of an earlier failed deploy:
# each new file needs its size rounded up to whole blocks, plus one block
# of metadata, while the old one still exists
FREE=$($MPREMOTE connect "$PORT" exec "
import os
for n in os.listdir():
    if n.endswith('.new'):
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

# Copy to <name>.new, then size + SHA-256 of each on the board
CP=()
EXPECT=""
for f in $FILES; do
    CP+=(cp "$f" ":$f.new" +)
    EXPECT+="$f $(( $(wc -c < "$f") )) $(sha256sum "$f" | cut -d' ' -f1)"$'\n'
done
GOT=$($MPREMOTE connect "$PORT" "${CP[@]}" exec "
import os, hashlib, binascii
b = bytearray(512)
for n in '$FILES'.split():
    h = hashlib.sha256()
    f = open(n + '.new', 'rb')
    while True:
        k = f.readinto(b)
        if not k:
            break
        h.update(memoryview(b)[:k])
    f.close()
    print('sum', n, os.stat(n + '.new')[6], binascii.hexlify(h.digest()).decode())" \
    | tr -d '\r' | grep '^sum ' | cut -d' ' -f2- || true)
if [ "$GOT"$'\n' != "$EXPECT" ]; then
    echo "Copy check FAILED, the old files are kept. Expected:" >&2
    printf '%s' "$EXPECT" >&2
    echo "Got:" >&2
    printf '%s\n' "$GOT" >&2
    $MPREMOTE connect "$PORT" exec "
import os
for n in '$FILES'.split():
    try:
        os.remove(n + '.new')
    except OSError:
        pass" >/dev/null || true
    exit 1
fi
echo "Copied and checked: $FILES"

# Swap them in (LittleFS renames over an existing file in one step; other
# file systems need the old one removed first).
# A leftover gps_logger.py would be imported instead of the .mpy (or the
# frozen module: '' comes first in sys.path).
# Clear the RTC boot-fail counter so a previous crash lock (REPL after 5
# failures) does not survive this deploy reset. Power-on also clears it.
$MPREMOTE connect "$PORT" \
    exec "
import os
for n in '$FILES'.split():
    try:
        os.rename(n + '.new', n)
    except OSError:
        os.remove(n)
        os.rename(n + '.new', n)
'gps_logger.py' in os.listdir() and os.remove('gps_logger.py')
'$FROZEN'.startswith('frozen') and not $KEEP_MPY and 'gps_logger.mpy' in os.listdir() and os.remove('gps_logger.mpy')" \
    + exec "from machine import RTC; RTC().memory(b'0')" \
    + ls : \
    + reset

echo "Deployed to $PORT"
