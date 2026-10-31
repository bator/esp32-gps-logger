#!/bin/bash
# Compile gps_logger.py to .mpy and install it (with main.py and the status
# page web.html) on the ESP32, then reboot.
# It must run precompiled: compiling on the device grows the Python heap,
# which leaves too little IDF heap for TLS uploads. main.py imports
# gps_logger at boot (hold GPIO25 during reset to skip).
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
FILES="gps_logger.mpy main.py web.html"

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

$PY -m mpy_cross -march="$ARCH" -o gps_logger.mpy gps_logger.py

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
# A leftover gps_logger.py would be imported instead of the .mpy.
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
'gps_logger.py' in os.listdir() and os.remove('gps_logger.py')" \
    + exec "from machine import RTC; RTC().memory(b'0')" \
    + ls : \
    + reset

echo "Deployed to $PORT"
