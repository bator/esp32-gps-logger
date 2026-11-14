# The LoRa radio: the Heltec Wireless Tracker's SX1262 sends the latest
# position as a Meshtastic packet, on a private channel of the EU_868
# MediumFast preset, while the board has no WiFi (on the train). Any
# Meshtastic node on that preset relays it (routers relay a channel they
# cannot read); the home node, which has the channel's key, takes it to MQTT
# and tools/lora_bridge.py on to Dawarich (README, "LoRa").
# Frozen with the logger (tools/build_firmware.sh), or installed beside it as
# gps_lora.mpy (deploy.sh). The logger imports it only when LoRa is on (.env
# LORA, a key): start() once, tick() every loop. Importable on a PC: the
# radio is reached only through _spi / _nss / _busy / _dio1 (tests replace
# them) and cryptolib (tests give a pure-Python one).
import math, struct, time
try:
    import cryptolib
except ImportError:
    cryptolib = None
try:
    from os import urandom
except ImportError:
    urandom = None

# The board (Heltec HTIT-Tracker V1.1 schematic; Meshtastic's variant
# heltec_wireless_tracker agrees): the SX1262 on its own SPI bus, its TCXO
# powered by DIO3 (1.8 V), DIO2 switching the antenna to TX. SPI(1) is the
# TFT's (gps_tft), so SPI(2).
NSS, SCK, MOSI, MISO, RST, BUSY, DIO1 = 8, 9, 10, 11, 12, 13, 14
SPI_ID = 2
SPI_HZ = 8000000

# The air: Meshtastic EU_868 (869.4-869.65 MHz, the ETSI band with 10 % duty
# cycle and 500 mW ERP): one 250 kHz slot at 869.525 MHz. MediumFast (SF9,
# what most Hungarian nodes listen on) has a quarter of LongFast's airtime.
FREQ = 869525000                         # Hz
SF = 9
BW = 250000                              # Hz (SX126x code 0x05)
CR = 5                                   # 4/5 (code 1)
PREAMBLE = 16                            # symbols
SYNC = (0x24, 0xB4)                      # Meshtastic's sync word 0x2B as the
                                         # SX126x registers take it (RadioLib)
DBM = 22                                 # the SX1262's most; with the stick's
                                         # ~2 dBi under the 27 dBm ERP
HOPS = 3                                 # Meshtastic's default hop limit
TX_MAX = 3000                            # ms one TX may take (~190 ms) before
                                         # it counts as failed
BUSY_MAX = 20                            # ms BUSY may stay high per command

# When: a position every EVERY ms while it moved MOVE m from the last one
# sent, every IDLE ms when it did not (a stay, a station). The logger skips
# it on the home WiFi (the upload has it). DUTY keeps the own airtime to 1 %
# (36 s an hour, ~180 packets), a tenth of the band's 10 %: each packet is
# relayed up to HOPS times on the public mesh.
EVERY = 60000
MOVE = 50                                # m
IDLE = 900000
DUTY = 100                               # 1/DUTY of the time on the air
DUTY_CAP = 36000                         # ms of airtime saved up at most
NODEINFO_EVERY = 10800000                # ms between the node's name packets
CHECK = 1000                             # ms between looks at the fix

# Meshtastic: the channel, the ports, the packet's fields (mesh.proto)
CHANNEL = "GpsLog"
PORT_POSITION = 3
PORT_NODEINFO = 4
HW_MODEL = 48                            # HELTEC_WIRELESS_TRACKER
BROADCAST = 0xFFFFFFFF

_FRF = FREQ * 33554432 // 32000000       # SetRfFrequency: f * 2^25 / 32 MHz

r = {}                                   # the radio's state (reset())


def reset():
    r.clear()
    r.update(on=False, node=0, key=None, chash=0, hops=HOPS, name="",
             tx_at=None, tx_ms=0, sent=0, fail=0, air=0, tok=DUTY_CAP // 4,
             tok_at=None, last=None, last_at=None, info_at=None, check_at=None,
             err=None)


reset()

_spi = _nss = _busy = _dio1 = None


# --- the SX1262 --------------------------------------------------------------

def _wait():
    t = time.ticks_ms()
    while _busy.value():
        if time.ticks_diff(time.ticks_ms(), t) > BUSY_MAX:
            raise OSError("lora busy")


def _cmd(op, data=b""):
    _wait()
    _nss(0)
    _spi.write(bytes((op,)) + bytes(data))
    _nss(1)


def _read(op, pre, n):
    """Command op with the bytes pre, then the status byte and n bytes read."""
    _wait()
    b = bytearray(bytes((op,)) + bytes(pre) + bytes(n + 1))
    _nss(0)
    _spi.write_readinto(b, b)
    _nss(1)
    return b[len(pre) + 2:]


def _reg_w(addr, data):
    _cmd(0x0D, bytes((addr >> 8, addr & 0xFF)) + bytes(data))


def _reg_r(addr, n=1):
    return _read(0x1D, (addr >> 8, addr & 0xFF), n)


def _wake():
    """An NSS edge wakes it from sleep (warm: the setup kept)."""
    _nss(0)
    time.sleep_ms(1)
    _nss(1)
    _wait()


def _sleep():
    _cmd(0x84, b"\x04")                  # SetSleep, warm start


def _irq():
    b = _read(0x12, b"", 2)              # GetIrqStatus
    return b[0] << 8 | b[1]


def _setup(dbm):
    """The SX1262 from its reset to LoRa TX at FREQ, then asleep."""
    _cmd(0x80, b"\x00")                  # SetStandby RC
    _cmd(0x97, b"\x02\x00\x01\x40")      # DIO3 powers the TCXO: 1.8 V, 5 ms
    _cmd(0x89, b"\x7f")                  # Calibrate all (with the TCXO)
    time.sleep_ms(5)
    _cmd(0x96, b"\x01")                  # SetRegulatorMode DC-DC
    _cmd(0x9D, b"\x01")                  # DIO2 is the RF switch
    _cmd(0x8A, b"\x01")                  # SetPacketType LoRa
    _cmd(0x86, struct.pack(">I", _FRF))  # SetRfFrequency
    _cmd(0x98, b"\xd7\xdb")              # CalibrateImage 863-870 MHz
    _cmd(0x95, b"\x04\x07\x00\x01")      # SetPaConfig: SX1262, +22 dBm table
    _cmd(0x8E, bytes((dbm & 0xFF, 0x04)))  # SetTxParams, 200 us ramp
    _reg_w(0x08E7, b"\x38")              # OCP 140 mA
    _reg_w(0x0740, bytes(SYNC))          # LoRa sync word
    v = _reg_r(0x08D8)[0]                # datasheet 15.2: the PA clamp
    _reg_w(0x08D8, bytes((v | 0x1E,)))
    _cmd(0x8B, bytes((SF, 0x05, CR - 4, 0)))   # SetModulationParams (no LDRO:
                                               # a 2 ms symbol)
    _cmd(0x8F, b"\x00\x00")              # SetBufferBaseAddress
    _cmd(0x08, b"\x02\x01\x02\x01\x00\x00\x00\x00")  # TxDone|Timeout on DIO1
    _sleep()


def airtime_ms(n):
    """ms on the air for an n-byte payload (Semtech's formula: explicit
    header, CRC on, no low data rate optimisation)."""
    tsym = (1 << SF) * 1000000 // BW     # us
    num = 8 * n - 4 * SF + 28 + 16
    den = 4 * SF
    sym = 8 + max(-(-num // den) * CR, 0)
    return ((PREAMBLE * 4 + 17) * tsym // 4 + sym * tsym + 999) // 1000


# --- Meshtastic ----------------------------------------------------------------

def channel_hash(name, key):
    """The header's channel byte: the name's bytes XOR the key's."""
    h = 0
    for c in name.encode():
        h ^= c
    for c in key:
        h ^= c
    return h


def _varint(b, v):
    v &= 0xFFFFFFFFFFFFFFFF              # a negative int32 as protobuf has it
    while v > 0x7F:
        b.append((v & 0x7F) | 0x80)
        v >>= 7
    b.append(v)


def _field_v(b, tag, v):
    _varint(b, tag)
    _varint(b, v)


def _field_b(b, tag, data):
    _varint(b, tag)
    _varint(b, len(data))
    b.extend(data)


def position(lat, lon, alt=None, t=None, kmh=None, sats=None, hdop=None):
    """A Position (latitude_i, longitude_i, altitude, time, location_source,
    HDOP*100, ground_speed m/s, sats_in_view, precision_bits 32)."""
    b = bytearray(b"\x0d")
    b.extend(struct.pack("<i", int(round(lat * 1e7))))
    b.append(0x15)
    b.extend(struct.pack("<i", int(round(lon * 1e7))))
    if alt is not None:
        _field_v(b, 0x18, int(round(alt)))
    if t is not None:
        b.append(0x25)
        b.extend(struct.pack("<I", t))
    _field_v(b, 0x28, 2)                 # LOC_INTERNAL
    if hdop is not None:
        _field_v(b, 0x60, int(round(hdop * 100)))
    if kmh is not None:
        _field_v(b, 0x78, int(round(kmh / 3.6)))
    if sats:
        _field_v(b, 0x98, sats)
    _field_v(b, 0xB8, 32)
    return b


def nodeinfo(node, name):
    """A User: id "!xxxxxxxx", long and short name, the hardware model."""
    b = bytearray()
    _field_b(b, 0x0A, ("!%08x" % node).encode())
    _field_b(b, 0x12, name.encode())
    _field_b(b, 0x1A, ("%04x" % (node & 0xFFFF)).encode())
    _field_v(b, 0x28, HW_MODEL)
    return b


def data(port, payload):
    """A Data: portnum, payload, bitfield OK_TO_MQTT (a gateway uplinks it)."""
    b = bytearray()
    _field_v(b, 0x08, port)
    _field_b(b, 0x12, payload)
    _field_v(b, 0x48, 1)
    return b


def nonce(node, pid):
    """Meshtastic's CryptoEngine nonce: the packet id (u64 LE), the sender
    (u32 LE), a 4-byte big-endian block counter from 0."""
    return struct.pack("<QI", pid, node) + b"\0\0\0\0"


def ctr(key, iv, b):
    """b encrypted (or decrypted) in place, AES-CTR with the 16-byte iv whose
    last 4 bytes count the blocks (big-endian, as Meshtastic's CTR with
    setCounterSize(4)). cryptolib's ECB makes the key stream: its CTR mode
    is not in the esp32 port."""
    aes = cryptolib.aes(key, 1)
    blk = bytearray(iv)
    c0 = struct.unpack(">I", iv[12:16])[0]
    i = 0
    while i < len(b):
        struct.pack_into(">I", blk, 12, (c0 + (i >> 4)) & 0xFFFFFFFF)
        ks = aes.encrypt(blk)
        for j in range(min(16, len(b) - i)):
            b[i + j] ^= ks[j]
        i += 16
    return b


def packet(node, pid, key, chash, hops, payload):
    """The over-the-air packet: the 16-byte header (to, from, id, flags
    hop_limit | hop_start << 5, channel hash, next hop, relay node) and the
    encrypted Data."""
    h = struct.pack("<IIIBBBB", BROADCAST, node, pid, hops | hops << 5, chash,
                    0, node & 0xFF)
    return h + ctr(key, nonce(node, pid), bytearray(payload))


# --- the logger's side ---------------------------------------------------------

def start(key, node, name="", dbm=DBM, hops=HOPS):
    """The radio set up and asleep; True. key: the channel's 16 or 32 bytes,
    node: the Meshtastic node number (the logger's: from the MAC). A board
    without the radio raises (the logger runs on without it)."""
    global _spi, _nss, _busy, _dio1
    reset()
    if cryptolib is None or len(key) not in (16, 32):
        raise ValueError("lora key")
    if _spi is None:
        from machine import Pin, SPI
        _nss = Pin(NSS, Pin.OUT, value=1)
        _busy = Pin(BUSY, Pin.IN)
        _dio1 = Pin(DIO1, Pin.IN)
        rst = Pin(RST, Pin.OUT, value=0)
        time.sleep_ms(2)
        rst(1)
        time.sleep_ms(10)
        _spi = SPI(SPI_ID, baudrate=SPI_HZ, polarity=0, phase=0, sck=Pin(SCK),
                   mosi=Pin(MOSI), miso=Pin(MISO))
    _setup(min(max(dbm, -9), 22))
    r.update(on=True, node=node, key=bytes(key), chash=channel_hash(CHANNEL, key),
             hops=min(max(hops, 0), 7), name=name or "GPS logger",
             tok_at=time.ticks_ms())
    return True


def _tokens(now):
    """The airtime budget at now (ms), DUTY of the time since, DUTY_CAP at most."""
    gain = time.ticks_diff(now, r["tok_at"]) // DUTY
    if gain > 0:
        r["tok"] = min(r["tok"] + gain, DUTY_CAP)
        r["tok_at"] = now if r["tok"] == DUTY_CAP else time.ticks_add(r["tok_at"], gain * DUTY)
    return r["tok"]


def _dist(a, b):
    x = math.radians(b[1] - a[1]) * math.cos(math.radians((a[0] + b[0]) / 2))
    y = math.radians(b[0] - a[0])
    return 6371000 * math.sqrt(x * x + y * y)


def due(now, pos):
    """Whether a position at pos (lat, lon) goes out now."""
    if r["last_at"] is None:
        return True
    age = time.ticks_diff(now, r["last_at"])
    if age >= IDLE:
        return True
    return age >= EVERY and _dist(r["last"], pos) >= MOVE


def _send(now, payload):
    """payload (a Data) on the air, if the budget has its airtime; True."""
    pid = struct.unpack("<I", urandom(4))[0] if urandom else time.ticks_ms()
    pid = pid or 1
    p = packet(r["node"], pid, r["key"], r["chash"], r["hops"], payload)
    ms = airtime_ms(len(p))
    if _tokens(now) < ms:
        return False
    _wake()
    _cmd(0x80, b"\x00")                  # SetStandby RC
    _cmd(0x8C, bytes((PREAMBLE >> 8, PREAMBLE & 0xFF, 0, len(p), 1, 0)))
    _cmd(0x0E, b"\x00" + p)              # WriteBuffer
    _cmd(0x02, b"\x03\xff")              # ClearIrqStatus
    _cmd(0x83, struct.pack(">I", TX_MAX * 64)[1:])   # SetTx, 15.625 us units
    r["tok"] -= ms
    r["tx_at"], r["tx_ms"] = now, ms
    return True


def _poll(now):
    """The TX under way: done (or failed, TX_MAX gone), the radio to sleep."""
    if not _dio1.value() and time.ticks_diff(now, r["tx_at"]) < TX_MAX + 500:
        return
    irq = _irq()
    _cmd(0x02, b"\x03\xff")
    _sleep()
    r["tx_at"] = None
    if irq & 1:
        r["sent"] += 1
        r["air"] += r["tx_ms"]
    else:
        r["fail"] += 1
        r["err"] = "tx irq %04x" % irq


def tick(now, fixfn):
    """Every loop. fixfn(): (lat, lon, alt, unix s, km/h, satellites, hdop)
    of a usable fix, or None (no fix, or the logger does not want it sent);
    asked at most every CHECK ms, not while a TX is under way. 'sent' when
    a position went on the air, else None."""
    if not r["on"]:
        return None
    if r["tx_at"] is not None:
        _poll(now)
        return None
    if r["check_at"] is not None and time.ticks_diff(now, r["check_at"]) < CHECK:
        return None
    r["check_at"] = now
    fix = fixfn()
    if fix is None:
        return None
    pos = (fix[0], fix[1])
    if r["info_at"] is None or time.ticks_diff(now, r["info_at"]) >= NODEINFO_EVERY:
        if _send(now, data(PORT_NODEINFO, nodeinfo(r["node"], r["name"]))):
            r["info_at"] = now
            r["check_at"] = None         # (the position once it is done)
        return None
    if not due(now, pos):
        return None
    if not _send(now, data(PORT_POSITION, position(*fix))):
        return None
    r["last"], r["last_at"] = pos, now
    return "sent"


def stop():
    """The radio asleep (an unfinished TX cut)."""
    if r["on"]:
        r["on"] = False
        try:
            _wake()
            _sleep()
        except OSError:
            pass
