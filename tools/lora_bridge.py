#!/usr/bin/env python3
"""The home side of the board's LoRa positions (gps_lora.py): a Meshtastic
node at home, on the board's private channel, uplinks what it hears to an
MQTT broker; this reads those packets, decrypts the board's positions and
posts them to Dawarich as an Overland batch (as the board's own upload).

  lora_bridge.py run [ENV_FILE]   the service (default /etc/gps-lora-bridge.env)
  lora_bridge.py key OTA_KEY_FILE the channel key a board derives from its
                                  .env OTA_KEY (base64: the home node's PSK)

ENV_FILE (KEY=VALUE lines, mode 600: it holds secrets):
  MQTT_HOST, MQTT_PORT (1883), MQTT_USER, MQTT_PASS
  MQTT_TOPIC   msh/# (the home node's root topic and below)
  LORA_KEY     the channel key, base64 (or OTA_KEY_FILE: derived from it)
  GEO_URL      Dawarich, as the board's .env (https://host)
  API_KEY      Dawarich's API key
  DEVICE_ID    the Overland device id (default: the board's node, !xxxxxxxx-lora)
  NODES        node ids to take (!a1b2c3d4,...; default: any on the channel)
  ALLOW_DECODED  1: also take packets a gateway decoded (its MQTT encryption
               off). Default 0: only encrypted ones, which only the channel
               key makes (anyone who may publish to the broker could forge a
               decoded one).

The channel is Meshtastic's AES-CTR: unauthenticated by design (the home
node must read it). A position is taken only with a time within MAX_AGE /
MAX_AHEAD of now and after the last one taken from its node: a replayed or
re-sent packet never becomes a point.

Needs python3 and paho-mqtt (Debian: apt install python3-paho-mqtt) for
"run"; the decoding is plain Python (no protobuf or crypto library), so
tests/test_gps_logger.py checks it against gps_lora's packets."""
import base64
import hashlib
import hmac
import json
import struct
import sys
import time
import urllib.request

CHANNEL = "GpsLog"                       # gps_lora.CHANNEL
LORA_SALT = b"gps-logger lora channel"   # gps_logger.LORA_SALT
PORT_POSITION = 3
MAX_AGE = 2 * 86400                      # s a position's time may be behind now
MAX_AHEAD = 600                          # s it may be ahead (clocks)

# --- AES (encryption only: CTR needs no more) ---------------------------------

_SBOX = bytearray(256)


def _sbox():
    p = q = 1
    while True:
        p = p ^ ((p << 1) & 0xFF) ^ (0x1B if p & 0x80 else 0)
        q ^= q << 1
        q ^= q << 2
        q ^= q << 4
        q &= 0xFF
        if q & 0x80:
            q ^= 0x09
        x = q ^ (q << 1 | q >> 7) ^ (q << 2 | q >> 6) ^ (q << 3 | q >> 5) ^ (q << 4 | q >> 4)
        _SBOX[p] = (x ^ 0x63) & 0xFF
        if p == 1:
            break
    _SBOX[0] = 0x63


_sbox()


def _xt(a):
    return ((a << 1) ^ 0x1B) & 0xFF if a & 0x80 else a << 1


class AES(object):
    """AES-128/256 block encryption (FIPS-197), as cryptolib.aes(key, 1)."""

    def __init__(self, key, mode=1):
        nk = len(key) // 4
        if len(key) not in (16, 32) or mode != 1:
            raise ValueError("AES-128/256 ECB only")
        self.nr = nk + 6
        w = [list(key[4 * i:4 * i + 4]) for i in range(nk)]
        rc = 1
        for i in range(nk, 4 * (self.nr + 1)):
            t = list(w[i - 1])
            if i % nk == 0:
                t = [_SBOX[t[1]] ^ rc, _SBOX[t[2]], _SBOX[t[3]], _SBOX[t[0]]]
                rc = _xt(rc)
            elif nk > 6 and i % nk == 4:
                t = [_SBOX[x] for x in t]
            w.append([w[i - nk][j] ^ t[j] for j in range(4)])
        self.rk = [sum(w[4 * r:4 * r + 4], []) for r in range(self.nr + 1)]

    def encrypt(self, block):
        s = [b ^ k for b, k in zip(block, self.rk[0])]
        for r in range(1, self.nr + 1):
            s = [_SBOX[b] for b in s]
            s = [s[(i + 4 * (i % 4)) % 16] for i in range(16)]      # ShiftRows
            if r != self.nr:
                m = []
                for c in range(4):
                    a = s[4 * c:4 * c + 4]
                    t = a[0] ^ a[1] ^ a[2] ^ a[3]
                    m += [a[j] ^ t ^ _xt(a[j] ^ a[(j + 1) % 4]) for j in range(4)]
                s = m
            s = [b ^ k for b, k in zip(s, self.rk[r])]
        return bytes(s)


def ctr(key, iv, b):
    """b decrypted (or encrypted), AES-CTR with a 4-byte big-endian block
    counter in the iv's last bytes (Meshtastic's; gps_lora.ctr)."""
    aes = AES(key)
    c0 = struct.unpack(">I", iv[12:16])[0]
    out = bytearray(b)
    for i in range(0, len(out), 16):
        ks = aes.encrypt(iv[:12] + struct.pack(">I", (c0 + i // 16) & 0xFFFFFFFF))
        for j in range(min(16, len(out) - i)):
            out[i + j] ^= ks[j]
    return bytes(out)


def nonce(node, pid):
    return struct.pack("<QI", pid, node) + b"\0\0\0\0"


def channel_hash(name, key):
    h = 0
    for c in name.encode() + bytes(key):
        h ^= c
    return h


def derived_key(ota_key):
    """The key a board derives from its .env OTA_KEY (gps_logger.lora_key)."""
    return hmac.new(ota_key, LORA_SALT, hashlib.sha256).digest()[:16]


# --- protobuf (the few messages read here) ------------------------------------

def _uv(b, i):
    v = s = 0
    while True:
        c = b[i]
        i += 1
        v |= (c & 0x7F) << s
        s += 7
        if not c & 0x80:
            return v, i


def fields(b):
    """{field number: [value, ...]}: varints as int, fixed32/64 as their
    bytes, length-delimited as bytes. Raises ValueError on a bad message."""
    out = {}
    i = 0
    try:
        while i < len(b):
            k, i = _uv(b, i)
            n, t = k >> 3, k & 7
            if t == 0:
                v, i = _uv(b, i)
            elif t == 1:
                v, i = bytes(b[i:i + 8]), i + 8
            elif t == 2:
                ln, i = _uv(b, i)
                v, i = bytes(b[i:i + ln]), i + ln
            elif t == 5:
                v, i = bytes(b[i:i + 4]), i + 4
            else:
                raise ValueError("wire type %d" % t)
            if i > len(b):
                raise ValueError("cut")
            out.setdefault(n, []).append(v)
    except IndexError:
        raise ValueError("cut")
    return out


def _one(f, n, default=None):
    v = f.get(n)
    return v[-1] if v else default


def _s32(v):
    return v - (1 << 64) if v >= 1 << 63 else (v - (1 << 32) if v >= 1 << 31 else v)


def position(b):
    """A Position's (lat, lon, alt, unix time, m/s, sats, hdop) or None."""
    f = fields(b)
    la, lo = _one(f, 1), _one(f, 2)
    if la is None or lo is None:
        return None
    alt = _one(f, 3)
    t = _one(f, 4)
    hd = _one(f, 12)
    return (struct.unpack("<i", la)[0] / 1e7, struct.unpack("<i", lo)[0] / 1e7,
            None if alt is None else _s32(alt),
            None if t is None else struct.unpack("<I", t)[0],
            _one(f, 15), _one(f, 19), None if hd is None else hd / 100.0)


def packet_data(mp, key, decoded=False):
    """A MeshPacket's (from, id, Data fields) with the channel key, or None
    (not ours: another channel, not decodable, or decoded by a gateway and
    decoded not allowed)."""
    f = fields(mp)
    frm, pid = _one(f, 1), _one(f, 6)
    if frm is None or pid is None:
        return None
    frm, pid = struct.unpack("<I", frm)[0], struct.unpack("<I", pid)[0]
    dec = _one(f, 4)
    if dec is not None and not decoded:
        return None
    if dec is None:
        enc = _one(f, 5)
        if enc is None:
            return None
        dec = ctr(key, nonce(frm, pid), enc)
    try:
        d = fields(dec)
    except ValueError:
        return None
    if not isinstance(_one(d, 1), int) or not isinstance(_one(d, 2), bytes):
        return None
    return frm, pid, d


def node_id(n):
    return "!%08x" % n


def handle(topic, payload, key, seen, nodes=None, device_id=None, decoded=False,
           now=None):
    """An MQTT message: the Overland features of the board's positions in
    it ([] for anything else). seen: the (from, id) already taken (the
    same packet comes again through every gateway and relay) and, under
    ("last", from), the newest time taken from each node."""
    if "/2/e/" not in topic:
        return []
    try:
        env = fields(payload)
        mp = _one(env, 1)
        ch = _one(env, 2)
        if mp is None or (ch is not None and ch.decode("utf-8", "replace") != CHANNEL):
            return []
        got = packet_data(mp, key, decoded)
    except ValueError:
        return []
    if got is None:
        return []
    frm, pid, d = got
    if _one(d, 1) != PORT_POSITION or (nodes and frm not in nodes):
        return []
    if (frm, pid) in seen:
        return []
    seen[(frm, pid)] = time.time()
    if len(seen) > 4096:
        for k in sorted((k for k in seen if k[0] != "last"), key=seen.get)[:2048]:
            del seen[k]
    try:
        p = position(_one(d, 2))
    except ValueError:
        return []
    if p is None or p[3] is None or not (-90 <= p[0] <= 90 and -180 <= p[1] <= 180):
        return []
    now = time.time() if now is None else now
    if not now - MAX_AGE <= p[3] <= now + MAX_AHEAD or p[3] <= seen.get(("last", frm), 0):
        return []
    seen[("last", frm)] = p[3]
    props = {"timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(p[3])),
             "device_id": device_id or node_id(frm) + "-lora"}
    if p[2] is not None:
        props["altitude"] = p[2]
    if p[4] is not None:
        props["speed"] = p[4]
    if p[6] is not None:
        props["horizontal_accuracy"] = round(p[6] * 5, 1)    # gps_logger.HDOP_M
    return [{"type": "Feature",
             "geometry": {"type": "Point", "coordinates": [p[1], p[0]]},
             "properties": props}]


# --- the service ----------------------------------------------------------------

def load_env(path):
    env = {}
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith("#") and "=" in line:
                k, v = line.split("=", 1)
                env[k.strip()] = v.strip().strip("'\"")
    return env


def config(env):
    """(key, nodes, device_id, url, api_key, decoded) from the env file's values."""
    if env.get("LORA_KEY"):
        key = base64.b64decode(env["LORA_KEY"])
    elif env.get("OTA_KEY_FILE"):
        with open(env["OTA_KEY_FILE"], "rb") as f:
            key = derived_key(f.read().strip())
    else:
        raise SystemExit("LORA_KEY or OTA_KEY_FILE needed")
    if len(key) not in (16, 32):
        raise SystemExit("LORA_KEY: 16 or 32 bytes")
    nodes = set(int(n.strip().lstrip("!"), 16)
                for n in env.get("NODES", "").split(",") if n.strip()) or None
    return (key, nodes, env.get("DEVICE_ID") or None,
            env["GEO_URL"].rstrip("/"), env["API_KEY"], env.get("ALLOW_DECODED") == "1")


def post(url, api_key, feats):
    """One Overland batch to Dawarich; the HTTP code."""
    req = urllib.request.Request(
        url + "/api/v1/overland/batches", json.dumps({"locations": feats}).encode(),
        {"Content-Type": "application/json", "Authorization": "Bearer " + api_key})
    with urllib.request.urlopen(req, timeout=30) as r:
        return r.status


def run(path):
    import paho.mqtt.client as mqtt
    env = load_env(path)
    key, nodes, dev, url, api_key, decoded = config(env)
    seen = {}

    def on_connect(c, ud, flags, rc, *a):
        print("mqtt connected", rc, flush=True)
        c.subscribe(env.get("MQTT_TOPIC", "msh/#"))

    def on_message(c, ud, msg):
        try:
            feats = handle(msg.topic, msg.payload, key, seen, nodes, dev, decoded)
        except Exception as e:           # (a bad message: logged, not fatal)
            print("bad message", msg.topic, repr(e), flush=True)
            return
        if not feats:
            return
        for _ in range(3):
            try:
                code = post(url, api_key, feats)
                print("posted", feats[0]["properties"]["timestamp"],
                      feats[0]["geometry"]["coordinates"], code, flush=True)
                return
            except Exception as e:           # (Dawarich down: a few tries)
                print("post failed", repr(e), flush=True)
                time.sleep(5)

    try:
        c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    except AttributeError:                   # paho-mqtt 1.x
        c = mqtt.Client()
    if env.get("MQTT_USER"):
        c.username_pw_set(env["MQTT_USER"], env.get("MQTT_PASS"))
    c.on_connect, c.on_message = on_connect, on_message
    c.connect(env.get("MQTT_HOST", "localhost"), int(env.get("MQTT_PORT", "1883")), 60)
    c.loop_forever()


def main(a):
    if a[:1] == ["run"]:
        run(a[1] if len(a) > 1 else "/etc/gps-lora-bridge.env")
    elif a[:1] == ["key"] and len(a) == 2:
        with open(a[1], "rb") as f:
            print(base64.b64encode(derived_key(f.read().strip())).decode())
    else:
        sys.stderr.write(__doc__ or "")
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
