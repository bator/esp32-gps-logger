from machine import Pin, I2C, UART, WDT
import machine, sys, time, os, network, gc, math

# Collect garbage early and often. When an allocation doesn't fit, the ESP32
# port grows the Python heap with the largest free IDF block (~60 kB) instead,
# and TLS (mbedTLS lives in the IDF heap) then fails with ENOMEM.
gc.threshold(8192)

BAUD = 9600                              # NEO-6M/7M/8M default

# OLED is optional: boards without a display (or a failed I²C probe) run
# headless. draw() and the boot banners become no-ops.
oled = None
try:
    import ssd1306
    _i2c = I2C(0, scl=Pin(22), sda=Pin(21))
    oled = ssd1306.SSD1306_I2C(128, 64, _i2c)
except Exception as e:
    print("oled", repr(e))

uart = UART(2, baudrate=BAUD, tx=17, rx=16, rxbuf=4096)   # rides out HTTP calls

REFRESH = 500                            # ms between screen redraws
NO_DATA = 3000                           # ms of silence = "No data!"

SAMPLE = 1000                            # ms between sampled positions
MIN_MOVE = 10                            # m from the last recorded point
FLUSH = 60000                            # ms between batch writes to flash
NEW_FILE_GAP = 300                       # s without fix before a new file
STALE = 2000                             # ms: older position = no fix
LOG_DIR = "/gpx"
MIN_FREE = 32768                         # bytes of flash kept free
MAX_BATCH = 600                          # points kept in RAM if writes fail
MAX_HDOP = 5                             # skip samples with worse dilution

ENV_FILE = "/.env"                       # SSID, WPA2, GEO_URL, API_KEY lines
WIFI_CHECK = FLUSH                       # ms between checks while connected
WIFI_RETRY = 15000                       # ms between reconnects while down

UPLOAD_BATCH = 50                        # points per POST to Dawarich
POINT_MAX = 192                          # bytes of JSON per point, at most
UPLOAD_GAP = 5000                        # ms between successful batches
UPLOAD_RETRY = 60000                     # ms to wait after a failed batch
HTTP_TIMEOUT = 15                        # s
DEVICE_ID = "esp32"
MAX_4XX = 3                              # consecutive payload rejections -> quarantine
WDT_TIMEOUT = 60000                      # ms; must exceed HTTP_TIMEOUT and count_points

# --- NMEA parsing -----------------------------------------------------------

st = {}

def reset():
    st.clear()
    st.update(start=time.ticks_ms(), rx=None, spin=0, ok=0, err=0,
              status=None, quality=None, fixtype=None, valid=False,
              time=None, date=None, lat=None, lon=None, alt=None,
              speed=None, course=None, used=None, view={},
              hdop=None, pdop=None, ttff=None, pos_at=None)

def _num(s, conv=float):
    return conv(s) if s else None

def _deg(v, hemi):
    """NMEA ddmm.mmmm / dddmm.mmmm + N/S/E/W -> signed decimal degrees."""
    if not v or not hemi:
        return None
    i = v.find(".")
    d = (i if i >= 0 else len(v)) - 2
    deg = int(v[:d]) + float(v[d:]) / 60
    return -deg if hemi in "SW" else deg

def _position(f, i):
    lat, lon = _deg(f[i], f[i + 1]), _deg(f[i + 2], f[i + 3])
    if lat is not None and lon is not None:
        st["lat"], st["lon"] = lat, lon

def _time(s):
    if len(s) >= 6:
        st["time"] = "%s:%s:%s" % (s[0:2], s[2:4], s[4:6])

def _update_fix():
    st["valid"] = st["status"] == "A" or (st["quality"] or 0) > 0
    if st["valid"]:
        st["pos_at"] = time.ticks_ms()
        if st["ttff"] is None:
            st["ttff"] = time.ticks_diff(st["pos_at"], st["start"]) // 1000

def rmc(talker, f):
    _time(f[1])
    st["status"] = f[2]
    _position(f, 3)
    kn = _num(f[7])
    st["speed"] = None if kn is None else kn * 1.852
    st["course"] = _num(f[8])
    d = f[9]
    if len(d) == 6:
        st["date"] = "20%s-%s-%s" % (d[4:6], d[2:4], d[0:2])
    _update_fix()

def gga(talker, f):
    _time(f[1])
    _position(f, 2)
    st["quality"] = _num(f[6], int)
    st["used"] = _num(f[7], int)
    st["hdop"] = _num(f[8])
    st["alt"] = _num(f[9])
    _update_fix()

def gsa(talker, f):
    st["fixtype"] = _num(f[2], int)
    if len(f) > 15:
        st["pdop"] = _num(f[15])

def gsv(talker, f):
    n = _num(f[3], int)
    if n is not None:
        st["view"][talker] = n          # per constellation, summed on display

HANDLERS = {"RMC": rmc, "GGA": gga, "GSA": gsa, "GSV": gsv}

def checksum_ok(line):
    star = line.rfind(b"*")
    if not line.startswith(b"$") or star < 0:
        return False
    c = 0
    for b in line[1:star]:
        c ^= b
    try:
        return c == int(line[star + 1:star + 3], 16)
    except ValueError:
        return False

def handle(line):
    line = line.strip()
    if not line:
        return
    if not checksum_ok(line):
        st["err"] += 1
        return
    try:
        f = line[1:line.rfind(b"*")].decode().split(",")
        fn = HANDLERS.get(f[0][2:])
        if fn:
            fn(f[0][:2], f)
        st["ok"] += 1
    except (ValueError, IndexError, UnicodeError):
        st["err"] += 1

_buf = b""
_chunk = bytearray(256)                  # read in small pieces: one big
                                         # allocation grows the heap (see
                                         # Upload) and starves TLS

def read_uart():
    global _buf
    while uart.any():
        k = uart.readinto(_chunk)
        if not k:
            return
        st["rx"] = time.ticks_ms()
        st["spin"] += 1
        _buf += _chunk[:k]
        while True:
            i = _buf.find(b"\n")
            if i < 0:
                break
            line, _buf = _buf[:i], _buf[i + 1:]
            handle(line)
        if len(_buf) > 256:              # no newline in sight: garbage/baud
            _buf = b""

# --- WiFi -------------------------------------------------------------------
# Scan once at start and connect if our network is visible. Afterwards only
# the status is polled; a dropped link is reconnected every WIFI_RETRY ms
# without a (blocking) re-scan, so GPS reading isn't stalled. The driver
# reports a handshake timeout on a weak link as STAT_WRONG_PASSWORD and
# then gives up, hence "auth fail" and our own retries.
#
# The link picks the mode (see mode_tick): while connected, nothing is logged
# and finished GPX files are uploaded; the moment it drops, logging resumes
# into a new file. GPS is read all the time, so that switch is immediate.

def load_env(path):
    """KEY=VALUE lines -> dict. Surrounding quotes are stripped."""
    env = {}
    try:
        with open(path) as f:
            for line in f:
                line = line.strip()
                if not line or line[0] == "#" or "=" not in line:
                    continue
                k, v = line.split("=", 1)
                k, v = k.strip(), v.strip()
                if len(v) >= 2 and v[0] == v[-1] and v[0] in "\"'":
                    v = v[1:-1]
                env[k] = v
    except OSError:
        pass                             # no .env on the device
    return env

WIFI_STATES = {}
for _n, _s in (("STAT_IDLE", "idle"), ("STAT_CONNECTING", "connecting"),
               ("STAT_GOT_IP", "connected"), ("STAT_NO_AP_FOUND", "no AP"),
               ("STAT_WRONG_PASSWORD", "auth fail"),
               ("STAT_BEACON_TIMEOUT", "lost"), ("STAT_ASSOC_FAIL", "assoc fail"),
               ("STAT_HANDSHAKE_TIMEOUT", "handshake")):
    if hasattr(network, _n):
        WIFI_STATES[getattr(network, _n)] = _s

wifi = {}

def wifi_reset():
    wifi.clear()
    wifi.update(wlan=None, ssid=None, pw=None, state="off", ip=None,
                rssi=None, tries=0, last_check=time.ticks_ms())

def _connect():
    w = wifi["wlan"]
    try:
        w.disconnect()
    except OSError:
        pass
    try:
        w.connect(wifi["ssid"], wifi["pw"])
        wifi["state"] = "connecting"
    except OSError:
        wifi["state"] = "error"
    wifi["tries"] += 1

def _oled_banner(msg):
    if oled is None:
        return
    oled.fill(0)
    oled.text(msg, 0, 28)
    oled.show()

def wifi_start():
    env = load_env(ENV_FILE)
    wifi["ssid"], wifi["pw"] = env.get("SSID"), env.get("WPA2")
    up["url"] = env.get("GEO_URL", "").rstrip("/") or None
    up["key"] = env.get("API_KEY") or None
    if up["url"] and up["url"].startswith("http://"):
        print("WARNING: GEO_URL is http:// — API key and GPS data are unencrypted")
    if not wifi["ssid"] or wifi["pw"] is None:
        wifi["state"] = "no .env"
        return
    _oled_banner("WiFi scanning...")
    try:
        w = wifi["wlan"] = network.WLAN(network.STA_IF)
        w.active(True)
        found = any(n[0] == wifi["ssid"].encode() for n in w.scan())
    except OSError:
        wifi["state"] = "error"
        return
    if found:
        _connect()
    else:
        wifi["state"] = "not found"
    wifi["last_check"] = time.ticks_ms()

def wifi_tick():
    w = wifi["wlan"]
    if w is None:
        return
    s = w.status()
    now = time.ticks_ms()
    every = WIFI_CHECK if s == network.STAT_GOT_IP else WIFI_RETRY
    check = time.ticks_diff(now, wifi["last_check"]) >= every
    if s == network.STAT_GOT_IP:
        if wifi["state"] != "connected" or check:
            wifi["state"] = "connected"
            wifi["ip"] = w.ifconfig()[0]
            try:
                wifi["rssi"] = w.status("rssi")
            except (OSError, ValueError):
                wifi["rssi"] = None
    elif s == network.STAT_CONNECTING:
        wifi["state"] = "connecting"
    elif wifi["state"] not in ("not found", "error"):
        wifi["state"] = WIFI_STATES.get(s, "lost")
    if not check:
        return
    wifi["last_check"] = now
    if s not in (network.STAT_GOT_IP, network.STAT_CONNECTING):
        wifi["ip"] = wifi["rssi"] = None
        _connect()

def wifi_stop():
    w = wifi["wlan"]
    if w is None:
        return
    try:
        w.disconnect()
        w.active(False)
    except OSError:
        pass
    wifi["state"] = "off"

def online():
    return wifi["state"] == "connected"

# --- GPX logger -------------------------------------------------------------
# The file is always a complete GPX document: each batch overwrites the
# closing tags with new points plus the closing tags again. LittleFS commits
# on close, so a power cut leaves either the previous or the new version.

HEADER = ('<?xml version="1.0" encoding="UTF-8"?>\n'
          '<gpx version="1.1" creator="esp32 gps_logger" '
          'xmlns="http://www.topografix.com/GPX/1/1">\n'
          '<trk><name>%s</name><trkseg>\n')
FOOTER = "</trkseg></trk></gpx>\n"
NEW_SEG = "</trkseg><trkseg>\n"

log = {}

def log_reset():
    log.clear()
    log.update(file=None, created=False, batch=[], points=0, err=0,
               fix=False, lost_at=None, new_seg=False, full=False, online=False,
               last_sample=None, last_flush=time.ticks_ms(), last_t=None,
               prev=None)

def free():
    try:
        s = os.statvfs(LOG_DIR)
        return s[0] * s[3]
    except OSError:
        return None

def has_fix(now):
    return (st["valid"] and st["date"] and st["time"] and st["lat"] is not None
            and st["pos_at"] is not None
            and time.ticks_diff(now, st["pos_at"]) < STALE)

def start_file():
    try:
        os.mkdir(LOG_DIR)
    except OSError:
        pass                             # already exists
    name = st["date"].replace("-", "") + "_" + st["time"].replace(":", "")
    log.update(file=LOG_DIR + "/" + name + ".gpx", created=False, points=0,
               new_seg=False, prev=None)
    flush()                              # create it right away

def flush():
    """Write pending points to the current file, opened only for the write."""
    log["last_flush"] = time.ticks_ms()
    path, batch = log["file"], log["batch"]
    if path is None or (log["created"] and not batch):
        return
    room = free()
    if room is not None and room < MIN_FREE:
        log["full"] = True
        del batch[:]
        return
    pts = "".join(batch)
    try:
        if log["created"]:
            f = open(path, "r+b")
            try:
                f.seek(-len(FOOTER), 2)
                f.write(((NEW_SEG if log["new_seg"] else "") + pts
                         + FOOTER).encode())
            finally:
                f.close()
        else:
            name = path[len(LOG_DIR) + 1:-4]
            f = open(path, "wb")
            try:
                f.write((HEADER % name + pts + FOOTER).encode())
            finally:
                f.close()
            log["created"] = True
    except OSError:
        log["err"] += 1                  # keep the batch, retry next FLUSH
        return
    log["points"] += len(batch)
    up["left"] += len(batch)
    del batch[:]
    log["new_seg"] = False

def _dist(lat1, lon1, lat2, lon2):
    """Metres between two nearby positions (equirectangular approximation)."""
    x = math.radians(lon2 - lon1) * math.cos(math.radians((lat1 + lat2) / 2))
    y = math.radians(lat2 - lat1)
    return 6371000 * math.sqrt(x * x + y * y)

def log_tick():
    """Sample a position every SAMPLE ms and record it if it is more than
    MIN_MOVE m from the last recorded point; write a batch every FLUSH ms."""
    if log["full"] or log["online"]:
        return
    now = time.ticks_ms()
    if not has_fix(now):
        if log["fix"]:
            log["fix"], log["lost_at"] = False, now
            flush()
        return
    if not log["fix"]:
        log["fix"] = True
        if (log["file"] is None or time.ticks_diff(now, log["lost_at"])
                > NEW_FILE_GAP * 1000):
            flush()                      # leftovers of a failed write
            del log["batch"][:]
            start_file()
        else:
            log["new_seg"] = True
            log["prev"] = None           # start the segment with this fix
    if (log["last_sample"] is None
            or time.ticks_diff(now, log["last_sample"]) >= SAMPLE):
        log["last_sample"] = now
        hdop = st["hdop"]
        if hdop is None or hdop <= MAX_HDOP:
            t = st["date"] + "T" + st["time"] + "Z"
            prev = log["prev"]
            if t != log["last_t"] and (
                    prev is None
                    or _dist(prev[0], prev[1], st["lat"], st["lon"]) > MIN_MOVE):
                log["last_t"] = t
                log["prev"] = (st["lat"], st["lon"])
                ele = ("<ele>%.1f</ele>" % st["alt"]
                       if st["alt"] is not None else "")
                log["batch"].append(
                    '<trkpt lat="%.6f" lon="%.6f">%s<time>%s</time></trkpt>\n'
                    % (st["lat"], st["lon"], ele, t))
                if len(log["batch"]) > MAX_BATCH:
                    log["batch"].pop(0)
    if time.ticks_diff(now, log["last_flush"]) >= FLUSH:
        flush()

def mode_tick():
    """Switch between logging (offline) and uploading (online)."""
    on = online()
    if on == log["online"]:
        return
    log["online"] = on
    if on:                               # close the file so it gets uploaded
        flush()
        del log["batch"][:]
        log.update(file=None, fix=False, new_seg=False, last_t=None,
                   prev=None)
        up["left"] = count_points()
        up["next_at"] = time.ticks_ms()
    else:                                # next fix starts a new file
        log.update(fix=False, lost_at=None, full=False)

# --- Upload ---------------------------------------------------------------
# Finished GPX files (never the one being recorded) are sent to Dawarich's
# Overland endpoint, oldest first, UPLOAD_BATCH points per POST. Progress is
# a <file>.gpx.pos sidecar (byte offset, written atomically via .tmp+rename).
# A power cut between the POST and the offset commit only resends points,
# which Dawarich deduplicates. The file and sidecar are removed when the
# offset passes the last <trkpt>.
#
# TLS (mbedTLS) needs ~40 kB contiguous from the ESP-IDF heap, outside the
# Python heap. If the Python heap ever grows (see gc.threshold at the top) it
# takes that block, so allocations here stay small: the request body goes
# into a buffer allocated once at import. For the same reason the module must
# be deployed precompiled (gps_logger.mpy): compiling the source on the
# device grows the heap for good.

_body = bytearray(UPLOAD_BATCH * POINT_MAX + 32)

up = {}

def _rm(path):
    try:
        os.remove(path)
    except OSError:
        pass

def up_reset():
    up.clear()
    up.update(url=None, key=None, state="idle", sent=0, err=0, files=0,
              left=0, fail4=0, next_at=time.ticks_ms())
    try:
        names = os.listdir(LOG_DIR)
    except OSError:
        names = []
    gpx = set(n for n in names if n.endswith(".gpx"))
    for n in names:
        if n.endswith(".tmp"):
            _rm(LOG_DIR + "/" + n)       # offset write or old rewrite cut off
        elif n.endswith(".gpx.pos") and n[:-4] not in gpx:
            _rm(LOG_DIR + "/" + n)       # leftover sidecar, GPX already gone
    _oled_banner("Counting points")
    up["left"] = count_points()

def read_offset(path):
    try:
        with open(path + ".pos") as f:
            return int(f.read().strip() or 0)
    except (OSError, ValueError):
        return 0

def write_offset(path, off):
    """Atomic sidecar update: write <file>.gpx.pos.tmp, then rename over .pos."""
    tmp = path + ".pos.tmp"
    with open(tmp, "w") as f:
        f.write("%d\n" % off)
    os.rename(tmp, path + ".pos")

def count_points():
    """Points still to upload in all GPX files, the one being recorded
    included. Counts <trkpt> lines from each file's sidecar offset. Runs at
    boot and when going online; flush() and commit_offset() keep up["left"]
    current in between."""
    try:
        names = [n for n in os.listdir(LOG_DIR) if n.endswith(".gpx")]
    except OSError:
        return 0
    total = 0
    for n in names:
        path = LOG_DIR + "/" + n
        try:
            off = read_offset(path)
            with open(path, "rb") as f:
                f.seek(off)
                for line in f:
                    if line.startswith(b"<trkpt"):
                        total += 1
        except OSError:
            pass                         # unreadable: upload reports fs err
    return total

def pending():
    """Finished GPX files, oldest first (names are YYYYMMDD_HHMMSS).
    Sidecars (.pos) and quarantined (.bad) files are not listed."""
    try:
        names = sorted(n for n in os.listdir(LOG_DIR) if n.endswith(".gpx"))
    except OSError:
        return []
    return [p for p in (LOG_DIR + "/" + n for n in names) if p != log["file"]]

def _attr(line, name):
    i = line.find(name + '="')
    if i < 0:
        return None
    i += len(name) + 2
    j = line.find('"', i)
    if j < 0:
        return None
    return line[i:j]

def _tag(line, name):
    i = line.find("<" + name + ">")
    if i < 0:
        return None
    i += len(name) + 2
    j = line.find("</" + name + ">", i)
    if j < 0:
        return None
    return line[i:j]

def _float(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return None

def _put(pos, b):
    _body[pos:pos + len(b)] = b
    return pos + len(b)

def read_batch(path, off=0):
    """Fill _body with up to UPLOAD_BATCH <trkpt> lines from byte offset off
    as Overland JSON -> (lines consumed, valid points, body length, new offset).
    Truncated or non-numeric points are skipped (consumed, not encoded).
    Body length is 0 if every consumed point was malformed."""
    n, n_ok, pos, sep = 0, 0, _put(0, b'{"locations":['), b""
    with open(path, "rb") as f:
        f.seek(off)
        for line in f:
            if not line.startswith(b"<trkpt"):
                continue
            n += 1
            try:
                s = line.decode()
            except UnicodeError:
                s = ""
            lat, lon, t = _attr(s, "lat"), _attr(s, "lon"), _tag(s, "time")
            lat_f, lon_f = _float(lat), _float(lon)
            if (lat_f is not None and lon_f is not None and t
                    and abs(lat_f) <= 90 and abs(lon_f) <= 180):
                ele = _tag(s, "ele")
                ele_f = _float(ele) if ele else None
                loc = ('{"type":"Feature","geometry":{"type":"Point",'
                       '"coordinates":[%s,%s]},"properties":{"timestamp":"%s",'
                       '%s"device_id":"%s"}}'
                       % (lon, lat, t,
                          '"altitude":%s,' % ele if ele_f is not None else "",
                          DEVICE_ID)).encode()
                if len(loc) < POINT_MAX:     # longer = garbage, skipped
                    pos = _put(_put(pos, sep), loc)
                    sep = b","
                    n_ok += 1
            if n >= UPLOAD_BATCH:
                break
        new_off = f.tell()
    if not sep:
        return n, 0, 0, new_off
    return n, n_ok, _put(pos, b"]}"), new_off

def commit_offset(path, n, new_off):
    """Record that n <trkpt> lines are done. Delete the file once none remain
    after new_off. Crash between POST and this call resends (at-least-once)."""
    write_offset(path, new_off)
    up["left"] = max(up["left"] - n, 0)
    if n < UPLOAD_BATCH:
        _rm(path)
        _rm(path + ".pos")

def _quarantine(path):
    """Park a file that Dawarich keeps rejecting so upload can move on."""
    left = 0
    try:
        off = read_offset(path)
        with open(path, "rb") as f:
            f.seek(off)
            for line in f:
                if line.startswith(b"<trkpt"):
                    left += 1
    except OSError:
        pass
    try:
        os.rename(path, path + ".bad")
    except OSError as e:
        print("quarantine", repr(e))
        return False
    _rm(path + ".pos")
    up["left"] = max(up["left"] - left, 0)
    up["fail4"] = 0
    up["state"] = "quarantine"
    return True

def upload_tick():
    """Send one batch when online and due."""
    now = time.ticks_ms()
    if (wifi["state"] != "connected" or up["state"] in ("auth fail", "no .env")
            or time.ticks_diff(now, up["next_at"]) < 0):
        return
    if not up["url"] or not up["key"]:
        up["state"] = "no .env"
        return
    files = pending()
    up["files"] = len(files)
    if not files:
        up["left"] = 0
        up["state"] = "idle"
        up["next_at"] = time.ticks_add(now, UPLOAD_RETRY)
        return
    path = files[0]
    try:
        n, n_ok, size, new_off = read_batch(path, read_offset(path))
        if not n:                        # header/footer only past the offset
            _rm(path)
            _rm(path + ".pos")
            return
        if not size:                     # only malformed points: skip them
            commit_offset(path, n, new_off)
            return
    except OSError:
        up["err"] += 1
        up["state"] = "fs err"
        up["next_at"] = time.ticks_add(now, UPLOAD_RETRY)
        return
    except MemoryError:
        _low_mem(now)
        return
    up["state"] = "sending"
    read_uart()                          # drain NMEA before blocking
    gc.collect()
    code = None
    try:
        import requests
        r = requests.post("%s/api/v1/overland/batches" % up["url"],
                          data=memoryview(_body)[:size],
                          headers={"Content-Type": "application/json",
                                   "Authorization": "Bearer %s" % up["key"]},
                          timeout=HTTP_TIMEOUT)
        code = r.status_code
        r.close()
    except MemoryError:
        r = None
        _low_mem(time.ticks_ms())
        return
    except Exception as e:
        print("upload", repr(e))
    r = None
    gc.collect()
    now = time.ticks_ms()
    print("upload", path, n, n_ok, code, gc.mem_free())
    if code is not None and 200 <= code < 300:
        try:
            commit_offset(path, n, new_off)
        except OSError:
            up["err"] += 1
            up["state"] = "fs err"
            up["next_at"] = time.ticks_add(now, UPLOAD_RETRY)
            return
        up["sent"] += n_ok
        up["fail4"] = 0
        up["state"] = "ok"
        up["next_at"] = time.ticks_add(now, UPLOAD_GAP)
    elif code == 401:
        up["state"] = "auth fail"        # wrong API_KEY: stop for this run
    elif code is not None and 400 <= code < 500:
        up["err"] += 1
        up["fail4"] += 1
        up["state"] = "err " + str(code)
        if up["fail4"] >= MAX_4XX:
            _quarantine(path)
        up["next_at"] = time.ticks_add(now, UPLOAD_RETRY)
    else:
        up["err"] += 1
        up["state"] = "err " + ("net" if code is None else str(code))
        up["next_at"] = time.ticks_add(now, UPLOAD_RETRY)

def _low_mem(now):
    gc.collect()
    up["err"] += 1
    up["state"] = "low mem"
    up["next_at"] = time.ticks_add(now, UPLOAD_RETRY)
    print("upload MemoryError, free", gc.mem_free())

# --- Screen -----------------------------------------------------------------

def status():
    if st["rx"] is None or time.ticks_diff(time.ticks_ms(), st["rx"]) > NO_DATA:
        return "No data!"
    if not st["valid"]:
        return "Waiting fix"
    s = {2: "2D fix", 3: "3D fix"}.get(st["fixtype"], "Fix")
    return s + " DGPS" if st["quality"] == 2 else s

def _fmt(fmt, v):
    return fmt % v if v is not None else "--"

def _short(n):
    """Fit a count into the 4 chars left after "Points left "."""
    return str(n) if n < 10000 else "%dk" % (n // 1000)

def draw():
    if oled is None:
        return
    oled.fill(0)
    oled.fill_rect(0, 0, 128, 8, 1)
    oled.text("GPS Logger", 0, 0, 0)
    oled.text("|/-\\"[st["spin"] % 4], 120, 0, 0)
    room = free()
    for row, line in enumerate((
            "WiFi " + wifi["state"],
            "GPS " + status(),
            "Mode " + ("UPLOAD" if log["online"] else "LOG"),
            "Flash " + _fmt("%d kB", None if room is None else room // 1024),
            "Points left " + _short(up["left"]))):
        oled.text(line, 0, 10 + row * 11)
    oled.show()

# --- Main loop --------------------------------------------------------------

def run():
    """Show the logger state on the OLED, log GPX offline, upload online."""
    reset()
    log_reset()
    wifi_reset()
    up_reset()
    wifi_start()
    try:
        _loop()
    except Exception as e:
        if hasattr(sys, "print_exception"):
            sys.print_exception(e)
        else:
            print("crash", e)
        try:
            flush()
        except Exception:
            pass
        time.sleep(1)
        machine.reset()
    finally:
        try:
            flush()
        except Exception:
            pass
        wifi_stop()

def _loop():
    # WDT starts only once the logger is actually running. Holding GPIO25 at
    # boot skips importing this module, so the REPL is not reset by the dog.
    wdt = WDT(timeout=WDT_TIMEOUT)
    last = None
    while True:
        wdt.feed()
        read_uart()
        log_tick()
        wifi_tick()
        mode_tick()
        upload_tick()
        now = time.ticks_ms()
        if last is None or time.ticks_diff(now, last) >= REFRESH:
            draw()
            last = now
        time.sleep_ms(20)
