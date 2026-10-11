from machine import Pin, I2C, UART, WDT
import machine, sys, time, os, network, gc, math

# Collect garbage early and often. When an allocation doesn't fit, the ESP32
# port grows the Python heap with the largest free IDF block (~60 kB) instead,
# and TLS (mbedTLS lives in the IDF heap) then fails with ENOMEM.
gc.threshold(8192)

BAUD = 9600                              # NEO-6M/7M/8M default

# OLED is optional: boards without a display (or a failed I²C probe) run
# headless. draw() and the boot banners become no-ops. Runtime I²C errors
# disable the display instead of resetting the chip.
oled = None
oled_err = 0
OLED_MAX_ERR = 3
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
HDOP_M = 5                               # metres of UERE per HDOP unit

ENV_FILE = "/.env"                       # SSID, WPA2, GEO_URL, API_KEY lines
WIFI_CHECK = FLUSH                       # ms between checks while connected
WIFI_RETRY = 15000                       # ms between reconnects while down
WIFI_BACKOFF_MAX = 300000                # ms cap on wifi reconnect backoff
CONNECT_TIMEOUT = 12000                  # ms to wait for GOT_IP per network
FAIL_HOLD = 120000                       # ms to demote a network that failed

UPLOAD_BATCH = 50                        # points per POST to Dawarich
BODY_CAP = UPLOAD_BATCH * 256 + 32       # JSON buffer; sized for long device_id
UPLOAD_GAP = 5000                        # ms between successful batches
BACKOFF_START = 15000                    # ms first retry after net/5xx/4xx
BACKOFF_MAX = 1800000                    # ms cap (30 min) on upload backoff
AUTH_BACKOFF = 1800000                   # ms retry after 401 (plan/key/limit)
HTTP_TIMEOUT = 15                        # s per socket op, not a wall clock
DEVICE_ID = "Dawarich-GPS-tracker"       # Overland tracker id (overridable)
HOSTNAME = "Dawarich-GPS-tracker"        # DHCP hostname (overridable)
MAX_BAD_BYTES = 65536                    # cap on appended <file>.gpx.bad
MAX_BAD_RETRY = 3                        # recovery attempts for .bad per boot
WDT_TIMEOUT = 120000                     # ms; ESP32 WDT cannot be disarmed
PAYLOAD_ERR = (400, 413, 422)            # only these quarantine a batch

# --- NMEA parsing -----------------------------------------------------------

st = {}

def reset():
    st.clear()
    st.update(start=time.ticks_ms(), rx=None, spin=0, ok=0, err=0,
              status=None, quality=None, fixtype=None, valid=False,
              time=None, date=None, lat=None, lon=None, alt=None,
              speed=None, course=None, used=None, view={},
              hdop=None, pdop=None, ttff=None, pos_at=None, hdop_at=None)

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
    st["hdop_at"] = time.ticks_ms()
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

# --- Watchdog ---------------------------------------------------------------
# Created only from run()/_loop(), so the GPIO25 boot-bypass REPL is not
# reset. ESP32 cannot disarm it: Ctrl-C starts a Timer that keeps feeding.

_wdt = None

def feed():
    if _wdt is not None:
        _wdt.feed()

def _wdt_keepalive():
    """Keep feeding after Ctrl-C so mpremote can talk until the next reset."""
    if _wdt is None:
        return
    try:
        t = machine.Timer(0)
        period = WDT_TIMEOUT // 3
        if period < 1000:
            period = 1000
        t.init(period=period, mode=machine.Timer.PERIODIC,
               callback=lambda timer: feed())
    except Exception as e:
        print("wdt timer", repr(e))

# --- WiFi -------------------------------------------------------------------
# Up to three networks (SSID/WPA2, optional SSID2/WPA2_2, SSID3/WPA2_3).
# Scan once, try visible known APs strongest-RSSI first (recently failed
# ones last), ~CONNECT_TIMEOUT each, feeding the WDT. If the scan fails or
# none of ours are visible, try config order 1 then 2 then 3. All-fail
# stays offline (logging continues) and retries with capped exponential
# backoff. Passwords are never printed.

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

def _load_nets(env):
    nets = []
    for sk, pk in (("SSID", "WPA2"), ("SSID2", "WPA2_2"), ("SSID3", "WPA2_3")):
        s = env.get(sk)
        if s:
            p = env.get(pk)
            nets.append((s, "" if p is None else p))
    return nets

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
    wifi.update(wlan=None, ssid=None, pw=None, nets=[], failed={},
                state="off", ip=None, rssi=None, tries=0,
                backoff=WIFI_RETRY, last_check=time.ticks_ms())

def _set_hostname(name):
    if not name:
        return
    try:
        network.hostname(name)
        return
    except (AttributeError, OSError, TypeError, ValueError):
        pass
    w = wifi.get("wlan")
    if w is None:
        return
    try:
        w.config(dhcp_hostname=name)
    except (AttributeError, OSError, TypeError, ValueError):
        pass

def _recently_failed(ssid, now):
    t = wifi["failed"].get(ssid)
    return t is not None and time.ticks_diff(now, t) < FAIL_HOLD

def _mark_failed(ssid):
    wifi["failed"][ssid] = time.ticks_ms()
    print("wifi fail", ssid)

def _scan_known(w):
    """Visible known nets as (rssi, ssid, pw), strongest first, or None."""
    known = {}
    for s, p in wifi["nets"]:
        known[s] = p
    try:
        aps = w.scan()
    except OSError:
        return None
    found = []
    for rec in aps:
        raw = rec[0]
        ssid = raw.decode() if isinstance(raw, bytes) else raw
        if ssid in known:
            found.append((rec[3], ssid, known[ssid]))
    found.sort(key=lambda x: -x[0])
    return found

def _candidate_order(w):
    """Prefer strongest visible known net; skip recently-failed first."""
    now = time.ticks_ms()
    scanned = _scan_known(w)
    if scanned:
        good, bad = [], []
        for rssi, s, p in scanned:
            (bad if _recently_failed(s, now) else good).append((s, p))
        return good + bad
    nets = wifi["nets"]
    good = [n for n in nets if not _recently_failed(n[0], now)]
    bad = [n for n in nets if _recently_failed(n[0], now)]
    return good + bad

def _wait_got_ip(w, timeout_ms):
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < timeout_ms:
        feed()
        read_uart()
        try:
            s = w.status()
        except OSError:
            return False
        if s == network.STAT_GOT_IP:
            return True
        if s not in (network.STAT_IDLE, network.STAT_CONNECTING):
            if hasattr(network, "STAT_GOT_IP") and s == network.STAT_GOT_IP:
                return True
            # auth fail / no AP / assoc fail: give up on this network
            if s != network.STAT_CONNECTING:
                return False
        time.sleep_ms(200)
    try:
        return w.status() == network.STAT_GOT_IP
    except OSError:
        return False

def _try_connect(w, ssid, pw):
    wifi["ssid"], wifi["pw"] = ssid, pw
    wifi["state"] = "connecting"
    print("wifi try", ssid)
    try:
        w.disconnect()
    except OSError:
        pass
    try:
        w.connect(ssid, pw)
    except OSError:
        _mark_failed(ssid)
        wifi["state"] = "error"
        return False
    wifi["tries"] += 1
    if _wait_got_ip(w, CONNECT_TIMEOUT):
        wifi["state"] = "connected"
        try:
            wifi["ip"] = w.ifconfig()[0]
        except OSError:
            wifi["ip"] = None
        try:
            wifi["rssi"] = w.status("rssi")
        except (OSError, ValueError):
            wifi["rssi"] = None
        wifi["failed"].pop(ssid, None)
        wifi["backoff"] = WIFI_RETRY
        print("wifi ok", ssid)
        return True
    _mark_failed(ssid)
    try:
        w.disconnect()
    except OSError:
        pass
    wifi["state"] = "lost"
    return False

def _connect_best(w):
    """Scan, then try known nets until one gets an IP. Feeds the WDT."""
    order = _candidate_order(w)
    if not order:
        wifi["state"] = "not found"
        return False
    for ssid, pw in order:
        feed()
        if _try_connect(w, ssid, pw):
            return True
    wifi["state"] = "not found"
    return False

def wifi_start():
    global DEVICE_ID, HOSTNAME
    env = load_env(ENV_FILE)
    wifi["nets"] = _load_nets(env)
    wifi["ssid"] = wifi["nets"][0][0] if wifi["nets"] else None
    wifi["pw"] = wifi["nets"][0][1] if wifi["nets"] else None
    up["url"] = env.get("GEO_URL", "").rstrip("/") or None
    up["key"] = env.get("API_KEY") or None
    if env.get("DEVICE_ID"):
        DEVICE_ID = env.get("DEVICE_ID")
    if env.get("HOSTNAME"):
        HOSTNAME = env.get("HOSTNAME")
    elif env.get("DEVICE_ID"):
        HOSTNAME = DEVICE_ID
    if up["url"] and up["url"].startswith("http://"):
        print("WARNING: GEO_URL is http:// - API key and GPS data are unencrypted")
    if not wifi["nets"]:
        wifi["state"] = "no .env"
        return
    _oled_banner("WiFi scanning...")
    try:
        w = wifi["wlan"] = network.WLAN(network.STA_IF)
        _set_hostname(HOSTNAME)
        w.active(True)
    except OSError:
        wifi["state"] = "error"
        return
    _connect_best(w)
    wifi["last_check"] = time.ticks_ms()

def wifi_tick():
    w = wifi["wlan"]
    if w is None:
        return
    now = time.ticks_ms()
    try:
        s = w.status()
    except OSError:
        s = None
    if s == network.STAT_GOT_IP:
        if wifi["state"] != "connected" or time.ticks_diff(now, wifi["last_check"]) >= WIFI_CHECK:
            wifi["state"] = "connected"
            try:
                wifi["ip"] = w.ifconfig()[0]
            except OSError:
                wifi["ip"] = None
            try:
                wifi["rssi"] = w.status("rssi")
            except (OSError, ValueError):
                wifi["rssi"] = None
            wifi["last_check"] = now
        return
    if wifi["state"] == "connected":
        wifi["state"] = WIFI_STATES.get(s, "lost")
        wifi["ip"] = wifi["rssi"] = None
        print("wifi lost", wifi.get("ssid") or "")
        wifi["last_check"] = now
        # fail over immediately to the next known net
        _connect_best(w)
        wifi["last_check"] = time.ticks_ms()
        if wifi["state"] != "connected":
            wifi["backoff"] = min(max(wifi["backoff"] * 2, WIFI_RETRY), WIFI_BACKOFF_MAX)
        return
    if time.ticks_diff(now, wifi["last_check"]) < wifi["backoff"]:
        return
    wifi["last_check"] = now
    if not _connect_best(w):
        wifi["backoff"] = min(max(wifi["backoff"] * 2, WIFI_RETRY), WIFI_BACKOFF_MAX)

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

def _fresh_hdop(now):
    """HDOP from a GGA in this epoch (not the previous fix)."""
    hdop, at = st.get("hdop"), st.get("hdop_at")
    if hdop is None or at is None:
        return None
    if time.ticks_diff(now, at) >= STALE:
        return None
    return hdop

def log_tick():
    """Sample a position every SAMPLE ms and record it if it is more than
    MIN_MOVE m from the last recorded point; write a batch every FLUSH ms."""
    if log["online"]:
        return
    if log["full"]:
        room = free()
        if room is None or room < MIN_FREE:
            return
        log["full"] = False
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
        hdop = _fresh_hdop(now)
        if hdop is not None and hdop > MAX_HDOP:
            pass                         # too dilute; try again next SAMPLE
        else:
            t = st["date"] + "T" + st["time"] + "Z"
            prev = log["prev"]
            if t != log["last_t"] and (
                    prev is None
                    or _dist(prev[0], prev[1], st["lat"], st["lon"]) > MIN_MOVE):
                log["last_t"] = t
                log["prev"] = (st["lat"], st["lon"])
                ele = ""
                if st["alt"] is not None and st.get("fixtype") != 2:
                    ele = "<ele>%.1f</ele>" % st["alt"]
                hd = ("<hdop>%.1f</hdop>" % hdop) if hdop is not None else ""
                log["batch"].append(
                    '<trkpt lat="%.6f" lon="%.6f">%s%s<time>%s</time></trkpt>\n'
                    % (st["lat"], st["lon"], ele, hd, t))
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
        # up["left"] is kept incrementally; a full recount here plus a POST
        # in the same loop iteration used to overrun the WDT.
        up["next_at"] = time.ticks_ms()
    else:                                # next fix starts a new file
        log.update(fix=False, lost_at=None, full=False)

# --- Upload ---------------------------------------------------------------
# Finished GPX files (never the one being recorded) are sent to Dawarich's
# Overland endpoint, oldest first, UPLOAD_BATCH points per POST. Progress is
# a <file>.gpx.pos sidecar (byte offset). LittleFS lfs_rename replaces the
# destination atomically; FAT vfs_fat does remove+rename (not atomic: a crash
# leaves no .pos, so the next boot resends from 0; Dawarich deduplicates via
# index_points_on_user_id_timestamp_lonlat / archival_safe_upsert_all).
#
# A power cut between the POST and the offset commit only resends points.
# Payload HTTP 400/413/422 appends that batch's <trkpt> lines to <file>.gpx.bad
# and advances the offset so the rest of the file still uploads. Other 4xx/5xx
# use exponential backoff (Retry-After honored) and never rename the GPX.
#
# TLS (mbedTLS) needs ~40 kB contiguous from the ESP-IDF heap, outside the
# Python heap. If the Python heap ever grows (see gc.threshold at the top) it
# takes that block, so allocations here stay small: the request body goes
# into a buffer allocated once at import. For the same reason the module must
# be deployed precompiled (gps_logger.mpy): compiling the source on the
# device grows the heap for good.

_body = bytearray(BODY_CAP)

up = {}

def _rm(path):
    try:
        os.remove(path)
    except OSError:
        pass

def _file_size(path):
    try:
        return os.stat(path)[6]
    except OSError:
        return None

def up_reset():
    up.clear()
    up.update(url=None, key=None, state="idle", sent=0, err=0, files=0,
              left=0, fail4=0, backoff=BACKOFF_START, allow_bad=True,
              bad_tries=0, next_at=time.ticks_ms())
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
    """Byte offset to resume from. Corrupt / past-EOF sidecars reset to 0
    (Dawarich dedup handles the resend). Never treat a bad .pos as EOF."""
    size = _file_size(path)
    if size is None:
        return 0
    try:
        with open(path + ".pos") as f:
            off = int(f.read().strip() or 0)
    except (OSError, ValueError):
        return 0
    if off < 0 or off > size:
        print("pos corrupt", path, off, size)
        return 0
    if off == 0:
        return 0
    try:
        with open(path, "rb") as f:
            f.seek(off - 1)
            b = f.read(1)
        if b != b"\n":
            print("pos misaligned", path, off)
            return 0
    except OSError:
        return 0
    return off

def write_offset(path, off):
    """Sidecar update: write <file>.gpx.pos.tmp, then rename over .pos.
    Atomic on LittleFS; on FAT a crash may drop the .pos (resend from 0)."""
    tmp = path + ".pos.tmp"
    with open(tmp, "w") as f:
        f.write("%d\n" % off)
    os.rename(tmp, path + ".pos")

def count_points():
    """Points still to upload in all GPX files, the one being recorded
    included. Counts <trkpt> lines from each file's sidecar offset. Runs at
    boot; flush() and commit_offset() keep up["left"] current in between.
    Feeds the WDT every 64 lines so a full flash cannot starve it."""
    try:
        names = [n for n in os.listdir(LOG_DIR) if n.endswith(".gpx")]
    except OSError:
        return 0
    total = 0
    nlines = 0
    for n in names:
        path = LOG_DIR + "/" + n
        try:
            off = read_offset(path)
            with open(path, "rb") as f:
                if off:
                    f.seek(off)
                for line in f:
                    nlines += 1
                    if nlines & 63 == 0:
                        feed()
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

def pending_bad():
    """Rejected-batch files left for later recovery."""
    try:
        names = sorted(n for n in os.listdir(LOG_DIR) if n.endswith(".gpx.bad"))
    except OSError:
        return []
    return [LOG_DIR + "/" + n for n in names]

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

def _finite(x):
    try:
        return math.isfinite(x)
    except AttributeError:
        return x == x and x != float("inf") and x != float("-inf")

def _valid_time(t):
    """ISO UTC `YYYY-MM-DDTHH:MM:SSZ` with year 2020-2099."""
    if not t or len(t) != 20:
        return False
    if (t[4] != "-" or t[7] != "-" or t[10] != "T"
            or t[13] != ":" or t[16] != ":" or t[19] != "Z"):
        return False
    for i in (0, 1, 2, 3, 5, 6, 8, 9, 11, 12, 14, 15, 17, 18):
        if t[i] < "0" or t[i] > "9":
            return False
    year = int(t[0:4])
    return 2020 <= year <= 2099

def _put(pos, b):
    _body[pos:pos + len(b)] = b
    return pos + len(b)

def read_batch(path, off=0):
    """Fill _body with up to UPLOAD_BATCH valid <trkpt> lines from byte offset
    off as Overland JSON -> (lines consumed, valid points, body length, new offset).
    Truncated / non-numeric / inf points are skipped (consumed, not encoded).
    A valid point that does not fit the remaining buffer is left unread
    (seek back) so it is never dropped silently."""
    n, n_ok, pos, sep = 0, 0, _put(0, b'{"locations":['), b""
    nlines = 0
    with open(path, "rb") as f:
        if off:
            f.seek(off)
        while True:
            line_start = f.tell()
            line = f.readline()
            if not line:
                break
            nlines += 1
            if nlines & 63 == 0:
                feed()
            if not line.startswith(b"<trkpt"):
                continue
            n += 1
            try:
                s = line.decode()
            except UnicodeError:
                s = ""
            lat, lon, t = _attr(s, "lat"), _attr(s, "lon"), _tag(s, "time")
            lat_f, lon_f = _float(lat), _float(lon)
            ok = (lat_f is not None and lon_f is not None and _finite(lat_f)
                  and _finite(lon_f) and abs(lat_f) <= 90 and abs(lon_f) <= 180
                  and _valid_time(t))
            loc = None
            if ok:
                ele = _tag(s, "ele")
                ele_f = _float(ele) if ele else None
                if ele_f is not None and not _finite(ele_f):
                    ele_f = None
                hdop = _tag(s, "hdop")
                hdop_f = _float(hdop) if hdop else None
                extra = ""
                if ele_f is not None:
                    extra += '"altitude":%.1f,' % ele_f
                if hdop_f is not None and _finite(hdop_f):
                    extra += '"horizontal_accuracy":%.1f,' % (hdop_f * HDOP_M)
                loc = ('{"type":"Feature","geometry":{"type":"Point",'
                       '"coordinates":[%.6f,%.6f]},"properties":{'
                       '"timestamp":"%s",%s"device_id":"%s"}}'
                       % (lon_f, lat_f, t, extra, DEVICE_ID)).encode()
                need = len(sep) + len(loc)
                if pos + need + 2 > len(_body):
                    if n_ok == 0:
                        print("point too large", len(loc))
                        loc = None       # cannot ever fit: skip
                    else:
                        f.seek(line_start)
                        n -= 1
                        break
            if loc:
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
    size = _file_size(path)
    if size is None:
        _rm(path + ".pos")
        return
    if n < UPLOAD_BATCH or new_off >= size - len(FOOTER) - 1:
        _rm(path)
        _rm(path + ".pos")

def _append_bad(path, off, new_off):
    """Copy consumed <trkpt> lines to <path>.bad (create/append)."""
    if path.endswith(".bad"):
        return path                      # already quarantined; just skip
    bad = path + ".bad"
    sz = _file_size(bad) or 0
    if sz >= MAX_BAD_BYTES:
        print("bad file full", bad)
        return bad
    try:
        with open(path, "rb") as src, open(bad, "ab") as dst:
            if off:
                src.seek(off)
            while src.tell() < new_off:
                line = src.readline()
                if not line:
                    break
                if line.startswith(b"<trkpt"):
                    dst.write(line)
    except OSError as e:
        print("quarantine", repr(e))
    return bad

def _quarantine_batch(path, off, n, new_off):
    """Park one rejected batch; keep uploading the rest of the file."""
    _append_bad(path, off, new_off)
    commit_offset(path, n, new_off)
    up["allow_bad"] = False              # do not immediately re-POST this batch
    up["fail4"] = 0
    up["state"] = "quarantine"
    return True

def _retry_after_ms(r):
    hdrs = getattr(r, "headers", None) or {}
    v = None
    if isinstance(hdrs, dict):
        for k in hdrs:
            if str(k).lower() == "retry-after":
                v = hdrs[k]
                break
    if v is None:
        return None
    try:
        return int(v) * 1000
    except (TypeError, ValueError):
        return None

def _schedule(now, ms=None):
    if ms is None:
        b = up.get("backoff", BACKOFF_START) or BACKOFF_START
        ms = b
        up["backoff"] = min(max(b * 2, BACKOFF_START), BACKOFF_MAX)
    up["next_at"] = time.ticks_add(now, ms)

def _http_post(size):
    """POST _body[:size]. Returns (status_code or None, retry_after_ms or None).
    Always closes the response so a non-OSError does not leak the TLS socket
    until the GC finalizer (micropython-lib requests)."""
    feed()
    r = None
    code, retry = None, None
    try:
        import requests
        r = requests.post("%s/api/v1/overland/batches" % up["url"],
                          data=memoryview(_body)[:size],
                          headers={"Content-Type": "application/json",
                                   "Authorization": "Bearer %s" % up["key"]},
                          timeout=HTTP_TIMEOUT)
        code = r.status_code
        retry = _retry_after_ms(r)
    except MemoryError:
        raise
    except Exception as e:
        print("upload", repr(e))
    finally:
        if r is not None:
            try:
                r.close()
            except Exception:
                pass
        feed()
        gc.collect()
    return code, retry

def upload_tick():
    """Send one batch when online and due."""
    now = time.ticks_ms()
    if (wifi["state"] != "connected" or up["state"] == "no .env"
            or time.ticks_diff(now, up["next_at"]) < 0):
        return
    if not up["url"] or not up["key"]:
        up["state"] = "no .env"
        return
    files = pending()
    from_bad = False
    if not files and up.get("allow_bad") and up.get("bad_tries", 0) < MAX_BAD_RETRY:
        files = pending_bad()
        from_bad = bool(files)
    up["files"] = len(pending())
    if not files:
        up["left"] = 0
        up["state"] = "idle"
        up["next_at"] = time.ticks_add(now, BACKOFF_START)
        return
    path = files[0]
    off = read_offset(path)
    try:
        n, n_ok, size, new_off = read_batch(path, off)
        feed()
        fsize = _file_size(path) or 0
        if not n:
            # Only delete when the offset is actually in the footer / EOF.
            # A corrupt .pos is reset to 0 by read_offset; if we still see
            # n==0 here the file has no remaining <trkpt>.
            if new_off >= fsize - len(FOOTER) - 1 or fsize == 0:
                _rm(path)
                _rm(path + ".pos")
            else:
                print("pos skip", path, off, new_off, fsize)
                write_offset(path, 0)
            return
        if not size:
            # only malformed points: skip them, do not POST garbage
            commit_offset(path, n, new_off)
            return
    except OSError:
        up["err"] += 1
        up["state"] = "fs err"
        _schedule(now)
        return
    except MemoryError:
        _low_mem(now)
        return
    up["state"] = "sending"
    read_uart()                          # drain NMEA before blocking
    gc.collect()
    try:
        code, retry = _http_post(size)
    except MemoryError:
        _low_mem(time.ticks_ms())
        return
    now = time.ticks_ms()
    print("upload", path, n, n_ok, code, gc.mem_free())
    if code in (301, 302, 303, 307, 308):
        print("upload redirect; set GEO_URL to the https origin")
        up["err"] += 1
        up["state"] = "err " + str(code)
        _schedule(now, retry)
        return
    if code is not None and 200 <= code < 300:
        try:
            commit_offset(path, n, new_off)
        except OSError:
            up["err"] += 1
            up["state"] = "fs err"
            _schedule(now)
            return
        up["sent"] += n_ok
        up["fail4"] = 0
        up["backoff"] = BACKOFF_START
        up["allow_bad"] = True
        if from_bad:
            up["bad_tries"] = up.get("bad_tries", 0) + 1
        up["state"] = "ok"
        up["next_at"] = time.ticks_add(now, UPLOAD_GAP)
    elif code in PAYLOAD_ERR:
        up["err"] += 1
        up["fail4"] += 1
        _quarantine_batch(path, off, n, new_off)
        up["next_at"] = time.ticks_add(now, UPLOAD_GAP)
    elif code == 401:
        up["err"] += 1
        up["state"] = "err 401"
        _schedule(now, retry if retry else AUTH_BACKOFF)
    elif code is not None and 400 <= code < 500:
        up["err"] += 1
        up["state"] = "err " + str(code)
        _schedule(now, retry)
    else:
        up["err"] += 1
        up["state"] = "err " + ("net" if code is None else str(code))
        _schedule(now, retry)

def _low_mem(now):
    gc.collect()
    up["err"] += 1
    up["state"] = "low mem"
    _schedule(now)
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

def _wifi_line():
    if wifi["state"] == "connected" and wifi.get("ssid"):
        ssid = wifi["ssid"]
        if len(ssid) > 11:
            ssid = ssid[:11]
        return "WiFi " + ssid
    return "WiFi " + wifi["state"]

def _oled_ok(fn):
    global oled, oled_err
    if oled is None:
        return
    try:
        fn()
        oled_err = 0
    except OSError as e:
        oled_err += 1
        print("oled", repr(e))
        if oled_err >= OLED_MAX_ERR:
            oled = None

def _oled_banner(msg):
    def _do():
        oled.fill(0)
        oled.text(msg, 0, 28)
        oled.show()
    _oled_ok(_do)

def draw():
    def _do():
        oled.fill(0)
        oled.fill_rect(0, 0, 128, 8, 1)
        oled.text("GPS Logger", 0, 0, 0)
        oled.text("|/-\\"[st["spin"] % 4], 120, 0, 0)
        room = free()
        for row, line in enumerate((
                _wifi_line(),
                "GPS " + status(),
                "Mode " + ("UPLOAD" if log["online"] else "LOG"),
                "Flash " + _fmt("%d kB", None if room is None else room // 1024),
                "Points left " + _short(up["left"]))):
            oled.text(line, 0, 10 + row * 11)
        oled.show()
    _oled_ok(_do)

# --- Main loop --------------------------------------------------------------

def run():
    """Show the logger state on the OLED, log GPX offline, upload online."""
    global _wdt
    try:
        # Arm the dog before any blocking init (count_points, WiFi scan).
        # GPIO25 boot-bypass never calls run(), so the REPL stays free.
        if _wdt is None:
            _wdt = WDT(timeout=WDT_TIMEOUT)
        feed()
        reset()
        log_reset()
        wifi_reset()
        up_reset()
        wifi_start()
        _loop()
    except KeyboardInterrupt:
        print("WDT armed - fed by timer; hold GPIO25 and reset for mpremote")
        try:
            flush()
        except Exception:
            pass
        wifi_stop()
        _wdt_keepalive()
        raise
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
        # machine.reset() does not return on hardware. KeyboardInterrupt
        # already flushed and stopped WiFi; a second flush is a no-op.
        try:
            flush()
        except Exception:
            pass
        wifi_stop()

def _loop():
    global _wdt
    if _wdt is None:
        _wdt = WDT(timeout=WDT_TIMEOUT)
    last = None
    while True:
        feed()
        read_uart()
        log_tick()
        wifi_tick()
        feed()
        mode_tick()
        feed()
        upload_tick()
        now = time.ticks_ms()
        if last is None or time.ticks_diff(now, last) >= REFRESH:
            draw()
            last = now
        time.sleep_ms(20)
