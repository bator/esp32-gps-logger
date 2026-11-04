# The Heltec Wireless Tracker's TFT: 0.96", 160x80, ST7735S on SPI, powered by
# Vext with the UC6580 (pins and the 1/26 offset measured on the board). The
# logger hands over what to show (set_state) and calls frame() every TFT_FRAME.
#
# Top: the speed, big and orange, the satellites in the fix, the WiFi icon and the
# clock. Bottom: how good the GPS is, four rising bars and a word, as on a phone,
# how full the flash is and how long it still logs, and what waits for upload.
#
# One band of 160x16 is all the RAM it keeps, not a 25.6 KB frame buffer, and the
# glyphs are read from flash, 16-level (anti-aliased) GS4: a zone goes up band by
# band, only when it changed. The WiFi icon has its own 12x12. Colours are RGB565
# byte-swapped: the panel's order.
import framebuf
import struct
import time
from machine import Pin, SPI

import gps_tft_font as F

W, H = 160, 80
XO, YO = 1, 26                           # the panel inside the controller
BAND_H = 16
TOP_H, LOW_Y = 41, 46                    # the zones: rows 0-40, rows 46-79
BL_PIN = 21
WIFI_X, WIFI_Y = 113, 28
STORE_X, STORE_W = 102, 56               # the flash's gauge, right-aligned at 158
STORE_WARN, STORE_BAD = 70, 90           # per cent: yellow, red

def _c(t):
    c = (t[0] & 0xF8) << 8 | (t[1] & 0xFC) << 3 | t[2] >> 3
    return c >> 8 | (c & 255) << 8

_BG, _DIM_T = (5, 6, 8), (51, 59, 70)
_TONES = {
    "org": (255, 106, 43), "txt": (245, 247, 249), "soft": (198, 206, 216),
    "mut": (141, 153, 167), "red": (255, 90, 90), "grn": (61, 220, 132),
    "yel": (255, 194, 61),
}
BG, DIM, ORG, OFF = _c(_BG), _c(_DIM_T), _c(_TONES["org"]), _c((42, 47, 55))
# by quality: 0 no fix, 1 weak, 2 fair, 3 strong: its word and tone
QUALITY = (("nincs", "red"), ("gyenge", "org"), ("közepes", "yel"), ("erős", "grn"))
UP_ICON = b"\x08\x00\x1c\x00\x3e\x00\x08\x00\x08\x00\x08\x00\x00\x00\x7f\x00"
# The WiFi icon's four parts, 12x12 MONO_HLSB: the dot and the arcs inner to outer.
# Level 0-3: that many arcs lit, None: all dim. It changes only with the level.
WIFI = (b"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x06\x00\x06\x00\x00\x00",
        b"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x0f\x00\x10\x80\x00\x00\x00\x00\x00\x00\x00\x00",
        b"\x00\x00\x00\x00\x00\x00\x0f\x00\x30\xc0\x40\x20\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00",
        b"\x1f\x80\x60\x60\x80\x10\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00")

_FONTS = [(f, memoryview(f[3])) for f in (F.HERO, F.NUM, F.TXT, F.LAB, F.WORD)]
_HERO, _NUM, _TXT, _LAB, _WORD = _FONTS
_pal = {}

class _D:
    spi = cs = dc = None
    buf = fb = ic = icb = None
    state = None
    top = low = lit = None               # what the panel shows

def _cmd(c, data=None):
    _D.cs(0)
    _D.dc(0)
    _D.spi.write(bytes((c,)))
    if data:
        _D.dc(1)
        _D.spi.write(data)
    _D.cs(1)

def _window(x, y, w, h):
    _cmd(0x2A, struct.pack(">HH", XO + x, XO + x + w - 1))
    _cmd(0x2B, struct.pack(">HH", YO + y, YO + y + h - 1))

def _send(y, h):
    """The band's top h rows to the panel's row y."""
    _window(0, y, W, h)
    _cmd(0x2C, memoryview(_D.buf)[:W * h * 2])

def start():
    """The panel on and blank, then the backlight (not before: no flash of its
    random RAM)."""
    Pin(BL_PIN, Pin.OUT, value=0)
    _D.cs = Pin(38, Pin.OUT, value=1)
    _D.dc = Pin(40, Pin.OUT, value=0)
    rst = Pin(39, Pin.OUT, value=1)
    _D.spi = SPI(1, baudrate=26000000, polarity=0, phase=0, sck=Pin(41),
                 mosi=Pin(42))
    rst(0)
    time.sleep_ms(20)
    rst(1)
    time.sleep_ms(120)
    _cmd(0x11)                           # sleep out
    time.sleep_ms(120)
    _cmd(0x3A, b"\x05")                  # 16 bits a pixel
    _cmd(0x36, b"\x68")                  # landscape
    _cmd(0x21)                           # IPS: colours inverted
    _cmd(0x13)
    _cmd(0x29)
    if _D.buf is None:
        _D.buf = bytearray(W * BAND_H * 2)
        _D.fb = framebuf.FrameBuffer(_D.buf, W, BAND_H, framebuf.RGB565)
        _D.icb = bytearray(12 * 12 * 2)
        _D.ic = framebuf.FrameBuffer(_D.icb, 12, 12, framebuf.RGB565)
    _D.top = _D.low = _D.lit = None
    _D.fb.fill(BG)
    for y in range(0, H, BAND_H):
        _send(y, BAND_H)
    Pin(BL_PIN, Pin.OUT, value=1)

def off():
    """Dark: a frozen speed must not stay lit. A dead SPI bus does not stop this."""
    Pin(BL_PIN, Pin.OUT, value=0)

def _mono_palette(col):
    p = _pal.get(col)
    if p is None:
        p = framebuf.FrameBuffer(bytearray(4), 2, 1, framebuf.RGB565)
        p.pixel(0, 0, 0)                 # (blit's key: transparent)
        p.pixel(1, 0, col)
        _pal[col] = p
    return p

def _gs4_palette(name):
    """16 levels from the background to a tone: the anti-aliasing."""
    p = _pal.get(name)
    if p is None:
        t = _TONES[name]
        p = framebuf.FrameBuffer(bytearray(32), 16, 1, framebuf.RGB565)
        for v in range(16):
            p.pixel(v, 0, _c(tuple((_BG[k] * (15 - v) + t[k] * v) // 15 for k in range(3))))
        _pal[name] = p
    return p

def _index(f, ch):
    return max(f[0].find(ch), 0)

def width(font, s):
    f = font[0]
    n = 0
    for ch in s:
        n += f[1][_index(f, ch)]
    return n

def _text(fb, font, s, x, base, tone):
    """s with its baseline at row base, each glyph blitted straight from flash.
    Nothing to blit outside the band (each glyph's slice and tuple, in each band,
    went to the garbage): only how wide it is, the x that follows."""
    f, mv = font
    y = base - f[5]
    h = f[4]
    if y >= BAND_H or y + h <= 0:
        return x + width(font, s)
    p = _gs4_palette(tone)
    for ch in s:
        i = _index(f, ch)
        w = f[1][i]
        o = f[2][i * 2] | f[2][i * 2 + 1] << 8
        fb.blit((mv[o:o + (w + 1) // 2 * h], w, h, framebuf.GS4_HMSB), x, y, BG, p)
        x += w
    return x

def _right(fb, font, s, x1, base, tone):
    _text(fb, font, s, x1 - width(font, s), base, tone)

def _wifi(fb, lit, col, x, y):
    """The dot always, the arcs up to lit in col, the others dim; dim all when
    lit is None (no WiFi)."""
    for n, part in enumerate(WIFI):
        c = DIM if lit is None or n > lit else col
        fb.blit((part, 12, 12, framebuf.MONO_HLSB), x, y, 0, _mono_palette(c))

def _wifi_frame(lit):
    """Only the icon goes up: a changed level needs no whole zone."""
    _D.ic.fill(BG)
    _wifi(_D.ic, lit, ORG, 0, 0)
    _window(WIFI_X, WIFI_Y, 12, 12)
    _cmd(0x2C, _D.icb)

def _draw_top(speed, fix, used, clock, lit):
    fb = _D.fb
    for top in range(0, TOP_H, BAND_H):
        fb.fill(BG)
        x = _text(fb, _HERO, speed, 4, 37 - top, "org" if fix else "mut")
        _text(fb, _LAB, "KM/H", x + 3, 37 - top, "mut")
        _right(fb, _NUM, str(used), 158, 17 - top, "txt" if fix else "red")
        _right(fb, _LAB, "műhold", 158, 26 - top, "mut")
        _right(fb, _TXT, clock, 158, 38 - top, "soft")
        _wifi(fb, lit, ORG, WIFI_X, WIFI_Y - top)
        _send(top, min(BAND_H, TOP_H - top))

def _draw_store(fb, top, store):
    """The flash: the per cent taken and how long it still logs over a gauge."""
    pct, left = store
    bad = pct >= STORE_BAD
    warn = pct >= STORE_WARN
    tone = "red" if bad else "yel" if warn else "soft"
    _text(fb, _TXT, "%d%%" % pct, STORE_X, 57 - top, tone)
    _right(fb, _TXT, left, STORE_X + STORE_W, 57 - top, tone)
    fb.fill_rect(STORE_X, 60 - top, STORE_W, 4, OFF)
    fb.fill_rect(STORE_X, 60 - top, max(1, STORE_W * pct // 100), 4,
                 _c(_TONES["red" if bad else "yel" if warn else "grn"]))

def _draw_low(quality, up, up_on, store):
    """How good the GPS is, as a phone's signal; the flash; what waits for upload."""
    fb = _D.fb
    word, tone = QUALITY[quality]
    col = _c(_TONES[tone])
    tone_up = "org" if up_on else "soft"
    for top in range(LOW_Y, H, BAND_H):
        fb.fill(BG)
        y0 = 74 - top
        for i in range(4):
            h = 6 + i * 6
            fb.fill_rect(5 + i * 8, y0 - h, 6, h, col if quality and i <= quality else OFF)
        _text(fb, _LAB, "GPS", 42, 62 - top, "mut")
        _text(fb, _WORD, word, 42, y0, tone)
        if up.strip():
            x = 158 - width(_TXT, up)
            fb.blit((UP_ICON, 9, 8, framebuf.MONO_HLSB), x - 12, y0 - 8, 0,
                    _mono_palette(_c(_TONES[tone_up])))
            _text(fb, _TXT, up, x, y0, tone_up)
        if store:
            _draw_store(fb, top, store)
        _send(top, min(BAND_H, H - top))

def set_state(speed, fix, used, clock, level, quality, up, up_on, store=None):
    """speed: text; fix: a usable fix; used: satellites in it; clock: text;
    level: WiFi 0-3 or None; quality: see QUALITY; up: text, up_on: points wait;
    store: (per cent of the flash taken, how long it still logs: text) or None."""
    _D.state = (speed, fix, used, clock, level, quality, up, up_on, store)

def frame():
    s = _D.state
    if s is None:
        return
    speed, fix, used, clock, level, quality, up, up_on, store = s
    top = (speed, fix, used, clock)
    if top != _D.top:
        _draw_top(speed, fix, used, clock, level)
        _D.top, _D.lit = top, level
    elif level != _D.lit:
        _wifi_frame(level)
        _D.lit = level
    low = (quality, up, up_on, store)
    if low != _D.low:
        _draw_low(quality, up, up_on, store)
        _D.low = low
