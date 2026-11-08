# The Heltec Wireless Tracker's TFT: 0.96", 160x80, ST7735S on SPI, powered by
# Vext with the UC6580 (pins and the 1/26 offset measured on the board). The
# logger hands over what to show (set_state) and calls frame() every TFT_FRAME.
#
# Ink on paper. Top zone, left: today's route, a line of 2x2 dots over a faint grid,
# the latest trip red, a round dot where the day began and one where it is now. Right:
# the clock, the satellites in the fix (an icon and the count), today's km and WiFi in
# four rising bars, a rule, what waits for upload. Low zone: the GPS's quality, a tile
# in its colour with a word, and how full the flash is with how long it still logs
# over a gauge, amber from 70%, red from 90%. The speed is on the status page.
#
# One band of 160x16 is all the RAM it keeps, not a 25.6 KB frame buffer, and the
# glyphs are read from flash, 16-level (anti-aliased) GS4: a zone goes up band by
# band, only when it changed (the top zone four bands, the low zone one: row 63,
# paper in both, belongs to the top). Colours are RGB565 byte-swapped: the
# panel's order.
import framebuf
import struct
import time
from machine import Pin, SPI

import gps_tft_font as F

W, H = 160, 80
XO, YO = 1, 26                           # the panel inside the controller
BAND_H = 16
TOP_H, LOW_Y = 64, 64                    # the zones: rows 0-63 (4 bands), 64-79 (1)
BL_PIN = 21
RIGHT = 158                              # the right column's text ends here
BARS_X, BARS_Y = 140, 42                 # WiFi: the first bar's left, their foot
KM_X = 92                                # today's km, on the bars' foot
RULE_X, RULE_Y = 96, 47
TILE_W, TILE_Y = 62, 64                  # the GPS tile
STORE_X, STORE_Y, STORE_W, STORE_H = 66, 75, 92, 3   # the flash's gauge
STORE_WARN, STORE_BAD = 70, 90           # per cent: amber, red

def _c(t):
    c = (t[0] & 0xF8) << 8 | (t[1] & 0xFC) << 3 | t[2] >> 3
    return c >> 8 | (c & 255) << 8

# The tones text comes in, by index: a palette's key stays a small int
PAPER, INK, RED, AMBER, GREEN, ORANGE = range(6)
_RGB = ((0xF4, 0xF0, 0xE6), (0x11, 0x11, 0x11), (0xD7, 0x26, 0x1E), (0xC9, 0x8A, 0x00),
        (0x1F, 0x8A, 0x4C), (0xD9, 0x60, 0x1A))
_COL = tuple(_c(t) for t in _RGB)
DIM, OFF = _c((0xE2, 0xDC, 0xCB)), _c((0xCF, 0xC9, 0xBA))   # unlit dots, gauge; bars
# by quality: 0 no fix, 1 weak, 2 fair, 3 strong: its word and tile
QUALITY = (("nincs", RED), ("gyenge", ORANGE), ("közepes", AMBER), ("erős", GREEN))

ROUTE_X, ROUTE_Y = 4, 4                  # the pixel a route's (0, 0) falls on
ROUTE_NOTE = "nincs út"                  # a day without a route

# 11x11 MONO_HLSB, 2 bytes a row: a satellite, a dish over a body between two
# solar panels, left of the count and on its baseline
SAT_ICON = bytes.fromhex("1f000e00e4e0eee0eee0ffe0eee0eee0e4e0e4e00e00")
SAT_W, SAT_GAP = 11, 3

_FONTS = [(f, memoryview(f[3])) for f in (F.CLOCK, F.SATS, F.UP, F.WORD, F.SMALL)]
_CLOCK, _SATS, _UP, _WORD, _SMALL = _FONTS
_pal = {}

class _D:
    spi = cs = dc = None
    buf = fb = None
    state = None
    top = low = None                     # what the panel shows

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
    _D.top = _D.low = None
    _D.fb.fill(_COL[PAPER])
    for y in range(0, H, BAND_H):
        _send(y, BAND_H)
    Pin(BL_PIN, Pin.OUT, value=1)

def off():
    """Dark: a frozen display must not stay lit. A dead SPI bus does not stop this."""
    Pin(BL_PIN, Pin.OUT, value=0)

def _mono_palette(tone):
    k = 64 | tone
    p = _pal.get(k)
    if p is None:
        p = framebuf.FrameBuffer(bytearray(4), 2, 1, framebuf.RGB565)
        p.pixel(0, 0, 0)                 # (blit's key: transparent)
        p.pixel(1, 0, _COL[tone])
        _pal[k] = p
    return p

def _gs4_palette(tone, bg=PAPER):
    """16 levels from the background to a tone: the anti-aliasing. One a pair and
    only seven pairs: ink, red, amber on paper, paper on the four tiles."""
    k = tone << 3 | bg
    p = _pal.get(k)
    if p is None:
        t, b = _RGB[tone], _RGB[bg]
        p = framebuf.FrameBuffer(bytearray(32), 16, 1, framebuf.RGB565)
        for v in range(16):
            p.pixel(v, 0, _c(tuple((b[k] * (15 - v) + t[k] * v) // 15 for k in range(3))))
        _pal[tone << 3 | bg] = p
    return p

def _index(f, ch):
    """The glyph of ch in the font f, -1 for one the font does not have: a
    blank as wide as _gap (not glyph 0, a digit in place of the character)."""
    return f[0].find(ch)

def _gap(f):
    return (f[4] + 1) // 2

def width(font, s):
    f = font[0]
    n = 0
    for ch in s:
        i = _index(f, ch)
        n += f[1][i] if i >= 0 else _gap(f)
    return n

def _text(fb, font, s, x, base, tone, bg=PAPER):
    """s with its baseline at row base, each glyph blitted straight from flash.
    Nothing to blit outside the band (each glyph's slice and tuple, in each band,
    went to the garbage): only how wide it is, the x that follows."""
    f, mv = font
    y = base - f[5]
    h = f[4]
    if y >= BAND_H or y + h <= 0:
        return x + width(font, s)
    p = _gs4_palette(tone, bg)
    key = _COL[bg]
    for ch in s:
        i = _index(f, ch)
        if i < 0:
            x += _gap(f)
            continue
        w = f[1][i]
        o = f[2][i * 2] | f[2][i * 2 + 1] << 8
        fb.blit((mv[o:o + (w + 1) // 2 * h], w, h, framebuf.GS4_HMSB), x, y, key, p)
        x += w
    return x

def _right(fb, font, s, x1, base, tone):
    _text(fb, font, s, x1 - width(font, s), base, tone)

def _round(fb, x, y, c):
    """A 6x6 dot without its corners."""
    fb.fill_rect(x + 1, y, 4, 6, c)
    fb.fill_rect(x, y + 1, 6, 4, c)

def _route(fb, top, route):
    """The grid; on it the route's rows in this band: (pixels, points, the latest
    trip's first point, a version). A point marked in its y starts a trip, no line
    to it."""
    for j in range(7):
        y = 4 + j * 8 - top
        if -2 < y < BAND_H:
            for i in range(11):
                fb.fill_rect(4 + i * 8, y, 2, 2, DIM)
    if route is None:
        return
    xy, n, red, _v = route
    if not n:
        _text(fb, _SMALL, ROUTE_NOTE, 46 - width(_SMALL, ROUTE_NOTE) // 2, 35 - top, INK)
        return
    ink, lit = _COL[INK], _COL[RED]
    x0 = y0 = 0
    for i in range(n):
        x1 = xy[2 * i] + ROUTE_X
        v = xy[2 * i + 1]
        y1 = (v & 127) + ROUTE_Y - top
        c = lit if i >= red else ink
        if v & 128 or not i:
            x0, y0 = x1, y1
        if (y0 > -2 or y1 > -2) and (y0 < BAND_H or y1 < BAND_H):
            dx, dy = x1 - x0, y1 - y0
            steps = max(abs(dx), abs(dy))
            for k in range(steps + 1):   # (steps 0: the one dot of a trip's start)
                if steps:
                    fb.fill_rect(x0 + dx * k // steps, y0 + dy * k // steps, 2, 2, c)
                else:
                    fb.fill_rect(x0, y0, 2, 2, c)
        x0, y0 = x1, y1
    v = n - 1
    _round(fb, xy[0] + ROUTE_X - 2, (xy[1] & 127) + ROUTE_Y - 2 - top, ink)
    _round(fb, xy[2 * v] + ROUTE_X - 2, (xy[2 * v + 1] & 127) + ROUTE_Y - 2 - top, lit)

def _draw_top(route, km, fix, used, clock, level, up):
    fb = _D.fb
    tone = INK if fix else RED
    used = str(used)
    x = RIGHT - width(_SATS, used)
    for top in range(0, TOP_H, BAND_H):
        fb.fill(_COL[PAPER])
        _route(fb, top, route)
        _right(fb, _CLOCK, clock, RIGHT, 14 - top, INK)
        _text(fb, _SATS, used, x, 28 - top, tone)
        if top < 28 and top + BAND_H > 28 - SAT_W:
            fb.blit((SAT_ICON, SAT_W, SAT_W, framebuf.MONO_HLSB), x - SAT_GAP - SAT_W,
                    28 - SAT_W - top, 0, _mono_palette(tone))
        for i in range(4):
            h = 2 + i * 3
            fb.fill_rect(BARS_X + i * 5, BARS_Y - h - top, 4, h,
                         OFF if level is None or i > level else _COL[INK])
        _text(fb, _SMALL, km, KM_X, BARS_Y - top, INK)
        fb.fill_rect(RULE_X, RULE_Y - top, W - RULE_X, 1, _COL[INK])
        if up.strip():
            _right(fb, _UP, up, RIGHT, 62 - top, INK)
        _send(top, min(BAND_H, TOP_H - top))

def _draw_low(quality, store):
    """How good the GPS is, a tile; the flash, per cent and time over a gauge."""
    fb = _D.fb
    word, tile = QUALITY[quality]
    x = TILE_W // 2 - width(_WORD, word) // 2
    if store:
        pct, left = store
        tone = RED if pct >= STORE_BAD else AMBER if pct >= STORE_WARN else INK
        pc = "%d%%" % pct
    for top in range(LOW_Y, H, BAND_H):
        fb.fill(_COL[PAPER])
        fb.fill_rect(0, TILE_Y - top, TILE_W, H - TILE_Y, _COL[tile])
        _text(fb, _WORD, word, x, 76 - top, PAPER, tile)
        if store:
            _text(fb, _SMALL, pc, STORE_X, 72 - top, tone)
            _right(fb, _SMALL, left, RIGHT, 72 - top, tone)
            fb.fill_rect(STORE_X, STORE_Y - top, STORE_W, STORE_H, DIM)
            fb.fill_rect(STORE_X, STORE_Y - top, max(1, STORE_W * pct // 100), STORE_H,
                         _COL[tone])
        _send(top, min(BAND_H, H - top))

def set_state(route, km, fix, used, clock, level, quality, up, up_on, store=None):
    """route: (pixels, points, the latest trip's first point, a version) or None, the
    day not known yet: see _route, a new one only when it changed; km: text;
    fix: a usable fix; used: satellites in it; clock: text; level: WiFi 0-3 or None;
    quality: see QUALITY; up: text (" ": no upload set up); up_on: points wait (not
    shown apart from up's text); store: (per cent of the flash taken, how long it
    still logs: text) or None."""
    _D.state = (route, km, fix, used, clock, level, quality, up, up_on, store)

def frame():
    s = _D.state
    if s is None:
        return
    route, km, fix, used, clock, level, quality, up, _on, store = s
    top = (route, km, fix, used, clock, level, up)
    if top != _D.top:
        _draw_top(route, km, fix, used, clock, level, up)
        _D.top = top
    low = (quality, store)
    if low != _D.low:
        _draw_low(quality, store)
        _D.low = low
