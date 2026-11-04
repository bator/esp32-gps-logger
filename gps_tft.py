# The Heltec Wireless Tracker's TFT: 0.96", 160x80, ST7735S on SPI, powered by
# Vext with the UC6580 (pins and the 1/26 offset measured on the board). The
# logger hands over what to show (set_state) and calls frame() every TFT_FRAME.
#
# One band of 160x16 is all the RAM it keeps, not a 25.6 KB frame buffer, and the
# glyphs are read from flash, not cached: the Python heap doubling for a display
# takes 64 KB from the IDF heap, and an upload needs a 48 KB block of that. The
# header goes up in two bands when it changes, a row whenever it changes or
# scrolls. A row too long for the screen holds, slides out to the left and comes
# back in from the right. Colours are RGB565 byte-swapped: the panel's order.
import framebuf
import struct
import time
from machine import Pin, SPI

import gps_tft_font as F

W, H = 160, 80
XO, YO = 1, 26                           # the panel inside the controller
HEAD_H, ROW_H = 30, 16
TEXT_X = 18                              # a row's text, after its icon
ROOM = W - TEXT_X
HOLD = 10                                # frames a long row waits at its start
STEP = 3                                 # px a frame it slides
BL_PIN = 21

def _c(r, g, b):
    c = (r & 0xF8) << 8 | (g & 0xFC) << 3 | b >> 3
    return c >> 8 | (c & 255) << 8

BG, BAND = _c(10, 13, 17), _c(21, 26, 33)
# a part's kind: bright, muted, green, amber, red
BRIGHT, MUTED, GREEN, AMBER, RED = 0, 1, 2, 3, 4
_COL = (_c(244, 246, 248), _c(142, 154, 168), _c(61, 220, 132), _c(255, 176, 32),
        _c(255, 90, 90))
# 12 x 12, MONO_HLSB: GPS is not on a row, it has the header
# The WiFi icon's four parts: the dot and the arcs inner to outer. Icon "w0".."w3":
# that many arcs fill up again and again, one a WAVE frames, "w-": all dim, still.
WIFI = (b"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x06\x00\x06\x00\x00\x00",
        b"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x0f\x00\x10\x80\x00\x00\x00\x00\x00\x00\x00\x00",
        b"\x00\x00\x00\x00\x00\x00\x0f\x00\x30\xc0\x40\x20\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00",
        b"\x1f\x80\x60\x60\x80\x10\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00")
WAVE = 2
DIM = _c(51, 59, 70)
ICONS = {
    "up": b"\x06\x00\x0f\x00\x1f\x80\x36\xc0\x06\x00\x06\x00\x06\x00\x06\x00\x00\x00\x7f\xe0\x7f\xe0\x00\x00",
    "mem": b"\x2a\x80\x7f\xc0\x80\x20\xbf\xa0\xa0\xa0\xa0\xa0\xa0\xa0\xbf\xa0\x80\x20\x7f\xc0\x2a\x80\x00\x00",
}

_FONTS = [(f, memoryview(f[3])) for f in (F.REG, F.BOLD, F.HERO)]
_REG, _BOLD, _HERO = _FONTS
_pal = {}

class _D:
    spi = cs = dc = None
    buf = fb = None
    ic = icb = None                      # the WiFi icon's own 12 x 12
    t = 0
    state = None
    head = None                          # the header on the panel
    rows = None                          # per row: parts, width, x, hold, drawn

def _cmd(c, data=None):
    _D.cs(0)
    _D.dc(0)
    _D.spi.write(bytes((c,)))
    if data:
        _D.dc(1)
        _D.spi.write(data)
    _D.cs(1)

def _send(y, h):
    """The band's top h rows to the panel's row y."""
    _cmd(0x2A, struct.pack(">HH", XO, XO + W - 1))
    _cmd(0x2B, struct.pack(">HH", YO + y, YO + y + h - 1))
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
        _D.buf = bytearray(W * ROW_H * 2)
        _D.fb = framebuf.FrameBuffer(_D.buf, W, ROW_H, framebuf.RGB565)
    _D.head = None
    _D.rows = [[None, 0, 0, HOLD, None, None] for _ in range(3)]
    if _D.ic is None:
        _D.icb = bytearray(12 * 12 * 2)
        _D.ic = framebuf.FrameBuffer(_D.icb, 12, 12, framebuf.RGB565)
    _D.fb.fill(BG)
    for y in range(0, H, ROW_H):
        _send(y, ROW_H)
    Pin(BL_PIN, Pin.OUT, value=1)

def off():
    """Dark: a frozen speed and a green fix must not stay lit. A dead SPI bus
    does not stop this."""
    Pin(BL_PIN, Pin.OUT, value=0)

def _palette(col):
    p = _pal.get(col)
    if p is None:
        p = framebuf.FrameBuffer(bytearray(4), 2, 1, framebuf.RGB565)
        p.pixel(0, 0, 0)                 # (blit's key: transparent)
        p.pixel(1, 0, col)
        _pal[col] = p
    return p

def _index(font, ch):
    i = font[0].find(ch)
    return i if i >= 0 else max(font[0].find("?"), 0)

def _width(font, s):
    f = font[0]
    n = 0
    for ch in s:
        n += f[1][_index(f, ch)]
    return n

def _text(fb, font, s, x, y, col):
    """Each glyph blitted straight from flash, none kept: 50-75 FrameBuffers
    with their bytes were 5-8 KB of the heap."""
    f, mv = font
    p = _palette(col)
    h = f[4]
    for ch in s:
        i = _index(f, ch)
        w = f[1][i]
        if x >= W:
            break
        if x + w > 0:
            o = f[2][i * 2] | f[2][i * 2 + 1] << 8
            fb.blit((mv[o:o + ((w + 7) // 8) * h], w, h, framebuf.MONO_HLSB), x, y, 0, p)
        x += w
    return x

def _part_font(kind):
    return _REG if kind == MUTED else _BOLD

def row_width(parts):
    n = 0
    for s, kind in parts:
        n += _width(_part_font(kind), s)
    return n

def scroll(c, w):
    """A row's x offset this frame (c: its state, x and hold at 2 and 3). It
    moves on from where it is, so a row changing width (-99 to -100 dBm) does
    not jump. 0 when it fits."""
    if w <= ROOM:
        c[2], c[3] = 0, HOLD
        return 0
    if c[3] > 0:
        c[3] -= 1
    else:
        c[2] += STEP
        if c[2] >= w + ROOM:
            c[2], c[3] = 0, HOLD
    x = c[2]
    return -x if x <= w else ROOM - (x - w)

def _draw_head(big, fix, fixkind, sats):
    fb = _D.fb
    for top, h in ((0, ROW_H), (ROW_H, HEAD_H - ROW_H)):
        fb.fill(BAND)
        x = _text(fb, _HERO, big, 3, 4 - top, _COL[BRIGHT])
        _text(fb, _REG, "km/h", x + 4, 14 - top, _COL[MUTED])
        _text(fb, _BOLD, fix, 104, 2 - top, _COL[fixkind])
        _text(fb, _BOLD, sats, 104, 16 - top, _COL[BRIGHT])
        _send(top, h)

def wifi_lit(level, t):
    """Arcs lit in frame t: 0 up to level, held for three steps, again."""
    n = level + 4
    s = (t // WAVE) % n
    return s if s < level else level

def _wifi(fb, lit, col, x, y):
    """The dot always, the arcs up to lit in col, the others dim; dim all when
    lit is None (no WiFi)."""
    for n, part in enumerate(WIFI):
        c = DIM if lit is None or n > lit else col
        fb.blit((part, 12, 12, framebuf.MONO_HLSB), x, y, 0, _palette(c))

def _wifi_frame(y, lit, col):
    """Only the icon goes up: a wave would redraw the row 2.5 times a second."""
    _D.ic.fill(BG)
    _wifi(_D.ic, lit, col, 0, 0)
    _cmd(0x2A, struct.pack(">HH", XO + 2, XO + 13))
    _cmd(0x2B, struct.pack(">HH", YO + y + 2, YO + y + 13))
    _cmd(0x2C, _D.icb)

def _draw_row(i, icon, kind, parts, off_x, lit):
    fb = _D.fb
    fb.fill(BG)
    x = TEXT_X + off_x
    for s, k in parts:
        x = _text(fb, _part_font(k), s, x, 1, _COL[k])
    fb.fill_rect(0, 0, TEXT_X, ROW_H, BG)
    if icon[0] == "w":
        _wifi(fb, lit, _COL[kind], 2, 2)
    else:
        fb.blit((ICONS[icon], 12, 12, framebuf.MONO_HLSB), 2, 2, 0, _palette(_COL[kind]))
    _send(HEAD_H + i * ROW_H, ROW_H)

def set_state(big, fix, fixkind, sats, rows):
    """rows: three (icon, its kind, [(text, kind), ...])."""
    _D.state = ((big, fix, fixkind, sats), rows)

def frame():
    s = _D.state
    if s is None:
        return
    _D.t += 1
    if s[0] != _D.head:
        _draw_head(*s[0])
        _D.head = s[0]
    for i, (icon, kind, parts) in enumerate(s[1]):
        c = _D.rows[i]
        if parts != c[0]:
            c[0], c[1] = parts, row_width(parts)
        off_x = scroll(c, c[1])
        lit = None
        if icon[0] == "w" and icon != "w-":
            lit = wifi_lit(int(icon[1]), _D.t)
        d = (icon, kind, parts, off_x)
        if d != c[4]:
            _draw_row(i, icon, kind, parts, off_x, lit)
            c[4], c[5] = d, lit
        elif lit != c[5]:
            _wifi_frame(HEAD_H + i * ROW_H, lit, _COL[kind])
            c[5] = lit
