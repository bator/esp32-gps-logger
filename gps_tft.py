# The Heltec Wireless Tracker's TFT: 0.96", 160x80, ST7735S on SPI, powered by
# Vext with the UC6580 (pins and the 1/26 offset measured on the board). The
# logger hands over what to show (set_state) and calls frame() every TFT_FRAME.
#
# One band of 160x30 is all the RAM it takes, not a 25.6 KB frame buffer: the
# header goes up once when it changes, a row whenever it changes or scrolls. A
# row too long for the screen holds, slides out to the left and comes back in
# from the right. Colours are RGB565 byte-swapped: the panel's order.
import framebuf
import struct
import time
from machine import Pin, SPI

import gps_tft_font as F

W, H = 160, 80
XO, YO = 1, 26                           # the panel inside the controller
HEAD_H, ROW_H = 30, 16
TEXT_X = 18                              # a row's text, after its icon
HOLD = 10                                # frames a long row waits at its start
STEP = 3                                 # px a frame it slides

def _c(r, g, b):
    c = (r & 0xF8) << 8 | (g & 0xFC) << 3 | b >> 3
    return c >> 8 | (c & 255) << 8

BG, BAND = _c(10, 13, 17), _c(21, 26, 33)
# a part's kind: bright, muted, green, amber, red
BRIGHT, MUTED, GREEN, AMBER, RED = 0, 1, 2, 3, 4
_COL = (_c(244, 246, 248), _c(142, 154, 168), _c(61, 220, 132), _c(255, 176, 32),
        _c(255, 90, 90))
# 12 x 12, MONO_HLSB: GPS is not on a row, it has the header
ICONS = {
    "net": b"\x00\x00\x3f\xc0\x60\x60\xcf\x30\x19\x80\x20\x40\x06\x00\x09\x00\x06\x00\x00\x00\x00\x00\x00\x00",
    "up": b"\x06\x00\x0f\x00\x1f\x80\x36\xc0\x06\x00\x06\x00\x06\x00\x06\x00\x00\x00\x7f\xe0\x7f\xe0\x00\x00",
    "mem": b"\x2a\x80\x7f\xc0\x80\x20\xbf\xa0\xa0\xa0\xa0\xa0\xa0\xa0\xbf\xa0\x80\x20\x7f\xc0\x2a\x80\x00\x00",
}

_g = {}                                  # glyphs used so far: (width, buffer)
_pal = {}
_ic = {}

class _D:
    spi = cs = dc = None
    buf = head = row = None
    t = 0
    state = None
    drawn = [None, None, None, None]     # header, rows: what the panel shows

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

def start():
    """The panel on and dark blue-black, then the backlight (not before: no
    flash of its random RAM)."""
    Pin(21, Pin.OUT, value=0)
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
    _D.buf = bytearray(W * HEAD_H * 2)
    _D.head = framebuf.FrameBuffer(_D.buf, W, HEAD_H, framebuf.RGB565)
    _D.row = framebuf.FrameBuffer(_D.buf, W, ROW_H, framebuf.RGB565)
    _D.drawn = [None, None, None, None]
    _D.head.fill(BG)
    _window(0, 0, W, HEAD_H)
    _cmd(0x2C, _D.buf)
    for y in range(HEAD_H, H, HEAD_H):
        _window(0, y, W, min(HEAD_H, H - y))
        _cmd(0x2C, memoryview(_D.buf)[:W * min(HEAD_H, H - y) * 2])
    Pin(21, Pin.OUT, value=1)

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
    return i if i >= 0 else font[0].find("?") if font is not F.HERO else 0

def _width(font, s):
    n = 0
    for ch in s:
        n += font[1][_index(font, ch)]
    return n

def _glyph(font, ch):
    k = (id(font), ch)
    g = _g.get(k)
    if g is None:
        i = _index(font, ch)
        w = font[1][i]
        h = font[4]
        n = ((w + 7) // 8) * h
        o = font[2][i * 2] | font[2][i * 2 + 1] << 8
        g = (w, framebuf.FrameBuffer(bytearray(font[3][o:o + n]), w, h,
                                     framebuf.MONO_HLSB))
        _g[k] = g
    return g

def _text(fb, font, s, x, y, col, x1=W):
    p = _palette(col)
    for ch in s:
        if x >= x1:
            break
        g = _glyph(font, ch)
        if x + g[0] > 0:
            fb.blit(g[1], x, y, 0, p)
        x += g[0]
    return x

def _part_font(kind):
    return F.REG if kind == MUTED else F.BOLD

def row_width(parts):
    n = 0
    for s, kind in parts:
        n += _width(_part_font(kind), s)
    return n

def pos(t, w, room):
    """A row's x offset in frame t: holds, slides out to the left, comes in
    from the right. 0 when it fits."""
    if w <= room:
        return 0
    t %= HOLD + (w + room) // STEP + 1
    if t < HOLD:
        return 0
    x = (t - HOLD) * STEP
    return -x if x <= w else room - (x - w)

def _draw_head(big, fix, fixkind, sats):
    fb = _D.head
    fb.fill(BAND)
    x = _text(fb, F.HERO, big, 3, 4, _COL[BRIGHT])
    _text(fb, F.REG, "km/h", x + 4, 14, _COL[MUTED])
    _text(fb, F.BOLD, fix, 104, 2, _COL[fixkind])
    _text(fb, F.BOLD, sats, 104, 16, _COL[BRIGHT])
    _window(0, 0, W, HEAD_H)
    _cmd(0x2C, _D.buf)

def _icon(fb, name, col):
    g = _ic.get(name)
    if g is None:
        g = framebuf.FrameBuffer(bytearray(ICONS[name]), 12, 12, framebuf.MONO_HLSB)
        _ic[name] = g
    fb.blit(g, 2, 2, 0, _palette(col))

def _draw_row(i, icon, kind, parts, off):
    fb = _D.row
    fb.fill(BG)
    x = TEXT_X + off
    for s, k in parts:
        font = _part_font(k)
        x = _text(fb, font, s, x, 1, _COL[k])
    fb.fill_rect(0, 0, TEXT_X, ROW_H, BG)
    _icon(fb, icon, _COL[kind])
    _window(0, HEAD_H + 1 + i * ROW_H - 1, W, ROW_H)
    _cmd(0x2C, memoryview(_D.buf)[:W * ROW_H * 2])

def set_state(big, fix, fixkind, sats, rows):
    """rows: three (icon, its kind, [(text, kind), ...])."""
    _D.state = ((big, fix, fixkind, sats), rows)

def frame():
    s = _D.state
    if s is None:
        return
    _D.t += 1
    if s[0] != _D.drawn[0]:
        _draw_head(*s[0])
        _D.drawn[0] = s[0]
    room = W - TEXT_X
    for i, (icon, kind, parts) in enumerate(s[1]):
        off = pos(_D.t, row_width(parts), room)
        d = (icon, kind, parts, off)
        if d != _D.drawn[i + 1]:
            _draw_row(i, icon, kind, parts, off)
            _D.drawn[i + 1] = d
