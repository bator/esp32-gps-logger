# The Heltec Wireless Tracker's TFT: 0.96", 160x80, ST7735S on SPI, powered by
# Vext with the UC6580 (pins and the 1/26 offset measured on the board). The
# logger hands over what to show (set_state) and calls frame() every TFT_FRAME.
#
# Top: the speed, big and orange, the satellites in the fix, the WiFi icon and the
# clock. Bottom: every satellite's signal as a bar, what waits for upload.
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
TOP_H, BARS_Y = 41, 46                   # the zones: rows 0-40, rows 46-79
BL_PIN = 21
WAVE = 2                                 # frames an arc of the WiFi icon stays
WIFI_X, WIFI_Y = 113, 28
N_BARS = 21

def _c(t):
    c = (t[0] & 0xF8) << 8 | (t[1] & 0xFC) << 3 | t[2] >> 3
    return c >> 8 | (c & 255) << 8

_BG, _DIM_T = (5, 6, 8), (51, 59, 70)
_TONES = {
    "org": (255, 106, 43), "txt": (245, 247, 249), "soft": (198, 206, 216),
    "mut": (141, 153, 167), "red": (255, 90, 90),
}
BG, DIM = _c(_BG), _c(_DIM_T)
# a bar's colour by its signal (dB-Hz): red, orange, yellow; not in the fix: brown
HEAT = (_c((255, 59, 31)), _c((255, 122, 43)), _c((255, 194, 61)))
UNUSED, STUB = _c((90, 70, 56)), _c((42, 26, 18))
# The WiFi icon's four parts, 12x12 MONO_HLSB: the dot and the arcs inner to outer.
# Level 0-3: that many arcs fill up again and again, None: all dim and still.
WIFI = (b"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x06\x00\x06\x00\x00\x00",
        b"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x0f\x00\x10\x80\x00\x00\x00\x00\x00\x00\x00\x00",
        b"\x00\x00\x00\x00\x00\x00\x0f\x00\x30\xc0\x40\x20\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00",
        b"\x1f\x80\x60\x60\x80\x10\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00")

_FONTS = [(f, memoryview(f[3])) for f in (F.HERO, F.NUM, F.TXT, F.LAB)]
_HERO, _NUM, _TXT, _LAB = _FONTS
_pal = {}

class _D:
    spi = cs = dc = None
    buf = fb = ic = icb = None
    t = 0
    state = None
    top = bars = lit = None              # what the panel shows

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
    _D.top = _D.bars = _D.lit = None
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
    """s with its baseline at row base, each glyph blitted straight from flash."""
    f, mv = font
    p = _gs4_palette(tone)
    y = base - f[5]
    h = f[4]
    for ch in s:
        i = _index(f, ch)
        w = f[1][i]
        o = f[2][i * 2] | f[2][i * 2 + 1] << 8
        fb.blit((mv[o:o + (w + 1) // 2 * h], w, h, framebuf.GS4_HMSB), x, y, BG, p)
        x += w
    return x

def _right(fb, font, s, x1, base, tone):
    _text(fb, font, s, x1 - width(font, s), base, tone)

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
        fb.blit((part, 12, 12, framebuf.MONO_HLSB), x, y, 0, _mono_palette(c))

def _wifi_frame(lit):
    """Only the icon goes up: a wave would redraw the zone 2.5 times a second."""
    _D.ic.fill(BG)
    _wifi(_D.ic, lit, HEAT[1], 0, 0)
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
        _wifi(fb, lit, HEAT[1], WIFI_X, WIFI_Y - top)
        _send(top, min(BAND_H, TOP_H - top))

def bar_h(cno):
    return max(2, (cno - 12) * 26 // 34)

def _draw_bars(bars, up, up_on):
    """bars: 2 bytes a satellite, strongest first: cno, in the fix."""
    fb = _D.fb
    for top in range(BARS_Y, H, BAND_H):
        fb.fill(BG)
        y0 = 74 - top
        for i in range(N_BARS):
            x = 3 + i * 6
            cno = bars[i * 2] if i * 2 < len(bars) else 0
            if cno:
                c = (HEAT[0] if cno < 25 else HEAT[1] if cno < 33 else HEAT[2]) if bars[i * 2 + 1] else UNUSED
                h = bar_h(cno)
                fb.fill_rect(x, y0 - h, 4, h, c)
            else:
                fb.fill_rect(x, y0 - 1, 4, 1, STUB)
        _right(fb, _TXT, up, 158, y0, "org" if up_on else "soft")
        _send(top, min(BAND_H, H - top))

def set_state(speed, fix, used, clock, level, bars, up, up_on):
    """speed: text; fix: a usable fix; used: satellites in it; clock: text;
    level: WiFi 0-3 or None; bars: see _draw_bars; up: text, up_on: points wait."""
    _D.state = (speed, fix, used, clock, level, bars, up, up_on)

def frame():
    s = _D.state
    if s is None:
        return
    _D.t += 1
    speed, fix, used, clock, level, bars, up, up_on = s
    lit = None if level is None else wifi_lit(level, _D.t)
    top = (speed, fix, used, clock, level)
    if top != _D.top:
        _draw_top(speed, fix, used, clock, lit)
        _D.top, _D.lit = top, lit
    elif lit != _D.lit:
        _wifi_frame(lit)
        _D.lit = lit
    b = (bars, up, up_on)
    if b != _D.bars:
        _draw_bars(bars, up, up_on)
        _D.bars = b
