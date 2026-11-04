#!/usr/bin/env python3
"""Write gps_tft_font.py: the Heltec TFT's anti-aliased fonts, from Inter (Rsms).

Needs Pillow and the Inter TTFs; the output is committed, the build does not run
this. Each font: CHARS, the WIDTHS (advance, bytes), OFFS (2 bytes each, little
endian) into DATA, rows top down of GS4_HMSB (16 levels, 2 pixels a byte, a row
(w + 1) // 2 bytes), HEIGHT rows a glyph, BASE the baseline's row. In one bytes
object each (frozen, in flash), not a dict of glyphs in the heap. Only the
characters the screen shows.
"""
from PIL import Image, ImageDraw, ImageFont

D = "/usr/share/fonts/truetype/inter-zorin-os/Inter-%s.ttf"


def font(weight, size, chars):
    f = ImageFont.truetype(D % weight, size)
    asc, desc = f.getmetrics()
    height, base = asc + desc, asc
    widths, offs, data = bytearray(), bytearray(), bytearray()
    for ch in chars:
        w = max(1, int(round(f.getlength(ch))))
        m = Image.new("L", (w, height), 0)
        ImageDraw.Draw(m).text((0, base), ch, fill=255, font=f, anchor="ls")
        px = m.load()
        offs += len(data).to_bytes(2, "little")
        widths.append(w)
        for y in range(height):
            for x in range(0, w, 2):
                hi = px[x, y] >> 4
                lo = px[x + 1, y] >> 4 if x + 1 < w else 0
                data.append(hi << 4 | lo)
    return chars, bytes(widths), bytes(offs), bytes(data), height, base


FONTS = (("HERO", font("Bold", 38, "0123456789-")),
         ("NUM", font("Bold", 17, "0123456789-")),
         ("TXT", font("SemiBold", 10, "0123456789:-kész %napóratle<")),
         ("LAB", font("SemiBold", 9, "KM/HGPSműhold")),
         ("WORD", font("Bold", 13, "erőögsynickzp")))

with open("gps_tft_font.py", "w", encoding="utf-8") as out:
    out.write("# Written by tools/make_tft_font.py (Inter): see there.\n")
    for name, (chars, widths, offs, data, height, base) in FONTS:
        out.write("%s = (%r,\n    %r,\n    %r,\n    %r,\n    %d, %d)\n"
                  % (name, chars, widths, offs, data, height, base))
