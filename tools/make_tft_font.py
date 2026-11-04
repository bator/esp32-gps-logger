#!/usr/bin/env python3
"""Write gps_tft_font.py: the Heltec TFT's bitmap fonts, from DejaVu Sans.

Needs Pillow and fonts-dejavu; the output is committed, the build does not run
this. Each font: CHARS, the WIDTHS (advance, bytes), OFFS (2 bytes each, little
endian) into DATA, rows top down of MONO_HLSB, HEIGHT rows a glyph. In one bytes
object each (frozen, in flash), not a dict of glyphs in the heap.
"""
import string

from PIL import Image, ImageDraw, ImageFont

D = "/usr/share/fonts/truetype/dejavu/"
TEXT = string.printable[:95] + "áéíóöőúüűÁÉÍÓÖŐÚÜŰ°±"


def font(path, size, chars, height, base):
    f = ImageFont.truetype(D + path, size)
    widths, offs, data = bytearray(), bytearray(), bytearray()
    for ch in chars:
        w = max(1, int(round(f.getlength(ch))))
        m = Image.new("1", (w, height), 0)
        ImageDraw.Draw(m).text((0, base), ch, fill=1, font=f, anchor="ls")
        offs += len(data).to_bytes(2, "little")
        widths.append(w)
        px = m.load()
        for y in range(height):
            byte = 0
            for x in range(w):
                byte = byte << 1 | (1 if px[x, y] else 0)
                if x % 8 == 7:
                    data.append(byte)
                    byte = 0
            if w % 8:
                data.append(byte << (8 - w % 8))
    return chars, bytes(widths), bytes(offs), bytes(data), height


FONTS = (("REG", font("DejaVuSans.ttf", 10, TEXT, 13, 10)),
         ("BOLD", font("DejaVuSans-Bold.ttf", 10, TEXT, 13, 10)),
         ("HERO", font("DejaVuSans-Bold.ttf", 26, "0123456789-", 22, 21)))

with open("gps_tft_font.py", "w", encoding="utf-8") as out:
    out.write("# Written by tools/make_tft_font.py (DejaVu Sans): see there.\n")
    for name, (chars, widths, offs, data, height) in FONTS:
        out.write("%s = (%r,\n    %r,\n    %r,\n    %r,\n    %d)\n"
                  % (name, chars, widths, offs, data, height))
