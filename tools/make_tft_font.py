#!/usr/bin/env python3
"""Write gps_tft_font.py: the Heltec TFT's anti-aliased fonts, from Space Grotesk.

Needs Pillow and the Space Grotesk TTFs; the output is committed, the build does not run
this. Each font: CHARS, the WIDTHS (advance, bytes), OFFS (2 bytes each, little
endian) into DATA, rows top down of GS4_HMSB (16 levels, 2 pixels a byte, a row
(w + 1) // 2 bytes), HEIGHT rows a glyph, BASE the baseline's row. In one bytes
object each (frozen, in flash), not a dict of glyphs in the heap. Only the
characters the screen shows, only the rows from the first to the last with ink
in any of them: the font's blank rows above and below (nothing to blit: level 0
is the blit's key) are over 1/3 of the file. BASE counts from the first row kept, so
a string stays on the same rows (it can reach below HEIGHT: no descender).
"""

D = "/usr/share/fonts/truetype/space-grotesk-zorin-os/SpaceGrotesk-%s.ttf"


def crop(widths, data, height, base):
    """The rows of a font's glyphs (widths, all of data, height rows each) from
    the first to the last with ink in any glyph -> (offs, data, height, base).
    All blank: one row."""
    glyphs, o = [], 0
    for w in widths:
        n = (w + 1) // 2
        glyphs.append([data[o + r * n:o + (r + 1) * n] for r in range(height)])
        o += n * height
    if o != len(data):
        raise ValueError("data is %d bytes, not %d" % (len(data), o))
    ink = [r for r in range(height) if any(any(g[r]) for g in glyphs)] or [0]
    top, bot = ink[0], ink[-1]
    offs, out = bytearray(), bytearray()
    for g in glyphs:
        offs += len(out).to_bytes(2, "little")
        for row in g[top:bot + 1]:
            out += row
    return bytes(offs), bytes(out), bot - top + 1, base - top


def font(weight, size, chars):
    from PIL import Image, ImageDraw, ImageFont
    f = ImageFont.truetype(D % weight, size)
    asc, desc = f.getmetrics()
    height, base = asc + desc, asc
    widths, data = bytearray(), bytearray()
    for ch in chars:
        w = max(1, int(round(f.getlength(ch))))
        m = Image.new("L", (w + 2, height), 0)
        ImageDraw.Draw(m).text((0, base), ch, fill=255, font=f, anchor="ls")
        box = m.getbbox()
        w = max(w, box[2]) if box else w  # ink past the advance (4 at 10 px) stays
        px = m.load()
        widths.append(w)
        for y in range(height):
            for x in range(0, w, 2):
                hi = px[x, y] >> 4
                lo = px[x + 1, y] >> 4 if x + 1 < w else 0
                data.append(hi << 4 | lo)
    offs, data, height, base = crop(widths, bytes(data), height, base)
    return chars, bytes(widths), offs, data, height, base


def main():
    fonts = (("CLOCK", font("Bold", 15, "0123456789:-")),
             ("SATS", font("Bold", 11, "0123456789")),
             ("UP", font("Bold", 14, "0123456789kész")),
             ("WORD", font("Bold", 12, "nicsgyekzpörő")),
             ("SMALL", font("Bold", 10, "0123456789% napóratel<km,icsú")))
    with open("gps_tft_font.py", "w", encoding="utf-8") as out:
        out.write("# Written by tools/make_tft_font.py (Space Grotesk): see there.\n")
        for name, (chars, widths, offs, data, height, base) in fonts:
            out.write("%s = (%r,\n    %r,\n    %r,\n    %r,\n    %d, %d)\n"
                      % (name, chars, widths, offs, data, height, base))


if __name__ == "__main__":
    main()
