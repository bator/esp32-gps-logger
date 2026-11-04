#!/usr/bin/env python3
"""web.html as it goes on the board: smaller, gzipped. deploy.sh and
build_firmware.sh --publish call this, web.html itself stays readable.

    tools/web_min.py web.html web.html.gz     the gzip the board keeps
    tools/web_min.py web.html                 the stripped page, on stdout

minify() takes out HTML, CSS and JS comments and the indentation, blank lines
and runs of spaces. Nothing else: the lines stay (a JS newline can end a
statement) and so do the spaces around JS operators (taking those out needs a
real parser: an earlier attempt broke a regex). The JS and the CSS are scanned
token by token, so a "//" or "/*" in a string or a regex is left alone; what
it cannot read (a string or a regex cut short, a template literal) stops the
build rather than ship a broken page.

The gzip: zlib level 9, no name or mtime (the same page gives the same bytes),
the window gps_logger.py's WEB_GZ_WBITS says. Not gzip -9: its 32 KB window is
what inflating it takes on the board, more than the Python heap has free.
"""
import os
import re
import sys
import zlib

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
# after these a "/" starts a regex, not a division
_JS_KEYWORDS = ("return", "typeof", "case", "in", "of", "delete", "void", "throw",
                "new", "else", "do", "instanceof", "yield", "await")
_PAREN_HEADS = ("if", "while", "for", "with")
_WORD = re.compile(r"[\w$\u0080-￿]+")


class MinifyError(ValueError):
    pass


def _space(out, ws):
    """A run of white space: a newline when it holds one (none twice, none at
    the start, no space before it), else one space."""
    if "\n" in ws or "\r" in ws:
        while out and out[-1] == " ":
            out.pop()
        if out and out[-1] != "\n":
            out.append("\n")
    elif out and out[-1] not in (" ", "\n"):
        out.append(" ")


def _string(src, i):
    """The end of the string starting at src[i]."""
    q, j, n = src[i], i + 1, len(src)
    while j < n:
        c = src[j]
        if c == "\\":
            j += 2
            continue
        if c == q:
            return j + 1
        if c == "\n":
            break
        j += 1
    raise MinifyError("a string cut short: %r" % src[i:i + 60])


def _regex(src, i):
    """The end of the regex literal starting at src[i], with its flags."""
    j, n, cls = i + 1, len(src), False
    while j < n:
        c = src[j]
        if c == "\\":
            j += 2
            continue
        if c == "\n":
            break
        if c == "[":
            cls = True
        elif c == "]":
            cls = False
        elif c == "/" and not cls:
            j += 1
            while j < n and (src[j].isalnum() or src[j] in "_$"):
                j += 1
            return j
        j += 1
    raise MinifyError("a regex cut short: %r" % src[i:i + 60])


def minify_js(src):
    """src without comments, indentation or blank lines, token for token."""
    out, i, n = [], 0, len(src)
    # what came before: may a "/" here start a regex?
    regex_ok, parens, word = True, [], ""
    while i < n:
        c = src[i]
        if c in " \t\r\n\f\v":
            j = i
            while j < n and src[j] in " \t\r\n\f\v":
                j += 1
            _space(out, src[i:j])
            i = j
        elif src.startswith("//", i):
            j = src.find("\n", i)
            i = n if j < 0 else j
        elif src.startswith("/*", i):
            j = src.find("*/", i + 2)
            if j < 0:
                raise MinifyError("a comment cut short: %r" % src[i:i + 60])
            # (one with a newline ends a statement as the newline would)
            _space(out, "\n" if "\n" in src[i:j] else " ")
            i = j + 2
        elif c in "\"'":
            j = _string(src, i)
            out.append(src[i:j])
            i, regex_ok, word = j, False, ""
        elif c == "`":
            raise MinifyError("a template literal: tools/web_min.py does not read those: %r" % src[i:i + 60])
        elif c == "/":
            if regex_ok:
                j = _regex(src, i)
                out.append(src[i:j])
                i, regex_ok, word = j, False, ""
            else:
                out.append(c)
                i, regex_ok, word = i + 1, True, ""
        else:
            m = _WORD.match(src, i)
            if m:
                word = m.group()
                out.append(word)
                i = m.end()
                regex_ok = word in _JS_KEYWORDS
                continue
            out.append(c)
            i += 1
            if src.startswith(c, i) and c in "+-":   # x++ / 2: as after x
                out.append(c)
                i += 1
                continue
            if c == "(":
                parens.append(word in _PAREN_HEADS)
                regex_ok = True
            elif c == ")":
                # "if (x) /re/.test(y)": a regex, "(a + b) / 2" a division
                regex_ok = parens.pop() if parens else False
            elif c == "]":
                regex_ok = False
            else:
                # "}": a block's end more often than an object's, and taking a
                # division for a regex fails loudly, the other way round not
                regex_ok = True
            word = ""
    return "".join(out).strip()


def minify_css(src):
    """src without comments, indentation or blank lines."""
    out, i, n = [], 0, len(src)
    while i < n:
        c = src[i]
        if c in " \t\r\n\f":
            j = i
            while j < n and src[j] in " \t\r\n\f":
                j += 1
            _space(out, src[i:j])
            i = j
        elif src.startswith("/*", i):
            j = src.find("*/", i + 2)
            if j < 0:
                raise MinifyError("a CSS comment cut short: %r" % src[i:i + 60])
            i = j + 2
            # (a/**/b are two tokens, a /* */ b needs no second space)
            if out and out[-1] not in (" ", "\n") and i < n and src[i] not in " \t\r\n\f":
                out.append(" ")
        elif c in "\"'":
            j = _string(src, i)
            out.append(src[i:j])
            i = j
        elif src[i:i + 4].lower() == "url(" and src[i + 4:i + 5] not in ("'", '"'):
            j = src.find(")", i)                 # (url(a/*b) is no comment)
            if j < 0:
                raise MinifyError("a url( cut short: %r" % src[i:i + 60])
            out.append(src[i:j + 1])
            i = j + 1
        else:
            out.append(c)
            i += 1
    return "".join(out).strip()


def minify_html(src):
    """Markup (no <script> or <style>): no comments, indentation or blank lines."""
    src = re.sub(r"<!--.*?-->", "", src, flags=re.S)
    return re.sub(r"[ \t\r]*\n[ \t\r\n]*", "\n", src)


_BLOCK = re.compile(r"(<(style|script)\b[^>]*>)(.*?)(</\2>)", re.S | re.I)


def minify(page):
    """The page's text, stripped; the same rendering, the same script."""
    out, pos = [], 0
    for m in _BLOCK.finditer(page):
        fn = minify_js if m.group(2).lower() == "script" else minify_css
        out.append(minify_html(page[pos:m.start()]))
        out.append(m.group(1) + "\n" + fn(m.group(3)) + "\n" + m.group(4))
        pos = m.end()
    out.append(minify_html(page[pos:]))
    return "".join(out).strip() + "\n"


def wbits(path=None):
    """gps_logger.py's WEB_GZ_WBITS: a wider window cannot be inflated on the
    board. The value in the source: the logger needs a board to import."""
    with open(path or os.path.join(ROOT, "gps_logger.py"), encoding="utf-8") as f:
        m = re.search(r"^WEB_GZ_WBITS = ([0-9]+)", f.read(), re.M)
    w = int(m.group(1)) if m else 0
    if not 9 <= w <= 15:
        raise MinifyError("No WEB_GZ_WBITS in gps_logger.py: %r" % (m and m.group(1)))
    return w


def gz(page, w):
    """page (bytes) gzipped with a 2**w window."""
    z = zlib.compressobj(9, zlib.DEFLATED, 16 + w)
    return z.compress(page) + z.flush()


def main(argv):
    if len(argv) not in (2, 3):
        print(__doc__, file=sys.stderr)
        return 2
    try:
        with open(argv[1], encoding="utf-8") as f:
            page = minify(f.read()).encode("utf-8")
        if len(argv) == 2:
            sys.stdout.buffer.write(page)
            return 0
        data = gz(page, wbits())
    except (OSError, UnicodeError, MinifyError) as e:
        print("%s: %s" % (argv[0], e), file=sys.stderr)
        return 1
    with open(argv[2], "wb") as f:
        f.write(data)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
