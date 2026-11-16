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
build rather than ship a broken page, and so does markup whose white space
shows (<pre>, <textarea>) or a <script>/<style> without an end tag it reads.
A JS line (and a // comment) ends at \n, \r, U+2028 or U+2029; an HTML
comment is out before the page is split, so a "<style>" named in one starts
no block, and a "<!--" in a script's string is no comment.

The gzip: zlib level 9, no name or mtime (the same page gives the same bytes),
the window gps_logger.py's WEB_GZ_WBITS says. Not gzip -9: its 32 KB window is
what inflating it takes on the board, more than the Python heap has free.
"""
import os
import re
import shutil
import subprocess
import sys
import tempfile
import zlib

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
# after these a "/" starts a regex, not a division (not after a "." : x.in /
# 8 is a property; not "of": an identifier as often as a keyword, and
# "for (x of /re/...)" is rarer than "of / 2")
_JS_KEYWORDS = ("return", "typeof", "case", "in", "delete", "void", "throw",
                "new", "else", "do", "instanceof", "yield", "await")
# before a "{" that opens an object literal (an expression: a "/" after its
# "}" is a division), not a block
_OBJ_AFTER = "(,=[?!&|+-*%<>~^"
# words after which a "{" opens an object literal (an expression)
_OBJ_WORDS = ("return", "typeof", "in", "of", "delete", "void", "throw", "new",
              "case", "instanceof", "yield", "await")
_PAREN_HEADS = ("if", "while", "for", "with")
_WORD = re.compile(r"[\w$\u0080-￿]+")
# what ends a JS line (and a // comment): not only \n
_JS_NL = "\n\r\u2028\u2029"
_JS_WS = " \t\f\v" + _JS_NL


class MinifyError(ValueError):
    pass


def _space(out, ws):
    """A run of white space: a newline when it holds one (none twice, none at
    the start, no space before it), else one space."""
    if any(c in ws for c in _JS_NL):
        while out and out[-1] == " ":
            out.pop()
        if out and out[-1] != "\n":
            out.append("\n")
    elif out and out[-1] not in (" ", "\n"):
        out.append(" ")


def _last(out, skip=0):
    """The last token written that is not white space, skip of them passed
    over first, or "" (no copy of out: at each "{" one made it quadratic)."""
    k = len(out) - 1
    while skip:
        while k >= 0 and out[k] in (" ", "\n"):
            k -= 1
        k -= 1
        skip -= 1
    while k >= 0 and out[k] in (" ", "\n"):
        k -= 1
    return out[k] if k >= 0 else ""


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
    regex_ok, parens, braces, word = True, [], [], ""
    tern, colon_tern = 0, False          # "?" not yet matched by ":"; the last
    terns = []                           # ":" a ternary's (its "{" an object);
                                         # the counts of the braces around
    while i < n:
        c = src[i]
        if c in _JS_WS:
            j = i
            while j < n and src[j] in _JS_WS:
                j += 1
            _space(out, src[i:j])
            i = j
        elif src.startswith("//", i):
            # (up to any line terminator: one ending only at \n dropped the
            # code after a \r or a U+2028 with the comment)
            j = i + 2
            while j < n and src[j] not in _JS_NL:
                j += 1
            i = j
        elif src.startswith("/*", i):
            j = src.find("*/", i + 2)
            if j < 0:
                raise MinifyError("a comment cut short: %r" % src[i:i + 60])
            # (one with a line terminator, a lone \r too, ends a statement
            # as the newline would)
            _space(out, src[i:j])
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
                dot = _last(out) == "."
                out.append(word)
                i = m.end()
                regex_ok = word in _JS_KEYWORDS and not dot
                if word == "of" and not dot:     # (a keyword only in for (x of)
                    regex_ok = bool(parens and parens[-1])
                if dot:
                    word = ""            # (x.if is no "if (" either)
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
            elif c == "{":
                prev = _last(out, 1)
                # an object literal: a "/" after its "}" divides; a block
                # (after ")", "else", "=>", "case x:", a label) does not
                if prev == ">" and _last(out, 2) == "=":
                    obj = False                  # an arrow function's body
                elif prev == ":":
                    obj = colon_tern or bool(braces and not braces[-1])
                else:
                    obj = bool(prev) and (prev in _OBJ_WORDS
                                          or prev[-1] in _OBJ_AFTER)
                braces.append(not obj)
                terns.append(tern)               # (an object's own ":" are no
                tern = 0                         # ternary's)
                regex_ok = True
            elif c == "}":
                regex_ok = braces.pop() if braces else True
                tern = terns.pop() if terns else 0
            elif c == ")":
                # "if (x) /re/.test(y)": a regex, "(a + b) / 2" a division
                regex_ok = parens.pop() if parens else False
            elif c == "]":
                regex_ok = False
            else:
                if c == "?" and not (src.startswith(".", i) and not src[i + 1:i + 2].isdigit()
                                     or src.startswith("?", i)):
                    tern += 1                    # (not "?." nor "??")
                elif c == ":":
                    colon_tern = tern > 0
                    if tern:
                        tern -= 1
                elif c == ";":
                    tern = 0
                regex_ok = True
            word = ""
    return "".join(out).strip()


def _css_word(c):
    """A character a CSS identifier or number goes on with."""
    return bool(c) and (c.isalnum() or c in "-_%#" or c > "\x7f")


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
            # (1px/**/solid are two tokens: a space; .a/**/.b is one compound
            # selector, a space made it a descendant one: none)
            if (out and _css_word(out[-1][-1:]) and i < n and _css_word(src[i])):
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


_COMMENT = re.compile(r"<!--.*?-->", re.S)
# markup this cannot strip: white space that shows (<pre>, <textarea>), and a
# <script> or <style> _BLOCK did not take (no end tag it reads)
_KEEP = re.compile(r"<(pre|textarea|script|style)\b", re.I)


def minify_html(src):
    """Markup (no <script> or <style>): no comments, indentation or blank lines."""
    src = _COMMENT.sub("", src)
    m = _KEEP.search(src)
    if m:
        raise MinifyError("markup web_min cannot strip: %r" % src[m.start():m.start() + 60])
    return re.sub(r"[ \t\r]*\n[ \t\r\n]*", "\n", src)


# an HTML comment or a whole <style>/<script> block, whichever starts first:
# a "<style>" named in a comment starts none, a "<!--" in a script's string
# is no comment
_BLOCK = re.compile(r"<!--.*?-->|(<(style|script)\b[^>]*>)(.*?)(</\2\s*>)",
                    re.S | re.I)


def minify(page):
    """The page's text, stripped; the same rendering, the same script."""
    out, markup, pos = [], [], 0
    for m in _BLOCK.finditer(page):
        markup.append(page[pos:m.start()])
        pos = m.end()
        if not m.group(1):
            continue                     # a comment: gone, the markup goes on
        fn = minify_js if m.group(2).lower() == "script" else minify_css
        out.append(minify_html("".join(markup)))
        markup = []
        out.append(m.group(1) + "\n" + fn(m.group(3)) + "\n" + m.group(4))
    markup.append(page[pos:])
    out.append(minify_html("".join(markup)))
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


def node_check(page):
    """Each stripped <script> through `node --check`, when node is there: a
    construct the token scan misreads (a division taken for a regex) fails
    the build, not the board's page (its script dead, "connecting..." for
    good). Without node: nothing (the suite's WEB-MIN-AST compares the
    trees)."""
    node = shutil.which("node")
    if not node:
        return
    for m in _BLOCK.finditer(page):
        if not m.group(1) or m.group(2).lower() != "script":
            continue
        fd, path = tempfile.mkstemp(suffix=".js")
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as f:
                f.write(m.group(3))
            r = subprocess.run([node, "--check", path], capture_output=True,
                               text=True, timeout=60)
        finally:
            os.remove(path)
        if r.returncode:
            lines = r.stderr.strip().splitlines()
            what = next((l for l in lines if re.match(r"\w*Error:", l)), "")
            at = next((l.rsplit(":", 1)[-1] for l in lines if l.startswith(path + ":")), "?")
            raise MinifyError("the stripped script does not parse (node --check), "
                              "line %s: %s" % (at, what or (lines or [""])[-1]))


_AST_JS = (
    "const a = require('internal/deps/acorn/acorn/dist/acorn');"
    "const fs = require('fs');"
    "const t = (p) => JSON.stringify(a.parse(fs.readFileSync(p, 'utf8'), {ecmaVersion: 'latest'}),"
    " (k, v) => k === 'start' || k === 'end' ? undefined : typeof v === 'bigint' ? String(v) : v);"
    "console.log(t(process.argv[1]) === t(process.argv[2]) ? 'same' : 'differ');")


def ast_check(src, page):
    """Each stripped <script> against its source in node's own acorn (the
    positions left out), when node and its acorn are there: a "/" taken
    for a regex that still parses (a regex's spaces folded, the same
    program no more) passes node --check and fails here. Without acorn:
    nothing (node_check). Raises MinifyError."""
    node = shutil.which("node")
    if not node:
        print("web_min: no node: the stripped script is not checked", file=sys.stderr)
        return
    sa = [m.group(3) for m in _BLOCK.finditer(src) if m.group(1) and m.group(2).lower() == "script"]
    sb = [m.group(3) for m in _BLOCK.finditer(page) if m.group(1) and m.group(2).lower() == "script"]
    if len(sa) != len(sb):               # (zip let a dropped or added one by)
        raise MinifyError("the stripped page has %d scripts, its source %d" % (len(sb), len(sa)))
    for a, b in zip(sa, sb):
        paths = []
        try:
            for t in (a, b):
                fd, path = tempfile.mkstemp(suffix=".js")
                with os.fdopen(fd, "w", encoding="utf-8") as f:
                    f.write(t)
                paths.append(path)
            r = subprocess.run([node, "--expose-internals", "-e", _AST_JS] + paths,
                               capture_output=True, text=True, timeout=120)
        finally:
            for path in paths:
                os.remove(path)
        if "Cannot find module" in r.stderr or "bad option" in r.stderr:
            print("web_min: this node has no acorn: only node --check", file=sys.stderr)
            return                       # (a node without its acorn to reach)
        if r.stdout.strip() != "same":
            raise MinifyError("the stripped script is not the same program as the source "
                              "(acorn): %s" % (r.stdout.strip() or r.stderr.strip()[-200:]))


def main(argv):
    if len(argv) not in (2, 3):
        print(__doc__, file=sys.stderr)
        return 2
    try:
        with open(argv[1], encoding="utf-8") as f:
            src = f.read()
        text = minify(src)
        node_check(text)
        ast_check(src, text)
        page = text.encode("utf-8")
        if len(argv) == 2:
            sys.stdout.buffer.write(page)
            return 0
        data = gz(page, wbits())
    except (OSError, UnicodeError, MinifyError, subprocess.SubprocessError) as e:
        print("%s: %s" % (argv[0], e), file=sys.stderr)
        return 1
    with open(argv[2], "wb") as f:
        f.write(data)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
