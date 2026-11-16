#!/usr/bin/env python3
"""The firmware recipe in one place, for tools/build_firmware.sh and deploy.sh:
what the gps_frozen marker records of the frozen sources, which of them a
deploy finds stale, and the OTA manifest and the string its HMAC covers (the
board checks the same string: gps_logger._ota_signed).

  firmware.py marker              the marker's source lines after COMMIT/SHA/BUILD
  firmware.py stale SHA TFT BOOT CLOCK RX
                                  the first frozen source that is not this one
                                  ("-" or "": not recorded, an older build), or
                                  nothing; SHA is always compared
  firmware.py manifest DIR KEY_FILE BUILD COMMIT BOARD
                                  DIR/manifest.json for DIR's image and page
  firmware.py key KEY_FILE        exit 1 (a message, never the key) when the
                                  key is not one the board takes
  firmware.py old KEEP... < manifest.json
                                  the files the published manifest names that
                                  are not KEEP (plain names only), a line each

Run from the repository root (the scripts cd there)."""
import hashlib
import hmac
import json
import os
import sys

# the marker's fields: its name, the sources it hashes (concatenated in this
# order), what deploy.sh names when it differs. SHA is the logger itself.
MARKER = (
    ("SHA", ("gps_logger.py",), "gps_logger.py"),
    ("TFT", ("gps_tft.py", "gps_tft_font.py"), "gps_tft.py / gps_tft_font.py"),
    ("BOOT", ("gps_boot.py",), "gps_boot.py"),
    ("CLOCK", ("gps_clock.py",), "gps_clock.py"),
    ("RX", ("gps_receiver.py",), "gps_receiver.py"),
)


def digest(names, root="."):
    """SHA-256 (hex) of the files names, one after the other."""
    h = hashlib.sha256()
    for n in names:
        with open(os.path.join(root, n), "rb") as f:
            h.update(f.read())
    return h.hexdigest()


def marker_lines(root="."):
    """The marker's lines for every field but SHA (build_firmware.sh writes
    COMMIT, SHA and BUILD itself, first)."""
    return ['%s = "%s"' % (k, digest(names, root)) for k, names, _ in MARKER[1:]]


def stale(values, root="."):
    """The name of the first frozen source that differs from the one in root
    (values: the marker's, in MARKER's order), or None. A field that is not
    recorded ("-", "" or missing: an older build) is not compared, except
    SHA: a marker without it is stale."""
    for i, (_k, names, said) in enumerate(MARKER):
        v = values[i] if i < len(values) else "-"
        if i and v in ("-", ""):
            continue
        if v != digest(names, root):
            return said
    return None


def signed(m):
    """The string the manifest's HMAC covers, as gps_logger._ota_signed:
    build|commit|size|sha256|url, path|size|sha256|url for each file, then
    |board=<name>."""
    t = "%d|%s|%d|%s|%s" % (m["build"], m["commit"], m["size"], m["sha256"], m["url"])
    for f in m.get("files") or ():
        t += "|%s|%d|%s|%s" % (f["path"], f["size"], f["sha256"], f["url"])
    if m.get("board") is not None:
        t += "|board=" + m["board"]
    return t


def read_key(key_file):
    """The key as the manifest is signed with it: the file stripped. It must
    be one line of 16+ printable ASCII without quotes or spaces (the board
    reads it from .env): the shell's check dropped the newlines first, so a
    two-line file passed and signed with the newline in it (bad hmac on
    every board). ValueError otherwise (its text holds no key)."""
    with open(key_file, "rb") as f:
        key = f.read().strip()
    if len(key) < 16 or any(b < 33 or b > 126 or b in (34, 39) for b in key):
        raise ValueError("the OTA key in %s must be one line of 16+ printable "
                         "ASCII characters without quotes or spaces (e.g. "
                         "openssl rand -hex 32)" % key_file)
    return key


def old_files(live, keep):
    """The files the published manifest live (its JSON text) names that are
    not in keep: what publish removes once the new manifest is up (an older
    signed manifest and its image could be served again otherwise). Plain
    names only (no "/", no URL): nothing outside the folder."""
    try:
        m = json.loads(live)
        names = [m.get("url")] + [f.get("url") for f in m.get("files") or ()]
    except (ValueError, AttributeError, TypeError):
        return []
    out = []
    for n in names:
        if (isinstance(n, str) and n and n not in keep and n not in out
                and n not in (".", "..", "manifest.json")
                and all(c.isascii() and (c.isalnum() or c in "._-") for c in n)):
            out.append(n)
    return out


def manifest(d, key_file, build, commit, board):
    """d/manifest.json for d's micropython-<commit>.bin and web-<commit>.html.gz,
    signed with the key in key_file (the page only gzipped: an older firmware
    after a rollback takes this too)."""
    # what the board's _ota_manifest_ok takes, checked here: a manifest it
    # refuses ("bad manifest" on every board, for good) was signed, written
    # and published, and the publish removed the previous build's files
    c = commit[:-6] if commit.endswith("-dirty") else commit
    if not (0 < len(c) <= 20 and all(ch in "0123456789abcdef" for ch in c)):
        raise ValueError("commit %r: the board takes 1-20 lowercase hex"
                         " (git's short hash), maybe -dirty" % commit)
    own = None
    try:                                 # the firmware's own OTA_BOARD
        with open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               "..", "gps_logger.py")) as f:
            for ln in f:
                if ln.startswith('OTA_BOARD = "'):
                    own = ln.split('"')[1]
                    break
    except OSError:
        pass
    if not board or "|" in board or (own is not None and board != own):
        raise ValueError("board %r: not the firmware's OTA_BOARD %r (its"
                         " boards would refuse the manifest)" % (board, own))
    key = read_key(key_file)

    def meta(name):
        with open(os.path.join(d, name), "rb") as f:
            b = f.read()
        return len(b), hashlib.sha256(b).hexdigest()

    binf = "micropython-%s.bin" % commit
    size, sha = meta(binf)
    m = {"build": build, "commit": commit, "size": size, "sha256": sha,
         "url": binf, "files": []}
    for path, name in (("/web.html.gz", "web-%s.html.gz" % commit),):
        n, h = meta(name)
        m["files"].append({"path": path, "size": n, "sha256": h, "url": name})
    m["board"] = board
    m["hmac"] = hmac.new(key, signed(m).encode(), hashlib.sha256).hexdigest()
    with open(os.path.join(d, "manifest.json"), "w") as f:
        json.dump(m, f)
    return m


def main(a):
    if a[:1] == ["marker"]:
        print("\n".join(marker_lines()))
    elif a[:1] == ["stale"]:
        s = stale(a[1:])
        if s:
            print(s)
    elif a[:1] == ["manifest"] and len(a) == 6:
        try:
            m = manifest(a[1], a[2], int(a[3]), a[4], a[5])
        except ValueError as e:
            sys.stderr.write("%s\n" % e)
            return 1
        print("manifest: build", m["build"], "commit", m["commit"], "size", m["size"])
    elif a[:1] == ["key"] and len(a) == 2:
        try:
            read_key(a[1])
        except (OSError, ValueError) as e:
            sys.stderr.write("%s\n" % (e if isinstance(e, ValueError)
                                       else "cannot read %s" % a[1]))
            return 1
    elif a[:1] == ["old"]:
        for n in old_files(sys.stdin.read(), a[1:]):
            print(n)
    else:
        sys.stderr.write(__doc__ or "")
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
