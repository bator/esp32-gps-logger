# MicroPython 1.29 unix-port checks. No machine/network imports.
# Run: /path/to/micropython tests/mpy_unix_check.py

print("version", __import__("sys").version)
print("isalnum", hasattr("a", "isalnum"))
print("isalpha", hasattr("a", "isalpha"))
print("isdigit", hasattr("a", "isdigit"))
print("isspace", hasattr("a", "isspace"))

if hasattr("a", "isalnum"):
    raise SystemExit("unexpected isalnum on this build")

ID_MAX = 32

def _ok_char(c, extra):
    return (("a" <= c <= "z") or ("A" <= c <= "Z") or ("0" <= c <= "9")
            or c in extra)

def _safe_name(s, default, host=False):
    if not s:
        return default
    if len(s) < 1 or len(s) > ID_MAX:
        return default
    extra = "-" if host else "._-"
    for c in s:
        if _ok_char(c, extra):
            continue
        return default
    return s

got = _safe_name("Dawarich-GPS-tracker", "x")
if got != "Dawarich-GPS-tracker":
    raise SystemExit("safe_name default id failed: %r" % got)
if _safe_name('bad"id', "Dawarich-GPS-tracker") != "Dawarich-GPS-tracker":
    raise SystemExit("safe_name should reject quotes")
if _safe_name("bad_host", "Dawarich-GPS-tracker", True) != "Dawarich-GPS-tracker":
    raise SystemExit("safe_name host should reject underscore")
print("safe_name ok")
print("mpy unix check passed")
