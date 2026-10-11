# MicroPython 1.29 unix-port checks. No machine/network imports.
# Run: /path/to/micropython tests/mpy_unix_check.py
# Helpers are extracted from gps_logger.py so this is not a stale copy.

print("version", __import__("sys").version)
print("isalnum", hasattr("a", "isalnum"))
print("isalpha", hasattr("a", "isalpha"))
print("isdigit", hasattr("a", "isdigit"))
print("isspace", hasattr("a", "isspace"))

if hasattr("a", "isalnum"):
    raise SystemExit("unexpected isalnum on this build")

here = __file__
i = here.rfind("/")
script_dir = here[:i] if i >= 0 else "."
i = script_dir.rfind("/")
# Relative `tests/mpy_unix_check.py` has no slash in the parent; repo root is "."
root = script_dir[:i] if i >= 0 else "."
src_path = root + "/gps_logger.py"
f = open(src_path)
try:
    src = f.read()
finally:
    f.close()

start = src.find("def _ok_char")
end = src.find("\ndef _load_nets")
if start < 0 or end < 0:
    raise SystemExit("could not extract _ok_char/_safe_name from gps_logger.py")
if ".isalnum(" in src[start:end]:
    raise SystemExit("extracted helpers still call isalnum")

ns = {"ID_MAX": 32}
exec(src[start:end], ns)
_ok_char = ns["_ok_char"]
_safe_name = ns["_safe_name"]
print("extracted from", src_path)

got = _safe_name("Dawarich-GPS-tracker", "x")
if got != "Dawarich-GPS-tracker":
    raise SystemExit("safe_name default id failed: %r" % got)
if _safe_name('bad"id', "Dawarich-GPS-tracker") != "Dawarich-GPS-tracker":
    raise SystemExit("safe_name should reject quotes")
if _safe_name("bad_host", "Dawarich-GPS-tracker", True) != "Dawarich-GPS-tracker":
    raise SystemExit("safe_name host should reject underscore")
if not _ok_char("A", "") or _ok_char("_", ""):
    raise SystemExit("ok_char ranges failed")
print("safe_name ok")
print("mpy unix check passed")
