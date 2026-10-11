#!/usr/bin/env python3
"""Optional on-device timing probes. Does not flash firmware.

Run against a board that already has the logger stopped (GPIO25 boot-bypass):

    .venv/bin/mpremote run tests/device_bench.py

Measures count_points() on whatever is already in /gpx (or a 1 MB synthetic
file if you create one). Safe to skip; CI uses the CPython harness instead.
"""
import sys
import time

try:
    import gps_logger as gl
except ImportError:
    print("import gps_logger failed (need GPIO25 bypass so main.py did not run)")
    sys.exit(1)

t0 = time.ticks_ms()
n = gl.count_points()
dt = time.ticks_diff(time.ticks_ms(), t0)
print("count_points", n, "ms", dt)
print("wdt timeout", gl.WDT_TIMEOUT)
print("connect timeout", gl.CONNECT_TIMEOUT)
