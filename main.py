from machine import Pin
import machine

# Hold GPIO25 at reset to skip the logger (REPL + mpremote, no WDT).
if Pin(25, Pin.IN, Pin.PULL_UP).value():
  try:
    import gps_logger
    gps_logger.run()
  except KeyboardInterrupt:
    raise
  except Exception:
    machine.reset()
