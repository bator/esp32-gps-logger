# GPS logger

A battery-carried tracker that reads a GPS receiver, writes the track to flash, and uploads it to Dawarich over WiFi.

## Language

### Positions

**Epoch**:
One position reading from the receiver. The receiver gives up to GPS_HZ epochs in a UTC second.
_Avoid_: sample, reading

**Second**:
The mean of the epochs of one UTC second. It is the unit the logger records: one point a second.
_Avoid_: tick, interval

**Fix**:
A published Second handed to the log: position, accuracy and UTC time, immutable.
_Avoid_: point, sample (a point is what the track stores)

### Time

**Clock**:
The answer to "what is the UTC time now", built from GPS time, NTP, or the uptime since the last anchor, in that order of trust.
_Avoid_: RTC (that is the hardware register the Clock may set)

**Anchor**:
A pair of a UTC time and the Stamp it was true at. It ties the Clock to the uptime.
_Avoid_: clock stamp, sync point

**Stamp**:
A moment kept in whole seconds of uptime, for anything that lives longer than a minute. It does not wrap.
_Avoid_: timestamp, ticks

**Mark**:
A moment kept in milliseconds of the tick counter, for anything that lives under a minute. It wraps after about 12 days, so it is never kept longer.
_Avoid_: stamp (a Stamp is the long-lived kind), timestamp

### Hardware roles

**Receiver**:
The GPS chip and the code that turns its bytes into Fixes.
_Avoid_: GPS module, gps (when the chip is meant)
