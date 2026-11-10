# Long-lived moments are Stamps in uptime seconds; short-lived ones stay Marks in tick milliseconds

`time.ticks_ms` wraps at 2**30 ms (12.4 days) and reads wrongly from 2**29 ms (6.2 days), and every stamp that outlived that read "young" again: five wrap fixes between e3c107a and dc9af7a, each adding a special case beside the last, and about 45 stamps still unprotected. Anything that can live longer than a minute is therefore a **Stamp**, whole seconds of uptime counted by the Clock, which does not wrap in 34 years and stays a small int. Anything under a minute stays a **Mark** in tick milliseconds, because those need millisecond precision and cannot wrap before they are replaced.

## Considered Options

- **Keep ms ticks and sweep.** One registry in the Clock rebases every long-lived stamp (what `_stamps_old` and `_clock_on` do for part of them today). Rejected: the sweep has to know every holder, and each new stamp that is forgotten is a new wrap bug.
- **One unbounded ms counter.** Rejected: past 2**30 it is a heap long int, allocated on every stamp and every loop pass on a RAM-starved board.

## Consequences

- Two kinds of moment exist in the code, with different clocks and different ages (`age(now, mark)` and `age_s(stamp)`); a Mark kept longer than a minute is a bug.
- A `None` Stamp means never: `age_s(None)` is a large constant and `recent_s(None, s)` is false.
- `_stamps_old` and the rebasing in `_loop` go away once the long-lived holders have moved over.
