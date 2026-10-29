# M2: own range and Doppler processing

**Status:** Planned (due 11/01)

## Question
Does my own implementation reproduce the reference numbers?

## Method
IIQQ unpacking, range FFT, Doppler FFT written from scratch; compared frame by frame with `xwr.rsp` and with the M1 numbers.

## Data
M1 range shots and `scene_room` (2026-10-21).

## Results
| Check | Reference | Mine | Pass |
|---|---|---|---|
| Range fit slope | 0.993 | TBD | ±0.005 |
| Range offset | 0.183 m | TBD | ±1 cm |
| Peak within ±1 bin of tape (after offset) | 4/4 | TBD | 4/4 |
| Range-Doppler map vs `xwr.rsp` (relative error) | — | TBD | < 1e-6 |
| Static energy in the zero-Doppler bin | > 70 dB above the rest | TBD | yes |

![Pending: my range profile overlaid on the reference for the 12 ft shot](../_assets/placeholder.svg)

*Pending: my range profile overlaid on the reference for the 12 ft shot. Source: TBD.*
![Pending: my range-Doppler map vs reference, and their difference](../_assets/placeholder.svg)

*Pending: my range-Doppler map vs reference, and their difference. Source: TBD.*
![Reference: raw ADC of one chirp and its FFT](../../notes/fig/rd2/f04_adc.png)

*Reference: raw ADC of one chirp and its FFT*
![Reference: range-Doppler maps](../../notes/fig/rd2/f06_range_doppler.png)

*Reference: range-Doppler maps*


## Conclusion
Pending.

## Open items
- Write `src/radar/adc.py`, `range_doppler.py` and tests against `xwr.rsp`

## B-roll to capture (for the 5-minute video)
- [ ] Screen recording: notebook running the range FFT on one frame

## Reproduce
TBD in `src/radar/`.

[Back to all experiments](../../EXPERIMENTS.md)
