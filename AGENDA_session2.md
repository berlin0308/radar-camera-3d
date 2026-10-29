# Session 2 agenda · campus (capstone data first)

About 3 hours. The CP range/angle tests (M1) were done on 2026-10-21 and are not repeated.
Every segment should have people or cars in view; the capstone uses them to test detection.

## Before leaving
- [ ] Battery charged, backpack (Tianshu's charger may still be in Deva's lab)
- [ ] LiDAR: the rig now carries the OS-0-128 (swapped 2026-10-29); the NUC config already has
      `beams: 128` and `addr: 169.254.149.190`
- [ ] iPhone charged (it is the Mac's internet) and a USB cable
- [ ] Route: which objects to orbit, an open spot for the spin, 2-3 places with people and cars

## +0:00 Start-up (20 min)
1. Mac: iPhone hotspot over USB; network service order with iPhone USB above Wi-Fi; Wi-Fi on `red-rover`
2. Open the shared ssh connection (type the rover password once)
3. NUC: `cd ~/red-rover/collect && export ROVER_CFG=config/radarhd-o-polin.yaml && sudo -E make config && make`
4. When the web page shows `lidar Initialized`, wait another 30 s, then record a 30 s dry run and
   check that all four sensors wrote data (LiDAR shape `[128, 2048]`)

## +0:20 Shooting (each segment <= 5 min, about 1 GB/min)
| # | What | How | Segments |
|---|---|---|---|
| 1 | 360° orbit of a parked car | radius 4-5 m, camera on the car, 0.5-1 m/s, start and stop at a marked spot | 1-2 |
| 2 | 360° orbit of a small metal object (trash can, hydrant, bike rack) | radius 2-3 m | 1 |
| 3 | 360° orbit of a tree or sculpture (non-metal) | radius 3-5 m | 1 |
| 4 | One stretch of a GHC wall or corner | walk along at 5-10 m | 1 |
| 5 | Spin in place | stand still in an open spot, one turn per ~1 min | 2-3 |
| 6 | Campus scenes with people and cars | crossings, parking-lot entrances, busy walkways; rig static or handheld | 4-6 × 2-3 min |
| 7 | If time: X4 multipath | rig parallel to a side wall at 1.5 m | 3 × 30 s |
| 8 | If time: open sky (X10) | rig tilted ~45° at empty sky, 60 s | 1 |

- Walking segments: START/STOP from the phone at `http://192.168.8.249:5000`, save location = the SSD
- Static segments: `./shoot.sh <label> <seconds>` (logs to `notes/shots.csv`)
- After each walking segment, note the time and what was shot

Why orbits of different objects instead of the whole building: the radar reaches ~20 m and large flat
walls act as mirrors, so a whole-building loop shows scattered corners; objects of different size and
material (car, small metal, tree, wall) give a clean comparison against the 128-beam LiDAR.

## Shut-down (20 min)
- Stop any recording, then `sync && sudo shutdown now` on the NUC; wait for the white light to go off
- Unplug the SSD, power, battery; put the radar cover back
- On the Mac the SSD shows "not readable": press **Ignore** (never Initialize); mount read-only with
  `sudo anylinuxfs mount /dev/diskNs1 -o ro`, copy only the `2026-*` folders, run `./validate.sh`

## Good to know
- Speeds beyond ±1.79 m/s (about 4 mph) wrap around in Doppler; range and angle stay correct
- Camera frames show faces and plates: anonymize (red-rover `anonymize`) before publishing

---

# Session 3 · off campus (about 3 h)
- Places unlike the GRT training data (Oakland streets, Squirrel Hill, Schenley Park, downtown),
  crossings and sidewalks with people and cars, so the capstone can be tested for overfitting
- If it rains or is foggy that day, record rain / fog for capstone validation; mud and water only on a
  plastic film in front of the radome (ask Tianshu first)
- Start-up and shut-down as in Session 2
