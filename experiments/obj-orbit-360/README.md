# 360° orbits: how object size and material change what the radar sees

**Status:** Data partly recorded (2026-10-29); analysis pending

## Question
For a car, a small metal object, a tree or sculpture and a building wall: how much of each object does the radar capture, compared with the 128-beam LiDAR?

## Method
Orbit each object once (radius 2 to 5 m, 0.5 to 1 m/s, start and stop at a marked spot), camera pointed at the object. Poses from LiDAR SLAM; radar points accumulated in the object frame.

## Data
Recorded 2026-10-29 (see [session log](../../notes/sessions/2026-10-29.md)): car = `2026-10-29.15-49-40` 11-17 min; sculpture (thin metal pole) = `2026-10-29.16-12-38` 5:30-9:40; small metal (Scotty) = `2026-10-29.16-22-36` 2:59-3:45. Still to record: tree, GHC wall, spin in place.

## Results
| Object | Size | Material | Radar points per frame on object | Extent error vs LiDAR | Mean echo (dB) | Visible from all sides? |
|---|---|---|---|---|---|---|
| Parked car | ~4.5 m | metal | TBD | TBD | TBD | TBD |
| Trash can / hydrant / bike rack | ~1 m | metal | TBD | TBD | TBD | TBD |
| Tree or sculpture | ~1 to 3 m | non-metal | TBD | TBD | TBD | TBD |
| GHC wall or corner | > 10 m | concrete / glass | TBD | TBD | TBD | TBD |

![Pending: one panel per object: accumulated radar points (orange) over the LiDAR cloud (grey), top view](../_assets/placeholder.svg)

*Pending: one panel per object: accumulated radar points (orange) over the LiDAR cloud (grey), top view. Source: TBD.*
![Pending: radar echo strength vs viewing angle around each object (polar plot); flat walls should show narrow specular spikes](../_assets/placeholder.svg)

*Pending: radar echo strength vs viewing angle around each object (polar plot); flat walls should show narrow specular spikes. Source: TBD.*
![Pending: 3D viewer screenshot of the car orbit](../_assets/placeholder.svg)

*Pending: 3D viewer screenshot of the car orbit. Source: TBD.*
![Pending: map of the orbit paths with LiDAR-SLAM poses](../_assets/placeholder.svg)

*Pending: map of the orbit paths with LiDAR-SLAM poses. Source: TBD.*


## Conclusion
Pending. Expected: the car is bright from every side, the small metal object is smeared by the ~14° beam, the tree is weak, the wall only lights up when faced squarely.

## Open items
- Poses: run red-rover's LiDAR SLAM or an alternative
- Decide how to segment each object in the LiDAR cloud

## B-roll to capture (for the 5-minute video)
- [ ] Phone video of each orbit from a second person
- [ ] Photo of each object
- [ ] Viewer orbit video of the accumulated car point cloud

## Reproduce
TBD in `src/ext/360/`.

[Back to all experiments](../../EXPERIMENTS.md)
