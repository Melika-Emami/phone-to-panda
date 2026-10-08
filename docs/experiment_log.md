# Learning milestones

These are **user-reported observations from the original interactive notebook**, not measurements produced by the packaged project.

| Milestone | Observation |
|---|---|
| Cylindrical sliding pusher | Failed; distance 8.8 cm, sideways offset inferred from x/y distance |
| Flat sliding pusher | 0.3 cm final distance; position success |
| Six sliding-pusher start cases | 3/6 position successes; sideways cases failed |
| Phone tracking | 314 decoded frames; 100% coverage; user visually verified marker identity |
| Phone-driven sliding pusher | 0.33 cm final distance; position success |
| Panda pose control without compensation | 8.04 mm endpoint error |
| Panda with compensation activated correctly | 0.12 mm endpoint error |
| Panda demonstration replay with orientation control | Maximum sampled tracking error 1.12 mm, final error 0.02 mm, line deviation 0.03 mm, orientation error 0.003 degrees |
| Initial contact scene | Finger overlap around 12 mm; corrected placement using full finger collision bounds |
| Panda pushing with recorded timing | Robot contacted block; travel 4.51 cm; final distance 0.49 cm; initial position success rule met |

The transition to the packaged notebook pins model/version dependencies and adds stricter physical success checks. Rerun and report the new metrics; do not assume exact equality with earlier observations.
