# Measurement: Kernel-space driver v1 (polling)

## Setup
- Driver: `hcsr04.c` v1 — polling-based, udelay(10) trigger pulse
- Same hardware as baseline (HC-SR04 on PIN_11/PIN_13, 3.3V supply)
- Target: iQOO Neo 7 SE smartphone (164.55 mm length)
- Method: `cat /dev/hcsr04` repeated by hand

## Results

### Phase 1 — phone as length reference
| # | Reading (cm) |
|---|-------------:|
| 1 | 16 |
| 2 | 16 |
| 3 | 16 |
| 4 | 15 |
| 5 | 16 |
| 6 | 16 |
| 7 | 16 |
| 8 | 16 |

Mean: 15.875 cm, StdDev: ~0.35 cm
True length: 16.455 cm, Error: 3.5%

### Phase 2 — distance halved
| # | Reading (cm) |
|---|-------------:|
| 1 | 7 |
| 2 | 7 |
| 3 | 8 |
| 4 | 8 |

Mean: 7.5 cm, StdDev: ~0.58 cm
Expected: ~8 cm

## Comparison with user-space baseline

| Metric                       | User-space gpiomon | Kernel v1 polling |
|------------------------------|--------------------|-----:|
| Phone length, 3 trials (cm)  | 15.9 / 16.8 / 15.8 | 16 / 16 / 16 |
| Outliers                     | 1 of 5 (172 cm)    | 0 of 8 |
| Trigger pulse width          | 1-100 ms (shell)   | 10 us (udelay) |
| Measurement code locale      | user space (gpiomon) | kernel space |

The kernel driver eliminates the outliers seen in user-space measurement
and produces tighter clustering, confirming that deterministic trigger
timing matters even on a tolerant module.

## Known limitations of v1
- `.read` busy-polls the ECHO line, occupying one CPU core for the full
  measurement duration (worst case 60 ms timeout window).
- ECHO edge timestamp resolution is limited by the polling loop period
  (each iteration calls `gpiod_get_value()` + `ktime_get()`, probably
  several microseconds per iteration).

These motivate v2: IRQ-based ECHO capture.
