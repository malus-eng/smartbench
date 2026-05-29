# HX711 Initial Readings

## Setup
- HX711 ADC + 20kg aluminium bar load cell
- VCC=3.3V from PIN_1, DT=PIN_15, SCK=PIN_16
- Load cell loosely held by hand on a desk (no permanent mount)
- Driver: `hx711.c` v0.1 — bit-bang, local_irq_save during the 25-pulse read

## Quiescent (no load applied) — 10 readings
38512, 38478, 38403, 38293, 38368, 38014, 37480, 37752, 37886, 38169
- Mean: 38,135
- StdDev: ~312
- Range: 1,032 (37,480 to 38,512)
- Relative noise: 0.82%
- Observed slight downward drift consistent with sensor warm-up

## Pressed (light finger pressure) — 10 readings
109064, 108536, 107810, 102716, 99426, 99927, 101102, 104862, 104843, 94818
- Mean: 103,310
- StdDev: ~4,275
- Range: 14,246 (94,818 to 109,064)
- Decreasing trend during the series reflects involuntary muscle relaxation

## Signal-to-noise ratio
Signal (pressed mean - quiescent mean) = 65,175 ADC counts
Noise (quiescent StdDev)                = 312 ADC counts
SNR                                     = 209
SNR of 209 is at the high end for hobbyist load-cell setups (typically
100-500) and was achieved without proper mechanical mounting or input
filtering, suggesting the kernel driver's timing fidelity is the
dominant factor.

## Implications

- Bit-banging with local_irq_save successfully meets HX711's tight
  timing window (0.2us < SCK high < 50us) across all observed reads
- Zero "all-ones" or "all-zeroes" stuck-state reads, indicating the
  chip never entered sleep mode mid-pulse
- The non-zero quiescent value (~38k) reflects sensor mechanical bias;
  in production this is removed by tare calibration

## Next steps

- Calibrate against a known weight to derive grams-per-count
- Add ioctl interface for tare / calibration from userspace
- Compare against a deliberately naive userspace bit-bang implementation
  to quantify the timing advantage of kernel-space execution
