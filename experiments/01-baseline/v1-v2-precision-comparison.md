# v1 vs v2 Precision/Accuracy Comparison (equal sample size)

## Method
- Phone fixed at one position, measured 20 times with each driver
- v1: polling, v2: IRQ-based
- Phone NOT moved between the two runs
- True distance by tape measure: 16.5 cm

## Results

v1 (polling), 20 reads: all 17 cm. StdDev = 0.
v2 (IRQ), 20 reads: mix of 15 and 16 cm. Mean ~15.75, StdDev ~0.5.

## Interpretation

True distance: 16.5 cm.
v1 mean = 17.0 (bias +0.5), v2 mean ~15.75 (bias -0.75).

Neither is clearly more accurate; both sit within ~1 cm of truth, and the
absolute error is comparable. The meaningful differences are:

1. v1's zero variance is a quantisation artefact. The polling loop's fixed
   iteration period bins the pulse width coarsely, so it always reports 17.
   This is precision-by-quantisation, not true precision.

2. v2 resolves sub-centimetre variation via ns-resolution ISR timestamps,
   appearing as 15/16 jitter at the quantisation boundary.

3. Absolute accuracy (~+/-1 cm) is bounded by HC-SR04 hardware (+/-0.3 cm
   typical) and the temperature sensitivity of the speed of sound (the
   driver uses the 20C constant 58 us/cm). Software cannot beat this limit.

Conclusion: the polling-vs-IRQ difference manifests in CPU utilisation and
resolution, NOT in absolute measurement accuracy, which is hardware-limited.

## Honesty note
An earlier chart implied "kernel = higher accuracy" using mismatched sample
sizes. With equal n=20 samples and a tape-measured true value, the real
story is the precision/accuracy tradeoff above, not a simple accuracy
ranking.
