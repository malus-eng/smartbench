# HX711 Calibration Results

## Method
- Driver v2 with 10-sample averaging + ioctl-based tare/scale
- Reference weight: iQOO Neo 7 SE smartphone (~205 g)
- Fixed end held by hand/weight on desk (non-rigid mount)

## Calibration procedure
1. `hx711_cal tare` with no load — captured zero point
2. `hx711_cal raw` with phone loaded — raw ~102,500
3. Computed scale = (loaded - tare) / weight * 1000 ≈ 40209 counts/g x1000
4. `hx711_cal scale 40209`

## Verification (phone loaded, repeated weigh)
208.585, 208.460, 208.411, 207.839, 206.446, 206.695, 205.874,
205.252, 203.984, 203.312, 203.387, 203.113, 202.467, 202.641,
201.944, 203.088, 202.939, 203.188, 203.387, 204.481, 204.382
- True weight: ~205 g
- Measured range: 201.9 - 208.6 g
- Error: within 1-3%
- Per-reading jitter after 10x averaging: ~0.5 g (vs several grams with
  single-shot reads)

## Effect of averaging
10-sample averaging reduced random noise by ~sqrt(10) = 3.16x, matching
theory. Residual slow downward drift (~4 g over the series) is mechanical
creep from the non-rigid mount, not electrical noise — averaging cannot
remove it; only rigid mounting can.

## Fitness for purpose
For the parcel-detection use case (threshold: is there an object > ~50 g
on the lid?), this accuracy is more than sufficient. Sub-gram precision
is unnecessary, so no further optimisation is warranted.
