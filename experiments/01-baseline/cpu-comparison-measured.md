# CPU Utilisation: v1 Polling vs v2 IRQ (Measured)

## Method
- `top -d 1` observing %Cpu, while `while true; do cat /dev/hcsr04; done`
- Target: phone at ~16 cm (short ECHO pulse, ~1 ms)
- 3 runs each scenario

## Results

| Scenario        | Run 1 | Run 2 | Run 3 | Mean |
|-----------------|------:|------:|------:|-----:|
| System idle     | 6     | 14    | 2     | ~7%  |
| v1 (polling)    | 27    | 38    | 21    | ~29% |
| v2 (IRQ)        | 24    | 26    | 24    | ~25% |

## Analysis

The difference between v1 and v2 (~4 percentage points) is far smaller
than the theoretical expectation that busy-wait polling saturates a core.

Reason: the test harness (`bash` fork + `exec cat` per iteration, plus
tmux output forwarding) dominates CPU cost. The actual busy-wait window
in v1 is only as long as the ECHO pulse — about 1 ms for a 16 cm target —
so polling overhead is a small fraction of each loop iteration.

v2 (IRQ) is consistently lower and far more stable (StdDev ~1% vs v1's
~9%), reflecting that the sleeping reader does not compete for CPU.

## Limitation and follow-up

This short-range test under-represents v1's worst case. With a distant or
absent target, v1 busy-waits up to the 60 ms timeout, which would saturate
a core, whereas v2 sleeps. A follow-up experiment aiming the sensor at open
space (forcing long ECHO / timeout) would expose the full polling penalty.

## Honesty note

An earlier draft of the comparison chart assumed v1 = 100% CPU. That figure
was never measured and has been replaced with the data above.
