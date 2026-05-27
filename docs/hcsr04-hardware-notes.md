# HC-SR04 Hardware Integration Notes

## Discovery 1: 3.3V supply works fine

Despite the datasheet specifying 5V, the HC-SR04 works correctly with 3.3V
supply from ROCK 3C PIN_1. This eliminates the need for level-shifting on
the ECHO line and avoids triggering the 5V protection circuit on the SBC.

Effective measurement range reduces from ~4m to ~2-3m, which is still well
above the requirement for a 75cm enclosure.

## Discovery 2: gpioset hold-behavior masks the trigger

`sudo gpioset gpiochip3 N=1` does not produce a pulse — it sets and holds
the line high until the command exits. HC-SR04 triggers on the falling
edge of a positive pulse, so a sustained high produces no measurement.

Sequencing two commands `gpioset ... =1; gpioset ... =0` produces a real
pulse, but the pulse width is determined by shell scheduling (1-100 ms),
far above the 10 us specified by the datasheet. The module tolerates this
because the trigger is edge-sensitive, but the timing jitter motivates
the kernel-driver implementation: only kernel-space udelay() or hrtimer
can produce a deterministic 10 us pulse.
