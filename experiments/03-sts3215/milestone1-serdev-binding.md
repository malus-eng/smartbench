# Milestone 1: serdev Driver Binding to UART3

## Goal
Bind a custom serdev kernel driver to the ROCK 3C's UART3 (RK3566 `serial@fe670000`),
so that the STS3215 servo's serial bus is controlled entirely in kernel space rather
than exposed to user space as `/dev/ttyS3`.

## Background: why serdev
serdev (serial device bus) is the Linux framework for "smart" devices hung off a UART
(GPS, Bluetooth, serial-bus servos, etc.). Instead of letting the UART surface as a
`/dev/ttySx` character device for user space, serdev lets a kernel driver take exclusive
ownership of the UART. This enables tighter timing control and lower latency — which is
exactly what half-duplex direction switching for the STS3215 needs.

## What was built
1. A serdev driver (`sts3215.c`) matching the device-tree `compatible = "malus,sts3215"`.
   - `probe()` calls `serdev_device_open()`, then configures 1 Mbps / 8N1.
   - A `receive_buf` callback is registered for asynchronous RX (servo replies).
2. A modification to the board device tree binding the servo node to UART3.

## Device tree change
The platform DTB is `rk3566-rock-3c.dtb`. UART3 (`serial@fe670000`) was disabled by
default. The node was modified:
- `status = "disabled"` -> `status = "okay"`
- Added a child node:
sts3215 {

compatible = "malus,sts3215";

};
The existing `pinctrl-0` (uart3m0 group = pins PIN_3 / PIN_5 on the 40-pin header) was
left untouched.

## Process
1. Backed up the original DTB (critical — a broken DTB can prevent boot; serial console
   is the recovery path).
2. Decompiled: `dtc -I dtb -O dts -o rock-3c.dts rock-3c.dtb`
3. Edited the `serial@fe670000` node as above.
4. Recompiled: `dtc -I dts -O dtb -o rock-3c-new.dtb rock-3c.dts`
5. Replaced the system DTB and rebooted.
6. Loaded the driver: `sudo insmod sts3215.ko`

## Key obstacle and how it was solved
The kernel (5.10.160-36-rk356x) does **not** enable `CONFIG_OF_OVERLAY`, so runtime
device-tree overlays via configfs (`/sys/kernel/config/device-tree/overlays/`) are not
available. The Radxa rsetup overlay mechanism also only manages vendor overlays and does
not cleanly accept a custom serdev binding. The solution was to modify the base board
device tree directly and recompile it — the most standard form of board-level DT work.

## Verification (success criteria all met)
- `ls /proc/device-tree/serial@fe670000/` shows the `sts3215` child node.
- `dmesg` after insmod:
sts3215: loading out-of-tree module taints kernel.

sts3215 serial1-0: sts3215: probe start

dw-apb-uart fe670000.serial: failed to request DMA, use interrupt mode

sts3215 serial1-0: sts3215: probe done, UART configured at 1Mbps
- The "failed to request DMA, use interrupt mode" line is benign: UART3 has no DMA
  channel configured, so it falls back to interrupt-driven transfer. For the servo's
  low data volume this is fine (confirmed earlier by a successful UART3 loopback test).

## Result
The serdev driver successfully takes ownership of UART3 in kernel space. `/dev/ttyS3` is
no longer exposed; the servo bus is now driven entirely by the kernel driver.

## Next: Milestone 2
Implement the Feetech protocol inside the driver (frame: 0xFF 0xFF ID LEN CMD PARAMS
CHECKSUM, checksum = ~(sum) & 0xFF), send via serdev_device_write, implement torque-enable
and goal-position, and expose a user-space interface — to make the servo physically move
under pure kernel-driver control.
ENDOFFILE
