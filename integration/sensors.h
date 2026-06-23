/* SPDX-License-Identifier: GPL-2.0 */
/*
 * sensors.h - hardware abstraction layer for the smart parcel locker
 *
 * Hides three heterogeneous kernel interfaces behind one uniform API so the
 * state machine in main.c never touches /dev or /sys directly:
 *
 *   HC-SR04 ultrasonic : read()  on /dev/hcsr04           -> distance (cm)
 *   HX711   load cell   : ioctl() on /dev/hx711           -> weight   (mg)
 *   STS3215 bus servo   : sysfs writes/reads under serial1-0
 *
 * All functions return 0 on success and a negative errno on failure, so the
 * caller can do uniform error handling regardless of the underlying transport.
 */
#ifndef LOCKER_SENSORS_H
#define LOCKER_SENSORS_H

/* Servo positions (0..4095 over ~360 deg). Tune to your mechanism. */
#define SERVO_POS_LOCKED    2048	/* door closed / latched */
#define SERVO_POS_OPEN      1024	/* door open             */

/* Open every device node and put the servo in a known (locked) state.
 * Returns 0 on success, negative errno on failure. */
int  sensor_init(void);

/* Release torque and close all file descriptors. Safe to call once. */
void sensor_cleanup(void);

/* Read one ultrasonic measurement.
 * On success writes the distance in centimetres to *cm and returns 0. */
int  distance_read_cm(int *cm);

/* Read the current weight from the load cell.
 * On success writes milligrams to *mg and returns 0. */
int  weight_read_mg(long *mg);

/* Tare the scale (define current reading as zero). Returns 0 / -errno. */
int  weight_tare(void);

/* Drive the door to the open / locked position. Returns 0 / -errno. */
int  lock_open(void);
int  lock_close(void);

/* Read back the servo's present position (0..4095). Returns 0 / -errno. */
int  servo_present_position(int *pos);

#endif /* LOCKER_SENSORS_H */
