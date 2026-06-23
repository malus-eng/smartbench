// SPDX-License-Identifier: GPL-2.0
/*
 * sensors.c - hardware abstraction layer implementation
 *
 * One file, three transports:
 *   distance -> read()  on /dev/hcsr04   (text integer, centimetres)
 *   weight   -> ioctl() on /dev/hx711    (HX711_IOC_READ_MG, milligrams)
 *   servo    -> sysfs   under the sts3215 driver dir (goal_position, etc.)
 *
 * The servo sysfs directory name (e.g. "serial1-0") is discovered at runtime
 * rather than hard-coded, so a different bind order still works.
 */
#include "sensors.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <dirent.h>
#include <sys/ioctl.h>
#include <sys/types.h>

/* ---- device paths ---- */
#define HCSR04_DEV   "/dev/hcsr04"
#define HX711_DEV    "/dev/hx711"
#define STS_DRV_DIR  "/sys/bus/serial/drivers/sts3215"

/* ---- HX711 ioctl interface (must match the driver's hx711_ioctl.h) ---- */
#define HX711_IOC_MAGIC     'h'
#define HX711_IOC_TARE      _IO(HX711_IOC_MAGIC, 1)
#define HX711_IOC_SET_SCALE _IOW(HX711_IOC_MAGIC, 2, int)
#define HX711_IOC_READ_MG   _IOR(HX711_IOC_MAGIC, 3, int)

/* module state */
static int  hx711_fd = -1;
static char servo_dir[256];	/* absolute path of the servo sysfs dir */

/* ---------- small sysfs helpers ---------- */

/* Write a decimal integer to servo_dir/attr. Returns 0 / -errno. */
static int sysfs_write_int(const char *attr, long val)
{
	char path[320];
	char buf[32];
	int fd, len, ret = 0;

	snprintf(path, sizeof(path), "%s/%s", servo_dir, attr);
	fd = open(path, O_WRONLY);
	if (fd < 0)
		return -errno;

	len = snprintf(buf, sizeof(buf), "%ld\n", val);
	if (write(fd, buf, len) < 0)
		ret = -errno;

	close(fd);
	return ret;
}

/* Read a decimal integer from servo_dir/attr. Returns 0 / -errno. */
static int sysfs_read_int(const char *attr, int *out)
{
	char path[320];
	char buf[64];
	int fd;
	ssize_t n;

	snprintf(path, sizeof(path), "%s/%s", servo_dir, attr);
	fd = open(path, O_RDONLY);
	if (fd < 0)
		return -errno;

	n = read(fd, buf, sizeof(buf) - 1);
	close(fd);
	if (n < 0)
		return -errno;

	buf[n] = '\0';
	*out = atoi(buf);
	return 0;
}

/* Find the servo's sysfs device dir (the one that contains goal_position). */
static int discover_servo_dir(void)
{
	DIR *d;
	struct dirent *e;
	char candidate[320];
	char attr[384];

	d = opendir(STS_DRV_DIR);
	if (!d)
		return -errno;

	while ((e = readdir(d))) {
		if (e->d_name[0] == '.')
			continue;
		/* device symlinks look like "serial1-0"; skip module/bind/etc. */
		snprintf(candidate, sizeof(candidate), "%s/%s",
			 STS_DRV_DIR, e->d_name);
		snprintf(attr, sizeof(attr), "%s/goal_position", candidate);
		if (access(attr, F_OK) == 0) {
			strncpy(servo_dir, candidate, sizeof(servo_dir) - 1);
			servo_dir[sizeof(servo_dir) - 1] = '\0';
			closedir(d);
			return 0;
		}
	}
	closedir(d);
	return -ENODEV;
}

/* ---------- public API ---------- */

int sensor_init(void)
{
	int ret;

	hx711_fd = open(HX711_DEV, O_RDWR);
	if (hx711_fd < 0) {
		fprintf(stderr, "open %s: %s\n", HX711_DEV, strerror(errno));
		return -errno;
	}

	ret = discover_servo_dir();
	if (ret) {
		fprintf(stderr, "servo sysfs dir not found under %s\n", STS_DRV_DIR);
		close(hx711_fd);
		hx711_fd = -1;
		return ret;
	}

	/* probe the ultrasonic node once so we fail early if it's missing */
	if (access(HCSR04_DEV, R_OK) != 0) {
		fprintf(stderr, "access %s: %s\n", HCSR04_DEV, strerror(errno));
		close(hx711_fd);
		hx711_fd = -1;
		return -errno;
	}

	/* put the door in a known state: torque on, latched */
	ret = sysfs_write_int("torque_enable", 1);
	if (ret)
		return ret;
	return sysfs_write_int("goal_position", SERVO_POS_LOCKED);
}

void sensor_cleanup(void)
{
	if (servo_dir[0])
		sysfs_write_int("torque_enable", 0);	/* release torque */
	if (hx711_fd >= 0) {
		close(hx711_fd);
		hx711_fd = -1;
	}
}

int distance_read_cm(int *cm)
{
	char buf[32];
	int fd;
	ssize_t n;

	/* open per measurement: each open+read triggers one ping, mirroring cat */
	fd = open(HCSR04_DEV, O_RDONLY);
	if (fd < 0)
		return -errno;

	n = read(fd, buf, sizeof(buf) - 1);
	close(fd);
	if (n <= 0)
		return n < 0 ? -errno : -EIO;

	buf[n] = '\0';
	*cm = atoi(buf);
	return 0;
}

int weight_read_mg(long *mg)
{
	int val = 0;

	if (hx711_fd < 0)
		return -EBADF;
	if (ioctl(hx711_fd, HX711_IOC_READ_MG, &val) < 0)
		return -errno;

	*mg = (long)val;
	return 0;
}

int weight_tare(void)
{
	if (hx711_fd < 0)
		return -EBADF;
	if (ioctl(hx711_fd, HX711_IOC_TARE) < 0)
		return -errno;
	return 0;
}

int lock_open(void)
{
	int ret = sysfs_write_int("torque_enable", 1);

	if (ret)
		return ret;
	return sysfs_write_int("goal_position", SERVO_POS_OPEN);
}

int lock_close(void)
{
	int ret = sysfs_write_int("torque_enable", 1);

	if (ret)
		return ret;
	return sysfs_write_int("goal_position", SERVO_POS_LOCKED);
}

int servo_present_position(int *pos)
{
	return sysfs_read_int("present_position", pos);
}
