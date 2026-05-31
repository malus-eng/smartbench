/*
 * hx711_ioctl.h - shared ioctl definitions for the HX711 driver
 *
 * Included by both the kernel driver and user-space programs so the
 * command numbers stay in sync.
 */
#ifndef HX711_IOCTL_H
#define HX711_IOCTL_H

#include <linux/ioctl.h>

#define HX711_IOC_MAGIC  'h'

/* Capture current raw reading as the zero point (tare) */
#define HX711_IOC_TARE        _IO(HX711_IOC_MAGIC, 1)

/* Set scale factor: ADC counts per gram, multiplied by 1000 (fixed-point) */
#define HX711_IOC_SET_SCALE   _IOW(HX711_IOC_MAGIC, 2, int)

/* Read weight in milligrams (grams * 1000) */
#define HX711_IOC_READ_MG     _IOR(HX711_IOC_MAGIC, 3, int)

#endif /* HX711_IOCTL_H */
