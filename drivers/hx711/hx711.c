/*
 * hx711.c - HX711 24-bit ADC driver for load cells (v2: tare, scale, averaging)
 *
 * Interfaces:
 *   read()                            -> raw 24-bit signed ADC value (ASCII)
 *   ioctl(HX711_IOC_TARE)             -> capture current reading as zero point
 *   ioctl(HX711_IOC_SET_SCALE, int)   -> set counts-per-gram * 1000
 *   ioctl(HX711_IOC_READ_MG, int*)    -> read weight in milligrams
 *
 * Reads use N-sample averaging to reduce per-sample noise.
 *
 * HX711 uses a custom 2-wire protocol (DT + SCK). An SCK high pulse > 50us
 * puts the chip to sleep, so we disable local interrupts during the 25-pulse
 * bit-bang. The kernel must not use floating point, so scale and weight use
 * fixed-point integer arithmetic (x1000).
 *
 * Pins: DT = PIN_15 (GPIO 104), SCK = PIN_16 (GPIO 105)
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/ktime.h>
#include <linux/mutex.h>
#include <linux/irqflags.h>
#include "hx711_ioctl.h"

#define HX711_DT_GPIO    104
#define HX711_SCK_GPIO   105
#define HX711_READY_TIMEOUT_MS  200
#define HX711_AVG_SAMPLES       10

static struct gpio_desc *dt_gpiod;
static struct gpio_desc *sck_gpiod;
static DEFINE_MUTEX(hx711_lock);

/* Calibration state */
static s32 tare_offset;          /* raw zero point */
static s32 scale_x1000 = 1000;   /* counts-per-gram * 1000; default 1.0 */

/* Perform one 24-bit read. Must hold hx711_lock. */
static int hx711_read_raw(s32 *out)
{
    ktime_t t_start;
    unsigned long flags;
    s32 value = 0;
    int i;

    t_start = ktime_get();
    while (gpiod_get_value(dt_gpiod) != 0) {
        if (ktime_ms_delta(ktime_get(), t_start) > HX711_READY_TIMEOUT_MS)
            return -ETIMEDOUT;
        usleep_range(100, 200);
    }

    local_irq_save(flags);
    for (i = 0; i < 25; i++) {
        gpiod_set_value(sck_gpiod, 1);
        udelay(1);
        if (i < 24)
            value = (value << 1) | gpiod_get_value(dt_gpiod);
        gpiod_set_value(sck_gpiod, 0);
        udelay(1);
    }
    local_irq_restore(flags);

    if (value & 0x00800000)
        value |= 0xFF000000;

    *out = value;
    return 0;
}

/* Read `samples` times and return the average. Must hold hx711_lock. */
static int hx711_read_avg(s32 *out, int samples)
{
    s64 sum = 0;
    s32 value;
    int i, ret;

    if (samples < 1)
        samples = 1;

    for (i = 0; i < samples; i++) {
        ret = hx711_read_raw(&value);
        if (ret < 0)
            return ret;
        sum += value;
    }

    *out = (s32)(sum / samples);
    return 0;
}

static ssize_t hx711_read(struct file *file, char __user *buf,
                          size_t count, loff_t *ppos)
{
    char kbuf[32];
    s32 value;
    int len, ret;

    if (*ppos > 0)
        return 0;

    if (mutex_lock_interruptible(&hx711_lock))
        return -ERESTARTSYS;
    ret = hx711_read_avg(&value, HX711_AVG_SAMPLES);
    mutex_unlock(&hx711_lock);

    if (ret < 0)
        len = scnprintf(kbuf, sizeof(kbuf), "timeout\n");
    else
        len = scnprintf(kbuf, sizeof(kbuf), "%d\n", value);

    if (count < len)
        return -EINVAL;
    if (copy_to_user(buf, kbuf, len))
        return -EFAULT;

    *ppos += len;
    return len;
}

static long hx711_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    s32 value;
    int ret;
    int scale_in;
    int weight_mg;

    switch (cmd) {
    case HX711_IOC_TARE:
        if (mutex_lock_interruptible(&hx711_lock))
            return -ERESTARTSYS;
        ret = hx711_read_avg(&value, HX711_AVG_SAMPLES);
        if (ret == 0)
            tare_offset = value;
        mutex_unlock(&hx711_lock);
        if (ret < 0)
            return ret;
        pr_info("hx711: tare set to %d\n", tare_offset);
        return 0;

    case HX711_IOC_SET_SCALE:
        if (copy_from_user(&scale_in, (int __user *)arg, sizeof(scale_in)))
            return -EFAULT;
        if (scale_in == 0)
            return -EINVAL;
        scale_x1000 = scale_in;
        pr_info("hx711: scale set to %d (counts/gram x1000)\n", scale_x1000);
        return 0;

    case HX711_IOC_READ_MG:
        if (mutex_lock_interruptible(&hx711_lock))
            return -ERESTARTSYS;
        ret = hx711_read_avg(&value, HX711_AVG_SAMPLES);
        mutex_unlock(&hx711_lock);
        if (ret < 0)
            return ret;
        {
            s64 numer = (s64)(value - tare_offset) * 1000 * 1000;
            weight_mg = (int)(numer / scale_x1000);
        }
        if (copy_to_user((int __user *)arg, &weight_mg, sizeof(weight_mg)))
            return -EFAULT;
        return 0;

    default:
        return -ENOTTY;
    }
}

static const struct file_operations hx711_fops = {
    .owner          = THIS_MODULE,
    .read           = hx711_read,
    .unlocked_ioctl = hx711_ioctl,
};

static struct miscdevice hx711_miscdev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name  = "hx711",
    .fops  = &hx711_fops,
    .mode  = 0666,
};

static int __init hx711_init(void)
{
    int ret;

    pr_info("hx711: initialising driver (v2, tare+scale+avg)\n");

    ret = gpio_request(HX711_DT_GPIO, "hx711-dt");
    if (ret) {
        pr_err("hx711: failed to request DT gpio: %d\n", ret);
        return ret;
    }
    dt_gpiod = gpio_to_desc(HX711_DT_GPIO);
    gpiod_direction_input(dt_gpiod);

    ret = gpio_request(HX711_SCK_GPIO, "hx711-sck");
    if (ret) {
        pr_err("hx711: failed to request SCK gpio: %d\n", ret);
        goto err_free_dt;
    }
    sck_gpiod = gpio_to_desc(HX711_SCK_GPIO);
    gpiod_direction_output(sck_gpiod, 0);

    ret = misc_register(&hx711_miscdev);
    if (ret) {
        pr_err("hx711: misc_register failed: %d\n", ret);
        goto err_free_sck;
    }

    pr_info("hx711: ready, /dev/hx711 created (DT=%d SCK=%d)\n",
            HX711_DT_GPIO, HX711_SCK_GPIO);
    return 0;

err_free_sck:
    gpio_free(HX711_SCK_GPIO);
err_free_dt:
    gpio_free(HX711_DT_GPIO);
    return ret;
}

static void __exit hx711_exit(void)
{
    misc_deregister(&hx711_miscdev);
    gpio_free(HX711_SCK_GPIO);
    gpio_free(HX711_DT_GPIO);
    pr_info("hx711: driver unloaded\n");
}

module_init(hx711_init);
module_exit(hx711_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("qingshan");
MODULE_DESCRIPTION("HX711 24-bit ADC driver with tare, scale and averaging");
MODULE_VERSION("0.2");
