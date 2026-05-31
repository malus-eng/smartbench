/*
 * hx711.c - HX711 24-bit ADC driver for load cells
 *
 * Reading from /dev/hx711 returns one 24-bit signed ADC sample as an
 * ASCII decimal string.
 *
 * HX711 uses a custom 2-wire protocol (DT + SCK), not SPI or I2C.
 * The protocol requires uninterrupted timing: any SCK high pulse
 * longer than 50us puts the chip into sleep mode. We therefore
 * disable local interrupts for the duration of the bit-bang.
 *
 * Pin assignment:
 *   DT  (data)  on PIN_15 = gpiochip3 line 8  = legacy GPIO 104
 *   SCK (clock) on PIN_16 = gpiochip3 line 9  = legacy GPIO 105
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

#define HX711_DT_GPIO    104
#define HX711_SCK_GPIO   105

/* Max time to wait for HX711 to assert "data ready" (DT low) */
#define HX711_READY_TIMEOUT_MS  200

static struct gpio_desc *dt_gpiod;
static struct gpio_desc *sck_gpiod;
static DEFINE_MUTEX(hx711_lock);

/*
 * Perform one 24-bit read from the HX711.
 * Returns 0 on success and writes the signed 24-bit value to *out.
 * Returns -ETIMEDOUT if HX711 never signals data ready.
 *
 * MUST be called with hx711_lock held.
 */
static int hx711_read_raw(s32 *out)
{
    ktime_t t_start;
    unsigned long flags;
    s32 value = 0;
    int i;

    /* --- 1. Wait for DT to go low (data ready) --- */
    t_start = ktime_get();
    while (gpiod_get_value(dt_gpiod) != 0) {
        if (ktime_ms_delta(ktime_get(), t_start) > HX711_READY_TIMEOUT_MS)
            return -ETIMEDOUT;
        usleep_range(100, 200);  /* OK to sleep here, we're outside the critical section */
    }

    /* --- 2. Critical timing section: disable local interrupts --- */
    local_irq_save(flags);

    /* Bit-bang 25 SCK pulses */
    for (i = 0; i < 25; i++) {
        gpiod_set_value(sck_gpiod, 1);
        udelay(1);                       /* SCK high ~1us, well within 0.2-50us spec */

        if (i < 24) {
            /* MSB first: shift left and OR in current bit */
            value = (value << 1) | gpiod_get_value(dt_gpiod);
        }
        /* The 25th pulse selects gain 128 for next reading; no data read here */

        gpiod_set_value(sck_gpiod, 0);
        udelay(1);                       /* SCK low ~1us */
    }

    local_irq_restore(flags);

    /* --- 3. Sign-extend the 24-bit two's complement value to 32 bits --- */
    if (value & 0x00800000)
        value |= 0xFF000000;

    *out = value;
    return 0;
}

static ssize_t hx711_read(struct file *file, char __user *buf,
                          size_t count, loff_t *ppos)
{
    char kbuf[32];
    s32 value;
    int len;
    int ret;

    if (*ppos > 0)
        return 0;

    if (mutex_lock_interruptible(&hx711_lock))
        return -ERESTARTSYS;

    ret = hx711_read_raw(&value);

    mutex_unlock(&hx711_lock);

    if (ret < 0) {
        len = scnprintf(kbuf, sizeof(kbuf), "timeout\n");
    } else {
        len = scnprintf(kbuf, sizeof(kbuf), "%d\n", value);
    }

    if (count < len)
        return -EINVAL;
    if (copy_to_user(buf, kbuf, len))
        return -EFAULT;

    *ppos += len;
    return len;
}

static const struct file_operations hx711_fops = {
    .owner = THIS_MODULE,
    .read  = hx711_read,
};

static struct miscdevice hx711_miscdev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name  = "hx711",
    .fops  = &hx711_fops,
    .mode  = 0444,
};

static int __init hx711_init(void)
{
    int ret;

    pr_info("hx711: initialising driver\n");

    ret = gpio_request(HX711_DT_GPIO, "hx711-dt");
    if (ret) {
        pr_err("hx711: failed to request DT gpio %d: %d\n",
               HX711_DT_GPIO, ret);
        return ret;
    }
    dt_gpiod = gpio_to_desc(HX711_DT_GPIO);
    gpiod_direction_input(dt_gpiod);

    ret = gpio_request(HX711_SCK_GPIO, "hx711-sck");
    if (ret) {
        pr_err("hx711: failed to request SCK gpio %d: %d\n",
               HX711_SCK_GPIO, ret);
        goto err_free_dt;
    }
    sck_gpiod = gpio_to_desc(HX711_SCK_GPIO);
    gpiod_direction_output(sck_gpiod, 0);  /* SCK starts low */

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
MODULE_DESCRIPTION("HX711 24-bit ADC driver for load cells");
MODULE_VERSION("0.1");
