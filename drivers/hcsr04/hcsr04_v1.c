/*
 * hcsr04.c - HC-SR04 ultrasonic distance sensor driver (v1, polling)
 *
 * Reading from /dev/hcsr04 triggers one measurement and returns the
 * distance in centimetres as an ASCII string.
 *
 * v1 design:
 *   - TRIG pulse generated with udelay(10) for a deterministic 10us width
 *     (the key advantage over user-space shell tooling)
 *   - ECHO pulse width measured by busy-polling the GPIO and timestamping
 *     edges with ktime_get_ns()
 *
 * This polling version is intentionally simple; an IRQ-based v2 will follow
 * to compare interrupt latency and CPU utilisation.
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

/* gpiochip3 line 2 = PIN_13 = (3*32)+2 = 98  (TRIG, output) */
/* gpiochip3 line 1 = PIN_11 = (3*32)+1 = 97  (ECHO, input)  */
#define HCSR04_TRIG_GPIO  98
#define HCSR04_ECHO_GPIO  97

/* Timeout: max echo wait. Sound travels ~340 m/s, so 4 m round trip
 * is ~23 ms. We use 60 ms as a generous timeout to avoid hanging. */
#define ECHO_TIMEOUT_US   60000

static struct gpio_desc *trig_gpiod;
static struct gpio_desc *echo_gpiod;

static int hcsr04_measure_cm(int *distance_cm)
{
    ktime_t t_start, t_rise, t_fall, t_now;
    s64 pulse_ns;
    int echo_val;

    /* --- 1. Send 10us trigger pulse --- */
    gpiod_set_value(trig_gpiod, 1);
    udelay(10);
    gpiod_set_value(trig_gpiod, 0);

    /* --- 2. Wait for ECHO rising edge (with timeout) --- */
    t_start = ktime_get();
    do {
        echo_val = gpiod_get_value(echo_gpiod);
        t_now = ktime_get();
        if (ktime_to_us(ktime_sub(t_now, t_start)) > ECHO_TIMEOUT_US)
            return -ETIMEDOUT;
    } while (echo_val == 0);
    t_rise = ktime_get();

    /* --- 3. Wait for ECHO falling edge (with timeout) --- */
    do {
        echo_val = gpiod_get_value(echo_gpiod);
        t_now = ktime_get();
        if (ktime_to_us(ktime_sub(t_now, t_rise)) > ECHO_TIMEOUT_US)
            return -ETIMEDOUT;
    } while (echo_val == 1);
    t_fall = ktime_get();

    /* --- 4. Convert pulse width to distance --- */
    pulse_ns = ktime_to_ns(ktime_sub(t_fall, t_rise));
    /* distance_cm = pulse_us / 58 = (pulse_ns / 1000) / 58 = pulse_ns / 58000 */
    *distance_cm = (int)(pulse_ns / 58000);

    return 0;
}

static ssize_t hcsr04_read(struct file *file, char __user *buf,
                           size_t count, loff_t *ppos)
{
    char kbuf[32];
    int distance_cm;
    int len;
    int ret;

    /* Support single-shot read: return 0 on second call to signal EOF */
    if (*ppos > 0)
        return 0;

    ret = hcsr04_measure_cm(&distance_cm);
    if (ret < 0) {
        len = scnprintf(kbuf, sizeof(kbuf), "timeout\n");
    } else {
        len = scnprintf(kbuf, sizeof(kbuf), "%d\n", distance_cm);
    }

    if (count < len)
        return -EINVAL;

    if (copy_to_user(buf, kbuf, len))
        return -EFAULT;

    *ppos += len;
    return len;
}

static const struct file_operations hcsr04_fops = {
    .owner = THIS_MODULE,
    .read  = hcsr04_read,
};

static struct miscdevice hcsr04_miscdev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name  = "hcsr04",
    .fops  = &hcsr04_fops,
    .mode  = 0444,  /* read-only for everyone */
};

static int __init hcsr04_init(void)
{
    int ret;

    pr_info("hcsr04: initialising driver\n");

    /* Request TRIG GPIO as output */
    ret = gpio_request(HCSR04_TRIG_GPIO, "hcsr04-trig");
    if (ret) {
        pr_err("hcsr04: failed to request TRIG gpio %d: %d\n",
               HCSR04_TRIG_GPIO, ret);
        return ret;
    }
    trig_gpiod = gpio_to_desc(HCSR04_TRIG_GPIO);
    gpiod_direction_output(trig_gpiod, 0);

    /* Request ECHO GPIO as input */
    ret = gpio_request(HCSR04_ECHO_GPIO, "hcsr04-echo");
    if (ret) {
        pr_err("hcsr04: failed to request ECHO gpio %d: %d\n",
               HCSR04_ECHO_GPIO, ret);
        gpio_free(HCSR04_TRIG_GPIO);
        return ret;
    }
    echo_gpiod = gpio_to_desc(HCSR04_ECHO_GPIO);
    gpiod_direction_input(echo_gpiod);

    ret = misc_register(&hcsr04_miscdev);
    if (ret) {
        pr_err("hcsr04: misc_register failed: %d\n", ret);
        gpio_free(HCSR04_ECHO_GPIO);
        gpio_free(HCSR04_TRIG_GPIO);
        return ret;
    }

    pr_info("hcsr04: ready, /dev/hcsr04 created (TRIG=%d ECHO=%d)\n",
            HCSR04_TRIG_GPIO, HCSR04_ECHO_GPIO);
    return 0;
}

static void __exit hcsr04_exit(void)
{
    misc_deregister(&hcsr04_miscdev);
gpio_free(HCSR04_ECHO_GPIO);
    gpio_free(HCSR04_TRIG_GPIO);
    pr_info("hcsr04: driver unloaded\n");
}

module_init(hcsr04_init);
module_exit(hcsr04_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("qingshan");
MODULE_DESCRIPTION("HC-SR04 ultrasonic sensor driver (v1, polling)");
MODULE_VERSION("0.1");
