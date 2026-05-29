/*
 * hcsr04.c - HC-SR04 ultrasonic distance sensor driver (v2, IRQ-based)
 *
 * Reading from /dev/hcsr04 triggers one measurement and returns the
 * distance in centimetres as an ASCII string.
 *
 * v2 design improvements over v1:
 *   - TRIG pulse still generated with udelay(10)
 *   - ECHO pulse width measured by capturing both edges via GPIO IRQ
 *   - .read() blocks the calling process on a wait queue while the
 *     measurement happens in interrupt context, freeing the CPU
 *   - ktime_get_ns() called from inside the ISR for minimal latency
 *
 * This eliminates the ~60 ms of busy-wait CPU usage in v1, while
 * keeping or improving timing precision.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/ktime.h>
#include <linux/wait.h>
#include <linux/mutex.h>

/* gpiochip3 line 2 = PIN_13 = (3*32)+2 = 98  (TRIG, output) */
/* gpiochip3 line 1 = PIN_11 = (3*32)+1 = 97  (ECHO, input)  */
#define HCSR04_TRIG_GPIO  98
#define HCSR04_ECHO_GPIO  97

/* Max wait for echo to complete; 60 ms = ~10 m round-trip */
#define ECHO_TIMEOUT_MS   60

static struct gpio_desc *trig_gpiod;
static struct gpio_desc *echo_gpiod;
static int echo_irq;

/* Per-measurement state. Protected by `meas_lock` to serialise concurrent
 * readers; only one measurement may be in flight at a time. */
static DEFINE_MUTEX(meas_lock);
static DECLARE_WAIT_QUEUE_HEAD(echo_done_wq);

static ktime_t t_rise;
static ktime_t t_fall;
static bool got_rise;
static bool got_fall;

/* Interrupt handler: called on both rising and falling edges of ECHO.
 *
 * Edge detection: we read the current GPIO value. If it's 1, this is a
 * rising edge (the ECHO line just went high). If it's 0, this is a
 * falling edge. This is simpler than tracking edge type separately. */
static irqreturn_t hcsr04_echo_isr(int irq, void *dev_id)
{
    ktime_t now = ktime_get();
    int val = gpiod_get_value(echo_gpiod);

    if (val == 1 && !got_rise) {
        t_rise = now;
        got_rise = true;
    } else if (val == 0 && got_rise && !got_fall) {
        t_fall = now;
        got_fall = true;
        wake_up_interruptible(&echo_done_wq);
    }

    return IRQ_HANDLED;
}

static int hcsr04_measure_cm(int *distance_cm)
{
    s64 pulse_ns;
    long ret;

    /* Reset state for this measurement */
    got_rise = false;
    got_fall = false;

    /* Send 10us trigger pulse */
    gpiod_set_value(trig_gpiod, 1);
    udelay(10);
    gpiod_set_value(trig_gpiod, 0);

    /* Sleep until the falling-edge ISR wakes us, or we time out */
    ret = wait_event_interruptible_timeout(echo_done_wq,
                                           got_fall,
                                           msecs_to_jiffies(ECHO_TIMEOUT_MS));
    if (ret == 0)
        return -ETIMEDOUT;
    if (ret < 0)
        return ret;  /* interrupted by signal */

    pulse_ns = ktime_to_ns(ktime_sub(t_fall, t_rise));
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

    if (*ppos > 0)
        return 0;

    /* Serialise concurrent readers — only one measurement at a time */
    if (mutex_lock_interruptible(&meas_lock))
        return -ERESTARTSYS;

    ret = hcsr04_measure_cm(&distance_cm);

    mutex_unlock(&meas_lock);

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
    .mode  = 0444,
};

static int __init hcsr04_init(void)
{
    int ret;

    pr_info("hcsr04: initialising driver (v2, IRQ-based)\n");

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
        goto err_free_trig;
    }
    echo_gpiod = gpio_to_desc(HCSR04_ECHO_GPIO);
    gpiod_direction_input(echo_gpiod);

    /* Translate the ECHO gpio descriptor into an IRQ number */
    echo_irq = gpiod_to_irq(echo_gpiod);
    if (echo_irq < 0) {
        pr_err("hcsr04: gpiod_to_irq failed: %d\n", echo_irq);
        ret = echo_irq;
        goto err_free_echo;
    }

    /* Register the IRQ handler for both edges */
    ret = request_irq(echo_irq, hcsr04_echo_isr,
                      IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
                      "hcsr04-echo", NULL);
    if (ret) {
        pr_err("hcsr04: request_irq failed: %d\n", ret);
        goto err_free_echo;
    }

    ret = misc_register(&hcsr04_miscdev);
    if (ret) {
        pr_err("hcsr04: misc_register failed: %d\n", ret);
        goto err_free_irq;
    }

    pr_info("hcsr04: ready, /dev/hcsr04 created (TRIG=%d ECHO=%d IRQ=%d)\n",
            HCSR04_TRIG_GPIO, HCSR04_ECHO_GPIO, echo_irq);
    return 0;

err_free_irq:
    free_irq(echo_irq, NULL);
err_free_echo:
    gpio_free(HCSR04_ECHO_GPIO);
err_free_trig:
    gpio_free(HCSR04_TRIG_GPIO);
    return ret;
}

static void __exit hcsr04_exit(void)
{
    misc_deregister(&hcsr04_miscdev);
    free_irq(echo_irq, NULL);
    gpio_free(HCSR04_ECHO_GPIO);
    gpio_free(HCSR04_TRIG_GPIO);
    pr_info("hcsr04: driver unloaded (v2)\n");
}

module_init(hcsr04_init);
module_exit(hcsr04_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("qingshan");
MODULE_DESCRIPTION("HC-SR04 ultrasonic sensor driver (v2, IRQ-based)");
MODULE_VERSION("0.2");
