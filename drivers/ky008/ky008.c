/*
 * ky008.c - KY-008 Laser Transmitter driver for Smartbench
 *
 * A minimal character device driver that controls a KY-008 laser module
 * via a single GPIO line. Writing "1" to /dev/ky008 turns the laser on,
 * writing "0" turns it off.
 *
 * This is a learning-stage driver using legacy GPIO numbering. A proper
 * device-tree-based version will replace this once the platform driver
 * model is introduced.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
/* GPIO3_A6 = (3 * 32) + 3 = 99 in legacy global GPIO numbering */
#define KY008_GPIO  99
static struct gpio_desc *ky008_gpiod;
static int ky008_open(struct inode *inode, struct file *file)
{
    pr_info("ky008: device opened\n");
    return 0;
}
static int ky008_release(struct inode *inode, struct file *file)
{
    pr_info("ky008: device closed\n");
    return 0;
}
static ssize_t ky008_write(struct file *file, const char __user *buf,
                           size_t count, loff_t *ppos)
{
    char kbuf[8];
    size_t len;
    if (count == 0)
        return 0;
    len = min(count, sizeof(kbuf) - 1);
    if (copy_from_user(kbuf, buf, len))
        return -EFAULT;
    kbuf[len] = '\0';
    /* Accept "1", "1\n", "on" -> ON; anything else -> OFF */
    if (kbuf[0] == '1') {
        gpiod_set_value(ky008_gpiod, 1);
        pr_info("ky008: laser ON\n");
    } else {
        gpiod_set_value(ky008_gpiod, 0);
        pr_info("ky008: laser OFF\n");
    }
    return count;
}
static const struct file_operations ky008_fops = {
    .owner   = THIS_MODULE,
    .open    = ky008_open,
    .release = ky008_release,
    .write   = ky008_write,
};
static struct miscdevice ky008_miscdev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name  = "ky008",
    .fops  = &ky008_fops,
    .mode  = 0666,  /* allow non-root write for testing */
};
static int __init ky008_init(void)
{
    int ret;
    pr_info("ky008: initialising driver\n");
    /* Request the GPIO via legacy API and obtain a descriptor */
    ret = gpio_request(KY008_GPIO, "ky008-laser");
    if (ret) {
        pr_err("ky008: failed to request GPIO %d: %d\n", KY008_GPIO, ret);
        return ret;
    }
    ky008_gpiod = gpio_to_desc(KY008_GPIO);
    if (!ky008_gpiod) {
        pr_err("ky008: gpio_to_desc failed for %d\n", KY008_GPIO);
        gpio_free(KY008_GPIO);
        return -ENODEV;
    }
    /* Configure as output, default low (laser off) */
    ret = gpiod_direction_output(ky008_gpiod, 0);
    if (ret) {
        pr_err("ky008: failed to set output direction: %d\n", ret);
        gpio_free(KY008_GPIO);
        return ret;
    }
    /* Register the misc device, creating /dev/ky008 */
    ret = misc_register(&ky008_miscdev);
    if (ret) {
        pr_err("ky008: misc_register failed: %d\n", ret);
        gpio_free(KY008_GPIO);
        return ret;
    }
    pr_info("ky008: driver loaded, /dev/ky008 created on GPIO %d\n",
            KY008_GPIO);
    return 0;
}
static void __exit ky008_exit(void)
{
    gpiod_set_value(ky008_gpiod, 0);  /* turn laser off before leaving */
    misc_deregister(&ky008_miscdev);
    gpio_free(KY008_GPIO);
    pr_info("ky008: driver unloaded\n");
}
module_init(ky008_init);
module_exit(ky008_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("qingshan");
MODULE_DESCRIPTION("KY-008 laser transmitter driver for Smartbench");
MODULE_VERSION("0.1");
