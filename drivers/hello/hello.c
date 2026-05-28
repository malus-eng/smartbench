#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>

static int __init hello_init(void)
{
    pr_info("smartbench: Hello, smartbench is alive!\n");
    return 0;
}

static void __exit hello_exit(void)
{
    pr_info("smartbench: Goodbye, smartbench is shutting down.\n");
}

module_init(hello_init);
module_exit(hello_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("Smartbench hello world module");
MODULE_VERSION("0.1");
