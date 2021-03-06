#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

BUILD_SALT;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif

static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0x2ae8735, "module_layout" },
	{ 0xc1514a3b, "free_irq" },
	{ 0x6091b333, "unregister_chrdev_region" },
	{ 0x16bd2233, "cdev_del" },
	{ 0x655f555e, "class_destroy" },
	{ 0x5f575f8d, "device_destroy" },
	{ 0xfe990052, "gpio_free" },
	{ 0x92d5838e, "request_threaded_irq" },
	{ 0x650f364d, "gpiod_to_irq" },
	{ 0x5b6aff8, "gpiod_direction_input" },
	{ 0xd5d8afe2, "gpiod_direction_output_raw" },
	{ 0x47229b5c, "gpio_request" },
	{ 0x8e1984a8, "device_create" },
	{ 0x21a54c1d, "__class_create" },
	{ 0x7ce62d80, "cdev_add" },
	{ 0x5175e3c0, "cdev_init" },
	{ 0xe3ec2f2b, "alloc_chrdev_region" },
	{ 0x2cfde9a2, "warn_slowpath_fmt" },
	{ 0x5f754e5a, "memset" },
	{ 0xae353d77, "arm_copy_from_user" },
	{ 0x86332725, "__stack_chk_fail" },
	{ 0x51a910c0, "arm_copy_to_user" },
	{ 0x8f678b07, "__stack_chk_guard" },
	{ 0xd697e69a, "trace_hardirqs_on" },
	{ 0xec3d2e1b, "trace_hardirqs_off" },
	{ 0x86bad034, "gpiod_get_raw_value" },
	{ 0xb5118889, "gpiod_set_raw_value" },
	{ 0x7b0746a0, "gpio_to_desc" },
	{ 0x526c3a6c, "jiffies" },
	{ 0xc5850110, "printk" },
	{ 0xb1ad28e0, "__gnu_mcount_nc" },
};

MODULE_INFO(depends, "");


MODULE_INFO(srcversion, "CE6967429030FA79128C8C8");
