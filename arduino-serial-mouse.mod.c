#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/elfnote-lto.h>
#include <linux/export-internal.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

#ifdef CONFIG_UNWINDER_ORC
#include <asm/orc_header.h>
ORC_HEADER;
#endif

BUILD_SALT;
BUILD_LTO_INFO;

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
	{ 0x3d55706c, "_dev_info" },
	{ 0x4dfa8d4b, "mutex_lock" },
	{ 0x82ee90dc, "timer_delete_sync" },
	{ 0x636bda06, "device_remove_file" },
	{ 0x3c12dfe, "cancel_work_sync" },
	{ 0x8c03d20c, "destroy_workqueue" },
	{ 0x2a9c2a86, "serdev_device_close" },
	{ 0x34db050b, "_raw_spin_lock_irqsave" },
	{ 0xd35cce70, "_raw_spin_unlock_irqrestore" },
	{ 0xc5b6f236, "queue_work_on" },
	{ 0x15ba50a6, "jiffies" },
	{ 0x3011c5fc, "input_event" },
	{ 0x3eeca751, "__dynamic_dev_dbg" },
	{ 0x80841722, "devm_kmalloc" },
	{ 0xcefb0c9f, "__mutex_init" },
	{ 0xbda2cc4, "devm_kasprintf" },
	{ 0x49cd25ed, "alloc_workqueue" },
	{ 0xc6f46339, "init_timer_key" },
	{ 0xf4daa2f1, "serdev_device_set_baudrate" },
	{ 0xc008c005, "serdev_device_set_flow_control" },
	{ 0x8066a321, "serdev_device_set_parity" },
	{ 0xe1b310bf, "serdev_device_open" },
	{ 0xb4528dba, "devm_input_allocate_device" },
	{ 0x72e46751, "input_set_capability" },
	{ 0x5743a53f, "input_register_device" },
	{ 0x267df1a8, "device_create_file" },
	{ 0xc38c83b8, "mod_timer" },
	{ 0xf9a482f9, "msleep" },
	{ 0x4c31241f, "_dev_err" },
	{ 0xce43cea3, "_dev_warn" },
	{ 0x9baf9ca5, "driver_unregister" },
	{ 0xbdfb6dbb, "__fentry__" },
	{ 0x25b219c2, "__serdev_device_driver_register" },
	{ 0x5b8239ca, "__x86_return_thunk" },
	{ 0xbb9ed3bf, "mutex_trylock" },
	{ 0x96848186, "scnprintf" },
	{ 0x3213f038, "mutex_unlock" },
	{ 0x69acdf38, "memcpy" },
	{ 0xd98c0f50, "serdev_device_write" },
	{ 0xf0fdf6cb, "__stack_chk_fail" },
	{ 0xb08e71bf, "module_layout" },
};

MODULE_INFO(depends, "");

MODULE_ALIAS("of:N*T*Carduino,serial-mouse");
MODULE_ALIAS("of:N*T*Carduino,serial-mouseC*");

MODULE_INFO(srcversion, "AE461F34C89BA17405BEF72");
