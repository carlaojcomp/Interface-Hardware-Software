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
	{ 0x15ba50a6, "jiffies" },
	{ 0x3011c5fc, "input_event" },
	{ 0x3213f038, "mutex_unlock" },
	{ 0x2cf56265, "__dynamic_pr_debug" },
	{ 0xcbd4898c, "fortify_panic" },
	{ 0xf0fdf6cb, "__stack_chk_fail" },
	{ 0x96848186, "scnprintf" },
	{ 0xc38c83b8, "mod_timer" },
	{ 0x746d1622, "kmalloc_caches" },
	{ 0x55151bf9, "kmalloc_trace" },
	{ 0xcefb0c9f, "__mutex_init" },
	{ 0xd9a5ea54, "__init_waitqueue_head" },
	{ 0xc6f46339, "init_timer_key" },
	{ 0x40df682c, "class_create" },
	{ 0xe8962364, "device_create" },
	{ 0x920f333d, "input_allocate_device" },
	{ 0x72e46751, "input_set_capability" },
	{ 0x5743a53f, "input_register_device" },
	{ 0x267df1a8, "device_create_file" },
	{ 0xe98ac415, "kthread_create_on_node" },
	{ 0x86c30bb3, "wake_up_process" },
	{ 0xfcb2c465, "kernel_write" },
	{ 0x98d68067, "input_free_device" },
	{ 0xb39b608e, "device_destroy" },
	{ 0x4b7b5a1a, "class_destroy" },
	{ 0x37a0cba, "kfree" },
	{ 0x636bda06, "device_remove_file" },
	{ 0xffae20c, "input_unregister_device" },
	{ 0x546428ed, "filp_open" },
	{ 0x50b9b3f, "filp_close" },
	{ 0x4dfa8d4b, "mutex_lock" },
	{ 0x82ee90dc, "timer_delete_sync" },
	{ 0xd970ef39, "kthread_stop" },
	{ 0x69acdf38, "memcpy" },
	{ 0xbdfb6dbb, "__fentry__" },
	{ 0x122c3a7e, "_printk" },
	{ 0xb3f7646e, "kthread_should_stop" },
	{ 0x576666fd, "kernel_read" },
	{ 0xf9a482f9, "msleep" },
	{ 0x5b8239ca, "__x86_return_thunk" },
	{ 0xb0e602eb, "memmove" },
	{ 0xbb9ed3bf, "mutex_trylock" },
	{ 0xb08e71bf, "module_layout" },
};

MODULE_INFO(depends, "");


MODULE_INFO(srcversion, "F165BE305089ECC22DD4C52");
