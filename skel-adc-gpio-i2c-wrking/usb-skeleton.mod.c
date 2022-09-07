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
	{ 0xf3bb3eea, "module_layout" },
	{ 0xd5aae2c2, "noop_llseek" },
	{ 0x10c09b86, "usb_deregister" },
	{ 0xb819053c, "usb_register_driver" },
	{ 0xa9fc8d1, "usb_unanchor_urb" },
	{ 0xdcb764ad, "memset" },
	{ 0x878d2821, "usb_free_coherent" },
	{ 0xcf2a6966, "up" },
	{ 0xe9ffc063, "down_trylock" },
	{ 0xc8d410db, "usb_anchor_urb" },
	{ 0x12a4e128, "__arch_copy_from_user" },
	{ 0xffc0920b, "usb_alloc_coherent" },
	{ 0x6bd0e573, "down_interruptible" },
	{ 0x3eeb2322, "__wake_up" },
	{ 0x7ceaf0d5, "generic_handle_irq" },
	{ 0x6cbbfc54, "__arch_copy_to_user" },
	{ 0x92540fbf, "finish_wait" },
	{ 0x8c26d495, "prepare_to_wait_event" },
	{ 0x1000e51, "schedule" },
	{ 0xfe487975, "init_wait_entry" },
	{ 0x800473f, "__cond_resched" },
	{ 0xb5136dc7, "mutex_lock_interruptible" },
	{ 0x9c1e5bf5, "queued_spin_lock_slowpath" },
	{ 0x4b0a3f52, "gic_nonsecure_priorities" },
	{ 0x2e2b6667, "usb_autopm_get_interface" },
	{ 0x39f23b95, "usb_find_interface" },
	{ 0x69a4dc53, "usb_deregister_dev" },
	{ 0x1cf7fe20, "device_remove_file" },
	{ 0xd164f7ab, "usb_register_dev" },
	{ 0xa85b236e, "gpiochip_add_data_with_key" },
	{ 0x8e13ede, "device_create_file" },
	{ 0x66762163, "_dev_info" },
	{ 0x655e4879, "__irq_alloc_descs" },
	{ 0x7bd9148f, "handle_simple_irq" },
	{ 0x4729bd24, "usb_submit_urb" },
	{ 0xeb233a45, "__kmalloc" },
	{ 0x10390b94, "usb_alloc_urb" },
	{ 0x86332725, "__stack_chk_fail" },
	{ 0x4db0f56c, "_dev_err" },
	{ 0x93c7edeb, "usb_find_common_endpoints" },
	{ 0x92382b70, "usb_get_intf" },
	{ 0x73c5ef8d, "usb_get_dev" },
	{ 0xd9a5ea54, "__init_waitqueue_head" },
	{ 0x977f511b, "__mutex_init" },
	{ 0x296695f, "refcount_warn_saturate" },
	{ 0x5e2d7875, "cpu_hwcap_keys" },
	{ 0x14b89635, "arm64_const_caps_ready" },
	{ 0xb3bf6a3c, "usb_autopm_put_interface" },
	{ 0xb0b1a325, "kmem_cache_alloc" },
	{ 0xdd7fb546, "kmalloc_caches" },
	{ 0x914a0af7, "irq_create_mapping_affinity" },
	{ 0xc5b6f236, "queue_work_on" },
	{ 0x2d3385d3, "system_wq" },
	{ 0x6792ddf7, "usb_control_msg" },
	{ 0x56470118, "__warn_printk" },
	{ 0x7e69e00f, "usb_put_dev" },
	{ 0x996759bf, "usb_put_intf" },
	{ 0x37a0cba, "kfree" },
	{ 0x165ec01d, "usb_free_urb" },
	{ 0x3c12dfe, "cancel_work_sync" },
	{ 0xc5850110, "printk" },
	{ 0x2ab7989d, "mutex_lock" },
	{ 0x962c8ae1, "usb_kill_anchored_urbs" },
	{ 0x3301ca1b, "usb_kill_urb" },
	{ 0x407af304, "usb_wait_anchor_empty_timeout" },
	{ 0x409bcb62, "mutex_unlock" },
};

MODULE_INFO(depends, "");

MODULE_ALIAS("usb:v1D50p6018d*dc*dsc*dp*icFFisc00ip00in*");
MODULE_ALIAS("usb:v0403pC631d*dc*dsc*dp*ic*isc*ip*in*");
