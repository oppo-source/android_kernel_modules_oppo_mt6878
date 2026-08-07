// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2022 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "oplus_locking_strategy:" fmt

#include <linux/sched.h>
#include <linux/module.h>
#include <linux/of.h>

#include "locking_main.h"
#include "shared_mem_channel.h"

#define FEATURE_ENABLE 0xFFFFF07F

unsigned int g_opt_enable;
unsigned int g_opt_debug;
unsigned int dynamic_switch;
unsigned int lazy_debug_enable;
unsigned int sysctl_sched_preempt_delay_us = SCHED_PREEMPT_DELAY_DEFAULT_US;
int sysctl_lock_boost_debug_enable;
int sysctl_lock_boost_yield_to_rt;
unsigned int sysctl_lock_boost_rt_yield_ms = 20;

void parse_dts_switch(void)
{
	struct device_node *np = NULL;
	int ret;

	np = of_find_node_by_name(NULL, "oplus_sync_ipc");

	if(np) {
		ret = of_property_read_u32(np, "disable", &dynamic_switch);
		if(ret) {
			pr_err("no oplus_sync_ipc disable!");
		} else {
			pr_info("oplus_sync_ipc : %d", dynamic_switch);
			return;
		}
	}

	dynamic_switch = FEATURE_ENABLE;
}
EXPORT_SYMBOL(g_opt_enable);

static int __init locking_opt_init(void)
{
	int ret = 0;

	parse_dts_switch();

	g_opt_enable |= LK_MUTEX_ENABLE;
	g_opt_enable |= LK_RWSEM_ENABLE;
	g_opt_enable |= LK_FUTEX_ENABLE;
	g_opt_enable |= LK_PCPU_RWSEM_RT_ENABLE;
	g_opt_enable |= LK_RWSEM_PRIORITY_ENABLE;
	g_opt_enable |= LK_LAZY_PREEMPTION_ENABLE;
	g_opt_enable |= LK_LOCK_BOOST_ENABLE;
#ifdef CONFIG_OPLUS_LOCKING_OSQ
	g_opt_enable |= LK_OSQ_ENABLE;
#endif
#ifdef CONFIG_OPLUS_LOCKING_PIFUTEX
	g_opt_enable |= LK_PIFUTEX_ENABLE;
#endif
#ifdef CONFIG_OPLUS_RWSEM_RSPIN
	g_opt_enable |= LK_RWSEM_RSPIN_ENABLE;
#endif

#ifdef CONFIG_LOCKING_PROTECT
	g_opt_enable |= LK_PROTECT_ENABLE;
	sched_assist_locking_init();
#endif

	g_opt_enable &= dynamic_switch;
	pr_info("oplus_locking g_opt_enable : %d", g_opt_enable);

	lk_sysfs_init();
	register_percpu_rwsem_vendor_hooks();
	register_rwsem_vendor_hooks();
	register_mutex_vendor_hooks();
	register_futex_vendor_hooks();
#ifdef CONFIG_OPLUS_LOCKING_PIFUTEX
	register_rtmutex_vendor_hooks();
#endif
	register_lazy_vendor_hooks();
	sched_boost_init();
	register_lock_boost_vendor_hooks();
#ifdef CONFIG_OPLUS_LOCKING_MONITOR
	g_opt_enable |= LK_MONITOR_ENABLE;
	kern_lstat_init();
#endif

	return ret;
}

static void __exit locking_opt_exit(void)
{
	g_opt_enable = 0;

	unregister_percpu_rwsem_vendor_hooks();
	unregister_rwsem_vendor_hooks();
	unregister_mutex_vendor_hooks();
	unregister_futex_vendor_hooks();
#ifdef CONFIG_OPLUS_LOCKING_PIFUTEX
	unregister_rtmutex_vendor_hooks();
#endif
	unregister_lazy_vendor_hooks();
	unregister_lock_boost_vendor_hooks();
	sched_boost_exit();
	lk_sysfs_exit();
#ifdef CONFIG_OPLUS_LOCKING_MONITOR
	kern_lstat_exit();
#endif
}

module_init(locking_opt_init);
module_exit(locking_opt_exit);
module_param_named(locking_enable, g_opt_enable, uint, 0660);
module_param_named(locking_debug, g_opt_debug, uint, 0660);
module_param_named(lazy_debug_enable, lazy_debug_enable, uint, 0660);

static int lock_boost_delay_us_set(const char *val, const struct kernel_param *kp)
{
	unsigned int v;
	int ret = kstrtouint(val, 0, &v);

	if (ret)
		return ret;
	if (v > SCHED_PREEMPT_DELAY_MAX_US)
		v = SCHED_PREEMPT_DELAY_MAX_US;
	sysctl_sched_preempt_delay_us = v;
	return 0;
}

static int lock_boost_delay_us_get(char *buf, const struct kernel_param *kp)
{
	return sysfs_emit(buf, "%u\n", sysctl_sched_preempt_delay_us);
}

static const struct kernel_param_ops lock_boost_delay_us_ops = {
	.set = lock_boost_delay_us_set,
	.get = lock_boost_delay_us_get,
};

module_param_cb(sched_preempt_delay_us, &lock_boost_delay_us_ops, NULL, 0660);
module_param_named(lock_boost_debug_enable, sysctl_lock_boost_debug_enable, int, 0660);
module_param_named(lock_boost_yield_to_rt, sysctl_lock_boost_yield_to_rt, int, 0660);
module_param_named(lock_boost_rt_yield_ms, sysctl_lock_boost_rt_yield_ms, uint, 0660);
MODULE_DESCRIPTION("Oplus Locking Strategy Vender Hooks Driver");
MODULE_LICENSE("GPL v2");
