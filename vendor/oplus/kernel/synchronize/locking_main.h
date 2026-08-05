/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2020-2022 Oplus. All rights reserved.
 */

#ifndef _OPLUS_LOCKING_MAIN_H_
#define _OPLUS_LOCKING_MAIN_H_

#define cond_trace_printk(cond, fmt, ...)	\
do {										\
	if (cond)								\
		trace_printk(fmt, ##__VA_ARGS__);	\
} while (0)



#define MAGIC_NUM       (0xdead0000)
#define MAGIC_MASK      (0xffff0000)
#define MAGIC_SHIFT     (16)
#define OWNER_BIT       (1 << 0)
#define THREAD_INFO_BIT (1 << 1)
#define TYPE_BIT        (1 << 2)

#define UX_FLAG_BIT       (1<<0)
#define SS_FLAG_BIT       (1<<1)
#define GRP_SHIFT         (2)
#define GRP_FLAG_MASK     (7 << GRP_SHIFT)
#define U_GRP_OTHER       (1 << GRP_SHIFT)
#define U_GRP_BACKGROUND  (2 << GRP_SHIFT)
#define U_GRP_FRONDGROUD  (3 << GRP_SHIFT)
#define U_GRP_TOP_APP     (4 << GRP_SHIFT)

#define LOCK_TYPE_SHIFT (30)
#define INVALID_TYPE    (0)
#define LOCK_ART        (1)
#define LOCK_JUC        (2)

#define lk_err(fmt, ...) \
		pr_err("[oplus_locking][%s]"fmt, __func__, ##__VA_ARGS__)
#define lk_warn(fmt, ...) \
		pr_warn("[oplus_locking][%s]"fmt, __func__, ##__VA_ARGS__)
#define lk_info(fmt, ...) \
		pr_info("[oplus_locking][%s]"fmt, __func__, ##__VA_ARGS__)

struct futex_uinfo {
	u32 cmd;
	u32 owner_tid;
	u32 type;
	u64 inform_user;
};

enum rwsem_waiter_type {
	RWSEM_WAITING_FOR_WRITE,
	RWSEM_WAITING_FOR_READ
};

struct rwsem_waiter {
	struct list_head list;
	struct task_struct *task;
	enum rwsem_waiter_type type;
	unsigned long timeout;
	bool handoff_set;
};

/* RWSEM Priority Optimization */
enum rwsem_process_type {
	RWSEM_PROCESS_NORMAL = 0,
	RWSEM_PROCESS_UX = 1,
	RWSEM_PROCESS_RT = 2,
	RWSEM_PROCESS_MAX
};

#define LK_MUTEX_ENABLE         (1 << 0)
#define LK_RWSEM_ENABLE         (1 << 1)
#define LK_FUTEX_ENABLE         (1 << 2)
#define LK_OSQ_ENABLE           (1 << 3)
#define LK_PIFUTEX_ENABLE       (1 << 4)
#define LK_PROTECT_ENABLE	(1 << 5)
#define LK_RWSEM_RSPIN_ENABLE   (1 << 6)
#define LK_MONITOR_ENABLE       (1 << 7)
#define LK_PCPU_RWSEM_RT_ENABLE (1 << 8)
#define LK_RWSEM_PRIORITY_ENABLE (1 << 9)
#define LK_LAZY_PREEMPTION_ENABLE (1 << 10)
#define LK_LOCK_BOOST_ENABLE      (1 << 11)
#define LK_FEATURE_MASK (LK_MUTEX_ENABLE | LK_RWSEM_ENABLE | LK_FUTEX_ENABLE | \
								LK_OSQ_ENABLE | LK_PROTECT_ENABLE | LK_MONITOR_ENABLE | \
								LK_PCPU_RWSEM_RT_ENABLE | LK_RWSEM_PRIORITY_ENABLE | \
								LK_LAZY_PREEMPTION_ENABLE | LK_LOCK_BOOST_ENABLE)

#ifdef CONFIG_OPLUS_LOCKING_MONITOR
/*
 * The bit definitions of the g_opt_enable:
 * bit 0-7: reserved bits for other locking optimation.
 * bit8 ~ bit10(each monitor version is exclusive):
 * 1 : monitor control, level-0(internal version).
 * 2 : monitor control, level-1(trial version).
 * 3 : monitor control, level-2(official version).
 */
#define LK_MONITOR_SHIFT  (8)
#define LK_MONITOR_MASK   (7 << LK_MONITOR_SHIFT)
#define LK_MONITOR_LEVEL0 (1 << LK_MONITOR_SHIFT)
#define LK_MONITOR_LEVEL1 (2 << LK_MONITOR_SHIFT)
#define LK_MONITOR_LEVEL2 (3 << LK_MONITOR_SHIFT)
#endif

#define LK_DEBUG_PRINTK (1 << 0)
#define LK_DEBUG_FTRACE (1 << 1)

extern unsigned int g_opt_enable;
extern unsigned int g_opt_debug;
extern unsigned int lazy_debug_enable;

#define SCHED_PREEMPT_DELAY_DEFAULT_US	5000
#define SCHED_PREEMPT_DELAY_MAX_US	50000

extern unsigned int sysctl_sched_preempt_delay_us;
extern int sysctl_lock_boost_debug_enable;
extern int sysctl_lock_boost_yield_to_rt;
extern unsigned int sysctl_lock_boost_rt_yield_ms;

static inline bool locking_opt_enable(unsigned int enable)
{
	return g_opt_enable & enable;
}

static inline void oplus_lk_feat_enable(unsigned int lk_feat, bool enable)
{
	lk_feat &= LK_FEATURE_MASK;
	if (enable)
		g_opt_enable |= lk_feat;
	else
		g_opt_enable &= ~lk_feat;
}

#ifdef CONFIG_OPLUS_LOCKING_MONITOR
static inline bool lock_supp_level(int level)
{
	return (g_opt_enable & LK_MONITOR_MASK) == level;
}
#endif

static inline bool locking_opt_debug(int debug)
{
	return g_opt_debug & debug;
}

void register_percpu_rwsem_vendor_hooks(void);
void register_rwsem_vendor_hooks(void);
void register_mutex_vendor_hooks(void);
void register_futex_vendor_hooks(void);
void register_monitor_vendor_hooks(void);
#ifdef CONFIG_OPLUS_LOCKING_PIFUTEX
void register_rtmutex_vendor_hooks(void);
#endif
void register_lock_boost_vendor_hooks(void);
void lk_sysfs_init(void);
#ifdef CONFIG_OPLUS_LOCKING_MONITOR
int kern_lstat_init(void);
#endif
void register_lazy_vendor_hooks(void);

void unregister_percpu_rwsem_vendor_hooks(void);
void unregister_rwsem_vendor_hooks(void);
void unregister_mutex_vendor_hooks(void);
void unregister_futex_vendor_hooks(void);
void unregister_monitor_vendor_hooks(void);
#ifdef CONFIG_OPLUS_LOCKING_PIFUTEX
void unregister_rtmutex_vendor_hooks(void);
#endif
void unregister_lock_boost_vendor_hooks(void);
void lk_sysfs_exit(void);
#ifdef CONFIG_OPLUS_LOCKING_MONITOR
void kern_lstat_exit(void);
#endif
void unregister_lazy_vendor_hooks(void);

#ifdef CONFIG_LOCKING_PROTECT
int sched_assist_locking_init(void);
#endif

#endif /* _OPLUS_LOCKING_MAIN_H_ */
