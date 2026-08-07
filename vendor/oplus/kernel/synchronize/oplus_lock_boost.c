// SPDX-License-Identifier: GPL-2.0

#include <linux/module.h>
#include <linux/sched.h>
#include <../kernel/sched/sched.h>
#include <linux/sched/signal.h>
#include <linux/sched/clock.h>
#include <linux/thread_info.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/percpu.h>
#include <trace/hooks/dtask.h>
#include <../kernel/oplus_cpu/sched/sched_assist/sa_common.h>
#include "locking_main.h"
#include "shared_mem_channel.h"
#include <trace/hooks/sched.h>
#include "oplus_lock_boost.h"
#include <asm/syscall.h>
#include <asm/unistd.h>
#include <linux/preempt.h>

#ifndef SM_PREEMPT
#define SM_PREEMPT	0x1
#endif

#ifdef CONFIG_SCHED_HRTICK

static DEFINE_PER_CPU(struct hrtimer, lock_boost_timer);
static DEFINE_PER_CPU(struct task_struct *, lock_boost_timer_owner);

static DEFINE_PER_CPU(int, prev_boost_flag);


static enum hrtimer_restart lock_boost_timer_callback(struct hrtimer *timer)
{
	struct task_struct *owner;
	struct oplus_task_struct *ots;

	owner = __this_cpu_read(lock_boost_timer_owner);
	__this_cpu_write(lock_boost_timer_owner, NULL);

	if (owner) {
		ots = get_oplus_task_struct(owner);
		if (owner == current) {
			set_tsk_need_resched(owner);
			set_preempt_need_resched();
			if (!IS_ERR_OR_NULL(ots))
				set_bit(LOCK_BOOST_TICKOUT_BIT, &ots->lkinfo.state_flags);
		}
		put_task_struct(owner);
	}

	return HRTIMER_NORESTART;
}

static int lock_boost_timer_init(void)
{
	int cpu;

	for_each_possible_cpu(cpu) {
		struct hrtimer *timer = &per_cpu(lock_boost_timer, cpu);

		hrtimer_init(timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL_HARD);
		timer->function = lock_boost_timer_callback;
		per_cpu(lock_boost_timer_owner, cpu) = NULL;
	}

	pr_info("lock_boost: timers initialized\n");
	return 0;
}

static void hrtick_local_start(u64 delay_ns)
{
	struct hrtimer *timer = this_cpu_ptr(&lock_boost_timer);
	struct task_struct *prev_owner;
	struct oplus_task_struct *ots = get_oplus_task_struct(current);

	if (hrtimer_is_queued(timer) &&
	    __this_cpu_read(lock_boost_timer_owner) == current) {
		return;
	}

	hrtimer_cancel(timer);

	prev_owner = __this_cpu_read(lock_boost_timer_owner);
	if (prev_owner) {
		put_task_struct(prev_owner);
		__this_cpu_write(lock_boost_timer_owner, NULL);
	}

	if (!IS_ERR_OR_NULL(ots))
		set_bit(LOCK_BOOST_COUNT_BIT, &ots->lkinfo.state_flags);

	get_task_struct(current);
	__this_cpu_write(lock_boost_timer_owner, current);
	hrtimer_start(timer, ns_to_ktime(delay_ns), HRTIMER_MODE_REL_HARD);
}

static void hrtick_local_cancel(void)
{
	struct task_struct *owner;

	hrtimer_cancel(this_cpu_ptr(&lock_boost_timer));

	owner = __this_cpu_read(lock_boost_timer_owner);
	if (owner) {
		put_task_struct(owner);
		__this_cpu_write(lock_boost_timer_owner, NULL);
	}
}

#endif

static bool task_is_lock_owner(struct task_struct *prev)
{
	struct sched_boost_shm *shm = get_proc_shm();
	pid_t pid;

	if (!shm)
		return false;

	pid = prev->pid;
	return (READ_ONCE(shm->flag1) && READ_ONCE(shm->tid1) == (__u32)pid) ||
	       (READ_ONCE(shm->flag2) && READ_ONCE(shm->tid2) == (__u32)pid) ||
	       (READ_ONCE(shm->flag3) && READ_ONCE(shm->tid3) == (__u32)pid);
}

static bool lock_delay_resched(struct oplus_task_struct *ots)
{
	struct hrtimer *timer = this_cpu_ptr(&lock_boost_timer);

	if (unlikely(!locking_opt_enable(LK_LOCK_BOOST_ENABLE)))
		return false;

	if (!IS_ENABLED(CONFIG_SCHED_HRTICK))
		return false;

	if (!sysctl_sched_preempt_delay_us)
		return false;

	if (!(hrtimer_is_queued(timer)) && test_bit(LOCK_BOOST_COUNT_BIT, &ots->lkinfo.state_flags)) {
		return false;
	}

	if (test_bit(LOCK_BOOST_TICKOUT_BIT, &ots->lkinfo.state_flags)) {
		return false;
	}

	return true;
}

void lock_delay_resched_fini(void)
{
#ifdef CONFIG_SCHED_HRTICK
	if (sysctl_sched_preempt_delay_us)
		hrtick_local_start(sysctl_sched_preempt_delay_us * NSEC_PER_USEC);
#endif
}

void handle_lock_boost_statistics(bool *ext_slice)
{
	unsigned int cpu;
	int boost_flag = 0;

	if (*ext_slice) {
		boost_flag = 1;
	} else {
		boost_flag = 0;
	}

	cpu = smp_processor_id();

	if (per_cpu(prev_boost_flag, cpu) != boost_flag) {
		char buf[256];

		snprintf(buf, sizeof(buf), "C|9999|Cpu%d_lock_boost|%d\n",
				cpu, boost_flag);
		tracing_mark_write(buf);
		per_cpu(prev_boost_flag, cpu) = boost_flag;
	}
}


static void lock_delay_schedule(struct task_struct *prev, struct oplus_task_struct *ots)
{
#ifdef CONFIG_SCHED_HRTICK
	hrtick_local_cancel();

	clear_bit(LOCK_BOOST_TICKOUT_BIT, &ots->lkinfo.state_flags);
#endif
}

static void android_vh_lock_delay_schedule(void *unused, struct task_struct *prev,
	int sched_mode, bool *ext_slice)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(prev);

	if (IS_ERR_OR_NULL(ots)) {
		*ext_slice = false;
		return;
	}

	if ((sched_mode & SM_PREEMPT) && task_is_lock_owner(prev) &&
		lock_delay_resched(ots)) {
		clear_tsk_need_resched(prev);
		clear_preempt_need_resched();
		*ext_slice = true;
		lock_delay_resched_fini();
	} else {
		*ext_slice = false;
	}
	if (sysctl_lock_boost_debug_enable)
		handle_lock_boost_statistics(ext_slice);
}

static void android_vh_lock_task_fork(void *unused, struct task_struct *p)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(p);

	if (!IS_ERR_OR_NULL(ots)) {
		clear_bit(LOCK_BOOST_TICKOUT_BIT, &ots->lkinfo.state_flags);
		clear_bit(LOCK_BOOST_COUNT_BIT, &ots->lkinfo.state_flags);
	}
}

static void android_vh_lock_task_exit(void *unused, struct task_struct *tsk)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(tsk);

	if (!IS_ERR_OR_NULL(ots)) {
		clear_bit(LOCK_BOOST_TICKOUT_BIT, &ots->lkinfo.state_flags);
		clear_bit(LOCK_BOOST_COUNT_BIT, &ots->lkinfo.state_flags);
	}
}

static void lock_boost_android_rvh_schedule(struct task_struct *prev,
	struct task_struct *next, struct rq *rq)
{
	struct oplus_task_struct *ots = get_oplus_task_struct(prev);

	if (IS_ERR_OR_NULL(ots))
		return;

	lock_delay_schedule(prev, ots);

	if (prev != next) {
		clear_bit(LOCK_BOOST_COUNT_BIT, &ots->lkinfo.state_flags);
	}
}

static int register_lock_sched_vendor_hooks(void)
{
	register_trace_android_vh_lock_delay_schedule(android_vh_lock_delay_schedule, NULL);
	register_trace_android_vh_lock_task_fork(android_vh_lock_task_fork, NULL);
	register_trace_android_vh_lock_task_exit(android_vh_lock_task_exit, NULL);
	return 0;
}

static void unregister_lock_sched_vendor_hooks(void)
{
	unregister_trace_android_vh_lock_delay_schedule(android_vh_lock_delay_schedule, NULL);
	unregister_trace_android_vh_lock_task_exit(android_vh_lock_task_exit, NULL);
	unregister_trace_android_vh_lock_task_fork(android_vh_lock_task_fork, NULL);

	pr_info("lock_boost: vendor hooks unregistered\n");
}

void register_lock_boost_vendor_hooks(void)
{
#ifdef CONFIG_SCHED_HRTICK
	lock_boost_timer_init();
#endif
	oplus_lock_boost_android_rvh_schedule_hook = lock_boost_android_rvh_schedule;
	register_lock_sched_vendor_hooks();
	pr_info("lock_boost: AMS/WMS lock protection vendor hooks registered\n");
}
EXPORT_SYMBOL(register_lock_boost_vendor_hooks);

void unregister_lock_boost_vendor_hooks(void)
{
#ifdef CONFIG_SCHED_HRTICK
	int cpu;
#endif

	oplus_lock_boost_android_rvh_schedule_hook = NULL;
	unregister_lock_sched_vendor_hooks();

#ifdef CONFIG_SCHED_HRTICK
	for_each_possible_cpu(cpu) {
		struct task_struct *owner;

		hrtimer_cancel(&per_cpu(lock_boost_timer, cpu));

		owner = per_cpu(lock_boost_timer_owner, cpu);
		if (owner) {
			put_task_struct(owner);
			per_cpu(lock_boost_timer_owner, cpu) = NULL;
		}
	}
#endif
}
EXPORT_SYMBOL(unregister_lock_boost_vendor_hooks);
