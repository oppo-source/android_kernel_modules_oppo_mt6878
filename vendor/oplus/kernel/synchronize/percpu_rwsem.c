#include <linux/sched/cputime.h>
#include <linux/sched.h>
#include <linux/sched/rt.h>
#include <linux/list.h>
#include <../kernel/sched/sched.h>
#include <trace/hooks/dtask.h>
#include <linux/rwsem.h>
#include <linux/percpu.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_SCHED_ASSIST)
#include <../kernel/oplus_cpu/sched/sched_assist/sa_common.h>
#endif

#include "locking_main.h"
#include "percpu_rwsem.h"

static atomic64_t priority_reader_bypass_count = ATOMIC64_INIT(0);
static atomic64_t priority_reader_blocked_count = ATOMIC64_INIT(0);
static atomic64_t bypass_delayed_failure_count = ATOMIC64_INIT(0);

extern struct proc_dir_entry *d_oplus_locking;

static int percpu_rwsem_stats_show(struct seq_file *m, void *v)
{
    u64 bypass_count = atomic64_read(&priority_reader_bypass_count);
    u64 blocked_count = atomic64_read(&priority_reader_blocked_count);
    u64 delayed_failure_count = atomic64_read(&bypass_delayed_failure_count);
    u64 total_attempts = bypass_count + blocked_count;
    int bypass_rate = total_attempts ? (bypass_count * 100) / total_attempts : 0;
    int delayed_failure_rate = bypass_count ? (delayed_failure_count * 100) / bypass_count : 0;

    seq_printf(m, "Percpu-RWSEM Priority Reader Statistics\n");
    seq_printf(m, "======================================\n");
    seq_printf(m, "Feature Status: %s\n",
               locking_opt_enable(LK_PCPU_RWSEM_RT_ENABLE) ? "Enabled" : "Disabled");
    seq_printf(m, "\n");

    seq_printf(m, "Priority Reader Bypass Count:  %llu\n", bypass_count);
    seq_printf(m, "Priority Reader Blocked Count: %llu\n", blocked_count);
    seq_printf(m, "Bypass Delayed Failure Count:  %llu\n", delayed_failure_count);
    seq_printf(m, "Total Priority Attempts:       %llu\n", total_attempts);
    seq_printf(m, "Bypass Success Rate:           %d%%\n", bypass_rate);
    seq_printf(m, "Bypass Delayed Failure Rate:   %d%%\n", delayed_failure_rate);
    seq_printf(m, "\n");

    if (total_attempts > 0) {
        seq_printf(m, "Performance Analysis:\n");
        seq_printf(m, "  - Priority tasks attempted lock: %llu times\n", total_attempts);
        seq_printf(m, "  - Successful bypasses: %llu times (reduced latency)\n", bypass_count);
        seq_printf(m, "  - Blocked priority tasks: %llu times (normal behavior)\n", blocked_count);
        seq_printf(m, "  - Bypass delayed failures: %llu times (check_blocked_writer_delayed=FALSE)\n", delayed_failure_count);
        seq_printf(m, "  - Overall effectiveness: %d%% of priority tasks benefited\n", bypass_rate);
        seq_printf(m, "  - Bypass delayed failure rate: %d%% of successful bypasses\n", delayed_failure_rate);
    }

    seq_printf(m, "\nLast Updated: %lu jiffies\n", jiffies);

    return 0;
}

static int percpu_rwsem_stats_open(struct inode *inode, struct file *file)
{
    return single_open(file, percpu_rwsem_stats_show, NULL);
}

static const struct proc_ops percpu_rwsem_stats_proc_ops = {
    .proc_open = percpu_rwsem_stats_open,
    .proc_read = seq_read,
    .proc_lseek = seq_lseek,
    .proc_release = single_release,
};

static int create_percpu_rwsem_proc_entry(void)
{
    struct proc_dir_entry *entry;

    if (!d_oplus_locking) {
        pr_err("d_oplus_locking directory not created yet\n");
        return -ENOENT;
    }

    entry = proc_create("percpu_rwsem_stats", 0444, d_oplus_locking, &percpu_rwsem_stats_proc_ops);
    if (!entry) {
        pr_err("Failed to create percpu_rwsem_stats proc entry\n");
        return -ENOMEM;
    }

    pr_info("Percpu-RWSEM stats proc entry created: /proc/oplus_locking/percpu_rwsem_stats\n");
    return 0;
}

static void remove_percpu_rwsem_proc_entry(void)
{
    if (d_oplus_locking) {
        remove_proc_entry("percpu_rwsem_stats", d_oplus_locking);
        pr_info("Percpu-RWSEM stats proc entry removed\n");
    }
}

/* True if per-CPU read_count sum is zero; assumes sem->block is set. */
static bool readers_active_check(struct percpu_rw_semaphore *sem)
{
	if (per_cpu_sum(*sem->read_count) != 0)
		return false;

	/*
	 * If we observed the decrement; ensure we see the entire critical
	 * section.
	 */

	smp_mb(); /* C matches B */

	return true;
}

static bool check_blocked_writer_delayed(struct percpu_rw_semaphore *sem)
{
    struct task_struct *p;
    struct oplus_task_struct *ots;
    bool ret = false;
    rcu_read_lock();
    p = rcu_dereference(sem->writer.task);
    if (!p)
        goto unlock_err;
    ots = get_oplus_task_struct(p);
    if (!ots)
        goto unlock_err;
    if (!ots->pcp_timeout_jiffies)
        goto unlock_err;
    ret = (jiffies >= ots->pcp_timeout_jiffies);
    rcu_read_unlock();

    return ret;

unlock_err:
    rcu_read_unlock();
    return false;
}

static bool should_rt_reader_bypass(struct percpu_rw_semaphore *sem)
{
    if (!rt_task(current) && !test_task_ux(current))
        return false;

    if (!atomic_read(&sem->block))
        return false;

    if (readers_active_check(sem))
        return false;

    if (check_blocked_writer_delayed(sem)) {
        atomic64_inc(&bypass_delayed_failure_count);
        return false;
    }

    return true;
}

static void rt_reader_bypass_handler(struct percpu_rw_semaphore *sem, bool *ret)
{
    this_cpu_inc(*sem->read_count);

    smp_mb();

    if (likely(!atomic_read_acquire(&sem->block))) {
        this_cpu_dec(*sem->read_count);
        return;
    }
    *ret = true;
}

static void android_vh_percpu_rwsem_down_read_handler(void *unused,
                                                     struct percpu_rw_semaphore *sem,
                                                     bool try, bool *ret)
{
    if (!locking_opt_enable(LK_PCPU_RWSEM_RT_ENABLE))
        return;

    if (*ret)
        return;

    if (should_rt_reader_bypass(sem)) {
        rt_reader_bypass_handler(sem, ret);
        atomic64_inc(&priority_reader_bypass_count);
        pr_info("Task %d pcp_rwsem reader bypassed for sem %p\n", current->pid, sem);
    } else if (rt_task(current) || test_task_ux(current)) {
        atomic64_inc(&priority_reader_blocked_count);
    }
}

static void android_vh_percpu_rwsem_up_write_handler(void *unused, struct percpu_rw_semaphore *sem)
{
    struct oplus_task_struct *ots = get_oplus_task_struct(current);
    if (!ots)
        return;
    ots->pcp_timeout_jiffies = 0;
}

static void android_rvh_percpu_rwsem_wait_complete_handler(void *unused,
                                                          struct percpu_rw_semaphore *sem,
                                                          long state, bool *complete)
{
    u64 timeout_threshold = TIMEOUT_NORMAL;
    struct oplus_task_struct *ots;

    if (!locking_opt_enable(LK_PCPU_RWSEM_RT_ENABLE))
        return;

    /* Blocked writer is current; sem->writer.task may not be set yet. */
    ots = get_oplus_task_struct(current);
    if (!ots)
        return;

    if (test_task_ux(current))
        timeout_threshold = TIMEOUT_UX;
    if (rt_task(current))
        timeout_threshold = TIMEOUT_RT;
    ots->pcp_timeout_jiffies = jiffies + timeout_threshold;
}

void register_percpu_rwsem_vendor_hooks(void)
{
    register_trace_android_vh_percpu_rwsem_down_read(android_vh_percpu_rwsem_down_read_handler, NULL);
    register_trace_android_vh_percpu_rwsem_up_write(android_vh_percpu_rwsem_up_write_handler, NULL);
    register_trace_android_rvh_percpu_rwsem_wait_complete(android_rvh_percpu_rwsem_wait_complete_handler, NULL);

    pr_info("percpu_rwsem vendor hooks registered with RT reader priority support\n");

    if (create_percpu_rwsem_proc_entry())
        pr_err("Failed to create percpu_rwsem proc entry, but hooks registered\n");
}

void unregister_percpu_rwsem_vendor_hooks(void)
{
    unregister_trace_android_vh_percpu_rwsem_down_read(android_vh_percpu_rwsem_down_read_handler, NULL);
    unregister_trace_android_vh_percpu_rwsem_up_write(android_vh_percpu_rwsem_up_write_handler, NULL);

    remove_percpu_rwsem_proc_entry();
}
