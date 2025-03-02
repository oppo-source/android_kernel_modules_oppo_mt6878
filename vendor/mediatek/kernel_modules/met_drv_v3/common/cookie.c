// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/cpu.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/cpu.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/version.h>
#include <asm/irq_regs.h>
#include <asm/stacktrace.h>
#include <linux/stacktrace.h>
#include "interface.h"
#include "met_drv.h"
#include "mtk_typedefs.h"

#define LINE_SIZE	256

struct cookie_info {
	int depth;
	int strlen;
	char strbuf[LINE_SIZE];
};

static unsigned int back_trace_depth;
static struct cookie_info __percpu *info;
static int __percpu *cpu_status;

static int reset_driver_stat(void)
{
	back_trace_depth = 0;
	met_cookie.mode = 0;
	return 0;
}


noinline void cookie(char *strbuf)
{
	MET_TRACE("%s\n", strbuf);
}

noinline void cookie2(char *strbuf)
{
	MET_TRACE("%s\n", strbuf);
}

static void get_kernel_cookie(unsigned long pc, struct cookie_info *pinfo)
{
	int ret;

	ret =
	    SNPRINTF(pinfo->strbuf + pinfo->strlen, LINE_SIZE - pinfo->strlen,
		     ",vmlinux,%lx", pc);
	pinfo->strlen += ret;
	/* cookie(current->comm, pc, "vmlinux", pc, 0); */
}

#if defined(__arm__)
static int report_trace(struct stackframe *frame, void *d)
{
	struct cookie_info *pinfo = d;
	unsigned long pc = frame->pc;

	if (pinfo->depth > 0) {
		get_kernel_cookie(pc, pinfo);
		pinfo->depth--;
		return 0;
	}
	return 1;
}
#endif

static void kernel_backtrace(struct pt_regs *const regs, struct cookie_info *pinfo)
{
#if defined(__arm__)
	struct stackframe frame;

	frame.fp = regs->ARM_fp;
	frame.sp = regs->ARM_sp;
	frame.lr = regs->ARM_lr;
	frame.pc = regs->ARM_pc;
	walk_stackframe(&frame, report_trace, pinfo);
#else
	return;
#endif
}


static int met_cookie_create_subfs(struct kobject *parent)
{
	int ret = 0;

	info = alloc_percpu(typeof(*info));
	if (!info) {
		PR_BOOTMSG("percpu info allocate fail\n");
		pr_debug("percpu info allocate fail\n");
		return 0;
	}

	cpu_status = alloc_percpu(typeof(*cpu_status));
	if (!cpu_status) {
		PR_BOOTMSG("percpu cpu_status allocate fail\n");
		pr_debug("percpu cpu_status allocate fail\n");
		return 0;
	}

	return ret;
}


static void met_cookie_delete_subfs(void)
{
	if (info) {
		free_percpu(info);
	}
	if (cpu_status) {
		free_percpu(cpu_status);
	}
}


void met_cookie_polling(unsigned long long stamp, int cpu)
{
	struct pt_regs *regs;
	struct cookie_info *pinfo;
	unsigned long pc;
	int ret, outflag = 0;
	off_t off;
#if KERNEL_VERSION(6, 0, 0) <= LINUX_VERSION_CODE
	MA_STATE(mas, 0, 0, 0);
#endif

	if (*per_cpu_ptr(cpu_status, cpu) != MET_CPU_ONLINE)
		return;

	regs = get_irq_regs();

	if (regs == 0)
		return;

	pc = profile_pc(regs);

	pinfo = per_cpu_ptr(info, cpu);
	pinfo->strlen = SNPRINTF(pinfo->strbuf, LINE_SIZE, "%s,%lx", current->comm, pc);

	if (user_mode(regs)) {
		struct mm_struct *mm;
		struct vm_area_struct *vma;
		struct path *ppath;

		mm = current->mm;
#if KERNEL_VERSION(6, 0, 0) <= LINUX_VERSION_CODE
		mas.tree = &current->mm->mm_mt;
		mas_for_each(&mas, vma, ULONG_MAX) {
#else
		for (vma = find_vma(mm, pc); vma; vma = vma->vm_next) {
#endif
			if (pc < vma->vm_start || pc >= vma->vm_end)
				continue;

			if (vma->vm_file) {
				ppath = &(vma->vm_file->f_path);

				off = (vma->vm_pgoff << PAGE_SHIFT) + pc - vma->vm_start;

				ret =
				    SNPRINTF(pinfo->strbuf + pinfo->strlen,
					     LINE_SIZE - pinfo->strlen, ",%s,%lx",
					     (char *)(ppath->dentry->d_name.name), off);
				pinfo->strlen += ret;
				outflag = 1;
			} else {
				/* must be an anonymous map */
				ret =
				    SNPRINTF(pinfo->strbuf + pinfo->strlen,
					     LINE_SIZE - pinfo->strlen, ",nofile,%lx", pc);
				pinfo->strlen += ret;
				outflag = 1;
			}
			break;
		}
	} else {
		/* kernel mode code */
		if (back_trace_depth > 0) {
			pinfo->depth = back_trace_depth + 1;
			kernel_backtrace(regs, pinfo);
		} else
			get_kernel_cookie(pc, pinfo);
		outflag = 1;
	}

	/* check task is resolvable */
	if (outflag == 0)
		return;

	if (back_trace_depth == 0)
		cookie(pinfo->strbuf);
	else
		cookie2(pinfo->strbuf);
}


static void met_cookie_start(void)
{
	int	cpu = raw_smp_processor_id();

	if (!info || !cpu_status) {
		MET_TRACE("percpu info/cpu_status allocate fail\n");
		reset_driver_stat();
		return;
	}

	*per_cpu_ptr(cpu_status, cpu) = MET_CPU_ONLINE;
	/* return; */
}

static void met_cookie_stop(void)
{
	/* return; */
}


static int met_cookie_process_argument(const char *arg, int len)
{
	unsigned int value = 0;

	if (!info || !cpu_status)
		return 0;

	if (met_parse_num(arg, &value, len) < 0) {
		met_cookie.mode = 0;
		return -EINVAL;
	}

	back_trace_depth = value;
	met_cookie.mode = 1;

	return 0;
}

static const char help[] =
"  --cookie                              enable sampling task and PC\n"
"  --cookie=N                            enable back trace (depth is N)\n";

static int met_cookie_print_help(char *buf, int len)
{
	len = SNPRINTF(buf, PAGE_SIZE, help);
	return len;
}


static const char header[] =
"# cookie: task_name,PC,cookie_name,offset\n"
"met-info [000] 0.0: cookie_header: task_name,PC,cookie_name,offset\n";

static const char header2_1[] = "# cookie2: task_name,PC,cookie,offset";
static const char header2_2[] = "met-info [000] 0.0: cookie2_header: task_name,PC,cookie,offset";

static int met_cookie_print_header(char *buf, int len)
{
	int i, ret;

	if (back_trace_depth == 0) {
		len = SNPRINTF(buf, PAGE_SIZE, header);
	} else {
		len = SNPRINTF(buf, PAGE_SIZE, header2_1);
		for (i = 0; i < back_trace_depth; i++) {
			ret = SNPRINTF(buf + len, PAGE_SIZE, ",cookie%d,offset%d", i + 1, i + 1);
			len += ret;
		}
		ret = SNPRINTF(buf + len, PAGE_SIZE, "\n");
		len += ret;

		ret = SNPRINTF(buf + len, PAGE_SIZE, header2_2);
		len += ret;
		for (i = 0; i < back_trace_depth; i++) {
			ret = SNPRINTF(buf + len, PAGE_SIZE, ",cookie%d,offset%d", i + 1, i + 1);
			len += ret;
		}
		ret = SNPRINTF(buf + len, PAGE_SIZE, "\n");
		len += ret;
	}

	return len;
}

static void met_cookie_cpu_state_notify(long cpu, unsigned long action)
{
	if (!info || !cpu_status)
		return;

	*per_cpu_ptr(cpu_status, cpu) = action;
}

struct metdevice met_cookie = {
	.name = "cookie",
	.type = MET_TYPE_PMU,
	.cpu_related = 1,
	.create_subfs = met_cookie_create_subfs,
	.delete_subfs = met_cookie_delete_subfs,
	.start = met_cookie_start,
	.stop = met_cookie_stop,
	.reset = reset_driver_stat,
	.polling_interval = 1,
	.timed_polling = met_cookie_polling,
	.process_argument = met_cookie_process_argument,
	.print_help = met_cookie_print_help,
	.print_header = met_cookie_print_header,
	.cpu_state_notify = met_cookie_cpu_state_notify,
};
