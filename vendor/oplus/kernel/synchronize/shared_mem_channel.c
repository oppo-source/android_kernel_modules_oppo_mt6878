// SPDX-License-Identifier: GPL-2.0

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include <../kernel/sched/sched.h>
#include <../kernel/oplus_cpu/sched/sched_assist/sa_common.h>
#include "shared_mem_channel.h"

struct sched_boost_ctx {
	struct task_struct *owner;
};

#define SCHED_BOOST_DEVICE_NAME "sched_boost"

static int sched_boost_major;
static struct class *sched_boost_class;
static struct device *sched_boost_device;

static struct page          *g_shm_page;
static struct sched_boost_shm *g_shm_vaddr;

static struct sched_boost_ctx *g_proc_ctx;

struct sched_boost_shm *get_proc_shm(void)
{
	return READ_ONCE(g_shm_vaddr);
}
EXPORT_SYMBOL(get_proc_shm);

static int sched_boost_open(struct inode *inode, struct file *file)
{
	struct sched_boost_ctx *ctx;

	if (!READ_ONCE(g_shm_vaddr)) {
		return -ENXIO;
	}

	if (READ_ONCE(g_proc_ctx)) {
		return -EBUSY;
	}

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	WRITE_ONCE(g_shm_vaddr->flag1, 0);
	WRITE_ONCE(g_shm_vaddr->tid1, 0);
	WRITE_ONCE(g_shm_vaddr->flag2, 0);
	WRITE_ONCE(g_shm_vaddr->tid2, 0);
	WRITE_ONCE(g_shm_vaddr->flag3, 0);
	WRITE_ONCE(g_shm_vaddr->tid3, 0);
	WRITE_ONCE(g_shm_vaddr->timestamp_ns, 0);

	ctx->owner = get_task_struct(current);
	file->private_data = ctx;
	WRITE_ONCE(g_proc_ctx, ctx);

	return 0;
}

static int sched_boost_release(struct inode *inode, struct file *file)
{
	struct sched_boost_ctx *ctx = file->private_data;

	if (!ctx)
		return 0;

	cmpxchg(&g_proc_ctx, ctx, (struct sched_boost_ctx *)NULL);

	if (READ_ONCE(g_shm_vaddr)) {
		WRITE_ONCE(g_shm_vaddr->flag1, 0);
		WRITE_ONCE(g_shm_vaddr->tid1,  0);
		WRITE_ONCE(g_shm_vaddr->flag2, 0);
		WRITE_ONCE(g_shm_vaddr->tid2,  0);
		WRITE_ONCE(g_shm_vaddr->flag3, 0);
		WRITE_ONCE(g_shm_vaddr->tid3,  0);
		WRITE_ONCE(g_shm_vaddr->timestamp_ns, 0);
	}

	put_task_struct(ctx->owner);
	kfree(ctx);

	return 0;
}

static void sched_boost_vma_close(struct vm_area_struct *vma)
{
	struct page *page = vma->vm_private_data;

	if (page)
		put_page(page);
}

static const struct vm_operations_struct sched_boost_vm_ops = {
	.close = sched_boost_vma_close,
};

static int sched_boost_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned long size = vma->vm_end - vma->vm_start;
	int ret;

	if (!g_shm_page)
		return -ENXIO;

	if (size != PAGE_SIZE)
		return -EINVAL;

	if (!(vma->vm_flags & VM_SHARED))
		return -EINVAL;

	vm_flags_set(vma, VM_DONTEXPAND | VM_DONTDUMP | VM_DONTCOPY);

	get_page(g_shm_page);
	vma->vm_private_data = g_shm_page;
	vma->vm_ops = &sched_boost_vm_ops;

	ret = remap_pfn_range(vma, vma->vm_start,
			      page_to_pfn(g_shm_page),
			      size, vma->vm_page_prot);
	if (ret) {
		pr_err("sched_boost: mmap failed for task %d: %d\n",
		       current->pid, ret);
		put_page(g_shm_page);
		vma->vm_private_data = NULL;
		return ret;
	}

	return 0;
}

static const struct file_operations sched_boost_fops = {
	.owner   = THIS_MODULE,
	.open    = sched_boost_open,
	.release = sched_boost_release,
	.mmap    = sched_boost_mmap,
};

int sched_boost_init(void)
{
	struct page *page;
	struct sched_boost_shm *shm;
	int ret;

	page = alloc_page(GFP_KERNEL | __GFP_ZERO);
	if (!page) {
		pr_err("sched_boost: failed to allocate SHM page\n");
		return -ENOMEM;
	}

	shm = page_address(page);
	if (!shm) {
		__free_page(page);
		pr_err("sched_boost: page_address returned NULL\n");
		return -ENOMEM;
	}

	shm->version = SCHED_BOOST_VERSION;
	g_shm_page = page;

	WRITE_ONCE(g_shm_vaddr, shm);

	ret = register_chrdev(0, SCHED_BOOST_DEVICE_NAME, &sched_boost_fops);
	if (ret < 0) {
		pr_err("sched_boost: failed to register chrdev: %d\n", ret);
		goto err_chrdev;
	}
	sched_boost_major = ret;

	sched_boost_class = class_create(THIS_MODULE, SCHED_BOOST_DEVICE_NAME);
	if (IS_ERR(sched_boost_class)) {
		ret = PTR_ERR(sched_boost_class);
		pr_err("sched_boost: failed to create class\n");
		goto err_class;
	}

	sched_boost_device = device_create(sched_boost_class, NULL,
					   MKDEV(sched_boost_major, 0),
					   NULL, SCHED_BOOST_DEVICE_NAME);
	if (IS_ERR(sched_boost_device)) {
		ret = PTR_ERR(sched_boost_device);
		pr_err("sched_boost: failed to create device\n");
		goto err_device;
	}

	pr_info("sched_boost: initialized, major=%d\n", sched_boost_major);
	return 0;

err_device:
	class_destroy(sched_boost_class);
err_class:
	unregister_chrdev(sched_boost_major, SCHED_BOOST_DEVICE_NAME);
err_chrdev:
	WRITE_ONCE(g_shm_vaddr, NULL);
	__free_page(page);
	g_shm_page = NULL;
	return ret;
}

void sched_boost_exit(void)
{
	/*
	 * Tear down the character device first so no new open() calls can
	 * arrive.  Any in-flight open() that passed the device check but has
	 * not yet returned will hold a module reference, keeping us here
	 * until it completes.
	 */
	device_destroy(sched_boost_class, MKDEV(sched_boost_major, 0));
	class_destroy(sched_boost_class);
	unregister_chrdev(sched_boost_major, SCHED_BOOST_DEVICE_NAME);

	WRITE_ONCE(g_shm_vaddr, NULL);
	if (g_shm_page) {
		put_page(g_shm_page);
		g_shm_page = NULL;
	}

	pr_info("sched_boost: unloaded\n");
}
