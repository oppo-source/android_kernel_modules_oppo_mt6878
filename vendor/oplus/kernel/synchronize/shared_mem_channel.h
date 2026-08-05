/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _UAPI_LINUX_SCHED_BOOST_H
#define _UAPI_LINUX_SCHED_BOOST_H

#include <linux/types.h>

#define SCHED_BOOST_VERSION  3

struct sched_boost_shm {
	__u32 flag1;
	__u32 tid1;
	__u32 flag2;
	__u32 tid2;
	__u32 flag3;
	__u32 tid3;
	__u32 version;
	__u64 timestamp_ns;
	__u32 reserved[7];
} __attribute__((aligned(64)));

struct sched_boost_shm *get_proc_shm(void);

int sched_boost_init(void);
void sched_boost_exit(void);

#endif
