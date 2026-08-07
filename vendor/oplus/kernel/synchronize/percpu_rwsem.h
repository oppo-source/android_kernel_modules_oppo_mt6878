// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2022 Oplus. All rights reserved.
 */

#ifndef __OPLUS_PERCPU_RWSEM_H__
#define __OPLUS_PERCPU_RWSEM_H__

#include <linux/percpu-rwsem.h>

enum JIFFIES_TIMEOUT_THRESHOLD {
    TIMEOUT_NORMAL = 8,
    TIMEOUT_UX = 2,
    TIMEOUT_RT = 0,
};

#define per_cpu_sum(var)						\
({									\
    typeof(var) __sum = 0;						\
    int cpu;							\
    compiletime_assert_atomic_type(__sum);				\
    for_each_possible_cpu(cpu)					\
        __sum += per_cpu(var, cpu);				\
    __sum;								\
})


#endif /* __OPLUS_PERCPU_RWSEM_H__ */
