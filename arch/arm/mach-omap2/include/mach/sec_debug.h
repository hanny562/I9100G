/**
 * arch/arm/mach-omap2/include/mach/sec_debug.h
 *
 * Copyright (C) 2010-2011, Samsung Electronics, Co., Ltd. All Rights Reserved.
 *  Written by System S/W Group, Open OS S/W R&D Team,
 *  Mobile Communication Division.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/**
 * Project Name : OMAP-Samsung Linux Kernel for Android
 *
 * Project Description :
 *
 * Comments : tabstop = 8, shiftwidth = 8, noexpandtab
 */

/**
 * File Name : sec_debug.h
 *
 * File Description : import from U1
 *
 * Author : Kernel System Part
 * Dept : System S/W Group (Open OS S/W R&D Team)
 * Created : 30/May/2011
 * Version : Baby-Raccoon
 */

#ifndef SEC_DEBUG_H
#define SEC_DEBUG_H

#include <linux/sched.h>
#include <linux/semaphore.h>

#if defined(CONFIG_MACH_OMAP4_SAMSUNG)
/* for OMAP4 chips - use scratchpad */
#define SEC_DEBUG_MAGIC_ADDR		0x4A326FF0
#define SEC_DEBUG_UPLOAD_CAUSE_ADDR	(SEC_DEBUG_MAGIC_ADDR - 0x04)
#else
/* for LSI chips - use external SRAM base */
#define SEC_DEBUG_MAGCI_ADDR		0xC0000000
#endif /* CONFIG_MACH_* */

#define SEC_DEBUG_MAGIC_DUMP		0x66262564

#ifdef CONFIG_SEC_DEBUG
extern int sec_debug_init(void);

extern void sec_debug_check_crash_key(unsigned int code, int value);

#else
static inline int sec_debug_init(void)
{
}

static inline void sec_debug_check_crash_key(unsigned int code, int value)
{
}

#endif /* CONFIG_SEC_DEBUG */

#ifdef CONFIG_SEC_DEBUG_SCHED_LOG
extern void sec_debug_task_sched_log(int cpu, struct task_struct *task);
extern void sec_debug_irq_sched_log(unsigned int irq, void *fn, int en);
extern void sec_debug_sched_log_init(void);
#else
static inline void sec_debug_task_sched_log(int cpu, struct task_struct *task)
{
}

static inline void sec_debug_irq_sched_log(unsigned int irq, void *fn, int en)
{
}

static inline void sec_debug_sched_log_init(void)
{
}
#endif /* CONFIG_SEC_DEBUG_SCHED_LOG */

#ifdef CONFIG_SEC_DEBUG_IRQ_EXIT_LOG
extern void sec_debug_irq_last_exit_log(void);
#else
static void sec_debug_irq_last_exit_log(void)
{
}
#endif /* CONFIG_SEC_DEBUG_IRQ_EXIT_LOG */

#ifdef CONFIG_SEC_DEBUG_SEMAPHORE_LOG
extern void debug_semaphore_init(void);
extern void debug_semaphore_down_log(struct semaphore *sem);
extern void debug_semaphore_up_log(struct semaphore *sem);
extern void debug_rwsemaphore_init(void);
extern void debug_rwsemaphore_down_log(struct rw_semaphore *sem, int dir);
extern void debug_rwsemaphore_up_log(struct rw_semaphore *sem);
#define debug_rwsemaphore_down_read_log(x)	\
	debug_rwsemaphore_down_log(x, READ_SEM)
#define debug_rwsemaphore_down_write_log(x)	\
	debug_rwsemaphore_down_log(x ,WRITE_SEM)
#else
static inline void debug_semaphore_init(void)
{
}

static inline void debug_semaphore_down_log(struct semaphore *sem)
{
}

static inline void debug_semaphore_up_log(struct semaphore *sem)
{
}

static inline void debug_rwsemaphore_init(void)
{
}

static inline void debug_rwsemaphore_down_read_log(struct rw_semaphore *sem)
{
}

static inline void debug_rwsemaphore_down_write_log(struct rw_semaphore *sem)
{
}

static inline void debug_rwsemaphore_up_log(struct rw_semaphore *sem)
{
}
#endif /* CONFIG_SEC_DEBUG_SEMAPHORE_LOG */

/* klaatu - schedule log */
#ifdef CONFIG_SEC_DEBUG_SCHED_LOG
#define SCHED_LOG_MAX			4096

struct irq_log {
	int cpu;
	int irq;
	void *fn;
	int en;
};

struct task_info {
	char comm[TASK_COMM_LEN];
	int cpu;
	pid_t pid;
};

union task_log {
	struct task_info task;
	struct irq_log irq;
};

struct sched_log {
	unsigned long long time;
	union task_log log;
};
#endif /* CONFIG_SEC_DEBUG_SCHED_LOG */

#ifdef CONFIG_SEC_DEBUG_SEMAPHORE_LOG
#define SEMAPHORE_LOG_MAX		100

struct sem_debug {
	struct list_head list;
	struct semaphore *sem;
	struct task_struct *task;
	pid_t pid;
	int cpu;
	/* char comm[TASK_COMM_LEN]; */
};

enum {
	READ_SEM,
	WRITE_SEM
};

#define RWSEMAPHORE_LOG_MAX		100
struct rwsem_debug {
	struct list_head list;
	struct rw_semaphore *sem;
	struct task_struct *task;
	pid_t pid;
	int cpu;
	int direction;
	/* char comm[TASK_COMM_LEN]; */
};
#endif /* CONFIG_SEC_DEBUG_SEMAPHORE_LOG */

#endif /* SEC_DEBUG_H */
