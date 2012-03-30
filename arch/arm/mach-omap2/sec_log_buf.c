/**
 * arch/arm/mach-omap2/sec_log_buf.c
 *
 * Copyright (C) 2010-2011, Samsung Electronics, Co., Ltd. All Rights Reserved.
 *  Written by System S/W Group, S/W Platform R&D Team,
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
 * File Name : sec_log_buf.c
 *
 * File Description :
 *
 * Author : Kernel System Part
 * Dept : System S/W Group (Open OS S/W R&D Team)
 * Created : 29/Jun/2011
 * Version : Baby-Raccoon
 */

#include <linux/io.h>
#include <linux/bootmem.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <mach/hardware.h>
#include <mach/sec_log_buf.h>
#include <mach/sec_getlog.h>

static struct sec_log_buf s_log_buf;
struct device *sec_log_dev;
EXPORT_SYMBOL(sec_log_dev);

extern struct class *sec_class;

extern struct sec_log_buf_inf *log_buf_inf;

static ssize_t sec_log_buf_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return *log_buf_inf->log_buf_len;
}

static DEVICE_ATTR(log, S_IRUGO | S_IWUSR | S_IWGRP, sec_log_buf_show, NULL);

static unsigned int sec_log_buf_start = 0;
static unsigned int sec_log_buf_size = 0;
const unsigned int sec_log_buf_flag_size = (4 * 1024);
const unsigned int sec_log_buf_magic = 0x404C4F47;	/* @LOG */

static int __init sec_log_buf_setup(char *str)
{
	sec_log_buf_size = memparse(str, &str);

	if (sec_log_buf_size && (*str == '@')) {
		sec_log_buf_start = simple_strtoul(++str, &str, 0);
		if (reserve_bootmem(sec_log_buf_start, sec_log_buf_size, BOOTMEM_EXCLUSIVE)) {
			pr_err("failed to reserve size %d@0x%X\n",
			       sec_log_buf_size / 1024, sec_log_buf_start);
			sec_log_buf_start = 0;
			sec_log_buf_size = 0;
			goto __return;
		}
	}

__return:
	return 1;
}

__setup("sec_log=", sec_log_buf_setup);

void sec_log_buf_init(void)
{
	char *start;
	int i, count, copy_log_len, copy_log_start;
	unsigned *log_start_p = log_buf_inf->log_start;
	unsigned *con_start_p = log_buf_inf->con_start;
	unsigned *log_end_p = log_buf_inf->log_end;
	char **log_buf_p = log_buf_inf->log_buf;
	int *log_buf_len_p = log_buf_inf->log_buf_len;
	int log_buf_mask = *log_buf_len_p - 1;

	if (sec_log_buf_start == 0 || sec_log_buf_size == 0)
		return;

	start = (char *)ioremap(sec_log_buf_start, sec_log_buf_size);

	s_log_buf.flag = (unsigned int *)start;
	s_log_buf.count = (unsigned int *)(start + 4);
	s_log_buf.data = (char *)(start + sec_log_buf_flag_size);

	sec_log_dev = device_create(sec_class, NULL, 0, NULL, "sec_log");
	if (IS_ERR(sec_log_dev))
		pr_err("Failed to create device(sec_log)!\n");

	if (device_create_file(sec_log_dev, &dev_attr_log))
		pr_err("Failed to create device file(log)!\n");

	if (*s_log_buf.flag == sec_log_buf_magic) {
		if (*log_end_p < *log_buf_len_p) {
			copy_log_start = 0;
			copy_log_len = *log_end_p;
		} else {
			copy_log_start = *log_end_p;
			copy_log_len = *log_buf_len_p;
		}

		count = (*s_log_buf.count & log_buf_mask);

		for (i = 0; i < copy_log_len; i++) {
			*(s_log_buf.data + ((count + i) & log_buf_mask)) =
			    *(*log_buf_p + ((copy_log_start + i) & log_buf_mask));
		}

		*s_log_buf.count =
		    ((*s_log_buf.count + copy_log_len) & log_buf_mask);

		*log_buf_p = s_log_buf.data;

		*log_start_p = (*log_start_p + count);
		*con_start_p = (*con_start_p + count);
		*log_end_p = (*log_end_p + count);
	}

	sec_getlog_supply_kloginfo((void *)(sec_log_buf_start +
					    sec_log_buf_flag_size));
}

void sec_log_buf_update(void)
{
	if (s_log_buf.count)
		(*s_log_buf.count)++;
}
