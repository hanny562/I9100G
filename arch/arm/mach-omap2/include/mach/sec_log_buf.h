/**
 * arch/arm/mach-omap2/include/mach/sec_log_buf.h
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
 * File Name : sec_log_buf.h
 *
 * File Description :
 *
 * Author : System Platform 2
 * Dept : System S/W Group (Open OS S/W R&D Team)
 * Created : 04/Apr/2011
 * Version : Baby-Raccoon
 */

#ifndef __SEC_LOG_BUF_H__
#define __SEC_LOG_BUF_H__

struct sec_log_buf {
	unsigned int *flag;
	unsigned int *count;
	char *data;
};

struct sec_log_buf_inf {
	unsigned *log_start;	/* ptr of log_start */
	unsigned *con_start;	/* ptr of con_start */
	unsigned *log_end;	/* ptr of log_end */
	char **log_buf;		/* ptr of log buffer */
	int *log_buf_len;	/* ptr of log_buf_len */
};

#ifdef CONFIG_SAMSUNG_USE_SEC_LOG_BUF

void sec_log_buf_init(void);

void sec_log_buf_update(void);

#else

/*
#define sec_log_buf_init(log_buf_inf)
#define sec_log_buf_update()
 */

static void sec_log_buf_init(void)
{
}

static void sec_log_buf_update(void)
{
}

#endif /* CONFIG_SAMSUNG_USE_SEC_LOG_BUF */

#endif /* __SEC_LOG_BUF_H__ */
