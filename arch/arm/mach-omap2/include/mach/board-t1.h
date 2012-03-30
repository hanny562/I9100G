/**
 * arch/arm/mach-omap2/include/mach/board-t1.h
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
 * File Name : board-t1.h
 *
 * File Description :
 *
 * Author : System Platform 2
 * Dept : System S/W Group (Open OS S/W R&D Team)
 * Created : 07/Mar/2011
 * Version : Baby-Raccoon
 */

#ifndef __BOARD_T1_H__
#define __BOARD_T1_H__
#ifdef CONFIG_MACH_SAMSUNG_T1 /* T1 hostwakeup */
	typedef irqreturn_t (*wl_isr_t)(int irq, void* dev_id);
#endif

#endif /* __BOARD_T1_H__ */

