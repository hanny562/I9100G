/**
 * arch/arm/mach-omap2/include/mach/sec_getlog.h
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
 * File Name : sec_getlog.c
 *
 * File Description :
 *
 * Author : Kernel System Part
 * Dept : System S/W Group (Open OS S/W R&D Team)
 * Created : 29/Jun/2011
 * Version : Baby-Raccoon
 */

#ifndef __SEC_GETLOG_H__
#define __SEC_GETLOG_H__

#if defined(CONFIG_SAMSUNG_USE_GETLOG)

extern void sec_getlog_supply_fbinfo(void *p_fb, u32 res_x, u32 res_y,
				     u32 bpp, u32 frames);

extern void sec_getlog_supply_meminfo(u32 size0, u32 addr0,
				      u32 size1, u32 addr1);

extern void sec_getlog_supply_loggerinfo(void *p_main, void *p_radio,
					 void *p_events, void *p_system);

extern void sec_getlog_supply_kloginfo(void *klog_buf);

#else

static inline void sec_getlog_supply_fbinfo(void *p_fb, u32 res_x, u32 res_y,
					    u32 bpp, u32 frames)
{
}

static inline void sec_getlog_supply_meminfo(u32 size0, u32 addr0,
					     u32 size1, u32 addr1)
{
}

static inline void sec_getlog_supply_loggerinfo(void *p_main, void *p_radio,
						void *p_events, void *p_system)
{
}

static inline void sec_getlog_supply_kloginfo(void *klog_buf)
{
}

#endif /* CONFIG_SAMSUNG_USE_GETLOG */


#endif /* __SEC_GETLOG_H__ */
