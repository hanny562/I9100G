/**
 * arch/arm/mach-omap2/include/mach/sec_gaf.h
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
 * File Name : sec_gaf.h
 *
 * File Description :
 *
 * Author : Kernel System Part
 * Dept : System S/W Group (Open OS S/W R&D Team)
 * Created : 29/Jun/2011
 * Version : Baby-Raccoon
 */

#ifndef __SEC_GAF_H__
#define __SEC_GAF_H__

#if defined (CONFIG_SAMSUNG_ADD_GAFORENSICINFO)

extern void sec_gaf_supply_rqinfo(unsigned short curr_offset,
				  unsigned short rq_offset);

#else /* CONFIG_SAMSUNG_ADD_GAFORENSICINFO */

/*
#define sec_gaf_supply_rqinfo(curr_offset, rq_offset)
 */

static inline void sec_gaf_supply_rqinfo(unsigned short curr_offset,
					 unsigned short rq_offset)
{
}

#endif /* CONFIG_SAMSUNG_ADD_GAFORENSICINFO */

#endif /* __SEC_GAF_H__ */