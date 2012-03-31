/**
 * arch/arm/plat-omap/include/plat/mux_t1_rev_r01.h
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
 * File Name : mux_t1_rev_r01.h
 *
 * File Description :
 *
 * Author : System Platform 2
 * Dept : System S/W Group (Open OS S/W R&D Team)
 * Created : 07/Mar/2011
 * Version : Baby-Raccoon
 */

#ifndef __MUX_T1_H__
#define __MUX_T1_H__

#define OMAP_GPIO_CP_USB_ON         34
#define OMAP_GPIO_UART_SEL          47
#define OMAP_GPIO_JIG_ON18			55

/* For Battery */
#define OMAP_GPIO_CHG_ING_N			11
#define OMAP_GPIO_TA_NCONNECTED	     		12
#define OMAP_GPIO_FUEL_ALERT            	44
#define OMAP_GPIO_FUEL_SCL              	61
#define OMAP_GPIO_FUEL_SDA              	62
#define OMAP_GPIO_CHG_EN			13
#define OMAP_GPIO_BAT_REMOVAL           	29   //GPIO_WK29

/* For MIPI HSI */
#define OMAP_GPIO_MIPI_HSI_CP_ON		36
#define OMAP_GPIO_MIPI_HSI_RESET_REQ_N		50
#define OMAP_GPIO_MIPI_HSI_PDA_ACTIVE		119
#define OMAP_GPIO_MIPI_HSI_PHONE_ACTIVE		120
#define OMAP_GPIO_JACK_NINT			121
#define OMAP_GPIO_MIPI_HSI_CP_RST		2	//wk2
#define OMAP_GPIO_MIPI_HSI_CP_DUMP_INT		1	//wk1

/* For H/W Revision */
#define OMAP_GPIO_HW_REV0			76
#define OMAP_GPIO_HW_REV1			75
#define OMAP_GPIO_HW_REV2			74
#define OMAP_GPIO_HW_REV3			73

#endif /* __MUX_T1_H__ */
