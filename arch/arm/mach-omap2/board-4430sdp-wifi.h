/*
 * linux/arch/arm/mach-omap2/board-zoom2-wifi.h
 *
 * Copyright (C) 2010 Texas Instruments Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _BOARD_4430SDP_WIFI_H
#define _BOARD_4430SDP_WIFI_H

#define SDP4430_WIFI_PMENA_GPIO     104

#ifdef CONFIG_MACH_OMAP4_TAB_10_1
#if (CONFIG_SAMSUNG_OMAP4_TAB_REV <= 2)
#define SDP4430_WIFI_IRQ_GPIO       83
#endif
#if (CONFIG_SAMSUNG_OMAP4_TAB_REV == 3) \
	|| (CONFIG_SAMSUNG_OMAP4_TAB_REV == 4) \
	|| (CONFIG_SAMSUNG_OMAP4_TAB_REV == 5) \
	|| (CONFIG_SAMSUNG_OMAP4_TAB_REV == 6) \
	|| (CONFIG_SAMSUNG_OMAP4_TAB_REV == 7) 
#define SDP4430_WIFI_IRQ_GPIO       61
#endif
#else
#define SDP4430_WIFI_IRQ_GPIO       61
#endif /* CONFIG_MACH_OMAP4_TAB_10_1 */

/* overwrite default configurations for OMAP4-Samsung Devices */
#if defined(CONFIG_MACH_OMAP4_SAMSUNG)
#undef SDP4430_WIFI_PMENA_GPIO
#undef SDP4430_WIFI_IRQ_GPIO

#define SDP4430_WIFI_PMENA_GPIO		OMAP_GPIO_WIFI_PMENA_GPIO
#define SDP4430_WIFI_IRQ_GPIO		OMAP_GPIO_WIFI_IRQ
#endif /* CONFIG_MACH_OMAP4_SAMSUNG */

void config_wlan_mux(void);

#endif /*_BOARD_4430SDP_WIFI_H*/
