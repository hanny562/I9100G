/*
 * Board support file for containing sensors specific details for OMAP4430 SDP.
 *
 * Copyright (C) 2009 Google, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>

#include <plat/mux.h>
#include <plat/gpio.h>
#include <plat/keypad.h>

#define OMAP4430SDP_AKM8973_INT_GPIO		157
#define OMAP4430SDP_AKM8973_RESET_GPIO		46
#define OMAP4430SDP_ISL29023_INT_GPIO		52
#define OMAP4430SDP_KXSD9_INT_GPIO		94

#define CONTROL_CORE_PAD0_GPMC_A22_PAD1_GPMC_A23 0x4A10006c


static void __init omap4430univ_akm8973_init(void)
{
	int val =0;
	u32 * ctrl_core_pad0_gpmc_a22;

	/* MUX-ing GPMC_A22 into GPIIO_46 */
	ctrl_core_pad0_gpmc_a22 = (u32 *) ioremap(CONTROL_CORE_PAD0_GPMC_A22_PAD1_GPMC_A23, 4);
	if (!ctrl_core_pad0_gpmc_a22) {
		printk(KERN_ERR"OMAP_pad_config: ioremap failed with addr %lx\n",
				CONTROL_CORE_PAD0_GPMC_A22_PAD1_GPMC_A23);
		return;
	}

	val =  __raw_readl(ctrl_core_pad0_gpmc_a22);
	val |= 0x3;  /* Set 0,1 bits high for GPMC_A22 to be configured as GPIO_46 */
	__raw_writel(val, ctrl_core_pad0_gpmc_a22);

	iounmap(ctrl_core_pad0_gpmc_a22);

	gpio_request(OMAP4430SDP_AKM8973_RESET_GPIO, "akm8973 reset");
	gpio_direction_output(OMAP4430SDP_AKM8973_RESET_GPIO, 0);
	gpio_direction_output(OMAP4430SDP_AKM8973_RESET_GPIO, 1);

	gpio_request(OMAP4430SDP_AKM8973_INT_GPIO, "akm8973 irq");
	gpio_direction_input(OMAP4430SDP_AKM8973_INT_GPIO);
}

static void __init omap4430univ_isl29023_init(void)
{
	gpio_request(OMAP4430SDP_ISL29023_INT_GPIO, "isl29023 irq");
	gpio_direction_input(OMAP4430SDP_ISL29023_INT_GPIO);
}

static void __init omap4430univ_kxsd9_init(void)
{
	gpio_request(OMAP4430SDP_KXSD9_INT_GPIO, "kxsd9 irq");
	gpio_direction_input(OMAP4430SDP_KXSD9_INT_GPIO);
}

void __init omap4430univ_sensors_init(void)
{
#ifdef CONFIG_MACH_OMAP4_TAB_10_1
#if (CONFIG_SAMSUNG_OMAP4_TAB_REV <=2)
	omap4430univ_akm8973_init();
	omap4430univ_isl29023_init();
#endif
#endif
	omap4430univ_kxsd9_init();
}

