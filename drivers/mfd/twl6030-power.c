/**
 * drivers/mfd/twl6030-power.c
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
 * File Name : twl6030-power.c
 *
 * File Description :
 *
 * Author : System Platform 2
 * Dept : System S/W Group (Open OS S/W R&D Team)
 * Created : 24/Mar/2011
 * Version : Baby-Raccoon
 */

#include <linux/i2c/twl.h>
#include <plat/io.h>

/* <Regulator>_CFG_GRP registers */
#define NO_GROUP		0x00
#define GROUP_APP		0x01
#define GROUP_CON		0x02
#define GROUP_MOD		0x04

/* <Regulator>_CFG_TRANS registers */
#define ON_STATE_TO_ON		0x03
#define ON_STATE_TO_AMS		0x01
#define ON_STATE_TO_OFF		0x00

#define SLEEP_STATE_TO_ON	0x0C
#define SLEEP_STATE_TO_AMS	0x04
#define SLEEP_STATE_TO_OFF	0x00

#define OFF_STATE_TO_ON		0x30
#define OFF_STATE_TO_AMS	0x10
#define OFF_STATE_TO_OFF	0x00

/* <Regulator>_CFG_STATE registers */
#define STATE_MOD_NOGRP		0x00
#define STATE_MOD_APP		0x20
#define STATE_MOD_CON		0x40
#define STATE_MOD_MOD		0x80

#define STATE_MOD_ON		0x01
#define STATE_MOD_SLEEP		0x03
#define STATE_MOD_OFF		0x00

/* PHOENIX_DEV_ON register */
#define APP_DEV_OFF		0x01
#define CON_DEV_OFF		0x02
#define MOD_DEV_OFF		0x04
#define APP_DEV_ON		0x08
#define CON_DEV_ON		0x10
#define MOD_DEV_ON		0x20

#define TWL6030_LDO_ON(LDO)						\
	twl_i2c_write_u8(TWL6030_MODULE_ID0,				\
	STATE_MOD_APP|STATE_MOD_CON|STATE_MOD_MOD|STATE_MOD_ON,		\
	TWL6030_REG_##LDO##_CFG_STATE)

#define TWL6030_LDO_SLEEP(LDO)						\
	twl_i2c_write_u8(TWL6030_MODULE_ID0,				\
	STATE_MOD_APP|STATE_MOD_CON|STATE_MOD_MOD|STATE_MOD_SLEEP,	\
	TWL6030_REG_##LDO##_CFG_STATE)

#define TWL6030_LDO_OFF(LDO)						\
	twl_i2c_write_u8(TWL6030_MODULE_ID0,				\
	STATE_MOD_APP|STATE_MOD_CON|STATE_MOD_MOD|STATE_MOD_OFF,	\
	TWL6030_REG_##LDO##_CFG_STATE)

extern unsigned int system_rev;

#if defined(CONFIG_MACH_SAMSUNG_T1)
void twl6030_t1_set_ldo_group(void)
{
	/* APP power group - ldo that turn off in suspend */
#if 0
	twl_i2c_write_u8(TWL6030_MODULE_ID0, GROUP_APP,
			TWL6030_REG_VANA_CFG_GRP);
#endif
	twl_i2c_write_u8(TWL6030_MODULE_ID0,
			OFF_STATE_TO_OFF|SLEEP_STATE_TO_OFF|ON_STATE_TO_AMS,
			TWL6030_REG_VANA_CFG_TRANS);

	if (system_rev == 2 || system_rev == 3) {
		twl_i2c_write_u8(TWL6030_MODULE_ID0, GROUP_APP,
			TWL6030_REG_VAUX2_CFG_GRP);
		twl_i2c_write_u8(TWL6030_MODULE_ID0,
			OFF_STATE_TO_OFF|SLEEP_STATE_TO_OFF|ON_STATE_TO_AMS,
			TWL6030_REG_VAUX2_CFG_TRANS);
	}

	twl_i2c_write_u8(TWL6030_MODULE_ID0, GROUP_APP,
			TWL6030_REG_V2V1_CFG_GRP);
	twl_i2c_write_u8(TWL6030_MODULE_ID0,
			OFF_STATE_TO_OFF|SLEEP_STATE_TO_OFF|ON_STATE_TO_AMS,
			TWL6030_REG_V2V1_CFG_TRANS);

	twl_i2c_write_u8(TWL6030_MODULE_ID0, GROUP_APP,
			TWL6030_REG_VCXIO_CFG_GRP);
	twl_i2c_write_u8(TWL6030_MODULE_ID0,
			OFF_STATE_TO_OFF|SLEEP_STATE_TO_OFF|ON_STATE_TO_AMS,
			TWL6030_REG_VCXIO_CFG_TRANS);

	twl_i2c_write_u8(TWL6030_MODULE_ID0, GROUP_APP,
			TWL6030_REG_VDAC_CFG_GRP);
	twl_i2c_write_u8(TWL6030_MODULE_ID0,
			OFF_STATE_TO_OFF|SLEEP_STATE_TO_OFF|ON_STATE_TO_AMS,
			TWL6030_REG_VDAC_CFG_TRANS);

	twl_i2c_write_u8(TWL6030_MODULE_ID0, GROUP_MOD,
			TWL6030_REG_VUSB_CFG_GRP);

	twl_i2c_write_u8(TWL6030_MODULE_ID0,
			OFF_STATE_TO_OFF|SLEEP_STATE_TO_OFF |ON_STATE_TO_AMS,
			TWL6030_REG_VUSB_CFG_TRANS);

	twl_i2c_write_u8(TWL6030_MODULE_ID0,
			STATE_MOD_APP | STATE_MOD_CON | STATE_MOD_OFF,
			TWL6030_REG_VUSB_CFG_STATE);
	twl_i2c_write_u8(TWL6030_MODULE_ID0,
			STATE_MOD_MOD | STATE_MOD_ON,
			TWL6030_REG_VUSB_CFG_STATE);

	/* APP power group - Movinand/sdcard power group */
	twl_i2c_write_u8(TWL6030_MODULE_ID0, GROUP_APP,
			TWL6030_REG_REGEN1_CFG_GRP);
	twl_i2c_write_u8(TWL6030_MODULE_ID0,
			OFF_STATE_TO_OFF|SLEEP_STATE_TO_OFF|ON_STATE_TO_AMS,
			TWL6030_REG_REGEN1_CFG_TRANS);

#if 0
	twl_i2c_write_u8(TWL6030_MODULE_ID0, GROUP_APP,
			TWL6030_REG_SYSEN_CFG_GRP);
#endif 
	twl_i2c_write_u8(TWL6030_MODULE_ID0,
			OFF_STATE_TO_OFF|SLEEP_STATE_TO_OFF|ON_STATE_TO_AMS,
			TWL6030_REG_SYSEN_CFG_TRANS);

	/* CON power group - not used in t1*/
	twl_i2c_write_u8(TWL6030_MODULE_ID0, GROUP_CON,
			TWL6030_REG_VPP_CFG_GRP);
	twl_i2c_write_u8(TWL6030_MODULE_ID0,
			OFF_STATE_TO_OFF|SLEEP_STATE_TO_OFF|ON_STATE_TO_AMS,
			TWL6030_REG_VPP_CFG_TRANS);

	twl_i2c_write_u8(TWL6030_MODULE_ID0, GROUP_CON,
			TWL6030_REG_VUSIM_CFG_GRP);
	twl_i2c_write_u8(TWL6030_MODULE_ID0,
			OFF_STATE_TO_OFF|SLEEP_STATE_TO_OFF|ON_STATE_TO_AMS,
			TWL6030_REG_VUSIM_CFG_TRANS);

	/* Set CLK32KG/CLK32KAUDIO alaways on */
	twl_i2c_write_u8(TWL6030_MODULE_ID0, NO_GROUP,
			TWL6030_REG_CLK32KG_CFG_GRP);
	twl_i2c_write_u8(TWL6030_MODULE_ID0,
			OFF_STATE_TO_OFF|SLEEP_STATE_TO_OFF|ON_STATE_TO_AMS,
			TWL6030_REG_CLK32KG_CFG_TRANS);
	twl_i2c_write_u8(TWL6030_MODULE_ID0, STATE_MOD_NOGRP | STATE_MOD_ON,
			TWL6030_REG_CLK32KG_CFG_STATE);

	twl_i2c_write_u8(TWL6030_MODULE_ID0, NO_GROUP,
			TWL6030_REG_CLK32KAUDIO_CFG_GRP);
	twl_i2c_write_u8(TWL6030_MODULE_ID0,
			OFF_STATE_TO_OFF|SLEEP_STATE_TO_OFF|ON_STATE_TO_AMS,
			TWL6030_REG_CLK32KAUDIO_CFG_TRANS);
	twl_i2c_write_u8(TWL6030_MODULE_ID0, STATE_MOD_NOGRP | STATE_MOD_ON,
			TWL6030_REG_CLK32KAUDIO_CFG_STATE);

}

#elif defined(CONFIG_MACH_SAMSUNG_Q1)
void twl6030_q1_set_ldo_group(void)
{
	/* APP power group - ldo that turn off in suspend */
#if 0
	twl_i2c_write_u8(TWL6030_MODULE_ID0, GROUP_APP,
			TWL6030_REG_VANA_CFG_GRP);
#endif
	twl_i2c_write_u8(TWL6030_MODULE_ID0,
			OFF_STATE_TO_OFF|SLEEP_STATE_TO_OFF|ON_STATE_TO_AMS,
			TWL6030_REG_VANA_CFG_TRANS);

	twl_i2c_write_u8(TWL6030_MODULE_ID0, GROUP_APP,
			TWL6030_REG_V2V1_CFG_GRP);
	twl_i2c_write_u8(TWL6030_MODULE_ID0,
			OFF_STATE_TO_OFF|SLEEP_STATE_TO_OFF|ON_STATE_TO_AMS,
			TWL6030_REG_V2V1_CFG_TRANS);

	twl_i2c_write_u8(TWL6030_MODULE_ID0, GROUP_APP,
			TWL6030_REG_VCXIO_CFG_GRP);
	twl_i2c_write_u8(TWL6030_MODULE_ID0,
			OFF_STATE_TO_OFF|SLEEP_STATE_TO_OFF|ON_STATE_TO_AMS,
			TWL6030_REG_VCXIO_CFG_TRANS);

	twl_i2c_write_u8(TWL6030_MODULE_ID0, GROUP_APP,
			TWL6030_REG_VDAC_CFG_GRP);
	twl_i2c_write_u8(TWL6030_MODULE_ID0,
			OFF_STATE_TO_OFF|SLEEP_STATE_TO_OFF|ON_STATE_TO_AMS,
			TWL6030_REG_VDAC_CFG_TRANS);

	twl_i2c_write_u8(TWL6030_MODULE_ID0, GROUP_APP,
			TWL6030_REG_VUSB_CFG_GRP);
	twl_i2c_write_u8(TWL6030_MODULE_ID0,
			OFF_STATE_TO_OFF|SLEEP_STATE_TO_OFF|ON_STATE_TO_AMS,
			TWL6030_REG_VUSB_CFG_TRANS);

	/* APP power group - Movinand/sdcard power group */
	twl_i2c_write_u8(TWL6030_MODULE_ID0, GROUP_APP,
			TWL6030_REG_REGEN1_CFG_GRP);
	twl_i2c_write_u8(TWL6030_MODULE_ID0,
			OFF_STATE_TO_OFF|SLEEP_STATE_TO_OFF|ON_STATE_TO_AMS,
			TWL6030_REG_REGEN1_CFG_TRANS);

	twl_i2c_write_u8(TWL6030_MODULE_ID0, GROUP_APP,
			TWL6030_REG_SYSEN_CFG_GRP);
	twl_i2c_write_u8(TWL6030_MODULE_ID0,
			OFF_STATE_TO_OFF|SLEEP_STATE_TO_OFF|ON_STATE_TO_AMS,
			TWL6030_REG_SYSEN_CFG_TRANS);

#if 0
	twl_i2c_write_u8(TWL6030_MODULE_ID0, GROUP_APP,
			TWL6030_REG_VMMC_CFG_GRP);
	twl_i2c_write_u8(TWL6030_MODULE_ID0,
			OFF_STATE_TO_OFF|SLEEP_STATE_TO_OFF|ON_STATE_TO_AMS,
			TWL6030_REG_VMMC_CFG_TRANS);
#endif

	/* CON power group - not used in t1*/
	twl_i2c_write_u8(TWL6030_MODULE_ID0, GROUP_CON,
			TWL6030_REG_VPP_CFG_GRP);
	twl_i2c_write_u8(TWL6030_MODULE_ID0,
			OFF_STATE_TO_OFF|SLEEP_STATE_TO_OFF|ON_STATE_TO_AMS,
			TWL6030_REG_VPP_CFG_TRANS);

	/* Set CLK32KG/CLK32KAUDIO alaways on */
	twl_i2c_write_u8(TWL6030_MODULE_ID0, NO_GROUP,
			TWL6030_REG_CLK32KG_CFG_GRP);
	twl_i2c_write_u8(TWL6030_MODULE_ID0,
			OFF_STATE_TO_OFF|SLEEP_STATE_TO_OFF|ON_STATE_TO_AMS,
			TWL6030_REG_CLK32KG_CFG_TRANS);
	twl_i2c_write_u8(TWL6030_MODULE_ID0, STATE_MOD_NOGRP | STATE_MOD_ON,
			TWL6030_REG_CLK32KG_CFG_STATE);

	twl_i2c_write_u8(TWL6030_MODULE_ID0, NO_GROUP,
			TWL6030_REG_CLK32KAUDIO_CFG_GRP);
	twl_i2c_write_u8(TWL6030_MODULE_ID0,
			OFF_STATE_TO_OFF|SLEEP_STATE_TO_OFF|ON_STATE_TO_AMS,
			TWL6030_REG_CLK32KAUDIO_CFG_TRANS);
	twl_i2c_write_u8(TWL6030_MODULE_ID0, STATE_MOD_NOGRP | STATE_MOD_ON,
			TWL6030_REG_CLK32KAUDIO_CFG_STATE);
}
#endif

void twl6030_power_init(void)
{
	/* TWL6030 configuration */
	twl_i2c_write_u8(TWL6030_MODULE_ID0, 0xC0,
				TWL6030_REG_PHOENIX_MSK_TRANSITION);

	/* Turn off charger block */
	twl_i2c_write_u8(TWL6030_MODULE_ID0, 0x00,
				TWL6030_REG_CONTROLLER_CTRL1);

	/* Set pull-up/down configuration */
	twl_i2c_write_u8(TWL6030_MODULE_ID0, 0x25,
				TWL6030_REG_CFG_INPUT_PUPD2);
	twl_i2c_write_u8(TWL6030_MODULE_ID0, 0x19,
				TWL6030_REG_CFG_INPUT_PUPD3);

	twl_i2c_write_u8(TWL6030_MODULE_ID0, 0x00,
				TWL6030_REG_CFG_INPUT_PUPD4);

	/* Set ldo&smps pull-up/down configuration */
	twl_i2c_write_u8(TWL6030_MODULE_ID0, 0xFF,
				TWL6030_REG_CFG_LDO_PD1);
	twl_i2c_write_u8(TWL6030_MODULE_ID0, 0x01,
				TWL6030_REG_CFG_LDO_PD2);
	twl_i2c_write_u8(TWL6030_MODULE_ID0, 0x3C,
				TWL6030_REG_CFG_SMPS_PD);


	/* Set misc1, 2 register */
	twl_i2c_write_u8(TWL6030_MODULE_ID0, 0x00,
				TWL6030_REG_MISC1);

	twl_i2c_write_u8(TWL6030_MODULE_ID0, 0x10,
				TWL6030_REG_MISC2);

	/* Set backup battery charge mode */
	twl_i2c_write_u8(TWL6030_MODULE_ID0, 0x7C,
				TWL6030_REG_BBSPOR_CFG);

	/* Set usb charger mode */
	twl_i2c_write_u8(TWL6030_MODULE_ID1, 0x21,
				TWL6030_REG_CHARGERUSB_CTRL3);

	/* Turn off fuel gauge and vabrator module */
	twl_i2c_write_u8(TWL6030_MODULE_ID1, 0x50,
				TWL6030_REG_TOGGLE1);
	twl_i2c_write_u8(TWL6030_MODULE_ID1, 0x40,
				TWL6030_REG_TOGGLE1);

	/* Set ldo group */
#if defined(CONFIG_MACH_SAMSUNG_T1)
	twl6030_t1_set_ldo_group();
#elif defined(CONFIG_MACH_SAMSUNG_Q1)
	twl6030_q1_set_ldo_group();
#endif

	/* Turn off ldo that is not used in T1 */
	twl_i2c_write_u8(TWL6030_MODULE_ID0, CON_DEV_OFF,
				TWL6030_REG_PHOENIX_DEV_ON);
}

void twl6030_suspend_ldo_off(void)
{
	/* Disable backup battery charge mode */
#if 0
	twl_i2c_write_u8(TWL6030_MODULE_ID0, 0x7C, TWL6030_REG_BBSPOR_CFG);


	twl_i2c_write_u8(TWL6030_MODULE_ID0, 0x00,
				TWL6030_REG_MISC2);
#endif

	twl_i2c_write_u8(TWL6030_MODULE_ID0, STATE_MOD_MOD | STATE_MOD_OFF,
			TWL6030_REG_VUSB_CFG_STATE);


	/* Congiuratoin for OTG & GPADC Module */
	twl_i2c_write_u8(TWL6030_MODULE_ID1, 0xFF,
				TWL6030_REG_USB_VBUS_CTRL_CLR);
	twl_i2c_write_u8(TWL6030_MODULE_ID1, 0xFF,
				TWL6030_REG_USB_ID_CTRL_CLR);
	twl_i2c_write_u8(TWL6030_MODULE_ID1, 0x00,
				TWL6030_REG_GPADC_CTRL);
	twl_i2c_write_u8(TWL6030_MODULE_ID1, 0x51,
				TWL6030_REG_TOGGLE1);

	/* Turn off con and mod group ldo */
	twl_i2c_write_u8(TWL6030_MODULE_ID0, CON_DEV_OFF|MOD_DEV_OFF,
				TWL6030_REG_PHOENIX_DEV_ON);
}

void twl6030_resume_ldo_on(void)
{
	u8 reg_val = 0;
	u8 reg_addr = 0;

	/* Check TWL6030 Interrupt state */
	reg_addr = TWL6030_REG_INT_STS_A;
	while (reg_addr <= TWL6030_REG_INT_STS_C) {
		twl_i2c_read_u8(TWL6030_MODULE_ID1, &reg_val, reg_addr);
		if (reg_val) {
			printk(KERN_ERR"[TWL6030 Interrupt] 0x%02X=0x%02X\n",
				reg_addr, reg_val);
		}
		reg_val = 0;
		reg_addr += 1;
	}

	/* Enable backup battery charge mode */
#if 0
	twl_i2c_write_u8(TWL6030_MODULE_ID0, 0x7C, TWL6030_REG_BBSPOR_CFG);


	/* Enable VUSB power supply */
	twl_i2c_write_u8(TWL6030_MODULE_ID0, 0x10, TWL6030_REG_MISC2);
#endif
	twl_i2c_write_u8(TWL6030_MODULE_ID0, STATE_MOD_MOD | STATE_MOD_ON,
			TWL6030_REG_VUSB_CFG_STATE);

	/* Turn off ldo that is not used in T1 */
	twl_i2c_write_u8(TWL6030_MODULE_ID0, CON_DEV_OFF,
				TWL6030_REG_PHOENIX_DEV_ON);
}
