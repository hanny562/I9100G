/**
 * arch/arm/mach-omap2/mux_t1_rev_r03.c
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
 * File Name : mux_t1_rev_r03.c
 *
 * File Description :
 *
 * Author : System Platform 2
 * Dept : System S/W Group (Open OS S/W R&D Team)
 * Created : 07/Mar/2011
 * Version : Baby-Raccoon
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/ctype.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <asm/system.h>
#include <plat/control.h>
#include <plat/mux.h>
#include <plat/gpio.h>
#include <linux/interrupt.h>

#include "mux.h"

#define SEC_OMAP_OUTPUT_GPIO(name, val)		{name, val, (unsigned int)#name},

static unsigned int __omap_board_output_gpio[][3] __initdata = {
	SEC_OMAP_OUTPUT_GPIO(OMAP_GPIO_PS_ON, 1)
	SEC_OMAP_OUTPUT_GPIO(OMAP_GPIO_ACC_EN, 1)
};	/* end array __omap_output_gpio */

unsigned int sec_board_output_gpio_size = ARRAY_SIZE(__omap_board_output_gpio);
EXPORT_SYMBOL(sec_board_output_gpio_size);
unsigned int (*sec_board_output_gpio_ptr)[3] = __omap_board_output_gpio;
EXPORT_SYMBOL(sec_board_output_gpio_ptr);

static unsigned int __omap_wakeup_gpio[] __initdata = {
};	/* end array omap_wakeup_gpio */

unsigned int sec_board_wakeup_gpio_size = ARRAY_SIZE(__omap_wakeup_gpio);
EXPORT_SYMBOL(sec_board_wakeup_gpio_size);
unsigned int (*sec_board_wakeup_gpio_ptr) = __omap_wakeup_gpio;
EXPORT_SYMBOL(sec_board_wakeup_gpio_ptr);

static struct omap_board_mux __omap_board_core_mux[] __initdata = {
	/* [-----] gpmc_noe -  - HSMMC_CLK */
	OMAP4_MUX(GPMC_NOE, OMAP_MUX_MODE1 | OMAP_PIN_INPUT_PULLDOWN),
	/* [-----] gpmc_new -  - HSMMC_CMD */
	OMAP4_MUX(GPMC_NWE, OMAP_MUX_MODE1 | OMAP_PIN_INPUT_PULLUP | OMAP_PIN_OFF_INPUT_PULLDOWN),
	/* [-----] gpmc_ad0 -  - HSMMC_D(0) */
	OMAP4_MUX(GPMC_AD0, OMAP_MUX_MODE1 | OMAP_PIN_INPUT_PULLUP | OMAP_PIN_OFF_INPUT_PULLDOWN),
	/* [-----] gpmc_ad1 -  - HSMMC_D(1) */
	OMAP4_MUX(GPMC_AD1, OMAP_MUX_MODE1 | OMAP_PIN_INPUT_PULLUP | OMAP_PIN_OFF_INPUT_PULLDOWN),
	/* [-----] gpmc_ad2 -  - HSMMC_D(2) */
	OMAP4_MUX(GPMC_AD2, OMAP_MUX_MODE1 | OMAP_PIN_INPUT_PULLUP | OMAP_PIN_OFF_INPUT_PULLDOWN),
	/* [-----] gpmc_ad3 -  - HSMMC_D(3) */
	OMAP4_MUX(GPMC_AD3, OMAP_MUX_MODE1 | OMAP_PIN_INPUT_PULLUP | OMAP_PIN_OFF_INPUT_PULLDOWN),
	/* [-----] gpmc_ad4 -  - HSMMC_D(4) */
	OMAP4_MUX(GPMC_AD4, OMAP_MUX_MODE1 | OMAP_PIN_INPUT_PULLUP | OMAP_PIN_OFF_INPUT_PULLDOWN),
	/* [-----] gpmc_ad5 -  - HSMMC_D(5) */
	OMAP4_MUX(GPMC_AD5, OMAP_MUX_MODE1 | OMAP_PIN_INPUT_PULLUP | OMAP_PIN_OFF_INPUT_PULLDOWN),
	/* [-----] gpmc_ad6 -  - HSMMC_D(6) */
	OMAP4_MUX(GPMC_AD6, OMAP_MUX_MODE1 | OMAP_PIN_INPUT_PULLUP | OMAP_PIN_OFF_INPUT_PULLDOWN),
	/* [-----] gpmc_ad7 -  - HSMMC_D(7) */
	OMAP4_MUX(GPMC_AD7, OMAP_MUX_MODE1 | OMAP_PIN_INPUT_PULLUP | OMAP_PIN_OFF_INPUT_PULLDOWN),

	/* [IN---] gpmc_ad8 - gpio_32 - 3_TOUCH_INT */
	OMAP4_MUX(GPMC_AD8, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
	/* [IN---] gpmc_ad9 - gpio_33 - PS_ALS_INT_18 */
	OMAP4_MUX(GPMC_AD9, OMAP_MUX_MODE3 | OMAP_PIN_INPUT | OMAP_PIN_OFF_WAKEUPENABLE),
	/* [--OUT] gpmc_ad10 - gpio_34 - CP_USB_ON */
	OMAP4_MUX(GPMC_AD10, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	/* [--OUT] gpmc_ad11 - gpio_35 - MLCD_RST */
	OMAP4_MUX(GPMC_AD11, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	/* [--OUT] gpmc_ad12 - gpio_36 - CP_ON */
	OMAP4_MUX(GPMC_AD12, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	/* [--OUT] gpmc_ad13 - gpio_37 - PS_ON */
	OMAP4_MUX(GPMC_AD13, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	/* [IN---] gpmc_ad14 - gpio_38 - OLED_DET */
	OMAP4_MUX(GPMC_AD14, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
	/* [IN---] gpmc_ad15 - gpio_39 - TA_CURRENT_SEL */
	OMAP4_MUX(GPMC_AD15, OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP),
	/* [IN---] gpmc_a16 - gpio_40 - FM_INT */
	OMAP4_MUX(GPMC_A16, OMAP_MUX_MODE3 | OMAP_PIN_INPUT | OMAP_WAKEUP_EN), 
	/* [--OUT] gpmc_a17 - gpio_41 - 8M_ISP_INT */
	OMAP4_MUX(GPMC_A17, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
	/* [--OUT] gpmc_a18 - gpio_42 - FM_RST */
	OMAP4_MUX(GPMC_A18, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	/* [--OUT] gpmc_a19 - gpio_43 - CAM_PMIC_EN */
	OMAP4_MUX(GPMC_A19, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	/* [IN---] gpmc_a20 - gpio_44 - FUEL_ALERT */
	OMAP4_MUX(GPMC_A20, OMAP_MUX_MODE3 | OMAP_PIN_INPUT | OMAP_WAKEUP_EN),
	/* [--OUT] gpmc_a21 - gpio_45 - GYRO_INT */
	OMAP4_MUX(GPMC_A21, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
	/* [IN---] gpmc_a22 - gpio_46 - TOUCH_nINT */
	OMAP4_MUX(GPMC_A22, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
	/* [--OUT] gpmc_a23 - gpio_47 - UART_SEL */
	OMAP4_MUX(GPMC_A23, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	/* [--OUT] gpmc_a24 - gpio_48 - MICBIAS_EN */
	OMAP4_MUX(GPMC_A24, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	/* [--OUT] gpmc_a25 - gpio_49 - EAR_MICBIAS_EN */
	OMAP4_MUX(GPMC_A25, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	/* [--OUT] gpmc_ncs0 - gpio_50 - RESET_REQ_N */
	OMAP4_MUX(GPMC_NCS0, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	/* [--OUT] gpmc_ncs1 - gpio_51 - VT_CAM_nSTBY */
	OMAP4_MUX(GPMC_NCS1, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW),
	/* [--OUT] gpmc_ncs2 - gpio_52 - GPS_CNTL */
	OMAP4_MUX(GPMC_NCS2, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
	/* [--OUT] gpmc_ncs3 - gpio_53 - MHL_SEL */
	OMAP4_MUX(GPMC_NCS3, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT), 
	/* [--OUT] gpmc_nwp - gpio_54 - TOUCH_EN */
	OMAP4_MUX(GPMC_NWP, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	/* [--OUT] gpmc_clk - gpio_55 - JIG_ON_18 */
	OMAP4_MUX(GPMC_CLK, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),

	/* [IN---] gpmc_nadv_ale - gpio_56 - CP_DUMP_INT */
	OMAP4_MUX(GPMC_NADV_ALE, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),

	/* [--OUT] gpmc_nbe0_cle - gpio_59 - SENSOR_EN */
	OMAP4_MUX(GPMC_NBE0_CLE, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	/* [IN---] gpmc_nbe1 - gpio_60 - MHL_RST */
	OMAP4_MUX(GPMC_NBE1, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	/* [--OUT] gpmc_wait0 - gpio_61 - FUEL_I2C_SCL */
	OMAP4_MUX(GPMC_WAIT0, OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP),
	/* [INOUT] gpmc_wait1 - gpio_62 - FUEL_I2C_SDA */
	OMAP4_MUX(GPMC_WAIT1, OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP),
	/* [IN---] gpmc_wait2 - gpio_100 - HDMI_EN */
	OMAP4_MUX(GPMC_WAIT2, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	/* [--OUT] gpmc_ncs4 - gpio_101 - 3_TOUCH_EN */
	OMAP4_MUX(GPMC_NCS4, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	/* [--OUT] gpmc_ncs5 - gpio_102 - 3_TOUCH_LED_EN */
	OMAP4_MUX(GPMC_NCS5, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	/* [--OUT] gpmc_ncs6 - gpio_103 - BT_EN */
	OMAP4_MUX(GPMC_NCS6, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	/* [--OUT] gpmc_ncs7 - gpio_104 - WLAN_EN */
	OMAP4_MUX(GPMC_NCS7, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),

	/* [-----] hdmi_hpd - gpio_63 - HDMI_HPD */
	OMAP4_MUX(HDMI_HPD, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLDOWN),
	/* [-----] hdmi_cec - gpio_64 - MHL_WAKE_UP */
	OMAP4_MUX(HDMI_CEC, OMAP_MUX_MODE7 | OMAP_PIN_INPUT_PULLDOWN),
	/* [-----] hdmi_ddc_scl - gpio_65 - DDC_SCL_3.3V */
	OMAP4_MUX(HDMI_DDC_SCL, OMAP_MUX_MODE0 | OMAP_PIN_INPUT | OMAP_PIN_OFF_INPUT_PULLDOWN),
	/* [-----] hdmi_ddc_sda - gpio_66 - DDC_SDA_3.3V */
	OMAP4_MUX(HDMI_DDC_SDA, OMAP_MUX_MODE0 | OMAP_PIN_INPUT | OMAP_PIN_OFF_INPUT_PULLDOWN),

#if 0
	/* [-----] csi21_dx0 - gpio_67 - 8M_CLK_P */
	OMAP4_MUX(CSI21_DX0, OMAP_MUX_MODE0 | OMAP_PIN_INPUT | OMAP_PIN_OFF_OUTPUT_LOW),
	/* [-----] csi21_dy0 - gpio_68 - 8M_CLK_N */
	OMAP4_MUX(CSI21_DY0, OMAP_MUX_MODE0 | OMAP_PIN_INPUT | OMAP_PIN_OFF_OUTPUT_LOW),
	/* [-----] csi21_dx1 - gpio_69 - 8M_DP_1 */
	OMAP4_MUX(CSI21_DX1, OMAP_MUX_MODE0 | OMAP_PIN_INPUT | OMAP_PIN_OFF_OUTPUT_LOW),
	/* [-----] csi21_dy1 - gpio_70 - 8M_DN_1 */
	OMAP4_MUX(CSI21_DY1, OMAP_MUX_MODE0 | OMAP_PIN_INPUT | OMAP_PIN_OFF_OUTPUT_LOW),
	/* [-----] csi21_dx2 - gpio_71 - 8M_DP_2 */
	OMAP4_MUX(CSI21_DX2, OMAP_MUX_MODE0 | OMAP_PIN_INPUT | OMAP_PIN_OFF_OUTPUT_LOW),
	/* [-----] csi21_dy2 - gpio_72 - 8M_DN_2 */
	OMAP4_MUX(CSI21_DY2, OMAP_MUX_MODE0 | OMAP_PIN_INPUT | OMAP_PIN_OFF_OUTPUT_LOW),
#endif

	/* [IN---] csi21_dx3 - gpio_73 - HW_REV3 */
	OMAP4_MUX(CSI21_DX3, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
	/* [IN---] csi21_dy3 - gpio_74 - HW_REV2 */
	OMAP4_MUX(CSI21_DY3, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
	/* [IN---] csi21_dx4 - gpio_75 - HW_REV1 */
	OMAP4_MUX(CSI21_DX4, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
	/* [IN---] csi21_dy4 - gpio_76 - HW_REV0 */
	OMAP4_MUX(CSI21_DY4, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),

#if 0
	/* [IN---] csi22_dx0 - gpio_77 - VT_CAM_CLK_P */
	OMAP4_MUX(CSI22_DX0, OMAP_MUX_MODE0 | OMAP_PIN_INPUT | OMAP_PIN_OFF_OUTPUT_LOW),
	/* [IN---] csi22_dy0 - gpio_78 - VT_CAM_CLK_N */
	OMAP4_MUX(CSI22_DY0, OMAP_MUX_MODE0 | OMAP_PIN_INPUT | OMAP_PIN_OFF_OUTPUT_LOW),
	/* [IN---] csi22_dx1 - gpio_79 - VT_CAM_DP */
	OMAP4_MUX(CSI22_DX1, OMAP_MUX_MODE0 | OMAP_PIN_INPUT | OMAP_PIN_OFF_OUTPUT_LOW),
	/* [IN---] csi22_dy1 - gpio_80 - VT_CAM_DN */
	OMAP4_MUX(CSI22_DY1, OMAP_MUX_MODE0 | OMAP_PIN_INPUT | OMAP_PIN_OFF_OUTPUT_LOW),
#endif

	/* [IN---] cam_shutter - gpio_81 - WLAN_HOST_WAKE */
	OMAP4_MUX(CAM_SHUTTER, OMAP_MUX_MODE3 | OMAP_PIN_INPUT | OMAP_PIN_OFF_WAKEUPENABLE),
	/* [--OUT] cam_strobe - gpio_82 - BT_nRST */
	OMAP4_MUX(CAM_STROBE, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	/* [IN---] cam_globalreset - gpio_83 - BT_HOST_WAKE */
	OMAP4_MUX(CAM_GLOBALRESET, OMAP_MUX_MODE3 | OMAP_PIN_INPUT | OMAP_PIN_OFF_WAKEUPENABLE),

	/* [IN---] usbb1_ulpitll_clk - gpio_84 - MIPI_HSI_TX_WAKE */
	OMAP4_MUX(USBB1_ULPITLL_CLK, OMAP_MUX_MODE1 | OMAP_PIN_INPUT | OMAP_PIN_OFF_WAKEUPENABLE),
	/* [IN---] usbb1_ulpitll_stp - gpio_85 - MIPI_HSI_TX_DATA */
	OMAP4_MUX(USBB1_ULPITLL_STP, OMAP_MUX_MODE1 | OMAP_PIN_INPUT),
	/* [IN---] usbb1_ulpitll_dir - gpio_86 - MIPI_HSI_TX_FLG */
	OMAP4_MUX(USBB1_ULPITLL_DIR, OMAP_MUX_MODE1 | OMAP_PIN_INPUT),
	/* [--OUT] usbb1_ulpitll_nxt - gpio_87 - MIPI_HSI_TX_RDY */
	OMAP4_MUX(USBB1_ULPITLL_NXT, OMAP_MUX_MODE1 | OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW),
	/* [--OUT] usbb1_ulpitll_dat0 - gpio_88 - MIPI_HSI_RX_WAKE */
	OMAP4_MUX(USBB1_ULPITLL_DAT0, OMAP_MUX_MODE1 | OMAP_PIN_OUTPUT),
	/* [--OUT] usbb1_ulpitll_dat1 - gpio_89 - MIPI_HSI_RX_DATA */
	OMAP4_MUX(USBB1_ULPITLL_DAT1, OMAP_MUX_MODE1 | OMAP_PIN_OUTPUT),
	/* [--OUT] usbb1_ulpitll_dat2 - gpio_90 - MIPI_HSI_RX_FLG */
	OMAP4_MUX(USBB1_ULPITLL_DAT2, OMAP_MUX_MODE1 | OMAP_PIN_OUTPUT),
	/* [IN---] usbb1_ulpitll_dat3 - gpio_91 - MIPI_HSI_RX_RDY */
	OMAP4_MUX(USBB1_ULPITLL_DAT3, OMAP_MUX_MODE1 | OMAP_PIN_INPUT),

	/* [IN---] usbb1_ulpitll_dat4 - gpio_92 - BOOT_MODE */
	OMAP4_MUX(USBB1_ULPITLL_DAT4, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
	/* [--OUT] usbb1_ulpitll_dat5 - gpio_93 - BT_WAKE */
	OMAP4_MUX(USBB1_ULPITLL_DAT5, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT), 
	/* [IN---] usbb1_ulpitll_dat6 - gpio_94 - MOTOR_PWM */
	OMAP4_MUX(USBB1_ULPITLL_DAT6, OMAP_MUX_MODE1 | OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW),
	/* [IN---] usbb1_ulpitll_dat7 - gpio_95 - MOTOR_EN in rev 0.6*/
	OMAP4_MUX(USBB1_ULPITLL_DAT7, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW),
	/* [-N-C-] usbb1_hsic_data - gpio_96 - NC */
	OMAP4_MUX(USBB1_HSIC_DATA, OMAP_MUX_MODE7 | OMAP_PIN_INPUT_PULLDOWN),
	/* [-N-C-] usbb1_hsic_strobe - gpio_97 - NC */
	OMAP4_MUX(USBB1_HSIC_STROBE, OMAP_MUX_MODE7 | OMAP_PIN_INPUT_PULLDOWN),

	/* [IN---] usbc1_icusb_dp - gpio_98 - MHL_SCL_1.8V */
	OMAP4_MUX(USBC1_ICUSB_DP, OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP),
	/* [IN---] usbc1_icusb_dm - gpio_99 - MHL_SDA_1.8V */
	OMAP4_MUX(USBC1_ICUSB_DM, OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP),

	/* [--OUT] sdmmc1_clk - gpio_100 - TFLASH_CLK */
	OMAP4_MUX(SDMMC1_CLK, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLDOWN),
	/* [--OUT] sdmmc1_cmd - gpio_101 - T_FLASH_CMD */
	OMAP4_MUX(SDMMC1_CMD, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* [INOUT] sdmmc1_dat0 - gpio_102 - T_FLASH_D(0) */
	OMAP4_MUX(SDMMC1_DAT0, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* [INOUT] sdmmc1_dat1 - gpio_103 - T_FLASH_D(1) */
	OMAP4_MUX(SDMMC1_DAT1, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* [INOUT] sdmmc1_dat2 - gpio_104 - T_FLASH_D(2) */
	OMAP4_MUX(SDMMC1_DAT2, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* [INOUT] sdmmc1_dat3 - gpio_105 - T_FLASH_D(3) */
	OMAP4_MUX(SDMMC1_DAT3, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* [-N-C-] sdmmc1_dat4 - gpio_106 - NC */
	OMAP4_MUX(SDMMC1_DAT4, OMAP_MUX_MODE7 | OMAP_PIN_INPUT_PULLDOWN),
	/* [-N-C-] sdmmc1_dat5 - gpio_107 - NC */
	OMAP4_MUX(SDMMC1_DAT5, OMAP_MUX_MODE7 | OMAP_PIN_INPUT_PULLDOWN),
	/* [-N-C-] sdmmc1_dat6 - gpio_108 - NC */
	OMAP4_MUX(SDMMC1_DAT6, OMAP_MUX_MODE7 | OMAP_PIN_INPUT_PULLDOWN),
	/* [-N-C-] sdmmc1_dat7 - gpio_109 - NC */
	OMAP4_MUX(SDMMC1_DAT7, OMAP_MUX_MODE7 | OMAP_PIN_INPUT_PULLDOWN),
#if 0
	/* [IN---] abe_mcbsp2_clkx - gpio_110 - REC_PCM_CLK */
	OMAP4_MUX(ABE_MCBSP2_CLKX, OMAP_MUX_MODE0),
	/* [--OUT] abe_mcbsp2_dr - gpio_111 - REC_PCM_IN */
	OMAP4_MUX(ABE_MCBSP2_DR, OMAP_MUX_MODE0),
	/* [IN---] abe_mcbsp2_dx - gpio_112 - REC_PCM_OUT */
	OMAP4_MUX(ABE_MCBSP2_DX, OMAP_MUX_MODE0),
	/* [IN---] abe_mcbsp2_fsx - gpio_113 - REC_PCM_SYNC */
	OMAP4_MUX(ABE_MCBSP2_FSX, OMAP_MUX_MODE0),
	/* [IN---] abe_mcbsp1_clkx - gpio_114 - BT_PCM_CLK */
	OMAP4_MUX(ABE_MCBSP1_CLKX, OMAP_MUX_MODE0),
	/* [--OUT] abe_mcbsp1_dr - gpio_115 - BT_PCM_DOUT */
	OMAP4_MUX(ABE_MCBSP1_DR, OMAP_MUX_MODE0),
	/* [IN---] abe_mcbsp1_dx - gpio_116 - BT_PCM_DIN */
	OMAP4_MUX(ABE_MCBSP1_DX, OMAP_MUX_MODE0),
	/* [IN---] abe_mcbsp1_fsx - gpio_117 - BT_PCM_SYNC */
	OMAP4_MUX(ABE_MCBSP1_FSX, OMAP_MUX_MODE0),
	/* [--OUT] abe_pdm_ul_data -  - PDM_UL_DATA */
	OMAP4_MUX(ABE_PDM_UL_DATA, OMAP_MUX_MODE0),
	/* [IN---] abe_pdm_dl_data -  - PDM_DL_DATA */
	OMAP4_MUX(ABE_PDM_DL_DATA, OMAP_MUX_MODE0),
	/* [INOUT] abe_pdm_frame -  - PDM_FRAME */
	OMAP4_MUX(ABE_PDM_FRAME, OMAP_MUX_MODE0),
	/* [INOUT] abe_pdm_lb_clk -  - PDM_CLK */
	OMAP4_MUX(ABE_PDM_LB_CLK, OMAP_MUX_MODE0),
	/* [INOUT] abe_clks - gpio_118 - ABE_CLKS */
	OMAP4_MUX(ABE_CLKS, OMAP_MUX_MODE0),
#endif

	/* [--OUT] abe_dmic_clk1 - gpio_119 - PDA_ACTIVE */
	OMAP4_MUX(ABE_DMIC_CLK1, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	/* [IN---] abe_dmic_din1 - gpio_120 - PHONE_ACTIVE */
	OMAP4_MUX(ABE_DMIC_DIN1, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),

	/* [IN---] abe_dmic_din2 - gpio_121 - JACK_nINT */
	OMAP4_MUX(ABE_DMIC_DIN2, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
	/* [IN---] abe_dmic_din3 - gpio_122 - ACC_INT */
	OMAP4_MUX(ABE_DMIC_DIN3, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),

	/* [IN---] uart2_cts - gpio_123 - BT_UART_CTS */
	OMAP4_MUX(UART2_CTS, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* [--OUT] uart2_rts - gpio_124 - BT_UART_RTS */
	OMAP4_MUX(UART2_RTS, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* [--OUT] uart2_rx - gpio_125 - BT_UART_RXD */
	OMAP4_MUX(UART2_RX, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* [IN---] uart2_tx - gpio_126 - BT_UART_TXD */
	OMAP4_MUX(UART2_TX, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),

	/* [--OUT] hdq_sio - gpio_127 - AUD_PWRON */
	OMAP4_MUX(HDQ_SIO, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	/* [--OUT] i2c1_scl -  - PHEONIX_I2C_SCL */
	OMAP4_MUX(I2C1_SCL, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* [INOUT] i2c1_sda -  - PHEONIX_I2C_SDA */
	OMAP4_MUX(I2C1_SDA, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* [--OUT] i2c2_scl - gpio_128 - CAM_I2C_SCL */
	OMAP4_MUX(I2C2_SCL, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* [INOUT] i2c2_sda - gpio_129 - CAM_I2C_SDA */
	OMAP4_MUX(I2C2_SDA, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* [--OUT] i2c3_scl - gpio_130 - AP_I2C_SCL */
	OMAP4_MUX(I2C3_SCL, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* [INOUT] i2c3_sda - gpio_131 - AP_I2C_SDA */
	OMAP4_MUX(I2C3_SDA, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* [--OUT] i2c4_scl - gpio_132 - GEN_I2C_SCL */
	OMAP4_MUX(I2C4_SCL, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* [INOUT] i2c4_sda - gpio_133 - GEN_I2C_SDA */
	OMAP4_MUX(I2C4_SDA, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),

	/* [-N-C-] mcspi1_clk - gpio_134 - NC */
	OMAP4_MUX(MCSPI1_CLK, OMAP_MUX_MODE7 | OMAP_PIN_INPUT_PULLDOWN),
	/* [-N-C-] mcspi1_somi - gpio_135 - NC */
	OMAP4_MUX(MCSPI1_SOMI, OMAP_MUX_MODE7 | OMAP_PIN_INPUT_PULLDOWN),
	/* [-N-C-] mcspi1_simo - gpio_136 - NC */
	OMAP4_MUX(MCSPI1_SIMO, OMAP_MUX_MODE7 | OMAP_PIN_INPUT_PULLDOWN),
	/* [-N-C-] mcspi1_cs0 - gpio_137 - NC */
	OMAP4_MUX(MCSPI1_CS0, OMAP_MUX_MODE7 | OMAP_PIN_INPUT_PULLDOWN),

	/* [--OUT] mcspi1_cs1 - gpio_138 - GPS_UART_RXD */
	OMAP4_MUX(MCSPI1_CS1, OMAP_MUX_MODE1 | OMAP_PIN_INPUT),
	/* [IN---] mcspi1_cs2 - gpio_139 - 3_TOUCH_SCL */
	OMAP4_MUX(MCSPI1_CS2, OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP),
	/* [--OUT] mcspi1_cs3 - gpio_140 - 3_TOUCH_SDA */
	OMAP4_MUX(MCSPI1_CS3, OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP),
	/* [--OUT] uart3_cts_rctx - gpio_141 - GPS_UART_TXD */
	OMAP4_MUX(UART3_CTS_RCTX, OMAP_MUX_MODE1 | OMAP_PIN_INPUT),

	/* [IN---] uart3_rts_sd - gpio_142 - OLED_ID */
	OMAP4_MUX(UART3_RTS_SD, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),

	/* [IN---] uart3_rx_irrx - gpio_143 - AP_FLM_RXD */ /* request by HW */
//	OMAP4_MUX(UART3_RX_IRRX, OMAP_MUX_MODE0 | OMAP_PIN_INPUT | OMAP_PIN_OFF_OUTPUT_HIGH),
	OMAP4_MUX(UART3_RX_IRRX, OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLDOWN),
	/* [--OUT] uart3_tx_irtx - gpio_144 - AP_FLM_TXD */ /* request by HW */
//	OMAP4_MUX(UART3_TX_IRTX, OMAP_MUX_MODE0 | OMAP_PIN_INPUT | OMAP_PIN_OFF_OUTPUT_HIGH),
	OMAP4_MUX(UART3_TX_IRTX, OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLDOWN),

	/* [--OUT] sdmmc5_clk - gpio_145 - WLAN_SD_CLK */
	OMAP4_MUX(SDMMC5_CLK, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLDOWN),
	/* [--OUT] sdmmc5_cmd - gpio_146 - WLAN_SD_CMD */
	OMAP4_MUX(SDMMC5_CMD, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* [INOUT] sdmmc5_dat0 - gpio_147 - WLAN_SD_D(0) */
	OMAP4_MUX(SDMMC5_DAT0, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* [INOUT] sdmmc5_dat1 - gpio_148 - WLAN_SD_D(1) */
	OMAP4_MUX(SDMMC5_DAT1, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* [INOUT] sdmmc5_dat2 - gpio_149 - WLAN_SD_D(2) */
	OMAP4_MUX(SDMMC5_DAT2, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* [INOUT] sdmmc5_dat3 - gpio_150 - WLAN_SD_D(3) */
	OMAP4_MUX(SDMMC5_DAT3, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),

	/* [--OUT] mcspi4_somi - gpio_153 - 8M_nRST */
	OMAP4_MUX(MCSPI4_SOMI, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW), 

	/* [IN---] uart4_rx - gpio_155 - AP_RXD */
	OMAP4_MUX(UART4_RX, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	/* [--OUT] uart4_tx - gpio_156 - AP_TXD */
	OMAP4_MUX(UART4_TX, OMAP_MUX_MODE0 | OMAP_PIN_OFF_OUTPUT_LOW),

	/* [IN---] usbb2_ulpitll_clk - gpio_157 - MSENSE_IRQ */
	OMAP4_MUX(USBB2_ULPITLL_CLK, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),

	/* [--OUT] usbb2_hsic_data - gpio_169 - OTG_USB_5V */
	OMAP4_MUX(USBB2_HSIC_DATA, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),

	/* [-N-C-] usbb2_hsic_strobe - gpio_170 - NC */
	OMAP4_MUX(USBB2_HSIC_STROBE, OMAP_MUX_MODE7 | OMAP_PIN_INPUT_PULLDOWN),

	/* [IN---] kpd_col3 - gpio_171 - USB_OTG_EN */
	OMAP4_MUX(KPD_COL3, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	/* [-N-C-] kpd_col4 - gpio_172 - AP_AGPS_TSYNC */
	OMAP4_MUX(KPD_COL4, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	/* [-N-C-] kpd_col5 - gpio_173 - GPS_EN */
	OMAP4_MUX(KPD_COL5, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
 
	/* [--OUT] kpd_col0 - gpio_174 - VT_CAM_nRST */
	OMAP4_MUX(KPD_COL0, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT | OMAP_PIN_OFF_OUTPUT_LOW),
 
	/* [-N-C-] kpd_col1 - gpio_0 - NC */
	OMAP4_MUX(KPD_COL1, OMAP_MUX_MODE7 | OMAP_PIN_INPUT_PULLDOWN),
	/* [IN---] kpd_col2 - gpio_1 - USB_OTG_ID */
	OMAP4_MUX(KPD_COL2, OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP | OMAP_PIN_OFF_WAKEUPENABLE),
	/* [IN---] kpd_row3 - gpio_175 - MHL_INT */
	OMAP4_MUX(KPD_ROW3, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),
	/* [IN---] kpd_row4 - gpio_176 - HOME_KEY (Before rev0.5) */
	OMAP4_MUX(KPD_ROW4, OMAP_MUX_MODE3 | OMAP_PIN_INPUT | OMAP_PIN_OFF_WAKEUPENABLE),
	/* [-N-C-] kpd_row5 - gpio_177 - SUB_MICBIAS_EN */
	OMAP4_MUX(KPD_ROW5, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	/* [-N-C-] kpd_row0 - gpio_178 - GPS_nRST */
	OMAP4_MUX(KPD_ROW0, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),
	/* [-N-C-] kpd_row1 - gpio_2 - NC */
	OMAP4_MUX(KPD_ROW1, OMAP_MUX_MODE7 | OMAP_PIN_INPUT_PULLDOWN),
	/* [-N-C-] kpd_row2 - gpio_3 - NC */
	OMAP4_MUX(KPD_ROW2, OMAP_MUX_MODE7 | OMAP_PIN_INPUT_PULLDOWN),
	/* [--OUT] usba0_otg_ce -  - USB_CHGEN */
	OMAP4_MUX(USBA0_OTG_CE, OMAP_MUX_MODE0),

	/* [-----] usba0_otg_dp - gpio_179 - AP_D+ */
	OMAP4_MUX(USBA0_OTG_DP, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),
	/* [-----] usba0_otg_dm - gpio_180 - AP_D- */
	OMAP4_MUX(USBA0_OTG_DM, OMAP_MUX_MODE0 | OMAP_PIN_INPUT),

	/* [--OUT] fref_clk1_out - gpio_181 - 8M_MCLK */
	OMAP4_MUX(FREF_CLK1_OUT, OMAP_MUX_MODE0 | OMAP_PIN_INPUT | OMAP_PIN_OFF_OUTPUT_LOW),
	/* [--OUT] fref_clk2_out - gpio_182 - VT_CAM_MCLK */
	OMAP4_MUX(FREF_CLK2_OUT, OMAP_MUX_MODE0 | OMAP_PIN_INPUT | OMAP_PIN_OFF_OUTPUT_LOW),

	/* [IN---] sys_nirq1 -	- SYS_nIRQ1 */
	OMAP4_MUX(SYS_NIRQ1, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* [IN---] sys_nirq2 - gpio_183 - SYS_nIRQ2 */
	OMAP4_MUX(SYS_NIRQ2, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),

	/* [IN---] sys_boot0 - gpio_184 - SYS_BOOT0 */
	OMAP4_MUX(SYS_BOOT0, OMAP_MUX_MODE7 | OMAP_PIN_INPUT_PULLDOWN),
	/* [IN---] sys_boot1 - gpio_185 - SYS_BOOT1 */
	OMAP4_MUX(SYS_BOOT1, OMAP_MUX_MODE7 | OMAP_PIN_INPUT_PULLDOWN),
	/* [IN---] sys_boot2 - gpio_186 - SYS_BOOT2 */
	OMAP4_MUX(SYS_BOOT2, OMAP_MUX_MODE7 | OMAP_PIN_INPUT_PULLDOWN),
	/* [IN---] sys_boot3 - gpio_187 - SYS_BOOT3 */
	OMAP4_MUX(SYS_BOOT3, OMAP_MUX_MODE7 | OMAP_PIN_INPUT_PULLDOWN),
	/* [IN---] sys_boot4 - gpio_188 - SYS_BOOT4 */
	OMAP4_MUX(SYS_BOOT4, OMAP_MUX_MODE7 | OMAP_PIN_INPUT_PULLDOWN),
	/* [IN---] sys_boot5 - gpio_189 - SYS_BOOT5 */
	OMAP4_MUX(SYS_BOOT5, OMAP_MUX_MODE7 | OMAP_PIN_INPUT_PULLDOWN),

	/* [IN---] dpm_emu0 - gpio_11 - CHG_ING_N */
	OMAP4_MUX(DPM_EMU0, OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP), 
	/* [--OUT] dpm_emu1 - gpio_12 - TA_nCONNECTED */
	OMAP4_MUX(DPM_EMU1, OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP), 

	/* [--OUT] dpm_emu2 - gpio_13 - CHG_EN */
	OMAP4_MUX(DPM_EMU2, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),

	/* [--OUT] mcspi4_clk - gpio_151 - LCD_SCLK */
	OMAP4_MUX(MCSPI4_CLK, OMAP_MUX_MODE0 | OMAP_PIN_OFF_OUTPUT_LOW),
	/* [--OUT] mcspi4_simo - gpio_152 - LCD_SDI */
	OMAP4_MUX(MCSPI4_SIMO, OMAP_MUX_MODE0 | OMAP_PIN_OFF_OUTPUT_LOW),
	/* [--OUT] mcspi4_cs0 - gpio_154 - LCD_nCS */
	OMAP4_MUX(MCSPI4_CS0, OMAP_MUX_MODE0 | OMAP_PIN_OFF_OUTPUT_LOW),

	/* [--OUT] usbb2_ulpitll_stp - gpio_158 - LCD_D(23) */
	OMAP4_MUX(USBB2_ULPITLL_STP, OMAP_MUX_MODE5 | OMAP_PIN_OFF_OUTPUT_LOW),
	/* [--OUT] usbb2_ulpitll_dir - gpio_159 - LCD_D(22) */
	OMAP4_MUX(USBB2_ULPITLL_DIR, OMAP_MUX_MODE5 | OMAP_PIN_OFF_OUTPUT_LOW),
	/* [--OUT] usbb2_ulpitll_nxt - gpio_160 - LCD_D(21) */
	OMAP4_MUX(USBB2_ULPITLL_NXT, OMAP_MUX_MODE5 | OMAP_PIN_OFF_OUTPUT_LOW),
	/* [--OUT] usbb2_ulpitll_dat0 - gpio_161 - LCD_D(20) */
	OMAP4_MUX(USBB2_ULPITLL_DAT0, OMAP_MUX_MODE5 | OMAP_PIN_OFF_OUTPUT_LOW),
	/* [--OUT] usbb2_ulpitll_dat1 - gpio_162 - LCD_D(19) */
	OMAP4_MUX(USBB2_ULPITLL_DAT1, OMAP_MUX_MODE5 | OMAP_PIN_OFF_OUTPUT_LOW),
	/* [--OUT] usbb2_ulpitll_dat2 - gpio_163 - LCD_D(18) */
	OMAP4_MUX(USBB2_ULPITLL_DAT2, OMAP_MUX_MODE5 | OMAP_PIN_OFF_OUTPUT_LOW),
	/* [--OUT] usbb2_ulpitll_dat3 - gpio_164 - LCD_D(15) */
	OMAP4_MUX(USBB2_ULPITLL_DAT3, OMAP_MUX_MODE5 | OMAP_PIN_OFF_OUTPUT_LOW),
	/* [--OUT] usbb2_ulpitll_dat4 - gpio_165 - LCD_D(14) */
	OMAP4_MUX(USBB2_ULPITLL_DAT4, OMAP_MUX_MODE5 | OMAP_PIN_OFF_OUTPUT_LOW),
	/* [--OUT] usbb2_ulpitll_dat5 - gpio_166 - LCD_D(13) */
	OMAP4_MUX(USBB2_ULPITLL_DAT5, OMAP_MUX_MODE5 | OMAP_PIN_OFF_OUTPUT_LOW),
	/* [--OUT] usbb2_ulpitll_dat6 - gpio_167 - LCD_D(12) */
	OMAP4_MUX(USBB2_ULPITLL_DAT6, OMAP_MUX_MODE5 | OMAP_PIN_OFF_OUTPUT_LOW),
	/* [--OUT] usbb2_ulpitll_dat7 - gpio_168 - LCD_D(11) */
	OMAP4_MUX(USBB2_ULPITLL_DAT7, OMAP_MUX_MODE5 | OMAP_PIN_OFF_OUTPUT_LOW),
	/* [--OUT] dpm_emu3 - gpio_14 - LCD_D(10) */
	OMAP4_MUX(DPM_EMU3, OMAP_MUX_MODE5 | OMAP_PIN_OFF_OUTPUT_LOW),
	/* [--OUT] dpm_emu4 - gpio_15 - LCD_D(9) */
	OMAP4_MUX(DPM_EMU4, OMAP_MUX_MODE5 | OMAP_PIN_OFF_OUTPUT_LOW),
	/* [--OUT] dpm_emu5 - gpio_16 - LCD_D(16) */
	OMAP4_MUX(DPM_EMU5, OMAP_MUX_MODE5 | OMAP_PIN_OFF_OUTPUT_LOW),
	/* [--OUT] dpm_emu6 - gpio_17 - LCD_D(17) */
	OMAP4_MUX(DPM_EMU6, OMAP_MUX_MODE5 | OMAP_PIN_OFF_OUTPUT_LOW),
	/* [--OUT] dpm_emu7 - gpio_18 - LCD_HSYNC */
	OMAP4_MUX(DPM_EMU7, OMAP_MUX_MODE5 | OMAP_PIN_OFF_OUTPUT_LOW),
	/* [--OUT] dpm_emu8 - gpio_19 - LCD_PCLK */
	OMAP4_MUX(DPM_EMU8, OMAP_MUX_MODE5 | OMAP_PIN_OFF_OUTPUT_LOW),
	/* [--OUT] dpm_emu9 - gpio_20 - LCD_VSYNC */
	OMAP4_MUX(DPM_EMU9, OMAP_MUX_MODE5 | OMAP_PIN_OFF_OUTPUT_LOW),
	/* [--OUT] dpm_emu10 - gpio_21 - LCD_DE */
	OMAP4_MUX(DPM_EMU10, OMAP_MUX_MODE5 | OMAP_PIN_OFF_OUTPUT_LOW),
	/* [--OUT] dpm_emu11 - gpio_22 - LCD_D(8) */
	OMAP4_MUX(DPM_EMU11, OMAP_MUX_MODE5 | OMAP_PIN_OFF_OUTPUT_LOW),
	/* [--OUT] dpm_emu12 - gpio_23 - LCD_D(7) */
	OMAP4_MUX(DPM_EMU12, OMAP_MUX_MODE5 | OMAP_PIN_OFF_OUTPUT_LOW),
	/* [--OUT] dpm_emu13 - gpio_24 - LCD_D(6) */
	OMAP4_MUX(DPM_EMU13, OMAP_MUX_MODE5 | OMAP_PIN_OFF_OUTPUT_LOW),
	/* [--OUT] dpm_emu14 - gpio_25 - LCD_D(5) */
	OMAP4_MUX(DPM_EMU14, OMAP_MUX_MODE5 | OMAP_PIN_OFF_OUTPUT_LOW),
	/* [--OUT] dpm_emu15 - gpio_26 - LCD_D(4) */
	OMAP4_MUX(DPM_EMU15, OMAP_MUX_MODE5 | OMAP_PIN_OFF_OUTPUT_LOW),
	/* [--OUT] dpm_emu16 - gpio_27 - LCD_D(3) */
	OMAP4_MUX(DPM_EMU16, OMAP_MUX_MODE5 | OMAP_PIN_OFF_OUTPUT_LOW),
	/* [--OUT] dpm_emu17 - gpio_28 - LCD_D(2) */
	OMAP4_MUX(DPM_EMU17, OMAP_MUX_MODE5 | OMAP_PIN_OFF_OUTPUT_LOW),
	/* [--OUT] dpm_emu18 - gpio_190 - LCD_D(1) */
	OMAP4_MUX(DPM_EMU18, OMAP_MUX_MODE5 | OMAP_PIN_OFF_OUTPUT_LOW),
	/* [--OUT] dpm_emu19 - gpio_191 - LCD_D(0) */
	OMAP4_MUX(DPM_EMU19, OMAP_MUX_MODE5 | OMAP_PIN_OFF_OUTPUT_LOW),

	{.reg_offset = OMAP_MUX_TERMINATOR},
};	/* end struct __omap_board_mux */

static struct omap_board_mux __omap_board_wk_mux[] __initdata = {
	/* [IN---] sim_io - gpio_wk0 - DET_3.5 */
	OMAP4_MUX(SIM_IO, OMAP_MUX_MODE3 | OMAP_PIN_INPUT | OMAP_PIN_OFF_WAKEUPENABLE),

	/* [-N-C-] sim_clk - gpio_wk1 - NC */
	OMAP4_MUX(SIM_CLK, OMAP_MUX_MODE7 | OMAP_PIN_INPUT_PULLDOWN),
	/* [--OUT] sim_reset - gpio_wk2 - CP_RST */
	OMAP4_MUX(SIM_RESET, OMAP_MUX_MODE3 | OMAP_PIN_OUTPUT),

	/* [-----] sr_scl - sr_scl - PMIC_I2C_SCL */
	OMAP4_MUX(SR_SCL, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	/* [-----] sr_sda - sr_sda - PMIC_I2C_SDA */
	OMAP4_MUX(SR_SDA, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),

	/* [IN---] sim_cd - gpio_wk3 - EXT_WAKEUP */
	OMAP4_MUX(SIM_CD, OMAP_MUX_MODE3 | OMAP_PIN_INPUT | OMAP_PIN_OFF_WAKEUPENABLE),
	/* [IN---] sim_pwrctrl - gpio_wk4 - EAR_SEND_END in rev 0.6*/
	OMAP4_MUX(SIM_PWRCTRL, OMAP_MUX_MODE3 | OMAP_PIN_INPUT | OMAP_PIN_OFF_WAKEUPENABLE),

	/* [-----] fref_clk0_out - sys_drm_msecure - SYS_DRM_MSEC */
	OMAP4_MUX(FREF_CLK0_OUT, OMAP_MUX_MODE2 | OMAP_PIN_OUTPUT),


	/* [IN---] fref_clk03_req - gpio_wk30 - VOL_UP */
	OMAP4_MUX(FREF_CLK3_REQ, OMAP_MUX_MODE3 | OMAP_PIN_INPUT | OMAP_PIN_OFF_WAKEUPENABLE),
	/* [IN---] fref_clk3_out - gpio_wk31 - HOME_KEY */
	OMAP4_MUX(FREF_CLK3_OUT, OMAP_MUX_MODE3 | OMAP_PIN_INPUT | OMAP_PIN_OFF_WAKEUPENABLE),
	/* [-N-C-] fref_clk04_req - gpio_wk7 - NC */
	OMAP4_MUX(FREF_CLK4_REQ, OMAP_MUX_MODE7 | OMAP_PIN_INPUT_PULLDOWN),
	/* [IN---] fref_clk4_out - gpio_wk8 - VOL_DOWN */
	OMAP4_MUX(FREF_CLK4_OUT, OMAP_MUX_MODE3 | OMAP_PIN_INPUT | OMAP_PIN_OFF_WAKEUPENABLE),

	/* [IN---] sys_pwron_reset_out - gpio_wk29 - BAT_REMOVAL */
	OMAP4_MUX(SYS_PWRON_RESET_OUT, OMAP_MUX_MODE3 | OMAP_PIN_INPUT),

	/* [IN---] sys_boot6 - gpio_wk9 - SYS_BOOT6 */
	OMAP4_MUX(SYS_BOOT6, OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLDOWN),
	/* [IN---] sys_boot7 - gpio_wk10 - SYS_BOOT7 */
	OMAP4_MUX(SYS_BOOT7, OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP),

	/* [-----] jtag_ntrst -  - AP_JTAG_nTRST */
	OMAP4_MUX(JTAG_NTRST, OMAP_PIN_INPUT_PULLDOWN),
	/* [-----] jtag_tck -  - AP_JTAG_TCK */
	OMAP4_MUX(JTAG_TCK, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLDOWN),
	/* [-----] jtag_rtck -	- AP_JTAG_RTCK */
	OMAP4_MUX(JTAG_RTCK, OMAP_PIN_OUTPUT),
	/* [-----] jtag_tms -  - AP_JTAG_TMS */
	OMAP4_MUX(JTAG_TMS_TMSC, OMAP_PIN_INPUT),
	/* [-----] jtag_tdi -  - AP_JTAG_TDI */
	OMAP4_MUX(JTAG_TDI, OMAP_PIN_INPUT_PULLUP),
	/* [-----] jtag_tdo -  - AP_JTAG_TDO */
	OMAP4_MUX(JTAG_TDO, OMAP_PIN_OUTPUT),

	{.reg_offset = OMAP_MUX_TERMINATOR},
};

unsigned int sec_board_mux_size = ARRAY_SIZE(__omap_board_core_mux);
EXPORT_SYMBOL(sec_board_mux_size);
struct omap_board_mux *sec_board_mux_ptr = __omap_board_core_mux;
EXPORT_SYMBOL(sec_board_mux_ptr);

struct omap_board_mux *sec_board_wk_mux_ptr = __omap_board_wk_mux;
EXPORT_SYMBOL(sec_board_wk_mux_ptr);

