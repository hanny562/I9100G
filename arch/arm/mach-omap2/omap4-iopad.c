/*
 * linux/arch/arm/mach-omap2/omap4-iopad.c
 *
 * OMAP4 I/O mapping code
 *
 * Copyright (C) 2011 Samsung Electronics
 *
 * Author: Sumeet Pawnikar <sumeet.p@samsung.com>
 *
 * Added OMAP4 io pad support - Configure the OFF Mode settings for io pads
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <plat/io.h>
#include "mux44xx.h"

#define WKUP_EN		(1 << 14)
#define OFF_PD		(1 << 12)
#define OFF_PU		(3 << 12)
#define OFF_OUT_PTD	(0 << 11)
#define OFF_OUT_PTU	(1 << 11)
#define OFF_IN		(1 << 10)
#define OFF_OUT		(0 << 10)
#define OFF_EN		(1 << 9)
#define OFF_DIS		(0 << 9)

#define IEN		(1 << 8)
#define IDIS		(0 << 8)
#define PTU		(1 << 4)
#define PTD		(0 << 4)
#define EN		(1 << 3)
#define DIS		(0 << 3)

#define M0		(0)
#define M1		(1)
#define M2		(2)
#define M3		(3)
#define M4		(4)
#define M5		(5)
#define M6		(6)
#define M7		(7)

#define OFF_IN_PD	(OFF_PD | OFF_IN | OFF_EN)
#define OFF_IN_PU	(OFF_PU | OFF_IN | OFF_EN)
#define OFF_OUT_PD	(OFF_OUT_PTD | OFF_OUT | OFF_EN)
#define OFF_OUT_PU	(OFF_OUT_PTU | OFF_OUT | OFF_EN)

#define MV(OFFSET, VALUE)	omap_writew((VALUE), OMAP4_CTRL_MODULE_PAD_CORE_MUX_PBASE + (OFFSET))
#define CP(x)			(OMAP4_CTRL_MODULE_PAD_##x##_OFFSET)


/*
 * IEN  - Input Enable
 * IDIS - Input Disable
 * PTD  - Pull type Down
 * PTU  - Pull type Up
 * DIS  - Pull type selection is inactive
 * EN   - Pull type selection is active
 * Mx   - Mode x
 */

struct omap44xx_pin_config {
	char           *omap_pad_name;
	char           *signal_name;
	unsigned int   offset;
	unsigned short func_mode;
	unsigned short off_mode;
};

struct omap44xx_pin_config pin_mux[] = {
#ifdef CONFIG_MACH_OMAP4_TAB_10_1
#if ( (CONFIG_SAMSUNG_OMAP4_TAB_REV <= 5) \
     || (CONFIG_SAMSUNG_OMAP4_TAB_REV == 7))
	/* HSMMC */
	{ "gpmc_ad0",	"HSMMC_D(0)",		0x040, (IEN | M1),	(OFF_PD | OFF_IN | OFF_EN) },
	{ "gpmc_ad1",	"HSMMC_D(1)",		0x042, (IEN | M1),	(OFF_PD | OFF_IN | OFF_EN) },
	{ "gpmc_ad2",	"HSMMC_D(2)",		0x044, (IEN | M1),	(OFF_PD | OFF_IN | OFF_EN) },
	{ "gpmc_ad3",	"HSMMC_D(3)",		0x046, (IEN | M1),	(OFF_PD | OFF_IN | OFF_EN) },
	{ "gpmc_ad4",	"HSMMC_D(4)",		0x048, (IEN | M1),	(OFF_PD | OFF_IN | OFF_EN) },
	{ "gpmc_ad5",	"HSMMC_D(5)",		0x04A, (IEN | M1),	(OFF_PD | OFF_IN | OFF_EN) },
	{ "gpmc_ad6",	"HSMMC_D(6)",		0x04C, (IEN | M1),	(OFF_PD | OFF_IN | OFF_EN) },
	{ "gpmc_ad7",	"HSMMC_D(7)",		0x04E, (IEN | M1),	(OFF_PD | OFF_IN | OFF_EN) },
	{ "gpmc_ad8",	"3_TOUCH_INT",		0x050, (IEN | M3),	(OFF_IN_PD) },
	{ "gpmc_ad9",	"PS_VOUT_18",		0x052, (IEN | M3),	(OFF_IN_PD) },
	{ "gpmc_ad10",	"CP_USB_ON",		0x054, (M3),		(OFF_IN_PU) },
	{ "gpmc_ad11",	"MLCD_RST",		0x056, (M3),		(OFF_IN_PD) },
	{ "gpmc_ad12",	"CP_ON",		0x058, (PTU | EN | M3),	(OFF_DIS) },
	{ "gpmc_ad13",	"PS_ON",		0x05A, (PTU | EN | M3),	(OFF_IN_PD) },
	//{ "gpmc_ad14",	"OLED_DET",	0x05C, (IEN | M3),	(OFF_IN | OFF_EN) },
	{ "gpmc_ad15",	"TA_CURRENT_SEL",	0x05E, (IEN | M3),	(OFF_IN | OFF_EN) },
	{ "gpmc_a16",	"LCD_EN",		0x060, (M3),		(OFF_IN_PD) },
	{ "gpmc_a17",	"XN_RST_OUT",		0x062, (IEN | EN | M3),(OFF_IN_PU) },
	{ "gpmc_a18",	"MOTOR_EN",		0x064, (M3),		(OFF_IN_PD) },
	{ "gpmc_a19",	"CAM_PMIC_EN",		0x066, (M3),		(OFF_IN_PD) },
	//{ "gpmc_a20",	"EARPATH_SEL",		0x068, (),	() },
	{ "gpmc_a21",	"GYRO_INT",		0x06A, (IEN | M3),	(OFF_IN | OFF_EN) },
	{ "gpmc_a22",	"TOUCH_nINT",		0x06C, (IEN | M3),	(OFF_IN | OFF_EN) },
	{ "gpmc_a23",	"UART_SEL",		0x06E, (PTU | EN | M3),	(OFF_DIS) },
	{ "gpmc_a24",	"MICBIAS_EN",		0x070, (M3),		(OFF_IN_PD) },
	{ "gpmc_a25",	"MICBIAS_EN2",		0x072, (M3),		(OFF_IN_PD) },
	//{ "gpmc_ncs0",	"RESET_REQ_N",		0x074, (),	() },
	{ "gpmc_ncs1",	"1.3M_nSTBY",		0x076, (PTU | EN | M3),	(OFF_OUT_PD) },
	//{ "gpmc_ncs2",	"IPC_HOST_WAKEUP",	0x078, (M3),		(OFF_DIS) },
	//{ "gpmc_ncs3",	"IPC_SLAVE_WAKEUP",	0x07A, (M3),		(OFF_OUT_PD) },
	{ "gpmc_nwp",	"TOUCH_EN",		0x07C, (PTU | EN | M3),	(OFF_OUT_PD) },
	{ "gpmc_clk",	"ACTIVE_STATE",		0x07E, (PTD | EN | M3),	(OFF_IN_PD) },
	//{ "gpmc_nadv_ale", "SUSPEND_REQUEST",	0x080, (),	() },
	{ "gpmc_noe",	"HSMMC_CLK",		0x082, (IEN | M1),	(OFF_PD | OFF_IN | OFF_EN) },
	{ "gpmc_nwe",	"HSMMC_CMD",		0x084, (IEN | M1),	(OFF_PD | OFF_IN | OFF_EN) },
	//{ "gpmc_nbe0_cle", "NFC_IRQ",		0x086, (),	() },
	{ "gpmc_nbe1",	"HDMI_LS_EN",		0x088, (M3),		(OFF_IN_PD) },
	{ "gpmc_wait0",	"WLAN_IRQ",		0x08A, (IEN | M3),	(OFF_IN | OFF_EN) },
	//{ "gpmc_wait1",	"DIC_ID",		0x08C, (),	() },
	{ "gpmc_wait2",	"HDMI_DCDC_EN",		0x08E, (M3),		(OFF_IN_PD) },
	{ "gpmc_ncs4",	"3_TOUCH_EN",		0x090, (PTU | EN | M3),	(OFF_OUT_PD) },
	{ "gpmc_ncs5",	"3_TOUCH_LED_EN",	0x092, (M3),		(OFF_OUT_PD) },
	{ "gpmc_ncs6",	"BT_EN",		0x094, (M3),		(OFF_IN_PD) },
	{ "gpmc_ncs7",	"WLAN_EN",		0x096, (M3),		(OFF_IN_PD) },

	{ "hdmi_hpd",	"AP_HDMI_HPD",		0x098, (IEN | PTD | EN | M0),	(OFF_DIS) },
	{ "hdmi_cec",	"AP_HDMI_CEC",		0x09A, (IEN | PTU | EN | M0),	(OFF_DIS) },
	{ "hdmi_ddc_scl", "AP_HDMI_I2C_SCL",	0x09C, (IEN | PTU | EN | M0),	(OFF_DIS) },
	{ "hdmi_ddc_sda", "AP_HDMI_I2C_SDA",	0x09E, (IEN | PTU | EN | M0),	(OFF_DIS) },

	//{ "csi21_dx0",	"8M_CLK_P",		0x0A0, (),		() },
	//{ "csi21_dy0",	"8M_CLK_N",		0x0A2, (),		() },
	//{ "csi21_dx1",	"8M_DP_1",		0x0A4, (),		() },
	//{ "csi21_dy1",	"8M_DN_1",		0x0A6, (),		() },
	//{ "csi21_dx2",	"8M_DP_2",		0x0A8, (),		() },
	//{ "csi21_dy2",	"8M_DN_2",		0x0AA, (),		() },
	{ "csi21_dx3",	"-nc-",		0x0AC, (PTD | EN | M7),		(OFF_DIS) },
	{ "csi21_dy3",	"-nc-",		0x0AE, (PTD | EN | M7),		(OFF_DIS) },
	{ "csi21_dx4",	"-nc-",		0x0B0, (PTD | EN | M7),		(OFF_DIS) },
	{ "csi21_dy4",	"-nc-",		0x0B2, (PTD | EN | M7),		(OFF_DIS) },

	//{ "csi22_dx0",	"1.3M_CLK_P",		0x0B4, (),		() },
	//{ "csi22_dy0",	"1.3M_CLK_N",		0x0B6, (),		() },
	//{ "csi22_dx1",	"1.3M_DATA_P",		0x0B8, (),		() },
	//{ "csi22_dy1",	"1.3M_DATA_N",		0x0BA, (),		() },

	{ "cam_shutter",	"-nc-",		0x0BC, (PTD | EN | M7),		(OFF_DIS) },
	{ "cam_strobe",		"-nc-",		0x0BE, (PTD | EN | M7),		(OFF_DIS) },
	{ "cam_globalreset",	"-nc-",		0x0C0, (PTD | EN | M7),		(OFF_DIS) },

	{ "usbb1_ulpitll_clk",	"MIPI_HSI_TX_WAKE",	0x0C2, (IEN | M1),	(OFF_IN_PD) },
	{ "usbb1_ulpitll_stp",	"MIPI_HSI_TX_DATA",	0x0C4, (IEN | M1),	(OFF_IN_PD) },
	{ "usbb1_ulpitll_dir",	"MIPI_HSI_TX_FLG",	0x0C6, (IEN | M1),	(OFF_IN_PD) },
	{ "usbb1_ulpitll_nxt",	"MIPI_HSI_TX_RDY",	0x0C8, (M1),		(OFF_OUT_PD) },
	{ "usbb1_ulpitll_dat0",	"MIPI_HSI_RX_WAKE",	0x0CA, (M1),		(OFF_OUT_PD) },
	{ "usbb1_ulpitll_dat1",	"MIPI_HSI_RX_DATA",	0x0CC, (M1),		(OFF_OUT_PD) },
	{ "usbb1_ulpitll_dat2",	"MIPI_HSI_RX_FLG",	0x0CE, (M1),		(OFF_OUT_PD) },
	{ "usbb1_ulpitll_dat3",	"MIPI_HSI_RX_RDY",	0x0D0, (IEN | M1),	(OFF_IN_PD) },
	//{ "usbb1_ulpitll_dat4",	"BOOT_MODE",		0x0D2, (),		() },
	{ "usbb1_ulpitll_dat5",	"-nc-",		0x0D4, (PTD | EN | M7),		(OFF_DIS) },
	//{ "usbb1_ulpitll_dat6",	"EAR_SEND_END",		0x0D6, (),		() },
	//{ "usbb1_ulpitll_dat7",	"JIG_ON_18",		0x0D8, (),		() },

	{ "usbb1_hsic_data",	"-nc-",		0x0DA, (M0),		(OFF_DIS) },
	{ "usbb1_hsic_strobe",	"-nc-",		0x0DC, (M0),		(OFF_DIS) },
	{ "usbc1_icusb_dp",	"-nc-",		0x0DE, (PTD | EN | M7),		(OFF_DIS) },
	{ "usbc1_icusb_dm",	"-nc-",		0x0E0, (PTD | EN | M7),		(OFF_DIS) },

	{ "sdmmc1_clk",  "T_FLASH_CLK",		0x0E2, (IEN | PTD | EN | M0),	(OFF_DIS) },
	{ "sdmmc1_cmd",  "T_FLASH_CLK",		0x0E4, (IEN | PTU | EN | M0),	(OFF_IN_PU) },
	{ "sdmmc1_dat0", "T_FLASH_D(0)",	0x0E6, (IEN | PTU | EN | M0),	(OFF_IN_PU) },
	{ "sdmmc1_dat1", "T_FLASH_D(1)",	0x0E8, (IEN | PTU | EN | M0),	(OFF_IN_PU) },
	{ "sdmmc1_dat2", "T_FLASH_D(2)",	0x0EA, (IEN | PTU | EN | M0),	(OFF_IN_PU) },
	{ "sdmmc1_dat3", "T_FLASH_D(3)",	0x0EC, (IEN | PTU | EN | M0),	(OFF_IN_PU) },
	{ "sdmmc1_dat4", "-nc-",		0x0EE, (PTD | EN | M7),		(OFF_DIS) },
	{ "sdmmc1_dat5", "-nc-",		0x0F0, (PTD | EN | M7),		(OFF_DIS) },
	{ "sdmmc1_dat6", "-nc-",		0x0F2, (PTD | EN | M7),		(OFF_DIS) },
	{ "sdmmc1_dat7", "-nc-",		0x0F4, (PTD | EN | M7),		(OFF_DIS) },

	//{ "abe_mcbsp2_clkx",	"REC_PCM_CLK",		0x0F6, (),		() },
	//{ "abe_mcbsp2_dr",	"REC_PCM_IN",		0x0F8, (),		() },
	//{ "abe_mcbsp2_dx",	"REC_PCM_OUT",		0x0FA, (),		() },
	//{ "abe_mcbsp2_fsx",	"REC_PCM_SYNC",		0x0FC, (),		() },
	{ "abe_mcbsp1_clkx",	"BT_PCM_CLK",		0x0FE, (IEN | M0),	(OFF_IN_PD) },
	{ "abe_mcbsp1_dr",	"BT_PCM_OUT",		0x100, (IEN | M0),	(OFF_IN_PD) },
	{ "abe_mcbsp1_dx",	"BT_PCM_DIN",		0x102, (M0),		(OFF_DIS) },
	{ "abe_mcbsp1_fsx",	"BT_PCM_SYNC",		0x104, (IEN | M0),	(OFF_IN_PD) },
	//{ "abe_pdm_ul_data",	"PDM_UL_DATA",		0x106, (),		() },
	//{ "abe_pdm_dl_data",	"PDM_DL_DATA",		0x108, (),		() },
	//{ "abe_pdm_frame",	"PDM_FRAME",		0x10A, (),		() },
	//{ "abe_pdm_lb_clk",	"PDM_CLK",		0x10C, (),		() },
	//{ "abe_clks",		"ABE_CLKS",		0x10E, (),		() },
	//{ "abe_dmic_clk1",	"PDA_ACTIVE",		0x110, (),		() },
	//{ "abe_dmic_din1",	"PHONE_ACTIVE",		0x112, (),		() },
	//{ "abe_dmic_din2",	"JACK_nINT",		0x114, (),		() },
	//{ "abe_dmic_din3",	"ACC_INT",		0x116, (),		() },

	//{ "uart2_cts",	"BT_UART_CTS",		0x118, (),		() },
	{ "uart2_rts",	"BT_UART_RTS",		0x11A, (M0),		(OFF_IN_PD) },
	//{ "uart2_rx",	"BT_UART_RXD",		0x11C, (),		() },
	//{ "uart2_tx",	"BT_UART_TXD",		0x11E, (),		() },

	{ "hdq_sio",	"AUD_PWRON",		0x120, (M3),		(OFF_IN_PD) },

	//{ "i2c1_scl",	"PHEONIX_I2C_SCL",	0x122, (),		() },
	//{ "i2c1_sda",	"PHEONIX_I2C_SDA",	0x124, (),		() },
	{ "i2c2_scl",	"CAM_I2C_SCL",		0x126, (IEN | PTU | EN | M0),	(OFF_DIS) },
	{ "i2c2_sda",	"CAM_I2C_SDA",		0x128, (IEN | PTU | EN | M0),	(OFF_DIS) },
	/* NOTE: Fuel gauge expects I2C SCL and SDA lines to be low to enter sleep mode. This save ~70uA */
	{ "i2c3_scl",	"AP_I2C_SCL",		0x12A, (IEN | PTU | EN | M0),	(OFF_IN_PD) },
	{ "i2c3_sda",	"AP_I2C_SDA",		0x12C, (IEN | PTU | EN | M0),	(OFF_IN_PD) },
	{ "i2c4_scl",	"GEN_I2C_SCL",		0x12E, (IEN | PTU | EN | M0),	(OFF_IN_PU) },
	{ "i2c4_sda",	"GEN_I2C_SDA",		0x130, (IEN | PTU | EN | M0),	(OFF_IN_PU) },

	{ "mcspi1_clk",  "USB_OTG_EN",		0x132, (PTD | EN | M3),		(OFF_IN_PD) },
	//{ "mcspi1_somi", "NFC_FIRMWARE",	0x134, (),		() },
	{ "mcspi1_simo", "NFC_EN",		0x136, (M3),		(OFF_IN_PD) },
	{ "mcspi1_cs0",  "-nc-",		0x138, (IEN | PTD | EN | M3),		(OFF_IN_PD) },
	{ "mcspi1_cs1",  "8M_ISP_INT",		0x13A, (IEN | EN | M3),		(OFF_IN_PD) },
	{ "mcspi1_cs2",  "8M_nRST",		0x13C, (M3),		(OFF_OUT_PD) },
	{ "mcspi1_cs3",  "1.3M_nRST",		0x13E, (M3),		(OFF_OUT_PD) },

	{ "uart3_cts_rctx",	"-nc-",		0x140, (PTD | EN | M3),		(OFF_IN_PD) },
	{ "uart3_rts_sd",	"-nc-",		0x142, (PTD | EN | M3),		(OFF_IN_PD) },
	{ "uart3_rx_irrx",	"AP_FLM_RXD",	0x144, (IEN | M0),		(OFF_DIS) },
	{ "uart3_tx_irtx",	"AP_FLM_TXD",	0x146, (M0),			(OFF_DIS) },

	{ "sdmmc5_clk",  "WLAN_SD_CLK",		0x148, (IEN | M0),		(OFF_IN_PD) },
	{ "sdmmc5_cmd",  "WLAN_SD_CMD",		0x14A, (IEN | PTU | EN | M0),	(OFF_DIS) },
	{ "sdmmc5_dat0", "WLAN_SD_D(0)",	0x14C, (IEN | PTU | EN | M0),	(OFF_DIS) },
	{ "sdmmc5_dat1", "WLAN_SD_D(1)",	0x14E, (IEN | PTU | EN | M0),	(OFF_DIS) },
	{ "sdmmc5_dat2", "WLAN_SD_D(2)",	0x150, (IEN | PTU | EN | M0),	(OFF_DIS) },
	{ "sdmmc5_dat3", "WLAN_SD_D(3)",	0x152, (IEN | PTU | EN | M0),	(OFF_DIS) },

	{ "mcspi4_clk",  "LCD_SCLK",		0x154, (IEN | PTD | EN | M0),	(OFF_OUT_PD) },
	{ "mcspi4_simo", "LCD_SDI",		0x156, (IEN | PTU | EN | M0),	(OFF_OUT_PU) },
	{ "mcspi4_somi", "-nc-",		0x158, (IEN | PTD | EN | M3),	(OFF_IN_PD) },
	{ "mcspi4_cs0",  "LCD_nCS",		0x15A, (IEN | PTU | EN | M0),	(OFF_OUT_PU) },

	{ "uart4_rx",	"AP_RXD",		0x15C, (IEN | M0),	(OFF_IN_PD) },
	{ "uart4_tx",	"AP_TXD",		0x15E, (M0),		(OFF_OUT_PD) },

	{ "usbb2_ulpitll_clk",	"MSENSE_IRQ",	0x160, (IEN | M3),	(OFF_IN_PU) },
	{ "usbb2_ulpitll_stp",	"LCD_D(23)",	0x162, (PTD | EN | M5),	(OFF_OUT_PD) },
	{ "usbb2_ulpitll_dir",	"LCD_D(22)",	0x164, (PTD | EN | M5),	(OFF_OUT_PD) },
	{ "usbb2_ulpitll_nxt",	"LCD_D(21)",	0x166, (PTD | EN | M5),	(OFF_OUT_PD) },
	{ "usbb2_ulpitll_dat0",	"LCD_D(20)",	0x168, (PTD | EN | M5),	(OFF_OUT_PD) },
	{ "usbb2_ulpitll_dat1",	"LCD_D(19)",	0x16A, (PTD | EN | M5),	(OFF_OUT_PD) },
	{ "usbb2_ulpitll_dat2",	"LCD_D(18)",	0x16C, (PTD | EN | M5),	(OFF_OUT_PD) },
	{ "usbb2_ulpitll_dat3",	"LCD_D(15)",	0x16E, (PTD | EN | M5),	(OFF_OUT_PD) },
	{ "usbb2_ulpitll_dat4",	"LCD_D(14)",	0x170, (PTD | EN | M5),	(OFF_OUT_PD) },
	{ "usbb2_ulpitll_dat5",	"LCD_D(13)",	0x172, (PTD | EN | M5),	(OFF_OUT_PD) },
	{ "usbb2_ulpitll_dat6",	"LCD_D(12)",	0x174, (PTD | EN | M5),	(OFF_OUT_PD) },
	{ "usbb2_ulpitll_dat7",	"LCD_D(11)",	0x176, (PTD | EN | M5),	(OFF_OUT_PD) },
	//{ "usbb2_hsic_data",	"2MIC_EN",	0x178, (),		() },
	{ "usbb2_hsic_strobe",	"-nc-",		0x17A, (PTD | EN | M7),		(OFF_DIS) },

	{ "kpd_col3",	"HW_REV3",	0x17C, (IEN | PTD | EN | M3),	(OFF_DIS) },
	{ "kpd_col4",	"HW_REV2",	0x17E, (IEN | PTU | EN | M3),	(OFF_DIS) },
	{ "kpd_col5",	"HW_REV0",	0x180, (IEN | PTU | EN | M3),	(OFF_DIS) },
	{ "kpd_col0",	"-nc-",		0x182, (PTD | EN | M3),		(OFF_DIS) },
	{ "kpd_col1",	"KBC(1)",	0x184, (M0),		(OFF_OUT_PD) },
	{ "kpd_col2",	"KBC(2)",	0x186, (M0),		(OFF_OUT_PD) },
	{ "kpd_row3",	"HW_REV1",	0x188, (IEN | PTD | EN | M3),	(OFF_DIS) },
	//{ "kpd_row4",	"HOME_KEY",	0x18A, (IEN | PTU | EN | M3),	(OFF_DIS) },
	{ "kpd_row5",	"-nc-",		0x18C, (PTD | EN | M3),		(OFF_DIS) },
	{ "kpd_row0",	"-nc-",		0x18E, (PTD | EN | M3),		(OFF_DIS) },
	//{ "kpd_row1",	"KBR(1)",	0x190, (IEN | M0),		(OFF_DIS) },
	//{ "kpd_row2",	"KBR(2)",	0x192, (IEN | M0),		(OFF_IN_PD) },

	//{ "usba0_otg_ce", "USB_CHGEN",	0x194, (),		() },
	//{ "usba0_otg_dp", "AP_D+",	0x196, (),		() },
	//{ "usba0_otg_dm", "AP_D-",	0x198, (),		() },

	{ "fref_clk1_out", "8M_MCLK",	0x19A, (M0),		(OFF_OUT_PD) },
	{ "fref_clk2_out", "1.3M_MCLK",	0x19C, (M0),		(OFF_OUT_PD) },

	//{ "sys_nirq1",	"SYS_nIRQ1",	0x19E, (),		() },
	//{ "sys_nirq2",	"SYS_nIRQ2",	0x1A0, (),		() },

	{ "sys_boot0",	"SYS_BOOT0",	0x1A2, (IEN | M0),		(OFF_DIS) },
	{ "sys_boot1",	"SYS_BOOT1",	0x1A4, (IEN | M0),		(OFF_DIS) },
	{ "sys_boot2",	"SYS_BOOT2",	0x1A6, (IEN | M0),		(OFF_DIS) },
	{ "sys_boot3",	"SYS_BOOT3",	0x1A8, (IEN | PTD | EN | M0),	(OFF_DIS) },
	{ "sys_boot4",	"SYS_BOOT4",	0x1AA, (IEN | PTD | EN | M0),	(OFF_DIS) },
	{ "sys_boot5",	"SYS_BOOT5",	0x1AC, (IEN | PTD | EN | M0),	(OFF_DIS) },

	/* NOTE: Do not set OFF mode setting for 2MIC_RST and 2MIC_POWERDOWN.
	 * Specific sequence is required to put the 2MIC in sleep mode. It is
	 * handled by driver. */
	{ "dpm_emu0",	"2MIC_RST",		0x1AE, (M3),	(OFF_DIS) },
	{ "dpm_emu1",	"2MIC_POWERDOWN",	0x1B0, (M3),	(OFF_DIS) },

	{ "dpm_emu2",	"OLED_ID",	0x1B2, (PTD | EN | M5),		(OFF_OUT_PD) },
	{ "dpm_emu3",	"LCD_D(10)",	0x1B4, (PTD | EN | M5),		(OFF_OUT_PD) },
	{ "dpm_emu4",	"LCD_D(9)",	0x1B6, (PTD | EN | M5),		(OFF_OUT_PD) },
	{ "dpm_emu5",	"LCD_D(16)",	0x1B8, (PTD | EN | M5),		(OFF_OUT_PD) },
	{ "dpm_emu6",	"LCD_D(17)",	0x1BA, (PTD | EN | M5),		(OFF_OUT_PD) },
	{ "dpm_emu7",	"LCD_HSYNC",	0x1BC, (PTD | EN | M5),		(OFF_OUT_PD) },
	{ "dpm_emu8",	"LCD_PCLK",	0x1BE, (PTD | EN | M5),		(OFF_OUT_PD) },
	{ "dpm_emu9",	"LCD_VSYNC",	0x1C0, (PTD | EN | M5),		(OFF_OUT_PD) },
	{ "dpm_emu10",	"LCD_DE",	0x1C2, (PTD | EN | M5),		(OFF_OUT_PD) },
	{ "dpm_emu11",	"LCD_D(8)",	0x1C4, (PTD | EN | M5),		(OFF_OUT_PD) },
	{ "dpm_emu12",	"LCD_D(7)",	0x1C6, (PTD | EN | M5),		(OFF_OUT_PD) },
	{ "dpm_emu13",	"LCD_D(6)",	0x1C8, (PTD | EN | M5),		(OFF_OUT_PD) },
	{ "dpm_emu14",	"LCD_D(5)",	0x1CA, (PTD | EN | M5),		(OFF_OUT_PD) },
	{ "dpm_emu15",	"LCD_D(4)",	0x1CC, (PTD | EN | M5),		(OFF_OUT_PD) },
	{ "dpm_emu16",	"LCD_D(3)",	0x1CE, (PTD | EN | M5),		(OFF_OUT_PD) },
	{ "dpm_emu17",	"LCD_D(2)",	0x1D0, (PTD | EN | M5),		(OFF_OUT_PD) },
	{ "dpm_emu18",	"LCD_D(1)",	0x1D2, (PTD | EN | M5),		(OFF_OUT_PD) },
	{ "dpm_emu19",	"LCD_D(0)",	0x1D4, (PTD | EN | M5),		(OFF_OUT_PD) },
#endif

#if (CONFIG_SAMSUNG_OMAP4_TAB_REV == 6)
	/* HSMMC */
	{ "gpmc_ad0",	"HSMMC_D(0)",		0x040, (IEN | M1),	(OFF_PD | OFF_IN | OFF_EN) },
	{ "gpmc_ad1",	"HSMMC_D(1)",		0x042, (IEN | M1),	(OFF_PD | OFF_IN | OFF_EN) },
	{ "gpmc_ad2",	"HSMMC_D(2)",		0x044, (IEN | M1),	(OFF_PD | OFF_IN | OFF_EN) },
	{ "gpmc_ad3",	"HSMMC_D(3)",		0x046, (IEN | M1),	(OFF_PD | OFF_IN | OFF_EN) },
	{ "gpmc_ad4",	"HSMMC_D(4)",		0x048, (IEN | M1),	(OFF_PD | OFF_IN | OFF_EN) },
	{ "gpmc_ad5",	"HSMMC_D(5)",		0x04A, (IEN | M1),	(OFF_PD | OFF_IN | OFF_EN) },
	{ "gpmc_ad6",	"HSMMC_D(6)",		0x04C, (IEN | M1),	(OFF_PD | OFF_IN | OFF_EN) },
	{ "gpmc_ad7",	"HSMMC_D(7)",		0x04E, (IEN | M1),	(OFF_PD | OFF_IN | OFF_EN) },
	{ "gpmc_ad8",	"3_TOUCH_INT",		0x050, (IEN | M3),	(OFF_IN_PD) },
	{ "gpmc_ad9",	"PS_VOUT_18",		0x052, (IEN | M3),	(OFF_IN_PD) },
	{ "gpmc_ad10",	"CP_USB_ON",		0x054, (M3),		(OFF_IN_PU) },
	{ "gpmc_ad11",	"MLCD_RST",		0x056, (M3),		(OFF_IN_PD) },
	{ "gpmc_ad12",	"CP_ON",		0x058, (PTU | EN | M3),	(OFF_DIS) },
	{ "gpmc_ad13",	"PS_ON",		0x05A, (PTU | EN | M3),	(OFF_IN_PD) },
	//{ "gpmc_ad14",	"OLED_DET",	0x05C, (IEN | M3),	(OFF_IN | OFF_EN) },
	{ "gpmc_ad15",	"TA_CURRENT_SEL",	0x05E, (IEN | M3),	(OFF_IN | OFF_EN) },
	{ "gpmc_a16",	"LCD_EN",		0x060, (M3),		(OFF_IN_PD) },
	{ "gpmc_a17",	"8M_ISP_INT",		0x062, (IEN | EN | M3), (OFF_IN_PD) },
	{ "gpmc_a18",	"-nc-",			0x064, (PTD | EN | M7),	(OFF_DIS) },
	{ "gpmc_a19",	"CAM_PMIC_EN",		0x066, (M3),		(OFF_IN_PD) },
	{ "gpmc_a20",	"WLAN_BT_SEL",		0x068, (PTD | EN | M7),	(OFF_DIS) },
	{ "gpmc_a21",	"GYRO_INT",		0x06A, (IEN | M3),	(OFF_IN | OFF_EN) },
	{ "gpmc_a22",	"TOUCH_nINT",		0x06C, (IEN | M3),	(OFF_IN | OFF_EN) },
	{ "gpmc_a23",	"UART_SEL",		0x06E, (PTU | EN | M3),	(OFF_DIS) },
	{ "gpmc_a24",	"MICBIAS_EN",		0x070, (M3),		(OFF_IN_PD) },
	{ "gpmc_a25",	"MICBIAS_EN2",		0x072, (M3),		(OFF_IN_PD) },
	//{ "gpmc_ncs0",	"RESET_REQ_N",		0x074, (),	() },
	{ "gpmc_ncs1",	"1.3M_nSTBY",		0x076, (PTU | EN | M3),	(OFF_OUT_PD) },
	//{ "gpmc_ncs2",	"IPC_HOST_WAKEUP",	0x078, (M3),		(OFF_DIS) },
	//{ "gpmc_ncs3",	"IPC_SLAVE_WAKEUP",	0x07A, (M3),		(OFF_OUT_PD) },
	{ "gpmc_nwp",	"TOUCH_EN",		0x07C, (PTU | EN | M3),	(OFF_OUT_PD) },
	{ "gpmc_clk",	"ACTIVE_STATE",		0x07E, (PTD | EN | M3),	(OFF_IN_PD) },
	//{ "gpmc_nadv_ale", "SUSPEND_REQUEST",	0x080, (),	() },
	{ "gpmc_noe",	"HSMMC_CLK",		0x082, (IEN | M1),	(OFF_PD | OFF_IN | OFF_EN) },
	{ "gpmc_nwe",	"HSMMC_CMD",		0x084, (IEN | M1),	(OFF_PD | OFF_IN | OFF_EN) },
	//{ "gpmc_nbe0_cle", "NFC_IRQ",		0x086, (),	() },
	//{ "gpmc_nbe1",	"MHL_RST",		0x088, (M3),		(OFF_OUT_PD) },
	{ "gpmc_wait0",	"WLAN_IRQ",		0x08A, (IEN | M3),	(OFF_IN | OFF_EN) },
	//{ "gpmc_wait1",	"DIC_ID",		0x08C, (),	() },
	{ "gpmc_wait2",	"HDMI_EN",		0x08E, (M3),		(OFF_OUT_PD) },
	{ "gpmc_ncs4",	"3_TOUCH_EN",		0x090, (PTU | EN | M3),	(OFF_OUT_PD) },
	//{ "gpmc_ncs5",	"CMC_LDO_EN2",		0x092, (M3),		(OFF_OUT_PD) },
	{ "gpmc_ncs6",	"BT_EN",		0x094, (M3),		(OFF_IN_PD) },
	{ "gpmc_ncs7",	"WLAN_EN",		0x096, (M3),		(OFF_IN_PD) },

	{ "hdmi_hpd",	"MHL_HPD",		0x098, (IEN | PTU | EN | M0),	(OFF_DIS) },
	{ "hdmi_cec",	"MHL_WAKE_UP",		0x09A, (PTD | EN | M0),		(OFF_DIS) },
	{ "hdmi_ddc_scl", "MHL_SCL18v",		0x09C, (PTU | EN | M0),		(OFF_DIS) },
	{ "hdmi_ddc_sda", "MHL_SDA18V",		0x09E, (IEN | PTU | EN | M0),	(OFF_DIS) },

	//{ "csi21_dx0",	"8M_CLK_P",		0x0A0, (),		() },
	//{ "csi21_dy0",	"8M_CLK_N",		0x0A2, (),		() },
	//{ "csi21_dx1",	"8M_DP_1",		0x0A4, (),		() },
	//{ "csi21_dy1",	"8M_DN_1",		0x0A6, (),		() },
	//{ "csi21_dx2",	"8M_DP_2",		0x0A8, (),		() },
	//{ "csi21_dy2",	"8M_DN_2",		0x0AA, (),		() },
	{ "csi21_dx3",	"HW_REV3",		0x0AC, (IEN | PTD | EN | M3),		(OFF_DIS) },
	{ "csi21_dy3",	"HW_REV2",		0x0AE, (IEN | PTU | EN | M3),		(OFF_DIS) },
	{ "csi21_dx4",	"HW_REV1",		0x0B0, (IEN | PTD | EN | M3),		(OFF_DIS) },
	{ "csi21_dy4",	"HW_REV0",		0x0B2, (IEN | PTU | EN | M3),		(OFF_DIS) },

	//{ "csi22_dx0",	"1.3M_CLK_P",		0x0B4, (),		() },
	//{ "csi22_dy0",	"1.3M_CLK_N",		0x0B6, (),		() },
	//{ "csi22_dx1",	"1.3M_DATA_P",		0x0B8, (),		() },
	//{ "csi22_dy1",	"1.3M_DATA_N",		0x0BA, (),		() },

	{ "cam_shutter", "WLAN_HOST_WAKE",	0x0BC, (PTD | EN | M7),		(OFF_DIS) },
	{ "cam_strobe",	"BT_nRST",		0x0BE, (PTD | EN | M3),		(OFF_OUT_PD) },
	{ "cam_globalreset", "BT_HOST_WAKE",	0x0C0, (PTD | EN | M7),		(OFF_DIS) },

	{ "usbb1_ulpitll_clk",	"MIPI_HSI_TX_WAKE",	0x0C2, (IEN | M1),	(OFF_IN_PD) },
	{ "usbb1_ulpitll_stp",	"MIPI_HSI_TX_DATA",	0x0C4, (IEN | M1),	(OFF_IN_PD) },
	{ "usbb1_ulpitll_dir",	"MIPI_HSI_TX_FLG",	0x0C6, (IEN | M1),	(OFF_IN_PD) },
	{ "usbb1_ulpitll_nxt",	"MIPI_HSI_TX_RDY",	0x0C8, (M1),		(OFF_OUT_PD) },
	{ "usbb1_ulpitll_dat0",	"MIPI_HSI_RX_WAKE",	0x0CA, (M1),		(OFF_OUT_PD) },
	{ "usbb1_ulpitll_dat1",	"MIPI_HSI_RX_DATA",	0x0CC, (M1),		(OFF_OUT_PD) },
	{ "usbb1_ulpitll_dat2",	"MIPI_HSI_RX_FLG",	0x0CE, (M1),		(OFF_OUT_PD) },
	{ "usbb1_ulpitll_dat3",	"MIPI_HSI_RX_RDY",	0x0D0, (IEN | M1),	(OFF_IN_PD) },
	//{ "usbb1_ulpitll_dat4",	"BOOT_MODE",		0x0D2, (),		() },
	{ "usbb1_ulpitll_dat5",	"BT_WAKE",	0x0D4, (PTD | EN | M3),		(OFF_OUT_PD) },
	//{ "usbb1_ulpitll_dat6",	"EAR_SEND_END",		0x0D6, (),		() },
	//{ "usbb1_ulpitll_dat7",	"JIG_ON_18",		0x0D8, (),		() },

	{ "usbb1_hsic_data",	"-nc-",		0x0DA, (M0),		(OFF_DIS) },
	{ "usbb1_hsic_strobe",	"-nc-",		0x0DC, (M0),		(OFF_DIS) },
	{ "usbc1_icusb_dp",	"-nc-",		0x0DE, (PTD | EN | M7),		(OFF_DIS) },
	{ "usbc1_icusb_dm",	"-nc-",		0x0E0, (PTD | EN | M7),		(OFF_DIS) },

	{ "sdmmc1_clk",  "T_FLASH_CLK",		0x0E2, (IEN | PTD | EN | M0),	(OFF_DIS) },
	{ "sdmmc1_cmd",  "T_FLASH_CLK",		0x0E4, (IEN | PTU | EN | M0),	(OFF_IN_PU) },
	{ "sdmmc1_dat0", "T_FLASH_D(0)",	0x0E6, (IEN | PTU | EN | M0),	(OFF_IN_PU) },
	{ "sdmmc1_dat1", "T_FLASH_D(1)",	0x0E8, (IEN | PTU | EN | M0),	(OFF_IN_PU) },
	{ "sdmmc1_dat2", "T_FLASH_D(2)",	0x0EA, (IEN | PTU | EN | M0),	(OFF_IN_PU) },
	{ "sdmmc1_dat3", "T_FLASH_D(3)",	0x0EC, (IEN | PTU | EN | M0),	(OFF_IN_PU) },
	{ "sdmmc1_dat4", "-nc-",		0x0EE, (PTD | EN | M7),		(OFF_DIS) },
	{ "sdmmc1_dat5", "-nc-",		0x0F0, (PTD | EN | M7),		(OFF_DIS) },
	{ "sdmmc1_dat6", "-nc-",		0x0F2, (PTD | EN | M7),		(OFF_DIS) },
	{ "sdmmc1_dat7", "-nc-",		0x0F4, (PTD | EN | M7),		(OFF_DIS) },

	//{ "abe_mcbsp2_clkx",	"REC_PCM_CLK",		0x0F6, (),		() },
	//{ "abe_mcbsp2_dr",	"REC_PCM_IN",		0x0F8, (),		() },
	//{ "abe_mcbsp2_dx",	"REC_PCM_OUT",		0x0FA, (),		() },
	//{ "abe_mcbsp2_fsx",	"REC_PCM_SYNC",		0x0FC, (),		() },
	{ "abe_mcbsp1_clkx",	"BT_PCM_CLK",		0x0FE, (IEN | M0),	(OFF_IN_PD) },
	{ "abe_mcbsp1_dr",	"BT_PCM_OUT",		0x100, (IEN | M0),	(OFF_IN_PD) },
	{ "abe_mcbsp1_dx",	"BT_PCM_DIN",		0x102, (M0),		(OFF_DIS) },
	{ "abe_mcbsp1_fsx",	"BT_PCM_SYNC",		0x104, (IEN | M0),	(OFF_IN_PD) },
	//{ "abe_pdm_ul_data",	"PDM_UL_DATA",		0x106, (),		() },
	//{ "abe_pdm_dl_data",	"PDM_DL_DATA",		0x108, (),		() },
	//{ "abe_pdm_frame",	"PDM_FRAME",		0x10A, (),		() },
	//{ "abe_pdm_lb_clk",	"PDM_CLK",		0x10C, (),		() },
	//{ "abe_clks",		"ABE_CLKS",		0x10E, (),		() },
	//{ "abe_dmic_clk1",	"PDA_ACTIVE",		0x110, (),		() },
	//{ "abe_dmic_din1",	"PHONE_ACTIVE",		0x112, (),		() },
	//{ "abe_dmic_din2",	"JACK_nINT",		0x114, (),		() },
	//{ "abe_dmic_din3",	"ACC_INT",		0x116, (),		() },

	//{ "uart2_cts",	"BT_UART_CTS",		0x118, (),		() },
	{ "uart2_rts",	"BT_UART_RTS",		0x11A, (M0),		(OFF_IN_PD) },
	//{ "uart2_rx",	"BT_UART_RXD",		0x11C, (),		() },
	//{ "uart2_tx",	"BT_UART_TXD",		0x11E, (),		() },

	{ "hdq_sio",	"AUD_PWRON",		0x120, (M3),		(OFF_IN_PD) },

	//{ "i2c1_scl",	"PHEONIX_I2C_SCL",	0x122, (),		() },
	//{ "i2c1_sda",	"PHEONIX_I2C_SDA",	0x124, (),		() },
	{ "i2c2_scl",	"CAM_I2C_SCL",		0x126, (IEN | PTU | EN | M0),	(OFF_DIS) },
	{ "i2c2_sda",	"CAM_I2C_SDA",		0x128, (IEN | PTU | EN | M0),	(OFF_DIS) },
	/* NOTE: Fuel gauge expects I2C SCL and SDA lines to be low to enter sleep mode. This save ~70uA */
	{ "i2c3_scl",	"AP_I2C_SCL",		0x12A, (IEN | PTU | EN | M0),	(OFF_IN_PD) },
	{ "i2c3_sda",	"AP_I2C_SDA",		0x12C, (IEN | PTU | EN | M0),	(OFF_IN_PD) },
	{ "i2c4_scl",	"GEN_I2C_SCL",		0x12E, (IEN | PTU | EN | M0),	(OFF_IN_PU) },
	{ "i2c4_sda",	"GEN_I2C_SDA",		0x130, (IEN | PTU | EN | M0),	(OFF_IN_PU) },

	{ "mcspi1_clk",  "AP_AGPS_TSYNC",	0x132, (PTD | EN | M3),		(OFF_OUT_PD) },
	//{ "mcspi1_somi", "GPS_LNA_EN",	0x134, (),		() },
	{ "mcspi1_simo", "GPS_nRST",		0x136, (PTD | EN | M3),		(OFF_OUT_PD) },
	{ "mcspi1_cs0",  "GPS_EN",		0x138, (PTD | EN | M3),		(OFF_OUT_PD) },
	{ "mcspi1_cs1",  "GPS_UART_RXD",	0x13A, (IEN | PTD | EN | M3),	(OFF_IN_PD) },
	{ "mcspi1_cs2",  "GPS_UART_RTS",	0x13C, (M3),		(OFF_OUT_PD) },
	{ "mcspi1_cs3",  "GPS_UART_CTS",	0x13E, (M3),		(OFF_OUT_PD) },
	{ "uart3_cts_rctx", "GPS_UART_TXD",	0x140, (PTD | EN | M3),		(OFF_IN_PD) },
	{ "uart3_rts_sd", "XN_RST_OUT",		0x142, (PTD | EN | M3),		(OFF_IN_PD) },
	{ "uart3_rx_irrx",	"AP_FLM_RXD",	0x144, (IEN | M0),		(OFF_DIS) },
	{ "uart3_tx_irtx",	"AP_FLM_TXD",	0x146, (M0),			(OFF_DIS) },

	{ "sdmmc5_clk",  "WLAN_SD_CLK",		0x148, (IEN | M0),		(OFF_IN_PD) },
	{ "sdmmc5_cmd",  "WLAN_SD_CMD",		0x14A, (IEN | PTU | EN | M0),	(OFF_DIS) },
	{ "sdmmc5_dat0", "WLAN_SD_D(0)",	0x14C, (IEN | PTU | EN | M0),	(OFF_DIS) },
	{ "sdmmc5_dat1", "WLAN_SD_D(1)",	0x14E, (IEN | PTU | EN | M0),	(OFF_DIS) },
	{ "sdmmc5_dat2", "WLAN_SD_D(2)",	0x150, (IEN | PTU | EN | M0),	(OFF_DIS) },
	{ "sdmmc5_dat3", "WLAN_SD_D(3)",	0x152, (IEN | PTU | EN | M0),	(OFF_DIS) },

	{ "mcspi4_clk",  "LCD_SCLK",		0x154, (IEN | PTD | EN | M0),	(OFF_OUT_PD) },
	{ "mcspi4_simo", "LCD_SDI",		0x156, (IEN | PTU | EN | M0),	(OFF_OUT_PU) },
	{ "mcspi4_somi", "8M_nRST",		0x158, (PTD | EN | M3),		(OFF_OUT_PD) },
	{ "mcspi4_cs0",  "LCD_nCS",		0x15A, (IEN | PTU | EN | M0),	(OFF_OUT_PU) },

	{ "uart4_rx",	"AP_RXD",		0x15C, (IEN | M0),	(OFF_IN_PD) },
	{ "uart4_tx",	"AP_TXD",		0x15E, (M0),		(OFF_OUT_PD) },

	{ "usbb2_ulpitll_clk",	"MSENSE_IRQ",	0x160, (IEN | M3),	(OFF_IN_PU) },
	{ "usbb2_ulpitll_stp",	"LCD_D(23)",	0x162, (PTD | EN | M5),	(OFF_OUT_PD) },
	{ "usbb2_ulpitll_dir",	"LCD_D(22)",	0x164, (PTD | EN | M5),	(OFF_OUT_PD) },
	{ "usbb2_ulpitll_nxt",	"LCD_D(21)",	0x166, (PTD | EN | M5),	(OFF_OUT_PD) },
	{ "usbb2_ulpitll_dat0",	"LCD_D(20)",	0x168, (PTD | EN | M5),	(OFF_OUT_PD) },
	{ "usbb2_ulpitll_dat1",	"LCD_D(19)",	0x16A, (PTD | EN | M5),	(OFF_OUT_PD) },
	{ "usbb2_ulpitll_dat2",	"LCD_D(18)",	0x16C, (PTD | EN | M5),	(OFF_OUT_PD) },
	{ "usbb2_ulpitll_dat3",	"LCD_D(15)",	0x16E, (PTD | EN | M5),	(OFF_OUT_PD) },
	{ "usbb2_ulpitll_dat4",	"LCD_D(14)",	0x170, (PTD | EN | M5),	(OFF_OUT_PD) },
	{ "usbb2_ulpitll_dat5",	"LCD_D(13)",	0x172, (PTD | EN | M5),	(OFF_OUT_PD) },
	{ "usbb2_ulpitll_dat6",	"LCD_D(12)",	0x174, (PTD | EN | M5),	(OFF_OUT_PD) },
	{ "usbb2_ulpitll_dat7",	"LCD_D(11)",	0x176, (PTD | EN | M5),	(OFF_OUT_PD) },
	//{ "usbb2_hsic_data",	"2MIC_EN",	0x178, (),		() },
	{ "usbb2_hsic_strobe",	"MHL_SEL",		0x17A, (PTD | EN | M7),		(OFF_DIS) },
	{ "kpd_col3",	"USB_OTG_EN",	0x17C, (PTD | EN | M3),	(OFF_IN_PD) },
	{ "kpd_col4",	"NFC_FIRMWARE",	0x17E, (IEN | PTU | EN | M3),	(OFF_DIS) },
	{ "kpd_col5",	"NFC_EN",	0x180, (M3),	(OFF_IN_PD) },
	{ "kpd_col0",	"13M_nRST",	0x182, (PTD | EN | M3),		(OFF_DIS) },
	{ "kpd_col1",	"KBC(1)",	0x184, (M0),		(OFF_OUT_PD) },
	{ "kpd_col2",	"KBC(2)",	0x186, (M0),		(OFF_OUT_PD) },
	{ "kpd_row3",	"MHL_INT",	0x188, (PTU | EN | M3),	(OFF_OUT_PU) },
	//{ "kpd_row4",	"HOME_KEY",	0x18A, (IEN | PTU | EN | M3),	(OFF_DIS) },
	{ "kpd_row5",	"NFC_EN",	0x18C, (PTD | EN | M3),		(OFF_DIS) },
	{ "kpd_row0",	"-nc-",		0x18E, (PTD | EN | M3),		(OFF_DIS) },
	//{ "kpd_row1",	"KBR(1)",	0x190, (IEN | M0),		(OFF_DIS) },
	//{ "kpd_row2",	"KBR(2)",	0x192, (IEN | M0),		(OFF_IN_PD) },

	//{ "usba0_otg_ce", "USB_CHGEN",	0x194, (),		() },
	//{ "usba0_otg_dp", "AP_D+",	0x196, (),		() },
	//{ "usba0_otg_dm", "AP_D-",	0x198, (),		() },

	{ "fref_clk1_out", "8M_MCLK",	0x19A, (M0),		(OFF_OUT_PD) },
	{ "fref_clk2_out", "1.3M_MCLK",	0x19C, (M0),		(OFF_OUT_PD) },

	//{ "sys_nirq1",	"SYS_nIRQ1",	0x19E, (),		() },
	//{ "sys_nirq2",	"SYS_nIRQ2",	0x1A0, (),		() },

	{ "sys_boot0",	"SYS_BOOT0",	0x1A2, (IEN | M0),		(OFF_DIS) },
	{ "sys_boot1",	"SYS_BOOT1",	0x1A4, (IEN | M0),		(OFF_DIS) },
	{ "sys_boot2",	"SYS_BOOT2",	0x1A6, (IEN | M0),		(OFF_DIS) },
	{ "sys_boot3",	"SYS_BOOT3",	0x1A8, (IEN | PTD | EN | M0),	(OFF_DIS) },
	{ "sys_boot4",	"SYS_BOOT4",	0x1AA, (IEN | PTD | EN | M0),	(OFF_DIS) },
	{ "sys_boot5",	"SYS_BOOT5",	0x1AC, (IEN | PTD | EN | M0),	(OFF_DIS) },

	/* NOTE: Do not set OFF mode setting for 2MIC_RST and 2MIC_POWERDOWN.
	 * Specific sequence is required to put the 2MIC in sleep mode. It is
	 * handled by driver. */
	{ "dpm_emu0",	"2MIC_RST",		0x1AE, (M3),	(OFF_DIS) },
	{ "dpm_emu1",	"2MIC_POWERDOWN",	0x1B0, (M3),	(OFF_DIS) },

	{ "dpm_emu2",	"OLED_ID",	0x1B2, (PTD | EN | M5),		(OFF_OUT_PD) },
	{ "dpm_emu3",	"LCD_D(10)",	0x1B4, (PTD | EN | M5),		(OFF_OUT_PD) },
	{ "dpm_emu4",	"LCD_D(9)",	0x1B6, (PTD | EN | M5),		(OFF_OUT_PD) },
	{ "dpm_emu5",	"LCD_D(16)",	0x1B8, (PTD | EN | M5),		(OFF_OUT_PD) },
	{ "dpm_emu6",	"LCD_D(17)",	0x1BA, (PTD | EN | M5),		(OFF_OUT_PD) },
	{ "dpm_emu7",	"LCD_HSYNC",	0x1BC, (PTD | EN | M5),		(OFF_OUT_PD) },
	{ "dpm_emu8",	"LCD_PCLK",	0x1BE, (PTD | EN | M5),		(OFF_OUT_PD) },
	{ "dpm_emu9",	"LCD_VSYNC",	0x1C0, (PTD | EN | M5),		(OFF_OUT_PD) },
	{ "dpm_emu10",	"LCD_DE",	0x1C2, (PTD | EN | M5),		(OFF_OUT_PD) },
	{ "dpm_emu11",	"LCD_D(8)",	0x1C4, (PTD | EN | M5),		(OFF_OUT_PD) },
	{ "dpm_emu12",	"LCD_D(7)",	0x1C6, (PTD | EN | M5),		(OFF_OUT_PD) },
	{ "dpm_emu13",	"LCD_D(6)",	0x1C8, (PTD | EN | M5),		(OFF_OUT_PD) },
	{ "dpm_emu14",	"LCD_D(5)",	0x1CA, (PTD | EN | M5),		(OFF_OUT_PD) },
	{ "dpm_emu15",	"LCD_D(4)",	0x1CC, (PTD | EN | M5),		(OFF_OUT_PD) },
	{ "dpm_emu16",	"LCD_D(3)",	0x1CE, (PTD | EN | M5),		(OFF_OUT_PD) },
	{ "dpm_emu17",	"LCD_D(2)",	0x1D0, (PTD | EN | M5),		(OFF_OUT_PD) },
	{ "dpm_emu18",	"LCD_D(1)",	0x1D2, (PTD | EN | M5),		(OFF_OUT_PD) },
	{ "dpm_emu19",	"LCD_D(0)",	0x1D4, (PTD | EN | M5),		(OFF_OUT_PD) },
#endif

	/* NOTE: configuring the pins for I2C-using GPIOs Operations required for MHL*/
	 #if ( (CONFIG_SAMSUNG_OMAP4_TAB_REV == 6) \
     || (CONFIG_SAMSUNG_OMAP4_TAB_REV == 7))

  	{ "usbc1_icusb_dm",     "MHL_SCL_1.8V", 0x0E0,  (PTD | EN | M3),        (OFF_OUT_PD) },
    	/* [SHANKAR] Offset 0x0E4 is not right offset for this PIN, it should be 0x0148 */
	/* { "usbc1_icusb_dp",     "MHL_SDA_1.8V", 0x0E4,  (PTD | EN | M3),        (OFF_OUT_PD) }, */
	#endif
#endif
};

void set_offmode_padconfig(void)
{
	int i, size;
	u32 reg_val32;
	struct omap44xx_pin_config *pmux;

	size = ARRAY_SIZE(pin_mux);
	for (i = 0; i < size; i++) {
		pmux = &pin_mux[i];

		omap_writew((pmux->off_mode | pmux->func_mode),
				(OMAP4_CTRL_MODULE_PAD_CORE_MUX_PBASE + pmux->offset));

		/*
		printk("%s, %s, 0x%08X, 0x%04X\n",
				pmux->omap_pad_name, pmux->signal_name,
				(pmux->offset + OMAP4_CTRL_MODULE_PAD_CORE_MUX_PBASE),
				(pmux->func_mode | pmux->off_mode));
		*/
	}

	/* FREF_CLK_REQ */
	omap_writel(0x80000, 0x4A31E050);
	/* sys_pwr_req */ omap_writew(0x0000, 0x4A31E064);

	/* sys_pwron_reset_out */ omap_writew(0x010B,0x4A31E066);
	/* jtag_ntrst and jtag_tck */ omap_writel(0x01080108,0x4A31E06C);
	/* jtag_tms and jtag_rtck */ omap_writel(0x01000008,0x4A31E070);
	/* jtag_tdi and jtag_tdo */ omap_writel(0x01180118, 0x4A31E074);

	/* SYSCTRL_PADCONF_WKUP Registers */
	/* sim_io */ omap_writew((PTD | EN | M7), 0x4A31E040);
	/* sim_clk */ omap_writew((M3), 0x4A31E042);
	/* sim_cd */ omap_writew((WKUP_EN | IEN | PTU | EN | M3), 0x4A31E046);
	/* sim_pwrctrl */ omap_writew((PTD | EN | M7), 0x4A31E048);
	/* fref_clk3_req */ omap_writew((PTD | EN | M3), 0x4A31E056);
	/* fref_clk4_req */ omap_writew((PTD | EN | M3), 0x4A31E05A);
	/* fref_clk4_out */ omap_writew((PTD | EN | M3), 0x4A31E05C);
	/* AUXCLK3_SCRM, clock is disabled by software */
	reg_val32 = omap_readl(0x4A30A31C);
	reg_val32 &= ~(0x1<<8);
	omap_writel(reg_val32, 0x4A30A31C);

	/* LPDDR_VREF set to zero */
	reg_val32 = omap_readl(0x4A100644);
	reg_val32 &= 0xFFFFFFF3;
	omap_writel(reg_val32, 0x4A100644);

	/* LPDDR_VREF set to zero */
	reg_val32 = omap_readl(0x4A100654);
	reg_val32 &= 0xFFFFFFF3;
	omap_writel(reg_val32, 0x4A100654);
}



