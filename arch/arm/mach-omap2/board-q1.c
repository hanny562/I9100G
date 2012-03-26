/*
 * Board support file for Samsung OMAP4 10.1' tablelet Board.
 *
 * Copyright (C) 2010 Samsung Electronics
 *
 * Author: Shankar Bandal <shankar.b@samsung.com>
 *
 * Based on mach-omap2/board-4430sdp.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mhl-sii9234.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/usb/otg.h>
#include <linux/bootmem.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/i2c/twl.h>
#include <linux/i2c/cma3000.h>
#include <linux/regulator/machine.h>
#include <linux/input/sfh7741.h>
#include <linux/leds.h>
#include <linux/leds_pwm.h>
#include <linux/leds-omap4430sdp-display.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/twl6040-vib.h>
#include <linux/pwm_backlight.h>
#include <linux/leds_pwm.h>
#include <linux/power_supply.h>

#include <mach/hardware.h>
#include <mach/omap4-common.h>
#include <mach/emif.h>
#include <mach/lpddr2-sec.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/control.h>
#include <plat/timer-gp.h>
#include <plat/display.h>
#include <plat/usb.h>
#include <plat/omap_device.h>
#include <plat/omap_hwmod.h>
#ifdef CONFIG_SERIAL_OMAP
#include <plat/omap-serial.h>
#include <plat/serial.h>
#endif
#include <linux/wakelock.h>
#include <plat/mmc.h>
#include <plat/syntm12xx.h>
#include <plat/omap4-keypad.h>
#include <plat/hwspinlock.h>
#include <plat/nokia-dsi-panel.h>
#include <plat/samsung-dsi-panel.h>
#include "hsmmc.h"
#include "smartreflex-class3.h"
#include <linux/haptic.h>
#include <plat/opp_twl_tps.h>
#include <linux/fsa9480.h>
#include <linux/gp2a.h>
#include <linux/cm3663.h>
#include <linux/mpu.h>
#include <linux/yas.h>
#include <linux/switch.h>

#include <linux/i2c/twl6030-gpadc.h>
#include "board-4430sdp-wifi.h"

#include <linux/lcd.h>
#include <linux/ld9040.h>

#include <linux/i2c/mxt540e_q1.h>
#include "mux.h"
#ifdef CONFIG_PN544_NFC
#include<linux/pn544.h>
#endif
#if defined( CONFIG_SAMSUNG_PHONE_SVNET )
#include <linux/phone_svn/modemctl.h>
#include <linux/phone_svn/ipc_hsi.h>
#include <linux/irq.h>
#endif // CONFIG_SAMSUNG_PHONE_SVNET

/* TODO: OMAP-Samsung Board-Porting Layer */
#include <mach/board-q1.h>
//#include <mach/sec_log_buf.h>
#include <mach/sec_param.h>
#include <mach/sec_common.h>
#include <mach/sec_mux.h>

#ifdef CONFIG_EPEN_WACOM_G5SP
#include <linux/wacom_i2c.h>
static struct wacom_g5_callbacks *wacom_callbacks;
#endif /* CONFIG_EPEN_WACOM_G5SP */

#ifdef CONFIG_OMAP_MUX
extern struct omap_board_mux *sec_board_mux_ptr;
extern struct omap_board_mux *sec_board_wk_mux_ptr;
#else
#define sec_board_mux_ptr		NULL
#define sec_board_wk_mux_ptr		NULL
#endif

extern struct class *sec_class;
extern u32 sec_bootmode;

struct device *gps_dev;
EXPORT_SYMBOL(gps_dev);

int panel_id;
EXPORT_SYMBOL(panel_id);

static __init int setup_current_panel(char *opt)
{
	panel_id = (u32)memparse(opt, &opt);
	return 0;
}
__setup("lcd_panel_id=", setup_current_panel);

#define _OMAP_MUX_SETTING

#if (CONFIG_SAMSUNG_REL_HW_REV > 0)
#define DISABLE_I2C2_HW_SPINLOCK
#else
#undef DISABLE_I2C2_HW_SPINLOCK
#endif /* end of (CONFIG_SAMSUNG_REL_HW_REV == 0) */

#define CONTROL_CORE_PAD0_I2C3_SDA_PAD1_I2C4_SCL	0x4A10012C
#define CONTROL_CORE_PAD0_I2C4_SDA_PAD1_MCSPI1_CLK	0x4A100130

static u8 get_hw_rev_gpio(void);
static void omap4_usb_cb(u8 attached);
static void omap4_charger_cb(u8 attached);
static void omap4_jig_cb(u8 attached);
static void omap4_fsa9480_reset_cb(void);
#ifdef CONFIG_VIDEO_MHL_V1
static void omap4_mhl_cb(u8 attached);
extern void sii9234_hw_reset(void);
#endif

static struct charging_status_callbacks {
	void	(*tsp_set_charging_cable) (int type);
} charging_cbs;

static bool is_cable_attached;

static struct wake_lock uart_lock;
#define TWL6030_RTC_GPIO			6
#define BLUETOOTH_UART				UART2

static unsigned lcd_en_gpio;
static unsigned mlcd_rst_gpio;

//extern struct ld9040_panel_data matchbox_panel_data;
extern bool mhl_audio_path_info;
static int lcd_enabled;

void omap4430univ_sensors_init(void);

#ifdef CONFIG_VIDEO_MHL_V1
/* MHL  */
static struct i2c_gpio_platform_data q1_omap4_gpio_i2c5_pdata = {
    .sda_pin = OMAP_GPIO_MHL_SDA_18V,
    .scl_pin = OMAP_GPIO_MHL_SCL_18V,
    .udelay = 3,
    .timeout = 0,
};

static struct platform_device q1_omap4_gpio_i2c5_device = {
    .name = "i2c-gpio",
    .id = 5,
    .dev = {
	        .platform_data = &q1_omap4_gpio_i2c5_pdata,
	    }
};

struct mhl_platform_data mhl_pdata = {
    .mhl_sel    = OMAP_GPIO_MHL_SEL,
    .mhl_rst    = OMAP_GPIO_MHL_RST,
    .mhl_int    = OMAP_GPIO_MHL_INT,
    .mhl_wake_up    = OMAP_GPIO_MHL_WAKEUP,
    //.power_onoff    = sii9234_power_onoff,
#ifdef CONFIG_MHL_SWITCH
    .switch_onoff   = sii9234_switch_onoff,
#endif
};

struct platform_device q1_omap4_mhl_device = {
    .name   = "mhl",
    .id = -1,
    .dev    ={
          .platform_data = &mhl_pdata,
        }
};

#endif
static struct platform_device q1_omap4_hdmi_audio_device = {
	.name = "hdmi-dai",
	.id = -1,
};

#ifdef CONFIG_SWITCH_GPIO
void ear_mic_bias_cb(bool value)
{
	printk(KERN_ERR "ear_mic_bias_cb %d\n", value);
	if (value)
		gpio_set_value(OMAP_GPIO_EAR_MICBIAS_EN, 1);
	else
		gpio_set_value(OMAP_GPIO_EAR_MICBIAS_EN, 0);
}

static struct gpio_switch_platform_data headset_switch_data = {
	.name = "h2w",
	.gpio = OMAP_GPIO_DET_35,	/* Omap3430 GPIO_27 For samsung zeus */
	.ear_mic_bias_cb = ear_mic_bias_cb
};

static struct platform_device headset_switch_device = {
	.name = "switch-gpio",
	.dev = {
		.platform_data = &headset_switch_data,
		}
};
#endif
#ifdef CONFIG_INPUT_EAR_KEY
static struct resource board_ear_key_resource = {
	.end = 0,
	.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL,
};

static struct gpio_switch_platform_data ear_key_switch_data = {
	.name = "send_end",
	.gpio = OMAP_GPIO_EAR_SEND_END,	/* Omap3430 GPIO_27 For samsung zeus */
};

static struct platform_device board_ear_key_device = {
	.name = "sec_jack",
	.id = -1,
	.num_resources = 1,
	.resource = &board_ear_key_resource,
	.dev = {
		.platform_data = &ear_key_switch_data,
		}
};
#endif
#ifdef CONFIG_EAR_MIC_ADC
static struct resource q1_omap4_ear_mic_adc_resource = {
	.start = 0,
	.end = 0,
	.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL,
};

static struct platform_device q1_omap4_ear_mic_adc_device = {
	.name = "ear_mic_adc",
	.id = -1,
	.num_resources = 1,
	.resource = &q1_omap4_ear_mic_adc_resource,
};
#endif

#ifdef CONFIG_EXTRA_DOCK_SPEAKER
static struct switch_dev switch_dock = {
	.name = "dock",
};

static void omap4_deskdock_cb(bool attached)
{
	if (attached)
		switch_set_state(&switch_dock, 1);
	else
		switch_set_state(&switch_dock, 0);
}

static void omap4_cardock_cb(bool attached)
{
	if (attached)
		switch_set_state(&switch_dock, 2);
	else
		switch_set_state(&switch_dock, 0);
}
#endif


#ifdef CONFIG_SWITCH_SIO
static void omap4_switch_gpio_init(void);
static void omap4_cp_usb_on(u8 value);
static void omap4_uart_gpio_sel(u8 value);


static struct switch_sio_platform_data switch_sio_data = {
	.name = "usb_uart_sel",
	.gpio_init = omap4_switch_gpio_init,
	.cp_usb_cb = omap4_cp_usb_on,
	.uart_sel_cb = omap4_uart_gpio_sel,
};

/* SIO Switch */
struct platform_device sec_sio_switch = {
	.name = "switch-sio",
	.id = -1,
	.dev = {
		.platform_data = &switch_sio_data,
		}
};
#endif /*config_switch_sio*/

#ifndef CONFIG_INPUT_GPIO_VOLUME_KEY
static int q1_omap4_keymap[] = {
	/* TODO: The row and column coded here are based on what is detected by
	 * OMAP4. It is different from the one shown in schematic. Need the
	 * identify the root cause of the problem */
	/* row, col, key */
	KEY(2, 1, KEY_VOLUMEUP),	//Volume up
	KEY(1, 1, KEY_VOLUMEDOWN),	//volume down
	0,
};
#endif

/* For uUSB Switch */
static struct fsa9480_platform_data omap4_fsa9480_pdata = {
	.intb_gpio = OMAP_GPIO_JACK_NINT,
	.usb_cb = omap4_usb_cb,
	.uart_cb = NULL,
	.charger_cb = omap4_charger_cb,
	.jig_cb = omap4_jig_cb,
	.reset_cb = omap4_fsa9480_reset_cb,
	#ifdef CONFIG_VIDEO_MHL_V1
    	.mhl_cb	    = omap4_mhl_cb,
   	#endif
	#ifdef CONFIG_EXTRA_DOCK_SPEAKER
	.cardock_cb = omap4_cardock_cb,
	.deskdock_cb = omap4_deskdock_cb,
	#endif
};

#ifdef CONFIG_MPU_SENSORS_MPU3050

#define SENSOR_MPU_NAME				"mpu3050"

static struct mpu3050_platform_data mpu_data = {
	.int_config = 0x12,
	.orientation = {1, 0, 0,
			0, 1, 0,
			0, 0, 1},
	/* accel */
	.accel = {
		  .get_slave_descr = bma250_get_slave_descr,
		  .adapt_num = 4,
		  .bus = EXT_SLAVE_BUS_SECONDARY,
		  .address = 0x18,
		  .orientation = {0, -1, 0,
				  1, 0, 0,
				  0, 0, 1},
		  },
	/* compass */
	.compass = {
		    .get_slave_descr = yas530_get_slave_descr,
		    .adapt_num = 4,
		    .bus = EXT_SLAVE_BUS_PRIMARY,
		    .address = 0x2E,
		    .orientation = {1, 0, 0,
				    0, 1, 0,
				    0, 0, 1},
		    },
};

signed char mpu_orientation_rev01[9] = {-1,0,0,
						0,-1,0,
						0,0,1};

void mpu_layout_set (void)
{
	int i;
	if (get_hw_rev_gpio()>=3) {
		for(i=0;i<9;i++) {
			mpu_data.orientation[i] = mpu_orientation_rev01[i];
		}
		printk("\n gyro sensor layout set for hw rev01 02 \n");
	} else {
			printk("\n gyro sensor no layout \n");
	}
}
#endif

struct yas_platform_data yas_data = {
	.hw_rev=0,	/* hw gpio value */
};


void yas_layout_set (void)
{
	int ver = 0;
	ver=get_hw_rev_gpio();
	if (ver >= 3) {
		
		yas_data.hw_rev = ver;
		printk("\n yas sensor layout set for hw rev01 02 \n");
	} else {
			printk("\n yas sensor layout set for others \n");
	}
}

#ifdef CONFIG_OPTICAL_GP2A
/* For gp2a light/proximity sensor */
static int gp2a_gpio_init(void)
{
	int ret = -ENODEV;
	gpio_free(OMAP_GPIO_PS_ON);
	ret = gpio_request(OMAP_GPIO_PS_ON, "gpio_ps_on");
	if (ret < 0) {
		pr_err("%s: gpio %d request failed (%d)\n",
		       __func__, OMAP_GPIO_PS_ON, ret);
		return ret;
	}
	ret = gpio_direction_input(OMAP_GPIO_PS_ON);
	if (ret < 0) {
		pr_err("%s: failed to set gpio %d as input (%d)\n",
		__func__, OMAP_GPIO_PS_ON, ret);
		return ret;
	}

	return 0;
}

static int gp2a_power(bool on)
{
	/* this controls the power supply rail to the gp2a IC */
	gpio_direction_output(OMAP_GPIO_PS_ON, on);
	return 0;
}

static struct twl6030_gpadc_request conv_request = {
	.channels = (0x1 << 4),
	.do_avg = 0,
	.method = TWL6030_GPADC_SW2,
	.type = 0,
	.active = 0,
	.result_pending = 0,
	.func_cb = NULL,
};

static int gp2a_light_adc_value(void)
{
	twl6030_gpadc_conversion(&conv_request);
	return (conv_request.rbuf[4]);

}

static struct gp2a_platform_data gp2a_pdata = {
	.power = gp2a_power,
	.p_out = OMAP_GPIO_PS_VOUT,
	.light_adc_value = gp2a_light_adc_value
};
#endif

#ifdef CONFIG_INPUT_CM3663
/* For gp2a light/proximity sensor */

static int cm3663_power(bool on)
{
	/* this controls the power supply rail to the gp2a IC */
	gpio_direction_output(OMAP_GPIO_PS_ON, on);
	return 0;
}

static struct cm3663_platform_data cm3663_pdata = {
	.proximity_power = cm3663_power,
	.irq = OMAP_GPIO_PS_VOUT,
};

void cm3663_gpio_request(void)
{
	int ret = -ENODEV;
	gpio_free(OMAP_GPIO_PS_ON);
	ret = gpio_request(OMAP_GPIO_PS_ON, "gpio_ps_on");
	if (ret < 0) {
		pr_err("%s: gpio %d request failed (%d)\n",
		       __func__, OMAP_GPIO_PS_ON, ret);
		return ret;
	}
}
#endif

#define PRM_VC_CFG_I2C_MODE		0x4A307BA8
#define CONTROL_I2C_0				0x4A100604

void i2c4_pullup_set(void)
{
	u32 reg_val;

	if (get_hw_rev_gpio()>=1) {
		omap_writel(0x54425442, CONTROL_I2C_0);
		printk("\n @@@@@ I2C4 diable Internal Pull up \n");
	} else {
		//omap_writel(0x44424442, CONTROL_I2C_0);
		omap_writel(0x24422442, CONTROL_I2C_0);
		printk("\n @@@@@ I2C4 Enable Internal Pull up 2.2kohm \n");
	}
	reg_val = omap_readl(CONTROL_I2C_0);
	printk("\n @@@@@ I2C4  0x%x @@@@@ \n", reg_val);	
}

void i2c4_config_read(void)
{
	
	u32 reg_val;
	reg_val = omap_readl(PRM_VC_CFG_I2C_MODE);
	printk("\n @@@@@ HSMODE  0x%x @@@@@ \n", reg_val);

	reg_val = omap_readl(CONTROL_I2C_0);
	printk("\n @@@@@ I2C4  0x%x @@@@@ \n", reg_val);

	
}


void acc_en_gpio_request(void)
{
	int ret = -ENODEV;
	int ver = 0;
	ver=get_hw_rev_gpio();

	if(ver >= 4) {
		gpio_free(OMAP_GPIO_ACC_EN);
		ret = gpio_request(OMAP_GPIO_ACC_EN, "gpio_acc_en");
		if (ret < 0) {
			pr_err("%s: gpio %d request failed (%d)\n",
			       __func__, OMAP_GPIO_ACC_EN, ret);
			return ret;
		}
		
		gpio_direction_output(OMAP_GPIO_ACC_EN,0);
		mdelay(70);
		printk("\n @@@@@ ACC_EN RESET @@@@@ \n");
		gpio_direction_output(OMAP_GPIO_ACC_EN,1);
	}
}

static void omap4_usb_cb(u8 attached)
{
	/* TODO: need to implement this once uUSB charging available in HW */
	printk("\nBoard file [FSA9480]: USB Callback \n");
	if(charging_cbs.tsp_set_charging_cable)
		charging_cbs.tsp_set_charging_cable(attached);
	is_cable_attached = attached;
}

static void omap4_charger_cb(u8 attached)
{
	/* TODO: need to implement this once uUSB charging available in HW */
	printk("\nBoard file [FSA9480]: Charger Callback \n");
	if(charging_cbs.tsp_set_charging_cable)
		charging_cbs.tsp_set_charging_cable(attached);
	is_cable_attached = attached;
}

static void omap4_jig_cb(u8 attached)
{
	/* TODO: need to implement this once uUSB charging available in HW */
	printk("\nBoard file [FSA9480]: Jig Callback \n");
}

static void omap4_fsa9480_reset_cb(void)
{
	/* TODO: need to implement this once uUSB charging available in HW */
	printk("\nBoard file [FSA9480]: Reset Callback \n");
}

#ifdef CONFIG_SWITCH_SIO
static void omap4_switch_gpio_init(void)
{

   if (gpio_request(OMAP_GPIO_UART_SEL, "UART_SEL"))
      printk(KERN_ERR "Failed to request OMAP_GPIO_UART_SEL!\n");

   if (gpio_request(OMAP_GPIO_CP_USB_ON, "USB_SEL"))
      printk(KERN_ERR "Failed to request OMAP_GPIO_CP_VBUS_EN!\n");
 
}


static void omap4_cp_usb_on(u8 value)
{   

   if(value == USB_SW_AP)
   	gpio_set_value(OMAP_GPIO_CP_USB_ON, 1);
	
   else if(value == USB_SW_CP)
   	gpio_set_value(OMAP_GPIO_CP_USB_ON, 0);

   else
   	printk("[SWITCH_SIO] %s: WRONG VALUE\n",__func__);
   	
}

static void omap4_uart_gpio_sel(u8 value)
{

   if(value == UART_SW_AP)
   	gpio_set_value(OMAP_GPIO_UART_SEL, 1);
   
   else if(value == UART_SW_CP)
   	gpio_set_value(OMAP_GPIO_UART_SEL, 0);
   
   else
   	printk("[SWITCH_SIO] %s: WRONG VALUE\n",__func__);
}

#endif /*config_switch_sio*/

#ifdef CONFIG_VIDEO_MHL_V1
extern void MHL_On(bool on);

static bool fsa9480_is_mhl_attached(void)
{
	int val;

	gpio_request(OMAP_GPIO_MHL_SEL, "MHL_SEL");
	val = gpio_get_value(OMAP_GPIO_MHL_SEL);
	gpio_free(OMAP_GPIO_MHL_SEL);

	return !!val;
}
extern int phy_control_interrupt(bool on);
static void omap4_mhl_cb(u8 attached)
{

        int ret=0;
            /*TODO */
       printk("\nBoard file [FSA9480]: MHL Callback == %d\n", attached);
        if( attached == 1)
        {
#if 0
                //Clearing the Connect and Disconnect Interrupts of HDMI_WP_IRQ_ENABLE_CLR
                u32 temp = omap_readl(0x48046030);
                printk(KERN_ERR"WP_IRQ_ENABLE_CLR..currently reads as:%x",temp);
                temp = temp | (1<<25)|(1<<26);
                omap_writel(temp,0x48046030);
                printk(KERN_ERR"WP_IRQ_ENABLE_CLR..changed to :%x",temp);
                //omap_writel(0x000F0118,0x4A100098);
                // Look at RSEN interrupt
#endif

                ret = phy_control_interrupt(1);
                if (ret == -1)
                {
                         printk("\nJUST return\n");
                         return;
                }
                else{
                   MHL_On(1);
                }
         } else {
                if (!mhl_audio_path_info){
                                gpio_set_value(OMAP_GPIO_MHL_SEL,0);
                        MHL_On(0);
                }
                else{ //audio dock path
                        sii9234_hw_reset();
                        msleep(5);
                        MHL_On(0);

                        }
         }

         //gpio_free(OMAP_GPIO_MHL_SEL
}
#endif

#ifndef CONFIG_INPUT_GPIO_VOLUME_KEY
static struct matrix_keymap_data q1_omap4_keymap_data = {
	.keymap = q1_omap4_keymap,
	.keymap_size = ARRAY_SIZE(q1_omap4_keymap),
};

static struct omap4_keypad_platform_data q1_omap4_keypad_data = {
	.keymap_data = &q1_omap4_keymap_data,
	.rows = 8,
	.cols = 8,
	.rep = 0,
};
#endif

static int q1_omap4_panel_enable_hdmi(struct omap_dss_device *dssdev)
{

	return 0;
}

static void q1_omap4_panel_disable_hdmi(struct omap_dss_device *dssdev)
{
}

static __attribute__ ((unused))
void __init q1_omap4_hdmi_init(void)
{
	return;
}


static int s6e8aa0_panel_enable_lcd(struct omap_dss_device *dssdev)
{
	struct regulator *regulator;
	printk(KERN_INFO "*** %s [%d]  \n", __func__, __LINE__);
	
	regulator = regulator_get(&dssdev->dev, "vaux3");
	if (IS_ERR(regulator)) {
		printk("*** %s [%d] failed to get VAUX3 regulator. \n",
		       __func__, __LINE__);
		return 0;
	}
	regulator_enable(regulator);
	regulator_put(regulator);
	return 0;
}

static void s6e8aa0_panel_disable_lcd(struct omap_dss_device *dssdev)
{
	struct regulator *regulator;
	printk(KERN_INFO "*** %s [%d]  \n", __func__, __LINE__);
	
	regulator = regulator_get(&dssdev->dev, "vaux3");
	if (IS_ERR(regulator)) {
		printk("*** %s [%d] failed to get VAUX3 regulator. \n",
		       __func__, __LINE__);
		return;
	}
	if (regulator_is_enabled(regulator))
		regulator_force_disable(regulator);
	regulator_put(regulator);

	return;

}

static struct samsung_dsi_panel_data dsi_panel = {
		.name	= "s6e8aa0_panel",
		.reset_gpio	= OMAP_GPIO_MLCD_RST,		// GPIO_35
		.use_ext_te	= false,
		.ext_te_gpio	= OMAP_GPIO_LCD_TE, // GPIO_19
		.use_esd_check	= false,
		.set_backlight	= NULL,
};

static struct omap_dss_device q1_omap4_lcd_device = {
	.name = "lcd",
	.driver_name = "s6e8aa0_panel",
	.type = OMAP_DISPLAY_TYPE_DSI,
	.data			= &dsi_panel,
	.phy.dsi		= {
		.clk_lane	= 1,
		.clk_pol	= 0,
		.data1_lane	= 2,
		.data1_pol	= 0,
		.data2_lane	= 3,
		.data2_pol	= 0,
		.data3_lane	= 4,
		.data3_pol	= 0,
		.data4_lane	= 5,
		.data4_pol	= 0,
		.div		= {
			.lck_div	= 1,	/* LCD */ 
			.pck_div	= 2,	/* PCD */ 
			.regm		= 246, 		/* DSI_PLL_REGM */ 
			.regn		= 19,  		/* DSI_PLL_REGN */ 
			.regm_dispc	= 6,	  	/* PLL_CLK1 (M4) */ 
			.regm_dsi 	= 6,		/* PLL_CLK2 (M5) */ 
			.lp_clk_div	= 14, 	  	/* LPDIV */ 
		},
		.xfer_mode = OMAP_DSI_XFER_VIDEO_MODE,
	},
	.platform_enable = s6e8aa0_panel_enable_lcd,
	.platform_disable = s6e8aa0_panel_disable_lcd,
	.channel = OMAP_DSS_CHANNEL_LCD,
};



static struct omap_dss_device q1_omap4_hdmi_device = {
	.name = "hdmi",
	.driver_name = "hdmi_panel",
	.type = OMAP_DISPLAY_TYPE_HDMI,
	.phy.dpi.data_lines = 24,
	.platform_enable = q1_omap4_panel_enable_hdmi,
	.platform_disable = q1_omap4_panel_disable_hdmi,
	.channel = OMAP_DSS_CHANNEL_DIGIT,
};

static struct omap_dss_device *q1_omap4_dss_devices[] = {
	&q1_omap4_lcd_device,
#ifdef CONFIG_OMAP2_DSS_HDMI
	&q1_omap4_hdmi_device,
#endif
};

static struct omap_dss_board_info q1_omap4_dss_data = {
	.num_devices = ARRAY_SIZE(q1_omap4_dss_devices),
	.devices = q1_omap4_dss_devices,
	.default_device = &q1_omap4_lcd_device,
};

/* wl128x BT, FM, GPS connectivity chip */
static int gpios[] = { 103, -1, -1 };

static struct platform_device wl128x_device = {
	.name = "kim",
	.id = -1,
	.dev.platform_data = &gpios,
};

static struct resource samsung_omap4_pwr_key_resources[] = {
	[0] = {
	       // PWRON KEY
	       .start = 0,
	       .end = 0,
	       .flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE | IORESOURCE_IRQ_LOWEDGE,
	       },
	[1] = {
	       // HOME KEY
	       .start = 0,
	       .end = 0,
	       .flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE | IORESOURCE_IRQ_LOWEDGE,
	       },
	[2] = {
	       // VOLUME UP
	       .start = 0,
	       .end = 0,
	       .flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE | IORESOURCE_IRQ_LOWEDGE,
	       },
	[3] = {
	       // VOLUME DOWN
	       .start = 0,
	       .end = 0,
	       .flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE | IORESOURCE_IRQ_LOWEDGE,
	       },
};

static struct platform_device samsung_omap4_pwr_device = {
	.name = "sec_power_key",
	.id = -1,
	.num_resources = ARRAY_SIZE(samsung_omap4_pwr_key_resources),
	.resource = &samsung_omap4_pwr_key_resources,
};

static inline void __init samsung_omap4_pwr_key_irq_init(void)
{
	samsung_omap4_pwr_key_resources[0].start = gpio_to_irq(OMAP_GPIO_KEY_PWRON);
	if (gpio_request(OMAP_GPIO_KEY_PWRON, "power_key_irq") < 0) {
		printk(KERN_ERR "\n FAILED TO REQUEST GPIO %d for POWER KEY IRQ \n", OMAP_GPIO_KEY_PWRON);
		return;
	}
	samsung_omap4_pwr_key_resources[1].start = gpio_to_irq(OMAP_GPIO_KEY_HOME);
	if (gpio_request(OMAP_GPIO_KEY_HOME, "home_key_irq") < 0) {
		printk(KERN_ERR "\n FAILED TO REQUEST GPIO %d for HOME KEY IRQ \n", OMAP_GPIO_KEY_HOME);
		return;
	}
#ifdef CONFIG_INPUT_GPIO_VOLUME_KEY
	samsung_omap4_pwr_key_resources[2].start = gpio_to_irq(OMAP_GPIO_KEY_VOL_UP);
	if (gpio_request(OMAP_GPIO_KEY_VOL_UP, "vol_up_key_irq") < 0) {
		printk(KERN_ERR "\n FAILED TO REQUEST GPIO %d for HOME KEY IRQ \n", OMAP_GPIO_KEY_VOL_UP);
		return;
	}
	samsung_omap4_pwr_key_resources[3].start = gpio_to_irq(OMAP_GPIO_KEY_VOL_DOWN);
	if (gpio_request(OMAP_GPIO_KEY_VOL_DOWN, "vol_down_key_irq") < 0) {
		printk(KERN_ERR "\n FAILED TO REQUEST GPIO %d for HOME KEY IRQ \n", OMAP_GPIO_KEY_VOL_DOWN);
		return;
	}
#endif
	gpio_direction_input(OMAP_GPIO_KEY_PWRON);
	gpio_direction_input(OMAP_GPIO_KEY_HOME);
#ifdef CONFIG_INPUT_GPIO_VOLUME_KEY
	gpio_direction_input(OMAP_GPIO_KEY_VOL_UP);
	gpio_direction_input(OMAP_GPIO_KEY_VOL_DOWN);
#endif
}

static struct resource samsung_omap4_gyro_lp530al_rsc[3] = {
	{
	 .start = 0,
	 .end = 0,
	 .flags = IORESOURCE_IRQ | IORESOURCE_IRQ_LOWLEVEL,
	 },
	{
	 .start = 1,
	 .end = 0,
	 .flags = IORESOURCE_IRQ | IORESOURCE_IRQ_LOWLEVEL,
	 },
	{
	 .start = 0x4A31E040,
	 .end = 0x4A31E044,
	 .flags = IORESOURCE_MEM,
	 },
};

static struct platform_device samsung_omap4_gyro_lp530al_device = {
	.name = "lp530al_gyro_sensor",
	.id = -1,
	.num_resources = 3,
	.resource = &samsung_omap4_gyro_lp530al_rsc[0],
};

//haptic device
static struct haptic_platform_data isa1000_platform_data[] = {
	[0] = {			/* Primary Controller */
	       .gpio = 42,
	       .name = "isa1000",
	       /*.ldo_level = ISA1200_LDOADJ_24V, */
	       .pwm_timer = 0,
	       },
	[1] = {			/* Secondary Controller */
	       .gpio = 43,
	       .name = "isa1000",
	       /*.ldo_level = ISA1200_LDOADJ_36V, */
	       .pwm_timer = 1,
	       },
};

static struct platform_device pwm_haptic_device = {
	.name = "samsung_pwm_haptic",
	.id = -1,
	.dev.platform_data = &isa1000_platform_data[0],
};

/* pwm enabled device */
static const struct platform_device_id pwm_id_table[] = {
	{"omap44xx-pwm", 0},
};

static struct platform_device pwmbacklight_device = {
	.name = "omap44xx-pwm",
	.id = 190,
	.dev.platform_data = &pwm_id_table,
};

/* lcd-backlight */
static const struct led_pwm ledpwm = {
	.name = "lcd-backlight",
	.pwm_id = 190,
	.active_low = 1,
	.max_brightness = 255,
	.pwm_period_ns = 85,
};

static const struct led_pwm_platform_data pwmblk_platformdata = {
	1,
	&ledpwm,
};

static struct platform_device pwm_bl_device = {
	.name = "leds_pwm",
	.id = -1,
	.dev.platform_data = &pwmblk_platformdata,
};

static struct resource samsung_charger_resources[] = {
	[0] = { // CHARGER_INTR
		.start = 0,
		.end   = 0,
		.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE | IORESOURCE_IRQ_LOWEDGE,
	},
	[1] = { // CHARGERFAULT_INTR
		.start = 0,
		.end   = 0,
		.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE | IORESOURCE_IRQ_LOWEDGE,
	},
	[2] = { // BAT_REMOVAL
		.start = 0,
		.end   = 0,
		.flags = IORESOURCE_IRQ,
	},
};

static int samsung_charger_config_data[] = {
	// [ CHECK VF USING ADC ]
	/*   1. ENABLE	(true, flase) */
	true,
	/*   2. ADCPORT (ADCPORT NUM) */
	0,
	// [ SUPPORT TA_NCHG IRQ FOR CHECKING FULL ]
	/*   1. ENABLE	(true, flase) */
	true,
	/* HW Rev info*/
	0,
};


struct battery_device_config
{
	/* SUPPORT MONITORING CHARGE CURRENT FOR CHECKING FULL */
	int MONITORING_CHG_CURRENT;
	int CHG_CURRENT_ADC_PORT;
	
	/* SUPPORT MONITORING TEMPERATURE OF THE SYSTEM FOR BLOCKING CHARGE */
	int MONITORING_SYSTEM_TEMP;
	int TEMP_ADC_PORT;

	/* Check Battery state for LPM */
	int boot_mode;
	unsigned int (*pm_poweroff) (void);
};

static struct battery_device_config samsung_battery_config_data = {
	/* [ SUPPORT MONITORING CHARGE CURRENT FOR CHECKING FULL ] */
	/* 1.ENABLE  (true, flase) */
	.MONITORING_CHG_CURRENT = true,
	/* 2. ADCPORT (ADCPORT NUM) */
	.CHG_CURRENT_ADC_PORT = 11,

	/* [ SUPPORT MONITORING TEMPERATURE OF THE SYSTEM FOR BLOCKING CHARGE ] */
	/* 1. ENABLE  (true, flase) */
	.MONITORING_SYSTEM_TEMP = true,
	/* 2. ADCPORT (ADCPORT NUM) */
	.TEMP_ADC_PORT = 1,

	/* [ Check Battery state for LPM ] */
	.boot_mode = 0,
	.pm_poweroff = NULL,
};

static struct platform_device samsung_charger_device = {
	.name = "secChargerDev",
	.id = -1,
	.num_resources = ARRAY_SIZE(samsung_charger_resources),
	.resource = samsung_charger_resources,
	/*
	.dev = {
		.platform_data = &samsung_charger_config_data,
		},*/
};

static struct platform_device samsung_battery_device = {
	.name = "secBattMonitor",
	.id = -1,
	.num_resources = 0,
	/*
	.dev = {
		.platform_data = &samsung_battery_config_data,
		},
	*/
};

static inline void sec_power_off()
{
	pm_power_off();
}

static inline void __init samsung_omap4_battery_init(void)
{
	/* CHARGER_INTR_OFFSET */
	samsung_charger_resources[0].start = TWL6030_IRQ_BASE + CHARGER_INTR_OFFSET;
	
	/* CHARGERFAULT_INTR */
	samsung_charger_resources[1].start = TWL6030_IRQ_BASE + CHARGERFAULT_INTR_OFFSET;

    /* BAT_REMOVAL */
	if (gpio_request(OMAP_GPIO_BAT_REMOVAL, "BAT REMOVAL irq") < 0) {
		printk(KERN_ERR "Failed to request GPIO%d for BAT_REMOVAL IRQ\n",
				OMAP_GPIO_BAT_REMOVAL);
		samsung_charger_resources[2].start = -1;
	} else {
		samsung_charger_resources[2].start = gpio_to_irq(OMAP_GPIO_BAT_REMOVAL);
		gpio_direction_input(OMAP_GPIO_BAT_REMOVAL);

	}
	samsung_charger_config_data[3] = get_hw_rev_gpio();
	samsung_charger_device.dev.platform_data = &samsung_charger_config_data;

	samsung_battery_config_data.boot_mode = sec_bootmode;
	samsung_battery_config_data.pm_poweroff = sec_power_off;
	samsung_battery_device.dev.platform_data = &samsung_battery_config_data;
}


#if defined( CONFIG_SAMSUNG_PHONE_SVNET )

static void ipc_hsi_cfg_gpio(void);

static struct ipc_hsi_platform_data ipc_hsi_data = {
	.cfg_gpio = ipc_hsi_cfg_gpio,
};

static struct resource ipc_hsi_res[] = {
//	[0] = {
//	       .start = OMAP_GPIO_IRQ(OMAP_GPIO_MIPI_HSI_SUSPEND_REQUEST),
//	       .end = OMAP_GPIO_IRQ(OMAP_GPIO_MIPI_HSI_SUSPEND_REQUEST),
//	       .flags = IORESOURCE_IRQ,
//	       },
};

#if defined( CONFIG_MIPI_HSI_LOOPBACK_TEST )
static struct platform_device mipi_hsi_test = {
	.name = "onedram",
	.id = -1,
	.num_resources = ARRAY_SIZE(mipi_hsi_res),
	.resource = mipi_hsi_res,
	.dev = {
		.platform_data = &mipi_hsi_data,
		},
};
#else
static struct platform_device ipc_hsi = {
	.name = "onedram",
	.id = -1,
	.num_resources = ARRAY_SIZE(ipc_hsi_res),
	.resource = ipc_hsi_res,
	.dev = {
		.platform_data = &ipc_hsi_data,
		},
};
#endif

static void ipc_hsi_cfg_gpio(void)
{
}

static void modemctl_cfg_gpio(void);

static struct modemctl_platform_data mdmctl_data = {
	.name = "xmm",

	.gpio_phone_on = OMAP_GPIO_MIPI_HSI_CP_ON,
	.gpio_phone_active = OMAP_GPIO_MIPI_HSI_PHONE_ACTIVE,
	.gpio_pda_active = OMAP_GPIO_MIPI_HSI_PDA_ACTIVE,
	.gpio_cp_reset = OMAP_GPIO_MIPI_HSI_CP_RST,
	.gpio_reset_req_n = OMAP_GPIO_MIPI_HSI_RESET_REQ_N,
	.gpio_cp_dump_int = OMAP_GPIO_MIPI_HSI_CP_DUMP_INT,

	//.gpio_con_cp_sel = OMAP_GPIO_CON_CP_SEL,
	//.gpio_phone_on = GPIO_PHONE_ON,
	//.gpio_usim_boot = GPIO_USIM_BOOT,
	//.gpio_sim_ndetect = GPIO_SIM_nDETECT,

	.cfg_gpio = modemctl_cfg_gpio,
};

static struct resource mdmctl_res[] = {
	[0] = {
	       .start = OMAP_GPIO_IRQ(OMAP_GPIO_MIPI_HSI_PHONE_ACTIVE),	// phone active irq
	       .end = OMAP_GPIO_IRQ(OMAP_GPIO_MIPI_HSI_PHONE_ACTIVE),
	       .flags = IORESOURCE_IRQ,
	       },
};

static struct platform_device modemctl = {
	.name = "modemctl",
	.id = -1,
	.num_resources = ARRAY_SIZE(mdmctl_res),
	.resource = mdmctl_res,
	.dev = {
		.platform_data = &mdmctl_data,
		},
};

static void modemctl_cfg_gpio(void)
{
	int err = 0;

	unsigned gpio_phone_on = mdmctl_data.gpio_phone_on;
	unsigned gpio_cp_rst = mdmctl_data.gpio_cp_reset;
	unsigned gpio_pda_active = mdmctl_data.gpio_pda_active;
	unsigned gpio_phone_active = mdmctl_data.gpio_phone_active;
	unsigned gpio_reset_req_n = mdmctl_data.gpio_reset_req_n;
	unsigned gpio_cp_dump_int = mdmctl_data.gpio_cp_dump_int;

	err = gpio_request(gpio_reset_req_n, "RESET_REQ_N");
	if (err) {
		printk("modemctl_cfg_gpio - fail to request gpio %s : %d\n",
		       "RESET_REQ_N", err);
	} //else {
		gpio_direction_output(gpio_reset_req_n, 0);
	//}

	err = gpio_request(gpio_phone_on, "CP_ON");
	if (err) {
		printk("modemctl_cfg_gpio - fail to request gpio %s : %d\n",
		       "CP_ON", err);
	} //else {
		gpio_direction_output(gpio_phone_on, 0);
	//}

	err = gpio_request(gpio_cp_rst, "CP_RST");
	if (err) {
		printk("modemctl_cfg_gpio - fail to request gpio %s : %d\n",
		       "CP_RST", err);
	} //else {
		gpio_direction_output(gpio_cp_rst, 0);
	//}

	err = gpio_request(gpio_pda_active, "PDA_ACTIVE");
	if (err) {
		printk("modemctl_cfg_gpio - fail to request gpio %s : %d\n",
		       "PDA_ACTIVE", err);
	} //else {
		gpio_direction_output(gpio_pda_active, 0);
	//}

	err = gpio_request(gpio_phone_active, "PHONE_ACTIVE");
	if (err) {
		printk("modemctl_cfg_gpio - fail to request gpio %s : %d\n",
		       "PHONE_ACTIVE", err);
	} //else {
		gpio_direction_input(gpio_phone_active);
	//}

	err = gpio_request(gpio_cp_dump_int, "CP_DUMP_INT");
	if (err) {
		printk("modemctl_cfg_gpio - fail to request gpio %s : %d\n",
		       "CP_DUMP_INT", err);
	} //else {
		gpio_direction_input(gpio_cp_dump_int);
	//}

	set_irq_type(OMAP_GPIO_IRQ(OMAP_GPIO_MIPI_HSI_PHONE_ACTIVE),
		     IRQ_TYPE_LEVEL_HIGH);
	//set_irq_type( gpio_sim_ndetect, IRQ_TYPE_EDGE_BOTH );
}
#endif // CONFIG_SAMSUNG_PHONE_SVNET

static struct platform_device sec_device_btrfkill = {
	.name = "bt_rfkill",
	.id = -1,
};

static struct platform_device *q1_omap4_devices[] __initdata = {
	&wl128x_device,
	&q1_omap4_hdmi_audio_device,
	&q1_omap4_gpio_i2c5_device,
	&q1_omap4_mhl_device,
	&samsung_omap4_pwr_device,
	&samsung_omap4_gyro_lp530al_device,
	&pwmbacklight_device,
	&pwm_bl_device,
	&samsung_battery_device,
	&samsung_charger_device,
	&pwm_haptic_device,
	&sec_device_btrfkill,
#ifdef CONFIG_EAR_MIC_ADC
	&q1_omap4_ear_mic_adc_device,
#endif
#ifdef CONFIG_SWITCH_GPIO
	&headset_switch_device,
#endif
#ifdef CONFIG_INPUT_EAR_KEY
	&board_ear_key_device,
#endif
#if defined( CONFIG_SAMSUNG_PHONE_SVNET )
#if defined( CONFIG_MIPI_HSI_LOOPBACK_TEST )
	&mipi_hsi_test,
#else
	&ipc_hsi,
#endif
	&modemctl,
#endif // CONFIG_SAMSUNG_PHONE_SVNET
#ifdef CONFIG_SWITCH_SIO
    &sec_sio_switch,
#endif
};

static void __init omap_board_init_irq(void)
{
	omap2_init_common_hw(NULL, NULL);
#ifdef CONFIG_OMAP_32K_TIMER
	omap2_gp_clockevent_set_gptimer(1);
#endif
	gic_init_irq();
	sr_class3_init();
}

static struct omap_musb_board_data musb_board_data = {
	.interface_type = MUSB_INTERFACE_UTMI,
#ifdef CONFIG_USB_MUSB_OTG
	.mode = MUSB_OTG,
#elif defined(CONFIG_USB_MUSB_HDRC_HCD)
	.mode = MUSB_HOST,
#elif defined(CONFIG_USB_GADGET_MUSB_HDRC)
	.mode = MUSB_PERIPHERAL,
#endif
	.power = 500,
};

static struct omap2_hsmmc_info mmc[] = {
	{
	 .mmc = 2,
	 .caps = MMC_CAP_8_BIT_DATA,
	 .gpio_cd = -EINVAL,
	 .gpio_wp = -EINVAL,
	 .ocr_mask = MMC_VDD_165_195,
	 .nonremovable = true,
#ifdef CONFIG_PM_RUNTIME
	 .power_saving = true,
#endif
	 },
	{
	 .mmc = 1,
	 .caps = MMC_CAP_8_BIT_DATA,
	 .gpio_cd = -EINVAL,
	 .gpio_wp = -EINVAL,
#ifdef CONFIG_PM_RUNTIME
	 .power_saving = true,
#endif
	 },
	{
	 .mmc = 5,
	 .caps = MMC_CAP_8_BIT_DATA,
	 .gpio_cd = -EINVAL,
	 .gpio_wp = -EINVAL,
	 .ocr_mask = MMC_VDD_165_195 | MMC_VDD_30_31 | MMC_VDD_31_32 | MMC_VDD_32_33,
#ifdef CONFIG_PM_RUNTIME
	 .power_saving = false,
#endif
	 },
	{}			/* Terminator */
};

static int omap4_twl6030_hsmmc_late_init(struct device *dev)
{
	int ret = 0;
	struct platform_device *pdev = container_of(dev,
						    struct platform_device,
						    dev);
	struct omap_mmc_platform_data *pdata = dev->platform_data;

	/* Setting MMC1 Card detect Irq */
	if (pdev->id == 0) {
		ret = twl6030_mmc_card_detect_config();
		if (ret)
			pr_err("Failed configuring MMC1 card detect\n");
		pdata->slots[0].card_detect_irq = TWL6030_IRQ_BASE +
		    MMCDETECT_INTR_OFFSET;
		pdata->slots[0].card_detect = twl6030_mmc_card_detect;
	}
	return ret;
}

static __init void omap4_twl6030_hsmmc_set_late_init(struct device *dev)
{
	struct omap_mmc_platform_data *pdata;

	/* dev can be null if CONFIG_MMC_OMAP_HS is not set */
	if (!dev)
		return;

	pdata = dev->platform_data;
	pdata->init = omap4_twl6030_hsmmc_late_init;
}

static int __init omap4_twl6030_hsmmc_init(struct omap2_hsmmc_info *controllers)
{
	struct omap2_hsmmc_info *c;

	omap2_hsmmc_init(controllers);
	for (c = controllers; c->mmc; c++)
		omap4_twl6030_hsmmc_set_late_init(c->dev);

	return 0;
}

static struct regulator_consumer_supply q1_omap4_vmmc_supply[] = {
	{
	 .supply = "vmmc",
	 .dev_name = "mmci-omap-hs.0",
	 },
};

static struct regulator_consumer_supply q1_omap4_cam2_supply[] = {
	{
	 .supply = "cam2pwr",
	 },
};

static struct regulator_consumer_supply q1_omap4_vusim_supply[] = {
	{
	 .supply = "vusim",
	 .dev_name = "mpu3050",	
	 },
};

static struct regulator_consumer_supply q1_omap4_vaux1_supply[] = {
	{
	 .supply = "vaux1",
	 .dev_name = "mpu3050",
	 },
};

static struct regulator_consumer_supply q1_omap4_vaux2_supply[] = {
	{
	 .supply = "vaux2",
	 .dev_name = "mpu3050",
	 },
};

static struct regulator_consumer_supply q1_omap4_vaux3_supply[] = {
	{
	 .supply = "vaux3",
	 .dev_name = "display0",
	 },
};
static struct regulator_consumer_supply q1_omap4_vana_supply[] = {
	{
	 .supply = "vana",
	 .dev_name = NULL,
	 },
};

static struct regulator_init_data q1_omap4_vaux1 = {
	.constraints = {
			.min_uV = 1000000,
			.max_uV = 3000000,
			.apply_uV = true,
			.valid_modes_mask = REGULATOR_MODE_NORMAL
					  | REGULATOR_MODE_STANDBY,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
			},
	.num_consumer_supplies = 1,
	.consumer_supplies = &q1_omap4_vaux1_supply,
};

static struct regulator_init_data q1_omap4_vaux2 = {
	.constraints = {
			.min_uV = 2800000,
			.max_uV = 2800000,
			.apply_uV = true,
			.boot_on = true,
			.valid_modes_mask = REGULATOR_MODE_NORMAL
					  | REGULATOR_MODE_STANDBY,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
			},
	.num_consumer_supplies = 1,
	.consumer_supplies = &q1_omap4_vaux2_supply,		
};

static struct regulator_init_data q1_omap4_vaux3 = {
	.constraints = {
			.min_uV = 3300000,
			.max_uV = 3300000,
			.apply_uV = true,
			.valid_modes_mask = REGULATOR_MODE_NORMAL
					  | REGULATOR_MODE_STANDBY,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
			},
	.num_consumer_supplies = 1,
	.consumer_supplies = q1_omap4_vaux3_supply,
};

/* VMMC1 for MMC1 card */
static struct regulator_init_data q1_omap4_vmmc = {
	.constraints = {
			.min_uV = 1200000,
			.max_uV = 3000000,
			.apply_uV = true,
			.valid_modes_mask = REGULATOR_MODE_NORMAL
					  | REGULATOR_MODE_STANDBY,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
			},
	.num_consumer_supplies = 1,
	.consumer_supplies = q1_omap4_vmmc_supply,
};

static struct regulator_init_data q1_omap4_vpp = {
	.constraints = {
			.min_uV = 1800000,
			.max_uV = 2500000,
			.apply_uV = true,
			.valid_modes_mask = REGULATOR_MODE_NORMAL
					  | REGULATOR_MODE_STANDBY,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
			},
};

static struct regulator_init_data q1_omap4_vusim = {
	.constraints = {
			.min_uV = 1800000,
			.max_uV = 1800000,
			.apply_uV = true,
			.boot_on = true,
			.valid_modes_mask = REGULATOR_MODE_NORMAL
					  | REGULATOR_MODE_STANDBY,
			.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
			},
	.num_consumer_supplies = 1,
	.consumer_supplies = &q1_omap4_vusim_supply,
};

static struct regulator_init_data q1_omap4_vana = {
	.constraints = {
			.min_uV = 2100000,
			.max_uV = 2100000,
			.apply_uV = true,
			.valid_modes_mask = REGULATOR_MODE_NORMAL
					  | REGULATOR_MODE_STANDBY,
			.valid_ops_mask = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
			},
	.num_consumer_supplies = ARRAY_SIZE(q1_omap4_vana_supply),
	.consumer_supplies = q1_omap4_vana_supply,
};

static struct regulator_init_data q1_omap4_vcxio = {
	.constraints = {
			.min_uV = 1800000,
			.max_uV = 1800000,
			.apply_uV = true,
			.valid_modes_mask = REGULATOR_MODE_NORMAL
					  | REGULATOR_MODE_STANDBY,
			.valid_ops_mask = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
			},
};

static struct regulator_init_data q1_omap4_vdac = {
	.constraints = {
			.min_uV = 1800000,
			.max_uV = 1800000,
			.apply_uV = true,
			.valid_modes_mask = REGULATOR_MODE_NORMAL
					  | REGULATOR_MODE_STANDBY,
			.valid_ops_mask = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
			},
};

static struct regulator_init_data q1_omap4_vusb = {
	.constraints = {
			.min_uV = 3300000,
			.max_uV = 3300000,
			.apply_uV = true,
			.valid_modes_mask = REGULATOR_MODE_NORMAL
					  | REGULATOR_MODE_STANDBY,
			.valid_ops_mask = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
			},
};

static struct twl4030_madc_platform_data q1_omap4_gpadc_data = {
	.irq_line = 1,
};

static int q1_omap4_batt_table[] = {
	/* adc code for temperature in degree C */
	929, 925,		/* -2 ,-1 */
	920, 917, 912, 908, 904, 899, 895, 890, 885, 880,	/* 00 - 09 */
	875, 869, 864, 858, 853, 847, 841, 835, 829, 823,	/* 10 - 19 */
	816, 810, 804, 797, 790, 783, 776, 769, 762, 755,	/* 20 - 29 */
	748, 740, 732, 725, 718, 710, 703, 695, 687, 679,	/* 30 - 39 */
	671, 663, 655, 647, 639, 631, 623, 615, 607, 599,	/* 40 - 49 */
	591, 583, 575, 567, 559, 551, 543, 535, 527, 519,	/* 50 - 59 */
	511, 504, 496		/* 60 - 62 */
};

static struct twl4030_bci_platform_data q1_omap4_bci_data = {
	.monitoring_interval = 10,
	.max_charger_currentmA = 1500,
	.max_charger_voltagemV = 4560,
	.max_bat_voltagemV = 4200,
	.low_bat_voltagemV = 3300,
	.termination_currentmA = 50,
	.battery_tmp_tbl = q1_omap4_batt_table,
	.tblsize = ARRAY_SIZE(q1_omap4_batt_table),
};

static void omap4_audio_conf(void)
{
//	int ret;
	/* twl6040 naudint */
	omap_mux_init_signal("sys_nirq2.sys_nirq2", OMAP_PIN_INPUT_PULLUP);
// delete for codec mic bias
#if 0 
	/* request mic bias gpio */
	ret = gpio_request(OMAP_GPIO_MICBIAS_EN, "MAIN_MIC_BIAS_GPIO");
	if (ret) {
		printk(KERN_ERR "failed to get GPIO_MICBIAS\n");
	}
	gpio_direction_output(OMAP_GPIO_MICBIAS_EN, 0);

	ret = gpio_request(OMAP_GPIO_SUB_MICBIAS_EN, "SUB_MIC_MIC_BIAS_GPIO");
	if (ret) {
		printk(KERN_ERR "failed to get GPIO_SUB_MICBIAS\n");
	}
	gpio_direction_output(OMAP_GPIO_SUB_MICBIAS_EN, 0);
#endif
}
void main_mic_bias_cb(bool value)
{
	printk(KERN_ERR "[Audio]main_mic_bias_cb %d\n", value);
	if (value)
		gpio_set_value(OMAP_GPIO_MICBIAS_EN, 1);
	else
		gpio_set_value(OMAP_GPIO_MICBIAS_EN, 0);
}
void sub_mic_bias_cb(bool value)
{
	printk(KERN_ERR "[Audio]sub_mic_bias_cb %d\n", value);
	if (value)
		gpio_set_value(OMAP_GPIO_SUB_MICBIAS_EN, 1);
	else
		gpio_set_value(OMAP_GPIO_SUB_MICBIAS_EN, 0);
}

static struct twl4030_codec_audio_data twl6040_audio = {
	/* Add audio only data */// delete for codec mic bias
//	.mainmic_gpio_cb = main_mic_bias_cb,
//	.submic_gpio_cb = sub_mic_bias_cb,
};

static struct twl4030_codec_vibra_data twl6040_vibra = {
	.max_timeout = 15000,
	.initial_vibrate = 0,
};

static struct twl4030_codec_data twl6040_codec = {
	.audio_mclk = 38400000,
	.audio = &twl6040_audio,
	.vibra = &twl6040_vibra,
	.audpwron_gpio = OMAP_GPIO_AUD_PWRON,
	.naudint_irq = OMAP44XX_IRQ_SYS_2N,
	.irq_base = TWL6040_CODEC_IRQ_BASE,
};

static void external_booster_enable(int enabled)
{
	int on = !!enabled;
	gpio_direction_output(OMAP_GPIO_USB_OTG_EN, on);
	pr_alert("usb %s: otg power = %d\n", __func__, on);
}

static void q1_muic_id_open(void)
{
	pr_alert("usb %s: id_open\n",__func__);

	if (omap4_fsa9480_pdata.id_open_cb)
		omap4_fsa9480_pdata.id_open_cb();
}

static struct twl4030_usb_data twl6030_usb_data = {
#if (CONFIG_SAMSUNG_REL_HW_REV == 1)
	.host_mode		= (TWL_HOST_EXTERNAL_POWER | TWL_HOST_GPIO_ID),
	.gpio_id		= OMAP_GPIO_USB_OTG_ID,
	.muic_id_open	= q1_muic_id_open,
	.booster_enable	= external_booster_enable,
#else
	.host_mode		= 0,
#endif
};

static struct twl4030_platform_data q1_omap4_twldata = {
	.irq_base = TWL6030_IRQ_BASE,
	.irq_end = TWL6030_IRQ_END,

	/* Regulators */
	.vmmc = &q1_omap4_vmmc,
	//.vpp = &q1_omap4_vpp,
	.vusim = &q1_omap4_vusim,
	.vana = &q1_omap4_vana,
	.vcxio = &q1_omap4_vcxio,
	.vdac = &q1_omap4_vdac,
	.vusb = &q1_omap4_vusb,
	.vaux1 = &q1_omap4_vaux1,
	.vaux2 = &q1_omap4_vaux2,
	.vaux3 = &q1_omap4_vaux3,
	.madc = &q1_omap4_gpadc_data,
	.bci = &q1_omap4_bci_data,

	/* children */
	.codec = &twl6040_codec,
	.usb = &twl6030_usb_data,
};

static struct i2c_board_info __initdata q1_omap4_i2c_boardinfo[] = {
	{
	 I2C_BOARD_INFO("twl6030", 0x48),
	 .flags = I2C_CLIENT_WAKE,
	 .irq = OMAP44XX_IRQ_SYS_1N,
	 .platform_data = &q1_omap4_twldata,
	 },
};

/*atmel_mxt224E*/
static void mxt224_power_on(void)
{
/*  temporal workaround for power sequence (rev0.0, rev0.1)
	gpio_direction_output(OMAP_GPIO_TSP_EN, 1); 
	msleep(80);
*/
	/* printk("mxt224_power_on is finished\n"); */
}

static void mxt224_power_off(void)
{
/*  temporal workaround for power sequence (rev0.0, rev0.1)
	gpio_direction_output(OMAP_GPIO_TSP_EN, 0);
*/
	/* printk("mxt224_power_off is finished\n"); */
}

static void mxt224_register_callback(void *function)
{
	printk("mxt224_register_callback\n");
	charging_cbs.tsp_set_charging_cable = function;
}

static void mxt224_read_ta_status(bool *ta_status)
{
	*ta_status = is_cable_attached;
}

#define MXT224_MAX_MT_FINGERS		10

/*
	Configuration for MXT224
*/

#define MXT224_THRESHOLD_BATT		40
#define MXT224_THRESHOLD_CHRG		70
#define MXT224_ATCHCALST		20
#define MXT224_ATCHCALTHR		0

static u8 t7_config[] = {GEN_POWERCONFIG_T7,
				64, 255, 20};
static u8 t8_config[] = {GEN_ACQUISITIONCONFIG_T8,
				64, 0, 20, 20, 0, 0, MXT224_ATCHCALST, MXT224_ATCHCALTHR, 50, 25};
static u8 t9_config[] = {TOUCH_MULTITOUCHSCREEN_T9,
				131, 0, 0, 16, 26, 0, 160, MXT224_THRESHOLD_BATT, 2, 6, 0, 5, 1,
				0, MXT224_MAX_MT_FINGERS, 10, 10, 5, 255, 3,
				255, 3, 0, 0, 0, 0, 136, 60, 136, 40, 40, 15, 0, 0};

static u8 t15_config[] = {TOUCH_KEYARRAY_T15,
				0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

static u8 t18_config[] = {SPT_COMCONFIG_T18,
				0, 0};

static u8 t40_config[] = {PROCI_GRIPSUPPRESSION_T40,
				0, 0, 0, 0, 0};

static u8 t42_config[] = {PROCI_TOUCHSUPPRESSION_T42,
				0, 0, 0, 0, 0, 0, 0, 0};

static u8 t43_config[] = {SPT_DIGITIZER_T43,
				0, 0, 0, 0};

static u8 t48_config[] = {PROCG_NOISESUPPRESSION_T48,
				1, 0, 65, 0, 12, 24, 36, 48, 8, 16, 11, 40, 0, 0,
				0, 0, 0};


static u8 t46_config[] = {SPT_CTECONFIG_T46,
				0, 0, 8, 24, 0, 0, 0, 0};
static u8 end_config[] = {RESERVED_T255};

static const u8 *mxt224_config[] = {
	t7_config,
	t8_config,
	t9_config,
	t15_config,
	t18_config,
	t40_config,
	t42_config,
	t43_config,
	t46_config,
	t48_config,
	end_config,
};


/*
	Configuration for MXT224-E
*/
#define MXT224E_THRESHOLD_BATT		50
#define MXT224E_THRESHOLD_CHRG		70
#define MXT224E_CALCFG_BATT		64
#define MXT224E_CALCFG_CHRG		80
#define MXT224E_ATCHFRCCALTHR_WAKEUP		8
#define MXT224E_ATCHFRCCALRATIO_WAKEUP		180
#define MXT224E_ATCHFRCCALTHR_NORMAL		40
#define MXT224E_ATCHFRCCALRATIO_NORMAL		55

static u8 t7_config_e[] = {GEN_POWERCONFIG_T7,
				48, 255, 25};

static u8 t8_config_e[] = {GEN_ACQUISITIONCONFIG_T8,
				68, 0, 5, 1, 0, 0, 8, 8, MXT224E_ATCHFRCCALTHR_WAKEUP, MXT224E_ATCHFRCCALRATIO_WAKEUP};

static u8 t9_config_e[] = {TOUCH_MULTITOUCHSCREEN_T9,
				139, 0, 0, 16, 26, 0, 176, MXT224E_THRESHOLD_BATT, 2, 6, 10, 3, 1,
				48, MXT224_MAX_MT_FINGERS, 5, 40, 15, 255, 3,
				255, 3, 0, 0, 0, 0, 0, 0, 0, 0, 18, 15, 0, 0, 2};

static u8 t15_config_e[] = {TOUCH_KEYARRAY_T15,
				0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

static u8 t18_config_e[] = {SPT_COMCONFIG_T18,
				0, 0};

static u8 t19_config_e[] = {SPT_GPIOPWM_T19,
				0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

static u8 t24_config_e[] = {PROCI_ONETOUCHGESTUREPROCESSOR_T24,
				0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

static u8 t25_config_e[] = {SPT_SELFTEST_T25,
				0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

static u8 t27_config_e[] = {PROCI_TWOTOUCHGESTUREPROCESSOR_T27,
				0, 0, 0, 0, 0, 0, 0};

static u8 t40_config_e[] = {PROCI_GRIPSUPPRESSION_T40,
				0, 0, 0, 0, 0};

static u8 t42_config_e[] = {PROCI_TOUCHSUPPRESSION_T42,
				0, 0, 0, 0, 0, 0, 0, 0};

static u8 t43_config_e[] = {SPT_DIGITIZER_T43,
				0, 0, 0, 0, 0, 0, 0};

static u8 t46_config_e[] = {SPT_CTECONFIG_T46,
				0, 0, 16, 48, 0, 0, 1, 0};

static u8 t47_config_e[] = {PROCI_STYLUS_T47,
				1, 30, 60, 15, 2, 20, 20, 150, 0, 40};

static u8 t48_config_e[] = {PROCG_NOISESUPPRESSION_T48,
				2, 12, MXT224E_CALCFG_BATT, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
				6,	6, 0, 0, 100, 4, 64, 10, 0, 20, 6, 0, 46, 0, 20,
				0, 0, 0, 0, 0, 0, 128, MXT224E_THRESHOLD_BATT, 2, 3, 1, 0, MXT224_MAX_MT_FINGERS, 5, 40, 10, 10,
				10, 10, 143, 40, 143, 80, 18, 15, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

static u8 t48_config_chrg_e[] = {PROCG_NOISESUPPRESSION_T48,
				1, 12, MXT224E_CALCFG_CHRG, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
				6,	6, 0, 0, 100, 4, 64, 10, 0, 20, 5, 0, 38, 0, 20,
				0, 0, 0, 0, 0, 0, 160, MXT224E_THRESHOLD_CHRG, 2, 5, 2, 46, MXT224_MAX_MT_FINGERS, 5, 40, 10, 0,
				10, 10, 143, 40, 143, 80, 18, 15, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

static u8 t52_config_e[] = {TOUCH_PROXKEY_T52,
				0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

static u8 end_config_e[] = {RESERVED_T255};

static const u8 *mxt224e_config[] = {
	t7_config_e,
	t8_config_e,
	t9_config_e,
	t15_config_e,
	t18_config_e,
	t19_config_e,
	t24_config_e,
	t25_config_e,
	t27_config_e,
	t40_config_e,
	t42_config_e,
	t43_config_e,
	t46_config_e,
	t47_config_e,
	t48_config_e,
	t52_config_e,
	end_config_e,
};

static struct mxt224_platform_data mxt224_data = {
	.max_finger_touches = MXT224_MAX_MT_FINGERS,
	.gpio_read_done = OMAP_GPIO_TSP_nINT,
	.config = mxt224_config,
	.config_e = mxt224e_config,
	.min_x = 0,
	.max_x = 1023,
	.min_y = 0,
	.max_y = 1023,
	.min_z = 0,
	.max_z = 255,
	.min_w = 0,
	.max_w = 30,
	.atchcalst = MXT224_ATCHCALST,
	.atchcalsthr = MXT224_ATCHCALTHR,
	.tchthr_batt = MXT224_THRESHOLD_BATT,
	.tchthr_charging = MXT224_THRESHOLD_CHRG,
	.tchthr_batt_e = MXT224E_THRESHOLD_BATT,
	.tchthr_charging_e = MXT224E_THRESHOLD_CHRG,
	.calcfg_batt_e = MXT224E_CALCFG_BATT,
	.calcfg_charging_e = MXT224E_CALCFG_CHRG,
	.atchfrccalthr_e = MXT224E_ATCHFRCCALTHR_NORMAL,
	.atchfrccalratio_e = MXT224E_ATCHFRCCALRATIO_NORMAL,
	.power_on = mxt224_power_on,
	.power_off = mxt224_power_off,
	.register_cb = mxt224_register_callback,
	.read_ta_status = mxt224_read_ta_status,
};

#ifdef CONFIG_EPEN_WACOM_G5SP

static void q1_wacom_register_callbacks(struct wacom_g5_callbacks *cb)
{
	wacom_callbacks = cb;
};

static int q1_wacom_init_hw(void)
{
	int ret;
	
	ret = gpio_request(OMAP_GPIO_PEN_LDO_EN, "PEN_LDO_EN");
	if (ret) {
		printk(KERN_ERR "[E-PEN]: faile to request gpio(OMAP_GPIO_PEN_LDO_EN)\n");
		return ret;
	}
	gpio_direction_output(OMAP_GPIO_PEN_LDO_EN, 0);

	ret = gpio_request(OMAP_GPIO_PEN_PDCT, "PEN_PDCT");
	if (ret) {
		printk(KERN_ERR "[E-PEN]: faile to request gpio(OMAP_GPIO_PEN_PDCT)\n");
		return ret;
	}
	gpio_direction_input(OMAP_GPIO_PEN_PDCT);
	
	ret = gpio_request(OMAP_GPIO_PEN_SLP, "PEN_SLP");
	if (ret) {
		printk(KERN_ERR "[E-PEN]: faile to request gpio(OMAP_GPIO_PEN_SLP)\n");
		return ret;
	}
	gpio_direction_output(OMAP_GPIO_PEN_SLP, 0);
	
	ret = gpio_request(OMAP_GPIO_PEN_IRQ, "PEN_IRQ");
	if (ret) {
		printk(KERN_ERR "[E-PEN]: faile to request gpio(OMAP_GPIO_PEN_IRQ)\n");
		return ret;
	}
	gpio_direction_input(OMAP_GPIO_PEN_IRQ);

#if (WACOM_HAVE_RESET_CONTROL == 1)	
	ret = gpio_request(OMAP_GPIO_PEN_RST, "PEN_RST");
	if (ret) {
		printk(KERN_ERR "[E-PEN]: faile to request gpio(OMAP_GPIO_PEN_RST)\n");
		return ret;
	}
	gpio_direction_output(OMAP_GPIO_PEN_RST, 0);
#endif	
	return 0;
}

static int __init q1_wacom_init(void)
{
	int ret;
	q1_wacom_init_hw();
	
	gpio_set_value(OMAP_GPIO_PEN_LDO_EN, 1);

#if (WACOM_HAVE_RESET_CONTROL == 1)	
	//100ms delay. H/W spec
	msleep(WACOM_DELAY_FOR_RST_RISING);
	gpio_set_value(OMAP_GPIO_PEN_RST, 1);
#endif	
	printk(KERN_INFO "[E-PEN] : %s.\n", __func__);
	return 0;
}

static int q1_wacom_reset_hw(void)
{

#if (WACOM_HAVE_RESET_CONTROL == 1)
	gpio_set_value(OMAP_GPIO_PEN_RST, 0);
	msleep(WACOM_DELAY_FOR_RST_RISING);
	gpio_set_value(OMAP_GPIO_PEN_RST, 1);
#endif

	printk(KERN_INFO "[E-PEN] : wacom warm reset(%d).\n", WACOM_HAVE_RESET_CONTROL);
	return 0;
}

static int q1_wacom_early_suspend_hw(void)
{
#if defined(WACOM_SLEEP_WITH_PEN_SLP)
	gpio_set_value(OMAP_GPIO_PEN_SLP, 1);
#elif defined(WACOM_SLEEP_WITH_PEN_LDO_EN)
	gpio_set_value(OMAP_GPIO_PEN_LDO_EN, 0);
#endif
	return 0;
}

static int q1_wacom_late_resume_hw(void)
{
#if defined(WACOM_SLEEP_WITH_PEN_SLP)
	gpio_set_value(OMAP_GPIO_PEN_SLP, 0);
#elif defined(WACOM_SLEEP_WITH_PEN_LDO_EN)
	gpio_set_value(OMAP_GPIO_PEN_LDO_EN, 1);
#endif

#if (WACOM_HAVE_RESET_CONTROL == 1)
	msleep(WACOM_DELAY_FOR_RST_RISING);
	gpio_set_value(OMAP_GPIO_PEN_RST, 1);
#endif
	return 0;
}

static int q1_wacom_suspend_hw(void)
{
	return(q1_wacom_early_suspend_hw());
}

static int q1_wacom_resume_hw(void)
{
	return(q1_wacom_late_resume_hw());
}

static struct wacom_g5_platform_data q1_wacom_platform_data = {
	.x_invert = 1,
	.y_invert = 0,
	.xy_switch = 1,
	.init_platform_hw = q1_wacom_init_hw,
/*	.exit_platform_hw =,	*/
	.suspend_platform_hw = q1_wacom_suspend_hw,
	.resume_platform_hw = q1_wacom_resume_hw,
	.early_suspend_platform_hw = q1_wacom_early_suspend_hw,
	.late_resume_platform_hw = q1_wacom_late_resume_hw,
	.reset_platform_hw = q1_wacom_reset_hw,
	.register_cb = q1_wacom_register_callbacks,
};
#endif /* CONFIG_EPEN_WACOM_G5SP */

#ifndef DISABLE_I2C2_HW_SPINLOCK
static struct i2c_board_info __initdata q1_omap4_i2c_2_boardinfo[] = {
	{
	 I2C_BOARD_INFO("cam_pmic", 0x7D),
	 },
	{
	 I2C_BOARD_INFO("M5MO", 0x3E >> 1),
	 }
};
#endif
#if 1
static struct pn544_i2c_platform_data pn544_data = {
	.irq_gpio = OMAP_GPIO_NFC_IRQ,
	.ven_gpio = OMAP_GPIO_NFC_VEN,
	.firm_gpio = OMAP_GPIO_NFC_FIRM,
};
#endif
static struct i2c_board_info __initdata q1_omap4_i2c_3_boardinfo[] = {
	{
	 I2C_BOARD_INFO("melfas_touchkey", 0x20),
	 },
	{
		I2C_BOARD_INFO("secFuelgaugeDev", 0x36),
		.flags = I2C_CLIENT_WAKE,
		.irq = OMAP_GPIO_IRQ(OMAP_GPIO_FUEL_ALERT),
	},
	{
	 I2C_BOARD_INFO(MXT224_DEV_NAME, 0x4C),
	 .irq = OMAP_GPIO_IRQ(OMAP_GPIO_TSP_nINT),
	 .platform_data = &mxt224_data,
	 },
     {
        I2C_BOARD_INFO("Si4709_driver", 0x10),
        .irq = OMAP_GPIO_IRQ(OMAP_GPIO_FM_INT),
     },	 
};

static struct i2c_board_info __initdata q1_omap4_i2c_4_boardinfo[] = {
#ifdef CONFIG_MPU_SENSORS_MPU3050
	{
		 I2C_BOARD_INFO(SENSOR_MPU_NAME, 0x68),
		 .irq = OMAP_GPIO_IRQ(OMAP_GPIO_MPU3050_INT),
		 .platform_data = &mpu_data,
	 },
#endif
#ifdef CONFIG_OPTICAL_GP2A
	{
		 I2C_BOARD_INFO("gp2a", (0x88 >> 1)),
		.platform_data = &gp2a_pdata,
	},
#endif	
#ifdef CONFIG_INPUT_CM3663
	{
	 	I2C_BOARD_INFO("cm3663", 0x11),
	 	.platform_data = &cm3663_pdata,
	 	//.irq = OMAP_GPIO_IRQ(OMAP_GPIO_PS_VOUT),
	 },
#endif	 
	{
		 I2C_BOARD_INFO("fsa9480", (0x4A >> 1)),
		 .platform_data = &omap4_fsa9480_pdata,
		 .irq = OMAP_GPIO_IRQ(OMAP_GPIO_JACK_NINT),
	 },
	 {
		I2C_BOARD_INFO("geomagnetic", 0x2e),
		.platform_data = &yas_data,		
	 },
#ifdef CONFIG_INPUT_BMP182	 
	 {
		I2C_BOARD_INFO("bmp182", 0x77),
	 },
#endif	 
#ifdef CONFIG_EPEN_WACOM_G5SP
	{
		I2C_BOARD_INFO("wacom_g5sp_i2c", 0x56),
		.platform_data = &q1_wacom_platform_data,
		.irq = OMAP_GPIO_IRQ(OMAP_GPIO_PEN_IRQ),
	},
#endif /* CONFIG_EPEN_WACOM_G5SP */
#if 1	 
	{
	 I2C_BOARD_INFO("pn544", 0x2b),
	 .platform_data = &pn544_data,
	 .irq = OMAP_GPIO_IRQ(OMAP_GPIO_NFC_IRQ),
	 },
#endif	 
};
#ifdef CONFIG_VIDEO_MHL_V1
static struct i2c_board_info q1_omap4_i2c_5_boardinfo[] = {
    {
        I2C_BOARD_INFO("SII9234", 0x72>>1),
    },
    {
        I2C_BOARD_INFO("SII9234A", 0x7A>>1),
    },
    {
        I2C_BOARD_INFO("SII9234B", 0x92>>1),
    },
    {
        I2C_BOARD_INFO("SII9234C", 0xC8>>1),
    },
};
#endif
static const struct usbhs_omap_platform_data usbhs_pdata __initconst = {
	.port_mode[0] = OMAP_OHCI_PORT_MODE_PHY_4PIN_DPDM,
	.port_mode[1] = OMAP_USBHS_PORT_MODE_UNUSED,
	.port_mode[2] = OMAP_USBHS_PORT_MODE_UNUSED,
	.phy_reset = false,
	.reset_gpio_port[0] = -EINVAL,
	.reset_gpio_port[1] = -EINVAL,
	.reset_gpio_port[2] = -EINVAL
};

static struct omap_i2c_bus_board_data __initdata q1_omap4_i2c_bus_pdata;
static struct omap_i2c_bus_board_data __initdata q1_omap4_i2c_2_bus_pdata;
static struct omap_i2c_bus_board_data __initdata q1_omap4_i2c_3_bus_pdata;
static struct omap_i2c_bus_board_data __initdata q1_omap4_i2c_4_bus_pdata;

/*
 * LPDDR2 Configuration Data:
 * The memory organisation is as below :
 *	EMIF1 - CS0 -	2 Gb
 *	CS1 -	NULL
 *	EMIF2 - CS0 -	2 Gb
 *	CS1 -	 NULL
 *	--------------------
 *	TOTAL - 	4 Gb
 *
 * Same devices installed on EMIF1 and EMIF2
 */
static __initdata struct emif_device_details emif_devices = {
	.cs0_device = &sec_4G_S4,
	.cs1_device = NULL
};

static void __init omap_i2c_hwspinlock_init(int bus_id, unsigned int spinlock_id,
					    struct omap_i2c_bus_board_data *pdata)
{
	pdata->handle = hwspinlock_request_specific(spinlock_id);
	if (pdata->handle != NULL) {
		pdata->hwspinlock_lock = hwspinlock_lock;
		pdata->hwspinlock_unlock = hwspinlock_unlock;
	} else {
		pr_err("I2C hwspinlock request failed for bus %d\n", bus_id);
	}
}

static int __init omap4_i2c_init(void)
{
	omap_i2c_hwspinlock_init(1, 0, &q1_omap4_i2c_bus_pdata);
#ifndef DISABLE_I2C2_HW_SPINLOCK
	omap_i2c_hwspinlock_init(2, 1, &q1_omap4_i2c_2_bus_pdata);
#endif
	omap_i2c_hwspinlock_init(3, 2, &q1_omap4_i2c_3_bus_pdata);
	omap_i2c_hwspinlock_init(4, 3, &q1_omap4_i2c_4_bus_pdata);
	/*
	 * Phoenix Audio IC needs I2C1 to
	 * start with 400 KHz or less
	 */
	omap_register_i2c_bus(1, 400, &q1_omap4_i2c_bus_pdata,
			      q1_omap4_i2c_boardinfo,
			      ARRAY_SIZE(q1_omap4_i2c_boardinfo));
#ifndef DISABLE_I2C2_HW_SPINLOCK
	omap_register_i2c_bus(2, 400, &q1_omap4_i2c_2_bus_pdata,
			      q1_omap4_i2c_2_boardinfo,
			      ARRAY_SIZE(q1_omap4_i2c_2_boardinfo));
#endif
	omap_register_i2c_bus(3, 400, &q1_omap4_i2c_3_bus_pdata,
			      q1_omap4_i2c_3_boardinfo,
			      ARRAY_SIZE(q1_omap4_i2c_3_boardinfo));
	omap_register_i2c_bus(4, 400, &q1_omap4_i2c_4_bus_pdata,
			      q1_omap4_i2c_4_boardinfo,
			      ARRAY_SIZE(q1_omap4_i2c_4_boardinfo));

#ifdef CONFIG_VIDEO_MHL_V1
	i2c_register_board_info(5, q1_omap4_i2c_5_boardinfo,
			            ARRAY_SIZE(q1_omap4_i2c_5_boardinfo));
#endif	
	return 0;
}

static u16 control_pbias_offset;

void q1_omap4_gpiowk_setup()
{
#define CONTROL_GPIOWK				0x4A31E600
	u32 val;
	u32 *ctrl_gpiowk;

	control_pbias_offset = OMAP44XX_CONTROL_PBIAS_LITE;
	val = omap_ctrl_readl(control_pbias_offset);

	val = val | (1 << 28);
	omap_ctrl_writel(val, control_pbias_offset);

	val = omap_ctrl_readl(control_pbias_offset);

	ctrl_gpiowk = (u32 *) ioremap(CONTROL_GPIOWK, 4);
	if (!ctrl_gpiowk) {
		printk(KERN_ERR
		       "OMAP_pad_config: ioremap failed with addr %lx\n",
		       CONTROL_GPIOWK);
		return;
	}

	val = __raw_readl(ctrl_gpiowk);

	val |= (1 << 28);
	__raw_writel(val, ctrl_gpiowk);

	val = __raw_readl(ctrl_gpiowk);

	val = omap_ctrl_readl(control_pbias_offset);

	if ((val & (1 << 29)) != 0) {
		val = val & ~(1 << 28);
		omap_ctrl_writel(val, control_pbias_offset);

		val = val & ~(1 << 28);
		__raw_writel(val, ctrl_gpiowk);
	}

	val = omap_ctrl_readl(control_pbias_offset);
	if ((val & (1 << 30)) == 1)
		printk("SIM_VDDS is set to 3 volt\n");
	else
		printk("SIM_VDDS is set to 1.8 volt\n");

	return;
}

#ifdef CONFIG_SWITCH_GPIO
static void __init q1_omap4_headset_init(void)
{
	u32 gpio_reg_addr;
	u32 reg_val;
	u32 ret;
	ret = gpio_request(OMAP_GPIO_EAR_MICBIAS_EN, "EAR_MIC_BIAS_GPIO");
	if (ret) {
		printk(KERN_ERR "failed to get EAR_MIC_BIAS_GPIO\n");
	}
	gpio_direction_output(OMAP_GPIO_EAR_MICBIAS_EN, 0);
#ifdef CONFIG_INPUT_EAR_KEY
	board_ear_key_resource.start = gpio_to_irq(OMAP_GPIO_EAR_SEND_END);
#endif
}
#endif
#ifdef CONFIG_EXTRA_DOCK_SPEAKER
static void __init desk_dock_init(void)
{
	int ret;

	/* for CarDock, DeskDock */
	ret = switch_dev_register(&switch_dock);
	if (ret < 0)
		pr_err("Failed to register dock switch. %d\n", ret);
}
#endif

static void __init q1_omap4_display_init(void)
{
	int r;

	mlcd_rst_gpio = OMAP_GPIO_MLCD_RST;

	r = gpio_request(mlcd_rst_gpio, "LCD Reset GPIO");
	if (r) {
		printk(KERN_ERR "failed to get LCD Reset GPIO\n");
		goto err1;
	}
	omap_writel(0x1F07C000, 0x4a100618 ); //CONTROL_DSIPHY, DSI1_LANE EN, DSI1 PD Dis	

	return;

err1:
	gpio_free(mlcd_rst_gpio);
err0:
	return;
}

static void q1_omap4_touch_init(void)
{
        gpio_request(OMAP_GPIO_TOUCH_EN, "TOUCH_EN");
        gpio_direction_output(OMAP_GPIO_TOUCH_EN, 1);
        gpio_request(OMAP_GPIO_TSP_EN, "TSP_EN");
        gpio_direction_output(OMAP_GPIO_TSP_EN, 1);
        gpio_request(OMAP_GPIO_TSP_nINT, "TSP_INT");
        gpio_direction_input(OMAP_GPIO_TSP_nINT);
}

static void q1_omap4_vib_motor_init(void)
{
	if (gpio_request(OMAP_GPIO_MOTOR_EN, "MOTOR_EN"))
	{
		printk(KERN_ERR "Filed to request OMAP_GPIO_MOTOR_EN!\n");
	}
	gpio_direction_output(OMAP_GPIO_MOTOR_EN, 0);
}

static void q1_omap4_usb_gpio_init(void)
{
	if (gpio_request(OMAP_GPIO_USB_OTG_EN, "USB_OTG_EN"))
	{
		printk(KERN_ERR "Filed to request OMAP_GPIO_USB_OTG_EN!\n");
	}
	gpio_direction_output(OMAP_GPIO_USB_OTG_EN, 0);
}

static void enable_board_wakeup_source(void)
{
	/*
	 * Enable IO daisy for sys_nirq1/2 & HSI CAWAKE line, to be able to
	 * wakeup from interrupts from PMIC/Audio IC.
	 * Needed only in Device OFF mode.
	 */
	omap_mux_enable_wakeup("sys_nirq1");
	omap_mux_enable_wakeup("sys_nirq2");
	omap_mux_enable_wakeup("usbb1_ulpitll_clk.hsi1_cawake");
}

static struct omap_volt_pmic_info omap_pmic_core = {
	.name = "twl",
	.slew_rate = 4000,
	.step_size = 12500,
	.i2c_addr = 0x12,
	.i2c_vreg = 0x61,
	.i2c_cmdreg = 0x62,
	.vsel_to_uv = omap_twl_vsel_to_uv,
	.uv_to_vsel = omap_twl_uv_to_vsel,
	.onforce_cmd = omap_twl_onforce_cmd,
	.on_cmd = omap_twl_on_cmd,
	.sleepforce_cmd = omap_twl_sleepforce_cmd,
	.sleep_cmd = omap_twl_sleep_cmd,
	.vp_config_erroroffset = 0,
	.vp_vstepmin_vstepmin = 0x01,
	.vp_vstepmax_vstepmax = 0x04,
	.vp_vlimitto_timeout_us = 0x200,
	.vp_vlimitto_vddmin = 0xA,
	.vp_vlimitto_vddmax = 0x28,
};

static struct omap_volt_pmic_info omap_pmic_mpu = {
	.name = "twl",
	.slew_rate = 4000,
	.step_size = 12500,
	.i2c_addr = 0x12,
	.i2c_vreg = 0x55,
	.i2c_cmdreg = 0x56,
	.vsel_to_uv = omap_twl_vsel_to_uv,
	.uv_to_vsel = omap_twl_uv_to_vsel,
	.onforce_cmd = omap_twl_onforce_cmd,
	.on_cmd = omap_twl_on_cmd,
	.sleepforce_cmd = omap_twl_sleepforce_cmd,
	.sleep_cmd = omap_twl_sleep_cmd,
	.vp_config_erroroffset = 0,
	.vp_vstepmin_vstepmin = 0x01,
	.vp_vstepmax_vstepmax = 0x04,
	.vp_vlimitto_timeout_us = 0x200,
	.vp_vlimitto_vddmin = 0xA,
	.vp_vlimitto_vddmax = 0x39,
};

static struct omap_volt_pmic_info omap_pmic_iva = {
	.name = "twl",
	.slew_rate = 4000,
	.step_size = 12500,
	.i2c_addr = 0x12,
	.i2c_vreg = 0x5b,
	.i2c_cmdreg = 0x5c,
	.vsel_to_uv = omap_twl_vsel_to_uv,
	.uv_to_vsel = omap_twl_uv_to_vsel,
	.onforce_cmd = omap_twl_onforce_cmd,
	.on_cmd = omap_twl_on_cmd,
	.sleepforce_cmd = omap_twl_sleepforce_cmd,
	.sleep_cmd = omap_twl_sleep_cmd,
	.vp_config_erroroffset = 0,
	.vp_vstepmin_vstepmin = 0x01,
	.vp_vstepmax_vstepmax = 0x04,
	.vp_vlimitto_timeout_us = 0x200,
	.vp_vlimitto_vddmin = 0xA,
	.vp_vlimitto_vddmax = 0x2D,
};

static struct omap_volt_vc_data vc_config = {
	.vdd0_on = 1375000,	/* 1.375v */
	.vdd0_onlp = 1375000,	/* 1.375v */
	.vdd0_ret = 837500,	/* 0.8375v */
	.vdd0_off = 0,		/* 0 v */
	.vdd1_on = 1300000,	/* 1.3v */
	.vdd1_onlp = 1300000,	/* 1.3v */
	.vdd1_ret = 837500,	/* 0.8375v */
	.vdd1_off = 0,		/* 0 v */
	.vdd2_on = 1200000,	/* 1.2v */
	.vdd2_onlp = 1200000,	/* 1.2v */
	.vdd2_ret = 837500,	/* .8375v */
	.vdd2_off = 0,		/* 0 v */
};

void plat_hold_wakelock(void *up, int flag)
{
	struct uart_omap_port *up2 = (struct uart_omap_port *)up;
	/*Specific wakelock for bluetooth usecases */
	if ((up2->pdev->id == BLUETOOTH_UART)
	    && ((flag == WAKELK_TX) || (flag == WAKELK_RX)))
		wake_lock_timeout(&uart_lock, 2 * HZ);

	/*Specific wakelock for console usecases */
	if ((up2->pdev->id != BLUETOOTH_UART)
	    && ((flag == WAKELK_IRQ) || (flag == WAKELK_RESUME)))
		wake_lock_timeout(&uart_lock, 5 * HZ);
	return;
}

static struct omap_uart_port_info omap_serial_platform_data[] = {
	{
	 .use_dma = 0,
	 .dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
	 .dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
	 .dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
	 .idle_timeout = DEFAULT_IDLE_TIMEOUT,
	 .flags = 1,
	 .plat_hold_wakelock = NULL,
	 .rts_padconf = 0,
	 .rts_override = 0,
	 .padconf = OMAP4_CTRL_MODULE_PAD_SDMMC1_CMD_OFFSET,
	 .padconf_wake_ev = 0,
	 .wk_mask = 0,

	 },
	{
	 .use_dma = 0,
	 .dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
	 .dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
	 .dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
	 .idle_timeout = DEFAULT_IDLE_TIMEOUT,
	 .flags = 1,
	 .plat_hold_wakelock = plat_hold_wakelock,
	 .rts_padconf = OMAP4_CTRL_MODULE_PAD_UART2_RTS_OFFSET,
	 .rts_override = 0,
	 .padconf = OMAP4_CTRL_MODULE_PAD_UART2_RX_OFFSET,
	 .padconf_wake_ev = OMAP4_CTRL_MODULE_PAD_CORE_PADCONF_WAKEUPEVENT_3,
	 .wk_mask = OMAP4_UART2_TX_DUPLICATEWAKEUPEVENT_MASK
		  | OMAP4_UART2_RX_DUPLICATEWAKEUPEVENT_MASK
		  | OMAP4_UART2_RTS_DUPLICATEWAKEUPEVENT_MASK 
		  | OMAP4_UART2_CTS_DUPLICATEWAKEUPEVENT_MASK,

	 },
	{
	 .use_dma = 0,
	 .dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
	 .dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
	 .dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
	 .idle_timeout = DEFAULT_IDLE_TIMEOUT,
	 .flags = 1,
	 .plat_hold_wakelock = plat_hold_wakelock,
	 .rts_padconf = 0,
	 .rts_override = 0,
	 .padconf = OMAP4_CTRL_MODULE_PAD_UART3_RX_IRRX_OFFSET,
	 .padconf_wake_ev = OMAP4_CTRL_MODULE_PAD_CORE_PADCONF_WAKEUPEVENT_4,
	 .wk_mask = OMAP4_UART3_TX_IRTX_DUPLICATEWAKEUPEVENT_MASK
		  | OMAP4_UART3_RX_IRRX_DUPLICATEWAKEUPEVENT_MASK
		  | OMAP4_UART3_RTS_SD_DUPLICATEWAKEUPEVENT_MASK
		  | OMAP4_UART3_CTS_RCTX_DUPLICATEWAKEUPEVENT_MASK,
	 },
	{
	 .use_dma = 0,
	 .dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
	 .dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
	 .dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
	 .idle_timeout = DEFAULT_IDLE_TIMEOUT,
	 .flags = 1,
	 .plat_hold_wakelock = NULL,
	 .rts_padconf = 0,
	 .rts_override = 0,
	 .padconf = OMAP4_CTRL_MODULE_PAD_UART4_RX_OFFSET,
	 .padconf_wake_ev = OMAP4_CTRL_MODULE_PAD_CORE_PADCONF_WAKEUPEVENT_4,
	 .wk_mask = OMAP4_UART4_TX_DUPLICATEWAKEUPEVENT_MASK
		  | OMAP4_UART4_RX_DUPLICATEWAKEUPEVENT_MASK,
	 },
	{
	 .flags = 0}
};

/* #ifdef DISABLE_I2C2_HW_SPINLOCK
void m5mo_fref_start(void)
{
	volatile u32 val = 0, val2 = 0;
	void __iomem *frefclk1_base = NULL, *frefclk2_base = NULL;

	val = 0;

	frefclk1_base = ioremap(0x4A30A314, 4);
	val = __raw_readl(frefclk1_base);
	val = val | (1 << 8) | (0x01 << 2) | (0xF << 16);
	__raw_writel(val, (frefclk1_base));
	val = __raw_readl(frefclk1_base);
	printk(KERN_ERR "\n\t [MBG043 MCLK] AUXCLK1 is %x \n\n", val);
	iounmap(frefclk1_base);

	val2 = 0;

	frefclk2_base = ioremap(0x4A30A318, 4);
	val2 = __raw_readl(frefclk2_base);
	val2 = val2 | (1 << 8) | (0x01 << 2) | (0xF << 16);
	__raw_writel(val2, (frefclk2_base));
	val2 = __raw_readl(frefclk2_base);
	printk(KERN_ERR "\n\t [S5KAAFX MCLK] AUXCLK2 is %x \n\n", val2);
}
#endif */

static void enable_rtc_gpio(void)
{
	/* To access twl registers we enable gpio6
	 * we need this so the RTC driver can work.
	 */
	gpio_request(TWL6030_RTC_GPIO, "h_SYS_DRM_MSEC");
	gpio_direction_output(TWL6030_RTC_GPIO, 1);

	omap_mux_init_signal("fref_clk0_out.gpio_wk6",
			     OMAP_PIN_OUTPUT | OMAP_PIN_OFF_NONE);
	return;
}

/* Disable default configuration of VREF_EN to minimize DDR leakage */
static void omap4_lpddr2_config(void)
{
	int control_io1_3;
	int control_io2_3;

	control_io1_3 =
		omap4_ctrl_pad_readl(OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_LPDDR2IO1_3);
	control_io2_3 =
		omap4_ctrl_pad_readl(OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_LPDDR2IO2_3);
	omap4_ctrl_pad_writel(control_io1_3 & 0xFFFFFFF0,
			      OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_LPDDR2IO1_3);
	omap4_ctrl_pad_writel(control_io2_3 & 0xFFFFFFF0,
			      OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_LPDDR2IO2_3);
}

static void omap4_mpl_init(void)
{
	gpio_request(OMAP_GPIO_MPU3050_INT, "MPUIRQ");
	gpio_direction_input(OMAP_GPIO_MPU3050_INT);
}

static void gsd4t_gps_init(void)
{
	BUG_ON(!sec_class);
	gps_dev = device_create(sec_class, NULL, 0, NULL, "gps");
	if (IS_ERR(gps_dev))
		pr_err("Failed to create device(gps)!\n");

#if (CONFIG_SAMSUNG_REL_HW_REV == 0)
	gpio_request(OMAP_GPIO_AP_AGPS_TSYNC, "AP_AGPS_TSYNC");
	gpio_direction_output(OMAP_GPIO_AP_AGPS_TSYNC, 0);
#endif /* end of (CONFIG_SAMSUNG_REL_HW_REV == 0) */

	gpio_request(OMAP_GPIO_GPS_nRST, "GPS_nRST");
	gpio_direction_output(OMAP_GPIO_GPS_nRST, 1);

	gpio_request(OMAP_GPIO_GPS_PWR_EN, "GPS_PWR_EN");
	gpio_direction_output(OMAP_GPIO_GPS_PWR_EN, 0);

	gpio_export(OMAP_GPIO_GPS_nRST, 1);
	gpio_export(OMAP_GPIO_GPS_PWR_EN, 1);

	BUG_ON(!gps_dev);
	gpio_export_link(gps_dev, "GPS_nRST", OMAP_GPIO_GPS_nRST);
	gpio_export_link(gps_dev, "GPS_PWR_EN", OMAP_GPIO_GPS_PWR_EN);
}



static u8 get_hw_rev_gpio(void)
{
	int err = 0;
	static u8 hw_rev = 0xFF;

	if(hw_rev == 0xFF){
		err = gpio_request(OMAP_GPIO_HW_REV0, "HW_REV0");
		if(err){
			printk("hw_rev_gpio - fail to request gpio %s : %d\n",
			       "HW_REV0", err);
		} else {
			gpio_direction_input(OMAP_GPIO_HW_REV0);
		}
		err = gpio_request(OMAP_GPIO_HW_REV1, "HW_REV1");
		if(err){
			printk("hw_rev_gpio - fail to request gpio %s : %d\n",
			       "HW_REV1", err);
		} else {
			gpio_direction_input(OMAP_GPIO_HW_REV1);
		}	
		err = gpio_request(OMAP_GPIO_HW_REV2, "HW_REV2");
		if(err){
			printk("hw_rev_gpio - fail to request gpio %s : %d\n",
			       "HW_REV2", err);
		} else {
			gpio_direction_input(OMAP_GPIO_HW_REV2);
		}	
		err = gpio_request(OMAP_GPIO_HW_REV3, "HW_REV3");
		if(err){
			printk("hw_rev_gpio - fail to request gpio %s : %d\n",
			       "HW_REV3", err);
		} else {
			gpio_direction_input(OMAP_GPIO_HW_REV3);
		}

		
		hw_rev = gpio_get_value(OMAP_GPIO_HW_REV3) << 3;
		hw_rev |= gpio_get_value(OMAP_GPIO_HW_REV2) << 2;
		hw_rev |= gpio_get_value(OMAP_GPIO_HW_REV1) << 1;
		hw_rev |= gpio_get_value(OMAP_GPIO_HW_REV0);
	}
	return hw_rev;
}



/* WLAN */

/* OMAP4 tablet 10-1 specific WLAN GPIOs */ 
#define OMAP4TAB_WLAN_EN_GPIO               104
#define OMAP4TAB_WLAN_HOST_WAKE_GPIO      	81

#define GPIO_LEVEL_LOW          0
#define GPIO_LEVEL_HIGH         1
#define GPIO_LEVEL_NONE         2

#if 1 /* new function */
extern void omap_hsmmc_force_presence_change_for_wlan(int); 
#else  /* old function */
extern void omap_hsmmc_force_presence_change(void);
#endif

/* WLAN GPIO mux config */
static void omap4_wlan_mux_config(void)
{
	omap_mux_init_gpio(OMAP4TAB_WLAN_HOST_WAKE_GPIO, OMAP_PIN_INPUT |
				OMAP_PIN_OFF_WAKEUPENABLE);
	omap_mux_init_gpio(OMAP4TAB_WLAN_EN_GPIO, OMAP_PIN_OUTPUT);
}

/* WLAN GPIO mux init */
static void omap4_wlan_init(void)
{
	int ret = -ENODEV;

	printk(KERN_WARNING"%s: start\n", __func__);

	ret = gpio_request(OMAP4TAB_WLAN_EN_GPIO, "wlan_en");
	if (ret < 0) {
		printk(KERN_ERR"%s: can't reserve GPIO: %d\n", __func__,
			OMAP4TAB_WLAN_EN_GPIO);
		goto out;
	}
	gpio_direction_output(OMAP4TAB_WLAN_EN_GPIO, 0);

	ret = gpio_request(OMAP4TAB_WLAN_HOST_WAKE_GPIO, "wlan_host_wake");
	if (ret < 0) {
		printk(KERN_ERR"%s: can't reserve GPIO: %d\n", __func__,
			OMAP4TAB_WLAN_HOST_WAKE_GPIO);
		goto out;
	}
	gpio_direction_input(OMAP4TAB_WLAN_HOST_WAKE_GPIO);

out:
	return;
}

/* WLAN power control */
void wlan_setup_power(int on, int flag)
{
	printk(KERN_WARNING"%s: %s", __func__, on ? "on" : "down");
	
	if (flag != 1) {
		printk(KERN_WARNING"%s: (on=%d, flag=%d)\n", __func__, on, flag);
		
		if (on) {
			/* wlan reset on */
			gpio_set_value(OMAP4TAB_WLAN_EN_GPIO, GPIO_LEVEL_HIGH);
		}
		else {
			/* wlan reset off */
			gpio_set_value(OMAP4TAB_WLAN_EN_GPIO, GPIO_LEVEL_LOW);
		}
	
		printk(KERN_DEBUG"%s: wlan enable GPIO value(%d)=%d\n", 
			__func__, OMAP4TAB_WLAN_EN_GPIO, 
			gpio_get_value(OMAP4TAB_WLAN_EN_GPIO));
		
		return;
	}

	if (on) {
		/* wlan power on */
		gpio_set_value(OMAP4TAB_WLAN_EN_GPIO, GPIO_LEVEL_HIGH);
	}
	else {
		/* wlan power off */
		gpio_set_value(OMAP4TAB_WLAN_EN_GPIO, GPIO_LEVEL_LOW);
	}
	
	printk(KERN_DEBUG"%s: wlan enable GPIO value(%d)=%d\n", 
		__func__, OMAP4TAB_WLAN_EN_GPIO, 
		gpio_get_value(OMAP4TAB_WLAN_EN_GPIO));

	/* MMC forced rescan */
#if 1 /* new function */
	omap_hsmmc_force_presence_change_for_wlan(on);
#else /* old function */
	omap_hsmmc_force_presence_change();
#endif	
}

EXPORT_SYMBOL(wlan_setup_power);

static void __init omap_board_init(void)
{
	int status;
	int package = OMAP_PACKAGE_CBS;

	sec_common_init_early();

	if (omap_rev() == OMAP4430_REV_ES1_0)
		package = OMAP_PACKAGE_CBL;
	omap4_mux_init(sec_board_mux_ptr, sec_board_wk_mux_ptr, package);
	sec_mux_init_gpio_out();
	sec_mux_set_wakeup_gpio();

	sec_common_init();

	omap_emif_setup_device_details(&emif_devices, &emif_devices);
	omap4_lpddr2_config();
	omap_init_emif_timings();
	enable_rtc_gpio();
	omap4_audio_conf();
	q1_omap4_gpiowk_setup();
	q1_omap4_touch_init();
	q1_omap4_vib_motor_init();
	i2c4_pullup_set();
	//acc_en_gpio_request();
#ifdef CONFIG_MPU_SENSORS_MPU3050	
	//mpu_layout_set();
#endif
	//yas_layout_set();
	omap4_i2c_init();
	q1_omap4_display_init();
	platform_add_devices(q1_omap4_devices, ARRAY_SIZE(q1_omap4_devices));
	wake_lock_init(&uart_lock, WAKE_LOCK_SUSPEND, "uart_wake_lock");
	omap_serial_init(omap_serial_platform_data);
	omap4_twl6030_hsmmc_init(mmc);
	omap4_mpl_init();
	q1_omap4_usb_gpio_init();
#ifdef CONFIG_INPUT_CM3663
	//cm3663_gpio_request();
#endif
	i2c4_config_read();
#ifdef CONFIG_TIWLAN_SDIO
	config_wlan_mux();
#endif
	omap4_wlan_mux_config();
	omap4_wlan_init();
# if 0
	usb_uhhtll_init(&usbhs_pdata);
	usb_ehci_init();
	usb_ohci_init();
#endif

	/* OMAP4 SDP uses internal transceiver so register nop transceiver */
	usb_nop_xceiv_register();
	usb_musb_init(&musb_board_data);

#ifndef CONFIG_INPUT_GPIO_VOLUME_KEY
	status = omap4_keypad_initialization(&q1_omap4_keypad_data);
	if (status)
		pr_err("Keypad initialization failed: %d\n", status);
#endif

	omap_display_init(&q1_omap4_dss_data);
	enable_board_wakeup_source();

	omap_voltage_register_pmic(&omap_pmic_core, "core");
	omap_voltage_register_pmic(&omap_pmic_mpu, "mpu");
	omap_voltage_register_pmic(&omap_pmic_iva, "iva");
	omap_voltage_init_vc(&vc_config);

	printk("[Board-Q1 init] : GPIO_HW_REV = %x\n", get_hw_rev_gpio());
	samsung_omap4_battery_init();
	
	samsung_omap4_pwr_key_irq_init();
	omap4430univ_sensors_init();
#ifdef DISABLE_I2C2_HW_SPINLOCK
//	m5mo_fref_start();
#endif

#ifdef CONFIG_SWITCH_GPIO
	q1_omap4_headset_init();
#endif
#ifdef CONFIG_EXTRA_DOCK_SPEAKER	
	desk_dock_init();
#endif
	gsd4t_gps_init();

#ifdef CONFIG_EPEN_WACOM_G5SP
	q1_wacom_init();
#endif /* CONFIG_EPEN_WACOM_G5SP */
	
	sec_common_init_post();
}

unsigned int sec_fb_buf_start = 0;
static unsigned int sec_fb_buf_size = 0;

static int __init sec_fb_buf_setup(char *str)
{
	sec_fb_buf_size = memparse(str, &str);

	if (sec_fb_buf_size && (*str == '@')) {
		sec_fb_buf_start = simple_strtoul(++str, &str, 0);
		if (reserve_bootmem(sec_fb_buf_start, sec_fb_buf_size, BOOTMEM_EXCLUSIVE)) {
			pr_err("failed to reserve size %d@0x%X\n",
			sec_fb_buf_size / 1024, sec_fb_buf_start);
			sec_fb_buf_start = 0;
			sec_fb_buf_size = 0;
			goto __return;
		}
	}
__return:
	return 1;
}
__setup("sec_bootfb=", sec_fb_buf_setup);

static void __init omap_board_map_io(void)
{
	omap2_set_globals_443x();
	omap44xx_map_common_io();
}

MACHINE_START(OMAP4_SAMSUNG, "Q1 Samsung board")
    .phys_io = 0x48000000,
    .io_pg_offst = ((0xfa000000) >> 18) & 0xfffc,
    .boot_params = 0x80000100,
    .map_io = omap_board_map_io,
    .init_irq = omap_board_init_irq,
    .init_machine = omap_board_init,
    .timer = &omap_timer,
MACHINE_END
