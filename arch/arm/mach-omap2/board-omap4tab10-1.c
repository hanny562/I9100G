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
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/usb/otg.h>
#include <linux/spi/spi.h>
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
//#include "mux.h"
#include "hsmmc.h"
#include "smartreflex-class3.h"
#include <linux/haptic.h>
#include <plat/opp_twl_tps.h>
#include <linux/fsa9480.h>
#include <linux/gp2a.h>
#include <linux/cm3663.h>
#include <linux/mpu.h>
#include <linux/switch.h>

#include <linux/i2c/twl6030-gpadc.h>
#include "board-4430sdp-wifi.h"

#if ((CONFIG_SAMSUNG_OMAP4_TAB_REV == 5) \
	 || (CONFIG_SAMSUNG_OMAP4_TAB_REV == 6) \
	 || (CONFIG_SAMSUNG_OMAP4_TAB_REV == 7))
#include <linux/lcd.h>
#include <linux/ld9040.h>
#include <linux/pn544.h>
//#include <linux/sii9234.h>
#endif

#if (CONFIG_SAMSUNG_OMAP4_TAB_REV == 3)
#include <linux/atmel_mxt1386.h> /* For mXT1386 Touch */
#elif (CONFIG_SAMSUNG_OMAP4_TAB_REV == 7)
#include <linux/atmel_mxt224E.h>
#endif

#include "mux.h"

#if defined( CONFIG_SAMSUNG_PHONE_SVNET )
#include <linux/phone_svn/modemctl.h>
#include <linux/phone_svn/ipc_hsi.h>
#include <linux/irq.h>
#endif // CONFIG_SAMSUNG_PHONE_SVNET

struct class *sec_class;
EXPORT_SYMBOL(sec_class);

struct device *gps_dev;
EXPORT_SYMBOL(gps_dev);

#define _OMAP_MUX_SETTING
#define SAMSUNG_OMAP4_PWR_GPIO	3
#define OMAP_GPIO_KEY_HOME	176
#if (CONFIG_SAMSUNG_OMAP4_TAB_REV == 5)  && !defined(CONFIG_VIDEO_M5MO)
#define DISABLE_I2C2_HW_SPINLOCK_FOR_REV5
#else
#undef DISABLE_I2C2_HW_SPINLOCK_FOR_REV5
#endif
#if (CONFIG_SAMSUNG_OMAP4_TAB_REV <= 2)
#define SDP4430_LCD_PANEL_BACKLIGHT_ENABLE_GPIO		16
#define SDP4430_MLCD1_PANEL_ENABLE_GPIO			2
#define SDP4430_MLCD2_PANEL_ENABLE_GPIO			40
#define SDP4430_LVDS_nRST_ENABLE_GPIO			17
#elif (CONFIG_SAMSUNG_OMAP4_TAB_REV == 3)
#define OMAP4TAB_MLCD_ON_GPIO				40
#define OMAP4TAB_LED_BACKLIGHT_RESET_GPIO		93
#define OMAP4TAB_LVDS_nSHDN_GPIO			53
#define OMAP4TAB_TOUCH_nINT_GPIO			90
#define OMAP4TAB_TOUCH_nRST_GPIO			89
#define OMAP4TAB_MOTOR_EN1_GPIO				42
#define OMAP4TAB_MOTOR_EN2_GPIO				43
#define OMAP4_TOUCH_EN_GPIO				101
#elif (CONFIG_SAMSUNG_OMAP4_TAB_REV == 4)
#define OMAP4TAB_LCD_EN_GPIO				40
#define OMAP4TAB_MLCD_RST_GPIO				37
#define OMAP4_TOUCH_EN_GPIO				101
#elif ((CONFIG_SAMSUNG_OMAP4_TAB_REV == 5) \
	|| (CONFIG_SAMSUNG_OMAP4_TAB_REV == 6) \
	|| (CONFIG_SAMSUNG_OMAP4_TAB_REV == 7))
#define OMAP4TAB_LCD_EN_GPIO                            40
#define OMAP4TAB_MLCD_RST_GPIO                          35
#define OMAP4_TOUCH_EN_GPIO				54
#define OMAP_GPIO_TSP_INT					46
#endif


/* For Sensors */
#if (CONFIG_SAMSUNG_OMAP4_TAB_REV <= 2)
#define OMAP4TAB_ISL29023_INT_GPIO			52
#define OMAP4TAB_KXSD9_INT_GPIO				94
#define OMAP4TAB_AKM_INT_GPIO				157
#elif (CONFIG_SAMSUNG_OMAP4_TAB_REV == 3)
#define OMAP4TAB_KXSD9_INT_GPIO				94
#define OMAP4TAB_AKM_INT_GPIO				157
#define OMAP4TAB_MPU3050_INT_GPIO			45
#elif (CONFIG_SAMSUNG_OMAP4_TAB_REV == 4)
#define OMAP4TAB_KXSD9_INT_GPIO				122
#define OMAP4TAB_AKM_INT_GPIO				157
#define OMAP4TAB_MPU3050_INT_GPIO			45
#define OMAP4TAB_GP2A_PS_ON				35
#define OMAP4TAB_GP2A_PS_VOUT				33
#elif ((CONFIG_SAMSUNG_OMAP4_TAB_REV == 5) \
	 || (CONFIG_SAMSUNG_OMAP4_TAB_REV == 6) \
	 || (CONFIG_SAMSUNG_OMAP4_TAB_REV == 7))
#define OMAP4TAB_KXSD9_INT_GPIO				122
#define OMAP4TAB_AKM_INT_GPIO				157
#define OMAP4TAB_MPU3050_INT_GPIO			45
#define OMAP4TAB_GP2A_PS_ON				37
#define OMAP4TAB_GP2A_PS_VOUT				33
#endif

#define CONTROL_CORE_PAD0_I2C3_SDA_PAD1_I2C4_SCL	0x4A10012C
#define CONTROL_CORE_PAD0_I2C4_SDA_PAD1_MCSPI1_CLK	0x4A100130

/* For Battery Driver */
#if (CONFIG_SAMSUNG_OMAP4_TAB_REV <= 2)
#define OMAP4430_GPIO_TA_NCONNECTED			151
#define OMAP4430_GPIO_TA_EN				54
#define OMAP4430_GPIO_TA_NSTAT				11
#define OMAP4430_GPIO_TA_NCHG				12
#endif
#if (CONFIG_SAMSUNG_OMAP4_TAB_REV == 3)
#define OMAP4430_GPIO_TA_NCONNECTED			152
#define OMAP4430_GPIO_TA_EN				56
#define OMAP4430_GPIO_TA_NSTAT				11
#define OMAP4430_GPIO_TA_NCHG				153

#endif
#define OMAP4430_GPIO_USBSW_NINT			44



/* For uUSB Switch */
#define OMAP4430_GPIO_JACK_NINT                121

/* For MIPI_HSI */
#if defined( CONFIG_SAMSUNG_PHONE_SVNET )
#define OMAP4430_GPIO_MIPI_HSI_RESET_REQ_N			50
#define OMAP4430_GPIO_MIPI_HSI_CP_ON					36
#define OMAP4430_GPIO_MIPI_HSI_CP_RST				2	//wk2
#define OMAP4430_GPIO_MIPI_HSI_SUSPEND_REQUEST		56
#define OMAP4430_GPIO_MIPI_HSI_PDA_ACTIVE			119
#define OMAP4430_GPIO_MIPI_HSI_PHONE_ACTIVE			120
#endif  //CONFIG_SAMSUNG_PHONE_SVNET

/*PN544 GPIOs*/
#if ((CONFIG_SAMSUNG_OMAP4_TAB_REV == 5) \
	 || (CONFIG_SAMSUNG_OMAP4_TAB_REV == 6) \
	 || (CONFIG_SAMSUNG_OMAP4_TAB_REV == 7))

#define OMAP4430_GPIO_NFC_IRQ		59
#define OMAP4430_GPIO_NFC_FIRM		135
#define OMAP4430_GPIO_NFC_VEN		136
#endif

/*MHL Related GPIOs */
#ifdef CONFIG_VIDEO_MHL_V1
#define OMAP4430_GPIO_MHL_RST           60
#define OMAP4430_GPIO_MHL_SEL          170
#define OMAP4430_GPIO_MHL_INT          175
#define OMAP4430_GPIO_MHL_WAKEUP    64
#define OMAP4430_GPIO_MHL_SCL_18V   99
#define OMAP4430_GPIO_MHL_SDA_18V   98
#endif
/*For FM*/
#ifdef CONFIG_FMRADIO

#define OMAP4430_GPIO_FM_INT    40
#define OMAP4430_GPIO_FM_RST    42
#endif

/* For Audio */
#define OMAP4TAB_MICBIAS_EN_GPIO 	48
#define OMAP4TAB_MICBIAS_EN2_GPIO 	49
#define OMAP4TAB_DET_35_GPIO 		0	/* GPIO_WK0 */
#define OMAP4TAB_EAR_SEND_END_GPIO 	94

/* For GPS */
#if ((CONFIG_SAMSUNG_OMAP4_TAB_REV == 6) \
         || (CONFIG_SAMSUNG_OMAP4_TAB_REV == 7))
#define OMAP_GPIO_AP_AGPS_TSYNC	134
#define OMAP_GPIO_GPS_LNA_EN			135
#define OMAP_GPIO_GPS_nRST			136
#define OMAP_GPIO_GPS_PWR_EN		137
#endif

#if ((CONFIG_SAMSUNG_OMAP4_TAB_REV == 5) \
	 || (CONFIG_SAMSUNG_OMAP4_TAB_REV == 6) \
	 || (CONFIG_SAMSUNG_OMAP4_TAB_REV == 7))

extern void set_offmode_padconfig(void);
#endif

#if (CONFIG_SAMSUNG_OMAP4_TAB_REV == 6 || CONFIG_SAMSUNG_OMAP4_TAB_REV == 7)
extern struct ld9040_panel_data matchbox_panel_data;
#endif
static void omap4_usb_cb(u8 attached);
static void omap4_charger_cb(u8 attached);
static void omap4_jig_cb(u8 attached);
static void omap4_fsa9480_reset_cb(void);

#ifdef CONFIG_VIDEO_MHL_V1
static void omap4_mhl_cb(u8 attached);
#endif

static struct wake_lock uart_lock;
#define TWL6030_RTC_GPIO 6
#define BLUETOOTH_UART UART2
#define CONSOLE_UART UART3

#if (CONFIG_SAMSUNG_OMAP4_TAB_REV <= 2)
static unsigned backlight_gpio;
static unsigned mlcd1_enable_gpio;
static unsigned mlcd2_enable_gpio;
static unsigned lvds_nshdn_gpio;	
#elif (CONFIG_SAMSUNG_OMAP4_TAB_REV == 3)
static unsigned lvds_nshdn_gpio;
static unsigned mlcd_on_gpio;
static unsigned led_backlight_reset_gpio;
static unsigned touch_nrst_gpio;
#elif ((CONFIG_SAMSUNG_OMAP4_TAB_REV == 4) \
	 || (CONFIG_SAMSUNG_OMAP4_TAB_REV == 5) \
	 || (CONFIG_SAMSUNG_OMAP4_TAB_REV == 6) \
	 || (CONFIG_SAMSUNG_OMAP4_TAB_REV == 7))
static unsigned lcd_en_gpio;
static unsigned mlcd_rst_gpio;
#else
#endif

static int lcd_enabled;

void omap4430univ_sensors_init(void);

static struct platform_device omap4tab_hdmi_audio_device = {
	.name		= "hdmi-dai",
	.id		= -1,
};

#ifdef CONFIG_SWITCH_GPIO
void ear_mic_bias_cb(bool value)
{
	printk(KERN_ERR "ear_mic_bias_cb %d\n", value);
	if(value)
		gpio_set_value(OMAP4TAB_MICBIAS_EN2_GPIO, 1);
	else
		gpio_set_value(OMAP4TAB_MICBIAS_EN2_GPIO, 0);
}
static struct gpio_switch_platform_data headset_switch_data = {
	.name = "h2w",
	.gpio = OMAP4TAB_DET_35_GPIO,	/* Omap3430 GPIO_27 For samsung zeus */
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
	.gpio = OMAP4TAB_EAR_SEND_END_GPIO,	/* Omap3430 GPIO_27 For samsung zeus */
};
static struct platform_device board_ear_key_device = {
	.name = "ear_key_device",
	.id = -1,
	.num_resources = 1,
	.resource = &board_ear_key_resource,
	.dev = {
		.platform_data = &ear_key_switch_data,
	} 
};
#endif
#ifdef CONFIG_EAR_MIC_ADC
static struct resource omap4tab_ear_mic_adc_resource = {
	.start  = 0,
	.end    = 0,
	.flags  = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL,
};

static struct platform_device omap4tab_ear_mic_adc_device = {
	.name           = "ear_mic_adc",
	.id             = -1,
	.num_resources  = 1,
	.resource       = &omap4tab_ear_mic_adc_resource,
};
#endif

#ifdef CONFIG_SWITCH_SIO
/* SIO Switch */
struct platform_device sec_sio_switch = {
	.name = "switch-sio",
	.id = -1,
};
#endif


static int omap4tab_keymap[] = {
	/* TODO: The row and column coded here are based on what is detected by
	 * OMAP4. It is different from the one shown in schematic. Need the
	 * identify the root cause of the problem */
	/* row, col, key */
#if (CONFIG_SAMSUNG_OMAP4_TAB_REV == 2)
        KEY(3, 3, KEY_HOME),            //HOME key
        KEY(3, 4, KEY_VOLUMEUP),        //Volume up
        KEY(4, 3, KEY_F1),              //Spare key
        KEY(4, 4, KEY_VOLUMEDOWN),      //volume down
        KEY(5, 3, KEY_ENTER),           //Search Key
        KEY(5, 4, KEY_BACK),
#endif
#if (CONFIG_SAMSUNG_OMAP4_TAB_REV == 3)
        KEY(4, 4, KEY_HOME),            //HOME key
        KEY(5, 4, KEY_F1),              //Spare key
        KEY(3, 4, KEY_BACK),
        KEY(5, 3, KEY_ENTER),           //Search Key
        KEY(4, 0, KEY_VOLUMEUP),        //Volume up
        KEY(3, 3, KEY_VOLUMEDOWN),      //volume down
#endif
#if ((CONFIG_SAMSUNG_OMAP4_TAB_REV == 4) \
	 || (CONFIG_SAMSUNG_OMAP4_TAB_REV == 5)\
	 || (CONFIG_SAMSUNG_OMAP4_TAB_REV == 6) \
	 || (CONFIG_SAMSUNG_OMAP4_TAB_REV == 7))

        KEY(2, 1, KEY_VOLUMEUP),        //Volume up
        KEY(1, 1, KEY_VOLUMEDOWN),      //volume down
#endif
	0,
};

/* For uUSB Switch */
static struct fsa9480_platform_data omap4_fsa9480_pdata = {
       .intb_gpio      = OMAP4430_GPIO_JACK_NINT,
       .usb_cb         = omap4_usb_cb,
       .uart_cb        = NULL,
       .charger_cb     = omap4_charger_cb,
       .jig_cb         = omap4_jig_cb,
       .reset_cb       = omap4_fsa9480_reset_cb,
	#ifdef CONFIG_VIDEO_MHL_V1
	   .mhl_cb         = omap4_mhl_cb,
	#endif
};


#define SENSOR_MPU_NAME "mpu3050"

#if (CONFIG_SAMSUNG_OMAP4_TAB_REV == 7)
static struct mpu3050_platform_data mpu_data = {
	.int_config  = 0x12,
	.orientation = {   0, 1,  0, 
			  -1,  0,  0, 
			   0,  0, 1 },
	/* accel */
	.accel = {
	.get_slave_descr = get_accel_slave_descr,
	.adapt_num   = 4,
	.bus         = EXT_SLAVE_BUS_SECONDARY,
	.address     = 0x19,
	.orientation = {   -1,  0,  0, 
			   0, -1,  0, 
			   0,  0,  1 },
	 },
	/* compass */
	.compass = {
	.get_slave_descr = yas530_get_slave_descr,
	.adapt_num   = 4,
	.bus         = EXT_SLAVE_BUS_PRIMARY,
	.address     = 0x2E,
	.orientation = {  1,  0,  0, 
			  0,  1,  0, 
			  0,  0,  1 },
	 },
};
#else
static struct mpu3050_platform_data mpu_data = {
	.int_config  = 0x12,
	.orientation = {   0, -1,  0, 
			  -1,  0,  0, 
			   0,  0, -1 },
	/* accel */
	.accel = {
	.get_slave_descr = get_accel_slave_descr,
	.adapt_num   = 4,
	.bus         = EXT_SLAVE_BUS_SECONDARY,
	.address     = 0x18,
	.orientation = {   1,  0,  0, 
			   0, -1,  0, 
			   0,  0,  -1 },
	 },
	/* compass */
	.compass = {
	.get_slave_descr = yas530_get_slave_descr,
	.adapt_num   = 4,
	.bus         = EXT_SLAVE_BUS_PRIMARY,
	.address     = 0x2E,
	.orientation = {  1,  0,  0, 
			  0,  1,  0, 
			  0,  0,  1 },
	 },
};

#endif

#ifdef CONFIG_OPTICAL_GP2A

/* For gp2a light/proximity sensor */

static int gp2a_power(bool on)
{
	/* this controls the power supply rail to the gp2a IC */
	gpio_direction_output(OMAP4TAB_GP2A_PS_ON, on);
	return 0;
}

static struct twl6030_gpadc_request conv_request = {
        .channels = (0x1 << 3),
        .do_avg = 0,
	.method = TWL6030_GPADC_SW2,
	.type = 0,
	.active = 0,
	.result_pending = 0,
	.func_cb = NULL,

};

static int gp2a_light_adc_value(void)
{
	// Ambient Light sensing
        twl6030_gpadc_conversion(&conv_request);
	return (conv_request.rbuf[3]);

}

static struct gp2a_platform_data gp2a_pdata = {
	.power = gp2a_power,
	.p_out = OMAP4TAB_GP2A_PS_VOUT,
	.light_adc_value = gp2a_light_adc_value
};
#endif

#ifdef CONFIG_INPUT_CM3663
/* For gp2a light/proximity sensor */

static int cm3663_power(bool on)
{
	/* this controls the power supply rail to the gp2a IC */
	gpio_direction_output(OMAP4TAB_GP2A_PS_ON, on);
	return 0;
}


static struct cm3663_platform_data cm3663_pdata = {
	.proximity_power = cm3663_power,
	.irq = OMAP4TAB_GP2A_PS_VOUT,
};

void cm3663_gpio_request(void)
{
	int ret = -ENODEV;
	ret = gpio_request(OMAP4TAB_GP2A_PS_ON, "gpio_ps_on");
	if (ret < 0) {
		pr_err("%s: gpio %d request failed (%d)\n",
			__func__, OMAP4TAB_GP2A_PS_ON, ret);
		return ret;
	}
	ret = gpio_direction_input(OMAP4TAB_GP2A_PS_ON);
	if (ret < 0) {
		pr_err("%s: failed to set gpio %d as input (%d)\n",
			__func__, OMAP4TAB_GP2A_PS_ON, ret);
		return ret;
	}
}
#endif

/*atmel_mxt224E*/
#if (CONFIG_SAMSUNG_OMAP4_TAB_REV == 7)
static struct mxt_callbacks *inform_charger_callbacks;

static void t1_touch_init_hw(struct mxt_platform_data *pdata)
{
	int error;
	pr_info("(%s,%d)\n", __func__, __LINE__);

	/* TSP INT GPIO initialize */
	error = gpio_request(pdata->irq_gpio, "tsp_int");
	if (error != 0) {
		pr_err("tsp_int request FAIL error = %d", error);
	}
	gpio_direction_input(pdata->irq_gpio);
}

static void t1_touch_exit_hw(struct mxt_platform_data *pdata)
{
	pr_info("(%s,%d)\n", __func__, __LINE__);
	gpio_free(pdata->irq_gpio);
}

static void t1_touch_suspend_hw(struct mxt_platform_data *pdata)
{
	pr_info("(%s,%d)\n", __func__, __LINE__);
	/* first power off (off sequence: avdd -> vdd) */
	gpio_direction_output(OMAP4_TOUCH_EN_GPIO, 0);
	pr_info("[TSP] Power Off!!");
}

static void t1_touch_resume_hw(struct mxt_platform_data *pdata)
{
	pr_info("(%s,%d)\n", __func__, __LINE__);
	gpio_direction_output(OMAP4_TOUCH_EN_GPIO, 1);
	msleep(100);
	pr_info("[TSP] Power On!!");
}

void t1_inform_charger_connection(int mode)
{
	if (inform_charger_callbacks &&
				inform_charger_callbacks->inform_charger)
		inform_charger_callbacks->inform_charger(inform_charger_callbacks, mode);
};
EXPORT_SYMBOL(t1_inform_charger_connection);

static void t1_register_touch_callbacks(struct mxt_callbacks *cb)
{
	 inform_charger_callbacks = cb;
}

static struct mxt_platform_data t1_mxt224E_platform_data = {
	.platform_name = "mxt224E TSP",
	.numtouch = 10,
	.max_x = 479,
	.max_y = 800,
	.init_platform_hw = t1_touch_init_hw,
	.exit_platform_hw = t1_touch_exit_hw,
	.suspend_platform_hw = t1_touch_suspend_hw,
	.resume_platform_hw = t1_touch_resume_hw,
	.register_cb = t1_register_touch_callbacks,
	.irq_gpio = OMAP_GPIO_TSP_INT,
};
#endif

static void omap4_usb_cb(u8 attached)
{
       /* TODO: need to implement this once uUSB charging available in HW */
       printk("\nBoard file [FSA9480]: USB Callback \n");
#if (CONFIG_SAMSUNG_OMAP4_TAB_REV == 7)
       t1_inform_charger_connection(attached);
#endif
}

static void omap4_charger_cb(u8 attached)
{
       /* TODO: need to implement this once uUSB charging available in HW */
       printk("\nBoard file [FSA9480]: Charger Callback \n");
#if (CONFIG_SAMSUNG_OMAP4_TAB_REV == 7)
       t1_inform_charger_connection(attached);
#endif
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
#ifdef CONFIG_VIDEO_MHL_V1
extern void MHL_On(bool on);

static bool fsa9480_is_mhl_attached(void)
{
        int val;

        gpio_request(OMAP4430_GPIO_MHL_SEL, "MHL_SEL");
        val = gpio_get_value(OMAP4430_GPIO_MHL_SEL);
        gpio_free(OMAP4430_GPIO_MHL_SEL);

        return !!val;
}

static void omap4_mhl_cb(u8 attached)
{
    /*TODO */
       printk("\nBoard file [FSA9480]: MHL Callback\n");
       gpio_request(OMAP4430_GPIO_MHL_SEL, "MHL_SEL");
       gpio_direction_output(OMAP4430_GPIO_MHL_SEL,1);
       gpio_free(OMAP4430_GPIO_MHL_SEL);
       MHL_On(attached);
}


#endif

static struct matrix_keymap_data omap4tab_keymap_data = {
	.keymap			= omap4tab_keymap,
	.keymap_size		= ARRAY_SIZE(omap4tab_keymap),
};

static struct omap4_keypad_platform_data omap4tab_keypad_data = {
	.keymap_data		= &omap4tab_keymap_data,
	.rows			= 8,
	.cols			= 8,
	.rep                    = 0,
};

static struct platform_device omap4tab_keypad_led = {
	.name	=	"keypad_led",
	.id	=	-1,
	.dev	= {
		.platform_data = NULL,
	},
};

void keyboard_mux_init(void)
{
#if (CONFIG_SAMSUNG_OMAP4_TAB_REV == 4 || \
	CONFIG_SAMSUNG_OMAP4_TAB_REV == 5 || \
	CONFIG_SAMSUNG_OMAP4_TAB_REV == 6 || \
	CONFIG_SAMSUNG_OMAP4_TAB_REV == 7)
        omap_mux_init_signal("kpd_col1.kpd_col1",
                                OMAP_WAKEUP_EN | OMAP_MUX_MODE0);
        omap_mux_init_signal("kpd_row1.kpd_row1",
                                OMAP_PULL_ENA | OMAP_PULL_UP |
                                OMAP_WAKEUP_EN | OMAP_MUX_MODE0 |
                                OMAP_INPUT_EN);
        omap_mux_init_signal("kpd_row2.kpd_row2",
                                OMAP_PULL_ENA | OMAP_PULL_UP |
                                OMAP_WAKEUP_EN | OMAP_MUX_MODE0 |
                                OMAP_INPUT_EN);
#endif
}


static int omap4tab_panel_enable_hdmi(struct omap_dss_device *dssdev)
{
	gpio_request(HDMI_GPIO_60 , "hdmi_gpio_60");
	gpio_request(HDMI_GPIO_41 , "hdmi_gpio_41");
	gpio_direction_output(HDMI_GPIO_60, 0);
	gpio_direction_output(HDMI_GPIO_41, 0);
	gpio_set_value(HDMI_GPIO_60, 1);
	gpio_set_value(HDMI_GPIO_41, 1);
	gpio_set_value(HDMI_GPIO_60, 0);
	gpio_set_value(HDMI_GPIO_41, 0);
	gpio_set_value(HDMI_GPIO_60, 1);
	gpio_set_value(HDMI_GPIO_41, 1);
	return 0;
}

static void omap4tab_panel_disable_hdmi(struct omap_dss_device *dssdev)
{
	gpio_set_value(HDMI_GPIO_60, 1);
	gpio_set_value(HDMI_GPIO_41, 1);
}

static __attribute__ ((unused)) void __init omap4tab_hdmi_init(void)
{
	return;
}
#if (CONFIG_SAMSUNG_OMAP4_TAB_REV <= 2)
static int ltn121at_panel_enable_lcd(struct omap_dss_device *dssdev)
{

        gpio_direction_output(mlcd1_enable_gpio, 1);
        gpio_direction_output(mlcd2_enable_gpio, 1);

        gpio_direction_output(lvds_nshdn_gpio, 1);
	gpio_direction_output(backlight_gpio, 1);

        lcd_enabled = 1;
        return 0;
}

static void ltn121at_panel_disable_lcd(struct omap_dss_device *dssdev)
{
        lcd_enabled = 0;

        gpio_direction_output(mlcd1_enable_gpio, 0);
        gpio_direction_output(mlcd2_enable_gpio, 0);
        gpio_direction_output(backlight_gpio, 0);
}

static struct omap_dss_device omap4tab_lcd_device = {
        .name                   = "lcd",
        .driver_name            = "samsung_ltn121at_panel",
        .type                   = OMAP_DISPLAY_TYPE_DPI,
        .phy.dpi.data_lines     = 24,
        .platform_enable        = ltn121at_panel_enable_lcd,
        .platform_disable       = ltn121at_panel_disable_lcd,
	.channel                        = OMAP_DSS_CHANNEL_LCD2,
};
#elif (CONFIG_SAMSUNG_OMAP4_TAB_REV == 3)
static int D10E50T6332_panel_enable_lcd(struct omap_dss_device *dssdev)
{

        gpio_direction_output(mlcd_on_gpio, 1);
        gpio_direction_output(lvds_nshdn_gpio, 1);
	
        lcd_enabled = 1;
        return 0;
}

static void D10E50T6332_panel_disable_lcd(struct omap_dss_device *dssdev)
{
        lcd_enabled = 0;

        gpio_direction_output(mlcd_on_gpio, 0);
        gpio_direction_output(lvds_nshdn_gpio, 0);
	return;

 }
 
static struct omap_dss_device omap4tab_lcd_device = {
        .name                   = "lcd",
        .driver_name            = "samsung_D10E50T6332_panel",
        .type                   = OMAP_DISPLAY_TYPE_DPI,
        .phy.dpi.data_lines     = 24,
        .platform_enable        = D10E50T6332_panel_enable_lcd,
        .platform_disable       = D10E50T6332_panel_disable_lcd,
	.channel                = OMAP_DSS_CHANNEL_LCD2,
};
/* maXTouch TSP platform data */
static void mxt1386_ts_init_hw()
{

	int r;

	touch_nrst_gpio	= OMAP4TAB_TOUCH_nRST_GPIO;

	r = gpio_request(touch_nrst_gpio, "TOUCH ENable TOUCH_nRST GPIO");
	if (r) {
			printk(KERN_ERR "failed to get TOUCH_nRST GPIO\n");
			goto err0;
	}

	gpio_direction_output(touch_nrst_gpio, 0);
	udelay(1);
	gpio_direction_output(touch_nrst_gpio, 1);
	return;

	err0:
	gpio_free(touch_nrst_gpio);
	return;

}

static void mxt1386_ts_exit_hw()
{

	gpio_direction_output(touch_nrst_gpio, 0);
	gpio_free(touch_nrst_gpio);
	return;
}

static struct mxt_platform_data mxt1386_ts_platform_data = {
	// Width: 1280, Height: 800
	.max_x = 1279,
	.max_y = 799,
	.init_platform_hw = mxt1386_ts_init_hw,
	.exit_platform_hw = mxt1386_ts_exit_hw,
};
#elif (CONFIG_SAMSUNG_OMAP4_TAB_REV == 4 || CONFIG_SAMSUNG_OMAP4_TAB_REV == 5)
static int ams353_panel_poweron()
{
	gpio_direction_output(mlcd_rst_gpio, 1);
	/*Power on LCD */
	gpio_direction_output(lcd_en_gpio, 1);
	mdelay(25);
	gpio_direction_output(mlcd_rst_gpio, 0);
	mdelay(1);
	gpio_direction_output(mlcd_rst_gpio, 1);
	mdelay(1);

	return 0;
}
static int ams353_panel_poweroff()
{
        gpio_direction_output(lcd_en_gpio, 0);
        gpio_direction_output(mlcd_rst_gpio, 0);	
	return 0;
}
static int ams353_panel_enable_lcd(struct omap_dss_device *dssdev)
{

	ams353_panel_poweron();
        lcd_enabled = 1;
        return 0;
}

static void ams353_panel_disable_lcd(struct omap_dss_device *dssdev)
{
       	lcd_enabled = 0;

       	ams353_panel_poweroff();
       	return;

 }

static struct omap_dss_device omap4tab_lcd_device = {
        .name                   = "lcd",
        .driver_name            = "ams353_panel",
        .type                   = OMAP_DISPLAY_TYPE_DPI,
        .phy.dpi.data_lines     = 24,
        .platform_enable        = ams353_panel_enable_lcd,
        .platform_disable       = ams353_panel_disable_lcd,
       .channel                = OMAP_DSS_CHANNEL_LCD2,
};

static struct spi_board_info omap4tab_spi_board_info[] __initdata = {
        {
                .modalias               = "ams353_disp_spi",
                .bus_num                = 4,
                .chip_select            = 0,
                .max_speed_hz           = 375000,
        },
};
#elif ((CONFIG_SAMSUNG_OMAP4_TAB_REV == 6))
static int ld9040_panel_enable_lcd(struct omap_dss_device *dssdev)
{
        return 0;
}

static void ld9040_panel_disable_lcd(struct omap_dss_device *dssdev)
{

       	return;

 }

static struct omap_dss_device omap4tab_lcd_device = {
        .name                   = "lcd",
        .driver_name            = "ld9040_panel",
        .type                   = OMAP_DISPLAY_TYPE_DPI,
        .phy.dpi.data_lines     = 24,
        .platform_enable        = ld9040_panel_enable_lcd,
        .platform_disable       = ld9040_panel_disable_lcd,
       .channel                = OMAP_DSS_CHANNEL_LCD2,
};

static int lcd_power_on(struct lcd_device *ld, int enable)
{
#if 0

    int lcd_enable_gpio = -1;
    int err;
    lcd_enable_gpio = OMAP4TAB_LCD_EN_GPIO;
 
	printk("[SHANKAR] %s [%d] entry \n", __func__, __LINE__);

    err = gpio_request(lcd_enable_gpio, "LCD_EN");
    if (err) {
        printk(KERN_ERR "failed to request MLCD_RST for "
                "lcd reset control\n");
        return err;
    }

    if (enable) {
		gpio_direction_output(lcd_enable_gpio, 1);
	} else {
    	gpio_direction_output(lcd_enable_gpio, 0);
	}

    gpio_free(lcd_enable_gpio);
#endif	
	if (enable) {
		gpio_set_value(mlcd_rst_gpio, 1);
		gpio_set_value(lcd_en_gpio, 1);
	} else {
    	gpio_set_value(lcd_en_gpio, 0);
		gpio_set_value(mlcd_rst_gpio, 0);
	}
    return 1;
}
static int reset_lcd(struct lcd_device *ld)
{
#if 0

    int reset_gpio = -1;
    int err;
    reset_gpio = OMAP4TAB_MLCD_RST_GPIO;
	
	printk("[SHANKAR] %s [%d] entry \n", __func__, __LINE__);

    err = gpio_request(reset_gpio, "MLCD_RST");
    if (err) {
        printk(KERN_ERR "failed to request MLCD_RST for "
                "lcd reset control\n");
        return err;
    }

    mdelay(10);
    gpio_direction_output(reset_gpio, 0);
    mdelay(10);
    gpio_direction_output(reset_gpio, 1);

    gpio_free(reset_gpio);
#endif	
    mdelay(10);
    gpio_set_value(mlcd_rst_gpio, 0);
    mdelay(10);
    gpio_set_value(mlcd_rst_gpio, 1);
	
    return 1;
}

static int lcd_gpio_cfg_earlysuspend(struct lcd_device *ld)
{
    int reset_gpio = -1;
    int err;
#if 0
    reset_gpio = S5PV310_GPY4(5);

    err = gpio_request(reset_gpio, "MLCD_RST");
    if (err) {
        printk(KERN_ERR "failed to request MLCD_RST for "
                "lcd reset control\n");
        return err;
    }

    mdelay(10);
    gpio_direction_output(reset_gpio, 0);

    gpio_free(reset_gpio);
#endif
    return 0;
}
static int lcd_gpio_cfg_lateresume(struct lcd_device *ld)
{
#if 0	
    /* MLCD_RST */
    s3c_gpio_cfgpin(S5PV310_GPY4(5), S3C_GPIO_OUTPUT);
    s3c_gpio_setpull(S5PV310_GPY4(5), S3C_GPIO_PULL_NONE);

    /* LCD_nCS */
    s3c_gpio_cfgpin(S5PV310_GPY4(3), S3C_GPIO_OUTPUT);
    s3c_gpio_setpull(S5PV310_GPY4(3), S3C_GPIO_PULL_NONE);
    /* LCD_SCLK */
    s3c_gpio_cfgpin(S5PV310_GPY3(1), S3C_GPIO_OUTPUT);
    s3c_gpio_setpull(S5PV310_GPY3(1), S3C_GPIO_PULL_NONE);
    /* LCD_SDI */
    s3c_gpio_cfgpin(S5PV310_GPY3(3), S3C_GPIO_OUTPUT);
    s3c_gpio_setpull(S5PV310_GPY3(3), S3C_GPIO_PULL_NONE);
#endif
    return 0;
}

/*   : Fix me regarding ADC kernel Panic
static void lcd_register_callback(void* function)
{
    sec_battery_cbs.lcd_set_adc_value = function;
}
*/


static struct lcd_platform_data ld9040_platform_data = {
    .reset          = reset_lcd,
    .power_on       = lcd_power_on,
    .gpio_cfg_earlysuspend  = lcd_gpio_cfg_earlysuspend,
    .gpio_cfg_lateresume    = lcd_gpio_cfg_lateresume,
/*   : Fix me regarding ADC kernel Panic
    .register_cb    = lcd_register_callback,
*/
    /* it indicates whether lcd panel is enabled from u-boot. */
    .lcd_enabled        = 0,
    .reset_delay        = 20,   /* 20ms */
    .power_on_delay     = 20,   /* 20ms */
    .power_off_delay    = 200,  /* 200ms */
    .sleep_in_delay     = 160,
    .pdata          = &matchbox_panel_data,
};

static struct spi_board_info omap4tab_spi_board_info[] __initdata = {
        {
                .modalias               = "ld9040",
                .bus_num                = 4,
                .chip_select            = 0,
                .max_speed_hz           = 375000,
                .platform_data		= (void *)&ld9040_platform_data,
        },
};
#elif ((CONFIG_SAMSUNG_OMAP4_TAB_REV == 7))
static int ld9040_panel_enable_lcd(struct omap_dss_device *dssdev)
{
        struct regulator *regulator;

        regulator = regulator_get(&dssdev->dev, "vusim");
        if (IS_ERR(regulator)) {
               printk("[SHANKAR] %s [%d] failed to get VUSIM regulator. \n", __func__, __LINE__);
                return 0;
        }
        regulator_enable(regulator);
        regulator_put(regulator);
        return 0;
}

static void ld9040_panel_disable_lcd(struct omap_dss_device *dssdev)
{
        struct regulator *regulator;

	regulator = regulator_get(&dssdev->dev, "vusim");
        if (IS_ERR(regulator)) {
               printk("[SHANKAR] %s [%d] failed to get VUSIM regulator. \n", __func__, __LINE__);
                return ;
        }
        if (regulator_is_enabled(regulator))
                regulator_force_disable(regulator);
        regulator_put(regulator);

       	return;

 }

static struct omap_dss_device omap4tab_lcd_device = {
        .name                   = "lcd",
        .driver_name            = "ld9040_panel",
        .type                   = OMAP_DISPLAY_TYPE_DPI,
        .phy.dpi.data_lines     = 24,
        .platform_enable        = ld9040_panel_enable_lcd,
        .platform_disable       = ld9040_panel_disable_lcd,
       .channel                = OMAP_DSS_CHANNEL_LCD2,
};

static int lcd_power_on(struct lcd_device *ld, int enable)
{
        return 1;

}
static int reset_lcd(struct lcd_device *ld)
{
    mdelay(10);
    gpio_set_value(mlcd_rst_gpio, 0);
    mdelay(10);
    gpio_set_value(mlcd_rst_gpio, 1);
	
	printk("[SHANKAR] %s [%d] exit \n", __func__, __LINE__);
    return 1;
}

static int lcd_gpio_cfg_earlysuspend(struct lcd_device *ld)
{
    int reset_gpio = -1;
    int err;
#if 0
    reset_gpio = S5PV310_GPY4(5);

    err = gpio_request(reset_gpio, "MLCD_RST");
    if (err) {
        printk(KERN_ERR "failed to request MLCD_RST for "
                "lcd reset control\n");
        return err;
    }

    mdelay(10);
    gpio_direction_output(reset_gpio, 0);

    gpio_free(reset_gpio);
#endif
    return 0;
}
static int lcd_gpio_cfg_lateresume(struct lcd_device *ld)
{
#if 0	
    /* MLCD_RST */
    s3c_gpio_cfgpin(S5PV310_GPY4(5), S3C_GPIO_OUTPUT);
    s3c_gpio_setpull(S5PV310_GPY4(5), S3C_GPIO_PULL_NONE);

    /* LCD_nCS */
    s3c_gpio_cfgpin(S5PV310_GPY4(3), S3C_GPIO_OUTPUT);
    s3c_gpio_setpull(S5PV310_GPY4(3), S3C_GPIO_PULL_NONE);
    /* LCD_SCLK */
    s3c_gpio_cfgpin(S5PV310_GPY3(1), S3C_GPIO_OUTPUT);
    s3c_gpio_setpull(S5PV310_GPY3(1), S3C_GPIO_PULL_NONE);
    /* LCD_SDI */
    s3c_gpio_cfgpin(S5PV310_GPY3(3), S3C_GPIO_OUTPUT);
    s3c_gpio_setpull(S5PV310_GPY3(3), S3C_GPIO_PULL_NONE);
#endif
    return 0;
}

/*   : Fix me regarding ADC kernel Panic
static void lcd_register_callback(void* function)
{
    sec_battery_cbs.lcd_set_adc_value = function;
}
*/


static struct lcd_platform_data ld9040_platform_data = {
    .reset          = reset_lcd,
    .power_on       = lcd_power_on,
    .gpio_cfg_earlysuspend  = lcd_gpio_cfg_earlysuspend,
    .gpio_cfg_lateresume    = lcd_gpio_cfg_lateresume,
/*   : Fix me regarding ADC kernel Panic
    .register_cb    = lcd_register_callback,
*/
    /* it indicates whether lcd panel is enabled from u-boot. */
    .lcd_enabled        = 0,
    .reset_delay        = 20,   /* 20ms */
    .power_on_delay     = 20,   /* 20ms */
    .power_off_delay    = 200,  /* 200ms */
    .sleep_in_delay     = 160,
    .pdata          = &matchbox_panel_data,
};

static struct spi_board_info omap4tab_spi_board_info[] __initdata = {
        {
                .modalias               = "ld9040",
                .bus_num                = 4,
                .chip_select            = 0,
                .max_speed_hz           = 375000,
                .platform_data		= (void *)&ld9040_platform_data,
        },
};

#else
/*TBD */
#endif

static struct omap_dss_device omap4tab_hdmi_device = {
	.name = "hdmi",
	.driver_name = "hdmi_panel",
	.type = OMAP_DISPLAY_TYPE_HDMI,
	.phy.dpi.data_lines = 24,
	.platform_enable = omap4tab_panel_enable_hdmi,
	.platform_disable = omap4tab_panel_disable_hdmi,
	.channel = OMAP_DSS_CHANNEL_DIGIT,
};



static struct omap_dss_device *omap4tab_dss_devices[] = {
	&omap4tab_lcd_device,
#ifdef CONFIG_OMAP2_DSS_HDMI
	&omap4tab_hdmi_device,
#endif
};

static struct omap_dss_board_info omap4tab_dss_data = {
	.num_devices	=	ARRAY_SIZE(omap4tab_dss_devices),
	.devices	=	omap4tab_dss_devices,
	.default_device	=	&omap4tab_lcd_device,
};

/* wl128x BT, FM, GPS connectivity chip */
static int gpios[] = {103, -1, -1};
static struct platform_device wl128x_device = {
	.name		= "kim",
	.id		= -1,
	.dev.platform_data = &gpios,
};

static struct resource samsung_omap4_pwr_key_rsc[] = {
	[0] = {
		// PWRON KEY
		.start	= 0,
		.end	= 0,
		.flags	= IORESOURCE_IRQ|IORESOURCE_IRQ_LOWLEVEL,
	},
	[1] = {
		// HOME KEY
		.start = 0,
		.end = 0,
		.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE | IORESOURCE_IRQ_LOWEDGE,
	},
};

static struct platform_device samsung_omap4_pwr_device = {
	.name		= "samsung_omap4_pwr_key",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(samsung_omap4_pwr_key_rsc),
	.resource	= &samsung_omap4_pwr_key_rsc,
};

static inline void __init samsung_omap4_pwr_key_irq_init(void)
{
	/*TODO: Do a gpio_to_irq() and gpio_request() followed by gpio_direction_input() */
	samsung_omap4_pwr_key_rsc[0].start = gpio_to_irq(SAMSUNG_OMAP4_PWR_GPIO);
	printk(KERN_ERR "Got GPIO to IRQ number %d\n", samsung_omap4_pwr_key_rsc[0].start);
	if (gpio_request(SAMSUNG_OMAP4_PWR_GPIO, "pwr_key_irq") < 0) {
		printk("\n Failed to request SAMSUNG_OMAP4_PWR_GPIO %d\n", SAMSUNG_OMAP4_PWR_GPIO);
		samsung_omap4_pwr_key_rsc[0].start = 0;
		return;
	}
	gpio_direction_input(SAMSUNG_OMAP4_PWR_GPIO);
	printk("Configured SAMSUNG_OMAP4_PWR_GPIO\n");

	samsung_omap4_pwr_key_rsc[1].start = gpio_to_irq(OMAP_GPIO_KEY_HOME);
	if (gpio_request(OMAP_GPIO_KEY_HOME, "home_key_irq") < 0) {
		printk(KERN_ERR "\n FAILED TO REQUEST GPIO %d for HOME KEY IRQ \n", OMAP_GPIO_KEY_HOME);
		samsung_omap4_pwr_key_rsc[1].start = 0;
		return;
	}
	gpio_direction_input(OMAP_GPIO_KEY_HOME);
}

static struct resource samsung_omap4_gyro_lp530al_rsc[3] = {
	{
		.start	= 0,
		.end	= 0,
		.flags	= IORESOURCE_IRQ|IORESOURCE_IRQ_LOWLEVEL,
	},
	{
		.start	= 1,
		.end	= 0,
		.flags	= IORESOURCE_IRQ|IORESOURCE_IRQ_LOWLEVEL,
	},
	{
		.start	= 0x4A31E040,
		.end	= 0x4A31E044,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device samsung_omap4_gyro_lp530al_device = {
	.name	= "lp530al_gyro_sensor",
	.id	= -1,
	.num_resources	= 3,
	.resource	= &samsung_omap4_gyro_lp530al_rsc[0],
};

//haptic device
static struct haptic_platform_data isa1000_platform_data[] = {
	[0] = { /* Primary Controller */
		.gpio = 42,
		.name = "isa1000",
		/*.ldo_level = ISA1200_LDOADJ_24V,*/
		.pwm_timer = 0,
	},
	[1] = { /* Secondary Controller */
		.gpio = 43,
		.name = "isa1000",
		/*.ldo_level = ISA1200_LDOADJ_36V,*/
		.pwm_timer = 1,
	},
};

static struct platform_device pwm_haptic_device = {
	.name	= "samsung_pwm_haptic",
	.id	= -1,
	.dev.platform_data = &isa1000_platform_data[0],
};


/* pwm enabled device */
static const struct platform_device_id pwm_id_table[] = {
	{ "omap44xx-pwm", 0 },
};

static struct platform_device pwmbacklight_device = {
	.name		= "omap44xx-pwm",
	.id		= 190,
	.dev.platform_data = &pwm_id_table,
};

/* lcd-backlight */
static const struct led_pwm ledpwm = {
	.name		= "lcd-backlight",
	.pwm_id		= 190,
	.active_low	= 1,
	.max_brightness	= 255,
	.pwm_period_ns	= 85,
};

static const struct led_pwm_platform_data pwmblk_platformdata = {
	1,
	&ledpwm,
};

static struct platform_device pwm_bl_device = {
	.name	= "leds_pwm",
	.id	= -1,
	.dev.platform_data = &pwmblk_platformdata,
};

static struct resource samsung_charger_resources[] = {
	[0] = { // USB IRQ
		.start = 0,
		.end   = 0,
		.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_SHAREABLE,
	},
	[1] = { // TA IRQ
		.start = 0,
		.end   = 0,
		.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE | IORESOURCE_IRQ_LOWEDGE,
	},
	[2] = { // CHG_ING_N
		.start = 0,
		.end   = 0,
		.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE | IORESOURCE_IRQ_LOWEDGE,
	},
	[3] = { // CHG_EN
		.start = 0,
		.end   = 0,
		.flags = IORESOURCE_IRQ,
	},
};

static int samsung_charger_config_data[] = {
	// [ CHECK VF USING ADC ]
	/*   1. ENABLE  (true, flase) */
	true,
	/*   2. ADCPORT (ADCPORT NUM) */
	0,
	// [ SUPPORT TA_NCHG IRQ FOR CHECKING FULL ]
	/*   1. ENABLE  (true, flase) */
	true,
};

static int samsung_battery_config_data[] = {
	// [ SUPPORT MONITORING CHARGE CURRENT FOR CHECKING FULL ]
	/* 1.ENABLE  (true, flase) */
	false,
	/* 2. ADCPORT (ADCPORT NUM) */
	4,

	// [ SUPPORT MONITORING TEMPERATURE OF THE SYSTEM FOR BLOCKING CHARGE ]
	/* 1. ENABLE  (true, flase) */
	true,
	/* 2. ADCPORT (ADCPORT NUM) */
	1,
	
};

static struct platform_device samsung_charger_device = {
	.name		= "secChargerDev",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(samsung_charger_resources),
	.resource	= samsung_charger_resources,
	.dev = {
		.platform_data = &samsung_charger_config_data,
	},
};

static struct platform_device samsung_battery_device = {
	.name		= "secBattMonitor",
	.id		= -1,
	.num_resources	= 0,
	.dev = {
		.platform_data = &samsung_battery_config_data,
	},
};


#if (CONFIG_SAMSUNG_OMAP4_TAB_REV == 3)
static inline void __init samsung_omap4_battery_init(void)
{
	samsung_charger_resources[0].start = 0; /* gpio_to_irq(OMAP4430_GPIO_USBSW_NINT); */

	/* TA_nCONNECTED */
	if (gpio_request(OMAP4430_GPIO_TA_NCONNECTED, "ta_nconnected irq") < 0) {
		printk(KERN_ERR "Failed to request GPIO%d for ta_nconnected IRQ\n",
				OMAP4430_GPIO_TA_NCONNECTED);
		samsung_charger_resources[1].start = -1;
	} else {
		samsung_charger_resources[1].start = gpio_to_irq(OMAP4430_GPIO_TA_NCONNECTED);
		gpio_direction_input(OMAP4430_GPIO_TA_NCONNECTED);
	}

	/* CHG_ING_N */
	if (gpio_request(OMAP4430_GPIO_CHG_ING_N, "charge full irq") < 0) {
		printk(KERN_ERR "Failed to request GPIO%d for charge full IRQ\n",
			   OMAP4430_GPIO_CHG_ING_N);
		samsung_charger_resources[2].start = -1;
	} else {
		samsung_charger_resources[2].start = gpio_to_irq(OMAP4430_GPIO_CHG_ING_N);
		gpio_direction_input(OMAP4430_GPIO_CHG_ING_N);
	}

	/* CHG_EN */
	if (gpio_request(OMAP4430_GPIO_CHG_EN, "Charge enable gpio") < 0) {
		printk(KERN_ERR "Failed to request GPIO%d for charge enable gpio\n",
				OMAP4430_GPIO_CHG_EN);
		samsung_charger_resources[3].start = -1;
	} else {
		samsung_charger_resources[3].start = gpio_to_irq(OMAP4430_GPIO_CHG_EN);
		gpio_direction_output(OMAP4430_GPIO_CHG_EN, 0);
	}
}
#endif

#if defined( CONFIG_SAMSUNG_PHONE_SVNET )

static void ipc_hsi_cfg_gpio( void );

static struct ipc_hsi_platform_data ipc_hsi_data = {
	.gpio_suspend_request = OMAP4430_GPIO_MIPI_HSI_SUSPEND_REQUEST,

	.cfg_gpio = ipc_hsi_cfg_gpio,
};

static struct resource ipc_hsi_res[] = {
	[ 0 ] = {	 // suspend_request irq
		.start = OMAP_GPIO_IRQ( OMAP4430_GPIO_MIPI_HSI_SUSPEND_REQUEST ),
		.end = OMAP_GPIO_IRQ( OMAP4430_GPIO_MIPI_HSI_SUSPEND_REQUEST ),
		.flags = IORESOURCE_IRQ,
	},
};
#if defined( CONFIG_MIPI_HSI_LOOPBACK_TEST )
static struct platform_device mipi_hsi_test = {
	.name = "onedram",
	.id = -1,
	.num_resources = ARRAY_SIZE( mipi_hsi_res ),
	.resource = mipi_hsi_res,
	.dev = {
		.platform_data = &mipi_hsi_data,
	},
};
#else
static struct platform_device ipc_hsi = {
	.name = "onedram",
	.id = -1,
	.num_resources = ARRAY_SIZE( ipc_hsi_res ),
	.resource = ipc_hsi_res,
	.dev = {
		.platform_data = &ipc_hsi_data,
	},
};
#endif

static void ipc_hsi_cfg_gpio( void )
{
	int err = 0;
	
#ifdef _OMAP_MUX_SETTING
	void __iomem* gpio_reg_addr;
	u32 reg_val;
#endif
	
	unsigned gpio_suspend_request = ipc_hsi_data.gpio_suspend_request;

#ifdef _OMAP_MUX_SETTING
	// gpio_56 ( SUSPEND_REQUEST )
      	gpio_reg_addr = ( void __iomem* )( OMAP4_CTRL_MODULE_PAD_CORE_MUX_PBASE + OMAP4_CTRL_MODULE_PAD_GPMC_NADV_ALE_OFFSET );
      	reg_val = omap_readl( ( u32 )gpio_reg_addr );
      	reg_val &= 0xFFFF0000; // save origin val
      	reg_val |= 0x0000c10b; // Muxmod 3,  14,15bit set1 : wakeup event
      	omap_writel( reg_val, ( u32 )gpio_reg_addr );
	printk( "ipc_hsi_cfg_gpio - %s gpio setting done...\n", "SUSPEND_REQUEST" );
#endif  //_OMAP_MUX_SETTING

	err = gpio_request( gpio_suspend_request, "SUSPEND_REQUEST" );
	if( err ) {
		printk( "ipc_hsi_cfg_gpio - fail to request gpio %s : %d\n", "SUSPEND_REQUEST", err );
	}
	else {
		gpio_direction_input( gpio_suspend_request );
	}
	
	// Irq Setting
	set_irq_type( OMAP_GPIO_IRQ( OMAP4430_GPIO_MIPI_HSI_SUSPEND_REQUEST ), IRQ_TYPE_EDGE_BOTH );
}

static void modemctl_cfg_gpio( void );


static struct modemctl_platform_data mdmctl_data = {
	.name = "xmm",
	
	.gpio_phone_on = OMAP4430_GPIO_MIPI_HSI_CP_ON,
	.gpio_phone_active = OMAP4430_GPIO_MIPI_HSI_PHONE_ACTIVE,
	.gpio_pda_active = OMAP4430_GPIO_MIPI_HSI_PDA_ACTIVE,
	.gpio_cp_reset = OMAP4430_GPIO_MIPI_HSI_CP_RST,
	.gpio_reset_req_n = OMAP4430_GPIO_MIPI_HSI_RESET_REQ_N,

	//.gpio_con_cp_sel = OMAP_GPIO_CON_CP_SEL,
	//.gpio_phone_on = GPIO_PHONE_ON,
	//.gpio_usim_boot = GPIO_USIM_BOOT,
	//.gpio_sim_ndetect = GPIO_SIM_nDETECT,
	
	.cfg_gpio = modemctl_cfg_gpio,
};

static struct resource mdmctl_res[] = {
	[ 0 ] = {
		.start = OMAP_GPIO_IRQ( OMAP4430_GPIO_MIPI_HSI_PHONE_ACTIVE ), // phone active irq
		.end = OMAP_GPIO_IRQ( OMAP4430_GPIO_MIPI_HSI_PHONE_ACTIVE ),
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device modemctl = {
	.name = "modemctl",
	.id = -1,
	.num_resources = ARRAY_SIZE( mdmctl_res ),
	.resource = mdmctl_res,
	.dev = {
		.platform_data = &mdmctl_data,
	},
};

#define SIM_GPIO
#ifdef SIM_GPIO
#define CONTROL_PBIASLITE 0x4A100600
#define CONTROL_MMC1 0x4A100628
#define CONTROL_GPIOWK 0x4A31E600
#endif
static void modemctl_cfg_gpio( void )
{
	int err = 0;

#ifdef _OMAP_MUX_SETTING
	u32 gpio_reg_addr;
	u32 reg_val;
#ifdef SIM_GPIO
	u32 val;
#endif
#endif

	unsigned gpio_phone_on = mdmctl_data.gpio_phone_on;
	unsigned gpio_cp_rst = mdmctl_data.gpio_cp_reset;
	unsigned gpio_pda_active = mdmctl_data.gpio_pda_active;
	unsigned gpio_phone_active = mdmctl_data.gpio_phone_active;
	unsigned gpio_reset_req_n = mdmctl_data.gpio_reset_req_n;

	// Mux Setting
#ifdef _OMAP_MUX_SETTING
	// gpio_50 ( RESET_REQ_N )
     	gpio_reg_addr = ( u32 ) ( OMAP4_CTRL_MODULE_PAD_CORE_MUX_PBASE + OMAP4_CTRL_MODULE_PAD_GPMC_NCS0_OFFSET );
      	//reg_val = __raw_readl( gpio_reg_addr );
      	reg_val = omap_readl( gpio_reg_addr );
      	reg_val &= 0xFFFF0000; // save origin val
      	reg_val |= 0x0000010b; // Muxmod 3
       //__raw_writel( reg_val, gpio_reg_addr );
       omap_writel( reg_val, ( OMAP4_CTRL_MODULE_PAD_CORE_MUX_PBASE + OMAP4_CTRL_MODULE_PAD_GPMC_NCS0_OFFSET ) );
	printk( "modemctl_cfg_gpio - %s gpio setting done...\n", "RESET_REQ_N" );

	// gpio_36 ( CP_ON )
      	gpio_reg_addr = ( u32 ) ( OMAP4_CTRL_MODULE_PAD_CORE_MUX_PBASE + OMAP4_CTRL_MODULE_PAD_GPMC_AD12_OFFSET );
      	reg_val = omap_readl( gpio_reg_addr );
      	reg_val &= 0xFFFF0000; // save origin val
      	reg_val |= 0x0000010b; // Muxmod 3
      	omap_writel( reg_val, gpio_reg_addr );
	printk( "modemctl_cfg_gpio - %s gpio setting done...\n", "CP_ON" );

	// gpio_wk2 ( CP_RST )

#ifdef SIM_GPIO
    // 1.
        // 1) 2)
        val = omap_readl(CONTROL_PBIASLITE);
        omap_writel(0xEFEFFFFF & val, CONTROL_PBIASLITE);   
        // 3)
        val = omap_readl(CONTROL_GPIOWK);
        omap_writel(0xEFFFFFFF & val, CONTROL_GPIOWK);  

    // 2.
        // 1)
        val = omap_readl(CONTROL_PBIASLITE);
        omap_writel(0x7FFFFFFF & val, CONTROL_PBIASLITE);   
#endif


      	gpio_reg_addr = ( u32 ) ( OMAP4_CTRL_MODULE_PAD_WKUP_MUX_PBASE + OMAP4_CTRL_MODULE_PAD_SIM_RESET_OFFSET );
      	reg_val = omap_readl( gpio_reg_addr );
      	reg_val &= 0xFFFF0000; // save origin val
      	reg_val |= 0x0000011b; // PULLUP (4bit) , Muxmod 3
      	omap_writel( reg_val, gpio_reg_addr );

#ifdef SIM_GPIO
        // 4)
        val = omap_readl(CONTROL_MMC1);
        omap_writel(0xFEFFFFFF & val, CONTROL_MMC1);    

        // 5)
        val = omap_readl(CONTROL_GPIOWK);
        omap_writel(0x5FFFFFFF & val, CONTROL_GPIOWK);  

    // 4.
        val = omap_readl(CONTROL_PBIASLITE);
        omap_writel(0xF7EFFFFF & val, CONTROL_PBIASLITE);   

    // 6.
        // 1) 2)
        val = omap_readl(CONTROL_PBIASLITE);
        omap_writel(0x10100000 | val, CONTROL_PBIASLITE);   

        // 3)
        val = omap_readl(CONTROL_GPIOWK);
        omap_writel(0x10000000 | val, CONTROL_GPIOWK);  

    // 7.
        val = omap_readl(CONTROL_PBIASLITE);
        omap_writel(0x10100000 | val, CONTROL_PBIASLITE);   
#endif

	printk( "modemctl_cfg_gpio - %s gpio setting done...\n", "CP_RST" );


	// gpio_120 ( PHONE_ACTIVE ), gpio_119 ( PDA_ACTIVE )
      	gpio_reg_addr = ( u32 ) ( OMAP4_CTRL_MODULE_PAD_CORE_MUX_PBASE + OMAP4_CTRL_MODULE_PAD_ABE_DMIC_CLK1_OFFSET );
      	reg_val = omap_readl( gpio_reg_addr );
      	reg_val &= 0x00000000; // save origin val
      	//reg_val |= 0x010b010b; // Muxmod 3, Muxmod 3
      	reg_val |= 0x01030103; // Muxmod 3, Muxmod 3, pullup/down disabled ( B -> 3)
      	omap_writel( reg_val, gpio_reg_addr );  
	printk( "modemctl_cfg_gpio - %s gpio setting done...\n", "PDA_ACTIVE" );
	printk( "modemctl_cfg_gpio - %s gpio setting done...\n", "PHONE_ACTIVE" );
#endif  //_OMAP_MUX_SETTING

	err = gpio_request( gpio_reset_req_n, "RESET_REQ_N" );
	if( err ) {
		printk( "modemctl_cfg_gpio - fail to request gpio %s : %d\n", "RESET_REQ_N", err );
	}
	else {
		gpio_direction_output( gpio_reset_req_n, 0 );
	}
	
	err = gpio_request( gpio_phone_on, "CP_ON" );
	if( err ) {
		printk( "modemctl_cfg_gpio - fail to request gpio %s : %d\n", "CP_ON", err );
	}
	else {
		gpio_direction_output( gpio_phone_on, 0 );
	}

	err = gpio_request( gpio_cp_rst, "CP_RST" );
	if( err ) {
		printk( "modemctl_cfg_gpio - fail to request gpio %s : %d\n", "CP_RST", err );
	}
	else {
		gpio_direction_output( gpio_cp_rst, 0 );
	}

	err = gpio_request( gpio_pda_active, "PDA_ACTIVE" );
	if( err ) {
		printk( "modemctl_cfg_gpio - fail to request gpio %s : %d\n", "PDA_ACTIVE", err );
	}
	else {
		gpio_direction_output( gpio_pda_active, 0 );
	}
	
	err = gpio_request( gpio_phone_active, "PHONE_ACTIVE" );
	if( err ) {
		printk( "modemctl_cfg_gpio - fail to request gpio %s : %d\n", "PHONE_ACTIVE", err );
	}
	else {
		gpio_direction_output( gpio_phone_active, 0 );
	}
	
	set_irq_type( OMAP_GPIO_IRQ( OMAP4430_GPIO_MIPI_HSI_PHONE_ACTIVE ), IRQ_TYPE_EDGE_BOTH );
	//set_irq_type( gpio_sim_ndetect, IRQ_TYPE_EDGE_BOTH );
}
#endif // CONFIG_SAMSUNG_PHONE_SVNET

static struct platform_device *omap4tab_devices[] __initdata = {
	&wl128x_device,
	&omap4tab_hdmi_audio_device,
	&omap4tab_keypad_led,
	&samsung_omap4_pwr_device,
	&samsung_omap4_gyro_lp530al_device,
	&pwmbacklight_device,
	&pwm_bl_device,
	&samsung_battery_device,
	&samsung_charger_device,
	&pwm_haptic_device,
#ifdef CONFIG_EAR_MIC_ADC
	&omap4tab_ear_mic_adc_device,
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

static void __init omap4_tab_10_1_init_irq(void)
{
	omap2_init_common_hw(NULL, NULL);
#ifdef CONFIG_OMAP_32K_TIMER
	omap2_gp_clockevent_set_gptimer(1);
#endif
	gic_init_irq();
	sr_class3_init();
}

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_UTMI,
#ifdef CONFIG_USB_MUSB_OTG
	.mode			= MUSB_OTG,
#elif defined(CONFIG_USB_MUSB_HDRC_HCD)
	.mode			= MUSB_HOST,
#elif defined(CONFIG_USB_GADGET_MUSB_HDRC)
	.mode			= MUSB_PERIPHERAL,
#endif
	.power			= 100,
};

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 2,
		.caps		= MMC_CAP_8_BIT_DATA,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.ocr_mask	= MMC_VDD_165_195,
		.nonremovable   = true,
#ifdef CONFIG_PM_RUNTIME
		.power_saving	= true,
#endif
	},
	{
		.mmc		= 1,
		.caps		= MMC_CAP_8_BIT_DATA,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
#ifdef CONFIG_PM_RUNTIME
		.power_saving	= true,
#endif
	},
	{
		.mmc		= 5,
		.caps		= MMC_CAP_8_BIT_DATA,
		.gpio_cd	= -EINVAL,
		.gpio_wp        = -EINVAL,
		.ocr_mask	= MMC_VDD_165_195,
#ifdef CONFIG_PM_RUNTIME
		.power_saving	= false,
#endif
	},
	{}	/* Terminator */
};


static int omap4_twl6030_hsmmc_late_init(struct device *dev)
{
	int ret = 0;
	struct platform_device *pdev = container_of(dev,
				struct platform_device, dev);
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

static struct regulator_consumer_supply omap4tab_vmmc_supply[] = {
	{
		.supply = "vmmc",
		.dev_name = "mmci-omap-hs.0",
	},
};

static struct regulator_consumer_supply omap4tab_cam2_supply[] = {
	{
		.supply = "cam2pwr",
	},
};

static struct regulator_consumer_supply omap4tab_vusim_supply[] = {
	{
		.supply = "vusim",
		.dev_name = "display0",
	},
};


static struct regulator_init_data omap4tab_vaux1 = {
	.constraints = {
		.min_uV			= 1000000,
		.max_uV			= 3000000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

static struct regulator_init_data omap4tab_vaux2 = {
	.constraints = {
		.min_uV			= 1200000,
		.max_uV			= 2800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

static struct regulator_init_data omap4tab_vaux3 = {
	.constraints = {
		.min_uV			= 1000000,
		.max_uV			= 3000000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = omap4tab_cam2_supply,
};

/* VMMC1 for MMC1 card */
static struct regulator_init_data omap4tab_vmmc = {
	.constraints = {
		.min_uV			= 1200000,
		.max_uV			= 3000000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = omap4tab_vmmc_supply,
};

static struct regulator_init_data omap4tab_vpp = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 2500000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

static struct regulator_init_data omap4tab_vusim = {
	.constraints = {
		.min_uV			= 1200000,
		.max_uV			= 3000000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
   .num_consumer_supplies  = 1,
   .consumer_supplies      = &omap4tab_vusim_supply,
};

static struct regulator_init_data omap4tab_vana = {
	.constraints = {
		.min_uV			= 2100000,
		.max_uV			= 2100000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

static struct regulator_init_data omap4tab_vcxio = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

static struct regulator_init_data omap4tab_vdac = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 = REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

static struct regulator_init_data omap4tab_vusb = {
	.constraints = {
		.min_uV			= 3300000,
		.max_uV			= 3300000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask	 =	REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
};

static struct twl4030_madc_platform_data omap4tab_gpadc_data = {
	.irq_line	= 1,
};

static int omap4tab_batt_table[] = {
	/* adc code for temperature in degree C */
	929, 925, /* -2 ,-1 */
	920, 917, 912, 908, 904, 899, 895, 890, 885, 880, /* 00 - 09 */
	875, 869, 864, 858, 853, 847, 841, 835, 829, 823, /* 10 - 19 */
	816, 810, 804, 797, 790, 783, 776, 769, 762, 755, /* 20 - 29 */
	748, 740, 732, 725, 718, 710, 703, 695, 687, 679, /* 30 - 39 */
	671, 663, 655, 647, 639, 631, 623, 615, 607, 599, /* 40 - 49 */
	591, 583, 575, 567, 559, 551, 543, 535, 527, 519, /* 50 - 59 */
	511, 504, 496 /* 60 - 62 */
};
static struct twl4030_bci_platform_data omap4tab_bci_data = {
	.monitoring_interval		= 10,
	.max_charger_currentmA		= 1500,
	.max_charger_voltagemV		= 4560,
	.max_bat_voltagemV		= 4200,
	.low_bat_voltagemV		= 3300,
	.termination_currentmA		= 50,
	.battery_tmp_tbl		= omap4tab_batt_table,
	.tblsize			= ARRAY_SIZE(omap4tab_batt_table),
};

static void omap4_audio_conf(void)
{
	int ret;
    /* twl6040 naudint */
    omap_mux_init_signal("sys_nirq2.sys_nirq2", \
			        OMAP_PIN_INPUT_PULLUP);
	ret = gpio_request(OMAP4TAB_MICBIAS_EN_GPIO, "MAIN_MIC_BIAS_GPIO");
	if (ret) {
		printk(KERN_ERR "failed to get GPIO_MICBIAS\n");
	}
	gpio_direction_output(OMAP4TAB_MICBIAS_EN_GPIO, 0);
}

void main_mic_bias_cb(bool value)
{
	printk(KERN_ERR "[Audio]main_mic_bias_cb %d\n", value);
	if (value)
		gpio_set_value(OMAP4TAB_MICBIAS_EN_GPIO, 1);
	else
		gpio_set_value(OMAP4TAB_MICBIAS_EN_GPIO, 0);
}
static struct twl4030_codec_audio_data twl6040_audio = {
	/* Add audio only data */
	.mainmic_gpio_cb = main_mic_bias_cb,
};

static struct twl4030_codec_vibra_data twl6040_vibra = {
	 .max_timeout    = 15000,
	 .initial_vibrate = 0,

};

static struct twl4030_codec_data twl6040_codec = {
	.audio_mclk	= 38400000,
	.audio = &twl6040_audio,
	.vibra = &twl6040_vibra,
    .audpwron_gpio  = 127,
    .naudint_irq    = OMAP44XX_IRQ_SYS_2N,
    .irq_base   = TWL6040_CODEC_IRQ_BASE,
};

static struct twl4030_platform_data omap4tab_twldata = {
	.irq_base	= TWL6030_IRQ_BASE,
	.irq_end	= TWL6030_IRQ_END,

	/* Regulators */
	.vmmc		= &omap4tab_vmmc,
	.vpp		= &omap4tab_vpp,
	.vusim		= &omap4tab_vusim,
	.vana		= &omap4tab_vana,
	.vcxio		= &omap4tab_vcxio,
	.vdac		= &omap4tab_vdac,
	.vusb		= &omap4tab_vusb,
	.vaux1		= &omap4tab_vaux1,
	.vaux2		= &omap4tab_vaux2,
	.vaux3		= &omap4tab_vaux3,
	.madc           = &omap4tab_gpadc_data,
	.bci            = &omap4tab_bci_data,

	/* children */
	.codec          = &twl6040_codec,
};

static struct i2c_board_info __initdata omap4tab_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("twl6030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = OMAP44XX_IRQ_SYS_1N,
		.platform_data = &omap4tab_twldata,
	},
};
#ifndef DISABLE_I2C2_HW_SPINLOCK_FOR_REV5
static struct i2c_board_info __initdata omap4tab_i2c_2_boardinfo[] = {
#if (CONFIG_SAMSUNG_OMAP4_TAB_REV <= 2)
	{
		I2C_BOARD_INFO("isl29023", 0x44),
		.irq	= OMAP_GPIO_IRQ(OMAP4TAB_ISL29023_INT_GPIO),
	},
#endif
	{
		I2C_BOARD_INFO("cam_pmic", 0x7D),
	},
	{
		I2C_BOARD_INFO("M5MO", 0x3E >> 1),
	}
};
#endif
#if ((CONFIG_SAMSUNG_OMAP4_TAB_REV == 5) \
	 || (CONFIG_SAMSUNG_OMAP4_TAB_REV == 6) \
	 || (CONFIG_SAMSUNG_OMAP4_TAB_REV == 7))
static struct pn544_i2c_platform_data pn544_data = {
         .irq_gpio = OMAP4430_GPIO_NFC_IRQ,
         .ven_gpio = OMAP4430_GPIO_NFC_VEN,
         .firm_gpio = OMAP4430_GPIO_NFC_FIRM,
};
#endif


static struct i2c_board_info __initdata omap4tab_i2c_3_boardinfo[] = {
#if (CONFIG_SAMSUNG_OMAP4_TAB_REV == 4 || \
	CONFIG_SAMSUNG_OMAP4_TAB_REV == 5 || \
	CONFIG_SAMSUNG_OMAP4_TAB_REV == 6 || \
	CONFIG_SAMSUNG_OMAP4_TAB_REV == 7)
	{
		I2C_BOARD_INFO("cypress_touchkey", 0x20),
	},
#endif
#if ((CONFIG_SAMSUNG_OMAP4_TAB_REV == 5)\
	 || (CONFIG_SAMSUNG_OMAP4_TAB_REV ==6) \
	 || (CONFIG_SAMSUNG_OMAP4_TAB_REV ==7))
	{
		I2C_BOARD_INFO("max17040", 0x36),
	},
#endif
#if (CONFIG_SAMSUNG_OMAP4_TAB_REV == 3)
	{
		I2C_BOARD_INFO("secFuelgaugeDev", 0x36),
		.flags = I2C_CLIENT_WAKE,
		.irq = OMAP_GPIO_IRQ(OMAP4430_GPIO_FUEL_ALERT),
	},
#endif
#if (CONFIG_SAMSUNG_OMAP4_TAB_REV == 3)
	{
		I2C_BOARD_INFO("maXTouch", 0x4c),
		.irq = OMAP_GPIO_IRQ(OMAP4TAB_TOUCH_nINT_GPIO),
		.platform_data = &mxt1386_ts_platform_data,
	},
#endif
#if (CONFIG_SAMSUNG_OMAP4_TAB_REV == 7)
	{
		I2C_BOARD_INFO("mxt_touch", 0x4a),
		.irq		= OMAP_GPIO_IRQ(OMAP_GPIO_TSP_INT),
		.platform_data = &t1_mxt224E_platform_data,
	},
#endif
#ifdef CONFIG_FMRADIO
        {
                I2C_BOARD_INFO("Si4709_driver", 0x10),
                .irq            = OMAP_GPIO_IRQ(OMAP4430_GPIO_FM_INT),
        },
#endif
};

static struct i2c_board_info __initdata omap4tab_i2c_4_boardinfo[] = {
#if (CONFIG_SAMSUNG_OMAP4_TAB_REV <= 2)
	{
		I2C_BOARD_INFO("kxsd9", 0x18),
		.irq	= OMAP_GPIO_IRQ(OMAP4TAB_KXSD9_INT_GPIO),
	},
	{
		I2C_BOARD_INFO("akm8973", 0x1C),
		.irq	= OMAP_GPIO_IRQ(OMAP4TAB_AKM_INT_GPIO),
	},
#elif (CONFIG_SAMSUNG_OMAP4_TAB_REV == 3)
	{
		I2C_BOARD_INFO("kxsd9", 0x18),
		.irq	= OMAP_GPIO_IRQ(OMAP4TAB_KXSD9_INT_GPIO),
	},
	{
		I2C_BOARD_INFO("mpu3050", 0x68),
		.irq	= OMAP_GPIO_IRQ(OMAP4TAB_MPU3050_INT_GPIO),
	},
	{
		I2C_BOARD_INFO("akm8975", 0x0C),
		.irq	= OMAP_GPIO_IRQ(OMAP4TAB_AKM_INT_GPIO),
	},
        {
                I2C_BOARD_INFO("cmc623_i2c_driver", 0x38),
        },

#elif (CONFIG_SAMSUNG_OMAP4_TAB_REV == 4 )
        {
                I2C_BOARD_INFO("kxsd9", 0x18),
                .irq    = OMAP_GPIO_IRQ(OMAP4TAB_KXSD9_INT_GPIO),
        },
        {
                I2C_BOARD_INFO("mpu3050", 0x68),
                .irq    = OMAP_GPIO_IRQ(OMAP4TAB_MPU3050_INT_GPIO),
        },
        {
                I2C_BOARD_INFO("akm8975", 0x0C),
                .irq    = OMAP_GPIO_IRQ(OMAP4TAB_AKM_INT_GPIO),
        },
        {
                I2C_BOARD_INFO("gp2a",0x44),
		.platform_data = &gp2a_pdata,
        },
        {
               I2C_BOARD_INFO("fsa9480", (0x4A >> 1)),
               .platform_data = &omap4_fsa9480_pdata,
               .irq = OMAP_GPIO_IRQ(OMAP4430_GPIO_JACK_NINT),
        },

#elif (CONFIG_SAMSUNG_OMAP4_TAB_REV == 5 )
	{
		I2C_BOARD_INFO(SENSOR_MPU_NAME, 0x68),
		.irq = OMAP_GPIO_IRQ(OMAP4TAB_MPU3050_INT_GPIO),
		.platform_data = &mpu_data,
	},
//	{
//		I2C_BOARD_INFO("kxsd9", 0x18),
//		.irq	= OMAP_GPIO_IRQ(OMAP4TAB_KXSD9_INT_GPIO),
//	},
//	{
//		I2C_BOARD_INFO("mpu3050", 0x68),
//		.irq	= OMAP_GPIO_IRQ(OMAP4TAB_MPU3050_INT_GPIO),
//	},
	{
		I2C_BOARD_INFO("gp2a",0x44),
		.platform_data = &gp2a_pdata,
		
	},
//	{
//		I2C_BOARD_INFO("yas530",0x2E),
//		
//	},
        {
               I2C_BOARD_INFO("fsa9480", (0x4A >> 1)),
               .platform_data = &omap4_fsa9480_pdata,
               .irq = OMAP_GPIO_IRQ(OMAP4430_GPIO_JACK_NINT),
        },
	{
               I2C_BOARD_INFO("pn544",0x2b),
              .platform_data = &pn544_data,
               .irq = OMAP_GPIO_IRQ(OMAP4430_GPIO_NFC_IRQ),
        },
		{
				I2C_BOARD_INFO("geomagnetic", 0x2e),
		},		
#elif (CONFIG_SAMSUNG_OMAP4_TAB_REV == 6 )
        {
                I2C_BOARD_INFO(SENSOR_MPU_NAME, 0x68),
                .irq = OMAP_GPIO_IRQ(OMAP4TAB_MPU3050_INT_GPIO),
                .platform_data = &mpu_data,
        },
        {
                I2C_BOARD_INFO("gp2a",0x44),
                .platform_data = &gp2a_pdata,

        },
        {
               I2C_BOARD_INFO("fsa9480", (0x4A >> 1)),
               .platform_data = &omap4_fsa9480_pdata,
               .irq = OMAP_GPIO_IRQ(OMAP4430_GPIO_JACK_NINT),
        },
        {
               I2C_BOARD_INFO("pn544",0x2b),
               .platform_data = &pn544_data,
               .irq = OMAP_GPIO_IRQ(OMAP4430_GPIO_NFC_IRQ),
        },
		{
				I2C_BOARD_INFO("geomagnetic", 0x2e),
		},
	#ifdef CONFIG_VIDEO_MHL_V1
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
    #endif

#elif (CONFIG_SAMSUNG_OMAP4_TAB_REV == 7 )
{
		I2C_BOARD_INFO(SENSOR_MPU_NAME, 0x68),
		.irq = OMAP_GPIO_IRQ(OMAP4TAB_MPU3050_INT_GPIO),
		.platform_data = &mpu_data,
},
{
		I2C_BOARD_INFO("cm3663",0x11),
		.platform_data = &cm3663_pdata,
		//.irq = OMAP_GPIO_IRQ(OMAP4TAB_GP2A_PS_VOUT),

},
{
		I2C_BOARD_INFO("geomagnetic", 0x2e),
},
{
	   I2C_BOARD_INFO("fsa9480", (0x4A >> 1)),
	   .platform_data = &omap4_fsa9480_pdata,
	   .irq = OMAP_GPIO_IRQ(OMAP4430_GPIO_JACK_NINT),
},
{
	   I2C_BOARD_INFO("pn544",0x2b),
	   .platform_data = &pn544_data,
	   .irq = OMAP_GPIO_IRQ(OMAP4430_GPIO_NFC_IRQ),
},
 	#ifdef CONFIG_VIDEO_MHL_V1
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
    #endif


#endif

};

static const struct usbhs_omap_platform_data usbhs_pdata __initconst = {
	.port_mode[0] = OMAP_OHCI_PORT_MODE_PHY_4PIN_DPDM,
	.port_mode[1] = OMAP_USBHS_PORT_MODE_UNUSED,
	.port_mode[2] = OMAP_USBHS_PORT_MODE_UNUSED,
	.phy_reset  = false,
	.reset_gpio_port[0]  = -EINVAL,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL
};

static struct omap_i2c_bus_board_data __initdata omap4tab_i2c_bus_pdata;
static struct omap_i2c_bus_board_data __initdata omap4tab_i2c_2_bus_pdata;
static struct omap_i2c_bus_board_data __initdata omap4tab_i2c_3_bus_pdata;
static struct omap_i2c_bus_board_data __initdata omap4tab_i2c_4_bus_pdata;

/*
 * LPDDR2 Configuration Data:
 * The memory organisation is as below :
 *	EMIF1 - CS0 -   2 Gb
 *	CS1 -   NULL
 *	EMIF2 - CS0 -   2 Gb
 *	CS1 -    NULL
 *	--------------------
 *	TOTAL -         4 Gb
 *
 * Same devices installed on EMIF1 and EMIF2
 */
static __initdata struct emif_device_details emif_devices = {
	.cs0_device = &sec_2G_S4,
	.cs1_device = NULL
};

static void __init omap_i2c_hwspinlock_init(int bus_id, unsigned int
			spinlock_id, struct omap_i2c_bus_board_data *pdata)
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
	omap_i2c_hwspinlock_init(1, 0, &omap4tab_i2c_bus_pdata);
#ifndef DISABLE_I2C2_HW_SPINLOCK_FOR_REV5
	omap_i2c_hwspinlock_init(2, 1, &omap4tab_i2c_2_bus_pdata);
#endif
	omap_i2c_hwspinlock_init(3, 2, &omap4tab_i2c_3_bus_pdata);
	omap_i2c_hwspinlock_init(4, 3, &omap4tab_i2c_4_bus_pdata);
	/*
	 * Phoenix Audio IC needs I2C1 to
	 * start with 400 KHz or less
	 */
	omap_register_i2c_bus(1, 400, &omap4tab_i2c_bus_pdata,
		omap4tab_i2c_boardinfo, ARRAY_SIZE(omap4tab_i2c_boardinfo));
#ifndef DISABLE_I2C2_HW_SPINLOCK_FOR_REV5
	omap_register_i2c_bus(2, 400, &omap4tab_i2c_2_bus_pdata,
		omap4tab_i2c_2_boardinfo, ARRAY_SIZE(omap4tab_i2c_2_boardinfo));
#endif
	omap_register_i2c_bus(3, 400, &omap4tab_i2c_3_bus_pdata,
		omap4tab_i2c_3_boardinfo, ARRAY_SIZE(omap4tab_i2c_3_boardinfo));
	omap_register_i2c_bus(4, 400, &omap4tab_i2c_4_bus_pdata,
		omap4tab_i2c_4_boardinfo, ARRAY_SIZE(omap4tab_i2c_4_boardinfo));
	return 0;
}


static u16 control_pbias_offset;

void omap4tab_gpiowk_setup()
{
	#define CONTROL_GPIOWK 0x4A31E600
	u32 val;
	u32 * ctrl_gpiowk;

	control_pbias_offset = OMAP44XX_CONTROL_PBIAS_LITE;
	val = omap_ctrl_readl(control_pbias_offset);

	val = val | (1 <<28);
	omap_ctrl_writel(val, control_pbias_offset);

	val = omap_ctrl_readl(control_pbias_offset);


	ctrl_gpiowk = (u32 *) ioremap(CONTROL_GPIOWK, 4);
	if (!ctrl_gpiowk) {
		printk(KERN_ERR"OMAP_pad_config: ioremap failed with addr %lx\n",
			CONTROL_GPIOWK);
		return;
	}

	val =  __raw_readl(ctrl_gpiowk);

	val |= (1 <<28);
	__raw_writel(val, ctrl_gpiowk);

	val =  __raw_readl(ctrl_gpiowk);

	val = omap_ctrl_readl(control_pbias_offset);

	if ((val & (1<<29))!=0)
	{
		val = val & ~(1 <<28);
		omap_ctrl_writel(val, control_pbias_offset);

		val = val & ~(1 <<28);
		__raw_writel(val, ctrl_gpiowk);
	}

	val = omap_ctrl_readl(control_pbias_offset);
 	if((val & (1<<30)) == 1)
		printk("SIM_VDDS is set to 3 volt\n");
	else
		printk("SIM_VDDS is set to 1.8 volt\n");

	return;
}
#ifdef CONFIG_SWITCH_GPIO
static void __init omap4tab_headset_init(void)
{
	u32 gpio_reg_addr;
	u32 reg_val;
	u32 ret;
       	ret = gpio_request(OMAP4TAB_MICBIAS_EN2_GPIO, "EAR_MIC_BIAS_GPIO");
        if (ret) {
                printk(KERN_ERR "failed to get EAR_MIC_BIAS_GPIO\n");  
        }
	gpio_direction_output(OMAP4TAB_MICBIAS_EN2_GPIO, 0);
      	gpio_reg_addr = ( u32 ) ( OMAP4_CTRL_MODULE_PAD_WKUP_MUX_PBASE + OMAP4_CTRL_MODULE_PAD_SIM_IO_OFFSET );
      	reg_val = omap_readl( gpio_reg_addr );
      	reg_val &= 0xFFFF0000; // save origin val
      	reg_val |= 0x0000011b; // PULLUP (4bit) , Muxmod 3
      	printk("0x%x, 0x%x\n", reg_val, gpio_reg_addr);
      	omap_writel( reg_val, gpio_reg_addr );
	printk( "omap4tab_headset_init - DET_3.5 gpio setting done...\n");
	#ifdef CONFIG_INPUT_EAR_KEY
	board_ear_key_resource.start = gpio_to_irq(OMAP4TAB_EAR_SEND_END_GPIO);
	#endif
}
#endif

#if (CONFIG_SAMSUNG_OMAP4_TAB_REV <= 2)
static void __init omap4tab_display_init(void)
{
        int r;

        mlcd1_enable_gpio    = SDP4430_MLCD1_PANEL_ENABLE_GPIO;
        mlcd2_enable_gpio    = SDP4430_MLCD2_PANEL_ENABLE_GPIO;
        backlight_gpio = SDP4430_LCD_PANEL_BACKLIGHT_ENABLE_GPIO;
        lvds_nshdn_gpio = SDP4430_LVDS_nRST_ENABLE_GPIO;

        r = gpio_request(mlcd1_enable_gpio, "LCD MLCD_ON");
       if (r) {
                printk(KERN_ERR "failed to get LCD MLCD_ON GPIO\n");
                goto err0;
        }

        r = gpio_request(mlcd2_enable_gpio, "LCD MLCD_ON");
        if (r) {
                printk(KERN_ERR "failed to get LCD MLCD_ON GPIO\n");
                goto err0;
        }
        r = gpio_request(backlight_gpio, "LCD Backlight");
        if (r) {
                printk(KERN_ERR "failed to get LCD backlight GPIO\n");
                goto err1;
        }
        r = gpio_request(lvds_nshdn_gpio, "LVDS_nRST");
        if (r) {
                printk(KERN_ERR "failed to get LCD MLCD_ON GPIO\n");
                goto err0;
        }

        gpio_direction_output(mlcd1_enable_gpio, 0);
        gpio_direction_output(mlcd2_enable_gpio, 0);
        gpio_direction_output(backlight_gpio, 0);
        gpio_direction_output(lvds_nshdn_gpio, 0);

        return;
err2:
        gpio_free(lvds_nshdn_gpio);
err1:
        gpio_free(mlcd1_enable_gpio);
        gpio_free(mlcd2_enable_gpio);
err0:
        return;

}
#elif (CONFIG_SAMSUNG_OMAP4_TAB_REV == 3)

static void __init omap4tab_display_init(void)
{
	    int r;

        mlcd_on_gpio    = OMAP4TAB_MLCD_ON_GPIO;
        led_backlight_reset_gpio    = OMAP4TAB_LED_BACKLIGHT_RESET_GPIO;
        lvds_nshdn_gpio  = OMAP4TAB_LVDS_nSHDN_GPIO;

        r = gpio_request(mlcd_on_gpio, "LCD MLCD_ON");
        if (r) {
                printk(KERN_ERR "failed to get LCD MLCD_ON GPIO\n");
                goto err0;
        }

        r = gpio_request(lvds_nshdn_gpio, "LVDS_nRST");
        if (r) {
                printk(KERN_ERR "failed to get LCD MLCD_ON GPIO\n");
                goto err1;
        }
        r = gpio_request(led_backlight_reset_gpio, "LCD LED_BACKLIGHT_RESET");
        if (r) {
                printk(KERN_ERR "failed to get LCD MLCD_ON GPIO\n");
                goto err0;
        }

        gpio_direction_output(mlcd_on_gpio, 0);
        gpio_direction_output(led_backlight_reset_gpio, 0);
	mdelay(1);
        gpio_direction_output(led_backlight_reset_gpio, 1);
        gpio_direction_output(lvds_nshdn_gpio, 0);

        return;

err1:
        gpio_free(lvds_nshdn_gpio);
err0:
		gpio_free(mlcd_on_gpio);
        return;
}

static void omap4tab_touch_init(void)
{
#if (CONFIG_SAMSUNG_OMAP4_TAB_REV == 3)
	printk("[SHANKAR] %s[%d] Entry \n", __func__, __LINE__);
	gpio_request(OMAP4_TOUCH_EN_GPIO, "TOUCH ENable GPIO");
	gpio_direction_output(OMAP4_TOUCH_EN_GPIO, 1);
	msleep(200);
        gpio_free(OMAP4_TOUCH_EN_GPIO);
	printk("[SHANKAR] %s[%d] exit \n", __func__, __LINE__);
#endif

}
static void omap4tab_vib_motor_init(void)
{
	/* Move this to proper place TBD */
	gpio_request(OMAP4TAB_MOTOR_EN1_GPIO, "Vib Motor 1 GPIO");
	gpio_direction_output(OMAP4TAB_MOTOR_EN1_GPIO, 0);
#if (CONFIG_SAMSUNG_OMAP4_TAB_REV == 3)
	gpio_request(OMAP4TAB_MOTOR_EN2_GPIO, "Vib Motor 2 GPIO");
	gpio_direction_output(OMAP4TAB_MOTOR_EN2_GPIO, 0);
#endif
}

#elif (CONFIG_SAMSUNG_OMAP4_TAB_REV == 4 || CONFIG_SAMSUNG_OMAP4_TAB_REV == 5)

static void __init omap4tab_display_init(void)
{
	    int r;

        lcd_en_gpio    = OMAP4TAB_LCD_EN_GPIO;
        mlcd_rst_gpio    = OMAP4TAB_MLCD_RST_GPIO;

        r = gpio_request(lcd_en_gpio, "LCD ENABLE");
        if (r) {
                printk(KERN_ERR "failed to get LCD ENABLE GPIO\n");
                goto err0;
        }

        r = gpio_request(mlcd_rst_gpio, "LCD Reset GPIO");
        if (r) {
                printk(KERN_ERR "failed to get LCD Reset GPIO\n");
                goto err1;
        }

        gpio_direction_output(lcd_en_gpio, 0);
        return;

err1:
        gpio_free(mlcd_rst_gpio);
err0:
		gpio_free(lcd_en_gpio);
        return;
}
static void omap4tab_touch_init(void)
{
	printk("[SHANKAR] %s[%d] Entry \n", __func__, __LINE__);
	gpio_request(OMAP4_TOUCH_EN_GPIO, "TOUCH ENable GPIO");
	gpio_direction_output(OMAP4_TOUCH_EN_GPIO, 1);
	msleep(250);
	printk("[SHANKAR] %s[%d] exit \n", __func__, __LINE__);
	
}
static void omap4tab_vib_motor_init(void)
{
}
#elif (CONFIG_SAMSUNG_OMAP4_TAB_REV == 6)
static void __init omap4tab_display_init(void)
{
		int r;

		lcd_en_gpio    = OMAP4TAB_LCD_EN_GPIO;
		mlcd_rst_gpio	 = OMAP4TAB_MLCD_RST_GPIO;

		r = gpio_request(lcd_en_gpio, "LCD ENABLE");
		if (r) {
				printk(KERN_ERR "failed to get LCD ENABLE GPIO\n");
				goto err0;
		}

		r = gpio_request(mlcd_rst_gpio, "LCD Reset GPIO");
		if (r) {
				printk(KERN_ERR "failed to get LCD Reset GPIO\n");
				goto err1;
		}

		gpio_direction_output(lcd_en_gpio, 1);
		return;

err1:
		gpio_free(mlcd_rst_gpio);
err0:
		gpio_free(lcd_en_gpio);
		return;
}

static void omap4tab_touch_init(void)
{
	printk("[SHANKAR] %s[%d] Entry \n", __func__, __LINE__);
	gpio_request(OMAP4_TOUCH_EN_GPIO, "TOUCH ENable GPIO");
	gpio_direction_output(OMAP4_TOUCH_EN_GPIO, 1);
	msleep(250);
	printk("[SHANKAR] %s[%d] exit \n", __func__, __LINE__);
	
}
static void omap4tab_vib_motor_init(void)
{
}
#elif (CONFIG_SAMSUNG_OMAP4_TAB_REV == 7)
static void __init omap4tab_display_init(void)
{
		int r;

		mlcd_rst_gpio	 = OMAP4TAB_MLCD_RST_GPIO;

		r = gpio_request(mlcd_rst_gpio, "LCD Reset GPIO");
		if (r) {
				printk(KERN_ERR "failed to get LCD Reset GPIO\n");
				goto err1;
		}
		
		return;

err1:
		gpio_free(mlcd_rst_gpio);
err0:
		gpio_free(lcd_en_gpio);
		return;
}

static void omap4tab_touch_init(void)
{
	printk("[SHANKAR] %s[%d] Entry \n", __func__, __LINE__);
	gpio_request(OMAP4_TOUCH_EN_GPIO, "TOUCH_EN");
	gpio_direction_output(OMAP4_TOUCH_EN_GPIO, 1);
	msleep(250);
	printk("[SHANKAR] %s[%d] exit \n", __func__, __LINE__);
	t1_touch_init_hw(&t1_mxt224E_platform_data);
}
static void omap4tab_vib_motor_init(void)
{
}
#else
#endif
static void enable_board_wakeup_source(void)
{
	u16 padconf;
	int gpio_val;

	/* NOTE: Use mx framework when available */
	/* Enable IO wakeup for the gpio used for primary touchscreen */
	padconf = omap_readw(CONTROL_CORE_PAD1_GPMC_AD11);
	padconf |= OMAP44XX_PADCONF_WAKEUPENABLE0;
	omap_writew(padconf, CONTROL_CORE_PAD1_GPMC_AD11);
	/* Enabling WKUP capability for TouchKey IRQ GPI0_32 */
	padconf = omap_readw(CONTROL_CORE_PAD0_GPMC_AD08);
	padconf |= OMAP44XX_PADCONF_WAKEUPENABLE0;
	omap_writew(padconf, CONTROL_CORE_PAD0_GPMC_AD08);
	/* Enabling WKUP capability for TouchScreen IRQ GPI0_43 */
	padconf = omap_readw(CONTROL_CORE_PAD1_GPMC_AD19);
	padconf |= OMAP44XX_PADCONF_WAKEUPENABLE1;
	omap_writew(padconf, CONTROL_CORE_PAD1_GPMC_AD19);

	// Proximity-Sensor setup as a wakeup-source
	gpio_val = omap_mux_get_gpio(OMAP4TAB_GP2A_PS_VOUT);
	if ((gpio_val & OMAP44XX_PADCONF_WAKEUPENABLE0) == 0) {
		gpio_val |= OMAP44XX_PADCONF_WAKEUPENABLE0;
		omap_mux_set_gpio(gpio_val, OMAP4TAB_GP2A_PS_VOUT);
	}

#if ((CONFIG_SAMSUNG_OMAP4_TAB_REV == 5) \
		|| (CONFIG_SAMSUNG_OMAP4_TAB_REV == 6) \
		|| (CONFIG_SAMSUNG_OMAP4_TAB_REV == 7) )
	/*
	 * Enable IO daisy for sys_nirq1/2 & HSI CAWAKE line, to be able to
	 * wakeup from interrupts from PMIC/Audio IC.
	 * Needed only in Device OFF mode.
	 */

	omap_mux_enable_wakeup("sys_nirq1");
	omap_mux_enable_wakeup("sys_nirq2");
	//omap_mux_enable_wakeup("usbb1_ulpitll_clk.hsi1_cawake");
#endif
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
	.vdd0_on = 1350000,        /* 1.35v */
	.vdd0_onlp = 1350000,      /* 1.35v */
	.vdd0_ret = 837500,       /* 0.8375v */
#if ((CONFIG_SAMSUNG_OMAP4_TAB_REV == 5) \
		|| (CONFIG_SAMSUNG_OMAP4_TAB_REV == 6) \
		|| (CONFIG_SAMSUNG_OMAP4_TAB_REV == 7) )
	.vdd0_off = 0,          /* 0 v */
#else
	.vdd0_off = 600000,       /* 0.6v */
#endif
	.vdd1_on = 1100000,        /* 1.1v */
	.vdd1_onlp = 1100000,      /* 1.1v */
	.vdd1_ret = 837500,       /* 0.8375v */
#if ((CONFIG_SAMSUNG_OMAP4_TAB_REV == 5) \
		|| (CONFIG_SAMSUNG_OMAP4_TAB_REV == 6) \
		|| (CONFIG_SAMSUNG_OMAP4_TAB_REV == 7) )
	.vdd1_off = 0,		/* 0 v */
#else
	.vdd1_off = 600000,       /* 0.6v */
#endif
	.vdd2_on = 1100000,        /* 1.1v */
	.vdd2_onlp = 1100000,      /* 1.1v */
	.vdd2_ret = 837500,       /* .8375v */
#if ((CONFIG_SAMSUNG_OMAP4_TAB_REV == 5) \
		|| (CONFIG_SAMSUNG_OMAP4_TAB_REV == 6) \
		|| (CONFIG_SAMSUNG_OMAP4_TAB_REV == 7) )
	.vdd2_off = 0,		/* 0 v */
#else
	.vdd2_off = 600000,       /* 0.6v */
#endif
};


void plat_hold_wakelock(void *up, int flag)
{
    struct uart_omap_port *up2 = (struct uart_omap_port *)up;
    /*Specific wakelock for bluetooth usecases*/
    if ((up2->pdev->id == BLUETOOTH_UART)
		        && ((flag == WAKELK_TX) || (flag == WAKELK_RX)))
	        wake_lock_timeout(&uart_lock, 2*HZ);

    /*Specific wakelock for console usecases*/
    if ((up2->pdev->id == CONSOLE_UART)
		        && ((flag == WAKELK_IRQ) || (flag == WAKELK_RESUME)))
	        wake_lock_timeout(&uart_lock, 5*HZ);
    return;
}

static struct omap_uart_port_info omap_serial_platform_data[] = {
	{
		.use_dma        = 0,
		.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
		.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
		.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
		.idle_timeout   = DEFAULT_IDLE_TIMEOUT,
		.flags          = 1,
		.plat_hold_wakelock = NULL,
        .rts_padconf    = 0,
        .rts_override   = 0,
        .padconf    = OMAP4_CTRL_MODULE_PAD_SDMMC1_CMD_OFFSET,
        .padconf_wake_ev = 0,
        .wk_mask    = 0,

	},
	{
		.use_dma        = 0,
		.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
		.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
		.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
		.idle_timeout   = DEFAULT_IDLE_TIMEOUT,
		.flags          = 1,
		.plat_hold_wakelock = plat_hold_wakelock,
        .rts_padconf    = OMAP4_CTRL_MODULE_PAD_UART2_RTS_OFFSET,
        .rts_override   = 0,
        .padconf    = OMAP4_CTRL_MODULE_PAD_UART2_RX_OFFSET,
        .padconf_wake_ev =
			            OMAP4_CTRL_MODULE_PAD_CORE_PADCONF_WAKEUPEVENT_3,
        .wk_mask    =
		            OMAP4_UART2_TX_DUPLICATEWAKEUPEVENT_MASK |
		            OMAP4_UART2_RX_DUPLICATEWAKEUPEVENT_MASK |
		            OMAP4_UART2_RTS_DUPLICATEWAKEUPEVENT_MASK |
		            OMAP4_UART2_CTS_DUPLICATEWAKEUPEVENT_MASK,

	},
	{
		.use_dma        = 0,
		.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
		.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
		.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
		.idle_timeout   = DEFAULT_IDLE_TIMEOUT,
		.flags          = 1,
		.plat_hold_wakelock = plat_hold_wakelock,
        .rts_padconf    = 0,
        .rts_override   = 0,
        .padconf    = OMAP4_CTRL_MODULE_PAD_UART3_RX_IRRX_OFFSET,
        .padconf_wake_ev =
			            OMAP4_CTRL_MODULE_PAD_CORE_PADCONF_WAKEUPEVENT_4,
        .wk_mask    =
		            OMAP4_UART3_TX_IRTX_DUPLICATEWAKEUPEVENT_MASK |
		            OMAP4_UART3_RX_IRRX_DUPLICATEWAKEUPEVENT_MASK |
		            OMAP4_UART3_RTS_SD_DUPLICATEWAKEUPEVENT_MASK |
		            OMAP4_UART3_CTS_RCTX_DUPLICATEWAKEUPEVENT_MASK,

	},
	{
		.use_dma        = 0,
		.dma_rx_buf_size = DEFAULT_RXDMA_BUFSIZE,
		.dma_rx_poll_rate = DEFAULT_RXDMA_POLLRATE,
		.dma_rx_timeout = DEFAULT_RXDMA_TIMEOUT,
		.idle_timeout   = DEFAULT_IDLE_TIMEOUT,
		.flags          = 1,
		.plat_hold_wakelock = NULL,
        .rts_padconf    = 0,
        .rts_override   = 0,
        .padconf    = OMAP4_CTRL_MODULE_PAD_UART4_RX_OFFSET,
        .padconf_wake_ev =
			            OMAP4_CTRL_MODULE_PAD_CORE_PADCONF_WAKEUPEVENT_4,
        .wk_mask    =
		            OMAP4_UART4_TX_DUPLICATEWAKEUPEVENT_MASK |
		            OMAP4_UART4_RX_DUPLICATEWAKEUPEVENT_MASK,

	},
	{
		.flags          = 0
	}
};

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
        { .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux       NULL
#endif
#ifdef DISABLE_I2C2_HW_SPINLOCK_FOR_REV5
void m5mo_fref_start(void)
{
    volatile u32 val=0,val2=0;
    void __iomem *frefclk1_base=NULL, *frefclk2_base=NULL;

    val = 0;

    frefclk1_base = ioremap(0x4A30A314, 4);
    val = __raw_readl(frefclk1_base);
    val = val | (1<<8) | (0x01<<2) | (0xF<<16);
    __raw_writel(val, (frefclk1_base));
    val = __raw_readl(frefclk1_base);
    printk(KERN_ERR"\n\t [MBG043 MCLK] AUXCLK1 is %x \n\n",val);
    iounmap(frefclk1_base);
      
	val2 = 0;

    frefclk2_base = ioremap(0x4A30A318, 4);
    val2 = __raw_readl(frefclk2_base);
    val2 = val2 | (1<<8) | (0x01<<2) | (0xF<<16);
    __raw_writel(val2, (frefclk2_base));
    val2 = __raw_readl(frefclk2_base);
    printk(KERN_ERR"\n\t [S5KAAFX MCLK] AUXCLK2 is %x \n\n",val2);
}
#endif


static void enable_rtc_gpio(void){
        /* To access twl registers we enable gpio6
         * we need this so the RTC driver can work.
         */
        gpio_request(TWL6030_RTC_GPIO, "h_SYS_DRM_MSEC");
        gpio_direction_output(TWL6030_RTC_GPIO, 1);

        omap_mux_init_signal("fref_clk0_out.gpio_wk6", \
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
	gpio_request(OMAP4TAB_MPU3050_INT_GPIO,"MPUIRQ");
	gpio_direction_input(OMAP4TAB_MPU3050_INT_GPIO);
}

#if ((CONFIG_SAMSUNG_OMAP4_TAB_REV == 6) \
         || (CONFIG_SAMSUNG_OMAP4_TAB_REV == 7))
static void gsd4t_gps_init(void)
{
	BUG_ON(!sec_class);
	gps_dev = device_create(sec_class, NULL, 0, NULL, "gps");
	if (IS_ERR(gps_dev))
		pr_err("Failed to create device(gps)!\n");

	gpio_request(OMAP_GPIO_AP_AGPS_TSYNC, "AP_AGPS_TSYNC");
	gpio_direction_output(OMAP_GPIO_AP_AGPS_TSYNC, 0);

	gpio_request(OMAP_GPIO_GPS_LNA_EN, "GPS_LNA_EN");
	gpio_direction_output(OMAP_GPIO_GPS_LNA_EN, 0);

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
#endif

static void __init omap4_tab_10_1_init(void)
{
	int status;
        int package = OMAP_PACKAGE_CBS;

        if (omap_rev() == OMAP4430_REV_ES1_0)
                package = OMAP_PACKAGE_CBL;
        //omap4_mux_init(board_mux, package);

	omap_emif_setup_device_details(&emif_devices, &emif_devices);
	omap4_lpddr2_config();
	omap_init_emif_timings();
	enable_rtc_gpio();
	omap4_audio_conf();
	omap4tab_gpiowk_setup();
	omap4tab_touch_init();
	omap4tab_vib_motor_init();
	omap4_i2c_init();
	omap4tab_display_init();
	platform_add_devices(omap4tab_devices, ARRAY_SIZE(omap4tab_devices));
	wake_lock_init(&uart_lock, WAKE_LOCK_SUSPEND, "uart_wake_lock");
	omap_serial_init(omap_serial_platform_data);
	omap4_twl6030_hsmmc_init(mmc);
	omap4_mpl_init();
#ifdef CONFIG_INPUT_CM3663	
	cm3663_gpio_request();
#endif
#ifdef CONFIG_TIWLAN_SDIO
	config_wlan_mux();
#endif
# if 0
	usb_uhhtll_init(&usbhs_pdata);
	usb_ehci_init();
	usb_ohci_init();
#endif

	/* OMAP4 SDP uses internal transceiver so register nop transceiver */
	usb_nop_xceiv_register();
	usb_musb_init(&musb_board_data);

	status = omap4_keypad_initialization(&omap4tab_keypad_data);
	if(status)
		pr_err("Keypad initialization failed: %d\n", status);

	omap_display_init(&omap4tab_dss_data);
	enable_board_wakeup_source();

	omap_voltage_register_pmic(&omap_pmic_core, "core");
	omap_voltage_register_pmic(&omap_pmic_mpu, "mpu");
	omap_voltage_register_pmic(&omap_pmic_iva, "iva");
	omap_voltage_init_vc(&vc_config);

	samsung_omap4_pwr_key_irq_init();
#if (CONFIG_SAMSUNG_OMAP4_TAB_REV == 4 || \
	CONFIG_SAMSUNG_OMAP4_TAB_REV == 5 || \
	CONFIG_SAMSUNG_OMAP4_TAB_REV == 6 || \
	CONFIG_SAMSUNG_OMAP4_TAB_REV == 7)
	spi_register_board_info(omap4tab_spi_board_info,
                                ARRAY_SIZE(omap4tab_spi_board_info));
#endif
	omap4430univ_sensors_init();
#if (CONFIG_SAMSUNG_OMAP4_TAB_REV == 3)
	samsung_omap4_battery_init();
#endif
#ifdef DISABLE_I2C2_HW_SPINLOCK_FOR_REV5
	m5mo_fref_start();
#endif
#if (CONFIG_SAMSUNG_OMAP4_TAB_REV >= 5)
	set_offmode_padconfig();
#endif
#ifdef CONFIG_SWITCH_GPIO
	omap4tab_headset_init();
#endif

	sec_class = class_create(THIS_MODULE, "sec");
	if (IS_ERR(sec_class))
		pr_err("Class(sec) Creating Fail!!!\n");

#if ((CONFIG_SAMSUNG_OMAP4_TAB_REV == 6) \
         || (CONFIG_SAMSUNG_OMAP4_TAB_REV == 7))
	gsd4t_gps_init();
#endif
}

static void __init omap4_tab_10_1_map_io(void)
{
	omap2_set_globals_443x();
	omap44xx_map_common_io();
}

MACHINE_START(OMAP4_TAB_10_1, "Samsung OMAP4 based 10.1 Tablet board")
	/* Maintainer: Shankar Bandal Samsung India (shankar.b@samsung.com) */
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xfa000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= omap4_tab_10_1_map_io,
	.init_irq	= omap4_tab_10_1_init_irq,
	.init_machine	= omap4_tab_10_1_init,
	.timer		= &omap_timer,
MACHINE_END












