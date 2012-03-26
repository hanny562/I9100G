/*
 * Driver for keys on GPIO lines capable of generating interrupts.
 *
 * Copyright 2005 Phil Blundell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <asm/gpio.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/earlysuspend.h>
#include <asm/io.h>
#include "cypress_common.h"
#include <plat/control.h>
#include <plat/mux.h>
#if defined(CONFIG_MACH_SAMSUNG_T1) && (CONFIG_SAMSUNG_REL_HW_REV >= 3)
#include <plat/i2c-omap-gpio.h>
#endif
#include <linux/spinlock.h>
#include <linux/regulator/machine.h>

static DEFINE_SPINLOCK(touchkey_lock);
static int i2c_lock;

#ifdef CONFIG_MACH_OMAP4_TAB_10_1
#if (CONFIG_SAMSUNG_OMAP4_TAB_REV <= 4)
#define _3_GPIO_TOUCH_EN	51
#define _3_GPIO_TOUCH_LED_EN	51
#define _3_GPIO_TOUCH_INT	32
#elif (CONFIG_SAMSUNG_OMAP4_TAB_REV == 5 \
	|| (CONFIG_SAMSUNG_OMAP4_TAB_REV == 6) \
 	|| (CONFIG_SAMSUNG_OMAP4_TAB_REV == 7))

#define _3_GPIO_TOUCH_EN        101
#define _3_GPIO_TOUCH_LED_EN	102
#define _3_GPIO_TOUCH_INT       32
#define IRQ_TOUCH_INT		OMAP_GPIO_IRQ(_3_GPIO_TOUCH_INT)
#endif
#else
#define _3_GPIO_TOUCH_EN        101
#define _3_GPIO_TOUCH_LED_EN	102
#define _3_GPIO_TOUCH_INT       32

#endif /* CONFIG_MACH_OMAP4_TAB_10_1 */

/* overwrite default configurations for OMAP4-Samsung Devices */
#if defined(CONFIG_MACH_OMAP4_SAMSUNG)
#define _3_GPIO_TOUCH_EN	OMAP_GPIO_TOUCH_EN
#define _3_GPIO_TOUCH_LED_EN	OMAP_GPIO_TOUCH_LED_EN
#define _3_GPIO_TOUCH_INT	OMAP_GPIO_TOUCH_INT
#define IRQ_TOUCH_INT		OMAP_GPIO_IRQ(OMAP_GPIO_TOUCH_INT)
#endif /* CONFIG_MACH_OMAP4_SAMSUNG */

#define OMAP_MUX_MODE0      0
#define OMAP_MUX_MODE3      3
#define OMAP_PULL_ENA			(1 << 3)
#define OMAP_PULL_UP			(1 << 4)
#define OMAP_INPUT_EN			(1 << 8)
#define OMAP_PIN_INPUT_PULLUP		(OMAP_PULL_ENA | OMAP_INPUT_EN | OMAP_PULL_UP)
#define OMAP_PIN_INPUT_PULLDOWN		(OMAP_PULL_ENA | OMAP_INPUT_EN)

#define SET_TOUCH_I2C()				omap4_ctrl_pad_writel(((OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP) << 16) | (OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP), 0x012A)

#define SET_TOUCH_I2C_TO_GPIO()		omap4_ctrl_pad_writel(((OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP) << 16) | (OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP), 0x012A)

#define SET_TOUCH_I2C_TO_PD()		omap4_ctrl_pad_writel(((OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLDOWN) << 16) | (OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLDOWN), 0x012A)

/*
Melfas touchkey register
*/
#define KEYCODE_REG 0x00
#define FIRMWARE_VERSION 0x01
#define TOUCHKEY_MODULE_VERSION 0x02
#define CMD_REG 0x03
#define THRESHOLD_REG 0x04
#define SENS_REG 0x05
#define IDAC_REG 0x06
#define DIFF_DATA_REG 0x0A
#define RAW_DATA_REG 0x0E
#define BASELINE_REG 0x12

/* Command for 0x00 */
#if defined(CONFIG_MACH_SAMSUNG_Q1)
#define LED_ON_CMD 0x01
#define LED_OFF_CMD 0x02
#else
#define LED_ON_CMD 0x10
#define LED_OFF_CMD 0x20
#endif
#define SENS_EN_CMD 0x40
#define AUTO_CAL_MODE_CMD 0x50
#define SLEEP_CMD 0x80

/* Command for 0x03 */
#define AUTO_CAL_EN_CMD 0x01

#define TOUCHKEY_ADDRESS	0x20

#define UPDOWN_EVENT_BIT 0x08
#define KEYCODE_BIT 0x07
#define COMMAND_BIT 0xF0

#define I2C_M_WR 0		/* for i2c */

#define DEVICE_NAME "sec_touchkey"

#if defined(CONFIG_MACH_SAMSUNG_Q1)
#define TOUCH_FIRMWARE_V03  0x03
#define CURRENT_FIRMWARE_VERSION TOUCH_FIRMWARE_V03
#else
#define TOUCH_FIRMWARE_V0B  0x0B
#define CURRENT_FIRMWARE_VERSION TOUCH_FIRMWARE_V0B
#endif

#define DOOSUNGTECH_TOUCH_V1_2  0x0C

extern bool tsp_deepsleep;

static int touchkey_keycode[3] = { 0, KEY_MENU, KEY_BACK };

static u16 menu_sensitivity = 0;
static u16 back_sensitivity = 0;
static u16 raw_data0 = 0;
static u16 raw_data1 = 0;
static u8 idac0 = 0;
static u8 idac1 = 0;
static int touchkey_enable = 0;
static int force_firm_update = 0;
/*sec_class sysfs*/
extern struct class *sec_class;
struct device *sec_touchkey;

extern int fmradio_on;

struct i2c_touchkey_driver *touchkey_driver = NULL;
struct work_struct touchkey_work;
#if 0
struct work_struct led_work;
#endif
struct workqueue_struct *touchkey_wq;

struct work_struct touch_update_work;
struct delayed_work touch_resume_work;

static const struct i2c_device_id melfas_touchkey_id[] = {
	{"melfas_touchkey", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, melfas_touchkey_id);

#if defined(CONFIG_MACH_SAMSUNG_T1) && (CONFIG_SAMSUNG_REL_HW_REV >= 3)
	static OMAP_GPIO_I2C_CLIENT * tk_client;
#endif

static void init_hw(void);
static int i2c_touchkey_probe(struct i2c_client *client,
			      const struct i2c_device_id *id);

static u8 touchkey_led_status;

struct i2c_driver touchkey_i2c_driver = {
	.driver = {
		   .name = "melfas_touchkey_driver",
		   },
	.id_table = melfas_touchkey_id,
	.probe = i2c_touchkey_probe,
};

static int touch_version = 0;
static int module_version = 0;
extern int touch_is_pressed;

#if defined(CONFIG_MACH_SAMSUNG_T1) && (CONFIG_SAMSUNG_REL_HW_REV >= 3)
static int enable_int_pull_up(int enable)
{
	struct regulator *regulator;

	if(tsp_deepsleep) return 1;

	regulator = regulator_get(&touchkey_driver->client->dev, "vaux2");
	if (IS_ERR(regulator)) {
		printk("*** %s [%d] failed to get VAUX2 regulator. \n",
		       __func__, __LINE__);
		return 0;
	}
	if(enable){
		regulator_enable(regulator);
	} else {
		if (regulator_is_enabled(regulator))
			regulator_force_disable(regulator);
	}
	regulator_put(regulator);

	return 1;
}
#endif


static ssize_t brightness_control(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t size)
{
	int data;
	if (sscanf(buf, "%d\n", &data) == 1) {
		printk(KERN_ERR "[TouchKey] touch_led_brightness: %d \n", data);
/* TODO : Implement brightness control routine */
	} else {
		printk(KERN_ERR "[TouchKey] touch_led_brightness Error\n");
	}

	return size;
}

static int i2c_touchkey_read_legacy(u8 reg, u8 * val, unsigned int len)
{
	int err = 0;
	int retry = 10;
	u8 buf[reg+len];
#if defined(CONFIG_MACH_SAMSUNG_T1) && (CONFIG_SAMSUNG_REL_HW_REV >= 7)
	OMAP_GPIO_I2C_RD_DATA i2c_rd_param;
#elif defined(CONFIG_MACH_SAMSUNG_T1) && (CONFIG_SAMSUNG_REL_HW_REV == 3)
	OMAP_GPIO_I2C_RD_DATA i2c_rd_param;
	struct i2c_msg msg[1];
#else
	struct i2c_msg msg[1];
#endif

	if ((touchkey_driver == NULL)) {
		printk(KERN_ERR "[TouchKey] touchkey is not enabled.R\n");
		return -ENODEV;
	}

	while (retry--) {
#if defined(CONFIG_MACH_SAMSUNG_T1) && (CONFIG_SAMSUNG_REL_HW_REV >= 7)
		i2c_rd_param.reg_len = 0;
		i2c_rd_param.reg_addr = NULL;
		i2c_rd_param.rdata_len = len;
		i2c_rd_param.rdata = buf;
		err = omap_gpio_i2c_read(tk_client, &i2c_rd_param);
#elif defined(CONFIG_MACH_SAMSUNG_T1) && (CONFIG_SAMSUNG_REL_HW_REV == 3)
		if(system_rev >= 4) {
			i2c_rd_param.reg_len = 0;
			i2c_rd_param.reg_addr = NULL;
			i2c_rd_param.rdata_len = len;
			i2c_rd_param.rdata = buf;
			err = omap_gpio_i2c_read(tk_client, &i2c_rd_param);
		}
		else {
			msg->addr = touchkey_driver->client->addr;
			msg->flags = I2C_M_RD;
			msg->len = len;
			msg->buf = buf;
			err = i2c_transfer(touchkey_driver->client->adapter, msg, 1);
		}
#else
		msg->addr = touchkey_driver->client->addr;
		msg->flags = I2C_M_RD;
		msg->len = len;
		msg->buf = buf;
		err = i2c_transfer(touchkey_driver->client->adapter, msg, 1);
#endif
		if (err >= 0) {
			memcpy(val, buf+reg, len);
			return 0;
		}
		printk(KERN_ERR "[TouchKey] %s %d i2c transfer error\n", __func__, __LINE__);	/* add by inter.park */
		msleep(10);
	}
	return err;
}

static int i2c_touchkey_read(u8 reg, u8 * val, unsigned int len)
{
#if defined(CONFIG_MACH_SAMSUNG_Q1)
	return i2c_touchkey_read_legacy(reg, val, len);
#else
	int err = 0;
	int retry = 10;
#if defined(CONFIG_MACH_SAMSUNG_T1) && (CONFIG_SAMSUNG_REL_HW_REV >= 7)
	OMAP_GPIO_I2C_RD_DATA i2c_rd_param;
#elif defined(CONFIG_MACH_SAMSUNG_T1) && (CONFIG_SAMSUNG_REL_HW_REV == 3)
	OMAP_GPIO_I2C_RD_DATA i2c_rd_param;
	struct i2c_msg msg[2];
#else
	struct i2c_msg msg[2];
#endif

	if ((touchkey_driver == NULL)) {
		printk(KERN_ERR "[TouchKey] touchkey is not enabled.R\n");
		return -ENODEV;
	}

	while (retry--) {
#if defined(CONFIG_MACH_SAMSUNG_T1) && (CONFIG_SAMSUNG_REL_HW_REV >= 7)
		i2c_rd_param.reg_len = 1;
		i2c_rd_param.reg_addr = &reg;
		i2c_rd_param.rdata_len = len;
		i2c_rd_param.rdata = val;
		err = omap_gpio_i2c_smbus_read(tk_client, &i2c_rd_param);
#elif defined(CONFIG_MACH_SAMSUNG_T1) && (CONFIG_SAMSUNG_REL_HW_REV == 3)
		if(system_rev >= 4) {
			i2c_rd_param.reg_len = 1;
			i2c_rd_param.reg_addr = &reg;
			i2c_rd_param.rdata_len = len;
			i2c_rd_param.rdata = val;
			err = omap_gpio_i2c_smbus_read(tk_client, &i2c_rd_param);
		}
		else {
			msg[0].addr = touchkey_driver->client->addr;
			msg[0].flags = I2C_M_WR;
			msg[0].len = 1;
			msg[0].buf = &reg;

			msg[1].addr = touchkey_driver->client->addr;
			msg[1].flags = I2C_M_RD;
			msg[1].len = len;
			msg[1].buf = val;

			err = i2c_transfer(touchkey_driver->client->adapter, msg, 2);
		}
#else
		msg[0].addr = touchkey_driver->client->addr;
		msg[0].flags = I2C_M_WR;
		msg[0].len = 1;
		msg[0].buf = &reg;

		msg[1].addr = touchkey_driver->client->addr;
		msg[1].flags = I2C_M_RD;
		msg[1].len = len;
		msg[1].buf = val;

		err = i2c_transfer(touchkey_driver->client->adapter, msg, 2);
#endif
		if (err >= 0) {
			return 0;
		}
		printk(KERN_ERR "[TouchKey] %s %d i2c transfer error\n", __func__, __LINE__);	/* add by inter.park */
		msleep(10);
	}
	return err;
#endif
}

#if defined(CONFIG_MACH_SAMSUNG_Q1)
static int i2c_touchkey_write_legacy(u8 * val, unsigned int len)
{
	int err = 0;
	struct i2c_msg msg[1];
	unsigned char data[2];
	int retry = 2;

	if ((touchkey_driver == NULL) || !(touchkey_enable == 1)) {
		//printk(KERN_ERR "[TouchKey] touchkey is not enabled.\n");
		return -ENODEV;
	}

	while (retry--) {
		data[0] = *val;
		msg->addr = touchkey_driver->client->addr;
		msg->flags = I2C_M_WR;
		msg->len = len;
		msg->buf = data;
		err = i2c_transfer(touchkey_driver->client->adapter, msg, 1);
		//printk("write value %d to address %d\n",*val, msg->addr);
		if (err >= 0) {
			
			return 0;
			
		}
		printk(KERN_DEBUG "[TouchKey] %s %d i2c transfer error\n",
		       __func__, __LINE__);
		msleep(10);
	}
	return err;
}
#endif

static int i2c_touchkey_write(u8 reg, u8 * val, unsigned int len)
{
#if defined(CONFIG_MACH_SAMSUNG_Q1)
	return i2c_touchkey_write_legacy(val, len);
#else
	int err = 0;
#if defined(CONFIG_MACH_SAMSUNG_T1) && (CONFIG_SAMSUNG_REL_HW_REV >= 7)
    OMAP_GPIO_I2C_WR_DATA i2c_wr_param;
#elif defined(CONFIG_MACH_SAMSUNG_T1) && (CONFIG_SAMSUNG_REL_HW_REV == 3)
    OMAP_GPIO_I2C_WR_DATA i2c_wr_param;
	struct i2c_msg msg[1];
	unsigned char data[len+1];
#else
	struct i2c_msg msg[1];
	unsigned char data[len+1];
#endif
	int retry = 2;

	if ((touchkey_driver == NULL) || !(touchkey_enable == 1)) {
		//printk(KERN_ERR "[TouchKey] touchkey is not enabled.\n");
		return -ENODEV;
	}

	while (retry--) {
#if defined(CONFIG_MACH_SAMSUNG_T1) && (CONFIG_SAMSUNG_REL_HW_REV >= 7)
		i2c_wr_param.reg_len = 1;
		i2c_wr_param.reg_addr = &reg;
		i2c_wr_param.wdata_len = len;
		i2c_wr_param.wdata = val;
		err = omap_gpio_i2c_smbus_write(tk_client, &i2c_wr_param);
#elif defined(CONFIG_MACH_SAMSUNG_T1) && (CONFIG_SAMSUNG_REL_HW_REV == 3)
		if(system_rev >= 4) {
			i2c_wr_param.reg_len = 1;
			i2c_wr_param.reg_addr = &reg;
			i2c_wr_param.wdata_len = len;
			i2c_wr_param.wdata = val;
			err = omap_gpio_i2c_smbus_write(tk_client, &i2c_wr_param);
		}
		else {
			data[0] = reg;
			memcpy(data + 1, val, len);
			msg->addr = touchkey_driver->client->addr;
			msg->flags = I2C_M_WR;
			msg->len = len+1;
			msg->buf = data;
			err = i2c_transfer(touchkey_driver->client->adapter, msg, 1);
		}
#else
		data[0] = reg;
		memcpy(data + 1, val, len);
		msg->addr = touchkey_driver->client->addr;
		msg->flags = I2C_M_WR;
		msg->len = len+1;
		msg->buf = data;
		err = i2c_transfer(touchkey_driver->client->adapter, msg, 1);
#endif
		//printk("write value %d to address %d\n",*val, msg->addr);
		if (err >= 0) {
			return 0;
		}
		printk(KERN_DEBUG "[TouchKey] %s %d i2c transfer error\n",
		       __func__, __LINE__);
		msleep(10);
	}
	return err;
#endif
}

void touchkey_work_func(struct work_struct *p)
{
	u8 data;
	int ret;

	if (!gpio_get_value(_3_GPIO_TOUCH_INT)) {
		while(1) { 
			spin_lock(&touchkey_lock);
			if(i2c_lock == 0) {
				break;
			}
			spin_unlock(&touchkey_lock);
		};
		i2c_lock = 1;
		spin_unlock(&touchkey_lock);
		ret = i2c_touchkey_read(KEYCODE_REG, &data, 1);
		spin_lock(&touchkey_lock);
		i2c_lock = 0;
		spin_unlock(&touchkey_lock);
#if defined(CONFIG_MACH_SAMSUNG_Q1)
		msleep(5);
#endif		
		enable_irq(IRQ_TOUCH_INT);
		/******************************************************************
		typedef struct I2CReg 
		{ 
			unsigned char	 BtnStatus; 							 // 0 :  
			unsigned char	 Version;								  // 1 :FW Version 
			unsigned char	 PcbStatus; 							 // 2 :Module Version 
			unsigned char	 Cmd;									  // 3 : 
			unsigned char	 Chip_id;								  // 4 :0x55(DEFAULT_CHIP_ID) 0 
			unsigned char	 Sens;									   // 5 :sensitivity grade(0x00(slow),0x01(mid),0x02(fast)) 
			unsigned char	 SetIdac[4];									   // 6~9
			WORD			 DiffData[CSD_TotalSensorCount];   // 10,11 - 12,13
			WORD			 RawData[CSD_TotalSensorCount];  // 14,15 - 16,17 
			WORD			 Baseline[CSD_TotalSensorCount];   // 18,19 - 20,21 
		}I2CReg; 
		******************************************************************/

		if(data & COMMAND_BIT) 	{
#if defined(CONFIG_SAMSUNG_KERNEL_DEBUG_USER)
/*			printk(KERN_DEBUG "[TouchKey] CMD %#02x\n", data); */
#endif
			return;
		}

		if (data & UPDOWN_EVENT_BIT) {
			input_report_key(touchkey_driver->input_dev,
					 touchkey_keycode[data &
							  KEYCODE_BIT], 0);
			input_sync(touchkey_driver->input_dev);
#if defined(CONFIG_SAMSUNG_KERNEL_DEBUG_USER)
/*			printk(KERN_DEBUG "[TouchKey] release keycode:%d \n",
			       touchkey_keycode[data & KEYCODE_BIT]); */
#endif
		} else {
			if (touch_is_pressed) {
#if defined(CONFIG_SAMSUNG_KERNEL_DEBUG_USER)
/*				printk(KERN_DEBUG
				       "[TouchKey] touchkey pressed but don't send event because touch is pressed. \n"); */
#endif
			} else {
				input_report_key(touchkey_driver->input_dev,
						 touchkey_keycode[data &
								  KEYCODE_BIT],
						 1);
				input_sync(touchkey_driver->input_dev);
#if defined(CONFIG_SAMSUNG_KERNEL_DEBUG_USER)
/*				printk(KERN_DEBUG
				       "[TouchKey] press keycode:%d \n",
				       touchkey_keycode[data & KEYCODE_BIT]); */
#endif
			}
		}
#if defined(CONFIG_SAMSUNG_KERNEL_DEBUG_USER)
/*		printk(KERN_ERR"## TOUCH KEY - CODE : %d\n ", touchkey_keycode[data & KEYCODE_BIT]); */
#endif
	}
}

static irqreturn_t touchkey_interrupt(int irq, void *dummy)
{
	disable_irq_nosync(IRQ_TOUCH_INT);
	queue_work(touchkey_wq, &touchkey_work);

	return IRQ_HANDLED;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static int melfas_touchkey_early_suspend(struct early_suspend *h)
{
#if defined(CONFIG_MACH_SAMSUNG_T1) && (CONFIG_SAMSUNG_REL_HW_REV == 3)
	unsigned char sleep_cmd = SLEEP_CMD;
#endif
	while(1) { 
		spin_lock(&touchkey_lock);
		if(i2c_lock == 0) {
			break;
		}
		spin_unlock(&touchkey_lock);
	};
	i2c_lock = 1;
	spin_unlock(&touchkey_lock);
	touchkey_enable = 0;
	spin_lock(&touchkey_lock);
	i2c_lock = 0;
	spin_unlock(&touchkey_lock);
	printk(KERN_DEBUG "[TouchKey] melfas_touchkey_early_suspend\n");
	if (touchkey_enable < 0) {
		printk(KERN_DEBUG "[TouchKey] ---%s---touchkey_enable: %d\n",
		       __func__, touchkey_enable);
		return 0;
	}

	disable_irq(IRQ_TOUCH_INT);
	flush_workqueue(touchkey_wq);
#if defined(CONFIG_MACH_SAMSUNG_T1) && (CONFIG_SAMSUNG_REL_HW_REV >= 7)
	gpio_direction_output(_3_GPIO_TOUCH_LED_EN, 0);
	gpio_direction_output(_3_GPIO_TOUCH_EN, 0);
	gpio_direction_output(OMAP_GPIO_3_TOUCH_SCL, 0);
	gpio_direction_output(OMAP_GPIO_3_TOUCH_SDA, 0);
	enable_int_pull_up(0);
#elif defined(CONFIG_MACH_SAMSUNG_T1) && (CONFIG_SAMSUNG_REL_HW_REV == 3)
	if(system_rev >= 4) {
		gpio_direction_output(_3_GPIO_TOUCH_LED_EN, 0);
		gpio_direction_output(_3_GPIO_TOUCH_EN, 0);
		gpio_direction_output(OMAP_GPIO_3_TOUCH_SCL, 0);
		gpio_direction_output(OMAP_GPIO_3_TOUCH_SDA, 0);
		enable_int_pull_up(0);
	}
	else {
		if(fmradio_on) {
			gpio_direction_output(_3_GPIO_TOUCH_INT, 1);
			while(1) { 
				spin_lock(&touchkey_lock);
				if(i2c_lock == 0) {
					break;
				}
				spin_unlock(&touchkey_lock);
			};
			i2c_lock = 1;
			spin_unlock(&touchkey_lock);
			touchkey_enable = 1;
			i2c_touchkey_write(KEYCODE_REG, &sleep_cmd, 1);
			touchkey_enable = 0;
			spin_lock(&touchkey_lock);
			i2c_lock = 0;
			spin_unlock(&touchkey_lock);
			gpio_direction_output(OMAP_GPIO_TSP_EN, 0);
			gpio_direction_output(_3_GPIO_TOUCH_LED_EN, 0);
		}
		else {
			gpio_direction_output(_3_GPIO_TOUCH_LED_EN, 0);
			gpio_direction_output(_3_GPIO_TOUCH_EN, 0);
			gpio_direction_output(OMAP_GPIO_TSP_EN, 0);
			SET_TOUCH_I2C_TO_PD();
		}
	}
#else
	gpio_direction_output(_3_GPIO_TOUCH_LED_EN, 0);
	gpio_direction_output(_3_GPIO_TOUCH_EN, 0);
	gpio_direction_output(OMAP_GPIO_TSP_EN, 0);
	SET_TOUCH_I2C_TO_PD();
#endif

	/* releae key */
	input_report_key(touchkey_driver->input_dev,
			 touchkey_keycode[1], 0);
	input_report_key(touchkey_driver->input_dev,
			 touchkey_keycode[2], 0);
	return 0;
}

static int melfas_touchkey_late_resume(struct early_suspend *h)
{
	u8 cmd;

	printk(KERN_DEBUG "[TouchKey] melfas_touchkey_late_resume\n");
#if defined(CONFIG_MACH_SAMSUNG_T1) && (CONFIG_SAMSUNG_REL_HW_REV >= 7)
	enable_int_pull_up(1);
	gpio_direction_output(OMAP_GPIO_3_TOUCH_SCL, 1);
	gpio_direction_output(OMAP_GPIO_3_TOUCH_SDA, 1);
	gpio_direction_output(_3_GPIO_TOUCH_EN, 1);
#elif defined(CONFIG_MACH_SAMSUNG_T1) && (CONFIG_SAMSUNG_REL_HW_REV == 3)
	if(system_rev >= 4)
	{
		enable_int_pull_up(1);
		gpio_direction_output(OMAP_GPIO_3_TOUCH_SCL, 1);
		gpio_direction_output(OMAP_GPIO_3_TOUCH_SDA, 1);
		gpio_direction_output(_3_GPIO_TOUCH_EN, 1);
	}
	else {
		if(fmradio_on) {
			gpio_direction_output(_3_GPIO_TOUCH_INT, 0);
			msleep(1);
			gpio_direction_output(_3_GPIO_TOUCH_INT, 1);
			gpio_direction_input(_3_GPIO_TOUCH_INT);
			gpio_direction_output(OMAP_GPIO_TSP_EN, 1);
		}
		else {
			SET_TOUCH_I2C();
			gpio_direction_output(OMAP_GPIO_TSP_EN, 1);
			gpio_direction_output(_3_GPIO_TOUCH_EN, 1);
		}
	}
#else
		SET_TOUCH_I2C();
		gpio_direction_output(OMAP_GPIO_TSP_EN, 1);
		gpio_direction_output(_3_GPIO_TOUCH_EN, 1);
#endif
	if (touchkey_enable < 0) {
		printk(KERN_DEBUG "[TouchKey] ---%s---touchkey_enable: %d\n",
		       __func__, touchkey_enable);
		return 0;
	}
	msleep(50);
	gpio_direction_output(_3_GPIO_TOUCH_LED_EN, 1);

	enable_irq(IRQ_TOUCH_INT);
	touchkey_enable = 1;

	while(1) { 
		spin_lock(&touchkey_lock);
		if(i2c_lock == 0) {
			break;
		}
		spin_unlock(&touchkey_lock);
	};
	i2c_lock = 1;
	spin_unlock(&touchkey_lock);

#if defined(CONFIG_MACH_SAMSUNG_T1)
	cmd = AUTO_CAL_MODE_CMD;
	i2c_touchkey_write(KEYCODE_REG, &cmd, 1);
	cmd = AUTO_CAL_EN_CMD;
	i2c_touchkey_write(CMD_REG, &cmd, 1);
#endif

	msleep(50);

	if(touchkey_led_status == LED_ON_CMD) {
		i2c_touchkey_write(KEYCODE_REG, &touchkey_led_status, 1);
		printk("LED returned on\n");
	}
	spin_lock(&touchkey_lock);
	i2c_lock = 0;
	spin_unlock(&touchkey_lock);

	return 0;
}
#endif

extern int mcsdl_download_binary_data(void);
static int i2c_touchkey_probe(struct i2c_client *client,
			      const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct input_dev *input_dev;
	int err = 0;
	u8 data[3] = { 0, };
	u8 cmd;

	printk(KERN_DEBUG "[TouchKey] melfas i2c_touchkey_probe\n");

	touchkey_driver =
	    kzalloc(sizeof(struct i2c_touchkey_driver), GFP_KERNEL);
	if (touchkey_driver == NULL) {
		dev_err(dev, "failed to create our state\n");
		return -ENOMEM;
	}

	touchkey_driver->client = client;
	touchkey_driver->client->irq = IRQ_TOUCH_INT;
	strlcpy(touchkey_driver->client->name, "melfas-touchkey",
		I2C_NAME_SIZE);
	touchkey_driver->client->dev.init_name = DEVICE_NAME;

	input_dev = input_allocate_device();

	if (!input_dev) {
		return -ENOMEM;
	}

	touchkey_driver->input_dev = input_dev;

	input_dev->name = DEVICE_NAME;
	input_dev->phys = "melfas-touchkey/input0";
	input_dev->id.bustype = BUS_HOST;

	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(touchkey_keycode[1], input_dev->keybit);
	set_bit(touchkey_keycode[2], input_dev->keybit);

	err = input_register_device(input_dev);
	if (err) {
		input_free_device(input_dev);
		return err;
	}

	init_hw();

	gpio_direction_output(_3_GPIO_TOUCH_LED_EN, 1);
	gpio_direction_output(_3_GPIO_TOUCH_EN, 1);

	msleep(50);

	while(1) { 
		spin_lock(&touchkey_lock);
		if(i2c_lock == 0) {
			break;
		}
		spin_unlock(&touchkey_lock);
	};
	i2c_lock = 1;
	spin_unlock(&touchkey_lock);
	err = i2c_touchkey_read_legacy(KEYCODE_REG, data, 3);
	spin_lock(&touchkey_lock);
	i2c_lock = 0;
	spin_unlock(&touchkey_lock);

	if (err) {
		printk(KERN_ERR "[TouchKey] touch key is not connected\n");
		input_free_device(input_dev);
		return err;
	}

	touch_version = data[1];
	module_version = data[2];

	printk("[touchkey] firm ver. = 0x%x, mod ver. = 0x%x\n", touch_version, module_version);
#if defined(CONFIG_MACH_SAMSUNG_T1)
	if(touch_version < CURRENT_FIRMWARE_VERSION || touch_version == 0xFF) {
		printk("[touchkey] Force firmware update\n");
		force_firm_update = 1;
		queue_work(touchkey_wq, &touch_update_work);
	}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	touchkey_driver->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 2;
	touchkey_driver->early_suspend.suspend = (void*) melfas_touchkey_early_suspend;
	touchkey_driver->early_suspend.resume = (void*) melfas_touchkey_late_resume;
	register_early_suspend(&touchkey_driver->early_suspend);
#endif				/* CONFIG_HAS_EARLYSUSPEND */

	touchkey_enable = 1;

#if defined(CONFIG_MACH_SAMSUNG_T1)
	cmd = AUTO_CAL_MODE_CMD;
	i2c_touchkey_write(KEYCODE_REG, &cmd, 1);
	cmd = AUTO_CAL_EN_CMD;
	i2c_touchkey_write(CMD_REG, &cmd, 1);
#endif

	if (request_irq
	    (IRQ_TOUCH_INT, touchkey_interrupt, IRQF_DISABLED, DEVICE_NAME,
	     NULL)) {
		printk(KERN_ERR "[TouchKey] %s Can't allocate irq ..\n",
		       __func__);
		return -EBUSY;
	}

	return 0;
}

static void init_hw(void)
{
	gpio_direction_output(_3_GPIO_TOUCH_EN, 1);
	msleep(50);
	set_irq_type(IRQ_TOUCH_INT, IRQ_TYPE_LEVEL_LOW);
}

int touchkey_update_open(struct inode *inode, struct file *filp)
{
	return 0;
}

ssize_t touchkey_update_read(struct file * filp, char *buf, size_t count,
			     loff_t * f_pos)
{
	char data[3] = { 0, };
	while(1) { 
		spin_lock(&touchkey_lock);
		if(i2c_lock == 0) {
			break;
		}
		spin_unlock(&touchkey_lock);
	};
	i2c_lock = 1;
	spin_unlock(&touchkey_lock);
	i2c_touchkey_read(KEYCODE_REG, data, 3);
	spin_lock(&touchkey_lock);
	i2c_lock = 0;
	spin_unlock(&touchkey_lock);
	put_user(data[1], buf);

	return 1;
}

int touchkey_update_release(struct inode *inode, struct file *filp)
{
	return 0;
}

struct file_operations touchkey_update_fops = {
	.owner = THIS_MODULE,
	.read = touchkey_update_read,
	.open = touchkey_update_open,
	.release = touchkey_update_release,
};

static struct miscdevice touchkey_update_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "melfas_touchkey",
	.fops = &touchkey_update_fops,
};

extern int ISSP_main(void);
static int touchkey_update_status = 0;

void touchkey_update_func(struct work_struct *p)
{
	int retry = 20;
	u8 cmd;

	touchkey_update_status = 1;
	printk(KERN_DEBUG "[TouchKey] %s start\n", __func__);
	touchkey_enable = 0;
	if(system_rev < 4) {
		disable_irq(gpio_to_irq(OMAP_GPIO_TSP_nINT));	
		gpio_direction_output(OMAP_GPIO_TSP_EN, 0);
		SET_TOUCH_I2C_TO_GPIO();
		gpio_request(OMAP_GPIO_AP_I2C_SDA, "AP_I2C_SDA");
		gpio_request(OMAP_GPIO_AP_I2C_SCL, "AP_I2C_SCL");
	}
	while (retry--) {
		if (ISSP_main() == 0) {
			msleep(100);			
			touchkey_enable = 1;
			if(system_rev < 4) {
				gpio_free(OMAP_GPIO_AP_I2C_SDA);
				gpio_free(OMAP_GPIO_AP_I2C_SCL);
				SET_TOUCH_I2C();
			}
#if defined(CONFIG_MACH_SAMSUNG_T1) && (CONFIG_SAMSUNG_REL_HW_REV >= 3)
			else {
				gpio_direction_output(OMAP_GPIO_3_TOUCH_SCL, 1);
				gpio_direction_output(OMAP_GPIO_3_TOUCH_SDA, 1);
			}
#endif
			touchkey_update_status = 0;
			printk(KERN_DEBUG
			       "[TouchKey] touchkey_update succeeded\n");

#if defined(CONFIG_MACH_SAMSUNG_T1)
			cmd = AUTO_CAL_MODE_CMD;
			i2c_touchkey_write(KEYCODE_REG, &cmd, 1);
			cmd = AUTO_CAL_EN_CMD;
			i2c_touchkey_write(CMD_REG, &cmd, 1);
#endif

			if (force_firm_update) {
				force_firm_update = 0;
			}
			else enable_irq(IRQ_TOUCH_INT);
			if (system_rev < 4) {
				gpio_direction_output(OMAP_GPIO_TSP_EN, 1);
				enable_irq(gpio_to_irq(OMAP_GPIO_TSP_nINT));
			}
			return;
		}
	}
	if(system_rev < 4) {
		gpio_free(OMAP_GPIO_AP_I2C_SDA);
		gpio_free(OMAP_GPIO_AP_I2C_SCL);
		SET_TOUCH_I2C();
		gpio_direction_output(OMAP_GPIO_TSP_EN, 1);
		enable_irq(gpio_to_irq(OMAP_GPIO_TSP_nINT));
	}
#if defined(CONFIG_MACH_SAMSUNG_T1) && (CONFIG_SAMSUNG_REL_HW_REV >= 3)
	else {
		gpio_direction_output(OMAP_GPIO_3_TOUCH_SCL, 1);
		gpio_direction_output(OMAP_GPIO_3_TOUCH_SDA, 1);
	}
#endif
	touchkey_update_status = -1;
	printk(KERN_DEBUG "[TouchKey] touchkey_update failed\n");
	return;
}

static ssize_t set_touchkey_firm_update_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t size)
{
	printk(KERN_DEBUG "[TouchKey] touchkey firmware update \n");

	if (*buf == 'S' || *buf =='F') {
		if ((*buf !='F' && touch_version >= CURRENT_FIRMWARE_VERSION) && touch_version != 0xFF) {
			touchkey_update_status = 0;
			printk(KERN_DEBUG "[TouchKey] already updated latest version\n");
			return size;
		}
		disable_irq(IRQ_TOUCH_INT);
		queue_work(touchkey_wq, &touch_update_work);
	}
	return size;
}

#if 0
void led_work_func(struct work_struct *p)
{
	while(1) { 
		spin_lock(&touchkey_lock);
		if(i2c_lock == 0) {
			break;
		}
		spin_unlock(&touchkey_lock);
	};
	i2c_lock = 1;
	spin_unlock(&touchkey_lock);
	i2c_touchkey_write(KEYCODE_REG, &touchkey_led_status, 1);
	spin_lock(&touchkey_lock);
	i2c_lock = 0;
	spin_unlock(&touchkey_lock);
	msleep(100);
}
#endif

static ssize_t touch_led_control(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t size)
{
#if 1
	int data;
	int errnum;
	if (sscanf(buf, "%d\n", &data) == 1) {
		printk(KERN_DEBUG "[TouchKey] touch_led_control: %d \n", data);
#if defined(CONFIG_MACH_SAMSUNG_T1)
		data = data<<4;
#endif
		while(1) { 
			spin_lock(&touchkey_lock);
			if(i2c_lock == 0) {
				break;
			}
			spin_unlock(&touchkey_lock);
		};
		i2c_lock = 1;
		spin_unlock(&touchkey_lock);
		errnum = i2c_touchkey_write(KEYCODE_REG, (u8 *)&data, 1);
		spin_lock(&touchkey_lock);
		i2c_lock = 0;
		spin_unlock(&touchkey_lock);
		touchkey_led_status = data;
	} else {
		printk(KERN_DEBUG "[TouchKey] touch_led_control Error\n");
	}

	return size;
#else
	int data;
	if (sscanf(buf, "%d\n", &data) == 1) {
		printk(KERN_DEBUG "[TouchKey] touch_led_control: %d \n", data);
		touchkey_led_status = data << 4;
		queue_work(touchkey_wq, &led_work);
	} else {
		printk(KERN_DEBUG "[TouchKey] touch_led_control Error\n");
	}

	return size;
#endif
}

static ssize_t touchkey_menu_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{	
	u8 data[2];

	printk("called %s \n",__func__);
	while(1) { 
		spin_lock(&touchkey_lock);
		if(i2c_lock == 0) {
			break;
		}
		spin_unlock(&touchkey_lock);
	};
	i2c_lock = 1;
	spin_unlock(&touchkey_lock);
	i2c_touchkey_read(DIFF_DATA_REG, data, 2);
	spin_lock(&touchkey_lock);
	i2c_lock = 0;
	spin_unlock(&touchkey_lock);
	printk("called %s menu sens = %d\n",__func__, menu_sensitivity);
	menu_sensitivity = ((0x00FF&data[0])<<8)|data[1];
	return sprintf(buf,"%d\n",menu_sensitivity);
}

static ssize_t touchkey_back_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{	
	u8 data[2];

	printk("called %s \n",__func__);
	while(1) { 
		spin_lock(&touchkey_lock);
		if(i2c_lock == 0) {
			break;
		}
		spin_unlock(&touchkey_lock);
	};
	i2c_lock = 1;
	spin_unlock(&touchkey_lock);
	i2c_touchkey_read(DIFF_DATA_REG+2, data, 2);
	spin_lock(&touchkey_lock);
	i2c_lock = 0;
	spin_unlock(&touchkey_lock);
	printk("called %s back sens = %d\n",__func__, back_sensitivity);
	back_sensitivity = ((0x00FF&data[0])<<8)|data[1];
	return sprintf(buf,"%d\n",back_sensitivity);
}

static ssize_t touchkey_raw_data0_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	u8 data[2];

	printk("called %s \n",__func__);
	while(1) { 
		spin_lock(&touchkey_lock);
		if(i2c_lock == 0) {
			break;
		}
		spin_unlock(&touchkey_lock);
	};
	i2c_lock = 1;
	spin_unlock(&touchkey_lock);
	i2c_touchkey_read(RAW_DATA_REG, data, 2);
	spin_lock(&touchkey_lock);
	i2c_lock = 0;
	spin_unlock(&touchkey_lock);
	printk("called %s raw_data0 = %d\n",__func__, raw_data0);
	raw_data0 = ((0x00FF&data[0])<<8)|data[1];
	return sprintf(buf,"%d\n",raw_data0);
}


static ssize_t touchkey_raw_data1_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	u8 data[2];

	printk("called %s \n",__func__);
	while(1) { 
		spin_lock(&touchkey_lock);
		if(i2c_lock == 0) {
			break;
		}
		spin_unlock(&touchkey_lock);
	};
	i2c_lock = 1;
	spin_unlock(&touchkey_lock);
	i2c_touchkey_read(RAW_DATA_REG+2, data, 2);
	spin_lock(&touchkey_lock);
	i2c_lock = 0;
	spin_unlock(&touchkey_lock);
	printk("called %s raw_data1 = %d\n",__func__, raw_data1);
	raw_data1 = ((0x00FF&data[0])<<8)|data[1];
	return sprintf(buf,"%d\n",raw_data1);
}

static ssize_t touchkey_idac0_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	u8 data;

	printk("called %s \n",__func__);
	while(1) { 
		spin_lock(&touchkey_lock);
		if(i2c_lock == 0) {
			break;
		}
		spin_unlock(&touchkey_lock);
	};
	i2c_lock = 1;
	spin_unlock(&touchkey_lock);
	i2c_touchkey_read(SENS_REG, &data, 1);
	printk("called %s sens = 0x%x\n",__func__,data);
	if(!(data & 0x80)) {
		spin_lock(&touchkey_lock);
		i2c_lock = 0;
		spin_unlock(&touchkey_lock);
		printk("called %s idac0 = %d\n",__func__,data);
		return sprintf(buf, "\n");
	}
	i2c_touchkey_read(IDAC_REG, &data, 1);
	spin_lock(&touchkey_lock);
	i2c_lock = 0;
	spin_unlock(&touchkey_lock);
	printk("called %s idac0 = %d\n",__func__,data);
	idac0 = data;
	return sprintf(buf,"%d\n",idac0);
}


static ssize_t touchkey_idac1_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	u8 data;

	printk("called %s \n",__func__);
	while(1) { 
		spin_lock(&touchkey_lock);
		if(i2c_lock == 0) {
			break;
		}
		spin_unlock(&touchkey_lock);
	};
	i2c_lock = 1;
	spin_unlock(&touchkey_lock);
	i2c_touchkey_read(SENS_REG, &data, 1);
	printk("called %s sens = 0x%x\n",__func__,data);
	if(!(data & 0x80)) {
		spin_lock(&touchkey_lock);
		i2c_lock = 0;
		spin_unlock(&touchkey_lock);
		printk("called %s idac0 = %d\n",__func__,data);
		return sprintf(buf, "\n");
	}
	i2c_touchkey_read(IDAC_REG+1, &data, 1);
	spin_lock(&touchkey_lock);
	i2c_lock = 0;
	spin_unlock(&touchkey_lock);
	printk("called %s idac1 = %d\n",__func__,data);
	idac1 = data;
	return sprintf(buf,"%d\n",idac1);
}

static ssize_t touch_sensitivity_control(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t size)
{
	unsigned char data = SENS_EN_CMD;
	while(1) { 
		spin_lock(&touchkey_lock);
		if(i2c_lock == 0) {
			break;
		}
		spin_unlock(&touchkey_lock);
	};
	i2c_lock = 1;
	spin_unlock(&touchkey_lock);
	i2c_touchkey_write(KEYCODE_REG, &data, 1);
	spin_lock(&touchkey_lock);
	i2c_lock = 0;
	spin_unlock(&touchkey_lock);
	return size;
}

static ssize_t set_touchkey_firm_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{

    /*TO DO IT */
	int count;
	count = sprintf(buf, "0x%X\n", CURRENT_FIRMWARE_VERSION);
	return count;    

}

static ssize_t set_touchkey_firm_version_read_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	char data[3] = { 0, };
	int count;
	while(1) { 
		spin_lock(&touchkey_lock);
		if(i2c_lock == 0) {
			break;
		}
		spin_unlock(&touchkey_lock);
	};
	i2c_lock = 1;
	spin_unlock(&touchkey_lock);
	i2c_touchkey_read(KEYCODE_REG, data, 3);
	spin_lock(&touchkey_lock);
	i2c_lock = 0;
	spin_unlock(&touchkey_lock);

	count = sprintf(buf, "0x%X\n", data[1]);
	touch_version = data[1];
	printk(KERN_DEBUG "[TouchKey] touch_version_read 0x%X\n", data[1]);
	return count;
}

static ssize_t set_touchkey_firm_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{ 
	int count = 0;

	printk(KERN_DEBUG
	       "[TouchKey] touch_update_read: touchkey_update_status %d\n",
	       touchkey_update_status);

	if (touchkey_update_status == 0) {
		count = sprintf(buf, "PASS\n");
	} else if (touchkey_update_status == 1) {
		count = sprintf(buf, "DOWNLOADING\n");
	} else if (touchkey_update_status == -1) {
		count = sprintf(buf, "FAIL\n");
	}

	return count;
}


static DEVICE_ATTR(brightness, S_IRUGO | S_IWUSR | S_IWGRP, NULL, touch_led_control);
static DEVICE_ATTR(touchkey_menu, S_IRUGO, touchkey_menu_show, NULL);
static DEVICE_ATTR(touchkey_back, S_IRUGO, touchkey_back_show, NULL);
static DEVICE_ATTR(touchkey_raw_data0, S_IRUGO, touchkey_raw_data0_show, NULL);
static DEVICE_ATTR(touchkey_raw_data1, S_IRUGO, touchkey_raw_data1_show, NULL);
static DEVICE_ATTR(touchkey_idac0, S_IRUGO, touchkey_idac0_show, NULL);
static DEVICE_ATTR(touchkey_idac1, S_IRUGO, touchkey_idac1_show, NULL);
static DEVICE_ATTR(touch_sensitivity, S_IRUGO | S_IWUSR | S_IWGRP, NULL, touch_sensitivity_control);
/*20110223N1 firmware sync*/
static DEVICE_ATTR(touchkey_firm_update, S_IWUSR | S_IWGRP, NULL, set_touchkey_firm_update_store);		/* firmware update */
static DEVICE_ATTR(touchkey_firm_update_status, S_IRUGO, set_touchkey_firm_status_show, NULL);	/* firmware update status return */
static DEVICE_ATTR(touchkey_firm_version_phone, S_IRUGO, set_touchkey_firm_version_show, NULL);/* PHONE*/	/* firmware version resturn in phone driver version */
static DEVICE_ATTR(touchkey_firm_version_panel, S_IRUGO, set_touchkey_firm_version_read_show, NULL);/*PART*/	/* firmware version resturn in touchkey panel version */
/*end N1 firmware sync*/

static DEVICE_ATTR(touchkey_brightness, S_IRUGO | S_IWUSR | S_IWGRP, NULL, brightness_control);


static int __init touchkey_init(void)
{
	int ret = 0;

	gpio_request(_3_GPIO_TOUCH_EN, "_3_GPIO_TOUCH_EN");
	gpio_request(_3_GPIO_TOUCH_LED_EN, "_3_GPIO_TOUCH_EN");
	gpio_request(_3_GPIO_TOUCH_INT, "_3_GPIO_TOUCH_INT");

	/*20110222 N1_firmware_sync*/
    sec_touchkey= device_create(sec_class, NULL, 0, NULL, "sec_touchkey");
	if (IS_ERR(sec_touchkey))
		{
			printk(KERN_ERR "Failed to create device(sec_touchkey)!\n");
		}
	if (device_create_file(sec_touchkey, &dev_attr_touchkey_firm_update)< 0)
		{
			printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_touchkey_firm_update.attr.name);
		}
	if (device_create_file(sec_touchkey, &dev_attr_touchkey_firm_update_status)< 0)
		{
			printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_touchkey_firm_update_status.attr.name);
		}
	if (device_create_file(sec_touchkey, &dev_attr_touchkey_firm_version_phone)< 0)
		{
			printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_touchkey_firm_version_phone.attr.name);
		}
	if (device_create_file(sec_touchkey, &dev_attr_touchkey_firm_version_panel)< 0)
		{
			printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_touchkey_firm_version_panel.attr.name);
		}
	if (device_create_file(sec_touchkey, &dev_attr_touchkey_brightness)< 0)
		{
			printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_touchkey_brightness.attr.name);
		}
	/*end N1_firmware_sync*/

	if (device_create_file
	    (sec_touchkey, &dev_attr_brightness) < 0) {
		printk(KERN_ERR
		       "[TouchKey] %s device_create_file fail dev_attr_touch_update\n",
		       __func__);
		pr_err("Failed to create device file(%s)!\n",
		       dev_attr_brightness.attr.name);
	}

	if (device_create_file
		(sec_touchkey, &dev_attr_touchkey_menu) < 0) {
		printk(KERN_ERR 
			"%s device_create_file fail dev_attr_touchkey_menu\n"
			,__func__);
		pr_err("Failed to create device file(%s)!\n", 
			dev_attr_touchkey_menu.attr.name);
	}

	if (device_create_file
		(sec_touchkey, &dev_attr_touchkey_back) < 0) {
		printk(KERN_ERR 
			"%s device_create_file fail dev_attr_touchkey_back\n",
			__func__);
		pr_err("Failed to create device file(%s)!\n", 
			dev_attr_touchkey_back.attr.name);
	}

	if (device_create_file
		(sec_touchkey, &dev_attr_touchkey_raw_data0) < 0) {
		printk(KERN_ERR 
			"%s device_create_file fail dev_attr_touchkey_raw_data0\n",
			__func__);
		pr_err("Failed to create device file(%s)!\n", 
			dev_attr_touchkey_raw_data0.attr.name);
	}
	if (device_create_file
		(sec_touchkey, &dev_attr_touchkey_raw_data1) < 0) {
		printk(KERN_ERR 
			"%s device_create_file fail dev_attr_touchkey_raw_data1\n",
			__func__);
		pr_err("Failed to create device file(%s)!\n", 
			dev_attr_touchkey_raw_data1.attr.name);
	}
	if (device_create_file
		(sec_touchkey, &dev_attr_touchkey_idac0) < 0) {
		printk(KERN_ERR 
			"%s device_create_file fail dev_attr_touchkey_idac0\n",
			__func__);
		pr_err("Failed to create device file(%s)!\n", 
			dev_attr_touchkey_idac0.attr.name);
	}

	if (device_create_file
		(sec_touchkey, &dev_attr_touchkey_idac1) < 0) {
		printk(KERN_ERR 
			"%s device_create_file fail dev_attr_touchkey_idac1\n",
			__func__);
		pr_err("Failed to create device file(%s)!\n", 
			dev_attr_touchkey_idac1.attr.name);
	}
	
	if (device_create_file
		(sec_touchkey, &dev_attr_touch_sensitivity) < 0) {
		printk("%s device_create_file fail dev_attr_touch_sensitivity\n",
			__func__);
		pr_err("Failed to create device file(%s)!\n", 
			dev_attr_touch_sensitivity.attr.name);
	}

	touchkey_wq = create_singlethread_workqueue("melfas_touchkey_wq");
	if (!touchkey_wq) {
		return -ENOMEM;
	}

	INIT_WORK(&touchkey_work, touchkey_work_func);
#if 0
	INIT_WORK(&led_work, led_work_func);
#endif
	INIT_WORK(&touch_update_work, touchkey_update_func);

#if defined(CONFIG_MACH_SAMSUNG_T1) && (CONFIG_SAMSUNG_REL_HW_REV >= 7)
    tk_client = omap_gpio_i2c_init(OMAP_GPIO_3_TOUCH_SDA, OMAP_GPIO_3_TOUCH_SCL, 0x20, 100);

	if(tk_client == NULL) {
		printk(KERN_ERR "[TouchKey] omap_gpio_i2c_init failed!\n");
	}
	printk("[TouchKey] add dummy i2c driver!\n");
#elif defined(CONFIG_MACH_SAMSUNG_T1) && (CONFIG_SAMSUNG_REL_HW_REV == 3)
	if(system_rev >= 4) {
	    tk_client = omap_gpio_i2c_init(OMAP_GPIO_3_TOUCH_SDA, OMAP_GPIO_3_TOUCH_SCL, 0x20, 100);

	    if(tk_client == NULL)
    	{
	        printk(KERN_ERR "[TouchKey] omap_gpio_i2c_init failed!\n");
    	}

		printk("[TouchKey] add dummy i2c driver!\n");
	}
#endif
	ret = i2c_add_driver(&touchkey_i2c_driver);

	if (ret) {
		printk(KERN_ERR
		       "[TouchKey] melfas touch keypad registration failed, module not inserted.ret= %d\n",
		       ret);
	}

	return ret;
}

static void __exit touchkey_exit(void)
{
	printk(KERN_DEBUG "[TouchKey] %s \n", __func__);
	i2c_del_driver(&touchkey_i2c_driver);
	misc_deregister(&touchkey_update_device);

	if (touchkey_wq) {
		destroy_workqueue(touchkey_wq);
	}

	gpio_free(_3_GPIO_TOUCH_EN);
	gpio_free(_3_GPIO_TOUCH_INT);
}

late_initcall(touchkey_init);
module_exit(touchkey_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("@@@");
MODULE_DESCRIPTION("melfas touch keypad");
