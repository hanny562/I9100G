/*
 * module/samsung_battery/fuelgauge_max17043.c
 *
 * SAMSUNG battery driver for Linux
 *
 * Copyright (C) 2009 SAMSUNG ELECTRONICS.
 * Author: EUNGON KIM (egstyle.kim@samsung.com)
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <mach/gpio.h>
#include <linux/power_supply.h>
#include "common.h"

#if defined(CONFIG_USE_GPIO_I2C)
#include <plat/i2c-omap-gpio.h>
#endif

#define DRIVER_NAME "secFuelgaugeDev"

#define I2C_M_WR    0x00

#define REG_VCELL   0x02
#define REG_SOC     0x04
#define REG_MODE    0x06
#define REG_VERSION 0x08
#define REG_RCOMP   0x0C
#define REG_CONFIG  0x0D
#define REG_COMMAND 0xFE

#if defined(CONFIG_USE_GPIO_I2C)
static OMAP_GPIO_I2C_CLIENT *fuelgauge_i2c_client;
static struct i2c_client *fuelgauge_i2c_dummy_client;
#else
static struct i2c_client *fuelgauge_i2c_client;
#endif

struct work_struct fuelgauge_lowbat_alrt_work;

static SEC_battery_charger_info *sec_bci;

/* Prototype */
int get_fuelgauge_adc_value(int);
int get_fuelgauge_ptg_value(int);
int fuelgauge_quickstart(void);
static int i2c_read(unsigned char);
static int i2c_write(unsigned char *, u8);
static irqreturn_t low_battery_isr(int, void *);
static void fuelgauge_lowbat_alrt_work_handler(struct work_struct *);
static int fuelgauge_probe(struct i2c_client *,
			   const struct i2c_device_id *);
static int fuelgauge_remove(struct i2c_client *);
static void fuelgauge_shutdown(struct i2c_client *);
static int fuelgauge_suspend(struct i2c_client *, pm_message_t);
static int fuelgauge_resume(struct i2c_client *);
int fuelgauge_init(void);
void fuelgauge_exit(void);

extern SEC_battery_charger_info *get_sec_bci(void);
extern int _low_battery_alert_(void);

static const struct i2c_device_id fuelgauge_i2c_id[] = {
	{DRIVER_NAME, 0},
	{},
};

static struct i2c_driver fuelgauge_i2c_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},

	.probe = fuelgauge_probe,
	.remove = __devexit_p(fuelgauge_remove),
	.shutdown = fuelgauge_shutdown,
	.suspend = fuelgauge_suspend,
	.resume = fuelgauge_resume,
	.id_table = fuelgauge_i2c_id,
};

int get_fuelgauge_adc_value(int count)
{
	int result;

	result = i2c_read(REG_VCELL);

	result = (result >> 4) * 125 / 100;
	return result;
}

int get_fuelgauge_ptg_value(int value_type)
{
	int val;
	int ret;

	unsigned char buf[2];

	val = i2c_read(REG_SOC);

	buf[0] = (val >> 8);
	buf[1] = (val & 0xFF);

	val = buf[0];

	if (value_type == RAW_SOC)
		return val;

	val = val * 1000;
	val = val + (buf[1]*3);

	if (val <= 300)
		return 0;

	if (val > 300 && val < 1258)
		return 1;

	ret = (val-300)*100/(96100-300);

	return (ret > 100) ? 100 : ret;
}

int fuelgauge_quickstart(void)
{
	unsigned char buf[3];

	buf[0] = REG_MODE;
	buf[1] = 0x40;
	buf[2] = 0x00;
	i2c_write(buf, 3);

	return 0;
}

static int i2c_read(unsigned char reg_addr)
{
	int ret = 0;
	unsigned char buf[2];

#if defined(CONFIG_USE_GPIO_I2C)
	OMAP_GPIO_I2C_RD_DATA i2c_rd_param;
#else
	struct i2c_msg msg1[1], msg2[1];
#endif

#if defined(CONFIG_USE_GPIO_I2C)
	i2c_rd_param.reg_len = 1;
	i2c_rd_param.reg_addr = &reg_addr;
	i2c_rd_param.rdata_len = 2;
	i2c_rd_param.rdata = buf;
	omap_gpio_i2c_read(fuelgauge_i2c_client, &i2c_rd_param);
#else
	msg1->addr = fuelgauge_i2c_client->addr;
	msg1->flags = I2C_M_WR;
	msg1->len = 1;
	msg1->buf = &reg_addr;

	ret = i2c_transfer(fuelgauge_i2c_client->adapter, msg1, 1);
	if (ret < 0) {
		printk(KERN_ERR "[FG] fail to read max17043.");
		return -1;
	} else {
		msg2->addr = fuelgauge_i2c_client->addr;
		msg2->flags = I2C_M_RD;
		msg2->len = 2;
		msg2->buf = buf;

		ret = i2c_transfer(fuelgauge_i2c_client->adapter, msg2, 1);

		if (ret < 0) {
			printk(KERN_ERR "[FG] fail to read max17043.");
			return -1;
		}
	}
#endif

	ret = buf[0] << 8 | buf[1];

	return ret;
}

static int i2c_write(unsigned char *buf, u8 len)
{
	int ret = 0;

#if defined(CONFIG_USE_GPIO_I2C)
	OMAP_GPIO_I2C_WR_DATA i2c_wr_param;
#else
	struct i2c_msg msg;
#endif

#if !defined(CONFIG_USE_GPIO_I2C)
	msg.addr = fuelgauge_i2c_client->addr;
	msg.flags = I2C_M_WR;
	msg.len = len;
	msg.buf = buf;

	ret = i2c_transfer(fuelgauge_i2c_client->adapter, &msg, 1);

	if (ret < 0) {
		printk(KERN_ERR "[FG] fail to write max17043.");
		return -1;
	}
#else
	i2c_wr_param.reg_len = 1;
	i2c_wr_param.reg_addr = &(buf[0]);
	i2c_wr_param.wdata_len = len;
	i2c_wr_param.wdata = &(buf[1]);
	omap_gpio_i2c_write(fuelgauge_i2c_client, &i2c_wr_param);
#endif

	return ret;
}

static irqreturn_t low_battery_isr(int irq, void *_di)
{
	if (sec_bci->ready)
		schedule_work(&fuelgauge_lowbat_alrt_work);

	return IRQ_HANDLED;
}

static void fuelgauge_lowbat_alrt_work_handler(struct work_struct *work)
{
	_low_battery_alert_();
}

static int fuelgauge_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	int ret = 0;
	unsigned char buf[3];

	printk(KERN_INFO "[FG] Fuelgauge Probe.\n");

	sec_bci = get_sec_bci();

	if (strcmp(client->name, DRIVER_NAME) != 0) {
		ret = -1;
		printk(KERN_ERR "[FG] device not supported.\n");
	}
#if defined(CONFIG_USE_GPIO_I2C)
	fuelgauge_i2c_dummy_client = client;
#else
	fuelgauge_i2c_client = client;
	printk(KERN_INFO "[FG] ADDR : %x\n", client->addr);

#endif

	if (client->irq) {
		INIT_WORK(&fuelgauge_lowbat_alrt_work, fuelgauge_lowbat_alrt_work_handler);

		/* set alert threshold to 1% */
		ret = i2c_read(REG_RCOMP);
		buf[0] = REG_RCOMP;
		buf[1] = ret >> 8;
		buf[2] = 0x1F;
		i2c_write(buf, 3);

		ret = i2c_read(REG_RCOMP);
		printk(KERN_INFO "[FG] val : %x\n", ret);

		ret = irq_to_gpio(client->irq);
		printk(KERN_INFO "[FG] FUEL_INT_GPIO : %d\n", ret);

		set_irq_type(client->irq, IRQ_TYPE_EDGE_FALLING);
		ret = request_irq(client->irq, low_battery_isr, IRQF_DISABLED, client->name, NULL);
		if (ret)
			printk(KERN_ERR "[FG] could not request irq %d, status %d\n", client->irq, ret);

	}

	sec_bci->charger.fuelgauge_full_soc = 95;

	return ret;
}

static int fuelgauge_remove(struct i2c_client *client)
{
	return 0;
}

static void fuelgauge_shutdown(struct i2c_client *client)
{
	return;
}

static int fuelgauge_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int batt_ptg = 0;
	int ret = 0;

	unsigned char alert_th;
	unsigned char buf[3];

	if (!sec_bci->charger.is_charging) {
		/* When chargine battery, set alert threshold by using remained battery percentage. */
		batt_ptg = get_fuelgauge_ptg_value(RAW_SOC);

		if (batt_ptg > BATTERY_LOW_PERCENTAGE)
			alert_th = 0x11;	/* BATTERY_LOW_PERCENTAGE, 15% */
		else if (batt_ptg > BATTERY_CRITICAL_LOW_PERCENTAGE)
			alert_th = 0x1C;	/* BATTERY_CRITICAL_LOW_PERCENTAGE, 4% */
		else
			alert_th = 0x1F;	/* BATTERY_POWEROFF_PERCENTAGE, 1% */

		ret = i2c_read(REG_RCOMP);
		buf[0] = REG_RCOMP;
		buf[1] = ret >> 8;
		buf[2] = alert_th;
		i2c_write(buf, 3);
		printk(KERN_INFO "[FG][%s] RAW_SOC=%d, Alert threshold=0x%x\n", __func__, batt_ptg, alert_th);
	}

	return 0;
}

static int fuelgauge_resume(struct i2c_client *client)
{
	int batt_ptg = 0;
	int ret = 0;
	unsigned char buf[3];

	batt_ptg = get_fuelgauge_ptg_value(RAW_SOC);
	if (batt_ptg <= BATTERY_POWEROFF_PERCENTAGE)
		schedule_work(&fuelgauge_lowbat_alrt_work);

	ret = i2c_read(REG_RCOMP);
	buf[0] = REG_RCOMP;
	buf[1] = ret >> 8;
	buf[2] = 0x1F;
	i2c_write(buf, 3);
	printk(KERN_INFO "[FG][%s] RAW_SOC=%d, Alert threshold 1%%\n", __func__, batt_ptg);

	return 0;
}

int fuelgauge_init(void)
{
	int ret;

#if defined(CONFIG_USE_GPIO_I2C)
	fuelgauge_i2c_client =
		omap_gpio_i2c_init(OMAP_GPIO_FUEL_SDA, OMAP_GPIO_FUEL_SCL, 0x36, 100);

	if (fuelgauge_i2c_client == NULL)
		printk(KERN_ERR "[FG] omap_gpio_i2c_init failed!\n");

	printk(KERN_INFO "[FG] Fuelgauge Init. add dummy i2c driver!\n");
	ret = i2c_add_driver(&fuelgauge_i2c_driver);
	if (ret < 0)
		printk(KERN_ERR "[FG] i2c_add_driver failed.\n");
#else
	printk(KERN_INFO "[FG] Fuelgauge Init. add i2c driver!\n");
	ret = i2c_add_driver(&fuelgauge_i2c_driver);
	if (ret < 0)
		printk(KERN_ERR "[FG] i2c_add_driver failed.\n");
#endif

	return ret;
}

void fuelgauge_exit(void)
{
	printk(KERN_INFO "[FG] Fuelgauge Exit.\n");

#if defined(CONFIG_USE_GPIO_I2C)
	omap_gpio_i2c_deinit(fuelgauge_i2c_client);
#endif
	i2c_del_driver(&fuelgauge_i2c_driver);

}
