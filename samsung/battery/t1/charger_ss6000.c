/*
 * module/samsung_battery/charger_ss6000.c
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/timer.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/wakelock.h>
#include <linux/regulator/consumer.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <mach/gpio.h>
#include <plat/mux.h>
#include <plat/microusbic.h>
#include <linux/i2c/twl.h>
#include "common.h"

#include <linux/uaccess.h>
#include <linux/fs.h>

#define DRIVER_NAME             "secChargerDev"


struct charger_device_info {
	struct device *dev;
	struct delayed_work cable_detection_work;
	struct delayed_work bat_removal_detection_work;
};

static struct device *this_dev;

static int BAT_REMOVAL_IRQ;
static int KCHG_EN_GPIO;
static int KTA_NCONN_GPIO;
static int KTA_NCONN_IRQ;
static int KUSB_CONN_IRQ;
static SEC_battery_charger_info *sec_bci;

enum charger_mode {
	USB500 = 0,
	ISET,
	USB100,
	FACTORY,
};
#if defined(CONFIG_MACH_T1_CHN)
static struct wake_lock sec_chg_wakelock;
#endif

/* Prototype */
int _battery_state_change_(int, int);
int _cable_status_now_(void);
static void clear_charge_start_time(void);
static void set_charge_start_time(void);
static void change_cable_status(int, struct charger_device_info *);
static void change_charge_status(int);
static void enable_charging(void);
static void disable_charging(void);
static bool check_battery_vf(void);
static irqreturn_t cable_detection_isr(int, void *);
static void cable_detection_work_handler(struct work_struct *);
static irqreturn_t bat_removal_detection_isr(int, void *);
static void bat_removal_detection_work_handler(struct work_struct *);
static int __devinit charger_probe(struct platform_device *);
static int __devexit charger_remove(struct platform_device *);
int charger_init(void);
void charger_exit(void);

extern SEC_battery_charger_info *get_sec_bci(void);
extern int _charger_state_change_(int category, int value);
extern int _get_t2adc_data_(int ch);
extern unsigned long long sched_clock(void);
extern int get_real_usbic_state(void);

#ifdef CONFIG_DYNAMIC_TSP_SETTINGS_FOR_CHARGER_STATE
extern int set_tsp_for_ta_detect(int state);
#endif
extern void check_lowbat_wakelock_condition(void);

int _battery_state_change_(int category, int value)
{
	struct charger_device_info *di;
	struct platform_device *pdev;
	int state;

	pdev = to_platform_device(this_dev);
	di = platform_get_drvdata(pdev);

	//printk( "[TA] cate: %d, value: %d, %s\n", category, value, di->dev->kobj.name );
	switch (category) {
	case STATUS_CATEGORY_TEMP:
		switch (value) {
		case BATTERY_TEMPERATURE_NORMAL:
			printk( KERN_INFO "[TA] Charging re start normal TEMP!!\n");
			change_charge_status(POWER_SUPPLY_STATUS_RECHARGING_FOR_TEMP);
			break;

		case BATTERY_TEMPERATURE_LOW:
			printk( KERN_INFO "[TA] Charging stop LOW TEMP!!\n");
			change_charge_status(POWER_SUPPLY_STATUS_NOT_CHARGING);
			break;

		case BATTERY_TEMPERATURE_HIGH:
			printk( KERN_INFO "[TA] Charging stop HI TEMP!!\n");
			change_charge_status(POWER_SUPPLY_STATUS_NOT_CHARGING);
			break;

#ifdef HIGH_TEMP_GAURD_FOR_CAMCORDER
		case BATTERY_TEMPERATURE_ETC:
			if(sec_bci->charger.camera_recording
			   && sec_bci->charger.camera_recording_inout){
				printk( KERN_INFO "[TA] Charger is set to the USB mode\n");
				state = sec_bci->charger.cable_status;
				sec_bci->charger.cable_status = POWER_SUPPLY_TYPE_USB;
				disable_charging();
				enable_charging();
				sec_bci->charger.cable_status = state;
				break;
			} else {
				printk( KERN_INFO "[TA] Charger is set to the original mode\n");
				disable_charging();
				enable_charging();
				break;
			}
#endif

		default:
			break;
		}

		break;

	case STATUS_CATEGORY_CHARGING:
		switch (value) {
		case POWER_SUPPLY_STATUS_FULL:
			printk( KERN_INFO "[TA] Charge FULL(#1)! (monitoring charge current)\n");
			change_charge_status(POWER_SUPPLY_STATUS_FULL);
			break;

		case POWER_SUPPLY_STATUS_FULL_END:
			printk( KERN_INFO "[TA] Charge FULL(#2)! (monitoring charge current)\n");
			change_charge_status(POWER_SUPPLY_STATUS_FULL_END);
			break;

		case POWER_SUPPLY_STATUS_CHARGING_OVERTIME:
			printk( KERN_INFO "[TA] CHARGING TAKE OVER 6 hours !!\n");
			change_charge_status(POWER_SUPPLY_STATUS_FULL_END);
			break;

		case POWER_SUPPLY_STATUS_FULL_DUR_SLEEP:
			printk( KERN_INFO "[TA] Charge FULL!\n");
			change_charge_status(POWER_SUPPLY_STATUS_FULL);
			break;

		case POWER_SUPPLY_STATUS_RECHARGING_FOR_FULL:
			printk( KERN_INFO "[TA] Re-Charging Start!!\n");
			change_charge_status(POWER_SUPPLY_STATUS_RECHARGING_FOR_FULL);
			break;

		default:
			break;
		}

		break;

	case STATUS_CATEGORY_ETC:
		switch (value) {
			case ETC_CABLE_IS_DISCONNECTED:
			printk( KERN_INFO "[TA] CABLE IS NOT CONNECTED.... Charge Stop!!\n");
			change_cable_status(POWER_SUPPLY_TYPE_BATTERY, di);
			break;

		default:
			break;
		}

		break;

	default:
		printk( KERN_INFO "[TA] Invalid category!!!!!\n");
		break;
	}

	return 0;
}

int _cable_status_now_(void)
{
	int ret = 0;

	if (sec_bci->charger.use_ta_nconnected_irq) {
		ret = gpio_get_value(KTA_NCONN_GPIO) ? 0 : 1;
	} else {
		ret = get_real_usbic_state();
	}

	return ret;
}

static void clear_charge_start_time(void)
{
	sec_bci->charger.charge_start_time = sched_clock();
}

static void set_charge_start_time(void)
{
	sec_bci->charger.charge_start_time = sched_clock();
}

static void change_cable_status(int status,
				struct charger_device_info *di)
{

	sec_bci->charger.prev_cable_status = sec_bci->charger.cable_status;
	sec_bci->charger.cable_status = status;

	_charger_state_change_(STATUS_CATEGORY_CABLE, status);

	switch (status) {
	case POWER_SUPPLY_TYPE_BATTERY:
		/*Diable charging */
		change_charge_status(POWER_SUPPLY_STATUS_DISCHARGING);

		break;

	case POWER_SUPPLY_TYPE_MAINS:
#if defined(CONFIG_MACH_T1_CHN)
		wake_lock(&sec_chg_wakelock);
		printk("TA wakelock \n");
#endif
	case POWER_SUPPLY_TYPE_USB:
		/*Enable charging */
		change_charge_status(POWER_SUPPLY_STATUS_CHARGING);

		break;

	default:
		break;
	}

}

static void change_charge_status(int status)
{

	switch (status) {
	case POWER_SUPPLY_STATUS_UNKNOWN:
	case POWER_SUPPLY_STATUS_NOT_CHARGING:
	case POWER_SUPPLY_STATUS_DISCHARGING:
		if (sec_bci->battery.battery_health != POWER_SUPPLY_HEALTH_UNSPEC_FAILURE)
			disable_charging();

#ifdef HIGH_TEMP_GAURD_FOR_CAMCORDER
		sec_bci->charger.camera_recording_inout = 0;
#endif
		break;

	case POWER_SUPPLY_STATUS_FULL:
		break;

	case POWER_SUPPLY_STATUS_FULL_END:
		sec_bci->charger.rechg_count = 4;
		clear_charge_start_time();
		disable_charging();
		break;

	case POWER_SUPPLY_STATUS_CHARGING:

		if (sec_bci->battery.battery_vf_ok) {
			sec_bci->battery.battery_health = POWER_SUPPLY_HEALTH_GOOD;
			set_charge_start_time();
			enable_charging();
			break;

		} else {
			sec_bci->battery.battery_health = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
			status = POWER_SUPPLY_STATUS_NOT_CHARGING;
			printk( KERN_INFO "[TA] INVALID BATTERY, %d !! \n", status);
			disable_charging();
			break;

		}

	case POWER_SUPPLY_STATUS_RECHARGING_FOR_FULL:
		sec_bci->charger.charging_timeout = DEFAULT_RECHARGING_TIMEOUT;
		set_charge_start_time();
		enable_charging();
		break;

	case POWER_SUPPLY_STATUS_RECHARGING_FOR_TEMP:
		enable_charging();
		break;

	default:
		;
	}

	sec_bci->charger.prev_charge_status = sec_bci->charger.charge_status;
	sec_bci->charger.charge_status = status;
	check_lowbat_wakelock_condition();

	_charger_state_change_(STATUS_CATEGORY_CHARGING, status);
}


static void change_charger_mode(enum charger_mode mode)
{
	int i;

	if (system_rev >= 1) {
		/* enable charing */
		gpio_set_value(KCHG_EN_GPIO, 0);

		for (i = 0; i < mode; i++) {
			udelay(200);
			gpio_set_value(KCHG_EN_GPIO, 1);
			udelay(200);
			gpio_set_value(KCHG_EN_GPIO, 0);

		}
		msleep(2);

	} else {
		/* enable charing */
		gpio_set_value(KCHG_EN_GPIO, 1);

		for (i = 0; i < mode; i++) {
			udelay(200);
			gpio_set_value(KCHG_EN_GPIO, 0);
			udelay(200);
			gpio_set_value(KCHG_EN_GPIO, 1);
		}
		msleep(2);

	}
}


static void enable_charging(void)
{
	int cable_status = sec_bci->charger.cable_status;

	if (get_real_usbic_state() == MICROUSBIC_MHL_CHARGER) {
		printk( KERN_INFO "[TA] MHL CHARGER operates as USB\n");
		cable_status = POWER_SUPPLY_TYPE_USB;
	}

	switch (cable_status) {
	case POWER_SUPPLY_TYPE_USB:	/* USB500 Mode */
		change_charger_mode(USB500);
		printk( KERN_INFO "[TA] charge mode : USB500\n");
		break;

	case POWER_SUPPLY_TYPE_MAINS:	/* ISET Mode */
		change_charger_mode(ISET);
		printk( KERN_INFO "[TA] charge mode : ISET\n");
		break;

	default:
		printk( KERN_INFO "[TA] inproper charge mode\n");
		break;
	}

	printk("========== ENABLE CHARGING =============\n");
	sec_bci->charger.is_charging = true;
}

static void disable_charging(void)
{
	if (system_rev >= 1)
		gpio_set_value(KCHG_EN_GPIO, 1);
	else
		gpio_set_value(KCHG_EN_GPIO, 0);

	msleep(2);
	printk("============== DISABLE CHARGING ============\n");
	sec_bci->charger.is_charging = false;
}

static bool check_battery_vf(void)
{
	int val;

	val = _get_t2adc_data_(0);
	printk( KERN_INFO "[TA] VF ADC : %d\n", val);

	if (val > 25000)
		return 0;
	return 1;
}

static irqreturn_t bat_removal_detection_isr(int irq, void *_di)
{
	struct charger_device_info *di = _di;

	if (sec_bci->ready) {
		cancel_delayed_work(&di->bat_removal_detection_work);
		queue_delayed_work(sec_bci->sec_battery_workq,
			   &di->bat_removal_detection_work, 0);

	}

	return IRQ_HANDLED;
}

static void bat_removal_detection_work_handler(struct work_struct *work)
{
	sec_bci->battery.battery_vf_ok = check_battery_vf();

	if (!sec_bci->battery.battery_vf_ok) {
		disable_irq(BAT_REMOVAL_IRQ);
		disable_charging();
	}

	return;
}

static irqreturn_t cable_detection_isr(int irq, void *_di)
{
	struct charger_device_info *di = _di;

	if (sec_bci->ready) {
		cancel_delayed_work(&di->cable_detection_work);

		if (gpio_get_value(KTA_NCONN_GPIO) == 0)
			queue_delayed_work(sec_bci->sec_battery_workq,
						&di->cable_detection_work, HZ/2);
		else
			queue_delayed_work(sec_bci->sec_battery_workq,
						&di->cable_detection_work, HZ/10);
	}

	return IRQ_HANDLED;
}

static void cable_detection_work_handler(struct work_struct *work)
{
	struct charger_device_info *di = container_of(work,
						  struct charger_device_info,
						  cable_detection_work.work);
	int n_usbic_state;
	int count;
	printk( KERN_INFO "[TA] cable_detection_work_handler start!!!!\n");

	clear_charge_start_time();

	n_usbic_state = get_real_usbic_state();
	printk( KERN_INFO "[TA] cable_detection_isr handler. usbic_state: %d\n",
	       n_usbic_state);

	if ((sec_bci->charger.use_ta_nconnected_irq)
		&& (gpio_get_value(KTA_NCONN_GPIO))) {
			count = 0;
			while (count < 10) {
				if (gpio_get_value(KTA_NCONN_GPIO)) {
					count++;
					msleep(1);
					if (count == 10) {
						n_usbic_state = -10;
						printk( KERN_INFO "[TA] CABLE UNPLUGGED.\n");
					}
				} else
					break;
			}
	}

	if ((gpio_get_value(KTA_NCONN_GPIO) == 0) && (n_usbic_state == MICROUSBIC_NO_DEVICE)) {
		msleep(500);
		n_usbic_state = get_real_usbic_state();
		printk( KERN_INFO "[TA] No device detected, retry. usbic_state: %d\n", n_usbic_state);
	}

	// Workaround for Archer [+]
	if (!n_usbic_state && sec_bci->charger.use_ta_nconnected_irq) {
		if (!gpio_get_value(KTA_NCONN_GPIO))
			n_usbic_state = MICROUSBIC_5W_CHARGER;
	}
	// Workaround for Archer [-]

	switch (n_usbic_state) {
	case MICROUSBIC_5W_CHARGER:
	case MICROUSBIC_TA_CHARGER:
	case MICROUSBIC_USB_CHARGER:
	case MICROUSBIC_PHONE_USB:
	case MICROUSBIC_USB_CABLE:
	case MICROUSBIC_MHL_CHARGER:
	case MICROUSBIC_JIG_USB_ON:
		if (sec_bci->charger.cable_status == POWER_SUPPLY_TYPE_USB
		    || sec_bci->charger.cable_status == POWER_SUPPLY_TYPE_MAINS) {
			goto Out_IRQ_Cable_Det;
		}

		/* Check VF */
		sec_bci->battery.battery_vf_ok = check_battery_vf();

		/* TA or USB or MHL is inserted */
		if (n_usbic_state == MICROUSBIC_USB_CABLE) {
			/* Charging current : 398mA */
			printk( KERN_INFO "[TA] USB CABLE PLUGGED\n");
			change_cable_status(POWER_SUPPLY_TYPE_USB, di);
#ifdef CONFIG_DYNAMIC_TSP_SETTINGS_FOR_CHARGER_STATE
			set_tsp_for_ta_detect(1);
#endif
		} else if (n_usbic_state == MICROUSBIC_MHL_CHARGER) {
			/* PowerSupplyType is MAINS(TA). But, charging current will be set as USB */
			printk( KERN_INFO "[TA] MHL CABLE PLUGGED\n");
			change_cable_status(POWER_SUPPLY_TYPE_MAINS, di);
#ifdef CONFIG_DYNAMIC_TSP_SETTINGS_FOR_CHARGER_STATE
			set_tsp_for_ta_detect(1);
#endif
		} else if (n_usbic_state == MICROUSBIC_JIG_USB_ON){
			/* Charging current : 638mA */
			printk( KERN_INFO "[TA] JIG_USB_ON CABLE PLUGGED\n");
			change_cable_status(POWER_SUPPLY_TYPE_MAINS, di);
#ifdef CONFIG_DYNAMIC_TSP_SETTINGS_FOR_CHARGER_STATE
			set_tsp_for_ta_detect(1);
#endif
		} else {
			/* Charging current : 638mA */
			printk( KERN_INFO "[TA] CHARGER CABLE PLUGGED\n");
			change_cable_status(POWER_SUPPLY_TYPE_MAINS, di);
#ifdef CONFIG_DYNAMIC_TSP_SETTINGS_FOR_CHARGER_STATE
			set_tsp_for_ta_detect(1);
#endif
		}
		break;

	case MICROUSBIC_NO_DEVICE:
	default:
		/* Check VF */
		sec_bci->battery.battery_vf_ok = check_battery_vf();

#if defined(CONFIG_MACH_T1_CHN)
		if(sec_bci->charger.prev_cable_status == POWER_SUPPLY_TYPE_BATTERY
			&& sec_bci->charger.cable_status == POWER_SUPPLY_TYPE_MAINS)
		{
			wake_unlock(&sec_chg_wakelock);
			printk("TA wake unlock \n");
		}
#endif

		if (sec_bci->charger.prev_cable_status != POWER_SUPPLY_TYPE_BATTERY
		    && sec_bci->charger.cable_status == POWER_SUPPLY_TYPE_BATTERY) {
			goto Out_IRQ_Cable_Det;
		} else if (sec_bci->charger.prev_cable_status == -1
			   && sec_bci->charger.cable_status == -1) {
			printk( KERN_INFO "[TA] Fisrt time after bootig.\n");
			goto FirstTime_Boot;
		}

	FirstTime_Boot:
	/* TA or USB is ejected */
	printk( KERN_INFO "[TA] CABLE UNPLUGGED\n");
	change_cable_status(POWER_SUPPLY_TYPE_BATTERY, di);
#ifdef CONFIG_DYNAMIC_TSP_SETTINGS_FOR_CHARGER_STATE
	set_tsp_for_ta_detect(0);
#endif
	break;
	}

Out_IRQ_Cable_Det:
	;
}

static int __devinit charger_probe(struct platform_device *pdev)
{
	int ret = 0;
	int irq = 0;
	struct charger_device_info *di;

	printk("[TA] Charger probe...\n");

	sec_bci = get_sec_bci();

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di)
		return -ENOMEM;

	platform_set_drvdata(pdev, di);
	di->dev = &pdev->dev;
	this_dev = &pdev->dev;

#if defined(CONFIG_MACH_T1_CHN)
	wake_lock_init(&sec_chg_wakelock, WAKE_LOCK_SUSPEND, "samsung-charger");
#endif

	/*Init Work */
	INIT_DELAYED_WORK(&di->cable_detection_work,
			  cable_detection_work_handler);
	INIT_DELAYED_WORK(&di->bat_removal_detection_work,
			  bat_removal_detection_work_handler);

	/* USB irq */
	KUSB_CONN_IRQ = platform_get_irq(pdev, 0);	//-> Latona does not used KUSB_CONN_IRQ.
	if (KUSB_CONN_IRQ) {
		ret = request_irq(KUSB_CONN_IRQ, cable_detection_isr,
			  IRQF_DISABLED | IRQF_SHARED, pdev->name, di);
		if (ret) {
			dev_err(di->dev,
				"[TA] could not request irq %d, status %d\n",
				KUSB_CONN_IRQ, ret);
			goto usb_irq_fail;
		}
		set_irq_type(KUSB_CONN_IRQ, IRQ_TYPE_EDGE_BOTH);
	} else {
		sec_bci->charger.use_ta_nconnected_irq = true;
	}

	/* TA connected irq */
	KTA_NCONN_IRQ = platform_get_irq(pdev, 1);
	ret =
		request_irq(KTA_NCONN_IRQ, cable_detection_isr, IRQF_DISABLED,
			    pdev->name, di);
	if (ret) {
		dev_err(di->dev, "[TA] could not request irq %d, status %d\n",
			KTA_NCONN_IRQ, ret);
		goto ta_irq_fail;
	}
	set_irq_type(KTA_NCONN_IRQ, IRQ_TYPE_EDGE_BOTH);
	if (sec_bci->charger.use_ta_nconnected_irq)
		KTA_NCONN_GPIO = irq_to_gpio(KTA_NCONN_IRQ);

	/* Not using CHG_ING_N(platform_get_irq(pdev, 2)) */

	/* Charging enable gpio */
	KCHG_EN_GPIO = irq_to_gpio(platform_get_irq(pdev, 3));
	dev_info(di->dev, "[TA] CHG_EN GPIO : %d \n", KCHG_EN_GPIO);

	/* Battery remove irq */
	BAT_REMOVAL_IRQ = platform_get_irq(pdev, 4);
	ret = request_irq(BAT_REMOVAL_IRQ, bat_removal_detection_isr,
			    IRQF_DISABLED, pdev->name, di);
	if (ret) {
		dev_err(di->dev, "[TA] could not request irq %d, status %d\n",
			BAT_REMOVAL_IRQ, ret);
		goto bat_removal_irq_fail;
	}
	set_irq_type(BAT_REMOVAL_IRQ, IRQ_TYPE_EDGE_BOTH);

	queue_delayed_work(sec_bci->sec_battery_workq,
			   &di->cable_detection_work, HZ);
	return 0;

bat_removal_irq_fail:
	irq = platform_get_irq(pdev, 2);
	free_irq(irq, di);

ta_irq_fail:
	irq = platform_get_irq(pdev, 0);
	free_irq(irq, di);

usb_irq_fail:
	kfree(di);

	return ret;
}

static int __devexit charger_remove(struct platform_device *pdev)
{
	struct charger_device_info *di = platform_get_drvdata(pdev);
	int irq;

	irq = platform_get_irq(pdev, 0);
	free_irq(irq, di);

	irq = platform_get_irq(pdev, 1);
	free_irq(irq, di);

	flush_scheduled_work();

	platform_set_drvdata(pdev, NULL);
	kfree(di);

	return 0;
}

struct platform_driver charger_platform_driver = {
	.probe = &charger_probe,
	.remove = __devexit_p(charger_remove),
	.driver = {
		.name = DRIVER_NAME,
	},
};

int charger_init(void)
{
	int ret;

	pr_alert("[TA] Charger Init\n");
	ret = platform_driver_register(&charger_platform_driver);

	return ret;
}

void charger_exit(void)
{
	platform_driver_unregister(&charger_platform_driver);
	pr_alert("[TA] Charger IC Exit\n");
}
