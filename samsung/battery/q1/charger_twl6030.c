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

#define DEBUG

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
#include <linux/i2c/twl6030-gpadc.h>
#include "common.h"

#include <linux/uaccess.h>
#include <linux/fs.h>

#define DRIVER_NAME             "secChargerDev"

#define CONTROLLER_INT_MASK	0x00
#define CONTROLLER_CTRL1	0x01
#define CONTROLLER_WDG		0x02
#define CONTROLLER_STAT1	0x03
#define CHARGERUSB_INT_STATUS	0x04
#define CHARGERUSB_INT_MASK	0x05
#define CHARGERUSB_STATUS_INT1	0x06
#define CHARGERUSB_STATUS_INT2	0x07
#define CHARGERUSB_CTRL1	0x08
#define CHARGERUSB_CTRL2	0x09
#define CHARGERUSB_CTRL3	0x0A
#define CHARGERUSB_STAT1	0x0B
#define CHARGERUSB_VOREG	0x0C
#define CHARGERUSB_VICHRG	0x0D
#define CHARGERUSB_CINLIMIT	0x0E
#define CHARGERUSB_CTRLLIMIT1	0x0F
#define CHARGERUSB_CTRLLIMIT2	0x10
#define ANTICOLLAPSE_CTRL1	0x11
#define ANTICOLLAPSE_CTRL2	0x12

/* CONTROLLER_INT_MASK */
#define MVAC_FAULT		(1 << 7)
#define MAC_EOC			(1 << 6)
#define MBAT_REMOVED		(1 << 4)
#define MFAULT_WDG		(1 << 3)
#define MBAT_TEMP		(1 << 2)
#define MVBUS_DET		(1 << 1)
#define MVAC_DET		(1 << 0)

/* CONTROLLER_CTRL1 */
#define CONTROLLER_CTRL1_EN_CHARGER	(1 << 4)
#define CONTROLLER_CTRL1_SEL_CHARGER	(1 << 3)

/* CONTROLLER_STAT1 */
#define CONTROLLER_STAT1_EXTCHRG_STATZ	(1 << 7)
#define CONTROLLER_STAT1_CHRG_DET_N	(1 << 5)
#define CONTROLLER_STAT1_FAULT_WDG	(1 << 4)
#define CONTROLLER_STAT1_VAC_DET	(1 << 3)
#define VAC_DET	(1 << 3)
#define CONTROLLER_STAT1_VBUS_DET	(1 << 2)
#define VBUS_DET	(1 << 2)
#define CONTROLLER_STAT1_BAT_REMOVED	(1 << 1)
#define CONTROLLER_STAT1_BAT_TEMP_OVRANGE (1 << 0)

/* CHARGERUSB_INT_STATUS */
#define CURRENT_TERM_INT	(1 << 3)
#define CHARGERUSB_STAT		(1 << 2)
#define CHARGERUSB_THMREG	(1 << 1)
#define CHARGERUSB_FAULT	(1 << 0)

/* CHARGERUSB_INT_MASK */
#define MASK_MCURRENT_TERM		(1 << 3)
#define MASK_MCHARGERUSB_STAT		(1 << 2)
#define MASK_MCHARGERUSB_THMREG		(1 << 1)
#define MASK_MCHARGERUSB_FAULT		(1 << 0)

/* CHARGERUSB_STATUS_INT1 */
#define CHARGERUSB_STATUS_INT1_TMREG	(1 << 7)
#define CHARGERUSB_STATUS_INT1_NO_BAT	(1 << 6)
#define CHARGERUSB_STATUS_INT1_BST_OCP	(1 << 5)
#define CHARGERUSB_STATUS_INT1_TH_SHUTD	(1 << 4)
#define CHARGERUSB_STATUS_INT1_BAT_OVP	(1 << 3)
#define CHARGERUSB_STATUS_INT1_POOR_SRC	(1 << 2)
#define CHARGERUSB_STATUS_INT1_SLP_MODE	(1 << 1)
#define CHARGERUSB_STATUS_INT1_VBUS_OVP	(1 << 0)

/* CHARGERUSB_STATUS_INT2 */
#define ICCLOOP		(1 << 3)
#define CURRENT_TERM	(1 << 2)
#define CHARGE_DONE	(1 << 1)
#define ANTICOLLAPSE	(1 << 0)

/* CHARGERUSB_CTRL1 */
#define SUSPEND_BOOT	(1 << 7)
#define OPA_MODE	(1 << 6)
#define HZ_MODE		(1 << 5)
#define TERM		(1 << 4)

/* CHARGERUSB_CTRL2 */
#define CHARGERUSB_CTRL2_VITERM_50	(0 << 5)
#define CHARGERUSB_CTRL2_VITERM_100	(1 << 5)
#define CHARGERUSB_CTRL2_VITERM_150	(2 << 5)
#define CHARGERUSB_CTRL2_VITERM_400	(7 << 5)

/* CHARGERUSB_CTRL3 */
#define VBUSCHRG_LDO_OVRD	(1 << 7)
#define CHARGE_ONCE		(1 << 6)
#define BST_HW_PR_DIS		(1 << 5)
#define AUTOSUPPLY		(1 << 3)
#define BUCK_HSILIM		(1 << 0)

/* CHARGERUSB_CTRLLIMIT2 */
#define CHARGERUSB_CTRLLIMIT2_1500	0x0E
#define		LOCK_LIMIT		(1 << 4)

/* ANTICOLLAPSE_CTRL2 */
#define BUCK_VTH_SHIFT			5

#define STS_HW_CONDITIONS	0x21
#define STS_USB_ID		(1 << 2)	/* Level status of USB ID */

#define MAX_CHARGER_CURRENT	1500
#define MAX_CHARGER_VOLTAGE	4560


// THIS CONFIG IS SET IN BOARD_FILE.(platform_data)
struct charger_device_config {
	/* CHECK BATTERY VF USING ADC */
	int VF_CHECK_USING_ADC;	// true or false
	int VF_ADC_PORT;

	/* SUPPORT CHG_ING IRQ FOR CHECKING FULL */
	int SUPPORT_CHG_ING_IRQ;
};

static struct charger_device_config *device_config;

struct charger_device_info {
	struct device *dev;
	struct delayed_work twl6030charger_ctrl_work;
	struct delayed_work twl6030charger_fault_work;
	struct delayed_work bat_removal_detection_work;

	int charger_status;

	u8 stat1;
	u8 status_int1;
	u8 status_int2;
	
	unsigned int max_charger_voltagemV;
	unsigned int max_charger_currentmA;
	unsigned int low_bat_voltagemV;	
};

static struct device *this_dev;

static int BAT_REMOVAL_IRQ;
static int CTRL_INT_IRQ;
static int CHARGERFAULT_INTR_IRQ;

/* workaround for invalid full charging at boot time
static int KCHARGING_BOOT;
*/
static SEC_battery_charger_info *sec_bci;

// Prototype
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
static irqreturn_t full_charge_isr(int, void *);
static void full_charge_work_handler(struct work_struct *);
static irqreturn_t bat_removal_detection_isr(int, void *);
static void bat_removal_detection_work_handler(struct work_struct *);
static int __devinit charger_probe(struct platform_device *);
static int __devexit charger_remove(struct platform_device *);
static int charger_suspend(struct platform_device *, pm_message_t);
static int charger_resume(struct platform_device *);
int charger_init(void);
void charger_exit(void);

extern SEC_battery_charger_info *get_sec_bci(void);
extern int _charger_state_change_(int category, int value);
extern int _get_t2adc_data_(int ch);
extern int _get_average_value_(int *data, int count);
extern int omap34xx_pad_get_wakeup_status(int gpio);
extern int omap34xx_pad_set_padoff(int gpio, int wakeup_en);
extern unsigned long long sched_clock(void);
extern int get_real_usbic_state(void);
extern int get_usbic_state(void);

#ifdef CONFIG_DYNAMIC_TSP_SETTINGS_FOR_CHARGER_STATE
extern int set_tsp_for_ta_detect(int state);
#endif


static enum charger_mode {
	USB500 = 0,
	ISET,
	USB100,
	FACTORY,
};


static void twl6030_config_iterm_reg(struct charger_device_info *di,
						unsigned int term_currentmA)
{
	if ((term_currentmA > 400) || (term_currentmA < 50)) {
		dev_dbg(di->dev, "invalid termination current\n");
		return;
	}

	term_currentmA = ((term_currentmA - 50)/50) << 5;
	twl_i2c_write_u8(TWL6030_MODULE_CHARGER, term_currentmA,
						CHARGERUSB_CTRL2);
	return;
}

static void twl6030_config_voreg_reg(struct charger_device_info *di,
							unsigned int voltagemV)
{
	if ((voltagemV < 3500) || (voltagemV > 4760)) {
		dev_dbg(di->dev, "invalid charger_voltagemV\n");
		return;
	}

	voltagemV = (voltagemV - 3500) / 20;
	twl_i2c_write_u8(TWL6030_MODULE_CHARGER, voltagemV,
						CHARGERUSB_VOREG);
	return;
}

static void twl6030_config_vichrg_reg(struct charger_device_info *di,
							unsigned int currentmA)
{
	if ((currentmA >= 300) && (currentmA <= 450))
		currentmA = (currentmA - 300) / 50;
	else if ((currentmA >= 500) && (currentmA <= 1500))
		currentmA = (currentmA - 500) / 100 + 4;
	else {
		dev_dbg(di->dev, "invalid charger_currentmA\n");
		return;
	}

	twl_i2c_write_u8(TWL6030_MODULE_CHARGER, currentmA,
						CHARGERUSB_VICHRG);
	return;
}

static void twl6030_config_cinlimit_reg(struct charger_device_info *di,
							unsigned int currentmA)
{
	if ((currentmA >= 50) && (currentmA <= 750))
		currentmA = (currentmA - 50) / 50;
	else if (currentmA >= 750)
		currentmA = (800 - 50) / 50;
	else {
		dev_dbg(di->dev, "invalid input current limit\n");
		return;
	}

	twl_i2c_write_u8(TWL6030_MODULE_CHARGER, currentmA,
					CHARGERUSB_CINLIMIT);
	return;
}

static void twl6030_config_limit1_reg(struct charger_device_info *di,
							unsigned int voltagemV)
{
	if ((voltagemV < 3500) || (voltagemV > 4760)) {
		dev_dbg(di->dev, "invalid max_charger_voltagemV\n");
		return;
	}

	voltagemV = (voltagemV - 3500) / 20;
	twl_i2c_write_u8(TWL6030_MODULE_CHARGER, voltagemV,
						CHARGERUSB_CTRLLIMIT1);
	return;
}

static void twl6030_config_limit2_reg(struct charger_device_info *di,
							unsigned int currentmA)
{
	if ((currentmA >= 300) && (currentmA <= 450))
		currentmA = (currentmA - 300) / 50;
	else if ((currentmA >= 500) && (currentmA <= 1500))
		currentmA = (currentmA - 500) / 100 + 4;
	else {
		dev_dbg(di->dev, "invalid max_charger_currentmA\n");
		return;
	}

	currentmA |= LOCK_LIMIT;
	twl_i2c_write_u8(TWL6030_MODULE_CHARGER, currentmA,
						CHARGERUSB_CTRLLIMIT2);
	return;
}


int _battery_state_change_(int category, int value)
{
	struct charger_device_info *di;
	struct platform_device *pdev;

	int state;

	pdev = to_platform_device(this_dev);
	di = platform_get_drvdata(pdev);


	printk( KERN_INFO "[TA] cate: %d, value: %d, %s\n", category, value, di->dev->kobj.name );
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
				enable_charging();
				sec_bci->charger.cable_status = state;
				break;
			} else {
				printk( KERN_INFO "[TA] Charger is set to the original mode\n");
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

	ret = get_real_usbic_state();

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
		if (sec_bci->battery.battery_health != POWER_SUPPLY_HEALTH_DEAD)
			disable_charging();

#ifdef HIGH_TEMP_GAURD_FOR_CAMCORDER
		sec_bci->charger.camera_recording_inout = 0;
#endif
		break;

	case POWER_SUPPLY_STATUS_FULL:
		break;

	case POWER_SUPPLY_STATUS_FULL_END:
		sec_bci->charger.rechg_count = 4;
		/*Cancel timer */
		clear_charge_start_time();

		disable_charging();
		break;
		
	case POWER_SUPPLY_STATUS_CHARGING:

		if (sec_bci->battery.battery_vf_ok) {
			sec_bci->battery.battery_health = POWER_SUPPLY_HEALTH_GOOD;

			/*Start monitoring charging time */
			set_charge_start_time();

			enable_charging();
			break;

		} else {
			sec_bci->battery.battery_health = POWER_SUPPLY_HEALTH_DEAD;

			status = POWER_SUPPLY_STATUS_DISCHARGING;

			printk( KERN_INFO "[TA] INVALID BATTERY, %d !! \n", status);

			disable_charging();
			break;

		}

	case POWER_SUPPLY_STATUS_RECHARGING_FOR_FULL:
		/*Start monitoring charging time */
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

	_charger_state_change_(STATUS_CATEGORY_CHARGING, status);

}


static void enable_charging(void)
{
	struct charger_device_info *di;
	di = this_dev;

	unsigned int charger_incurrentmA;
	unsigned int charger_outcurrentmA;
	unsigned int regulation_voltagemV;
	unsigned int termination_currentmA;
	u8 state = 0;
	
	
	int cable_status = sec_bci->charger.cable_status;

	twl_i2c_read_u8(TWL6030_MODULE_ID1, &state,
			TWL6030_REG_GPADC_CTRL);
	state |= TWL6030_GPADC_CTRL_SCALER_EN_CH11;
	twl_i2c_write_u8(TWL6030_MODULE_ID1, state,
			 TWL6030_REG_GPADC_CTRL);

	switch (cable_status) {
	case POWER_SUPPLY_TYPE_USB:
		charger_outcurrentmA = 400;
		charger_incurrentmA = 400;
		regulation_voltagemV = 4200;
		termination_currentmA = 150;
		dev_dbg(di->dev,"[TA] USB charging start\n");
		break;
		
	case POWER_SUPPLY_TYPE_MAINS:
		charger_outcurrentmA = 1000;
		charger_incurrentmA = 1000;
		regulation_voltagemV = 4200;
		termination_currentmA = 150;
		dev_dbg(di->dev,"[TA] TA charging start\n");
		break;

	default:
		dev_dbg(di->dev,"[TA] inproper charge mode\n");
		goto end_enable_charging;
	}
	
	twl6030_config_vichrg_reg(di, charger_outcurrentmA);
	twl6030_config_cinlimit_reg(di, charger_incurrentmA);
	twl6030_config_voreg_reg(di, regulation_voltagemV);
	twl6030_config_iterm_reg(di, termination_currentmA);

	twl_i2c_write_u8(TWL6030_MODULE_CHARGER,
			CONTROLLER_CTRL1_EN_CHARGER,
			CONTROLLER_CTRL1);

	dev_dbg(di->dev, "ENABLE CHARGING\n");
	
	sec_bci->charger.is_charging = true;

end_enable_charging:
	;
	

}

static void disable_charging(void)
{
	struct charger_device_info *di;
	di = this_dev;

	u8 state = 0;
	
	twl_i2c_read_u8(TWL6030_MODULE_ID1, &state,
			TWL6030_REG_GPADC_CTRL);
	state &= ~TWL6030_GPADC_CTRL_SCALER_EN_CH11;
	twl_i2c_write_u8(TWL6030_MODULE_ID1, state,
			 TWL6030_REG_GPADC_CTRL);

	twl_i2c_write_u8(TWL6030_MODULE_CHARGER, 0, CONTROLLER_CTRL1);
	dev_dbg(di->dev, "DISABLE CHARGING\n");
	sec_bci->charger.is_charging = false;
}

static bool check_battery_vf(void)
{
	int val;
	/*
	 * Prevent charging on batteries were isistor is
	 * less than 5K.
	 */
#if 0
	int ret;
	u8 present_battery;

	ret = twl_i2c_read_u8(TWL6030_MODULE_ID0, &present_battery,
						  TWL6030_REG_SIMCTRL);

	if( present_battery & 0x02 ) {
		printk( KERN_INFO "[TA] Battery is present\n");
		return 1;
	} else {
		printk( KERN_INFO "[TA] Battery is not present\n");
		return 0;
	}
#endif

	val = _get_t2adc_data_(0);

	printk( KERN_INFO "[TA] VF ADC : %d\n", val);

	if (val > 50000)
		return 0;
	return 1;

}

static irqreturn_t twl6030charger_ctrl_isr(int irq, void *_di)
{
	struct charger_device_info *di = _di;

	if (sec_bci->ready) {
		cancel_delayed_work(&di->twl6030charger_ctrl_work);
		queue_delayed_work(sec_bci->sec_battery_workq,
			   &di->twl6030charger_ctrl_work, 15*HZ/10);
	}

	return IRQ_HANDLED;

}

static irqreturn_t twl6030charger_fault_isr(int irq, void *_di)
{
	struct charger_device_info *di = _di;

	if (sec_bci->ready) {
		cancel_delayed_work(&di->twl6030charger_fault_work);
		queue_delayed_work(sec_bci->sec_battery_workq,
			   &di->twl6030charger_fault_work, 0);

	}

	return IRQ_HANDLED;

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
		disable_charging();
	}
	disable_irq(BAT_REMOVAL_IRQ);

	return;
}

static void twl6030charger_ctrl_work_handler(struct work_struct *work)
{
	struct charger_device_info *di = container_of(work,
						      struct charger_device_info,
						      twl6030charger_ctrl_work.work);
	
	int ret;
	int n_usbic_state;
	
	int charger_fault = 0;
	long int events;
	u8 stat_toggle, stat_reset, stat_set = 0;
	u8 charge_state = 0;
	u8 present_charge_state = 0;
	u8 ac_or_vbus, no_ac_and_vbus = 0;
	u8 hw_state = 0, temp = 0;

	dev_dbg(di->dev, "[TA] %s start\n",__func__);

	clear_charge_start_time();
	
	/* read charger controller_stat1 */
	ret = twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &present_charge_state,
		CONTROLLER_STAT1);
	if (ret)
		return 0;

	twl_i2c_read_u8(TWL6030_MODULE_ID0, &hw_state, STS_HW_CONDITIONS);

	charge_state = di->stat1;

	stat_toggle = charge_state ^ present_charge_state;
	stat_set = stat_toggle & present_charge_state;
	stat_reset = stat_toggle & charge_state;

	no_ac_and_vbus = !((present_charge_state) & (VBUS_DET | VAC_DET));
	ac_or_vbus = charge_state & (VBUS_DET | VAC_DET);
	if (no_ac_and_vbus && ac_or_vbus) {
		dev_dbg(di->dev, "[TA] No Charging source\n");
		/* disable charging when no source present */
	}

	charge_state = present_charge_state;
	di->stat1 = present_charge_state;

	if (stat_reset & VBUS_DET) {
		/* On a USB detach, UNMASK VBUS OVP if masked*/
		twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &temp,
			CHARGERUSB_INT_MASK);
		if (temp & MASK_MCHARGERUSB_FAULT)
			twl_i2c_write_u8(TWL6030_MODULE_CHARGER,
				(temp & ~MASK_MCHARGERUSB_FAULT),
					CHARGERUSB_INT_MASK);
		dev_dbg(di->dev, "[TA] Charging source removed\n");
		change_cable_status(POWER_SUPPLY_TYPE_BATTERY, di);
#ifdef CONFIG_DYNAMIC_TSP_SETTINGS_FOR_CHARGER_STATE
		set_tsp_for_ta_detect(0);
#endif

	}

	if (stat_set & VBUS_DET) {
		/* In HOST mode (ID GROUND) when a device is connected, Mask
		 * VBUS OVP interrupt and do no enable usb charging
		 */
		if (hw_state & STS_USB_ID) {
			twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &temp,
				CHARGERUSB_INT_MASK);
			if (!(temp & MASK_MCHARGERUSB_FAULT))
				twl_i2c_write_u8(TWL6030_MODULE_CHARGER,
					(temp | MASK_MCHARGERUSB_FAULT),
						CHARGERUSB_INT_MASK);
		} else {
			n_usbic_state = get_real_usbic_state();
			dev_dbg(di->dev,
				"[TA] cable_detection_isr handler. usbic_state: %d\n",
				n_usbic_state);

			switch (n_usbic_state) {
			case MICROUSBIC_5W_CHARGER:
			case MICROUSBIC_TA_CHARGER:
			case MICROUSBIC_USB_CHARGER:
			case MICROUSBIC_PHONE_USB:
			case MICROUSBIC_USB_CABLE:
			case MICROUSBIC_MHL_CHARGER:
			case MICROUSBIC_JIG_USB_ON:
				if (sec_bci->charger.cable_status == POWER_SUPPLY_TYPE_USB
				    || sec_bci->charger.cable_status == POWER_SUPPLY_TYPE_MAINS ){
					dev_dbg(di->dev,
						"[TA] Already Plugged\n");
					break;
				}

				/*Check VF */
				sec_bci->battery.battery_vf_ok = check_battery_vf();

				/*TA or USB or MHL is inserted */
				if (n_usbic_state == MICROUSBIC_USB_CABLE) {
					//current : 395mA
					dev_dbg(di->dev,"[TA] USB CABLE PLUGGED\n");
					change_cable_status(POWER_SUPPLY_TYPE_USB, di);
#ifdef CONFIG_DYNAMIC_TSP_SETTINGS_FOR_CHARGER_STATE
					set_tsp_for_ta_detect(1);
#endif
				} else if (n_usbic_state == MICROUSBIC_MHL_CHARGER) {
					//current : 395mA
					dev_dbg(di->dev,"[TA] MHL CABLE PLUGGED\n");
					change_cable_status(POWER_SUPPLY_TYPE_USB, di);
#ifdef CONFIG_DYNAMIC_TSP_SETTINGS_FOR_CHARGER_STATE
					set_tsp_for_ta_detect(1);
#endif
				} else if (n_usbic_state == MICROUSBIC_JIG_USB_ON) {
					//current : 1000mA
					dev_dbg(di->dev,"[TA] JIG_USB_ON CABLE PLUGGED\n");
					change_cable_status(POWER_SUPPLY_TYPE_MAINS, di);
#ifdef CONFIG_DYNAMIC_TSP_SETTINGS_FOR_CHARGER_STATE
					set_tsp_for_ta_detect(1);
#endif
				} else {
					//current : 1000mA
					dev_dbg(di->dev,"[TA] CHARGER CABLE PLUGGED\n");
					change_cable_status(POWER_SUPPLY_TYPE_MAINS, di);
#ifdef CONFIG_DYNAMIC_TSP_SETTINGS_FOR_CHARGER_STATE
					set_tsp_for_ta_detect(1);
#endif
				}

				break;

			default:
				;
				
			}
		}
	}
	
	
	if (sec_bci->charger.prev_cable_status == -1
		&& sec_bci->charger.cable_status == -1) {
		dev_dbg(di->dev,"[TA] Fisrt time after bootig.\n");
		change_cable_status(POWER_SUPPLY_TYPE_BATTERY, di);
#ifdef CONFIG_DYNAMIC_TSP_SETTINGS_FOR_CHARGER_STATE
		set_tsp_for_ta_detect(0);
#endif
	}
	

	if (stat_set & CONTROLLER_STAT1_FAULT_WDG) {
//		charger_fault = 1;
		dev_dbg(di->dev, "Fault watchdog fired\n");
	}
	if (stat_reset & CONTROLLER_STAT1_FAULT_WDG)
		dev_dbg(di->dev, "Fault watchdog recovered\n");
	if (stat_set & CONTROLLER_STAT1_BAT_REMOVED)
		dev_dbg(di->dev, "Battery removed\n");
	if (stat_reset & CONTROLLER_STAT1_BAT_REMOVED)
		dev_dbg(di->dev, "Battery inserted\n");
	if (stat_set & CONTROLLER_STAT1_BAT_TEMP_OVRANGE)
		dev_dbg(di->dev, "Battery temperature overrange\n");
	if (stat_reset & CONTROLLER_STAT1_BAT_TEMP_OVRANGE)
		dev_dbg(di->dev, "Battery temperature within range\n");

	if (charger_fault) {
		change_charge_status(POWER_SUPPLY_STATUS_NOT_CHARGING);
		dev_err(di->dev, "Charger Fault stop charging\n");
	}

}

static void twl6030charger_fault_work_handler(struct work_struct *work)
{
	struct charger_device_info *di = container_of(work,
						      struct charger_device_info,
						      twl6030charger_fault_work.work);

	int charger_fault = 0;
	int ret;

	u8 usb_charge_sts = 0, usb_charge_sts1 = 0, usb_charge_sts2 = 0;

	ret = twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &usb_charge_sts,
						CHARGERUSB_INT_STATUS);
	ret = twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &usb_charge_sts1,
						CHARGERUSB_STATUS_INT1);
	ret = twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &usb_charge_sts2,
						CHARGERUSB_STATUS_INT2);

	di->status_int1 = usb_charge_sts1;
	di->status_int2 = usb_charge_sts2;
	if (usb_charge_sts & CURRENT_TERM_INT)
		dev_dbg(di->dev, "USB CURRENT_TERM_INT\n");
	if (usb_charge_sts & CHARGERUSB_THMREG)
		dev_dbg(di->dev, "USB CHARGERUSB_THMREG\n");
	if (usb_charge_sts & CHARGERUSB_FAULT)
		dev_dbg(di->dev, "USB CHARGERUSB_FAULT\n");

	if (usb_charge_sts1 & CHARGERUSB_STATUS_INT1_TMREG)
		dev_dbg(di->dev, "USB CHARGER Thermal regulation activated\n");
	if (usb_charge_sts1 & CHARGERUSB_STATUS_INT1_NO_BAT)
		dev_dbg(di->dev, "No Battery Present\n");
	if (usb_charge_sts1 & CHARGERUSB_STATUS_INT1_BST_OCP)
		dev_dbg(di->dev, "USB CHARGER Boost Over current protection\n");
	if (usb_charge_sts1 & CHARGERUSB_STATUS_INT1_TH_SHUTD) {
//		charger_fault = 1;
		dev_dbg(di->dev, "USB CHARGER Thermal Shutdown\n");
	}
	if (usb_charge_sts1 & CHARGERUSB_STATUS_INT1_BAT_OVP)
		dev_dbg(di->dev, "USB CHARGER Bat Over Voltage Protection\n");
	if (usb_charge_sts1 & CHARGERUSB_STATUS_INT1_POOR_SRC)
		dev_dbg(di->dev, "USB CHARGER Poor input source\n");
	if (usb_charge_sts1 & CHARGERUSB_STATUS_INT1_SLP_MODE)
		dev_dbg(di->dev, "USB CHARGER Sleep mode\n");
	if (usb_charge_sts1 & CHARGERUSB_STATUS_INT1_VBUS_OVP)
		dev_dbg(di->dev, "USB CHARGER VBUS over voltage\n");

	if (usb_charge_sts2 & CHARGE_DONE) 
		dev_dbg(di->dev, "CHARGE DONE\n");

	if (usb_charge_sts2 & CURRENT_TERM){
//		change_charge_status(POWER_SUPPLY_STATUS_FULL_END);
		dev_dbg(di->dev, "[TA] Charge FULL\n");
	}
	
	if (usb_charge_sts2 & ICCLOOP)
		dev_dbg(di->dev, "USB ICCLOOP\n");
	if (usb_charge_sts2 & ANTICOLLAPSE)
		dev_dbg(di->dev, "USB ANTICOLLAPSE\n");

	if (charger_fault) {
		change_charge_status(POWER_SUPPLY_STATUS_NOT_CHARGING);
		dev_err(di->dev, "Charger Fault stop charging\n");
	}
	dev_dbg(di->dev, "Charger fault detected STS, INT1, INT2 %x %x %x\n",
	    usb_charge_sts, usb_charge_sts1, usb_charge_sts2);

}

static int __devinit charger_probe(struct platform_device *pdev)
{

	int ret = 0;
	int irq = 0;
	u8 state = 0;

	struct charger_device_info *di;

	printk( KERN_INFO "[TA] Charger probe...\n");

	sec_bci = get_sec_bci();

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di)
		return -ENOMEM;

	platform_set_drvdata(pdev, di);
	di->dev = &pdev->dev;
	device_config = pdev->dev.platform_data;

#if 0
	printk( KERN_INFO "[TA] %d, %d, %d\n ",
	       device_config->VF_CHECK_USING_ADC,
	       device_config->VF_ADC_PORT, 
		   device_config->SUPPORT_CHG_ING_IRQ);
#endif

	this_dev = &pdev->dev;

	di->stat1 = 0x80;

	/*Init Work */
	INIT_DELAYED_WORK(&di->twl6030charger_ctrl_work,
					  twl6030charger_ctrl_work_handler);

	INIT_DELAYED_WORK(&di->twl6030charger_fault_work,
					  twl6030charger_fault_work_handler);

	INIT_DELAYED_WORK(&di->bat_removal_detection_work,
			  bat_removal_detection_work_handler);

	// Refer board_init_battery for checking resources

	CTRL_INT_IRQ = platform_get_irq(pdev,0);
	
	ret = request_threaded_irq(CTRL_INT_IRQ, NULL, twl6030charger_ctrl_isr, 
							   0,
							   pdev->name, di);
	if (ret) {
		dev_err(di->dev, "[TA] 1. could not request irq %d, status %d\n",
			CTRL_INT_IRQ, ret);
		goto ctrl_int_irq_fail;
	}

	CHARGERFAULT_INTR_IRQ = platform_get_irq(pdev,1);
	
	ret = request_threaded_irq(CHARGERFAULT_INTR_IRQ, NULL, twl6030charger_fault_isr, 
							   0,
							   pdev->name, di);

	if (ret) {
		dev_err(di->dev, "[TA] 2. could not request irq %d, status %d\n",
			CHARGERFAULT_INTR_IRQ, ret);
		goto chargerfault_irq_fail;
	}

	BAT_REMOVAL_IRQ = platform_get_irq(pdev, 2);
	ret =
		request_irq(BAT_REMOVAL_IRQ, bat_removal_detection_isr,
			    IRQF_DISABLED, pdev->name, di);
	if (ret) {
		dev_err(di->dev, "[TA] 3. could not request irq %d, status %d\n",
			BAT_REMOVAL_IRQ, ret);
		goto bat_removal_irq_fail;
	}
	set_irq_type(BAT_REMOVAL_IRQ, IRQ_TYPE_EDGE_BOTH);

	/* initialize for USB charging */

	/* MAX Charger Voltage/Current limit was set in the Bootloader */
	/* Please find the related code in the sbl_board_charger.c */

	/* twl6030_config_limit1_reg(di, MAX_CHARGER_VOLTAGE); */
	/* twl6030_config_limit2_reg(di, MAX_CHARGER_CURRENT); */
	
	twl_i2c_write_u8(TWL6030_MODULE_CHARGER, MBAT_TEMP,
			 CONTROLLER_INT_MASK);
	twl_i2c_write_u8(TWL6030_MODULE_CHARGER, MASK_MCHARGERUSB_THMREG,
			 CHARGERUSB_INT_MASK);
		
	twl_i2c_read_u8(TWL6030_MODULE_CHARGER, &state,
			CHARGERUSB_CTRL1);
	state |= TERM;
	twl_i2c_write_u8(TWL6030_MODULE_CHARGER, state,
			 CHARGERUSB_CTRL1);

	twl6030_interrupt_unmask(TWL6030_CHARGER_CTRL_INT_MASK,
						REG_INT_MSK_LINE_C);
	twl6030_interrupt_unmask(TWL6030_CHARGER_CTRL_INT_MASK,
						REG_INT_MSK_STS_C);
	twl6030_interrupt_unmask(TWL6030_CHARGER_FAULT_INT_MASK,
						REG_INT_MSK_LINE_C);
	twl6030_interrupt_unmask(TWL6030_CHARGER_FAULT_INT_MASK,
						REG_INT_MSK_STS_C);


	queue_delayed_work(sec_bci->sec_battery_workq,
			   &di->twl6030charger_ctrl_work,HZ);

	return 0;


bat_removal_irq_fail:
	irq = platform_get_irq(pdev, 1);
	free_irq(irq, di);

chargerfault_irq_fail:
	irq = platform_get_irq(pdev, 0);
	free_irq(irq, di);

ctrl_int_irq_fail:


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

static int charger_suspend(struct platform_device *pdev,
			   pm_message_t state)
{
	//disable_irq_wake( KCHG_ING_IRQ );
	//omap34xx_pad_set_padoff( KCHG_ING_GPIO, 0 );
	printk( KERN_INFO "%s", __func__);
	
	if(sec_bci->charger.cable_status != POWER_SUPPLY_TYPE_BATTERY) {

		twl_i2c_write_u8(TWL6030_MODULE_ID1, 0x00,
				 TWL6030_REG_GPADC_CTRL);
	}

	return 0;
}

static int charger_resume(struct platform_device *pdev)
{
	//omap34xx_pad_set_padoff( KCHG_ING_GPIO, 1 );
	//enable_irq_wake( KCHG_ING_IRQ );
	u8 state = 0;
	
	printk( KERN_INFO "%s", __func__);

	if(sec_bci->charger.cable_status != POWER_SUPPLY_TYPE_BATTERY) {

		twl_i2c_read_u8(TWL6030_MODULE_ID1, &state,
				TWL6030_REG_GPADC_CTRL);
		state |= TWL6030_GPADC_CTRL_SCALER_EN_CH11;
		twl_i2c_write_u8(TWL6030_MODULE_ID1, state,
				 TWL6030_REG_GPADC_CTRL);

	}
	
	return 0;
}

struct platform_driver charger_platform_driver = {
	.probe = &charger_probe,
	.remove = __devexit_p(charger_remove),
	.suspend = &charger_suspend,
	.resume = &charger_resume,
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













