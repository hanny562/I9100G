
/*
 * module/samsung_battery/battery_monitor.c
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
#include <linux/workqueue.h>
#include <linux/i2c/twl6030-gpadc.h>
#include <linux/i2c/twl.h>
#include <linux/power_supply.h>
#include <linux/wakelock.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/android_alarm.h>
#include <linux/i2c/twl.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <plat/microusbic.h>
#include "common.h"

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#ifdef DEBUG
#include <plat/microusbic.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#endif

#include <linux/gpio.h>
#include <plat/mux.h>

#if defined(CONFIG_MACH_T1_CHN)
#include <linux/rtc.h>
#endif

#define DRIVER_NAME "secBattMonitor"

#define TEMP_DEG    0
#define TEMP_ADC    1

#ifdef DEBUG
extern int get_real_usbic_state(void);
#endif

static DEFINE_MUTEX(battery_lock);

static int NCP15WB473J_batt_table[] = {

	/* -20C ~ 70 */

	/* 0[-20] ~ 19[-1] */
	815, 800, 786, 771, 757,
	742, 728, 713, 699, 684,
	670, 662, 654, 646, 637,
	629, 621, 613, 607, 600,

	/* 20[0] ~ 39[19] */
	594, 582, 570, 557, 545,
	533, 521, 509, 496, 484,
	472, 460, 449, 437, 425,
	414, 402, 390, 379, 367,

	/* 40[20] ~ 59[39] */
	356, 346, 337, 327, 318,
	309, 299, 290, 280, 271,
	262, 254, 247, 240, 233,
	225, 218, 211, 204, 196,

	/* 60[40] ~ 90[70] */
	189, 182, 176, 169, 164,
	159, 154, 149, 144, 139,
	135, 131, 127, 123, 119,
	115, 111, 107, 103, 99,
	95, 92, 89, 86, 83,
	81, 78, 75, 72, 70,
	69,

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

struct battery_device_info {
	struct device *dev;
	struct delayed_work battery_monitor_work;
	struct alarm        alarm;

	struct power_supply sec_battery;
	struct power_supply sec_ac;
	struct power_supply sec_usb;

	struct early_suspend early_suspend;
};

static struct device *this_dev;
static struct wake_lock sec_bc_wakelock;
static struct wake_lock lowbat_wakelock;
static int lowbat_wakelock_state = false;

static SEC_battery_charger_info sec_bci;
static struct battery_device_config *device_config;

#ifdef HIGH_TEMP_GAURD_FOR_CAMCORDER
static struct proc_dir_entry *sec_battery_proc, *camera_recording_proc;
#endif

static char *samsung_bci_supplied_to[] = {
	"battery",
};

SEC_battery_charger_info *get_sec_bci(void)
{
	return &sec_bci;
}

enum full_charge_mode {
	NOT_FULL = 0,
	DISPLAY_FULL,
	END_FULL,
};

/* Prototype */
int _charger_state_change_(int, int);
int _low_battery_alert_(void);
int _get_average_value_(int *, int);
int _get_t2adc_data_(int);
static int get_cable_status(void);
static int get_elapsed_time_secs(unsigned long long *);
static int t2adc_to_temperature(int, int);
static int get_battery_level_adc(void);
static int get_battery_level_ptg(void);
static int get_system_temperature(bool);
static int get_charging_current_adc_val(void);
static enum full_charge_mode check_full_charge_using_chg_current(int);
static int battery_monitor_core(void);
static void battery_monitor_work_handler(struct work_struct *);
static void battery_program_alarm(struct battery_device_info *, ktime_t);
static void battery_monitor_alarm(struct alarm *);
static int battery_proc_init(void);
static void battery_proc_remove(void);
static int proc_read_camera_recording(char *page, char **start,
				off_t off, int count, int *eof, void *data);
static int proc_write_camera_recording(struct file *filp,
				 const char *buffer, unsigned long count, void *data);
static int samsung_battery_get_property(struct power_supply *,
					enum power_supply_property,
					union power_supply_propval *);
static int samsung_ac_get_property(struct power_supply *,
				   enum power_supply_property,
				   union power_supply_propval *);
static int samsung_usb_get_property(struct power_supply *,
				    enum power_supply_property,
				    union power_supply_propval *);
static void samsung_pwr_external_power_changed(struct power_supply *);
static ssize_t store_event(struct kobject *kobj,
			   struct kobj_attribute *attr, const char *buf,
			   size_t size);
static int __devinit battery_probe(struct platform_device *);
static int __devexit battery_remove(struct platform_device *);
static void battery_early_suspend(struct early_suspend *);
static void battery_early_resume(struct early_suspend *);
static int __init battery_init(void);
static void __exit battery_exit(void);

/* Charger */
extern int charger_init(void);
extern void charger_exit(void);
extern int _battery_state_change_(int category, int value);
extern int _cable_status_now_(void);


/* Fuel Guage */
extern int fuelgauge_init(void);
extern void fuelgauge_exit(void);
extern int fuelgauge_quickstart(void);
extern int get_fuelgauge_adc_value(int count);
extern int get_fuelgauge_ptg_value(int);
extern unsigned long long sched_clock(void);

extern u32 sec_bootmode;

static bool boot_complete = false;
static int boot_monitor_count = 0;
int stop_temperature_overheat = CHARGE_STOP_TEMPERATURE_MAX;
int recover_temperature_overheat = CHARGE_RECOVER_TEMPERATURE_MAX;

#ifdef CONFIG_SEC_BATTERY_USE_RECOVERY_MODE
static int recovery_mode = 0;
module_param(recovery_mode, bool, 0);
#endif				/* CONFIG_SEC_BATTER_USE_RECOVERY_MODE */


// ------------------------------------------------------------------------- //
//                           sysfs interface                                 //
// ------------------------------------------------------------------------- //
#define  __ATTR_SHOW_CALLBACK( _name, _ret_val ) \
static ssize_t _name( struct kobject *kobj, \
		      struct kobj_attribute *attr,	\
		      char *buf )			\
{ \
    return sprintf ( buf, "%d\n", _ret_val ); \
}

static ssize_t store_fuelgauge_reset(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t size)
{
	printk(KERN_INFO "[BM] fuel gauge reset\n");
	fuelgauge_quickstart();
	return size;
}

static int get_batt_monitor_temp(void)
{
	return sec_bci.battery.support_monitor_temp;
}

static ssize_t store_batt_monitor_temp(struct kobject *kobj,
				       struct kobj_attribute *attr,
				       const char *buf, size_t size)
{
	int flag;
	sscanf(buf, "%d", &flag);
	printk(KERN_INFO "[BM] change value %d\n", flag);
	sec_bci.battery.support_monitor_temp = flag;
	sec_bci.battery.support_monitor_timeout = flag;
	sec_bci.battery.support_monitor_full = flag;

	return size;
}

static ssize_t store_batt_boot_complete(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t size)
{
	int flag;
	sscanf(buf, "%d", &flag);
	printk(KERN_INFO "[BM] boot complete flag:%d, buf:%s, size:%d\n", flag, buf, size);
	boot_complete = true;
	return size;
}

static ssize_t show_check_lp_charging( struct kobject *kobj,
		      struct kobj_attribute *attr, char *buf )
{
	int ret;

	if( device_config->boot_mode == 5 ) {
		ret = 1;
		if( sec_bci.charger.charge_status == POWER_SUPPLY_STATUS_DISCHARGING ) {
			if( device_config->pm_poweroff ) {
				device_config->pm_poweroff();
			} else {
				pr_info("pm_poweroff is NULL !!\n");
			}
		}
	} else {
		ret = 0;
	}
	sprintf ( buf, "%d\n", ret );
	printk(KERN_INFO "%s, boot_mode = %d, charge_status = %d\n", __func__,
		device_config->boot_mode, sec_bci.charger.charge_status);

    return ret;
}

static ssize_t show_battery_type( struct kobject *kobj,
		      struct kobj_attribute *attr, char *buf )
{
	return sprintf ( buf, "%s\n", "SDI_SDI" ); \
}

__ATTR_SHOW_CALLBACK(show_batt_vol, get_battery_level_adc() * 1000)
__ATTR_SHOW_CALLBACK(show_batt_vol_adc, 0)
__ATTR_SHOW_CALLBACK(show_batt_temp, get_system_temperature(TEMP_DEG)*10)
__ATTR_SHOW_CALLBACK(show_batt_temp_adc, get_system_temperature(TEMP_ADC))
__ATTR_SHOW_CALLBACK(show_batt_v_f_adc, 0)
__ATTR_SHOW_CALLBACK(show_batt_capacity, get_battery_level_ptg())
__ATTR_SHOW_CALLBACK(show_batt_monitor_temp, get_batt_monitor_temp())
__ATTR_SHOW_CALLBACK(show_batt_temp_check, sec_bci.battery.battery_health)
__ATTR_SHOW_CALLBACK(show_batt_full_check,
		     (sec_bci.charger.charge_status ==
		  POWER_SUPPLY_STATUS_FULL) ? 1 : 0)
__ATTR_SHOW_CALLBACK(show_charging_source, sec_bci.charger.cable_status)
#ifdef _OMS_FEATURES_
__ATTR_SHOW_CALLBACK(show_batt_vol_toolow,
		     sec_bci.battery.battery_vol_toolow)
static struct kobj_attribute batt_vol_toolow =
__ATTR(batt_vol_toolow, 0644, show_batt_vol_toolow, NULL);
#endif

static struct kobj_attribute batt_sysfs_testmode[] = {
	/* Event logging - Put these attributes at first position of this array
	   For using the call back function 'store_event'
	*/
	__ATTR(mp3, 0664, NULL, store_event),
	__ATTR(talk_wcdma, 0664, NULL, store_event),
	__ATTR(talk_gsm, 0664, NULL, store_event),
	__ATTR(data_call, 0664, NULL, store_event),
	__ATTR(vt_call, 0664, NULL, store_event),
	__ATTR(camera_preview, 0664, NULL, store_event),
	__ATTR(camera_recording, 0664, NULL, store_event),
	__ATTR(video, 0664, NULL, store_event),
	__ATTR(g_map, 0664, NULL, store_event),
	__ATTR(e_book, 0664, NULL, store_event),
	__ATTR(bt_call, 0664, NULL, store_event),
	__ATTR(wap_browsing, 0664, NULL, store_event),
	__ATTR(wifi_browsing, 0664, NULL, store_event),
	__ATTR(browser, 0664, NULL, store_event),
	__ATTR(game, 0664, NULL, store_event),
	/* END of Event logging */

	__ATTR(batt_vol, 0644, show_batt_vol, NULL),
	__ATTR(batt_vol_adc, 0644, show_batt_vol_adc, NULL),
	__ATTR(batt_temp, 0644, show_batt_temp, NULL),
	__ATTR(batt_temp_adc, 0644, show_batt_temp_adc, NULL),
	__ATTR(batt_v_f_adc, 0644, show_batt_v_f_adc, NULL),
	__ATTR(batt_capacity, 0644, show_batt_capacity, NULL),
	__ATTR(fg_reset_soc, 0664, NULL, store_fuelgauge_reset),
	__ATTR(batt_monitor_temp, 0644, show_batt_monitor_temp,
	       store_batt_monitor_temp),
	__ATTR(batt_boot_complete, 0644, NULL, store_batt_boot_complete),
	__ATTR(fg_read_soc, 0644, show_batt_capacity, NULL),
	__ATTR(batt_temp_check, 0644, show_batt_temp_check, NULL),
	__ATTR(batt_full_check, 0644, show_batt_full_check, NULL),
	__ATTR(charging_source, 0644, show_charging_source, NULL),
	__ATTR(batt_lp_charging, 0644, show_check_lp_charging, NULL),
	__ATTR(batt_type, 0644, show_battery_type, NULL),

};

/* Event logging */
u32 event_logging = 0;

enum {
	MP3 =0, TALK_WCDMA, TALK_GSM, DATA_CALL, VT_CALL, CAMERA_PREVIEW,
	CAMERA_RECORDING,
	VIDEO, G_MAP, E_BOOK, BT_CALL, WAP_BROWSING, WIFI_BROWSING, BROWSER,
	GAME
};
static ssize_t store_event(struct kobject *kobj,
			   struct kobj_attribute *attr,
			   const char *buf, size_t size)
{
	int flag;
	const ptrdiff_t off = attr - batt_sysfs_testmode;
	sscanf(buf, "%d", &flag);
	if (flag == 1)
		event_logging |= (0x1 << off);
	else if (flag == 0)
		event_logging &= ~(0x1 << off);

	printk("[BM] store_event offset=%d, value=%d\n", off, flag);
	return size;
}
/* END of Event logging */

#ifdef HIGH_TEMP_GAURD_FOR_CAMCORDER
static int battery_proc_init(void)
{
	int ret = 1;

	sec_battery_proc = proc_mkdir(DRIVER_NAME, NULL);
	if(sec_battery_proc == NULL)
		goto out;

	camera_recording_proc = create_proc_entry("camera_recording",
						  0666, sec_battery_proc );
	if( camera_recording_proc == NULL)
		goto out;

	camera_recording_proc->data = NULL;
	camera_recording_proc->read_proc = proc_read_camera_recording;
	camera_recording_proc->write_proc = proc_write_camera_recording;

	return 0;

out:
	return ret;
}

static void battery_proc_remove(void)
{
	remove_proc_entry("camera_recording",sec_battery_proc);
	remove_proc_entry(DRIVER_NAME, NULL);
}

static int proc_read_camera_recording(char *page, char **start,
		off_t off, int count, int *eof, void *data)
{
	int len;
	len = sprintf(page, "%d\n", sec_bci.charger.camera_recording);

	return len;
}

static int proc_write_camera_recording(struct file *filp,
	       const char __user *buffer, unsigned long count, void *data)
{
	int len = count;
	char camera_recording_status[2];

	if(copy_from_user(camera_recording_status, buffer, len))
		return -EFAULT;

	camera_recording_status[1] = '\0';

	sec_bci.charger.camera_recording =
		(int) simple_strtoul(camera_recording_status, NULL, 10 );

	printk(KERN_INFO "[BM] Camera recording : %d\n" ,
				sec_bci.charger.camera_recording);

	return len;
}

#endif

int _charger_state_change_(int category, int value)
{

	struct battery_device_info *di;
	struct platform_device *pdev;
	pdev = to_platform_device(this_dev);
	di = platform_get_drvdata(pdev);

	printk(KERN_DEBUG "[BM] cate: %d, value: %d\n", category, value);
	if (category == STATUS_CATEGORY_CABLE) {
		switch (value) {
		case POWER_SUPPLY_TYPE_BATTERY:
			/* Stop monitoring the batt. level for Re-charging */
			sec_bci.battery.monitor_field_rechg_vol = false;

			/* Stop monitoring the temperature */
			sec_bci.battery.monitor_field_temp = false;

			sec_bci.battery.confirm_full_by_current = 0;
			sec_bci.battery.confirm_recharge = 0;

			sec_bci.charger.charging_timeout = DEFAULT_CHARGING_TIMEOUT;
			break;

		case POWER_SUPPLY_TYPE_MAINS:
			sec_bci.charger.charging_timeout = DEFAULT_CHARGING_TIMEOUT;
			wake_lock_timeout(&sec_bc_wakelock, HZ);
			break;

		case POWER_SUPPLY_TYPE_USB:
			break;

		default:
			break;
		}

		goto Out_Charger_State_Change;
	} else if (category == STATUS_CATEGORY_CHARGING) {
		switch (value) {
		case POWER_SUPPLY_STATUS_UNKNOWN:
		case POWER_SUPPLY_STATUS_DISCHARGING:
			/* Stop monitoring the batt. level for Re-charging */
			sec_bci.battery.monitor_field_rechg_vol = false;

			if (sec_bci.battery.battery_health !=
			    POWER_SUPPLY_HEALTH_OVERHEAT
			    && sec_bci.battery.battery_health !=
			    POWER_SUPPLY_HEALTH_COLD) {
				/* Stop monitoring the temperature */
				sec_bci.battery.monitor_field_temp = false;
			}
			break;

		case POWER_SUPPLY_STATUS_NOT_CHARGING:
			break;

		case POWER_SUPPLY_STATUS_FULL:
			/* Start monitoring the batt. level for Re-charging */
			sec_bci.battery.monitor_field_rechg_vol = false;

			/* Start monitoring the temperature */
			sec_bci.battery.monitor_field_temp = true;
			break;

		case POWER_SUPPLY_STATUS_FULL_END:
			/* Start monitoring the batt. level for Re-charging */
			sec_bci.battery.monitor_field_rechg_vol = true;

			/* Start monitoring the temperature */
			sec_bci.battery.monitor_field_temp = true;
			break;

		case POWER_SUPPLY_STATUS_CHARGING:
			/* Start monitoring the temperature */
			sec_bci.battery.monitor_field_temp = true;

			/* Stop monitoring the batt. level for Re-charging */
			sec_bci.battery.monitor_field_rechg_vol = false;
			break;

		case POWER_SUPPLY_STATUS_RECHARGING_FOR_FULL:
			/* Start monitoring the temperature */
			sec_bci.battery.monitor_field_temp = true;

			/* Stop monitoring the batt. level for Re-charging */
			sec_bci.battery.monitor_field_rechg_vol = false;
			break;

		case POWER_SUPPLY_STATUS_RECHARGING_FOR_TEMP:
			/*Start monitoring the temperature */
			sec_bci.battery.monitor_field_temp = true;

			/*Stop monitoring the batt. level for Re-charging */
			sec_bci.battery.monitor_field_rechg_vol = false;
			break;

		default:
			break;
		}

	}

	power_supply_changed(&di->sec_battery);
	power_supply_changed(&di->sec_ac);
	power_supply_changed(&di->sec_usb);

	cancel_delayed_work(&di->battery_monitor_work);
	queue_delayed_work(sec_bci.sec_battery_workq,
					   &di->battery_monitor_work, 5 * HZ);

Out_Charger_State_Change:
	return 0;
}

int _low_battery_alert_()
{
	struct battery_device_info *di;
	struct platform_device *pdev;
	int batt_ptg;
	int batt_vol;

	pdev = to_platform_device(this_dev);
	di = platform_get_drvdata(pdev);

	batt_ptg = get_battery_level_ptg();
	batt_vol = get_battery_level_adc();

	printk("[BM] Check low battery level : %d%%, %dmV\n", batt_ptg, batt_vol);
	sec_bci.battery.battery_level_ptg = batt_ptg;

	wake_lock_timeout(&sec_bc_wakelock, HZ);
	power_supply_changed(&di->sec_battery);

	return 0;
}

int _get_average_value_(int *data, int count)
{
	int average;
	int min = 0;
	int max = 0;
	int sum = 0;
	int i;

	if (count >= 5) {
		min = max = data[0];
		for (i = 0; i < count; i++) {
			if (data[i] < min)
				min = data[i];

			if (data[i] > max)
				max = data[i];

			sum += data[i];
		}
		average = (sum - max - min) / (count - 2);
	} else {
		for (i = 0; i < count; i++)
			sum += data[i];

		average = sum / count;
	}

	return average;
}

int _get_t2adc_data_(int ch)
{
	int ret = 0;
	int val[5];
	int i;
	struct twl6030_gpadc_request req;


	req.channels = (1 << ch);
	req.method = TWL6030_GPADC_SW2;
	req.active = 0;
	req.func_cb = NULL;

	for (i = 0; i < 5; i++) {
		twl6030_gpadc_conversion(&req);
		val[i] = req.rbuf[ch];
	}

	ret = _get_average_value_(val, 5);

	return ret;
}

static int get_cable_status(void)
{
	int ret = 0;
	ret = gpio_get_value(OMAP_GPIO_TA_NCONNECTED) ? 0 : 1;
	return ret;
}

static int get_elapsed_time_secs(unsigned long long *start)
{
	unsigned long long now;
	unsigned long long diff;
	unsigned long long max = 0xFFFFFFFF;

	max = (max << 32) | 0xFFFFFFFF;

	now = sched_clock();

	if (now >= *start) {
		diff = now - *start;
	} else {
		sec_bci.charger.charge_start_time = now;
		diff = 0;
		//diff = max - *start + now;
	}

	do_div(diff, 1000000000L);
	/*
	  printk(KERN_DEBUG "[BM] now: %llu, start: %llu, diff:%d\n",
	  now, *start, (int)diff );
	*/
	return (int) diff;
}

static int t2adc_to_temperature(int value, int channel)
{
	int temp;
	int threshold_value;

	/*calculating temperature */
	for (temp = 90; temp >= 0; temp--) {
		if ( temp == 0 ) {
			threshold_value = NCP15WB473J_batt_table[temp];
		} else {
			threshold_value =
				(NCP15WB473J_batt_table[temp] + NCP15WB473J_batt_table[temp-1]) / 2;
		}

		if ( (threshold_value - value) >= 0)
			break;
	}
	temp = temp - 20;

	return temp;
}

void check_lowbat_wakelock_condition(void) {
	/*
	 * lowbat wakelock condition
	 * lock condition : soc <= 1% or not charging(soc will be getting low)
	 * unlock condition : soc > 1% and charging(==TA/USB connected, soc will be getting high)
	 */
	if ((lowbat_wakelock_state == false) &&
		((sec_bci.battery.battery_level_ptg <= 1) && (sec_bci.charger.charge_status != POWER_SUPPLY_STATUS_CHARGING))) {
		wake_lock(&lowbat_wakelock);
		lowbat_wakelock_state = true;
		printk(KERN_INFO "[BM] Low battery wakelock : lock(SOC:%d%%, ChargeStatus:%d)\n",
			sec_bci.battery.battery_level_ptg, sec_bci.charger.charge_status);
	} else if ((lowbat_wakelock_state == true) &&
		((sec_bci.battery.battery_level_ptg > 1) || (sec_bci.charger.charge_status == POWER_SUPPLY_STATUS_CHARGING))){
		wake_unlock(&lowbat_wakelock);
		lowbat_wakelock_state = false;
		printk(KERN_INFO "[BM] Low battery wakelock : unlock(SOC:%d%%, ChargeStatus:%d)\n",
			sec_bci.battery.battery_level_ptg, sec_bci.charger.charge_status);
	}
}

static int get_battery_level_adc(void)
{
	int value;
	value = get_fuelgauge_adc_value(5);
	if (value < 0)
		value = sec_bci.battery.battery_level_vol;

	return value;
}

static int get_battery_level_ptg(void)
{
	int batt_ptg;
	int batt_adc;
	
	batt_ptg = get_fuelgauge_ptg_value(ADJUSTED_SOC);
	batt_adc = get_battery_level_adc();
	
	if (sec_bci.charger.charge_status == POWER_SUPPLY_STATUS_FULL)
		batt_ptg = 100;

	if ((!boot_complete && batt_ptg <= 0)
	     || (batt_ptg == 0 && batt_adc > 3380)) {
		printk( KERN_INFO "[BM] raw       Batt SOC : %d, Batt Vol : %d\n",
							batt_ptg, batt_adc);
		batt_ptg = 1;
		printk( KERN_INFO "[BM] corrected Batt SOC : %d, Batt Vol : %d\n", 
							batt_ptg, batt_adc);
	}
	
	check_lowbat_wakelock_condition();

	return batt_ptg;
}

static int get_system_temperature(bool flag)
{
	int adc;
	int temp;
	adc = _get_t2adc_data_(device_config->TEMP_ADC_PORT);

	if (flag)
		return adc;

	temp = t2adc_to_temperature(adc, device_config->TEMP_ADC_PORT);

	return temp;
}

static int get_charging_current_adc_val(void)
{
	int adc;
	adc = _get_t2adc_data_(device_config->CHG_CURRENT_ADC_PORT);

	return adc;
}

static enum full_charge_mode check_full_charge_using_chg_current(int charge_current_adc)
{

	if (sec_bci.battery.battery_level_vol < 4000) {
		sec_bci.battery.confirm_full_by_current = 0;
		return NOT_FULL;
	}

	if (sec_bci.battery.support_monitor_full) {
		if (charge_current_adc <= CHARGE_FULL_CURRENT_DISPLAY_ADC
		    && charge_current_adc > CHARGE_FULL_CURRENT_END_ADC) {
			sec_bci.battery.confirm_full_by_current++;

			/* changing freq. of monitoring adc to Burst. */
			sec_bci.battery.monitor_duration = 5;
			sec_bci.battery.monitor_duration_sleep = 30;

		} else if (charge_current_adc <= CHARGE_FULL_CURRENT_END_ADC) {
			sec_bci.battery.monitor_duration = MONITOR_DEFAULT_DURATION;
			sec_bci.battery.monitor_duration_sleep = MONITOR_DURATION_DUR_SLEEP;

			sec_bci.battery.confirm_full_by_current = 0;

			return END_FULL;

		} else {
			sec_bci.battery.confirm_full_by_current = 0;

			/* changing freq. of monitoring adc to Default. */
			sec_bci.battery.monitor_duration = MONITOR_DEFAULT_DURATION;
			sec_bci.battery.monitor_duration_sleep = MONITOR_DURATION_DUR_SLEEP;
		}

		if (sec_bci.battery.confirm_full_by_current >= 4) {
			sec_bci.battery.monitor_duration = MONITOR_DEFAULT_DURATION;
			sec_bci.battery.monitor_duration_sleep = MONITOR_DURATION_DUR_SLEEP;

			sec_bci.battery.confirm_full_by_current = 0;

			return DISPLAY_FULL;
		}
	}
	return NOT_FULL;
}

static int battery_monitor_core(void)
{
	int charging_time;
	int rechg_voltage;
	struct battery_device_info *di;
	struct platform_device *pdev;

	pdev = to_platform_device(this_dev);
	di = platform_get_drvdata(pdev);

	if (event_logging) {
		stop_temperature_overheat = CHARGE_STOP_TEMPERATURE_EVENT;
		recover_temperature_overheat = CHARGE_RECOVER_TEMPERATURE_EVENT;
	} else {
		stop_temperature_overheat = CHARGE_STOP_TEMPERATURE_MAX;
		recover_temperature_overheat = CHARGE_RECOVER_TEMPERATURE_MAX;
	}

	/*Monitoring the system temperature */
	if (sec_bci.battery.monitor_field_temp) {
		if (sec_bci.battery.support_monitor_timeout) {
			/*Check charging time */
			charging_time =
				get_elapsed_time_secs(&sec_bci.charger.charge_start_time);
			if (charging_time >= sec_bci.charger.charging_timeout) {
				_battery_state_change_(STATUS_CATEGORY_CHARGING,
									   POWER_SUPPLY_STATUS_CHARGING_OVERTIME);
				return -1;
			}
		}

		if (sec_bci.battery.support_monitor_temp) {
#ifdef HIGH_TEMP_GAURD_FOR_CAMCORDER
			if(sec_bci.charger.charge_status == POWER_SUPPLY_STATUS_CHARGING) {
				if(sec_bci.charger.camera_recording){
					if(sec_bci.battery.battery_temp
					   >= HIGH_TEMP_GAURD_FOR_CAMCORDER_THRSHOLD) {
						if(!sec_bci.charger.camera_recording_inout) {
						printk(KERN_INFO "[TA] In camera recording\n");
						printk(KERN_INFO "[TA] HIGH TEMP is detected\n");
						sec_bci.charger.camera_recording_inout = 1;
						_battery_state_change_(STATUS_CATEGORY_TEMP,
								       BATTERY_TEMPERATURE_ETC);
						}
					} else {
						if(sec_bci.charger.camera_recording_inout) {
						sec_bci.charger.camera_recording_inout = 0;
						printk(KERN_INFO "[TA] In camera recording\n");
						printk(KERN_INFO "[TA] TEMP is recovred\n");
						_battery_state_change_(STATUS_CATEGORY_TEMP,
								       BATTERY_TEMPERATURE_ETC);
						}
					}
				} else {
					if(sec_bci.charger.camera_recording_inout) {
						printk(KERN_INFO "[TA] Out camera recording\n");
						sec_bci.charger.camera_recording_inout = 0;
						_battery_state_change_(STATUS_CATEGORY_TEMP,
								       BATTERY_TEMPERATURE_ETC);
					}
				}

			} else
				sec_bci.charger.camera_recording_inout = 0;
#endif
			if (sec_bci.battery.battery_health == POWER_SUPPLY_HEALTH_OVERHEAT
			    || sec_bci.battery.battery_health == POWER_SUPPLY_HEALTH_COLD) {
				if (sec_bci.battery.battery_temp <= recover_temperature_overheat
					&& sec_bci.battery.battery_temp >= CHARGE_RECOVER_TEMPERATURE_MIN) {
					sec_bci.battery.battery_health = POWER_SUPPLY_HEALTH_GOOD;
					_battery_state_change_(STATUS_CATEGORY_TEMP,
							       BATTERY_TEMPERATURE_NORMAL);
				}
			} else {
				if (sec_bci.battery.monitor_duration >  MONITOR_TEMP_DURATION)
					sec_bci.battery.monitor_duration = MONITOR_TEMP_DURATION;

				if (sec_bci.battery.battery_temp >= stop_temperature_overheat) {
					printk(KERN_INFO "[TA] Temperature is high (%d*)\n",
					       sec_bci.battery.battery_temp);
					if (sec_bci.battery.battery_health != POWER_SUPPLY_HEALTH_OVERHEAT) {
						sec_bci.battery.battery_health = POWER_SUPPLY_HEALTH_OVERHEAT;
						_battery_state_change_(STATUS_CATEGORY_TEMP,
								       BATTERY_TEMPERATURE_HIGH);
					}
				} else if (sec_bci.battery.battery_temp <= CHARGE_STOP_TEMPERATURE_MIN) {
					printk(KERN_INFO "[TA] Temperature is low (%d*)\n",
					       sec_bci.battery.battery_temp);
					if (sec_bci.battery.battery_health != POWER_SUPPLY_HEALTH_COLD) {
						sec_bci.battery.battery_health = POWER_SUPPLY_HEALTH_COLD;
						_battery_state_change_(STATUS_CATEGORY_TEMP,
								       BATTERY_TEMPERATURE_LOW);
					}
				} else {
					if (sec_bci.battery.battery_health != POWER_SUPPLY_HEALTH_GOOD) {
						sec_bci.battery.battery_health = POWER_SUPPLY_HEALTH_GOOD;
						_battery_state_change_(STATUS_CATEGORY_TEMP,
								       BATTERY_TEMPERATURE_NORMAL);
					}
				}
			}
		}
	}

	/* Monitoring the battery level for Re-charging */
	if (sec_bci.battery.monitor_field_rechg_vol
	    && sec_bci.charger.rechg_count <= 0 ) {
		if (sec_bci.battery.monitor_duration > MONITOR_RECHG_VOL_DURATION)
			sec_bci.battery.monitor_duration = MONITOR_RECHG_VOL_DURATION;

		if (sec_bootmode == 5)	// offmode charging
			rechg_voltage = CHARGE_RECHG_VOLTAGE_OFFMODE;
		else
			rechg_voltage = CHARGE_RECHG_VOLTAGE;

		if (sec_bci.battery.battery_level_vol <= rechg_voltage) {
			sec_bci.battery.confirm_recharge++;
			if (sec_bci.battery.confirm_recharge >= 2) {
				printk(KERN_INFO "[BM] RE-charging vol rechg_voltage = %d\n",
				       rechg_voltage);
				sec_bci.battery.confirm_recharge = 0;
				_battery_state_change_(STATUS_CATEGORY_CHARGING,
						       POWER_SUPPLY_STATUS_RECHARGING_FOR_FULL);
			}
		} else
			sec_bci.battery.confirm_recharge = 0;
	}

	return 0;
}

#if defined(CONFIG_MACH_T1_CHN)
struct rtc_time current_alarm_time;
extern int twl_i2c_read_u8(u8 mod_no, u8 *value, u8 reg);
extern unsigned bcd2bin(unsigned char val);

static int check_alarm_boot_kernel()
{
	int ret;
	u8 i;
	unsigned long time_sec, alarm_sec;
	u8 t_time[6];
	struct rtc_time current_rtc_time;

	// current time read
	for(i = 0x00 ; i < 0x06 ; i++){
		ret = twl_i2c_read_u8(0x16,&t_time[i], i);
		if (ret < 0)
		pr_err("twl_rtc: Could not read TWL"
		       "register %X - error %d\n", i, ret);
	}

	current_rtc_time.tm_sec = bcd2bin(t_time[0]);
	current_rtc_time.tm_min = bcd2bin(t_time[1]);
	current_rtc_time.tm_hour = bcd2bin(t_time[2]);
	current_rtc_time.tm_mday = bcd2bin(t_time[3]);
	current_rtc_time.tm_mon = bcd2bin(t_time[4]) - 1;
	current_rtc_time.tm_year = bcd2bin(t_time[5]) + 100;

	rtc_tm_to_time(&current_rtc_time, &time_sec);

	rtc_tm_to_time(&current_alarm_time, &alarm_sec);

	printk("[Alarm Boot]check_alarm_boot_kernel :"
		"Diff(alarm, time) =%ld\n", alarm_sec - time_sec);

	if((time_sec > alarm_sec - 25) && (time_sec < alarm_sec + 5))
		return 1;

	return 0;
}
#endif

static void battery_monitor_work_handler(struct work_struct *work)
{
	int charge_current_adc;
	enum full_charge_mode is_full;
#if defined(CONFIG_MACH_T1_CHN)
	int ret = 0;
	u8 alm_en = 0;
	extern void machine_restart(char *cmd);
#endif

	struct battery_device_info *di = container_of(work,
						  struct  battery_device_info,
						  battery_monitor_work.work);

	boot_monitor_count++;
	if (!boot_complete && boot_monitor_count >= 2) {
		printk(KERN_DEBUG "[BM] boot complete \n");
		boot_complete = true;
	}
	if (sec_bci.charger.rechg_count > 0)
		sec_bci.charger.rechg_count--;

	if (device_config->MONITORING_SYSTEM_TEMP)
		sec_bci.battery.battery_temp = get_system_temperature(TEMP_DEG);
	else
		sec_bci.battery.battery_temp = 0;

	/* Monitoring the battery info. */
	sec_bci.battery.battery_level_ptg = get_battery_level_ptg();
	msleep(10);
	sec_bci.battery.battery_level_vol = get_battery_level_adc();

	wake_lock_timeout(&sec_bc_wakelock, HZ);
	power_supply_changed(&di->sec_battery);

	if (!(sec_bci.battery.monitor_field_temp)
	    && !(sec_bci.battery.monitor_field_rechg_vol)) {
		sec_bci.battery.monitor_duration = MONITOR_DEFAULT_DURATION;
	} else {
		// Workaround : check status of cabel at this point.
		if (!_cable_status_now_()) {
			_battery_state_change_(STATUS_CATEGORY_ETC,
					       ETC_CABLE_IS_DISCONNECTED);
		}

		if (sec_bci.charger.is_charging
		    && device_config->MONITORING_CHG_CURRENT
		    && sec_bci.battery.battery_level_vol >= CHARGE_FULL_VOLTAGE) {
			/* in charging && enable monitor_chg_current       */
			/*	       && greater than CHARGE_FULL_VOLTAGE */
			charge_current_adc = get_charging_current_adc_val();
			is_full =
				check_full_charge_using_chg_current(charge_current_adc);

			switch(is_full) {
			case DISPLAY_FULL:
				_battery_state_change_(STATUS_CATEGORY_CHARGING,
						       POWER_SUPPLY_STATUS_FULL);
				break;

			case END_FULL:
				_battery_state_change_(STATUS_CATEGORY_CHARGING,
						       POWER_SUPPLY_STATUS_FULL_END);
				break;
			default:
				battery_monitor_core();
				break;
			}
		} else {
			battery_monitor_core();
		}
	}

#if defined(CONFIG_MACH_T1_CHN)
	ret = twl_i2c_read_u8(0x16, &alm_en, 0x12);
	if((alm_en&0x08) && sec_bootmode == 5) {		
		printk("battery_monitor_work_handler : alarm_en[0x%x] alarm boot check START !!!\n", alm_en);
		if(check_alarm_boot_kernel()){
			printk( "IT_ALARM is 1\n");
			machine_restart(NULL);
		}
	}
#endif

	printk(KERN_INFO "[BM] monitor BATT.(%d%%(%d%%), %dmV, %d*, cnt=%d, Charging=%d, Type=%d, ChgCur:%d)\n",
		sec_bci.battery.battery_level_ptg,
		get_fuelgauge_ptg_value(RAW_SOC),
		sec_bci.battery.battery_level_vol,
		sec_bci.battery.battery_temp,
		boot_monitor_count,
		sec_bci.charger.is_charging,
		sec_bci.charger.cable_status,
		get_charging_current_adc_val()
		);
#ifdef DEBUG
	printk(KERN_DEBUG "[BM] Temp ADC : %d\n", get_system_temperature(TEMP_ADC));
	if (1) {
		struct file *filp;
		char buf[1024];
		int count;
		int retval = 0;
		mm_segment_t fs;

		fs = get_fs();
		set_fs(KERNEL_DS);
		filp = filp_open("/tmp/b", 00000100 /*O_CREAT */  | 00000002,
				 0666);
		memset(buf, 0, 1024);
		sprintf(buf,
			"Voltage : %d,\nSOC : %d,\nCHG CURRENT MONITORING : %d,\nCABLE STATUS : %d,\nUSBIC STATUS : %d,\n",
			sec_bci.battery.battery_level_vol,
			sec_bci.battery.battery_level_ptg,
			get_charging_current_adc_val(),
			sec_bci.charger.cable_status, get_real_usbic_state()
		    );
		count = filp->f_op->write(filp, &buf, 1024, &filp->f_pos);
		retval = filp_close(filp, NULL);
		set_fs(fs);
	}
#endif

	power_supply_changed(&di->sec_battery);
	power_supply_changed(&di->sec_ac);
	power_supply_changed(&di->sec_usb);

	queue_delayed_work(sec_bci.sec_battery_workq,
		&di->battery_monitor_work,
		sec_bci.battery.monitor_duration * HZ);
}

static void battery_program_alarm(struct battery_device_info *di, ktime_t start_time)
{
	ktime_t interval = ktime_set(sec_bci.battery.monitor_duration_sleep, 0);
	ktime_t slack = ktime_set(20, 0);
	ktime_t next;

	next = ktime_add(start_time, interval);

	printk(KERN_DEBUG "[BM] Wake-up alarm set : now->%d, next->%d \n",
	       start_time.tv.sec, next.tv.sec);

	alarm_start_range(&di->alarm, next, ktime_add(next, slack));
}

static void battery_monitor_alarm(struct alarm *alarm)
{
}

// ------------------------------------------------------------------------- //
//                            Power supply monitor                           //
// ------------------------------------------------------------------------- //
static enum power_supply_property samsung_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
#ifdef _OMS_FEATURES_
	POWER_SUPPLY_PROP_TEMP,
#endif
	POWER_SUPPLY_PROP_CAPACITY,	// in percents
};

static enum power_supply_property samsung_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

static enum power_supply_property samsung_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

static int samsung_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (sec_bci.charger.charge_status == POWER_SUPPLY_STATUS_FULL_END
			|| sec_bci.charger.charge_status == POWER_SUPPLY_STATUS_RECHARGING_FOR_FULL)
			val->intval = POWER_SUPPLY_STATUS_FULL;
		else if (sec_bci.charger.charge_status == POWER_SUPPLY_STATUS_RECHARGING_FOR_TEMP)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else
			val->intval = sec_bci.charger.charge_status;

		break;

	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = sec_bci.battery.battery_health;
		break;

	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = sec_bci.battery.battery_level_vol * 1000;
		val->intval = val->intval <= 0 ? 0 : 1;
		break;

	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = sec_bci.charger.cable_status;
		break;

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = sec_bci.battery.battery_technology;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = sec_bci.battery.battery_level_vol * 1000;
		break;

#ifdef _OMS_FEATURES_
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = sec_bci.battery.battery_temp * 10;
		break;
#endif
	case POWER_SUPPLY_PROP_CAPACITY:	/* in percents! */
		val->intval = sec_bci.battery.battery_level_ptg;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int samsung_ac_get_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (sec_bci.charger.cable_status == POWER_SUPPLY_TYPE_MAINS)
			val->intval = 1;
		else
			val->intval = 0;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = sec_bci.battery.battery_level_vol * 1000;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int samsung_usb_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (sec_bci.charger.cable_status == POWER_SUPPLY_TYPE_USB)
			val->intval = 1;
		else
			val->intval = 0;

		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = sec_bci.battery.battery_level_vol * 1000;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static void samsung_pwr_external_power_changed(struct power_supply *psy)
{
}

// ------------------------------------------------------------------------- //
//                           Driver interface                                //
// ------------------------------------------------------------------------- //
static int __devinit battery_probe(struct platform_device *pdev)
{
	int ret = 0;
	int i = 0;
#if defined(CONFIG_MACH_T1_CHN)
	static u8 t_alarm_time[6];
#endif

	struct battery_device_info *di;

	pr_info("[BM] Battery Probe..\n");

	this_dev = &pdev->dev;

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di)
		return -ENOMEM;

	platform_set_drvdata(pdev, di);

	di->dev = &pdev->dev;
	device_config = pdev->dev.platform_data;

	INIT_DELAYED_WORK(&di->battery_monitor_work,
			  battery_monitor_work_handler);

	/* Create power supplies */
	di->sec_battery.name = "battery";
	di->sec_battery.type = POWER_SUPPLY_TYPE_BATTERY;
	di->sec_battery.properties = samsung_battery_props;
	di->sec_battery.num_properties = ARRAY_SIZE(samsung_battery_props);
	di->sec_battery.get_property = samsung_battery_get_property;
	di->sec_battery.external_power_changed = samsung_pwr_external_power_changed;

	di->sec_ac.name = "ac";
	di->sec_ac.type = POWER_SUPPLY_TYPE_MAINS;
	di->sec_ac.supplied_to = samsung_bci_supplied_to;
	di->sec_ac.num_supplicants = ARRAY_SIZE(samsung_bci_supplied_to);
	di->sec_ac.properties = samsung_ac_props;
	di->sec_ac.num_properties = ARRAY_SIZE(samsung_ac_props);
	di->sec_ac.get_property = samsung_ac_get_property;
	di->sec_ac.external_power_changed = samsung_pwr_external_power_changed;

	di->sec_usb.name = "usb";
	di->sec_usb.type = POWER_SUPPLY_TYPE_USB;
	di->sec_usb.supplied_to = samsung_bci_supplied_to;
	di->sec_usb.num_supplicants = ARRAY_SIZE(samsung_bci_supplied_to);
	di->sec_usb.properties = samsung_usb_props;
	di->sec_usb.num_properties = ARRAY_SIZE(samsung_usb_props);
	di->sec_usb.get_property = samsung_usb_get_property;
	di->sec_usb.external_power_changed = samsung_pwr_external_power_changed;

#ifdef CONFIG_HAS_EARLYSUSPEND
	di->early_suspend.suspend = battery_early_suspend;
	di->early_suspend.resume = battery_early_resume;
	di->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1;
	register_early_suspend(&di->early_suspend);
#endif

	printk(KERN_DEBUG "[BM] Cable status in %s : %d \n", __func__, get_cable_status());

	sec_bci.charger.charge_status = get_cable_status();

#ifdef HIGH_TEMP_GAURD_FOR_CAMCORDER
	ret = battery_proc_init();
	if(ret) {
		printk("[BM] Failed to make proc node for camera_recording\n");
		goto batt_regi_fail;
	}
#endif

	alarm_init(&di->alarm, ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP,
			   battery_monitor_alarm);

	ret = power_supply_register(&pdev->dev, &di->sec_battery);
	if (ret) {
		printk("[BM] Failed to register main battery, charger\n");
		goto batt_regi_fail1;
	}

	ret = power_supply_register(&pdev->dev, &di->sec_ac);
	if (ret) {
		printk("[BM] Failed to register ac\n");
		goto batt_regi_fail2;
	}

	ret = power_supply_register(&pdev->dev, &di->sec_usb);
	if (ret) {
		printk("[BM] Failed to register usb\n");
		goto batt_regi_fail3;
	}

#ifdef _OMS_FEATURES_
	/* Create battery sysfs files for sharing battery information with platform. */
	ret =
		sysfs_create_file(&di->sec_battery.dev->kobj,
			  &batt_vol_toolow.attr);
	if (ret) {
		printk("[BM] sysfs create fail - %s\n", batt_vol_toolow.attr.name);
	}
#endif

	for (i = 0; i < ARRAY_SIZE(batt_sysfs_testmode); i++) {
		ret = sysfs_create_file(&di->sec_battery.dev->kobj,
					&batt_sysfs_testmode[i].attr);
		if (ret) {
			printk("[BM] sysfs create fail - %s\n",
			       batt_sysfs_testmode[i].attr.name);
		}
	}

	/* Enable GPADC for monitoring temperature  */
	get_system_temperature(TEMP_DEG);

#ifdef CONFIG_SEC_BATTERY_USE_RECOVERY_MODE
	if (likely(recovery_mode == 0))
		queue_delayed_work(sec_bci.sec_battery_workq,
				   &di->battery_monitor_work, HZ / 2);
	else
		queue_delayed_work(sec_bci.sec_battery_workq,
				   &di->battery_monitor_work, 0);
#else
	queue_delayed_work(sec_bci.sec_battery_workq,
			   &di->battery_monitor_work, HZ / 2);
#endif
	sec_bci.ready = true;

#if defined(CONFIG_MACH_T1_CHN)
	// alarm time read
	for(i = 0x00 ; i < 0x06 ; i++){
		ret = twl_i2c_read_u8(0x16,&t_alarm_time[i], i+0x08);
		if (ret < 0)
		pr_err("twl_rtc: Could not read TWL"
		       "register %X - error %d\n", i+0x08, ret);
	}
	current_alarm_time.tm_sec = bcd2bin(t_alarm_time[0]);
	current_alarm_time.tm_min = bcd2bin(t_alarm_time[1]);
	current_alarm_time.tm_hour = bcd2bin(t_alarm_time[2]);
	current_alarm_time.tm_mday = bcd2bin(t_alarm_time[3]);
	current_alarm_time.tm_mon = bcd2bin(t_alarm_time[4]) - 1;
	current_alarm_time.tm_year = bcd2bin(t_alarm_time[5]) + 100;
#endif

	return 0;

batt_regi_fail3:
	power_supply_unregister(&di->sec_ac);

batt_regi_fail2:
	power_supply_unregister(&di->sec_battery);

batt_regi_fail1:
	alarm_cancel(&di->alarm);

batt_regi_fail:
	kfree(di);

	return ret;
}

static int __devexit battery_remove(struct platform_device *pdev)
{
	struct battery_device_info *di = platform_get_drvdata(pdev);

#ifdef HIGH_TEMP_GAURD_FOR_CAMCORDER
	battery_proc_remove();
#endif
	alarm_cancel(&di->alarm);

	flush_scheduled_work();
	cancel_delayed_work(&di->battery_monitor_work);

	power_supply_unregister(&di->sec_ac);
	power_supply_unregister(&di->sec_battery);

	platform_set_drvdata(pdev, NULL);
	kfree(di);

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void battery_early_suspend(struct early_suspend *handler)
{
	struct battery_device_info *di = container_of(handler,
						      struct battery_device_info,
						      early_suspend);
	printk(KERN_DEBUG "%s\n", __func__);

	if( sec_bci.charger.cable_status == POWER_SUPPLY_TYPE_BATTERY
	    && sec_bci.battery.battery_level_ptg > 1 )
		cancel_delayed_work(&di->battery_monitor_work);

}

static void battery_early_resume(struct early_suspend *handler)
{
	struct battery_device_info *di = container_of(handler,
						      struct battery_device_info,
						      early_suspend);
	printk(KERN_DEBUG "%s\n", __func__);

	cancel_delayed_work(&di->battery_monitor_work);
	queue_delayed_work(sec_bci.sec_battery_workq,
		&di->battery_monitor_work, HZ * 1);
}
#endif

static int battery_suspend_alarm(struct device *dev)
{
	struct battery_device_info *di = dev_get_drvdata(dev);
	ktime_t start_time;

	cancel_delayed_work(&di->battery_monitor_work);

	printk(KERN_DEBUG "%s\n",__func__);
	printk(KERN_DEBUG "CABLE_TYPE :   %d\n", sec_bci.charger.cable_status);

	if (sec_bci.charger.cable_status == POWER_SUPPLY_TYPE_MAINS) {
		start_time = alarm_get_elapsed_realtime();
		battery_program_alarm(di, start_time);
	}

	return 0;
}

static void battery_resume_alarm(struct device *dev)
{
	struct battery_device_info *di = dev_get_drvdata(dev);

	printk(KERN_DEBUG "battery_resume_alarm\n");

	get_system_temperature(TEMP_DEG);

	wake_lock_timeout(&sec_bc_wakelock, HZ * 2);

	power_supply_changed(&di->sec_battery);
	power_supply_changed(&di->sec_ac);
	power_supply_changed(&di->sec_usb);

	queue_delayed_work(sec_bci.sec_battery_workq,
		&di->battery_monitor_work, HZ * 1);
}


static const struct dev_pm_ops charger_pm_ops = {
	.prepare = battery_suspend_alarm,
	.complete = battery_resume_alarm,
};

struct platform_driver battery_platform_driver = {
	.probe = battery_probe,
	.remove = __devexit_p(battery_remove),
	.driver = {
		.name = DRIVER_NAME,
		.pm = &charger_pm_ops,
	},
};

static int __init battery_init(void)
{
	int ret;

	pr_alert("\n[BM] Battery Init\n");

	sec_bci.ready = false;

	sec_bci.battery.battery_health = POWER_SUPPLY_HEALTH_GOOD;
	sec_bci.battery.battery_technology = POWER_SUPPLY_TECHNOLOGY_LION;
	sec_bci.battery.battery_level_ptg = 0;
	sec_bci.battery.battery_level_vol = 0;
	sec_bci.battery.monitor_duration = MONITOR_DEFAULT_DURATION;
	sec_bci.battery.monitor_duration_sleep = MONITOR_DURATION_DUR_SLEEP;
	sec_bci.battery.monitor_field_temp = false;
	sec_bci.battery.monitor_field_rechg_vol = false;
	sec_bci.battery.confirm_full_by_current = 0;
	sec_bci.battery.support_monitor_temp = 1;
	sec_bci.battery.support_monitor_timeout = 1;
	sec_bci.battery.support_monitor_full = 1;
	sec_bci.battery.confirm_recharge = 0;

	sec_bci.charger.prev_cable_status = -1;
	sec_bci.charger.cable_status = -1;
	sec_bci.charger.prev_charge_status = 0;
	sec_bci.charger.charge_status = 0;
	sec_bci.charger.is_charging = false;
	sec_bci.charger.charge_start_time = 0;
	sec_bci.charger.charged_time = 0;
	sec_bci.charger.charging_timeout = DEFAULT_CHARGING_TIMEOUT;
	sec_bci.charger.camera_recording = 0;
	sec_bci.charger.camera_recording_inout = 0;
	sec_bci.charger.use_ta_nconnected_irq = false;
	sec_bci.charger.rechg_count = 0;
	sec_bci.sec_battery_workq =
		create_singlethread_workqueue("sec_battery_workq");

	wake_lock_init(&sec_bc_wakelock, WAKE_LOCK_SUSPEND, "samsung-battery");
	wake_lock_init(&lowbat_wakelock, WAKE_LOCK_SUSPEND, "samsung-battery-lowbat");

	/* Get the charger driver */
	ret = charger_init();
	if (ret < 0) {
		printk(KERN_DEBUG "[BM] Fail to get charger driver.\n");
		return ret;
	}

	/* Get the fuelgauge driver */
	ret = fuelgauge_init();
	if (ret < 0) {
		printk(KERN_DEBUG "[BM] Fail to get fuelgauge driver.\n");
		return ret;
	}

	ret = platform_driver_register(&battery_platform_driver);

	return ret;
}

module_init(battery_init);

static void __exit battery_exit(void)
{
	/*Remove the charger driver */
	charger_exit();
	/*Remove the fuelgauge driver */
	fuelgauge_exit();

	platform_driver_unregister(&battery_platform_driver);
	pr_alert("[BM] Battery Driver Exit\n");
}

module_exit(battery_exit);

MODULE_AUTHOR("EUNGON KIM <egstyle.kim@samsung.com>");
MODULE_DESCRIPTION("Samsung Battery monitor driver");
MODULE_LICENSE("GPL");
