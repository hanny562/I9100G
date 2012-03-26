#ifndef _COMMON_H_
#define _COMMON_H_

//#define WR_ADC

#ifdef WR_ADC
/* Workaround to get proper adc value */
#include <linux/i2c/twl.h>
#endif



// Unit is sec.
#define MONITOR_DURATION_DUR_SLEEP	60
#define MONITOR_DEFAULT_DURATION	30
#define MONITOR_TEMP_DURATION		30
#define MONITOR_RECHG_VOL_DURATION	30


#define DEFAULT_CHARGING_TIMEOUT	6 * 60 * 60
#define DEFAULT_RECHARGING_TIMEOUT	(1 * 60 * 60 + 30 * 60)

#define CHARGE_STOP_TEMPERATURE_MAX	65
#define CHARGE_RECOVER_TEMPERATURE_MAX	43

#define CHARGE_STOP_TEMPERATURE_MIN	-3
#define CHARGE_RECOVER_TEMPERATURE_MIN	0

#define HIGH_TEMP_GAURD_FOR_CAMCORDER

/* This temp is concerned with HOT SPOT temp */
/* When the HOT SPOT is measured to 43C, */
/* The temp which is measured by thermisotr is 39C */
/* Threshold ADC value should be 194 */
#define HIGH_TEMP_GAURD_FOR_CAMCORDER_THRSHOLD	39

#define CHARGE_RECHG_VOLTAGE		4130
#define CHARGE_RECHG_VOLTAGE_OFFMODE	4130


#define CHARGE_STOP_TEMPERATURE_EVENT		67	// 63
#define CHARGE_RECOVER_TEMPERATURE_EVENT	60	// 58

#define CHARGE_FULL_CURRENT_DISPLAY_ADC 388
#define CHARGE_FULL_CURRENT_END_ADC	188
#define CHARGE_FULL_VOLTAGE		4150

#define CHARGE_DUR_ACTIVE 0
#define CHARGE_DUR_SLEEP  1

#define STATUS_CATEGORY_CABLE       1
#define STATUS_CATEGORY_CHARGING    2
#define STATUS_CATEGORY_TEMP        3
#define STATUS_CATEGORY_ETC         4

#define BATTERY_LOW_PERCENTAGE		15
#define BATTERY_CRITICAL_LOW_PERCENTAGE	4
#define BATTERY_POWEROFF_PERCENTAGE	1
#define BATTERY_POWEROFF_VOLTAGE	3300

enum {
/* power_supply.h
   POWER_SUPPLY_STATUS_UNKNOWN = 0,
   POWER_SUPPLY_STATUS_CHARGING,
   POWER_SUPPLY_STATUS_DISCHARGING,
   POWER_SUPPLY_STATUS_NOT_CHARGING,
   POWER_SUPPLY_STATUS_FULL,
*/
	POWER_SUPPLY_STATUS_CHARGING_OVERTIME = 5,
	POWER_SUPPLY_STATUS_RECHARGING_FOR_FULL = 6,
	POWER_SUPPLY_STATUS_RECHARGING_FOR_TEMP = 7,
	POWER_SUPPLY_STATUS_FULL_DUR_SLEEP = 8,
	POWER_SUPPLY_STATUS_FULL_END = 9,
};

enum {
	BATTERY_TEMPERATURE_NORMAL = 0,
	BATTERY_TEMPERATURE_LOW,
	BATTERY_TEMPERATURE_HIGH,
	BATTERY_TEMPERATURE_ETC,
};

enum {
	ETC_CABLE_IS_DISCONNECTED = 0,
};

enum {
	RAW_SOC = 0,
	ADJUSTED_SOC,
};

typedef struct {
	bool ready;

	/*Battery info */
	struct _battery {
		int battery_technology;
		int battery_level_ptg;
		int battery_level_vol;
		int battery_temp;
		int battery_health;
		bool battery_vf_ok;
		bool battery_vol_toolow;

		int monitor_duration;
		int monitor_duration_sleep;
		
		bool monitor_field_temp;
		bool monitor_field_rechg_vol;
		int support_monitor_temp;
		int support_monitor_timeout;
		int support_monitor_full;

		int confirm_full_by_current;
		int confirm_recharge;
	} battery;

	/*Charger info */
	struct _charger {
		int prev_cable_status;
		int cable_status;

		int prev_charge_status;
		int charge_status;

		bool is_charging;

		unsigned long long charge_start_time;
		unsigned long charged_time;
		int charging_timeout;
		int camera_recording;
		int camera_recording_inout;

		bool use_ta_nconnected_irq;


		// for adjust fuelgauge
		int fuelgauge_full_soc;
		int rechg_count;
	} charger;

	struct workqueue_struct *sec_battery_workq;

} SEC_battery_charger_info;

#endif


