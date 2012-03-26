/*
 * input/touchscreen/omap/omap_ts.c
 *
 * touchscreen input device driver for various TI OMAP boards
 * Copyright (c) 2002 MontaVista Software Inc.
 * Copyright (c) 2004 Texas Instruments, Inc.
 * Cleanup and modularization 2004 by Dirk Behme <dirk.behme@de.bosch.com>
 *
 * Assembled using driver code copyright the companies above.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 * History:
 * 12/12/2004    Srinath Modified and intergrated code for H2 and H3
 *
 */
#define RESERVED_T0				0u
#define RESERVED_T1				1u
#define DEBUG_DELTAS_T2				2u
#define DEBUG_REFERENCES_T3			3u
#define DEBUG_SIGNALS_T4			4u
#define GEN_MESSAGEPROCESSOR_T5			5u
#define GEN_COMMANDPROCESSOR_T6			6u
#define GEN_POWERCONFIG_T7			7u
#define GEN_ACQUISITIONCONFIG_T8		8u
#define TOUCH_MULTITOUCHSCREEN_T9		9u
#define TOUCH_SINGLETOUCHSCREEN_T10		10u
#define TOUCH_XSLIDER_T11			11u
#define TOUCH_YSLIDER_T12			12u
#define TOUCH_XWHEEL_T13			13u
#define TOUCH_YWHEEL_T14			14u
#define TOUCH_KEYARRAY_T15			15u
#define PROCG_SIGNALFILTER_T16			16u
#define PROCI_LINEARIZATIONTABLE_T17		17u
#define SPT_COMCONFIG_T18			18u
#define SPT_GPIOPWM_T19				19u
#define PROCI_GRIPFACESUPPRESSION_T20		20u
#define RESERVED_T21				21u
#define PROCG_NOISESUPPRESSION_T22		22u
#define TOUCH_PROXIMITY_T23			23u
#define PROCI_ONETOUCHGESTUREPROCESSOR_T24	24u
#define SPT_SELFTEST_T25			25u
#define DEBUG_CTERANGE_T26			26u
#define PROCI_TWOTOUCHGESTUREPROCESSOR_T27	27u
#define SPT_CTECONFIG_T28			28u
#define SPT_GPI_T29				29u
#define SPT_GATE_T30				30u
#define TOUCH_KEYSET_T31			31u
#define TOUCH_XSLIDERSET_T32			32u

#ifdef CONFIG_MACH_OMAP4_TAB_10_1
#if (CONFIG_SAMSUNG_OMAP4_TAB_REV == 4)
#define  OMAP_GPIO_TSP_INT		43
#elif ((CONFIG_SAMSUNG_OMAP4_TAB_REV == 5) \
	 || (CONFIG_SAMSUNG_OMAP4_TAB_REV == 6) \
	 || (CONFIG_SAMSUNG_OMAP4_TAB_REV == 7))
#define  OMAP_GPIO_TSP_INT              46
#endif
#endif

#if defined(CONFIG_MACH_OMAP4_SAMSUNG)
#undef OMAP_GPIO_TSP_INT

#define  OMAP_GPIO_TSP_INT              46
#endif
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/wait.h>
#include <linux/interrupt.h>
#include <linux/suspend.h>
#include <linux/platform_device.h>
#include <linux/semaphore.h>
#include <asm/mach-types.h>
#include <plat/gpio.h>
#include <plat/mux.h>
#include <linux/delay.h>
#include <asm/irq.h>
#include <asm/mach/irq.h>
#include <linux/firmware.h>
#include <linux/time.h>
#include <linux/i2c/twl.h>
#include <linux/earlysuspend.h>
#include <plat/omap-pm.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/wakelock.h>

#include "atmel_touch.h"

extern unsigned char g_version;
extern uint8_t cal_check_flag;

extern void disable_irq_nosync(unsigned int);
extern void enable_irq(unsigned int);

static ssize_t ts_show(struct kobject *, struct kobj_attribute *, char *);
static ssize_t ts_store(struct kobject *k, struct kobj_attribute *,  const char *buf, size_t n);
static ssize_t firmware_show(struct kobject *, struct kobj_attribute *, char *);
static ssize_t firmware_store(struct kobject *k, struct kobj_attribute *attr,  const char *buf, size_t n);

#define TOUCHSCREEN_NAME		"sec_touchscreen"

#define DEFAULT_PRESSURE_UP		0
#define DEFAULT_PRESSURE_DOWN		256

#define TOUCH_MENU		KEY_MENU
#define TOUCH_SEARCH		KEY_SEARCH
#define TOUCH_HOME		KEY_HOME
#define TOUCH_BACK		KEY_BACK

#define MAX_TOUCH_X_RESOLUTION		480
#define MAX_TOUCH_Y_RESOLUTION		800

unsigned short enable_touch_boost;
static struct touchscreen_t tsp;
static struct workqueue_struct *tsp_wq;
struct	semaphore	sem_touch_onoff;
struct	semaphore	sem_touch_handler;
static int g_enable_touchscreen_handler = 0;

int atmel_ts_tk_keycode[] = { TOUCH_MENU, TOUCH_BACK };

struct touchscreen_t;

struct touchscreen_t {
	struct input_dev * inputdevice;
	struct timer_list ts_timer;
	int touched;
	int irq;
	int irq_type;
	int irq_enabled;
	struct ts_device *dev;
	spinlock_t lock;
	struct early_suspend	early_suspend;
	struct work_struct  tsp_work;
	struct timer_list opp_set_timer;
	struct work_struct constraint_wq;
	int opp_high;
};

#ifdef CONFIG_HAS_EARLYSUSPEND
void atmel_ts_early_suspend(struct early_suspend *h);
void atmel_ts_late_resume(struct early_suspend *h);
#endif	/* CONFIG_HAS_EARLYSUSPEND */

#ifdef FEATURE_MOVE_LIMIT
int pre_x, pre_y, pre_size;
#endif

int touch_proc_ioctl(struct inode *, struct file *, unsigned int, unsigned long);
int touch_proc_read(char *page, char **start, off_t off,int count, int *eof, void *data);
int touch_proc_write(struct file *file, const char __user *buffer, unsigned long count, void *data);

#define IOCTL_TOUCH_PROC 'T'
enum {
	TOUCH_GET_VERSION = _IOR(IOCTL_TOUCH_PROC, 0, char*),
	TOUCH_GET_T_KEY_STATE = _IOR(IOCTL_TOUCH_PROC, 1, int),
	TOUCH_GET_SW_VERSION = _IOR(IOCTL_TOUCH_PROC, 2, char*),
};

const char fw_version[10]="0X16";
struct proc_dir_entry *touch_proc;
struct file_operations touch_proc_fops = {
	.ioctl=touch_proc_ioctl,
};



#ifdef FEATURE_2_MORE_MULTI_TOUCH
void handle_2_more_touch(uint8_t *atmel_msg);

typedef struct {
	uint8_t id;		/*!< id */
	int8_t status;		/*!< dn=1, up=0, none=-1 */
	int16_t x;		/*!< X */
	int16_t y;		/*!< Y */
	unsigned int size ;	/*!< size */
} report_finger_info_t;

#define MAX_NUM_FINGER	10

static report_finger_info_t fingerInfo[MAX_NUM_FINGER];
#endif

struct wake_lock tsp_firmware_wake_lock;

typedef struct {
	int x;
	int y;
	int press;
} dec_input;

static dec_input id2 = {0};
static dec_input id3 = {0};
static dec_input tmp2 = {0};
static dec_input tmp3 = {0};

#define REPORT( touch, width, x, y) {	\
		input_report_abs(tsp.inputdevice, ABS_MT_TOUCH_MAJOR, touch ); \
		input_report_abs(tsp.inputdevice, ABS_MT_WIDTH_MAJOR, width ); \
		input_report_abs(tsp.inputdevice, ABS_MT_POSITION_X, x); \
		input_report_abs(tsp.inputdevice, ABS_MT_POSITION_Y, y); \
		input_mt_sync(tsp.inputdevice);	\
	}


void read_func_for_only_single_touch(struct work_struct *work);
void read_func_for_multi_touch(struct work_struct *work);
void handle_multi_touch(uint8_t *atmel_msg);
void handle_keyarray_of_archer(uint8_t * atmel_msg);
void initialize_multi_touch(void);


static irqreturn_t touchscreen_handler(int irq, void *dev_id);
void set_touch_irq_gpio_init(void);
void set_touch_irq_gpio_disable(void);

static struct kobj_attribute touch_boost_attr =
    __ATTR(touch_boost, 0644, ts_show, ts_store);
static struct kobj_attribute firmware_attr =
    __ATTR(firmware, 0666, firmware_show, firmware_store);

/*------------------------------ for tunning ATmel - start ----------------------------*/
extern  ssize_t set_power_show(struct device *dev, struct device_attribute *attr, char *buf);
extern  ssize_t set_power_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
extern  ssize_t set_acquisition_show(struct device *dev, struct device_attribute *attr, char *buf);
extern  ssize_t set_acquisition_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
extern  ssize_t set_touchscreen_show(struct device *dev, struct device_attribute *attr, char *buf);
extern  ssize_t set_touchscreen_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
extern  ssize_t set_keyarray_show(struct device *dev, struct device_attribute *attr, char *buf);
extern  ssize_t set_keyarray_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
extern  ssize_t set_grip_show(struct device *dev, struct device_attribute *attr, char *buf);
extern  ssize_t set_grip_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
extern  ssize_t set_noise_show(struct device *dev, struct device_attribute *attr, char *buf);
extern  ssize_t set_noise_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
extern  ssize_t set_total_show(struct device *dev, struct device_attribute *attr, char *buf);
extern  ssize_t set_total_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
extern  ssize_t set_write_show(struct device *dev, struct device_attribute *attr, char *buf);
extern  ssize_t set_write_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);

extern ssize_t set_tsp_for_extended_indicator_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t set_tsp_for_extended_indicator_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
extern ssize_t set_tsp_for_inputmethod_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t set_tsp_for_inputmethod_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);

#ifdef FEATURE_TSP_FOR_TA
extern ssize_t set_tsp_for_TA_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t set_tsp_for_TA_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
#endif

#ifdef FEATURE_DYNAMIC_SLEEP
extern ssize_t set_sleep_way_of_TSP_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t set_sleep_way_of_TSP_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
#endif

extern ssize_t set_TSP_debug_on_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t set_TSP_debug_on_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
extern ssize_t set_test_value_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t set_test_value_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
ssize_t TSP_filter_status_show(struct device *dev, struct device_attribute *attr, char *buf);

static DEVICE_ATTR(set_power, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, set_power_show, set_power_store);
static DEVICE_ATTR(set_acquisition, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, set_acquisition_show, set_acquisition_store);
static DEVICE_ATTR(set_touchscreen, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, set_touchscreen_show, set_touchscreen_store);
static DEVICE_ATTR(set_keyarray, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, set_keyarray_show, set_keyarray_store);
static DEVICE_ATTR(set_grip , S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, set_grip_show, set_grip_store);
static DEVICE_ATTR(set_noise, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, set_noise_show, set_noise_store);
static DEVICE_ATTR(set_total, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, set_total_show, set_total_store);
static DEVICE_ATTR(set_write, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, set_write_show, set_write_store);
static DEVICE_ATTR(set_tsp_for_extended_indicator, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, set_tsp_for_extended_indicator_show, set_tsp_for_extended_indicator_store);
static DEVICE_ATTR(set_tsp_for_inputmethod, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, set_tsp_for_inputmethod_show, set_tsp_for_inputmethod_store);
#ifdef FEATURE_TSP_FOR_TA
static DEVICE_ATTR(set_tsp_for_TA, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, set_tsp_for_TA_show, set_tsp_for_TA_store);
#endif
#ifdef FEATURE_DYNAMIC_SLEEP
static DEVICE_ATTR(set_sleep_way_of_TSP, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, set_sleep_way_of_TSP_show, set_sleep_way_of_TSP_store);
#endif
static DEVICE_ATTR(set_TSP_debug_on, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, set_TSP_debug_on_show, set_TSP_debug_on_store);
static DEVICE_ATTR(set_test_value, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, set_test_value_show, set_test_value_store);
static DEVICE_ATTR(TSP_filter_status, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, TSP_filter_status_show, NULL);
/*------------------------------ for tunning ATmel - end ----------------------------*/

extern void restore_acquisition_config(void);
extern void restore_power_config(void);
extern uint8_t calibrate_chip(void);

#ifdef FEATURE_CALIBRATION_BY_KEY
extern unsigned char first_key_after_probe_or_wake_up;
#endif

static unsigned char menu_button = 0;
static unsigned char back_button = 0;


#ifdef FEATURE_VERIFY_INCORRECT_PRESS
static unsigned char figure_x_count(int id, int x);
static void init_acc_count(int id);
int incorrect_repeat_count=4;
#endif

unsigned char debug_print_on = 0;
static bool en_touch_log = 1;

extern uint8_t report_id_to_type(uint8_t report_id, uint8_t *instance);


static ssize_t ts_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf) {
	if (attr == &touch_boost_attr)
		return sprintf(buf, "%hu\n", enable_touch_boost);
	else
		return -EINVAL;
}

static ssize_t ts_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t n) {
	unsigned short value;

	if (sscanf(buf, "%hu", &value) != 1) {
		perr("ts_store: Invalid value\n");
		return -EINVAL;
	}

	if (attr == &touch_boost_attr) {
		enable_touch_boost = value;
	} else {
		return -EINVAL;
	}
	return n;
}

static void tsc_timer_out (unsigned long v) {
	schedule_work(&(tsp.constraint_wq));
	return;
}

void tsc_remove_constraint_handler(struct work_struct *work) {
	tsp.opp_high = 0;
}

void set_touch_irq_gpio_init(void) {
	if (gpio_request(OMAP_GPIO_TSP_INT, "atmel_touch") < 0) {
		perr("can't get atmel pen down GPIO\n");
		return;
	}
	gpio_direction_input(OMAP_GPIO_TSP_INT);
}

void set_touch_irq_gpio_disable(void) {
	if (g_enable_touchscreen_handler == 1) {
		free_irq(tsp.irq, &tsp);
		gpio_free(OMAP_GPIO_TSP_INT);
		g_enable_touchscreen_handler = 0;
	}
	return;
}

unsigned char get_touch_irq_gpio_value(void) {
	return gpio_get_value(OMAP_GPIO_TSP_INT);
}

static int touchscreen_poweronoff(int power_state) {
	int ret  = 0;
	return ret;
}

void atmel_touchscreen_poweron(void) {
	if (touchscreen_poweronoff(1) < 0)
		perr("power on error!!!\n");
	return;
}

#define U8	__u8
#define U16 	unsigned short int
#define READ_MEM_OK                 1u

extern U8 read_mem(U16 start, U8 size, U8 *mem);
extern uint16_t message_processor_address;
extern uint8_t max_message_length;
extern uint8_t *atmel_msg;
extern uint8_t QT_Boot(uint8_t qt_force_update);
extern unsigned long simple_strtoul(const char *,char **,unsigned int);
extern unsigned char g_version, g_build, qt60224_notfound_flag;

extern void check_chip_calibration(unsigned char one_touch_input_flag);


void disable_tsp_irq(void) {
	disable_irq_nosync(tsp.irq);
}

void enable_tsp_irq(void) {
	enable_irq(tsp.irq);
}

static ssize_t firmware_show(struct kobject *k, struct kobj_attribute *attr, char *buf) {
	dbg("QT602240 Firmware Ver %x, Build = %x.\n", g_version, g_build);
	return sprintf(buf, "0X%x", g_version);
}

static ssize_t firmware_store(struct kobject *k, struct kobj_attribute *attr,  const char *buf, size_t size) {
	char *after;
	int firmware_ret_val = -1;
	unsigned long value = simple_strtoul(buf, &after, 10);

	if (value == 1) { // auto update.
		printk("[TSP] Firmware update start!!\n" );
		printk("[TSP] version = 0x%x\n", g_version );

		if ( ((g_version != 0x14) && (g_version < 0x16))
		        || ((g_version == 0x16) && (g_build == 0xaa)) )	{
			wake_lock(&tsp_firmware_wake_lock);
			firmware_ret_val = QT_Boot(qt60224_notfound_flag);
			if (firmware_ret_val == 0)
				qt60224_notfound_flag = 1;
			else if (firmware_ret_val == 1)
				qt60224_notfound_flag = 0;
			wake_unlock(&tsp_firmware_wake_lock);
		} else {
			firmware_ret_val = 1;
		}
		printk("[TSP] Firmware result = %d\n", firmware_ret_val);
	}
	return size;
}


void initialize_multi_touch(void) {
	id2.x = 0;
	id2.y = 0;
	id2.press = 0;

	id3.x = 0;
	id3.y = 0;
	id3.press = 0;

	tmp2.x = 0;
	tmp2.y = 0;
	tmp2.press = 0;

	tmp3.x = 0;
	tmp3.y = 0;
	tmp3.press = 0;
}

void handle_keyarray_of_archer(uint8_t * atmel_msg)
{
	printk("If we see this print then there is something wrong in our configuration\n");

	if ( (atmel_msg[2] & 0x1) && (menu_button==0) ) {
		menu_button = 1;
		dbg("menu_button is pressed\n");
		input_report_key(tsp.inputdevice, 139, DEFAULT_PRESSURE_DOWN);
		input_sync(tsp.inputdevice);
	} else if ( (atmel_msg[2] & 0x2) && (back_button==0) ) {
		back_button = 1;
		dbg("back_button is pressed\n");
		input_report_key(tsp.inputdevice, 158, DEFAULT_PRESSURE_DOWN);
		input_sync(tsp.inputdevice);
	} else if ( (~atmel_msg[2] & (0x1)) && menu_button==1 ) {
#ifdef FEATURE_CALIBRATION_BY_KEY
		if ( first_key_after_probe_or_wake_up ) {
			dbg("calibratiton by first key after waking-up or probing\n");
			first_key_after_probe_or_wake_up = 0;
			calibrate_chip();
		}
#endif
		menu_button = 0;
		dbg("menu_button is released\n");
		input_report_key(tsp.inputdevice, 139, DEFAULT_PRESSURE_UP);
		input_sync(tsp.inputdevice);
	} else if ( (~atmel_msg[2] & (0x2)) && back_button==1 ) {
#ifdef FEATURE_CALIBRATION_BY_KEY
		if ( first_key_after_probe_or_wake_up ) {
			dbg("calibratiton by first key after waking-up or probing\n");
			first_key_after_probe_or_wake_up = 0;
			calibrate_chip();
		}
#endif
		back_button = 0;
		dbg("back_button is released\n");
		input_report_key(tsp.inputdevice, 158, DEFAULT_PRESSURE_UP);
		input_sync(tsp.inputdevice);
	} else {
		menu_button=0;
		back_button=0;
		dbg("Unknow state of touch key\n");
	}
}


#ifdef FEATURE_MOVE_LIMIT
#define MOVE_LIMIT_SQUARE (150*150) // 100*100
#define DISTANCE_SQUARE(X1, Y1, X2, Y2)    (((X2-X1)*(X2-X1))+((Y2-Y1)*(Y2-Y1)))

int pre_x, pre_y, pre_size;

#endif

#ifdef FEATURE_VERIFY_INCORRECT_PRESS
static u16 new_detect_id=0;
static u16 new_detect_x=0;

static u16 fixed_id2_x = 0;
static u16 fixed_id3_x = 0;

static u16 fixed_id2_count = 0;
static u16 fixed_id3_count = 0;
#endif

void handle_multi_touch(uint8_t *atmel_msg)
{
	u16 x = 0, y = 0;
	unsigned int size;
	static int check_flag = 0;
	uint8_t touch_message_flag = 0;
	unsigned char one_touch_input_flag = 0;

	x = atmel_msg[2];
	x = x << 2;
	x = x | (atmel_msg[4] >> 6);

	y = atmel_msg[3];
	y = y << 2;
	y = y | ((atmel_msg[4] & 0x6)  >> 2);

	size = atmel_msg[5];

	/* for ID=2 & 3, these are valid inputs. */
	if ((atmel_msg[0]==2)||(atmel_msg[0]==3)) {
		if ( atmel_msg[1] & 0x60) {
			dbg("msg0:0x%x\n", atmel_msg[1]);
		}

		/* for Single Touch input */
		if ((id2.press+id3.press)!= 2) {
			/* case.1 - 11000000 -> DETECT & PRESS */
			if ( ( atmel_msg[1] & 0xC0 ) == 0xC0  ) {
				touch_message_flag = 1;

				if (atmel_msg[0] % 2)
					id3.press = 1;   // for ID=3
				else
					id2.press = 1;	 // for ID=2

				if ( (id2.press + id3.press) == 2 ) {
					if ( atmel_msg[0] % 2) {
						REPORT( 1, size, tmp2.x, tmp2.y );  // for ID=3
					} else {
						REPORT( 1, size, tmp3.x, tmp3.y );  // for ID=2
					}
					REPORT( 1, size, x, y );
#ifdef FEATURE_VERIFY_INCORRECT_PRESS
					new_detect_id = atmel_msg[0];
					new_detect_x = x;
#endif
					dbg("Detect one with new detect id=%d, x=%d, y=%d\n", atmel_msg[0] , x, y );
					if (atmel_msg[0] % 2) {
						id2.x = tmp2.x;
						id2.y = tmp2.y;
						id3.x = x;
						id3.y = y;
					} else {
						id2.x = x;
						id2.y = y;
						id3.x = tmp3.x;
						id3.y = tmp3.y;
					}
				} else {
					one_touch_input_flag = 1;
					REPORT( 1, size, x, y );
					dbg("one detect - id=%d, x=%d, y=%d\n", atmel_msg[0] , x, y);
				}
			}

			if ( ( atmel_msg[1] & 0x90 ) == 0x90 ) {
				dbg("One detect - id=%d, x=%d, y=%d\n", atmel_msg[0] , x, y);
				REPORT( 1, size , x, y);
			}

			/* case.3 - case 00100000 -> RELEASE */
			else if ( ((atmel_msg[1] & 0x20 ) == 0x20)) {
				REPORT( 0, 1, x, y );
				dbg("id = %d, x = %d, y = %d - release\n", atmel_msg[0] , x, y);

				if (atmel_msg[0] % 2)
					id3.press = 0; 	// for ID=3
				else
					id2.press = 0;	// for ID=2

#ifdef FEATURE_VERIFY_INCORRECT_PRESS
				init_acc_count(atmel_msg[0]);
#endif
			}

			input_sync(tsp.inputdevice);

			if (atmel_msg[0] % 2) {
				tmp3.x = x;
				tmp3.y = y;
			} else {
				tmp2.x = x;
				tmp2.y = y;
			}
		}

		/* for Two Touch input */
		else if ( id2.press && id3.press ) {
			if ( atmel_msg[0] % 2 ) {
				id3.x = x;
				id3.y = y;
			} else {
				id2.x = x;
				id2.y = y;
			}

			if ( ((atmel_msg[1] & 0x20 ) == 0x20)) {
				if (atmel_msg[0] % 2)
					id3.press = 0;
				else
					id2.press = 0;

				dbg("Two input & one release\t id=2, x = %d, y = %d, id = 3, x = %d, y = %d release id = %d\n",
						id2.x, id2.y ,id3.x, id3.y,  atmel_msg[0] );
#ifdef FEATURE_VERIFY_INCORRECT_PRESS
				{
					int count_id, count_x;

					// when a ID release event is reported
					// another ID is checked if it is fixed by noise.
					if (atmel_msg[0] % 2) {
						count_id = 2;
						count_x = id2.x;
					} else {
						count_id = 3;
						count_x = id3.x;
					}

					if ( new_detect_id == atmel_msg[0] && new_detect_x != x) {
						if ( figure_x_count(count_id, count_x) ) {
							extern uint8_t calibrate_chip(void);
							calibrate_chip();
							initialize_multi_touch();
						}
					} else {
						init_acc_count(count_id);
					}
					init_acc_count(atmel_msg[0]);
				}
#endif
			}

			if ( id2.y && id3.y ) {
				REPORT( id2.press, size, id2.x, id2.y );
				REPORT( id3.press, size, id3.x, id3.y );
				input_sync(tsp.inputdevice);
				id2.y = 0;  // clear
				id3.y = 0;
			} else if ((atmel_msg[0]==3) // report one real value and one dummy value
			           || (atmel_msg[0]==2 && check_flag) // id2 is reported twice
			           || ((atmel_msg[1] & 0x20) == 0x20 )) { // any one is released
				if ( id3.y == 0 ) {
					REPORT( id2.press , size , id2.x, id2.y);
					REPORT( id3.press , size , tmp3.x, tmp3.y);
					input_sync(tsp.inputdevice);
					id2.y = 0;  // clear
				} else if ( id2.y == 0 ) {
					REPORT( id2.press , size , tmp2.x, tmp2.y);
					REPORT( id3.press , size , id3.x, id3.y);
					input_sync(tsp.inputdevice);
					id3.y = 0; // clear
				}
			} else if ( atmel_msg[0] == 2 ) { // for ID=2
				check_flag=1;
			}

			if ( atmel_msg[0] % 2 ) {
				tmp3.x = x;
				tmp3.y = y;
				check_flag=0;
			} else {
				tmp2.x = x;
				tmp2.y = y;
			}
		}
	}
	/* case.4 - Palm Touch & Unknow sate */
	else if ( atmel_msg[0] == 14 ) {
		if ((atmel_msg[1]&0x01) == 0x00) {
			dbg("Palm Touch! - %d <released from palm touch>\n", atmel_msg[1]);
			id2.press = 0;
			id3.press = 0;
		} else {
			dbg("Test Palm Touch! - %d <face suppresstion status bit>\n", atmel_msg[1]);
			touch_message_flag = 1;
		}
	} else if ( atmel_msg[0] == 1 ) {
		dbg("msg[0] = %d, msg[1] = 0x%x\n", atmel_msg[0], atmel_msg[1]);
	} else if ( atmel_msg[0] == 0 ) {
		dbg("Error : What happen to TSP chip?\n");
	}

	if (touch_message_flag && (cal_check_flag)) {
		check_chip_calibration(one_touch_input_flag);
	}
}


#ifdef FEATURE_2_MORE_MULTI_TOUCH
void handle_2_more_touch(uint8_t *atmel_msg)
{
	int i;
	uint8_t id;
	unsigned long x, y;
	int btn_report = 0;
	int touchCount = 0;
	int bChangeUpDn = 0;
	unsigned int size = 0;
	unsigned int press = 3;
	static int nPrevID = -1;
	uint8_t touch_message_flag = 0;

	size = atmel_msg[5];
	id   = atmel_msg[0] - 2;

	x = atmel_msg[2];
	x = x << 2;
	x = x | atmel_msg[4] >> 6;

	y = atmel_msg[3];
	y = y << 2;
	y = y | ((atmel_msg[4] & 0x6)  >> 2);
	/* for ID=2 ~ 11, these are valid inputs. */
	if (atmel_msg[0] >=2 && atmel_msg[0]<=11) {

		if ( (atmel_msg[1] & 0x80) && (atmel_msg[1] & 0x40) ) {
			/* Detect & Press */
			fingerInfo[id].id= id;
			fingerInfo[id].status= 1;
			fingerInfo[id].x= (int16_t)x;
			fingerInfo[id].y= (int16_t)y;
			fingerInfo[id].size= (unsigned int)size;
			bChangeUpDn= 1;
			touch_message_flag = 1;
			dbg("Finger[%d] Down (%d,%d)!\n", id, fingerInfo[id].x, fingerInfo[id].y);
		} else if ( (atmel_msg[1] & 0x80) && (atmel_msg[1] & 0x10) ) {
			/* Detect & Move */
			fingerInfo[id].x= (int16_t)x;
			fingerInfo[id].y= (int16_t)y;
			fingerInfo[id].size= (unsigned int)size;
		} else if ( (atmel_msg[1] & 0x20 ) ) {
			/* Release */
			fingerInfo[id].status= 0;
			bChangeUpDn= 1;
			dbg("Finger[%d] Up (%d,%d)!\n", id, fingerInfo[id].x, fingerInfo[id].y);
			for (i = 0; i < MAX_NUM_FINGER; ++i ) {
				if (fingerInfo[id].status == 1)
					touchCount++;
				else
					break;
			}
		} else {
			press = 3;
			perr("Unknown state!\n");
		}
	}
	/* case. - Palm Touch & Unknow sate */
	else if ( atmel_msg[0] == 14 ) {
		if ((atmel_msg[1]&0x01) == 0x00) {
			dbg("Palm Touch! - %d <released from palm touch>\n", atmel_msg[1]);
			// touch history clear
			for ( i= 0; i<MAX_NUM_FINGER; ++i ) {
				fingerInfo[i].status = -1;
			}

		} else {
			dbg("Test Palm Touch! - %d <face suppresstion status bit>\n", atmel_msg[1] );
			touch_message_flag = 1;
		}
	} else if ( atmel_msg[0] == 1 ) {
		dbg("msg[0] = %d, msg[1] = %d\n", atmel_msg[0], atmel_msg[1]);
	} else if ( atmel_msg[0] == 0 ) {
		dbg("Error : %d - What happen to TSP chip?\n", __LINE__ );
	}

	if (touch_message_flag && cal_check_flag) {
		check_chip_calibration();
	}

	if ( nPrevID >= id || bChangeUpDn ) {
		for ( i= 0; i<MAX_NUM_FINGER; ++i ) {
			if ( fingerInfo[i].status == -1 )
				continue;

			REPORT(fingerInfo[i].status, fingerInfo[id].size, fingerInfo[i].x, fingerInfo[i].y);
			if ( fingerInfo[i].status == 0 )
				fingerInfo[i].status= -1;
		}
		input_sync(tsp.inputdevice);
	}
	nPrevID= id;
}
#endif

void read_func_for_only_single_touch(struct work_struct *work) {
	u16 x=0, y=0;
	u16 x480, y800, press;
	struct touchscreen_t *ts = container_of(work, struct touchscreen_t, tsp_work);

	if (enable_touch_boost) {
		if (timer_pending(&ts->opp_set_timer))
			del_timer(&ts->opp_set_timer);
		mod_timer(&ts->opp_set_timer, jiffies + (1000 * HZ) / 1000);
	}

	if (read_mem(message_processor_address, max_message_length, atmel_msg) == READ_MEM_OK) {

		if ((atmel_msg[0] < 2) || (atmel_msg[0] > 11)) {
			perr("Read failed\n");
			enable_irq(tsp.irq);
			return;
		}
		x = atmel_msg[2];
		x = x << 2;
		x = x | (atmel_msg[4] >> 6);

		y = atmel_msg[3];
		y = y << 2;
		y = y | ((atmel_msg[4] & 0x6)  >> 2);

		x480 = x;
		y800 = y;

		if ( ((atmel_msg[1] & 0x40) == 0x40  || (atmel_msg[1] & 0x10) == 0x10) && (atmel_msg[1] & 0x20) == 0)
			press = 1;
		else if ( (atmel_msg[1] & 0x40) == 0 && (atmel_msg[1] & 0x20) == 0x20)
			press = 0;
		else {
			enable_irq(tsp.irq);
			return;
		}

		if (press == 1) {
			input_report_abs(tsp.inputdevice, ABS_X, x480);
			input_report_abs(tsp.inputdevice, ABS_Y, y800);
			input_report_key(tsp.inputdevice, BTN_TOUCH, DEFAULT_PRESSURE_DOWN);
			input_report_abs(tsp.inputdevice, ABS_PRESSURE, DEFAULT_PRESSURE_DOWN);
			input_sync(tsp.inputdevice);
			if (en_touch_log) {
				printk("[TSP][DOWN] id=%d, x=%d, y=%d, press=%d \n",(int)atmel_msg[0], x480, y800, press);
				en_touch_log = 0;
			} else if (press == 0) {
#ifdef 	CONFIG_MACH_TICKERTAPE
				if (keycode) {
					input_report_key(tsp.inputdevice, keycode, DEFAULT_PRESSURE_UP	);
					printk("[TSP][KEY][TICKER] \n");
				}
#ifdef  TICKERTAPE_SFN
				else if (y800 == 1024)
					printk("[TSP][BLACK][TICKER] \n");
#endif
				else {
					input_report_key(tsp.inputdevice, BTN_TOUCH, DEFAULT_PRESSURE_UP	);
					input_report_abs(tsp.inputdevice, ABS_PRESSURE, DEFAULT_PRESSURE_UP);
				}
#else
				input_report_key(tsp.inputdevice, BTN_TOUCH, DEFAULT_PRESSURE_UP	);
				input_report_abs(tsp.inputdevice, ABS_PRESSURE, DEFAULT_PRESSURE_UP);
#endif
				input_sync(tsp.inputdevice);
				printk("[TSP][UP] id=%d, x=%d, y=%d, press=%d \n",(int)atmel_msg[0], x480, y800, press);
				en_touch_log = 1;
			}
		} else {
			printk("[TSP][ERROR] %s() - read fail \n", __FUNCTION__);
		}
		enable_irq(tsp.irq);
		return;
	}
}

void read_func_for_multi_touch(struct work_struct *work) {
	uint8_t object_type, instance;
	struct touchscreen_t *ts = container_of(work, struct touchscreen_t, tsp_work);

	if (enable_touch_boost) {
		if (timer_pending(&ts->opp_set_timer))
			del_timer(&ts->opp_set_timer);
		mod_timer(&ts->opp_set_timer, jiffies + (1000 * HZ) / 1000);
	}

	if (read_mem(message_processor_address, max_message_length, atmel_msg) != READ_MEM_OK) {
		dbg("read fail\n");
		enable_irq(tsp.irq);
		return;
	}

	object_type = report_id_to_type(atmel_msg[0], &instance);

	switch (object_type) {
	case GEN_COMMANDPROCESSOR_T6:
	case TOUCH_MULTITOUCHSCREEN_T9:
	case PROCI_GRIPFACESUPPRESSION_T20:
#ifdef FEATURE_2_MORE_MULTI_TOUCH
		handle_2_more_touch(atmel_msg);
#else
		handle_multi_touch(atmel_msg);
#endif
		break;

	case TOUCH_KEYARRAY_T15:
		handle_keyarray_of_archer(atmel_msg);
		break;

	case PROCG_NOISESUPPRESSION_T22: {
		extern void noise_detected(void);
		extern unsigned char is_ta_on(void);
		printk("[TSP]   PROCG_NOISESUPPRESSION_T22 detected\n");
		if ( is_ta_on() && (atmel_msg[1] & 0x1) ) {
			dbg("FREQ CHANGE detected\n");
			noise_detected();
		}
	}
	case SPT_GPIOPWM_T19:
	case PROCI_ONETOUCHGESTUREPROCESSOR_T24:
	case PROCI_TWOTOUCHGESTUREPROCESSOR_T27:
	case SPT_SELFTEST_T25:
	case SPT_CTECONFIG_T28:
	default:
		dbg("0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\n",
		    atmel_msg[0], atmel_msg[1], atmel_msg[2],
		    atmel_msg[3], atmel_msg[4], atmel_msg[5],
		    atmel_msg[6], atmel_msg[7], atmel_msg[8]);
		break;
	}

	enable_irq(tsp.irq);
	return;
}


void atmel_touchscreen_read(struct work_struct *work) {
	read_func_for_multi_touch(work);
}

void clear_touch_int(void) {
	if (read_mem(message_processor_address, max_message_length, atmel_msg) == READ_MEM_OK) {
		dbg("[REAL]: 0x%02x, 0x%02x \n", atmel_msg[2], atmel_msg[3]);
	}
}

static irqreturn_t touchscreen_handler(int irq, void *dev_id) {
	disable_irq_nosync(irq);
	queue_work(tsp_wq, &tsp.tsp_work);
	return IRQ_HANDLED;
}

extern int atmel_touch_probe(void);

int  enable_irq_handler(void) {
	if (tsp.irq != -1) {
		tsp.irq = OMAP_GPIO_IRQ(OMAP_GPIO_TSP_INT);
		tsp.irq_type = IRQF_TRIGGER_LOW;

		if (request_irq(tsp.irq, touchscreen_handler, tsp.irq_type, TOUCHSCREEN_NAME, &tsp)) {
			perr("Could not allocate touchscreen IRQ!\n");
			tsp.irq = -1;
			input_free_device(tsp.inputdevice);
			return -EINVAL;
		}
		dbg("Registered touchscreen IRQ!\n");
		if (g_enable_touchscreen_handler == 0)
			g_enable_touchscreen_handler = 1;
		tsp.irq_enabled = 1;
	}
	return 0;
}


static int __init touchscreen_probe(struct platform_device *pdev) {
	int ret;
	int error = -1;
	struct kobject *ts_kobj;

	dbg("Atmel touch driver!!\n");

	set_touch_irq_gpio_disable();
	memset(&tsp, 0, sizeof(tsp));

	tsp.inputdevice = input_allocate_device();
	if (!tsp.inputdevice) {
		perr("Failed to allocate input device\n");
		return -ENOMEM;
	}

	spin_lock_init(&tsp.lock);

	if (tsp.irq != -1) {
		tsp.irq = OMAP_GPIO_IRQ(OMAP_GPIO_TSP_INT);
		tsp.irq_type = IRQF_TRIGGER_LOW;
		tsp.irq_enabled = 1;
	}

	tsp.inputdevice->name		= TOUCHSCREEN_NAME;
	tsp.inputdevice->evbit[0]	= BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) | BIT_MASK(EV_SYN);
	tsp.inputdevice->keybit[BIT_WORD(BTN_TOUCH)]	= BIT_MASK(BTN_TOUCH);
	tsp.inputdevice->id.bustype	= BUS_I2C;
	tsp.inputdevice->id.vendor	= 0;
	tsp.inputdevice->id.product	= 0;
	tsp.inputdevice->id.version	= 0;

	tsp.inputdevice->keybit[BIT_WORD(TOUCH_MENU)] |= BIT_MASK(TOUCH_MENU);
	tsp.inputdevice->keybit[BIT_WORD(TOUCH_BACK)] |= BIT_MASK(TOUCH_BACK);
	tsp.inputdevice->keycode = atmel_ts_tk_keycode;
	input_set_abs_params(tsp.inputdevice, ABS_MT_POSITION_X, 0, MAX_TOUCH_X_RESOLUTION, 0, 0);
	input_set_abs_params(tsp.inputdevice, ABS_MT_POSITION_Y, 0, MAX_TOUCH_Y_RESOLUTION, 0, 0);
	input_set_abs_params(tsp.inputdevice, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(tsp.inputdevice, ABS_MT_WIDTH_MAJOR, 0, 15, 0, 0);

	ret = input_register_device(tsp.inputdevice);
	if (ret) {
		perr("Unable to register input device: %s\n", tsp.inputdevice->name);
	}

	tsp_wq = create_singlethread_workqueue("tsp_wq");
	INIT_WORK(&tsp.tsp_work, atmel_touchscreen_read);

	touch_proc = create_proc_entry("touch_proc", S_IFREG | S_IRUGO | S_IWUGO, 0);
	if (touch_proc) {
		touch_proc->proc_fops = &touch_proc_fops;
		dbg("Succeeded in initializing touch proc file\n");
	} else {
		perr("Error occured in initializing touch proc file\n");
	}

	if (atmel_touch_probe() < 0) {
		return ENXIO;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	tsp.early_suspend.level		= EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	tsp.early_suspend.suspend	= atmel_ts_early_suspend;
	tsp.early_suspend.resume	= atmel_ts_late_resume;
	register_early_suspend(&tsp.early_suspend);
#endif

	ts_kobj = kobject_create_and_add("touchscreen", NULL);
	if (!ts_kobj)
		return -ENOMEM;

	error = sysfs_create_file(ts_kobj, &touch_boost_attr.attr);
	if (error) {
		perr("sysfs_create_file failed: %d\n", error);
		return error;
	}

	error = sysfs_create_file(ts_kobj, &firmware_attr.attr);
	if (error) {
		perr("sysfs_create_file failed: %d\n", error);
		return error;
	}

	/*------------------------------ for tunning ATmel - start ----------------------------*/
	error = sysfs_create_file(ts_kobj, &dev_attr_set_power.attr);
	if (error) {
		dbg("sysfs_create_file failed: %d\n", error);
		return error;
	}

	error = sysfs_create_file(ts_kobj, &dev_attr_set_acquisition.attr);
	if (error) {
		dbg("sysfs_create_file failed: %d\n", error);
		return error;
	}

	error = sysfs_create_file(ts_kobj, &dev_attr_set_touchscreen.attr);
	if (error) {
		dbg("sysfs_create_file failed: %d\n", error);
		return error;
	}

	error = sysfs_create_file(ts_kobj, &dev_attr_set_keyarray.attr);
	if (error) {
		dbg("sysfs_create_file failed: %d\n", error);
		return error;
	}

	error = sysfs_create_file(ts_kobj, &dev_attr_set_grip.attr);
	if (error) {
		dbg("sysfs_create_file failed: %d\n", error);
		return error;
	}

	error = sysfs_create_file(ts_kobj, &dev_attr_set_noise.attr);
	if (error) {
		dbg("sysfs_create_file failed: %d\n", error);
		return error;
	}

	error = sysfs_create_file(ts_kobj, &dev_attr_set_total.attr);
	if (error) {
		dbg("sysfs_create_file failed: %d\n", error);
		return error;
	}

	error = sysfs_create_file(ts_kobj, &dev_attr_set_write.attr);
	if (error) {
		dbg("sysfs_create_file failed: %d\n", error);
		return error;
	}

	error = sysfs_create_file(ts_kobj, &dev_attr_set_tsp_for_extended_indicator.attr);
	if (error) {
		dbg("sysfs_create_file failed: %d\n", error);
		return error;
	}

	error = sysfs_create_file(ts_kobj, &dev_attr_set_tsp_for_inputmethod.attr);
	if (error) {
		dbg("sysfs_create_file failed: %d\n", error);
		return error;
	}

#ifdef FEATURE_TSP_FOR_TA
	error = sysfs_create_file(ts_kobj, &dev_attr_set_tsp_for_TA.attr);
	if (error) {
		dbg("sysfs_create_file failed: %d\n", error);
		return error;
	}
#endif

#ifdef FEATURE_DYNAMIC_SLEEP
	error = sysfs_create_file(ts_kobj, &dev_attr_set_sleep_way_of_TSP.attr);
	if (error) {
		dbg("sysfs_create_file failed: %d\n", error);
		return error;
	}
#endif

	error = sysfs_create_file(ts_kobj, &dev_attr_set_TSP_debug_on.attr);
	if (error) {
		dbg("sysfs_create_file failed: %d\n", error);
		return error;
	}

	error = sysfs_create_file(ts_kobj, &dev_attr_set_test_value.attr);
	if (error) {
		dbg("sysfs_create_file failed: %d\n", error);
		return error;
	}

	error = sysfs_create_file(ts_kobj, &dev_attr_TSP_filter_status.attr);
	if (error) {
		dbg("sysfs_create_file failed: %d\n", error);
		return error;
	}

	init_timer(&tsp.opp_set_timer);
	tsp.opp_set_timer.data = (unsigned long)&tsp;
	tsp.opp_set_timer.function = tsc_timer_out;

	INIT_WORK(&(tsp.constraint_wq), tsc_remove_constraint_handler);

	dbg("Probe Success\n");
	return 0;
}

int touch_proc_ioctl(struct inode *p_node, struct file *p_file, unsigned int cmd, unsigned long arg) {
	switch (cmd) {
	case TOUCH_GET_VERSION : {
		char fv[10];
		sprintf(fv,"0X%x", g_version);
		if (copy_to_user((void*)arg, (const void*)fv, sizeof(fv)))
			return -EFAULT;
	}
	break;

	case TOUCH_GET_T_KEY_STATE : {
		int key_short = 0;
		key_short = menu_button || back_button;
		if (copy_to_user((void*)arg, (const void*)&key_short, sizeof(int)))
			return -EFAULT;
	}
	break;

	case TOUCH_GET_SW_VERSION :
		if (copy_to_user((void*)arg, (const void*)fw_version, sizeof(fw_version)))
			return -EFAULT;
		break;
	}
	return 0;
}

static int touchscreen_remove(struct platform_device *pdev) {
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&tsp.early_suspend);
#endif
	input_unregister_device(tsp.inputdevice);
	if (tsp.irq != -1) {
		if (g_enable_touchscreen_handler == 1) {
			free_irq(tsp.irq, &tsp);
			g_enable_touchscreen_handler = 0;
		}
	}
	touchscreen_poweronoff(0);
	return 0;
}

extern int atmel_suspend(void);
extern int atmel_resume(void);

static int touchscreen_suspend(struct platform_device *pdev, pm_message_t state) {
	dbg("touchscreen_suspend : touch power off\n");
	disable_irq_nosync(tsp.irq);
	atmel_suspend();
	initialize_multi_touch();
	return 0;
}

static int touchscreen_resume(struct platform_device *pdev) {
	dbg("Touchscreen_resume start : touch power on\n");
#ifdef FEATURE_VERIFY_INCORRECT_PRESS
	new_detect_id	= 0;
	new_detect_x	= 0;
	fixed_id2_x	= 0;
	fixed_id3_x	= 0;
	fixed_id2_count	= 0;
	fixed_id3_count	= 0;

	init_acc_count(2);
	init_acc_count(3);
#endif
	atmel_resume();
	initialize_multi_touch();
	enable_irq(tsp.irq);
	dbg("touchscreen_resume end\n");
	return 0;
}

static void touchscreen_device_release(struct device *dev) {
	/* Nothing */
}

static struct platform_driver touchscreen_driver = {
	.probe 		= touchscreen_probe,
	.remove 	= touchscreen_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend 	= &touchscreen_suspend,
	.resume 	= &touchscreen_resume,
#endif
	.driver = {
		.name	= TOUCHSCREEN_NAME,
	},
};

static struct platform_device touchscreen_device = {
	.name 		= TOUCHSCREEN_NAME,
	.id 		= -1,
	.dev = {
		.release 	= touchscreen_device_release,
	},
};

#ifdef CONFIG_HAS_EARLYSUSPEND
void atmel_ts_early_suspend(struct early_suspend *h) {
	touchscreen_suspend(&touchscreen_device, PMSG_SUSPEND);
}

void atmel_ts_late_resume(struct early_suspend *h) {
	touchscreen_resume(&touchscreen_device);
}
#endif

static int __init touchscreen_init(void) {
	int ret;

	mdelay(10);

	if (gpio_is_valid(OMAP_GPIO_TSP_EN)) {
		if (gpio_request(OMAP_GPIO_TSP_EN, "touch") < 0) {
			printk("Filed to request OMAP_GPIO_TSP_EN!\n");
		}
		gpio_direction_output(OMAP_GPIO_TSP_EN, 0);
	}

	gpio_set_value(OMAP_GPIO_TSP_EN, 1);
	mdelay(10);
	wake_lock_init(&tsp_firmware_wake_lock, WAKE_LOCK_SUSPEND, "TSP");

	ret = platform_device_register(&touchscreen_device);
	if (ret != 0)
		return -ENODEV;

	ret = platform_driver_register(&touchscreen_driver);
	if (ret != 0) {
		platform_device_unregister(&touchscreen_device);
		return -ENODEV;
	}

	return 0;
}

static void __exit touchscreen_exit(void) {
	wake_lock_destroy(&tsp_firmware_wake_lock);
	platform_driver_unregister(&touchscreen_driver);
	platform_device_unregister(&touchscreen_device);
}

int touchscreen_get_tsp_int_num(void) {
	return tsp.irq;
}


#ifdef FEATURE_CAL_BY_INCORRECT_PRESS
unsigned char is_pressed(void) {
	return ((id2.press == 1) || (id3.press == 1));
}
#endif


#ifdef FEATURE_VERIFY_INCORRECT_PRESS
static int repeat_count_id2=0;
static int repeat_count_id3=0;
static int saved_x_id2 = 0;
static int saved_x_id3 = 0;

static unsigned char figure_x_count(int id, int x) {
	if (id == 2) {
		if (saved_x_id2 == x) {
			repeat_count_id2++;
			dbg("x of id2:%d is repeated %d times\n",x, repeat_count_id2);
		} else {
			saved_x_id2 = x;
			repeat_count_id2 = 1;
			dbg("x of id2:%d is updated newly as %d times\n", x, repeat_count_id2);
		}
	}

	if (id == 3) {
		if (saved_x_id3 == x) {
			repeat_count_id3++;
			dbg("x of id3:%d is repeated %d times\n", x, repeat_count_id3);
		} else {
			saved_x_id3 = x;
			repeat_count_id3 = 1;
			dbg("x of id3:%d is updated newly as %d times\n",x, repeat_count_id3);
		}
	}

	if ( (repeat_count_id3 >= incorrect_repeat_count)
	        || (repeat_count_id2 >= incorrect_repeat_count) ) {
		saved_x_id3 = 0;
		saved_x_id2 = 0;
		repeat_count_id3 = 0;
		repeat_count_id2 = 0;
		dbg("incorrect press by noise is detected\n");
		return 1;
	}
	return 0;
}

static void init_acc_count(int id) {
	dbg("ID%d's variable is initialized\n", id);

	if (id == 2) {
		repeat_count_id2 = 0;
		saved_x_id2 = 0;
	}

	if (id == 3) {
		repeat_count_id3 = 0;
		saved_x_id3 = 0;
	}
}

void check_whether_x_is_fixed(void) {
	if (id2.press) {
		if (fixed_id2_x != 0 && tmp2.x==fixed_id2_x) {
			fixed_id2_count++;
			dbg("id2.x:%d, fixed_id2_count increased to %d\n",tmp2.x, fixed_id2_count);
		} else {
			fixed_id2_count = 1;
			fixed_id2_x = tmp2.x;
			dbg("new fixed_id2_count:%d happened\n",fixed_id2_x );
		}
	} else {
		dbg("fixed information id2 is cleared to 0\n");
		fixed_id2_count = 0;
		fixed_id2_x = 0;
	}

	if (id3.press) {
		if (fixed_id3_x != 0 &&  tmp3.x==fixed_id3_x) {
			fixed_id3_count++;
			dbg("id3.x:%d, fixed_id3_count increased to %d\n",tmp3.x, fixed_id3_count);
		} else {
			fixed_id3_count = 1;
			fixed_id3_x = tmp3.x;
			dbg("new fixed_id3_count:%d happened\n",fixed_id3_x );
		}
	} else {
		dbg("fixed information id3 is cleared to 0\n");
		fixed_id3_count = 0;
		fixed_id3_x = 0;
	}

	if (fixed_id3_count >= 3 || fixed_id2_count >= 3) {
		fixed_id2_x = 0;
		fixed_id3_x = 0;
		fixed_id2_count = 0;
		fixed_id3_count = 0;

		dbg("Do calibrate chip because of noise when home key is pressed\n");
		calibrate_chip();
	}
}
#endif

module_init(touchscreen_init);
module_exit(touchscreen_exit);

MODULE_LICENSE("GPL");

