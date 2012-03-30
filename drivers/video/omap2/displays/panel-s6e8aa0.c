/*
 * Samsung MIPI panel support
 *
 * Copyright 2011 samsung electronics.
 *
 * based on d2l panel driver by Jerry Alexander <x0135174@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <plat/display.h>

#include <plat/samsung-dsi-panel.h>

#include "panel-s6e8aa0.h"


/* Indicates the state of the device */
int bd_brightness = 0;
struct s6e8aa0_data *d2d;

/* define this if you want debug print messages */
/* #define DEB */

#ifdef DEB
#define PRINT_DEBUG(...) printk(__VA_ARGS__)
#else
#define PRINT_DEBUG(...)
#endif

static int s6e8aa0_update(struct omap_dss_device *dssdev,
		      u16 x, u16 y, u16 w, u16 h);


/**
 * struct panel_config - panel configuration
 * @name: panel name
 * @type: panel type
 * @timings: panel resolution
 * @sleep: various panel specific delays, passed to msleep() if non-zero
 * @reset_sequence: reset sequence timings, passed to udelay() if non-zero
 */
struct panel_config {
	const char *name;
	int type;

	struct omap_video_timings timings;
	struct omap_dsi_video_timings dsi_video_timings;

	struct {
		unsigned int sleep_in;
		unsigned int sleep_out;
		unsigned int hw_reset;
		unsigned int enable_te;
	} sleep;

	struct {
		unsigned int high;
		unsigned int low;
	} reset_sequence;

	u32 width_in_mm;
	u32 height_in_mm;;
};

enum {
	PANEL_S6E8AA0,
};

static struct panel_config panel_configs[] = {
	{
		.name		= "s6e8aa0_panel",
		.type		= PANEL_S6E8AA0,
		.timings	= {	
			.x_res		= AMS529_WIDTH,
			.y_res		= AMS529_HEIGHT,
			.pixel_clock = AMS529_PCLK,
			.hsw		= AMS529_HSW,
			.hfp		= AMS529_HFP,
			.hbp		= AMS529_HBP,
			.vfp		= AMS529_VFP,
			.vbp		= AMS529_VBP,	
			.vsw 		= AMS529_VSW,			
		},
		.dsi_video_timings = {
				.hsa = DSI_HSA, 
				.hfp = DSI_HFP, 
				.hbp = DSI_HBP, 
				.vsa = DSI_VSA,
				.vfp = DSI_VFP,
				.vbp = DSI_VBP,
				},		
		.width_in_mm = 56,
		.height_in_mm = 93,
	},
};

struct s6e8aa0_data {
	struct mutex lock;
	struct backlight_device *bldev;
	struct omap_dss_device *dssdev;
	bool enabled;
	u8 rotate;
	bool mirror;
	bool use_dsi_bl;
	unsigned int			power;
	unsigned int			gamma_mode;
	unsigned int			current_brightness;
	unsigned int			gamma_table_count;
	unsigned int			bl;
	unsigned int			beforepower;
	unsigned int			ldi_enable;
	unsigned int 			acl_enable;
	unsigned int 			cur_acl;	
	unsigned long hw_guard_end;	/* next value of jiffies when we can
					 * issue the next sleep in/out command
					 */
	unsigned long hw_guard_wait;	/* max guard time in jiffies */

	atomic_t do_update;
	struct {
		u16 x;
		u16 y;
		u16 w;
		u16 h;
	} update_region;

	bool cabc_broken;
	unsigned cabc_mode;

	bool force_update;
	struct panel_config *panel_config;
};


static inline struct samsung_dsi_panel_data
*get_panel_data(const struct omap_dss_device *dssdev)
{
	return (struct samsung_dsi_panel_data *)dssdev->data;
}

/*************************************
**** Interface utility functions *****
*************************************/
static void hw_guard_start(int guard_msec)
{
	d2d->hw_guard_wait = msecs_to_jiffies(guard_msec);
	d2d->hw_guard_end = jiffies + d2d->hw_guard_wait;
}

static void hw_guard_wait(void)
{
	unsigned long wait = d2d->hw_guard_end - jiffies;

	if ((long)wait > 0 && wait <= d2d->hw_guard_wait) {
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(wait);
	}
}

static int s6e8aa0_read(u8 cmd, u8 *buf, int len)
{
	int ret;
	enum omap_dsi_index lcd_ix = DSI1;

	ret = dsi_vc_dcs_read(lcd_ix, TCH, cmd, buf, len);
	mdelay(5);

	return ret;
}

static int s6e8aa0_write( u8 dcs_cmd)
{
	enum omap_dsi_index lcd_ix = DSI1;
	return dsi_vc_dcs_write(lcd_ix, TCH, &dcs_cmd, 1);
}

static int s6e8aa0_write_reg(u8 dcs_cmd, u8 param)
{
	enum omap_dsi_index lcd_ix = DSI1;
	u8 buf[2];
	buf[0] = dcs_cmd;
	buf[1] = param;
	return dsi_vc_dcs_write(lcd_ix, TCH, buf, 2);
}

static int s6e8aa0_write_reg2(u8 dcs_cmd, u8 param1, u8 param2)
{
	
	enum omap_dsi_index lcd_ix = DSI1;
	u8 buf[3];
	buf[0] = dcs_cmd;
	buf[1] = param1;
	buf[2] = param2;


	return dsi_vc_dcs_write(lcd_ix, TCH, buf, 3);
}

static int s6e8aa0_write_block(const u8 *buf, int len)
{
	int ret;
	enum omap_dsi_index lcd_ix = DSI1;

	ret = dsi_vc_dcs_write(lcd_ix, TCH, (u8 *)buf, len);;
	mdelay(5);

	return ret;
}

static int s6e8aa0_panel_id(void)
{
	enum omap_dsi_index lcd_ix = DSI1;
	u8 buf[3] = {0,};

	dsi_vc_set_max_rx_packet_size(lcd_ix, TCH, 3);

	s6e8aa0_read(0xDA, buf, 1);
	printk("* PANEL_S6E8AA0_ID_READ : 0x%02x\n", buf[0]);
	
	s6e8aa0_read(0xD1, buf, 3);
	printk("* PANEL_S6E8AA0_ID_READ : 0x%02x, 0x%02x, 0x%02x.\n", buf[0], buf[1], buf[2]);

	return buf[1];
}
EXPORT_SYMBOL(s6e8aa0_panel_id);

/***********************
*** DUMMY FUNCTIONS ****
***********************/

static int s6e8aa0_rotate(struct omap_dss_device *dssdev, u8 rotate)
{
	return 0;
}

static u8 s6e8aa0_get_rotate(struct omap_dss_device *dssdev)
{
	return 0;
}

static int s6e8aa0_mirror(struct omap_dss_device *dssdev, bool enable)
{
	return 0;
}

static bool s6e8aa0_get_mirror(struct omap_dss_device *dssdev)
{
	return 0;
}

static void s6e8aa0_get_timings(struct omap_dss_device *dssdev,
			    struct omap_video_timings *timings)
{
	*timings = dssdev->panel.timings;
}

static void s6e8aa0_set_timings(struct omap_dss_device *dssdev,
			    struct omap_video_timings *timings)
{
	dssdev->panel.timings.x_res = timings->x_res;
	dssdev->panel.timings.y_res = timings->y_res;
	dssdev->panel.timings.pixel_clock = timings->pixel_clock;
	dssdev->panel.timings.hsw = timings->hsw;
	dssdev->panel.timings.hfp = timings->hfp;
	dssdev->panel.timings.hbp = timings->hbp;
	dssdev->panel.timings.vsw = timings->vsw;
	dssdev->panel.timings.vfp = timings->vfp;
	dssdev->panel.timings.vbp = timings->vbp;
}

static int s6e8aa0_check_timings(struct omap_dss_device *dssdev,
			     struct omap_video_timings *timings)
{
	if (timings->x_res != AMS529_WIDTH || timings->y_res != AMS529_HEIGHT)
		return -EINVAL;

	return 0;
}

static void s6e8aa0_get_resolution(struct omap_dss_device *dssdev,
			       u16 *xres, u16 *yres)
{
//	struct s6e8aa0_data *d2d = dev_get_drvdata(&dssdev->dev);

	if (d2d->rotate == 0 || d2d->rotate == 2) {
		*xres = dssdev->panel.timings.x_res;
		*yres = dssdev->panel.timings.y_res;
	} else {
		*yres = dssdev->panel.timings.x_res;
		*xres = dssdev->panel.timings.y_res;
	}
}

static int s6e8aa0_enable_te(struct omap_dss_device *dssdev, bool enable)
{
	return 0;
}

static int s6e8aa0_hw_reset(struct omap_dss_device *dssdev)
{
	struct samsung_dsi_panel_data *panel_data = get_panel_data(dssdev);

	if (panel_data->reset_gpio == -1)
		return -1;

	gpio_set_value(panel_data->reset_gpio, 1);
	mdelay(20);
	/* reset the panel */
	gpio_set_value(panel_data->reset_gpio, 0);
	/* assert reset */
	mdelay(10);
	gpio_set_value(panel_data->reset_gpio, 1);
	/* wait after releasing reset */
//	mdelay(10);

	return 0;
}

#ifdef ACL_ENABLE 
static void s6e8aa0_acl_onoff(unsigned int isOn)
{
	if (isOn)
		s6e8aa0_write_reg(0xC0, 0x01);
	else
		s6e8aa0_write_reg(0xC0, 0x00);        
}

static void panel_acl_send_sequence(const unsigned char * acl_param_tbl, unsigned int count)
{
	
	/* Level 2 key command */
	s6e8aa0_write_block(s6e8aa0_init_pre1, ARRAY_SIZE(s6e8aa0_init_pre1));
	
	s6e8aa0_write_block(acl_param_tbl, count);

	s6e8aa0_acl_onoff(1);   
}

static int s6e8aa0_set_acl(void)
{
	int ret = 0;
	enum omap_dsi_index lcd_ix = DSI1;

	if(!d2d->enabled) {
		ret = -1;
		goto acl_err;
	}

	if (d2d->acl_enable) 
	{
		if(d2d->cur_acl == 0) s6e8aa0_acl_onoff(1);
		
		switch (d2d->bl) {			
		case 0 ... 1: /* 30cd ~ 40cd */			
			if (d2d->cur_acl != 0) {
				panel_acl_send_sequence(acl_cutoff_param_set_tbl[0], ACL_CUTOFF_PARAM_SET_DATA_COUNT);
				printk(KERN_INFO "ACL_cutoff_set Percentage : off!!\n");
				d2d->cur_acl = 0;
			}
			break;
		case 2 ... 12: /* 70cd ~ 180cd */
			if (d2d->cur_acl != 40) {
				panel_acl_send_sequence(acl_cutoff_param_set_tbl[1], ACL_CUTOFF_PARAM_SET_DATA_COUNT);
				printk(KERN_INFO "ACL_cutoff_set Percentage : 40!!\n");
				d2d->cur_acl = 40;
			}
			break;
#if 0			
		case 13: /* 190cd */
			if (d2d->cur_acl != 43) {
				panel_acl_send_sequence(acl_cutoff_param_set_tbl[2], ACL_CUTOFF_PARAM_SET_DATA_COUNT);
				dev_dbg(&dssdev->dev, "ACL_cutoff_set Percentage : 43!!\n");
				d2d->cur_acl = 43;
			}
			break;
		case 14: /* 200cd */			
			if (d2d->cur_acl != 45) {
				panel_acl_send_sequence(acl_cutoff_param_set_tbl[3], ACL_CUTOFF_PARAM_SET_DATA_COUNT);
				dev_dbg(&dssdev->dev, "ACL_cutoff_set Percentage : 45!!\n");
				d2d->cur_acl = 45;
			}
			break;
		case 15: /* 210cd */
			if (d2d->cur_acl != 47) {
				panel_acl_send_sequence(acl_cutoff_param_set_tbl[4], ACL_CUTOFF_PARAM_SET_DATA_COUNT);
				dev_dbg(&dssdev->dev, "ACL_cutoff_set Percentage : 47!!\n");
				d2d->cur_acl = 47;
			}
			break;
		case 16: /* 220cd */			
			if (d2d->cur_acl != 48) {
				panel_acl_send_sequence(acl_cutoff_param_set_tbl[5], ACL_CUTOFF_PARAM_SET_DATA_COUNT);
				dev_dbg(&dssdev->dev, "ACL_cutoff_set Percentage : 48!!\n");
				d2d->cur_acl = 48;
			}
			break;
#endif			
		default:
			if (d2d->cur_acl != 50) {
				panel_acl_send_sequence(acl_cutoff_param_set_tbl[6], ACL_CUTOFF_PARAM_SET_DATA_COUNT);
				printk(KERN_INFO "ACL_cutoff_set Percentage : 50!!\n");
				d2d->cur_acl = 50;
			}
			break;
		}
	}
	else {
		if (d2d->cur_acl != 0) {
			panel_acl_send_sequence(acl_cutoff_param_set_tbl[0], ACL_CUTOFF_PARAM_SET_DATA_COUNT);
			d2d->cur_acl = 0;
			printk(KERN_INFO "ACL_cutoff_set Percentage : off!!\n");
			}
	}
	
	if (ret) {
		ret = -1;
		goto acl_err;
	}
	printk(KERN_INFO "%s - %d, %d\n", __func__, d2d->acl_enable, d2d->cur_acl);
acl_err:
	return ret;
}
#endif

static void s6e8aa0_gamma_update(void)
{
	/* GAMMA Update */
	s6e8aa0_write_reg(0xF7, 0x01);
}

static int s6e8aa0_gamma_ctl(void)
{
	int ret = 0;

	if(!d2d->enabled) {
		ret = -1;
		goto gamma_err;
	}
	
	if(d2d->gamma_mode)
		s6e8aa0_write_block(ams529ha01_19gamma_set_tbl[d2d->bl], GAMMA_PARAM_SET_DATA_COUNT);
	else
		s6e8aa0_write_block(ams529ha01_22gamma_set_tbl[d2d->bl], GAMMA_PARAM_SET_DATA_COUNT);

	s6e8aa0_gamma_update();

gamma_err:
	return ret;
}

static int update_brightness(void)
{
	int ret = 0;
	enum omap_dsi_index lcd_ix = DSI1;
	
	mutex_lock(&d2d->lock);
	dsi_bus_lock(lcd_ix);	
	
#ifdef ACL_ENABLE 
	ret = s6e8aa0_set_acl();
	if (ret) {
		printk(KERN_ERR "lcd ACL setting failed.\n");
		goto err;
		}
#endif

	ret = s6e8aa0_gamma_ctl();
	if (ret) {
		printk(KERN_ERR "lcd brightness setting failed.\n");
		goto err;
		}

err:
	dsi_bus_unlock(lcd_ix);	
	mutex_unlock(&d2d->lock);	
	
	return ret;
}

static int get_gamma_value_from_bl(int bl)
{
	int gamma_value =0;
	int gamma_val_x10 =0;

	if(bl >= MIN_BL){
		gamma_val_x10 = 10 *(MAX_GAMMA_VALUE-1)*bl/(MAX_BL-MIN_BL) + (10 - 10*(MAX_GAMMA_VALUE-1)*(MIN_BL)/(MAX_BL-MIN_BL));
		gamma_value=(gamma_val_x10 +5)/10;
	}else{
		gamma_value =0;
	}

	return gamma_value;
}

static int s6e8aa0_set_brightness(struct backlight_device* bd)
{
	int bl = bd->props.brightness;
	int ret = 0;
	int current_bl;

	if (bl < MIN_BRIGHTNESS || bl > bd->props.max_brightness) {
		dev_err(&bd->dev, "lcd brightness should be %d to %d. now %d\n",
			MIN_BRIGHTNESS, MAX_BRIGHTNESS, bl);
		return -EINVAL;
	}

	current_bl = get_gamma_value_from_bl(bl);
	if ( d2d->bl != current_bl)	{
		d2d->bl = current_bl;
		dev_info(&bd->dev, "update brightness....%d\n", d2d->bl);
		ret = update_brightness();
		if (ret < 0) {
			dev_err(&bd->dev, "skip update brightness. because lcd is on suspend state...\n");
			return -EINVAL;
		}
	}

	return ret;
}

static int s6e8aa0_get_brightness(struct backlight_device* bd)
{
	dev_info(&bd->dev, "called %s \n",__func__);
	return bd_brightness;
}

static const struct backlight_ops s6e8aa0_backlight_ops  = {
	.get_brightness = s6e8aa0_get_brightness,
	.update_status = s6e8aa0_set_brightness,
};


/***************************
*** LCD Device Sysfs Entry ****
***************************/

static int s6e8aa0_start(struct omap_dss_device *dssdev);
static void s6e8aa0_stop(struct omap_dss_device *dssdev);

static ssize_t lcdtype_show(struct device *dev, 
						struct device_attribute *attr, char *buf)
{

	char temp[15];
	sprintf(temp, "SMD_AMS529HA01\n");
	strcat(buf, temp);
	return strlen(buf);
}

static ssize_t s6e8aa0_panel_id_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
//	struct omap_dss_device *dssdev = to_dss_device(dev);
	enum omap_dsi_index lcd_ix = DSI1;
	int ret;
	int id;
#if 1
	mutex_lock(&d2d->lock);
	dsi_bus_lock(lcd_ix);	

	if (d2d->enabled) {
		id = s6e8aa0_panel_id();

	} else {
		ret = -ENODEV;
	}
	dsi_bus_unlock(lcd_ix);	
	mutex_unlock(&d2d->lock);
#endif
	if (ret)
		return ret;

	return snprintf(buf, PAGE_SIZE, "%02x\n", id);
}

static ssize_t acl_set_show(struct device *dev,
						struct device_attribute *attr, char *buf)
{
//	struct omap_dss_device *dssdev = to_dss_device(dev);
	char temp[3];

	sprintf(temp, "%d\n", d2d->acl_enable);
	strcpy(buf, temp);

	return strlen(buf);
}
static ssize_t acl_set_store(struct device *dev, 
						struct device_attribute *attr, const char *buf, size_t size)
{
//	struct omap_dss_device *dssdev = to_dss_device(dev);
	enum omap_dsi_index lcd_ix = DSI1;
	int value;
	int rc;
	
	rc = strict_strtoul(buf, (unsigned int) 0, (unsigned long *)&value);
	if (rc < 0)
		return rc;
	else{
#ifdef ACL_ENABLE 		
		dev_info(dev, "acl_set_store - %d, %d\n", d2d->acl_enable, value);
		if (d2d->acl_enable != value) {
			
			mutex_lock(&d2d->lock);
			
			dsi_bus_lock(lcd_ix); 

			d2d->acl_enable = value;
			s6e8aa0_set_acl();

			
			dsi_bus_unlock(lcd_ix);
			
			mutex_unlock(&d2d->lock);
		}
#endif		
		return 0;
	}
}

static ssize_t s6e8aa0_sysfs_backlihgt_level_test(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t len)
{
//	struct omap_dss_device *dssdev = to_dss_device(dev);
	unsigned long brightness;
	int rc;

	rc = strict_strtoul(buf, 0, &brightness);
	if (rc < 0)
		return rc;
	else
		d2d->bldev->props.brightness = brightness;

	s6e8aa0_set_brightness(d2d->bldev);
	return 0;
}

static ssize_t s6e8aa0_sysfs_show_gamma_mode(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
//	struct omap_dss_device *dssdev = to_dss_device(dev);
	char temp[10];

	switch (d2d->gamma_mode) {
	case 0:
		sprintf(temp, "2.2 mode\n");
		strcat(buf, temp);
		break;
	case 1:
		sprintf(temp, "1.9 mode\n");
		strcat(buf, temp);
		break;
	default:
		dev_info(dev, "gamma mode could be 0:2.2, 1:1.9 or 2:1.7)n");
		break;
	}

	return strlen(buf);
}

static ssize_t s6e8aa0_sysfs_store_gamma_mode(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t len)
{
//	struct omap_dss_device *dssdev = to_dss_device(dev);
	int rc;
	
	rc = strict_strtoul(buf, 0, (unsigned long *)&d2d->gamma_mode);
	if (rc < 0)
		return rc;

	dev_info(dev, "s6e8aa0_sysfs_store_gamma_mode - %d\n", d2d->gamma_mode);
	
	if (d2d->enabled)
		 s6e8aa0_gamma_ctl();
	
	return len;
}

static ssize_t s6e8aa0_sysfs_store_lcd_power(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t len)
{
	int ret;
	int rc;
	int lcd_enable;
	struct omap_dss_device *dssdev = to_dss_device(dev);

	rc = strict_strtoul(buf, 0, (unsigned long *)&lcd_enable);
	if (rc < 0)
		return rc;
	dev_info(dev, "s6e8aa0_sysfs_store_lcd_power - %d\n", lcd_enable);

	if(lcd_enable) {
		if (dssdev->platform_enable)
			rc = dssdev->platform_enable(dssdev);
		ret = s6e8aa0_start(dssdev);
	}
	else {
		s6e8aa0_stop(dssdev);
		if (dssdev->platform_disable)
		        dssdev->platform_disable(dssdev);
	}

	return len;
}

static DEVICE_ATTR(lcdtype, S_IRUGO,lcdtype_show, NULL);
static DEVICE_ATTR(lcd_id, S_IRUGO, s6e8aa0_panel_id_show, NULL);
static DEVICE_ATTR(baktst, S_IWUGO, NULL, s6e8aa0_sysfs_backlihgt_level_test);
static DEVICE_ATTR(lcd_power, S_IWUGO, NULL, s6e8aa0_sysfs_store_lcd_power);
static DEVICE_ATTR(acl_set, S_IRUGO | S_IWUGO, acl_set_show, acl_set_store);
static DEVICE_ATTR(gamma_mode, S_IRUGO | S_IWUGO,
					s6e8aa0_sysfs_show_gamma_mode, s6e8aa0_sysfs_store_gamma_mode);



static struct attribute *s6e8aa0_attrs[] = {
	&dev_attr_lcdtype.attr,
	&dev_attr_lcd_id.attr,
	&dev_attr_acl_set.attr,
	&dev_attr_baktst.attr,
	&dev_attr_lcd_power.attr,
	&dev_attr_gamma_mode.attr,
	NULL,
};

static struct attribute_group s6e8aa0_attr_group = {
	.attrs = s6e8aa0_attrs,
};

static int s6e8aa0_probe(struct omap_dss_device *dssdev)
{
	struct backlight_properties props;
	struct backlight_device *bldev;
	struct samsung_dsi_panel_data *panel_data = get_panel_data(dssdev);
	struct panel_config *panel_config = NULL;
	int ret = 0;
	int i;

	dev_info(&dssdev->dev, "s6e8aa0_probe\n");

	if (!panel_data || !panel_data->name) {
		ret = -EINVAL;
		goto err;
	}

	for (i = 0; i < ARRAY_SIZE(panel_configs); i++) {
		if (strcmp(panel_data->name, panel_configs[i].name) == 0) {
			panel_config = &panel_configs[i];
			break;
		}
	}

	if (!panel_config) {
		ret = -EINVAL;
		goto err;
	}

	dssdev->panel.config = OMAP_DSS_LCD_TFT;
	dssdev->panel.timings = panel_config->timings;	
	dssdev->panel.data_type = DSI_DT_PXLSTREAM_24BPP_PACKED;
	dssdev->phy.dsi.vm_timing = panel_config->dsi_video_timings;
	dssdev->panel.width_in_mm = panel_config->width_in_mm;
	dssdev->panel.height_in_mm = panel_config->height_in_mm;
	dssdev->ctrl.pixel_size = 24;
	dssdev->panel.acbi = 0;
	dssdev->panel.acb = 40;

	d2d = kzalloc(sizeof(*d2d), GFP_KERNEL);
	if (!d2d) {
		ret = -ENOMEM;
		goto err;
	}

	d2d->dssdev = dssdev;

	d2d->panel_config = panel_config;

	mutex_init(&d2d->lock);

	atomic_set(&d2d->do_update, 0);
	dev_set_drvdata(&dssdev->dev, d2d);

	/* if no platform set_backlight() defined, presume DSI backlight  control */
	memset(&props, 0, sizeof(struct backlight_properties));

	bldev = backlight_device_register("pwm-backlight",
			&dssdev->dev, dssdev, &s6e8aa0_backlight_ops, &props);
	if (IS_ERR(bldev)) {
		ret = PTR_ERR(bldev);
		goto err;
	}

	d2d->bldev = bldev;

	bldev->props.fb_blank = FB_BLANK_UNBLANK;
	bldev->props.power = FB_BLANK_UNBLANK;
	bldev->props.max_brightness = MAX_BRIGHTNESS;
	bldev->props.brightness = 127;
	
	d2d->bl = 11;
	d2d->acl_enable = 1;
	d2d->cur_acl = 0;
	d2d->gamma_mode = 0;

	if (cpu_is_omap44xx())
		d2d->force_update = true;

		/* Do not turn off lcd during booting */
#ifndef CONFIG_FB_OMAP_BOOTLOADER_INIT
		d2d->enabled = 0;
#else
		d2d->enabled = 1;
#endif

	ret = sysfs_create_group(&dssdev->dev.kobj, &s6e8aa0_attr_group);
	if (ret)
		dev_err(&dssdev->dev, "failed to create sysfs files\n");

	dev_info(&dssdev->dev, "s6e8aa0 lcd panel driver based on mipi-dsi"
			" has been probed.\n");

	return ret;

err:
	kfree(d2d);

	return ret;
}

static void s6e8aa0_remove(struct omap_dss_device *dssdev)
{
//	struct s6e8aa0_data *d2d = dev_get_drvdata(&dssdev->dev);
	kfree(d2d);
}

static void s6e8aa0_ldi_on(struct omap_dss_device *dssdev)
{
	dev_info(&dssdev->dev, "++ %s, [%d] ++\n", __func__, __LINE__);
	s6e8aa0_write_block(s6e8aa0_init_pre1, ARRAY_SIZE(s6e8aa0_init_pre1));
	s6e8aa0_write_block(s6e8aa0_init_pre2, ARRAY_SIZE(s6e8aa0_init_pre2));

	s6e8aa0_write(0x11);

	s6e8aa0_write_block(s6e8aa0_init_panel, ARRAY_SIZE(s6e8aa0_init_panel));
	s6e8aa0_write_block(s6e8aa0_init_display, ARRAY_SIZE(s6e8aa0_init_display));

	s6e8aa0_write_block(s6e8aa0_init_post0, ARRAY_SIZE(s6e8aa0_init_post0));
	s6e8aa0_write_block(s6e8aa0_init_elvss, ARRAY_SIZE(s6e8aa0_init_elvss));
	s6e8aa0_write_block(s6e8aa0_init_post1, ARRAY_SIZE(s6e8aa0_init_post1));
	s6e8aa0_write_block(s6e8aa0_init_post2, ARRAY_SIZE(s6e8aa0_init_post2));
//	s6e8aa0_write_block(s6e8aa0_init_post3, ARRAY_SIZE(s6e8aa0_init_post3));

	msleep(120); 

	s6e8aa0_write(0x29);

	d2d->enabled = 1;
	dev_info(&dssdev->dev, "-- %s, [%d] --\n", __func__, __LINE__);
}

static void s6e8aa0_ldi_off(struct omap_dss_device *dssdev)
{
	dev_info(&dssdev->dev, "++ %s, [%d] ++\n", __func__, __LINE__);
	s6e8aa0_write(0x28);
	s6e8aa0_write(0x10);
	
	msleep(120);
	d2d->enabled = 0;
	
	dev_info(&dssdev->dev, "-- %s, [%d] --\n", __func__, __LINE__);
}

static int s6e8aa0_power_on(struct omap_dss_device *dssdev)
{
	int ret = 0;
	
	if (d2d->enabled != 1) {

		ret = omapdss_dsi_display_enable(dssdev);
		if (ret) {
			dev_err(&dssdev->dev, "failed to enable DSI\n");
			goto err;
		}

		/* reset s6e8aa0 bridge */
		s6e8aa0_hw_reset(dssdev);

		dsi_videomode_panel_preinit(dssdev);
		mdelay(20);
		
		s6e8aa0_ldi_on(dssdev);		
		
		dsi_videomode_panel_postinit(dssdev);
	}
	
err:
	return ret;
}

static void s6e8aa0_power_off(struct omap_dss_device *dssdev)
{
	s6e8aa0_ldi_off(dssdev);

	omapdss_dsi_display_disable(dssdev);
}

static int s6e8aa0_start(struct omap_dss_device *dssdev)
{
//	struct s6e8aa0_data *d2d = dev_get_drvdata(&dssdev->dev);
	int r = 0;
	enum omap_dsi_index lcd_ix;

	lcd_ix = DSI1;

	mutex_lock(&d2d->lock);

	dsi_bus_lock(lcd_ix);

	r = s6e8aa0_power_on(dssdev);

	dsi_bus_unlock(lcd_ix);

	if (r) {
		dev_info(&dssdev->dev, "enable failed\n");
		dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
	} else {
		dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
		dispc_enable_channel(dssdev->channel, true);
	}

	mutex_unlock(&d2d->lock);
	return r;
}

static void s6e8aa0_stop(struct omap_dss_device *dssdev)
{
//	struct s6e8aa0_data *d2d = dev_get_drvdata(&dssdev->dev);
	enum omap_dsi_index lcd_ix;

	lcd_ix = DSI1;

	mutex_lock(&d2d->lock);
	
	dsi_bus_lock(lcd_ix);

	s6e8aa0_power_off(dssdev);

	dsi_bus_unlock(lcd_ix);

	dispc_enable_channel(dssdev->channel, false);

	mutex_unlock(&d2d->lock);
}

static void s6e8aa0_disable(struct omap_dss_device *dssdev)
{
	dev_info(&dssdev->dev, "disable\n");

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE ||
	    dssdev->state == OMAP_DSS_DISPLAY_TRANSITION)
		s6e8aa0_stop(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static int s6e8aa0_enable(struct omap_dss_device *dssdev)
{
	dev_info(&dssdev->dev, "enable\n");

	if (dssdev->state != OMAP_DSS_DISPLAY_DISABLED)
		return -EINVAL;

	return s6e8aa0_start(dssdev);
}

static void s6e8aa0_framedone_cb(int err, void *data)
{
	struct omap_dss_device *dssdev = data;
	enum omap_dsi_index lcd_ix;

	lcd_ix = DSI1;

	dev_info(&dssdev->dev, "framedone, err %d\n", err);
	dsi_bus_unlock(lcd_ix);
}

static int s6e8aa0_update(struct omap_dss_device *dssdev,
		      u16 x, u16 y, u16 w, u16 h)
{
//	struct s6e8aa0_data *d2d = dev_get_drvdata(&dssdev->dev);
	int r;
	enum omap_dsi_index lcd_ix;

	lcd_ix = DSI1;

	dev_info(&dssdev->dev, "update %d, %d, %d x %d\n", x, y, w, h);

	mutex_lock(&d2d->lock);

	dsi_bus_lock(lcd_ix);

	if (!d2d->enabled) {
		r = 0;
		goto err;
	}

	r = omap_dsi_prepare_update(dssdev, &x, &y, &w, &h, true);
	if (r)
		goto err;

	/* We use VC(0) for VideoPort Data and VC(1) for commands */
	r = omap_dsi_update(dssdev, 0, x, y, w, h, s6e8aa0_framedone_cb, dssdev);
	if (r)
		goto err;

	dsi_bus_unlock(lcd_ix);
	/* note: no bus_unlock here. unlock is in framedone_cb */
	mutex_unlock(&d2d->lock);
	return 0;
err:
	dsi_bus_unlock(lcd_ix);
	mutex_unlock(&d2d->lock);
	return r;
}

static int s6e8aa0_sync(struct omap_dss_device *dssdev)
{
	/* TODO? */
	return 0;
}

static int s6e8aa0_set_update_mode(struct omap_dss_device *dssdev,
			       enum omap_dss_update_mode mode)
{
//	struct s6e8aa0_data *d2d = dev_get_drvdata(&dssdev->dev);

	if (d2d->force_update) {
		if (mode != OMAP_DSS_UPDATE_AUTO)
			return -EINVAL;
	} else {
		if (mode != OMAP_DSS_UPDATE_MANUAL)
			return -EINVAL;
	}

	return 0;
}

static enum omap_dss_update_mode s6e8aa0_get_update_mode(struct omap_dss_device
						     *dssdev)
{
//	struct s6e8aa0_data *d2d = dev_get_drvdata(&dssdev->dev);

	if (d2d->force_update)
		return OMAP_DSS_UPDATE_AUTO;
	else
		return OMAP_DSS_UPDATE_MANUAL;
}

/**
 * s6e8aa0_panel_get_hs_mode_timing - sets timing for panel
 * @dssdev: Pointer to the dss device
 *
 * Timing values for the transition from low-power to high-speed mode.
 */
static void s6e8aa0_panel_get_hs_mode_timing(struct omap_dss_device *dssdev)
{
	/* The following time values are required for MIPI timing
	* per OMAP spec */
	dssdev->phy.dsi.hs_timing.ths_prepare = 17;
	dssdev->phy.dsi.hs_timing.ths_prepare_ths_zero = 39;
	dssdev->phy.dsi.hs_timing.ths_trail = 18;
	dssdev->phy.dsi.hs_timing.ths_exit = 31;
	dssdev->phy.dsi.hs_timing.tlpx_half = 6;
	dssdev->phy.dsi.hs_timing.tclk_trail = 15;
	dssdev->phy.dsi.hs_timing.tclk_prepare = 14;
	dssdev->phy.dsi.hs_timing.tclk_zero = 56;

	dev_info(&dssdev->dev, "Programmed values: ths_prepare=%u "
		"ths_prepare_ths_zero=%u\n ths_trail=%u ths_exit=%u "
		"tlpx_half=%u\n tclk_trail =%u tclk_prepare=%u tclk_zero=%u\n",
		dssdev->phy.dsi.hs_timing.ths_prepare,
		dssdev->phy.dsi.hs_timing.ths_prepare_ths_zero,
		dssdev->phy.dsi.hs_timing.ths_trail,
		dssdev->phy.dsi.hs_timing.ths_exit,
		dssdev->phy.dsi.hs_timing.tlpx_half,
		dssdev->phy.dsi.hs_timing.tclk_trail,
		dssdev->phy.dsi.hs_timing.tclk_prepare,
		dssdev->phy.dsi.hs_timing.tclk_zero);
}


#ifdef CONFIG_PM
static int s6e8aa0_resume(struct omap_dss_device *dssdev)
{
	dev_info(&dssdev->dev, "resume.\n");
	int ret = 0;
	if (dssdev->state != OMAP_DSS_DISPLAY_SUSPENDED)
		return -EINVAL;

	if (dssdev->platform_enable)
			ret = dssdev->platform_enable(dssdev);
	
	ret = s6e8aa0_start(dssdev);
	
	update_brightness();
	
	dev_info(&dssdev->dev, "*** %s ***\n", __func__);
	return ret;
}

static int s6e8aa0_suspend(struct omap_dss_device *dssdev)
{
	dev_info(&dssdev->dev, "suspend.\n");
	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE)
		return -EINVAL;

	s6e8aa0_stop(dssdev);

	if (dssdev->platform_disable)
	        dssdev->platform_disable(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;
	
	dev_info(&dssdev->dev, "*** %s ***\n", __func__);
	return 0;
}
#endif


static struct omap_dss_driver s6e8aa0_driver = {
	.probe		= s6e8aa0_probe,
	.remove		= s6e8aa0_remove,

	.enable		= s6e8aa0_enable,
	.disable	= s6e8aa0_disable,
#ifdef CONFIG_PM	
	.suspend	= s6e8aa0_suspend,
	.resume		= s6e8aa0_resume,
#endif
	.set_update_mode = s6e8aa0_set_update_mode,
	.get_update_mode = s6e8aa0_get_update_mode,

	.hs_mode_timing = s6e8aa0_panel_get_hs_mode_timing,

	.update		= s6e8aa0_update,
	.sync		= s6e8aa0_sync,

	.get_resolution	= s6e8aa0_get_resolution,
	.get_recommended_bpp = omapdss_default_get_recommended_bpp,

	.enable_te	= s6e8aa0_enable_te,
	.set_rotate	= s6e8aa0_rotate,
	.get_rotate	= s6e8aa0_get_rotate,
	.set_mirror	= s6e8aa0_mirror,
	.get_mirror	= s6e8aa0_get_mirror,

	.get_timings	= s6e8aa0_get_timings,
	.set_timings	= s6e8aa0_set_timings,
	.check_timings	= s6e8aa0_check_timings,

	.driver         = {
		.name   = "s6e8aa0_panel",
		.owner  = THIS_MODULE,
	},
};


static int __init s6e8aa0_init(void)
{
	omap_dss_register_driver(&s6e8aa0_driver);
	return 0;
}

static void __exit s6e8aa0_exit(void)
{
	omap_dss_unregister_driver(&s6e8aa0_driver);
}

module_init(s6e8aa0_init);
module_exit(s6e8aa0_exit);

MODULE_DESCRIPTION("MIPI-DSI based S6E8AA0 Panel Driver");
MODULE_LICENSE("GPL");
