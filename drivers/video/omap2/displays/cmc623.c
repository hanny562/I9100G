/*
 * cmc623.c
 * CMC623 -Image Converter
 *
 * Copyright (C) 2010 Samsung Elcetronic
 * Author: Shankar Bandal <shankar.b@samsung.com>
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




#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include "cmc623.h"


#define MAX_LEVEL 	1600
#define USE_IMACONV_CMC623 1

#define CMC623_I2C_SPEED_KHZ  400
#define CMC623_DEVICE_ADDR  (0x38 << 1)

static int current_gamma_level = MAX_LEVEL;

#define CMC623_DRIVER_NAME	"cmc623_i2c_driver"

#define CMC623_IMA_nRST_GPIO                  62
#define CMC623_IMA_BYPASS_GPIO                55
#define CMC623_IMA_PWR_EN_GPIO                36
#define CMC623_IMA_SLEEP_GPIO                 35

static unsigned cmc623_ima_nrst_gpio;
static unsigned cmc623_ima_bypass_gpio;
static unsigned cmc623_ima_pwr_en_gpio;
static unsigned cmc623_ima_sleep_gpio;


struct cmc623_context {
        struct i2c_client    *client;
        struct mutex xfer_lock;
        } *sd;

struct cmc623_state_type{
	unsigned int cabc_enabled;
	unsigned int brightness;
	unsigned int suspended;
	int white;
	int black;
	int saturation;
};

static struct cmc623_state_type cmc623_state = { 
	.cabc_enabled = -1,
	.brightness = 32,
	.suspended = -1,
	.white = 0,
	.black = 0,
	.saturation = 0,
};

typedef struct {
	u16 addr;
	u16 data;
} mDNIe_data_type;

typedef enum
{
	mDNIe_UI_MODE,
	mDNIe_VIDEO_MODE,
	mDNIe_VIDEO_WARM_MODE,
	mDNIe_VIDEO_COLD_MODE,
	mDNIe_CAMERA_MODE,
	mDNIe_NAVI,
	mDNIe_DMB_MODE,
	mDNIe_VT_MODE,
}Lcd_mDNIe_UI;

typedef enum
{
	mode_type_CABC_none,
	mode_type_CABC_on,
	mode_type_CABC_off,
}mDNIe_mode_CABC_type;

#if 0 /* [SHANKAR] Unused variable */

static Lcd_CMC623_UI_mode current_cmc623_UI = CMC623_UI_MODE; // mDNIe Set Status Checking Value.
static int current_cmc623_OutDoor_OnOff = -1;
static int current_cmc623_CABC_OnOff = -1;

static int setting_first = -1;
static int cmc623_bypass_mode = -1;
static int current_autobrightness_enable = -1;
static int cmc623_current_region_enable = -1;
#endif /* [SHANKAR] Unused variable */


mDNIe_mode_CABC_type cmc623_cabc_mode[]=
{
	mode_type_CABC_none,		// UI
	mode_type_CABC_on,		// Video
	mode_type_CABC_on,		// Video warm
	mode_type_CABC_on,		// Video cold
	mode_type_CABC_off, 	// Camera
	mode_type_CABC_none,		// Navi
};

#define NUM_ITEM_POWER_LUT	9
#define NUM_POWER_LUT	2

static int current_power_lut_num = 0;

unsigned char cmc623_Power_LUT[NUM_POWER_LUT][NUM_ITEM_POWER_LUT]={

	{ 0x3e, 0x43, 0x3a, 0x48, 0x3e, 0x3b, 0x36, 0x33, 0x3b },
	{ 0x3e, 0x43, 0x3a, 0x48, 0x3e, 0x3b, 0x36, 0x33, 0x3b },

};

#ifdef UNUSED_CMC623_SETTINGS /* [SHANKAR] commented to avoid compiler warnings */

/**
 * functions for I2C transactions
 */
static int cmc623_i2c_write8(
    u8 addr,
    u8 data)
{
	struct i2c_msg msg; 
	unsigned char wb[2];        
	//int retry = 0;
	int err;

	//printk("[SHANKAR] %s [%d] ENTER \n", __func__, __LINE__);

	if (!sd->client->adapter)
	{                
		return -ENODEV;        
	}

	mutex_lock(&sd->xfer_lock);
    wb[0] = addr & 0xFF;   
    wb[1] = data & 0xFF;  

    msg.addr = sd->client->addr;
	msg.flags = 0;
	msg.len = 2;
	msg.buf = wb;

	err = i2c_transfer(sd->client->adapter, &msg, 1);
	mutex_unlock(&sd->xfer_lock);

	//printk("[SHANKAR] %s [%d] EXIT \n", __func__, __LINE__);
    if (err > 0)
        return 0;
    else
    {
		//printk("[SHANKAR] %s [%d] I2C write faild \n", __func__, __LINE__);
		return -1;
    }

}

#endif  /* [SHANKAR] commented to avoid compiler warnings */
static int cmc623_i2c_write16(
    u8 addr,
    u32 data)
{

		struct i2c_msg msg; 
		unsigned char wb[4];		  
		//int retry = 0;
		int err;
	
		//printk("[SHANKAR] %s [%d] ENTER \n", __func__, __LINE__);
	
		if (!sd->client->adapter)
		{				 
			return -ENODEV; 	   
		}
	
		mutex_lock(&sd->xfer_lock);
		wb[0] = addr & 0xFF;	
		wb[1] = (u8)((data >>  8) & 0xFF);
		wb[2] = (u8)(data & 0xFF);
	
		msg.addr = sd->client->addr;
		msg.flags = 0;
		msg.len = 3;
		msg.buf = wb;
	
		err = i2c_transfer(sd->client->adapter, &msg, 1);
		mutex_unlock(&sd->xfer_lock);
	
		//printk("[SHANKAR] %s [%d] EXIT \n", __func__, __LINE__);
		if (err > 0)
			return 0;
		else
		{
			//printk("[SHANKAR] %s [%d] I2C write faild \n", __func__, __LINE__);
			return -1;
		}

    
}

#ifdef UNUSED_CMC623_SETTINGS /* [SHANKAR] commented to avoid compiler warnings */

static int cmc623_i2c_read8(
    u8 addr,
    u32 *data)
{

	struct i2c_msg msg[2]; 
	unsigned char in_data[1];		  
	//int retry = 0;
	int err;
	
	//printk("[SHANKAR] %s [%d] ENTER \n", __func__, __LINE__);
	
	if (!sd->client->adapter)
	{				 
		return -ENODEV; 	   
	}
	
	mutex_lock(&sd->xfer_lock);
	in_data[0]= addr & 0xFF;
	msg[0].addr = sd->client->addr;
	msg[0].len = 1;
	msg[0].flags = 0;
	msg[0].buf = in_data;
	
	msg[1].addr = sd->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = data;

	
	err = i2c_transfer(sd->client->adapter, msg, 2);
	mutex_unlock(&sd->xfer_lock);
	if (err > 0)
        return 0;
    else
    {
		//printk("[SHANKAR] %s [%d] I2C write faild \n", __func__, __LINE__);
		return -1;
    }
    
}

static int cmc623_i2c_read16(
    u8 addr,
    u32 *data)
{
	struct i2c_msg msg[2]; 
	unsigned char in_data[1];		  
	//int retry = 0;
	int err;
	
	//printk("[SHANKAR] %s [%d] ENTER \n", __func__, __LINE__);
	
	if (!sd->client->adapter)
	{				 
		return -ENODEV; 	   
	}
	
	mutex_lock(&sd->xfer_lock);
	in_data[0]= addr & 0xFF;
	msg[0].addr = sd->client->addr;
	msg[0].len = 1;
	msg[0].flags = 0;
	msg[0].buf = in_data;
	
	msg[1].addr = sd->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 2;
	msg[1].buf = data;

	
	err = i2c_transfer(sd->client->adapter, msg, 2);
	mutex_unlock(&sd->xfer_lock);
	if (err > 0)
        return 0;
    else
    {
		//printk("[SHANKAR] %s [%d] I2C write faild \n", __func__, __LINE__);
		return -1;
    }

    
}

#endif  /* [SHANKAR] commented to avoid compiler warnings */
#define CMC623_INITSEQ cmc623_init_cabcon2_gamma22

static int cmc623_setup(void)
{
    u32 i=0;
    u32 num = ARRAY_SIZE(CMC623_INITSEQ);

    for (i=0; i<num; i++) {
        if (cmc623_i2c_write16(CMC623_INITSEQ[i].RegAddr, CMC623_INITSEQ[i].data) !=0) 
            return -1;

        if (CMC623_INITSEQ[i].RegAddr == CMC623_REG_SWRESET && 
            CMC623_INITSEQ[i].data == 0xffff)
            mdelay(3);  // 3ms
    }
//	cmc623_state.cabc_enabled = 0;
    return 0;
}


// value: 0 ~ 100
static void cmc623_cabc_pwm_brightness_reg(int value)
{
	u32 reg;
	unsigned char * p_plut;
#if 0
	if(!p_cmc623_data)
		{
		//printk(KERN_ERR "%s cmc623 is not initialized\n", __func__);
		return;
		}
#endif
	p_plut = cmc623_Power_LUT[current_power_lut_num];

	cmc623_i2c_write16(0x76,(p_plut[0] * value / 100) << 8 | (p_plut[1] * value / 100));	//PowerLUT
	cmc623_i2c_write16(0x77,(p_plut[2] * value / 100) << 8 | (p_plut[3] * value / 100));	//PowerLUT
	cmc623_i2c_write16(0x78,(p_plut[4] * value / 100) << 8 | (p_plut[5] * value / 100));	//PowerLUT
	cmc623_i2c_write16(0x79,(p_plut[6] * value / 100) << 8 | (p_plut[7] * value / 100));	//PowerLUT
	cmc623_i2c_write16(0x7a,(p_plut[8] * value / 100) << 8);	//PowerLUT

	reg = 0x5000 | (value<<4);

	cmc623_i2c_write16(0xB4, reg);			//pwn duty
}

// value: 0 ~ 100
static void cmc623_cabc_pwm_brightness(int value)
{
	unsigned char * p_plut;
#if 0
	if(!p_cmc623_data)
		{
		//printk(KERN_ERR "%s cmc623 is not initialized\n", __func__);
		return;
		}
#endif
	p_plut = cmc623_Power_LUT[current_power_lut_num];

	cmc623_i2c_write16(0x00,0x0000);	//BANK 0

	cmc623_cabc_pwm_brightness_reg(value);

	cmc623_i2c_write16(0x28,0x0000);
}
#if 0
// value: 0 ~ 100
// This should be used only for special purpose as resume
static void cmc623_manual_pwm_brightness_reg_nosync(int value)
{
	u32 reg;
#if 0
	if(!p_cmc623_data)
		{
		//printk(KERN_ERR "%s cmc623 is not initialized\n", __func__);
		return;
		}
#endif
	reg = 0xC000 | (value);

	cmc623_i2c_write16(0xB4, reg);			//pwn duty
}

// value: 0 ~ 100
static void cmc623_manual_pwm_brightness_reg(int value)
{
	u32 reg;
#if 0
	if(!p_cmc623_data)
		{
		//printk(KERN_ERR "%s cmc623 is not initialized\n", __func__);
		return;
		}
#endif
	reg = 0x4000 | (value<<4);

	cmc623_i2c_write16(0xB4, reg);			//pwn duty
}
#endif
// value: 0 ~ 100
static void cmc623_manual_pwm_brightness(int value)
{
	u32 reg;
#if 0
	if(!p_cmc623_data)
		{
		//printk(KERN_ERR "%s cmc623 is not initialized\n", __func__);
		return;
		}
#endif
	reg = 0x4000 | (value<<4);

	cmc623_i2c_write16(0x00, 0x0000);		//bank0
	cmc623_i2c_write16(0xB4, reg);			//pwn duty
	cmc623_i2c_write16(0x28, 0x0000);

}

// value: 0 ~ 1600
static  void cmc623_tune_pwm_brightness(int value)
{
	u32 data;


	if(value<0)
		data = 0;
	else if(value>1600)
		data = 1600;
	else
		data = value;
#if 0		
	if(data == 1280 && current_autobrightness_enable)
	{//outdoor mode on
		current_cmc623_OutDoor_OnOff = TRUE;
		cmc623_Set_Mode(current_cmc623_UI, current_cmc623_CABC_OnOff);
	}
	else if (current_cmc623_OutDoor_OnOff == TRUE && data < 1280)
	{//outdoor mode off
		current_cmc623_OutDoor_OnOff = FALSE;
		cmc623_Set_Mode(current_cmc623_UI, current_cmc623_CABC_OnOff);
	}
#endif
	data >>= 4;

	cmc623_state.brightness = data;
	if(cmc623_state.cabc_enabled == 0)
	{
		cmc623_cabc_pwm_brightness(data);
	}
	else
	{
		cmc623_manual_pwm_brightness(data);
	}
}

void cmc623_set_backlight_intensity(int intensity)
{

// brightness tuning
#define MAX_BRIGHTNESS_LEVEL 255
#define MID_BRIGHTNESS_LEVEL 140
#define LOW_BRIGHTNESS_LEVEL 30
#define DIM_BACKLIGHT_LEVEL 20

#define MAX_BACKLIGHT_VALUE 1280 //80
#define MID_BACKLIGHT_VALUE 512 //32
#define LOW_BACKLIGHT_VALUE 128 //5
#define DIM_BACKLIGHT_VALUE 80  // 3	

	int tune_level;

	//printk( "%s : BL brightness level = %d\n", __func__, intensity);
	// brightness tuning
	if(intensity >= MID_BRIGHTNESS_LEVEL)
			tune_level = (intensity - MID_BRIGHTNESS_LEVEL) * (MAX_BACKLIGHT_VALUE-MID_BACKLIGHT_VALUE) / (MAX_BRIGHTNESS_LEVEL-MID_BRIGHTNESS_LEVEL) + MID_BACKLIGHT_VALUE;
	else if(intensity >= LOW_BRIGHTNESS_LEVEL)
			tune_level = (intensity - LOW_BRIGHTNESS_LEVEL) * (MID_BACKLIGHT_VALUE-LOW_BACKLIGHT_VALUE) / (MID_BRIGHTNESS_LEVEL-LOW_BRIGHTNESS_LEVEL) + LOW_BACKLIGHT_VALUE;
	else if(intensity >= DIM_BACKLIGHT_LEVEL)
			tune_level = (intensity - DIM_BACKLIGHT_LEVEL) * (LOW_BACKLIGHT_VALUE-DIM_BACKLIGHT_VALUE) / (LOW_BRIGHTNESS_LEVEL-DIM_BACKLIGHT_LEVEL) + DIM_BACKLIGHT_VALUE;
	else if(intensity > 0)
			tune_level = (intensity) * (DIM_BACKLIGHT_VALUE) / (DIM_BACKLIGHT_LEVEL);
	else
			tune_level = intensity;
	cmc623_tune_pwm_brightness(intensity);

	current_gamma_level = intensity;
}
EXPORT_SYMBOL(cmc623_set_backlight_intensity);

int cmc623_init(void)
{
	int err=0;
	//printk("[SHANKAR] %s [%d] ENTER \n", __func__, __LINE__);
	mutex_init(&sd->xfer_lock);
	
	cmc623_ima_nrst_gpio = CMC623_IMA_nRST_GPIO;
	cmc623_ima_bypass_gpio = CMC623_IMA_BYPASS_GPIO;
	cmc623_ima_pwr_en_gpio = CMC623_IMA_PWR_EN_GPIO;
	cmc623_ima_sleep_gpio = CMC623_IMA_SLEEP_GPIO;
	
	err = gpio_request(cmc623_ima_nrst_gpio, "CMC623 IMA_nRST GPIO");
	if (err) {
	        //printk(KERN_ERR "failed to get CMC623 IMA_nRST GPIO\n");
	        goto err0;
	}

	err = gpio_request(cmc623_ima_bypass_gpio, "CMC623 IMA_BYPASS GPIO");
	if (err) {
	        //printk(KERN_ERR "failed to get CMC623 IMA_BYPASS GPIO\n");
	        goto err1;
	}

	err = gpio_request(cmc623_ima_pwr_en_gpio, "CMC623 IMA_PWR_EN GPIO");
	if (err) {
	        //printk(KERN_ERR "failed to get CMC623 IMA_PWR_EN GPIO\n");
	        goto err2;
	}

	err = gpio_request(cmc623_ima_sleep_gpio, "CMC623 IMA_SLEEP GPIO");
	if (err) {
	        //printk(KERN_ERR "failed to get CMC623 IMA_SLEEP GPIO\n");
	        goto err3;
	}	

	/* enable image converter */
	gpio_direction_output(cmc623_ima_pwr_en_gpio, 1); // set FAIL_SAFEB to HIGH

	gpio_direction_output(cmc623_ima_bypass_gpio, 1); // set BYPASSB to HIGH

	gpio_direction_output(cmc623_ima_sleep_gpio, 1);  // set SLEEPB to HIGH
	udelay(100);
	
	gpio_direction_output(cmc623_ima_nrst_gpio, 1);   // set RESETB to HIGH
	udelay(1000);
	
	gpio_direction_output(cmc623_ima_nrst_gpio, 0);   // set RESETB to LOW
	mdelay(4);
	
	gpio_direction_output(cmc623_ima_nrst_gpio, 1);   // set RESETB to HIGH after 4ms

	
#if USE_IMACONV_CMC623  // setup CMC623 thru I2C
    mdelay(4);  // 4ms
    if (cmc623_setup()<0)
	{
		//printk("[SHANKAR] %s [%d] CMC623 setup failed \n ", __func__, __LINE__);
		goto err3;
	}
#else  // set by-pass mode
    mdelay(16);  // 16ms
	gpio_direction_output(cmc623_ima_bypass_gpio, 0); // set BYPASSB to LOW
#endif

	//Set default Back Light Intensity
	cmc623_set_backlight_intensity(200);

	//printk("[SHANKAR] %s [%d] EXIT \n", __func__, __LINE__);
	return 0;

err3:
        gpio_free(cmc623_ima_sleep_gpio);
err2:
		gpio_free(cmc623_ima_pwr_en_gpio);

err1:
        gpio_free(cmc623_ima_bypass_gpio);
err0:
		gpio_free(cmc623_ima_nrst_gpio);
		return -1;

}
EXPORT_SYMBOL(cmc623_init);

void cmc623_sleep(void)
{

	gpio_direction_output(cmc623_ima_sleep_gpio, 0);  // set SLEEPB to LOW
	udelay(100);
	gpio_direction_output(cmc623_ima_pwr_en_gpio, 0); // set FAIL_SAFEB to LOW
	udelay(1000);
	
}
EXPORT_SYMBOL(cmc623_sleep);

static int cmc623_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
        //printk("[SHANKAR] %s [%d] ENTER \n", __func__, __LINE__);
        sd = kzalloc(sizeof(struct cmc623_context), GFP_KERNEL);
        if (sd == NULL)
                return -ENOMEM;
        i2c_set_clientdata(client, sd);
        sd->client = client;
		//printk("[SHANKAR] %s [%d] EXIT \n", __func__, __LINE__);
        return 0;
}
static int cmc623_remove(struct i2c_client *client, const struct i2c_device_id *id)
{
        //printk("[SHANKAR] %s [%d] ENTER \n", __func__, __LINE__);

		//printk("[SHANKAR] %s [%d] EXIT \n", __func__, __LINE__);
        return 0;
}


static const struct i2c_device_id cmc623_id[] = {
        { CMC623_DRIVER_NAME, 0 },
        { },
};


static struct i2c_driver cmc623_i2c_driver = {
        .driver = {
                .name           = CMC623_DRIVER_NAME,
        },
        .probe          = cmc623_probe,
        .remove         = __exit_p(cmc623_remove),
        .id_table       = cmc623_id,

};

static int __init cmc623_i2c_init(void)
{
        int r;
        r = i2c_add_driver(&cmc623_i2c_driver);
        if (r < 0) {
                printk(KERN_WARNING CMC623_DRIVER_NAME
                " driver registration failed\n");
                return r;
        }
        
        return 0;
}


static void __exit cmc623_i2c_exit(void)
{
        i2c_del_driver(&cmc623_i2c_driver);
       
}


module_init(cmc623_i2c_init);
module_exit(cmc623_i2c_exit);



MODULE_DESCRIPTION("Samsung cmc623 image converter driver");
MODULE_AUTHOR("Shankar Bandal<shankar.b@samsung.com>");

MODULE_LICENSE("GPL");

