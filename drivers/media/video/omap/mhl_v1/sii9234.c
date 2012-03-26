/***************************************************************************
* SiI9244  MHL Transmitter Driver
*
* Copyright (C) (2011, Silicon Image Inc)
*
* This program is free software; you can redistribute it and/or modify
*
* it under the terms of the GNU General Public License as published by
*
* the Free Software Foundation version 2.
*
* This program is distributed as it is WITHOUT ANY WARRANTY of any
*
* kind, whether express or implied; without even the implied warranty
*
* of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*
* GNU General Public License for more details.
*****************************************************************************/

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/gpio.h>

#include <asm/irq.h>
#include <linux/delay.h>

#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/fcntl.h>
#include <asm/uaccess.h>
#include <linux/regulator/consumer.h>
#include <linux/mhl-sii9234.h>

#include "sii9234_driver.h"
#include "Common_Def.h"
#include <plat/i2c-omap-gpio.h>

#define SUBJECT "MHL_DRIVER"

#define SII_DEV_DBG(format,...)\
	printk ("[ "SUBJECT " (%s) ] " format "\n", __func__, ## __VA_ARGS__)


struct i2c_driver sii9234_i2c_driver;
struct i2c_client *sii9234_i2c_client = NULL;

struct i2c_driver sii9234a_i2c_driver;
struct i2c_client *sii9234a_i2c_client = NULL;

struct i2c_driver sii9234b_i2c_driver;
struct i2c_client *sii9234b_i2c_client = NULL;

struct i2c_driver sii9234c_i2c_driver;
struct i2c_client *sii9234c_i2c_client = NULL;
struct mhl_dev *g_mhl_dev ;

unsigned int mhl_irq;

extern bool sii9234_init(void);

/**Callback for Enabling or Disabling Interrupts of FSA9480
while MHL has been shutdown or restarted respectively.
*/
extern void FSA9480_EnableIntrruptByMHL(bool _bDo);

static struct i2c_device_id sii9234_id[] = {
	{"SII9234", 0},
	{}
};
static struct i2c_device_id sii9234a_id[] = {
        {"SII9234A", 0},
        {}
};
static struct i2c_device_id sii9234b_id[] = {
        {"SII9234B", 0},
        {}
};
static struct i2c_device_id sii9234c_id[] = {
        {"SII9234C", 0},
        {}
};

//MODULE_DEVICE_TABLE(i2c,sii9234_id);

static int mhl_onoff_status = 0;
static int mhl_i2c_init = 0;


struct sii9234_state {
	struct i2c_client *client;
};
void sii9234_cfg_power(bool on);

static void sii9234_cfg_gpio(void);

irqreturn_t mhl_int_irq_handler(int irq, void *dev_id);

irqreturn_t mhl_wake_up_irq_handler(int irq, void *dev_id);

void MHL_On(bool on);

#define MHL_SWITCH_TEST	1

#ifdef MHL_SWITCH_TEST
struct class *sec_mhl;
EXPORT_SYMBOL(sec_mhl);

struct device *mhl_switch;
EXPORT_SYMBOL(mhl_switch);
/**hw_get_rev_gpio()
Used for getting H/W revisions
*/
u8 hw_get_rev_gpio()
{
        u8 hw_rev = 0xFF;

        hw_rev = gpio_get_value(OMAP_GPIO_HW_REV3) << 3;
        hw_rev |= gpio_get_value(OMAP_GPIO_HW_REV2) << 2;
        hw_rev |= gpio_get_value(OMAP_GPIO_HW_REV1) << 1;
        hw_rev |= gpio_get_value(OMAP_GPIO_HW_REV0);
return hw_rev;
}

static ssize_t check_MHL_command(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count;
	int res;

	SII_DEV_DBG( "[MHL]: check_MHL_command\n");
	sii9234_cfg_power(1);
	res = SiI9234_startTPI();
	count = sprintf(buf,"%d\n", res );
	sii9234_cfg_power(0);
	return count;

}

static ssize_t change_switch_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	char *after;
	unsigned long value = simple_strtoul(buf, &after, 10);
	int i;
	int ver =0;
	ver = hw_get_rev_gpio();

	SII_DEV_DBG("[MHL_SWITCH] Change the switch: %ld\n", value);

	if (value == 0) {
		for (i = 0; i <20; i++) {
			SII_DEV_DBG("[MHL] try %d\n", i+1);
			mdelay(500);
		}
		MHL_On(1);
	} else {
		MHL_On(0);
	}


	return size;
}
/*
 * Name         : MHL_On
 *
 * Synopsis     : void MHL_On(bool on) 

 * Arguments    : bool on
 *
 * Description  : Handle for Controlling MHL driver;used by Board file,
		  FSA9480 driver and internal MHL driver(sii9234_driver.c)
		  for controlling MHL operations
 * 
 * Returns      : void
 */

void MHL_On(bool on)
{
	int ver =0;
	if (on == 1 && mhl_onoff_status == 0)
	{
		//Get H/W revision and correct MHL_SEL gpio
		ver = hw_get_rev_gpio();
		SII_DEV_DBG(" - MHL on - hw:%d\n", ver);
		if(ver>=6)
			gpio_direction_output(GPIO_ACC_EN, GPIO_LEVEL_HIGH);
		else
			gpio_direction_output(GPIO_MHL_SEL, GPIO_LEVEL_HIGH);
		//Power-up MHL chip
		sii9234_cfg_power(1);
		mhl_onoff_status = 1;
		//Initialize the MHL driver registers and other initialization operations
		sii9234_init();
		//Enable MHL Interrupt
		enable_irq(mhl_irq);
	}
	else if(on == 0 && mhl_onoff_status == 1)
	{
		ver = hw_get_rev_gpio();
		SII_DEV_DBG(" - MHL off - hw:%d\n",ver);
		mhl_onoff_status = 0;
		//Power-down MHL chip
		sii9234_cfg_power(0);
		//MHL_SEL low
		if(ver>=6)
			gpio_direction_output(GPIO_ACC_EN, GPIO_LEVEL_LOW);
		else
			gpio_direction_output(GPIO_MHL_SEL, GPIO_LEVEL_LOW);
		//Disable MHL Interrupt line
		disable_irq_nosync(mhl_irq);
		//IMPORTANT:Enable Interrupts for FSA9480 after MHL has been shut down or cable plugged out.
		FSA9480_EnableIntrruptByMHL(true);
	}
	else
	{
		SII_DEV_DBG(" onff - %d=>%d  - no change\n", mhl_onoff_status, (int)on);
	}

}
EXPORT_SYMBOL(MHL_On);


static DEVICE_ATTR(mhl_sel, S_IRUGO | S_IWUSR | S_IXOTH, check_MHL_command, change_switch_store);
#endif

static ssize_t MHD_check_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count;
	int res = 0;
	#if 0

	s3c_gpio_setpull(GPIO_MHL_SEL, S3C_GPIO_PULL_UP);	//MHL_SEL

	gpio_set_value(GPIO_MHL_SEL, GPIO_LEVEL_HIGH);


	//TVout_LDO_ctrl(true);

	if(!MHD_HW_IsOn())
	{
		sii9234_tpi_init();
		res = MHD_Read_deviceID();
		MHD_HW_Off();
	}
	else
	{
		sii9234_tpi_init();
		res = MHD_Read_deviceID();
	}

	I2C_WriteByte(0x72, 0xA5, 0xE1);
	res = 0;
	res = I2C_ReadByte(0x72, 0xA5);

	printk(KERN_ERR "A5 res %x",res);

	res = 0;
	res = I2C_ReadByte(0x72, 0x1B);

	printk(KERN_ERR "Device ID res %x",res);

	res = 0;
	res = I2C_ReadByte(0x72, 0x1C);

	printk(KERN_ERR "Device Rev ID res %x",res);

	res = 0;
	res = I2C_ReadByte(0x72, 0x1D);

	printk(KERN_ERR "Device Reserved ID res %x",res);

	printk(KERN_ERR "\n####HDMI_EN1 %x MHL_RST %x GPIO_MHL_SEL %x\n",gpio_get_value(GPIO_HDMI_EN),gpio_get_value(GPIO_MHL_RST),gpio_get_value(GPIO_MHL_SEL));

	res = I2C_ReadByte(0x7A, 0x3D);

	res = I2C_ReadByte(0x7A, 0xFF);

	s3c_gpio_setpull(GPIO_MHL_SEL, S3C_GPIO_PULL_NONE);	//MHL_SEL

	gpio_set_value(GPIO_MHL_SEL, GPIO_LEVEL_LOW);
#endif
	count = sprintf(buf,"%d\n", res );
	//TVout_LDO_ctrl(false);
	return count;
}

static ssize_t MHD_check_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	SII_DEV_DBG("input data --> %s\n", buf);

	return size;
}

static DEVICE_ATTR(MHD_file, S_IRUGO | S_IWUSR , MHD_check_read, MHD_check_write);
//static DEVICE_ATTR(MHD_file, S_IRUGO | S_IWUSR | S_IWOTH | S_IXOTH, MHD_check_read, MHD_check_write);
void sii9234_interrupt_event_work( void);
extern void sii9234_interrupt_event(void);

/*
 * Name         : get_sii9234_client
 *
 * Synopsis     : struct i2c_client* get_sii9234_client(u8 device_id)

 * Arguments    : u8 device_id
 *
 * Description  : Handle for different sii9234 i2c client
 * 
 * Returns      : struct i2c_client* 
 */
struct i2c_client* get_sii9234_client(u8 device_id)
{

	struct i2c_client* client_ptr;

	if(device_id == 0x72)
		client_ptr = sii9234_i2c_client;
	else if(device_id == 0x7A)
		client_ptr = sii9234a_i2c_client;
	else if(device_id == 0x92)
		client_ptr = sii9234b_i2c_client;
	else if(device_id == 0xC8)
		client_ptr = sii9234c_i2c_client;
	else
		client_ptr = NULL;

	return client_ptr;
}

EXPORT_SYMBOL(get_sii9234_client);

/*
 * Name         : sii9234_i2c_read
 *
 * Synopsis     : u8 sii9234_i2c_read(struct i2c_client *client, u8 reg) 

 * Arguments    : struct i2c_client *client,
		  u8 reg
 *
 * Description  : sii9234 i2c read operation
 * 
 * Returns      : u8 
 */

u8 sii9234_i2c_read(struct i2c_client *client, u8 reg)
{
	u8 ret;

	if(!mhl_i2c_init)
	{
		SII_DEV_DBG("I2C not ready");
		return 0;
	}

	i2c_smbus_write_byte(client, reg);


	ret = i2c_smbus_read_byte(client);

	if (ret < 0) {
		SII_DEV_DBG("i2c read fail");
		return -EIO;
	}

	return ret;

}
EXPORT_SYMBOL(sii9234_i2c_read);

/*
 * Name         : sii9234_i2c_write
 *
 * Synopsis     : u8 sii9234_i2c_write(struct i2c_client *client, u8 reg, u8 data) 

 * Arguments    : struct i2c_client *client,
                  u8 reg,
		  u8 data
 *
 * Description  : sii9234 i2c write operation
 * 
 * Returns      : u8 
 */

int sii9234_i2c_write(struct i2c_client *client, u8 reg, u8 data)
{
	if(!mhl_i2c_init) {
		SII_DEV_DBG("I2C not ready");
		return 0;
	}

	return i2c_smbus_write_byte_data(client, reg, data);
}
EXPORT_SYMBOL(sii9234_i2c_write);

/*Interrupt Handler work function*/
void sii9234_interrupt_event_work()
{

	SII_DEV_DBG( "sii9234_interrupt_event_work() is called\n");

	sii9234_interrupt_event();

}

/*Handler for MHL_INT interrupt*/
irqreturn_t mhl_int_irq_handler(int irq, void *dev_id)
{
	struct mhl_dev *mhl_dev = dev_id;

	SII_DEV_DBG( "mhl_int_irq_handler() is called\n");

	queue_work(mhl_dev->sii9234_wq, &mhl_dev->sii9234_int_work);

	return IRQ_HANDLED;
}

/*Handler for MHL_WAKE_UP interrupt*/
irqreturn_t mhl_wake_up_irq_handler(int irq, void *dev_id)
{
	struct mhl_dev *mhl_dev = dev_id;

	SII_DEV_DBG( "mhl_wake_up_irq_handler() is called\n");

	queue_work(mhl_dev->sii9234_wq, &mhl_dev->sii9234_int_work);

	return IRQ_HANDLED;
}

static int sii9234_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	/* int retval; */

	struct sii9234_state *state;

     struct device *mhl_dev;

	state = kzalloc(sizeof(struct sii9234_state), GFP_KERNEL);
	if (state == NULL) {
		pr_err("failed to allocate memory \n");
		return -ENOMEM;
	}

	state->client = client;
	i2c_set_clientdata(client, state);
	SII_DEV_DBG( "SII9234 attach success!!!\n");
	mhl_i2c_init = 1;
	sii9234_i2c_client = client;


	return 0;

}



static int __devexit sii9234_i2c_remove(struct i2c_client *client)
{
	struct sii9234_state *state = i2c_get_clientdata(client);
	kfree(state);

	return 0;
}
static int sii9234a_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct sii9234_state *state;

	state = kzalloc(sizeof(struct sii9234_state), GFP_KERNEL);
	if (state == NULL) {
		pr_err("failed to allocate memory \n");
		return -ENOMEM;
	}

	state->client = client;
	i2c_set_clientdata(client, state);
        SII_DEV_DBG( "SII9234A attach success!!!\n");
        mhl_i2c_init = 1;
	sii9234a_i2c_client = client;
        return 0;

}
static int __devexit sii9234a_i2c_remove(struct i2c_client *client)
{
	struct sii9234_state *state = i2c_get_clientdata(client);
	kfree(state);
        return 0;
}

static int sii9234b_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct sii9234_state *state;

	state = kzalloc(sizeof(struct sii9234_state), GFP_KERNEL);
	if (state == NULL) {
		pr_err("failed to allocate memory \n");
		return -ENOMEM;
        }

	state->client = client;
	i2c_set_clientdata(client, state);

	/* rest of the initialisation goes here. */

	SII_DEV_DBG( "SII9234B attach success!!!\n");

	sii9234b_i2c_client = client;


	return 0;

}
static int __devexit sii9234b_i2c_remove(struct i2c_client *client)
{
        struct sii9234_state *state = i2c_get_clientdata(client);
        kfree(state);
        return 0;
}
static int sii9234c_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct sii9234_state *state;

	state = kzalloc(sizeof(struct sii9234_state), GFP_KERNEL);
	if (state == NULL) {
		pr_err("failed to allocate memory \n");
		return -ENOMEM;
	}

	state->client = client;
	i2c_set_clientdata(client, state);

	/* rest of the initialisation goes here. */

	pr_err("SII9234C attach success!!!\n");

	sii9234c_i2c_client = client;

        return 0;

}




static int __devexit sii9234c_i2c_remove(struct i2c_client *client)
{
        struct sii9234_state *state = i2c_get_clientdata(client);
        kfree(state);
        return 0;
}


struct i2c_driver sii9234_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "SII9234",
	},
	.id_table	= sii9234_id,
	.probe	= sii9234_i2c_probe,
	.remove	= __devexit_p(sii9234_i2c_remove),
	.command = NULL,
};

struct i2c_driver sii9234a_i2c_driver = {
        .driver = {
                .owner  = THIS_MODULE,
                .name   = "SII9234A",
        },
        .id_table       = sii9234a_id,
        .probe  = sii9234a_i2c_probe,
	.remove	= __devexit_p(sii9234a_i2c_remove),
        .command = NULL,
};

struct i2c_driver sii9234b_i2c_driver = {
        .driver = {
                .owner  = THIS_MODULE,
                .name   = "SII9234B",
        },
        .id_table       = sii9234b_id,
        .probe  = sii9234b_i2c_probe,
	.remove	= __devexit_p(sii9234b_i2c_remove),
        .command = NULL,
};

struct i2c_driver sii9234c_i2c_driver = {
        .driver = {
                .owner  = THIS_MODULE,
                .name   = "SII9234C",
        },
        .id_table       = sii9234c_id,
        .probe  = sii9234c_i2c_probe,
	.remove	= __devexit_p(sii9234c_i2c_remove),
        .command = NULL,
};

#if 0 // xmoondash
void sii9234_cfg_power(bool on)
{

	s3c_gpio_cfgpin(GPIO_HDMI_EN,S3C_GPIO_OUTPUT);		//HDMI_EN1 for LDO3,4 and BUCK MAX8893	GPJ1(2)

	if(on)
	{
	//	max8893_ldo_enable_direct(MAX8893_LDO3);//VCC_1.8V_MHL LD04 MAX8893	Turn on
// xmonodash ::  enable  LDO3
//
		s3c_gpio_cfgpin(GPIO_MHL_RST,S3C_GPIO_OUTPUT);		//MHL_RST	MP0(4)
		s3c_gpio_setpull(GPIO_MHL_RST, S3C_GPIO_PULL_NONE);


		s3c_gpio_setpull(GPIO_AP_SCL_18V, S3C_GPIO_PULL_DOWN);
		gpio_set_value(GPIO_MHL_RST, GPIO_LEVEL_LOW);
		mdelay(200);
		gpio_set_value(GPIO_MHL_RST, GPIO_LEVEL_HIGH);
		s3c_gpio_setpull(GPIO_AP_SCL_18V, S3C_GPIO_PULL_NONE);

	}

	else
	{

//		max8893_ldo_disable_direct(MAX8893_LDO3);//VCC_1.8V_MHL LD04 MAX8893	Turn off
// xmoondash :: disable LDO3
		gpio_set_value(GPIO_HDMI_EN,GPIO_LEVEL_LOW);

	}

}
#else

extern struct device * fimc_get_active_device(void);

int already_get = 0;
int reg_en = 0;
/*
 * Name         : sii9234_cfg_power
 *
 * Synopsis     : void sii9234_cfg_power(bool on) 
 *
 * Arguments    : bool on
 *
 * Description  : Function for handling power-operations on MHL(powering up or down depending on the function argument)
 * 
 * Returns      : void
 */

extern int mhl_wa_force_det;
void sii9234_cfg_power(bool on)
{
       	unsigned int volatile val;
	SII_DEV_DBG("MHL power on : %d\n",(int)on);
	if(on)
	{
	 #if 1
        // Pulling(internal) up the HPD line
               SII_DEV_DBG("Before (Powering up  MHL) : HPD-padconfig Read value:%x\n",omap_readl(0x4A100098));
               omap_writel(0x000F0118,0x4A100098);
               SII_DEV_DBG("After (Powering up MHL) : HPD-padconfigRead value:%x\n",omap_readl(0x4A100098));
               mdelay(50);
        #endif

	/*Enable HDMI_EN(powering up MHL chip)*/
            if(gpio_direction_output(GPIO_HDMI_EN,GPIO_LEVEL_HIGH))
                SII_DEV_DBG("[MHL_TEST]error in making HDMI_EN High\n");

	    SII_DEV_DBG("[MHL_TEST]HDMI_EN:%d\n",gpio_get_value(GPIO_HDMI_EN));

 #if 0
		 if(gpio_direction_output(GPIO_MHL_RST, GPIO_LEVEL_LOW))
                printk("[MHL_TEST]error in making GPIO_MHL_RST high\n");
	   printk("[MHL_TEST]MHL_RST:%d",gpio_get_value(GPIO_MHL_RST));


 #endif
	
	   mdelay(20);
	/*Assert MHL_RESET for MHL to get reset irrespective of previous state*/
           if(gpio_direction_output(GPIO_MHL_RST, GPIO_LEVEL_HIGH))
                SII_DEV_DBG("[MHL_TEST]error in making GPIO_MHL_RST Low\n");
           SII_DEV_DBG("[MHL_TEST]MHL_RST:%d",gpio_get_value(GPIO_MHL_RST));
	#if 0
	// Pulling(internal) up the HPD line
               SII_DEV_DBG("Before (Powering up  MHL) : HPD-padconfig Read value:%x\n",omap_readl(0x4A100098));
               omap_writel(0x000F0118,0x4A100098);
               SII_DEV_DBG("After (Powering up MHL) : HPD-padconfigRead value:%x\n",omap_readl(0x4A100098));
		mdelay(500);
	#endif
	}
	else
	{
		SII_DEV_DBG("sii9234_cfg_power off GPIO_HDMI_EN low\n");

		if (mhl_wa_force_det == 1)
		{
			val = omap_readl(0x4804630C);
			val = val & 0xFFFF7FFF;
			val = val | 0x00000000;
			omap_writel(val, 0x4804630C);
			val = omap_readl(0x4804630C);
			mhl_wa_force_det = 0;
			printk(KERN_INFO " releasing force connect %x \n", val);
		}

	/*shut down HDMI_EN and MHL_RST GPIOs while shutting down MHL*/
	  	gpio_direction_output(GPIO_HDMI_EN,GPIO_LEVEL_LOW);
          	gpio_direction_output(GPIO_MHL_RST, GPIO_LEVEL_LOW);
          #if 1
		// Pulling(internal) Down the HPD line
               SII_DEV_DBG("Before (Powering down MHL) : HPD-padconfig Read value:%x\n",omap_readl(0x4A100098));
               omap_writel(0x000F0108,0x4A100098);
               SII_DEV_DBG("After (Powering down MHL) : HPD-padconfig Read value:%x\n",omap_readl(0x4A100098));
	#endif
	}
	SII_DEV_DBG( "[MHL]%s : %d \n",__func__,on);

out:
	return;
}
#endif
/*
 * Name         : sii9234_cfg_gpio
 *
 * Synopsis     : static void sii9234_cfg_gpio() 
 *
 * Arguments    : None
 *
 * Description  : Function for initializing and configuring GPIOs/IRQ lines related
		  to MHL Driver
 * 
 * Returns      : void
 */

static void sii9234_cfg_gpio()
{
	set_irq_type(MHL_WAKEUP_IRQ, IRQ_TYPE_EDGE_RISING);
    set_irq_type(MHL_INT_IRQ, IRQ_TYPE_EDGE_RISING);
    gpio_request(GPIO_HDMI_EN,NULL);
    //check for HDMI_EN
    if(gpio_direction_output(GPIO_HDMI_EN,GPIO_LEVEL_LOW))
         SII_DEV_DBG("%s error in making GPIO_HDMI_EN Low\n",__func__);
    gpio_request(GPIO_MHL_RST,NULL);
    //check for MHL_RST
    if(gpio_direction_output(GPIO_MHL_RST,GPIO_LEVEL_LOW))
        SII_DEV_DBG("%s error in making GPIO_MHL_RST Low\n",__func__);
    int ver =0;
    /*check for MHL_SEL;MHL_SEL(gpio for MHL_SEL)
      changes with H/W revision*/
    ver = hw_get_rev_gpio();
    if(ver>=6){
	gpio_request(GPIO_ACC_EN,NULL);
    	if(gpio_direction_output(GPIO_ACC_EN,GPIO_LEVEL_LOW))
        SII_DEV_DBG("%s error in making GPIO_ACC_EN Low\n",__func__);
	}
    else{
    	 gpio_request(GPIO_MHL_SEL,NULL);
    	 if(gpio_direction_output(GPIO_MHL_SEL,GPIO_LEVEL_LOW))
        	SII_DEV_DBG("%s error in making GPIO_MHL_SEL Low\n",__func__);
	}
}

static int mhl_open(struct inode *ip, struct file *fp)
{
	SII_DEV_DBG("[%s] \n",__func__);
	return 0;

}

static int mhl_release(struct inode *ip, struct file *fp)
{

	SII_DEV_DBG( "[%s] \n",__func__);
	return 0;
}


static int mhl_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	SII_DEV_DBG( "[%s] \n",__func__);

#if 0
	byte data;

	switch(cmd)
	{
		case MHL_READ_RCP_DATA:
			data = GetCbusRcpData();
			ResetCbusRcpData();
			put_user(data,(byte *)arg);
			printk(KERN_ERR "MHL_READ_RCP_DATA read");
			break;

		default:
		break;
	}
#endif
	return 0;
}

static struct file_operations mhl_fops = {
	.owner  = THIS_MODULE,
	.open   = mhl_open,
    	.release = mhl_release,
    	.ioctl = mhl_ioctl,
};

static int sii9234_probe(struct platform_device *pdev)
{
	int ret;
	struct mhl_platform_data *mhl_pdata = pdev->dev.platform_data;
	struct mhl_dev *mhl_dev;
/*	unsigned int mhl_irq;*/
	unsigned int mhl_wakeup_irq;

	if (mhl_pdata == NULL) {
		pr_err("MHL probe fail\n");
		return -ENODEV;
	}
	//configure GPIOs required for MHL driver
	sii9234_cfg_gpio();
	ret = i2c_add_driver(&sii9234_i2c_driver);
	if (ret != 0) {
		pr_err("[MHL SII9234] can't add i2c driver\n");
		return ret;
	} else {
		SII_DEV_DBG("[MHL SII9234] add i2c driver\n");
	}

	ret = i2c_add_driver(&sii9234a_i2c_driver);
	if (ret != 0) {
		pr_err("[MHL SII9234A] can't add i2c driver\n");
		goto err_i2c_a_add;
	} else {
		SII_DEV_DBG("[MHL SII9234A] add i2c driver\n");
	}

	ret = i2c_add_driver(&sii9234b_i2c_driver);
	if (ret != 0) {
		pr_err("[MHL SII9234B] can't add i2c driver\n");
		goto err_i2c_b_add;
	} else {
		pr_err("[MHL SII9234B] add i2c driver\n");
	}

	ret = i2c_add_driver(&sii9234c_i2c_driver);
	if (ret != 0) {
		pr_err("[MHL SII9234C] can't add i2c driver\n");
		goto err_i2c_c_add;
	} else {
		pr_err("[MHL SII9234C] add i2c driver\n");
	}

	mhl_dev = kzalloc(sizeof(struct mhl_dev), GFP_KERNEL);
	if (!mhl_dev) {
		ret = -ENOMEM;
		goto err_mem;
	}

	mhl_dev->pdata = mhl_pdata;
	mhl_dev->irq_gpio = mhl_pdata->mhl_int;
	mhl_dev->wake_up_gpio = mhl_pdata->mhl_wake_up;

	INIT_WORK(&mhl_dev->sii9234_int_work, sii9234_interrupt_event_work);
	mhl_dev->sii9234_wq = create_singlethread_workqueue("sii9234_wq");

	mhl_dev->mdev.minor = MISC_DYNAMIC_MINOR;
	mhl_dev->mdev.name = "mhl";
	mhl_dev->mdev.fops = &mhl_fops;

	dev_set_drvdata(&pdev->dev, mhl_dev);

	mhl_dev->process_dev = &pdev->dev;

	ret = misc_register(&mhl_dev->mdev);
	if (ret) {
		pr_err("mhl misc_register failed\n");
		goto err_misc_register;
	}
      g_mhl_dev = mhl_dev;

	mhl_irq = gpio_to_irq(mhl_dev->irq_gpio);
	set_irq_type(mhl_irq, IRQ_TYPE_EDGE_RISING);
	ret = request_threaded_irq(mhl_irq, NULL, mhl_int_irq_handler,
			IRQF_DISABLED, "mhl_int", mhl_dev);
	if (ret) {
		pr_err("unable to request irq mhl_int err:: %d\n", ret);
		goto err_irq_request;
	}
	disable_irq_nosync(mhl_irq);

#if 0
	mhl_wakeup_irq = gpio_to_irq(mhl_dev->wake_up_gpio);
	set_irq_type(mhl_wakeup_irq, IRQ_TYPE_EDGE_RISING);
	ret = request_threaded_irq(mhl_wakeup_irq, NULL,
				mhl_wake_up_irq_handler,
				IRQF_DISABLED,
				"mhl_wake_up",
				mhl_dev);
	if (ret) {
		pr_err("unable to request irq mhl_wake_up err:: %d\n", ret);
		goto err_wake_up_irq_request;
	}
#endif

	if (device_create_file(mhl_dev->process_dev, &dev_attr_MHD_file) < 0)
		printk("Failed to create device file(%s)!\n", dev_attr_MHD_file.attr.name);

#if 0
	disable_irq_nosync(mhl_irq);
	disable_irq_nosync(mhl_wakeup_irq);
#endif
 //   sii9234_cfg_power(1);
	return 0;

err_wake_up_irq_request:
	free_irq(gpio_to_irq(mhl_dev->irq_gpio), mhl_dev);
err_irq_request:
	misc_deregister(&mhl_dev->mdev);
err_misc_register:
	destroy_workqueue(mhl_dev->sii9234_wq);
	kfree(mhl_dev);
err_mem:
	i2c_del_driver(&sii9234c_i2c_driver);
err_i2c_c_add:
	i2c_del_driver(&sii9234b_i2c_driver);
err_i2c_b_add:
	i2c_del_driver(&sii9234a_i2c_driver);
err_i2c_a_add:
	i2c_del_driver(&sii9234_i2c_driver);

	return ret;
}

static int sii9234_remove(struct platform_device *pdev)
{
	struct mhl_dev *mhl_dev = platform_get_drvdata(pdev);

	SII_DEV_DBG("");

	disable_irq_nosync(gpio_to_irq(mhl_dev->irq_gpio));
	disable_irq_nosync(gpio_to_irq(mhl_dev->wake_up_gpio));

	i2c_del_driver(&sii9234_i2c_driver);
	i2c_del_driver(&sii9234a_i2c_driver);
	i2c_del_driver(&sii9234b_i2c_driver);
	i2c_del_driver(&sii9234c_i2c_driver);

	destroy_workqueue(mhl_dev->sii9234_wq);

	kfree(mhl_dev);
	return 0;
}

static int sii9234_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct mhl_platform_data *mhl_pdata = pdev->dev.platform_data;
	int ret;

	SII_DEV_DBG("");

#ifdef CONFIG_MHL_SWITCH
	ret = gpio_get_value(mhl_pdata->mhl_sel);
	if(ret) {
#endif
		if(mhl_pdata->power_onoff)
			mhl_pdata->power_onoff(0);
	//Disable Interrupts while going in suspend mode
		disable_irq_nosync(gpio_to_irq(mhl_pdata->mhl_int));
		disable_irq_nosync(gpio_to_irq(mhl_pdata->mhl_wake_up));
#ifdef CONFIG_MHL_SWITCH
	}
#endif
	return 0;
	}

static int sii9234_resume(struct platform_device *pdev)
{
	struct mhl_platform_data *mhl_pdata = pdev->dev.platform_data;
	int ret;

	SII_DEV_DBG("");

#ifdef CONFIG_MHL_SWITCH
	ret = gpio_get_value(mhl_pdata->mhl_sel);
	if (ret) {
#endif
		if(mhl_pdata->power_onoff)
			mhl_pdata->power_onoff(1);
	//Enable it back while resuming it back
		enable_irq(gpio_to_irq(mhl_pdata->mhl_int));
		enable_irq(gpio_to_irq(mhl_pdata->mhl_wake_up));
		//sii9234_init();
		//SiI9234_init();
#ifdef CONFIG_MHL_SWITCH
	}
#endif
	return 0;
}

static struct platform_driver mhl_driver = {
	.probe		= sii9234_probe,
	.remove		= sii9234_remove,
	.suspend	= sii9234_suspend,
	.resume		= sii9234_resume,
	.driver		= {
		.name	= "mhl",
		.owner	= THIS_MODULE,
	},
};

static int __init sii9234_module_init(void)
{
	int ret;

	ret = platform_driver_register(&mhl_driver);

	return ret;
}
module_init(sii9234_module_init);
static void __exit sii9234_exit(void)
{
	i2c_del_driver(&sii9234_i2c_driver);
	i2c_del_driver(&sii9234a_i2c_driver);
	i2c_del_driver(&sii9234b_i2c_driver);
	i2c_del_driver(&sii9234c_i2c_driver);
	platform_driver_unregister(&mhl_driver);
};
module_exit(sii9234_exit);

MODULE_DESCRIPTION("Sii9234 MHL driver");
MODULE_AUTHOR("Aakash Manik");
MODULE_LICENSE("GPL");
