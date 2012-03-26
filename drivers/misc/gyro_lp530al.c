/*
 *   drivers/misc/gyro_lp530al.c
 *
 *   SAMSUNG gyro sensor driver for Linux
 *
 *   Copyright (C) 2009 SAMSUNG ELECTRONICS.
 *   Author: SRINIDHI RAO (srinidhi.rao@samsung.com)
 *   	     SPOORTHI SHIVAKUMAR (spoorthi.s@samsung.com)
 *
 *   This package is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License version 2 as
 *   published by the Free Software Foundation.
 *
 *   THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 *   IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 *   WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSEra
 */

#include <asm/uaccess.h>		//for copy_to_user
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <plat/gpio.h>
#include <plat/hardware.h>
#include <plat/board.h>
#include <plat/mux.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/miscdevice.h>

#include <linux/i2c/twl6030-gpadc.h>
#include <asm-generic/ioctl.h>


#define LP530AL_POWER_OFF	0
#define LP530AL_POWER_ON	1

#define GYRO_LP530AL_IOCTL_MAGIC	'G'
#define GYRO_LP530AL_IOCTL_MAX_NR	3

#define GYRO_LP530AL_IOCTL_GET_GYRO_VALUES	_IOWR(GYRO_LP530AL_IOCTL_MAGIC, 0, unsigned char)
#define GYRO_LP530AL_IOCTL_SET_POWER_STATE_ON	_IOWR(GYRO_LP530AL_IOCTL_MAGIC, 1, unsigned char)
#define GYRO_LP530AL_IOCTL_SET_POWER_STATE_OFF	_IOWR(GYRO_LP530AL_IOCTL_MAGIC, 2, unsigned char)


typedef struct  {
	short int x;	//< holds x-axis angular velocity data sign extended. Range -300 to 300.
	short int z; 	//< holds z-axis angular velocity data sign extended. Range -300 to 300.
}lp530al_t;

static struct twl6030_gpadc_request conv_request = {
        .channels = (0x7 << 3),
        .do_avg = 0,
	.method = TWL6030_GPADC_SW2,
	.type = 0,
	.active = 0,
	.result_pending = 0,
	.func_cb = NULL,

};

/*TODO: Allocate and initialize an Input driver */
struct input_dev *gyro_input=NULL;

static int lp530al_pd_gpio=0, lp530al_hp_gpio=0,cal_offset=0;


#if 0
static struct workqueue_struct *lp530al_work_q;
static struct delayed_work lp530al_delayed_work_q;
static int lp530al_work(void);

static int gyro_lp530al_calibrate(int cal_offset);
#endif


/*TODO: OPTIONAL: Once the Gyro sensor is working properly,
 *	register a misc driver with IOCTL support to provide
 *	the calibration Information to User Space
 */

#if 0
static int gyro_lp530al_calibrate(int cal_offset)
{
	/*TODO: Not yet implemented */
	struct twl6030_gpadc_request cal_request;
	memset(&cal_request, 0, sizeof(struct twl6030_gpadc_request));
	cal_request = (struct twl6030_gpadc_request){
                .channels = (0x1 << 3),
		.do_avg = 0,
                .method = TWL6030_GPADC_SW1,
		.type = 0,
        	.active = 0,
	        .result_pending = 0,
        	.func_cb = NULL,

        };
        twl6030_gpadc_conversion(&cal_request);
        printk("\n\n\t ## Gyro Sensor Vref : %d\n\n", cal_request.rbuf[3]);
	return 0;

}
#endif

static int lp530al_open (struct inode *inode, struct file *filp)
{
	printk("%s called",__func__);
	return 0;
}

static int lp530al_release (struct inode *inode, struct file *filp)
{
	printk("%s called",__func__);
	return 0;
}

static ssize_t lp530al_read(struct file *filp, char __user *buf, size_t count , loff_t *f_pos )
{
	printk("%s called",__func__);
	return 0;
}

static void gyro_lp530al_report_values(struct input_dev *ip_dev, struct twl6030_gpadc_request *values)
{
	//printk("gyro_lp530al_report_values called\n");

	int gyro_temp[2];
	int gyro_vref;

	// (0 to 1024) mapped to (-300 to 300) in HAL

	gyro_vref = values->rbuf[3]/2;
	//printk( "x=%d, z=%d, ref=%d\n", values->rbuf[4], values->rbuf[5], values->rbuf[3]/2 );

	gyro_temp[0] = ((values->rbuf[4] - gyro_vref) );
	gyro_temp[1] = ((values->rbuf[5] - gyro_vref) );

	input_report_abs(ip_dev, ABS_HAT1X, gyro_temp[0] );
	input_report_abs(ip_dev, ABS_HAT1Y, gyro_temp[1] );

	input_sync(ip_dev);
}

static int gyro_lp530al_poll(void)
{
        twl6030_gpadc_conversion(&conv_request);
	gyro_lp530al_report_values(gyro_input, &conv_request);
	return 0;
}

int lp530al_ioctl(struct inode *inode, struct file *filp, unsigned int cmd,  unsigned long arg)
{
	int err = 0;
	lp530al_t gyro_data;


	/* check cmd */
	if(_IOC_TYPE(cmd) != GYRO_LP530AL_IOCTL_MAGIC)
	{
		printk("cmd magic type error\n");
		return -ENOTTY;
	}
	if(_IOC_NR(cmd) > GYRO_LP530AL_IOCTL_MAX_NR)
	{
		printk("cmd number error\n");
		return -ENOTTY;
	}

	if(err)
	{
		printk("cmd access_ok error\n");
		return -EFAULT;
	}


	switch(cmd)
	{
		case GYRO_LP530AL_IOCTL_GET_GYRO_VALUES:
			//printk("GYRO_LP530AL_IOCTL_GET_GYRO_VALUES IOCTL caught in driver\n");
			gyro_lp530al_poll();

			return 0;

		case GYRO_LP530AL_IOCTL_SET_POWER_STATE_ON:
			//printk("GYRO_LP530AL_IOCTL_SET_POWER_STATE_ON caught in driver\n");
			//set PD pin=0 to turn on the gyrosensor-chip
			printk("lp530al power-state = LP530AL_POWER_ON");
			gpio_set_value( lp530al_pd_gpio, 0);
			return 0;

		case GYRO_LP530AL_IOCTL_SET_POWER_STATE_OFF:
			//printk("GYRO_LP530AL_IOCTL_SET_POWER_STATE_OFF caught in driver\n");
			//set PD pin=1 to turn off the gyrosensor-chip
			printk("lp530al power-state = LP530AL_POWER_OFF");
			gpio_set_value( lp530al_pd_gpio, 1);
			return 0;

		default:
			printk("lp530al UNKNOWN IOCTL %d\n",cmd);
			return -ENOTTY;
	}
}

#if 0
static int lp530al_work(void)
{
	unsigned long timeout=0;

	gyro_lp530al_poll();

	printk(KERN_ERR"Gyro Sensor x-axis=%d z-axis(ch5)=%d vref=%d\n", conv_request.rbuf[4], conv_request.rbuf[5], conv_request.rbuf[3]);
	printk("Gyro Sensor x-axis voltage value GPADC_IN4 : %d\n", conv_request.rbuf[4]);
	printk("Gyro Sensor z-axis voltage value GPADC_IN5 : %d\n", conv_request.rbuf[5]);
	printk("Gyro Sensor V-ref  voltage value GPADC_IN3 : %d\n", conv_request.rbuf[3]);

	timeout = msecs_to_jiffies(900);
        queue_delayed_work(lp530al_work_q,&lp530al_delayed_work_q, timeout);

	gyro_lp530al_report_values(gyro_input, &conv_request);
	return 0;

}
#endif

int configure_gyro_gpio(struct platform_device *dev)
{
	u32 gyro_gpio_map=0,gyro_gpio_phy=0, val=0;

        lp530al_hp_gpio = platform_get_irq(dev,0);	//irq_to_gpio(platform_get_irq(dev,0));
        lp530al_pd_gpio = platform_get_irq(dev,1);	//irq_to_gpio(platform_get_irq(dev,0));
	printk(KERN_ERR"\n[GYRO-LP530AL] gpios pd = %d, gpio hp = %d",lp530al_pd_gpio,lp530al_hp_gpio);

	gyro_gpio_map = (u32)platform_get_resource(dev,IORESOURCE_MEM,0);
	printk(KERN_ERR"\n[GYRO-LP530AL] In configure gpio, gpio phy addrs is %x",gyro_gpio_map);

	gyro_gpio_phy = __raw_readl(gyro_gpio_map);
	//printk(KERN_ERR"\n[GYRO-LP530AL]  gpio pad conf value %x",val);

	val = omap_readl(gyro_gpio_phy);
	val |= 0x011B011B;
	omap_writel(val, gyro_gpio_phy);

	if(gpio_request(lp530al_pd_gpio,"gyro_lp530al_gpio_pd") < 0){
		printk(KERN_ERR"\n [GYRO-LP530AL] Error gpio_request for GYRO_PD");
		return -EIO;
	}

	if(gpio_request(lp530al_hp_gpio,"gyro_lp530al_gpio_hp") < 0){
                printk(KERN_ERR"\n [GYRO-LP530AL] Error gpio_request for GYRO_HP");
                return -EIO;
        }

	gpio_direction_output( lp530al_pd_gpio, 1);
	gpio_direction_output( lp530al_hp_gpio, 0);

	return 0;
}

static int __devinit lp530al_probe(struct platform_device *plat_dev)
{
	int err=0;

	printk("[Gyro sensor] Inside lp530al_probe\n");
	gyro_input = input_allocate_device();
	if( NULL == gyro_input){
		printk(KERN_ERR"\n [GYRO-LP530AL] Error while allocating input device");
		return -EIO;
	}

	set_bit(EV_ABS, gyro_input->evbit);
	set_bit(ABS_HAT1X, gyro_input->absbit);
	set_bit(ABS_HAT1Y, gyro_input->absbit);

	/* x-axis of raw magnetic vector */
	input_set_abs_params(gyro_input, ABS_HAT1X, 0, 1024, 0, 0);

	/* z-axis of raw magnetic vector */
	input_set_abs_params(gyro_input, ABS_HAT1Y, 0, 1024, 0, 0);

	gyro_input->name = "gyro_lp530al";
	gyro_input->dev.parent = &plat_dev->dev;
	platform_set_drvdata(plat_dev,gyro_input);

	printk("Registering the gyro device driver\n");
	err = input_register_device(gyro_input);
  	if (err) {
    		printk("Gyro Sensor couldn't be registered: %d\n", err);
    		goto release_input_dev;
  	}

	if(configure_gyro_gpio(plat_dev) != 0){
		printk(KERN_ERR"\n [GYRO-LP530AL] Error while configuring GPIOs");
		err = -EIO;
		goto release_input_dev;
	}

	return 0;

release_input_dev:
	input_free_device(gyro_input);
	return err;
}

static int __devexit lp530al_remove(struct platform_device *plat_dev)
{
	struct input_dev *ip_dev= platform_get_drvdata(plat_dev);

	gpio_direction_output( lp530al_pd_gpio, 1);
        gpio_direction_output( lp530al_hp_gpio, 1);
	gpio_free(lp530al_pd_gpio);
	gpio_free(lp530al_hp_gpio);

	flush_scheduled_work();
	//cancel_delayed_work(&lp530al_delayed_work_q);

	input_unregister_device(ip_dev);

	platform_set_drvdata(plat_dev, NULL);

	return 0;

}

static struct file_operations lp530al_fops =
{
    .owner = THIS_MODULE,
    .open = lp530al_open,
    .read = lp530al_read,
    .ioctl = lp530al_ioctl,
    .release = lp530al_release,
};

static struct miscdevice lp530al_misc_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "gyroscope",
    .fops = &lp530al_fops,
};

static struct platform_driver lp530al_plat_driver = {
	.probe = &lp530al_probe,
	.remove = &lp530al_remove,
	.suspend = NULL, //TODO:need to add PM support later
	.resume = NULL,
	.driver = {
			.name = "lp530al_gyro_sensor",
			.owner = THIS_MODULE,
		},
};

static int __init lp530al_init(void)
{
	int ret=0;

	/*misc device registration*/
	if( (ret = misc_register(&lp530al_misc_device)) < 0 )
	{
		printk("lp530al_init misc_register failed");
		return ret;
	}

	return platform_driver_register(&lp530al_plat_driver);

}
late_initcall(lp530al_init);

static void __exit lp530al_exit(void)
{
	platform_driver_unregister(&lp530al_plat_driver);
}
module_exit(lp530al_exit);
