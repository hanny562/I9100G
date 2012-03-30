/*****************************************************************************
 *
 * COPYRIGHT(C) : Samsung Electronics Co.Ltd, 2006-2015 ALL RIGHTS RESERVED
 *
 *****************************************************************************/
/*Linux Kernel Driver for ISL29023 Ambient Light Sensor Driver*/ 

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <asm/uaccess.h>

#include <plat/irqs.h>
#include <mach/gpio.h>
#include "isl29023.h"
#include "isl29023_func.h"
#include "isl29023_i2c.h"

static int isl29023_open (struct inode *, struct file *);
static int isl29023_release (struct inode *, struct file *);
static ssize_t isl29023_read(struct file *, char __user *, size_t, loff_t *);
static int isl29023_isr( int , void * );
#ifdef USE_ISL29023_IOCTLS 
static int isl29023_ioctl(struct inode *, struct file *, unsigned int ,unsigned long);
#endif


static struct workqueue_struct *isl29023_work_q;
static struct delayed_work isl29023_delayed_work_q;
static int isl29023_work(void);
int isl29023_timeout = -1;

struct class *lightsensor_class;
struct device *switch_cmd_dev;
static bool light_enable = OFF;

static struct input_dev *isl29023_input_dev;

static struct file_operations isl29023_fops =
{
    .owner = THIS_MODULE,
    .open = isl29023_open,
	.read = isl29023_read,
#ifdef USE_ISL29023_IOCTLS 
    .ioctl = isl29023_ioctl,
#endif
    .release = isl29023_release,
};

static struct miscdevice isl29023_misc_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "light",
    .fops = &isl29023_fops,
};

static int isl29023_open (struct inode *inode, struct file *filp)
{
    int ret=0;
	debug("%s called",__func__);
	if((ret=isl29023_set_operation_mode(ALS_CONTINUOUS))<0)
		ret=-1;
	return ret;
}

static int isl29023_release (struct inode *inode, struct file *filp)
{
	int ret=0;
	debug("%s called",__func__);
	if((ret=isl29023_set_operation_mode(POWER_DOWN))<0)
		ret=-1;
	return ret;
}

static ssize_t isl29023_read(struct file *filp, char __user *buf, size_t count , loff_t *f_pos )
{
	u32 lux_value;	
	debug("%s called",__func__);	
	isl29023_get_lux_value(&lux_value);
	debug("lux_value =%d",lux_value);
	put_user(lux_value,buf);
	return 1;
}

static int isl29023_isr( int irq, void *unused )
{
	//TBD
	debug("%s called",__func__);
	return 0;
}

#ifdef USE_ISL29023_IOCTLS 
static int isl29023_ioctl(struct inode *inode, struct file *filp, 
	                        unsigned int ioctl_cmd,  unsigned long arg)
{	int ret = 0;
	void __user *argp = (void __user *)arg;   
	
	if( _IOC_TYPE(ioctl_cmd) != ISL29023_LIGHT_IOC_MAGIC )
    {
        debug("Wrong _IOC_TYPE 0x%x",ioctl_cmd);
        return -ENOTTY;
    }
    if( _IOC_NR(ioctl_cmd) > ISL29023_LIGHT_IOC_NR_MAX )
    {
        debug("Wrong _IOC_NR 0x%x",ioctl_cmd);	
        return -ENOTTY;
    }
	switch (ioctl_cmd)
    {
        case ISL29023_LIGHT_IOC_SET_OPERATION_MODE:
			{
				int mode;
				debug("ISL29023_LIGHT_IOC_SET_OPERATION_MODE called");
				
				if(copy_from_user((void*) &mode, argp, sizeof(int)))
				{       	   	 
					debug("copy_from_user failed");
					ret = -EFAULT;        	   	   
				}

				printk ("mode == %d\n", mode);
				if((int)mode == 0)	//power-down mode
				{
					debug("power-down");
					isl29023_timeout = -1;
					flush_workqueue(isl29023_work_q);
				}
				else
				{
					//power-on modes
					debug("power-up");
					//Initially schedule workqueue with lowest polling frequency
					isl29023_timeout = msecs_to_jiffies(200);
					queue_delayed_work(isl29023_work_q, &isl29023_delayed_work_q, isl29023_timeout);
				}


				if( (ret = isl29023_set_operation_mode(mode)) < 0 )        		
				{
					debug("ISL29023_LIGHT_IOC_SET_OPERATION_MODE failed"); 			
				}

			break;
			}
        case ISL29023_LIGHT_IOC_SET_LUX_RANGE:			
			{
				u32 range;
				debug("ISL29023_LIGHT_IOC_SET_LUX_RANGE called");	
				
				if(copy_from_user((void*) &range, argp, sizeof(u32)))        	   	 
        	   	       ret = -EFAULT;        	   	   
                else if( (ret = isl29023_set_lux_range(range)) < 0 )       			
					debug("ISL29023_LIGHT_IOC_SET_LUX_RANGE failed"); 			
				break;
			}
		case ISL29023_LIGHT_IOC_SET_ADC_RESOLUTION:
			{
				u8 resol;				debug("ISL29023_LIGHT_IOC_SET_ADC_RESOLUTION called");	
				
				if(copy_from_user((void*) &resol, argp, sizeof(u8)))        	   	 
        	   	       ret = -EFAULT;        	   	   
                else if( (ret = isl29023_set_adc_resolution(resol)) < 0 )        
					debug("ISL29023_LIGHT_IOC_SET_ADC_RESOLUTION failed"); 			
				break;
			}
		case ISL29023_LIGHT_IOC_SET_INTERRUPT_PERSIST:
			{
				u8 persist;
				debug("ISL29023_LIGHT_IOC_SET_INTERRUPT_PERSIST called");	
			
				if(copy_from_user((void*) &persist, argp, sizeof(u8)))        	   	 
        	   	       ret = -EFAULT;        	   	   
                else if( (ret = isl29023_set_interrupt_persist(persist)) < 0 )        	
					debug("ISL29023_LIGHT_IOC_SET_INTERRUPT_PERSIST failed"); 			
				break;
			}		
		case ISL29023_LIGHT_IOC_GET_OPERATION_MODE:
			{
				u8 mode;
				debug("ISL29023_LIGHT_IOC_GET_OPERATION_MODE called");	
		
                if( (ret = isl29023_get_operation_mode(&mode)) < 0 )        
					debug("ISL29023_LIGHT_IOC_GET_OPERATION_MODE failed");
				else if(copy_to_user(argp, (void*) &mode, sizeof(u8)))    	   	 
        	   	       ret = -EFAULT; 
				break;
			}
		case ISL29023_LIGHT_IOC_GET_LUX_RANGE:
			{
				u32 range;
				debug("ISL29023_LIGHT_IOC_GET_LUX_RANGE called");	
			
                if( (ret = isl29023_get_lux_range(&range)) < 0 )        
					debug("ISL29023_LIGHT_IOC_GET_LUX_RANGE failed");
				else if(copy_to_user(argp, (void*) &range, sizeof(u32)))    	   	 
        	   	       ret = -EFAULT; 
				break;
			}
		case ISL29023_LIGHT_IOC_GET_ADC_RESOLUTION:
			{
				u8 resol;
				debug("ISL29023_LIGHT_IOC_GET_ADC_RESOLUTION called");	
			
                if( (ret = isl29023_get_adc_resolution(&resol)) < 0 )        
					debug("ISL29023_LIGHT_IOC_GET_ADC_RESOLUTION failed");
				else if(copy_to_user(argp, (void*) &resol, sizeof(u8)))    	   	 
        	   	       ret = -EFAULT; 
				break;
			}
		case ISL29023_LIGHT_IOC_GET_INTERRUPT_PERSIST:
			{
				u8 persist;
				debug("ISL29023_LIGHT_IOC_GET_INTERRUPT_PERSIST called");	
			
                if( (ret = isl29023_get_interrupt_persist(&persist)) < 0 )        
					debug("ISL29023_LIGHT_IOC_GET_INTERRUPT_PERSIST failed");
				else if(copy_to_user(argp, (void*) &persist, sizeof(u8)))    	   	 
        	   	       ret = -EFAULT; 
				break;
			}

		case ISL29023_LIGHT_IOC_SET_POLL_DELAY:
		{
			u32 delay;
			debug("ISL29023_LIGHT_IOC_SET_POLL_DELAY called");	


			//obtain delay from userspace and update the polling frequency accordingly
			if(copy_from_user((void*) &delay, argp, sizeof(u32)))        	   	 
			{
				ret = -EFAULT;
			}
			else
			{
				isl29023_timeout = msecs_to_jiffies(delay);
				printk("lp530al polling delay =%d\n", isl29023_timeout);
			}

			break;
		}

		default:
			debug("Unknown IOCTL command");
            ret = -ENOTTY;
            break;
	}
	return ret;
}
#endif

/*sysfs -operation_mode*/
static ssize_t isl29023_show_operation_mode(struct device *dev,struct device_attribute *attr, char *buf)
{      
		u8 mode;
		isl29023_get_operation_mode(&mode);
        return sprintf(buf, "%d\n", mode);
}
static ssize_t isl29023_store_operation_mode(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{      
		int mode,ret;
		ret=0;
		sscanf(buf,"%d",&mode);
		if(mode>=POWER_DOWN && mode <=IR_CONTINUOUS)
		{
			if((ret=isl29023_set_operation_mode((u8)mode))<0)
				return ret;
		}
		else
			return -EINVAL;
		return count;
}
static DEVICE_ATTR(operation_mode, S_IRUGO | S_IWUGO, isl29023_show_operation_mode, isl29023_store_operation_mode);

/*sysfs -lux_range*/
static ssize_t isl29003_show_lux_range(struct device *dev,struct device_attribute *attr, char *buf)
{      
		u32 lux_range;
		isl29023_get_lux_range(&lux_range);
        return sprintf(buf, "%d\n", lux_range);
}
static ssize_t isl29003_store_lux_range(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{      
		int lux_range,ret;
		ret=0;
		sscanf(buf,"%d",&lux_range);
		if(lux_range>=LUX_RANGE_1 && lux_range<=LUX_RANGE_4)
		{
			if((ret=isl29023_set_lux_range((u32)lux_range))<0)
				return ret;
		}
		else
			return -EINVAL;
		return count;
}
static DEVICE_ATTR(lux_range, S_IRUGO | S_IWUGO, isl29003_show_lux_range, isl29003_store_lux_range);

/*sysfs -adc_resolution*/
static ssize_t isl29003_show_adc_resolution(struct device *dev,struct device_attribute *attr, char *buf)
{      
		u8 adc_resol;
		isl29023_get_adc_resolution(&adc_resol);
        return sprintf(buf, "%d\n", adc_resol);
}
static ssize_t isl29003_store_adc_resolution(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{      
		int adc_resol,ret;
		ret=0;
		sscanf(buf,"%d",&adc_resol);
		if(adc_resol>=ADC_RESOLUTION_4 && adc_resol<=ADC_RESOLUTION_16)
		{
			if((ret=isl29023_set_adc_resolution((u8)adc_resol))<0)
				return ret;
		}
		else
			return -EINVAL;			
		return count;
}
static DEVICE_ATTR(adc_resolution, S_IRUGO | S_IWUGO, isl29003_show_adc_resolution, isl29003_store_adc_resolution);

/*sysfs -interrupt_persist*/
static ssize_t isl29023_show_interrupt_persist(struct device *dev,struct device_attribute *attr, char *buf)
{      
		u8 persist;
		isl29023_get_interrupt_persist(&persist);
        return sprintf(buf, "%d\n", persist);
}
static ssize_t isl29023_store_interrupt_persist(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{      
		int persist,ret;
		ret=0;
		sscanf(buf,"%d",&persist);
		if(persist>=INTERRUPT_PERSIST_1 && persist<=INTERRUPT_PERSIST_16)
		{
			if((ret=isl29023_set_interrupt_persist((u8)persist))<0)
				return ret;
		}
		else
			return -EINVAL;
		return count;
}
static DEVICE_ATTR(interrupt_persist, S_IRUGO | S_IWUGO, isl29023_show_interrupt_persist, isl29023_store_interrupt_persist);

/*sysfs -lux_value*/
static ssize_t isl29023_show_lux_value(struct device *dev,struct device_attribute *attr, char *buf)
{   	
		u32 lux_value;
		isl29023_get_lux_value(&lux_value);		
        return sprintf(buf, "%d\n", lux_value);	
}
static DEVICE_ATTR(lux_value, S_IRUGO, isl29023_show_lux_value,NULL);

static struct attribute *isl29003_attributes[] = {
        &dev_attr_operation_mode.attr,
        &dev_attr_lux_range.attr,
		&dev_attr_adc_resolution.attr,
        &dev_attr_interrupt_persist.attr,
        &dev_attr_lux_value.attr,
        NULL
};

static const struct attribute_group isl29003_attr_group = {
         .attrs = isl29003_attributes,
 };

/*For Factory Test Mode*/
static ssize_t lightsensor_file_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
		u32 lux_value;
		isl29023_get_lux_value(&lux_value);		
        return sprintf(buf, "%d\n", lux_value);	
}
static DEVICE_ATTR(lightsensor_file_state, S_IRUGO, lightsensor_file_state_show,NULL);

/* for light sensor on/off control from platform */
static ssize_t lightsensor_file_cmd_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 mode;
	isl29023_get_operation_mode(&mode);
	if(mode==POWER_DOWN)
		light_enable = OFF;
	else if(mode==ALS_CONTINUOUS)
		light_enable = ON;
	return sprintf(buf, "%d\n", light_enable);
}
static ssize_t lightsensor_file_cmd_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int mode,ret;
	ret=0;
	sscanf(buf,"%d",&mode);
	if(mode>=POWER_DOWN && mode <=IR_CONTINUOUS)
	{
		switch(mode)
		{
			case 0:
				ret=isl29023_set_operation_mode(POWER_DOWN);					
				break;
			case 1:
				ret=isl29023_set_operation_mode(ALS_CONTINUOUS);					
				break;
			default:
				break;
		}
		if(ret<0)
			return ret;
	}
	else
		return -EINVAL;
	return count;		
}
static DEVICE_ATTR(lightsensor_file_cmd, S_IRUGO | S_IWUGO, lightsensor_file_cmd_show, lightsensor_file_cmd_store);

static int isl29023_work(void)
{
	debug("ISL29023_WORK called!!");

	int ret = 0;
	u32 lux_value;

	ret = isl29023_get_lux_value(&lux_value);
	
	if( ret < 0 )        
	{
		debug("ISL29023_LIGHT_IOC_GET_LUX_VALUE failed");
		return(-1);
	}

	input_report_abs(isl29023_input_dev, ABS_MISC, lux_value );
	input_report_abs(isl29023_input_dev, ABS_DISTANCE, lux_value );
	input_sync(isl29023_input_dev);

	if (isl29023_timeout > -1)
	{
		queue_delayed_work(isl29023_work_q, &isl29023_delayed_work_q, isl29023_timeout);
	}

	return 0;

}

int __init isl29023_drv_init(void)
{
	int ret =0;
	debug("%s called",__func__);
	/*misc device registration*/
    if( (ret = misc_register(&isl29023_misc_device)) < 0 )
    {
        error("isl29023_drv_init misc_register failed");
        return ret; 	  	
    }
	
	/*mutex initialisation*/
	isl29023_dev_mutex_init();	

	/* set sysfs for light sensor test mode*/
	lightsensor_class = class_create(THIS_MODULE, "lightsensor");
	if (IS_ERR(lightsensor_class))
	{
		debug("Failed to create class(lightsensor)!\n");
		goto FREE_IRQ;
	}
	switch_cmd_dev = device_create(lightsensor_class, NULL, 0, NULL, "switch_cmd");
	if (IS_ERR(switch_cmd_dev))
	{
		debug("Failed to create device(switch_cmd_dev)!\n");
		goto DESTROY_CLASS;
	}
	if (device_create_file(switch_cmd_dev, &dev_attr_lightsensor_file_cmd) < 0)
	{
		debug("Failed to create device file(%s)!\n", dev_attr_lightsensor_file_cmd.attr.name);
		goto DESTROY_DEVICE;
	}
	if (device_create_file(switch_cmd_dev, &dev_attr_lightsensor_file_state) < 0)
	{
		debug("Failed to create device file(%s)!\n", dev_attr_lightsensor_file_state.attr.name);
		device_remove_file(switch_cmd_dev, &dev_attr_lightsensor_file_cmd);
		goto DESTROY_DEVICE;
	}
	

	//input-device initialisation
	isl29023_input_dev = input_allocate_device();
	set_bit(EV_ABS, isl29023_input_dev->evbit);
	set_bit(ABS_MISC, isl29023_input_dev->absbit);
	set_bit(ABS_DISTANCE, isl29023_input_dev->absbit);

	isl29023_input_dev->name = "isl29023_light";

	input_set_abs_params(isl29023_input_dev, ABS_MISC, 0, 30000, 0, 0);
	input_set_abs_params(isl29023_input_dev, ABS_DISTANCE, 0, 1, 0, 0);
	
	ret = input_register_device(isl29023_input_dev);
  	if (ret)
	{
    		printk("Light/Proximity Sensor couldn't be registered as input device: %d\n", ret);
    		goto release_input_dev;
  	}

	//Workqueue Initialisation
	isl29023_work_q = create_singlethread_workqueue("light_sensor_isl29023_work_queue");
	INIT_DELAYED_WORK( (struct delayed_work *)&isl29023_delayed_work_q, isl29023_work);


	/*create sysfs attributes*/
	ret = sysfs_create_group(&isl29023_misc_device.this_device->kobj, &isl29003_attr_group);
    if (ret)
    {
		printk("Failed to create sysfs attributes");
		goto REMOVE_DEVICE_FILE;
	}
	
	if ( (ret = isl29023_i2c_init() < 0) ) 
    {
		goto REMOVE_SYSFS_ATTRS;
    }
	return ret;
	
REMOVE_SYSFS_ATTRS:
	sysfs_remove_group(&isl29023_misc_device.this_device->kobj, &isl29003_attr_group);
release_input_dev:
	input_free_device(isl29023_input_dev);
REMOVE_DEVICE_FILE:
	device_remove_file(switch_cmd_dev, &dev_attr_lightsensor_file_state);
	device_remove_file(switch_cmd_dev, &dev_attr_lightsensor_file_cmd);
DESTROY_DEVICE:
	device_destroy(lightsensor_class,0);
DESTROY_CLASS:
	class_destroy(lightsensor_class);
FREE_IRQ:
	// free_irq(LIGHTSENSOR_IRQ,NULL);
MISC_DREG:
	misc_deregister(&isl29023_misc_device);
	return ret;
}

void __init isl29023_drv_exit(void)
{	debug("%s called",__func__);	
	isl29023_i2c_exit();
	sysfs_remove_group(&isl29023_misc_device.this_device->kobj, &isl29003_attr_group);
	device_remove_file(switch_cmd_dev, &dev_attr_lightsensor_file_state);
	device_remove_file(switch_cmd_dev, &dev_attr_lightsensor_file_cmd);
	device_destroy(lightsensor_class,0);
	class_destroy(lightsensor_class);
	// free_irq(LIGHTSENSOR_IRQ,NULL);
	misc_deregister(&isl29023_misc_device);
}
module_init(isl29023_drv_init);
module_exit(isl29023_drv_exit);			

MODULE_AUTHOR("V.N.V.Srikanth, SAMSUNG ELECTRONICS, vnv.srikanth@samsung.com");
MODULE_DESCRIPTION("Ambient Light Sensor driver for ISL29023");
MODULE_LICENSE("GPL"); 
