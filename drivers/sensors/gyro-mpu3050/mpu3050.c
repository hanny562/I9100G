#include<asm/uaccess.h>
#include<asm/mach-types.h>
#include<linux/module.h>
#include<linux/init.h>
#include<linux/input.h>
#include<linux/i2c.h>
#include<linux/hrtimer.h>
#include<linux/delay.h>
#include<linux/workqueue.h>
#include <linux/slab.h>

#include "mpu3050.h"

#define MODULE_NAME "mpu3050"

static struct i2c_client* mpu3050_client;
struct mpu3050_dev *mpu3050_dev;
struct class *gyro_class;

struct input_dev *gyro_input;

static struct workqueue_struct *mpu3050_work_q;
static struct delayed_work mpu3050_delayed_work_q;

// Used globally in current driver.
static int mpu3050_timeout = -1;

// Used in suspend/resume in current driver.
static int timeout_at_suspend = -1;
static int shutdown_at_suspend = 0;

int mpu3050_i2c_read( u8 reg, u8 *rdata , int length) //Read from register "reg", in buffer "rdata","length" number of bytes
{
	int count =0;
	int ret =0;
	struct i2c_msg msg[2];

	msg[0].addr = mpu3050_client->addr;
	msg[0].flags=0;
	msg[0].len=1;
	msg[0].buf=&reg;

	count = i2c_transfer(mpu3050_client->adapter, msg, 1);
	
	if(count==1){
	        msg[0].addr=mpu3050_client->addr;
		msg[0].flags=I2C_M_RD;
		msg[0].len=length;
		msg[0].buf=rdata;
		count = i2c_transfer(mpu3050_client->adapter, msg, 1);
		if(count != 1){
			ret = -1;
		}
	}
	else{
		ret=-1;
	}

	return ret;
}
	
int mpu3050_i2c_write(u8 reg, u8 value)
{
	u8 data[2];
	int ret =0, count=0;
	struct i2c_msg msg[1];

	data[0]=reg;
	data[1]=value;

	msg[0].addr = mpu3050_client->addr;
	msg[0].flags=0;
	msg[0].len=2;
	msg[0].buf=data;

	count = i2c_transfer(mpu3050_client->adapter, msg, 1);
	if(count !=1)
	{
		ret = -1;
	}

	return ret;
}

//TODO
static int mpu3050_open(struct inode *inode,struct file *filep)
{
return 0;
}


//TODO
static int mpu3050_release(struct inode *inode, struct file *filep)
{
return 0;
}


//TODO
static int mpu3050_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
return 0;
}

//TODO
static int mpu3050_write (struct file *filp, const char *buf, size_t count, loff_t *f_pos)
{
	return 0;
}

static int mpu3050_set_operation_mode(u8 mode)
{
	int ret=0;

	//Power-OFF
	if( mode == 0 )
	{
		if ( (ret = mpu3050_i2c_write( MPU3050_PWR_MGMNT, GYRO_MODE_STANDBY )) < 0 )
		{
			ret = -EFAULT;
			printk("Error in mpu3050_set_operation_mode: MPU3050_PWR_MGMNT\n");
		}

		mpu3050_timeout =-1;
		flush_workqueue(mpu3050_work_q);
	        cancel_delayed_work(&mpu3050_delayed_work_q);
	}

	//Power-ON
	else if ( mode == 1 )
	{
		if ( (ret = mpu3050_i2c_write( MPU3050_PWR_MGMNT, GYRO_MODE_ALL_ON )) < 0 )
		{
			ret = -EFAULT;
			printk("Error in mpu3050_set_operation_mode: MPU3050_PWR_MGMNT\n");
		}

		#if 0
		if( (ret = mpu3050_i2c_write( MPU3050_USER_CTRL, 0x01 )) < 0 )
		{
			ret = -EFAULT;
			printk("Error in mpu3050_set_operation_mode: MPU3050_USER_CTRL\n");
		}
		#endif

		if( (ret = mpu3050_i2c_write( MPU3050_DLPF_FS_SYNC, GYRO_RANGE_2000 )) < 0 )
		{
			ret = -EFAULT;
			printk("Error in mpu3050_set_operation_mode: MPU3050_DLPF_FS_SYNC\n");
		}


		// If poll_delay not set,
		// initialise it to default value 200ms
		if (mpu3050_timeout == -1) {
			mpu3050_timeout = msecs_to_jiffies(200);
		}

		//start the polling workqueue
		queue_delayed_work(mpu3050_work_q, &mpu3050_delayed_work_q, mpu3050_timeout);

	}

	return ret;
}

static int mpu3050_read_gyro_xyz(u16 *gyro_data_16)
{

	//printk("mpu3050_read_gyro_xyz called\n");

	int ret = -1;
	u8 gyro_data[6];

	ret = mpu3050_i2c_read( MPU3050_GYRO_XOUT_H, gyro_data, 6 );
	if( ret < 0)
	{
		printk("mpu3050_i2c_read failed!!!!\n\n");
	}
	else
	{
		gyro_data_16[0]= (gyro_data[0] * 256) + gyro_data[1];
		gyro_data_16[1]= (gyro_data[2] * 256) + gyro_data[3];
		gyro_data_16[2]= (gyro_data[4] * 256) + gyro_data[5];

		#ifdef MPU3050_DEBUG
		printk("XH=%x XL=%x\nYH=%x YL=%x\nZH=%x ZL=%x\n\n",gyro_data[0], gyro_data[1], gyro_data[2], gyro_data[3], gyro_data[4], gyro_data[5]);
		printk("gyrodata X=%d Y=%d Z=%d \n\n\n\n",gyro_data_16[0], gyro_data_16[1], gyro_data_16[2]);
		#endif
	}

	return ret;
}

#if 0
static int mpu3050_set_interrupt_config(u8 interrupt_mode)
{
	u8 interrupt = 0;
	int ret=0;
	interrupt |= (1<<interrupt_mode);
	ret = mpu3050_i2c_write(MPU3050_INT_CFG, interrupt);
	if(ret < 0)
	{
		printk("Error in Interrupt Configuration");
		ret = -EFAULT;
	}
	return (ret);
}

static int mpu3050_get_interrupt_config(u8 * interrupt_config)
{
	int ret =0;
	ret = mpu3050_i2c_read( MPU3050_INT_CFG, interrupt_config, 1);

	return (ret);
}
#endif

static int mpu3050_work(void)
{
	//printk("mpu3050_work called\n");
	u16 gyro_values[3];
	int ret =0;

	ret = mpu3050_read_gyro_xyz(gyro_values);

	if(ret < 0)
	{
		printk("mpu3050_read_gyro_xyz FAILED\n");
		return (-1);
	}

	input_report_rel(gyro_input, REL_X, gyro_values[0] );
	input_report_rel(gyro_input, REL_Y, gyro_values[1] );
	input_report_rel(gyro_input, REL_Z, gyro_values[2] );
	input_sync(gyro_input);

	if(mpu3050_timeout > -1)
	{
		queue_delayed_work(mpu3050_work_q, &mpu3050_delayed_work_q, mpu3050_timeout);
	}

	return 0;
}


static int mpu3050_ioctl( struct inode *inode, struct file *filp, unsigned int ioctl_cmd, unsigned long arg )
{
	int ret =0;
	void __user *argp = (void __user*)arg;
	
	if(_IOC_TYPE(ioctl_cmd) != MPU3050_GYRO_IOC_MAGIC)
	{
		printk("mpu3050 cmd magic type error\n");
		return -ENOTTY;
	}

	if(_IOC_NR(ioctl_cmd)>MPU3050_GYRO_IOC_MAXNR)
	{
		printk("mpu3050 cmd number error\n");
		return -ENOTTY;
	}

	switch(ioctl_cmd)
	{

		case MPU3050_GYRO_IOC_GET_OPERATION_MODE:
		{
			// u8 mode;
			// if(ret = mpu3050_get_operation_mode(&mode) < 0)
			//	//debug message
			// else if(copy_to_user(argp,(void *)&mode,sizeof(u8)))
			//	return (-EFAULT);
			//
			printk("[mpu3050] get_operation_mode called\n");
			return(0);
		}

		case MPU3050_GYRO_IOC_SET_OPERATION_MODE:
		{		
			unsigned char mode = 0;
			if(copy_from_user( &mode, argp, sizeof(mode)))
			{
				printk(KERN_ERR"Copy_from_user failed\n");
				return (-EFAULT);
			}

			#if MPU3050_DEBUG
				printk("\n\nmpu3050 mode==%x\n\n",mode);
			#endif		

			// mode is either GYRO_POWER_OFF or GYRO_POWER_ON
			ret = mpu3050_set_operation_mode(mode);

			if( ret < 0 ) {
				printk("mpu3050: set_operation_mode Failed\n");
				return (-EFAULT);			
			}

			return(0);
		}


		case MPU3050_GYRO_IOC_SET_POLL_DELAY:
		{
			u32 delay;

			if(copy_from_user(&delay, argp, sizeof(delay) ))
			{
				printk("mpu3050: copy_from_user failed\n");
				return (-EFAULT);
			}

			else
			{
				mpu3050_timeout = msecs_to_jiffies(delay);
				printk("mpu3050 polling delay=%d\n", mpu3050_timeout);
			}

			return(0);
		}


		default:
		{
			printk(KERN_ERR"mpu3050: UNKNOWN IOCTL\n");
			return (-ENOTTY);
		}
	}
}

/***********************Sysfs Interface *********************************/
static ssize_t poll_delay_show( struct device *dev, struct device_attribute *attr, char *buf)
{
        long int delay_to_report;

        if (mpu3050_timeout<0)
        {
                delay_to_report = -1 ;
        }
        else
        {
                delay_to_report = jiffies_to_msecs(mpu3050_timeout*1000000);
        }

#ifdef MPU3050_DEBUG
        printk("delay_to_report=%ld\n", delay_to_report );
#endif

        return snprintf(buf, PAGE_SIZE, "%ld\n", delay_to_report);
}


static ssize_t poll_delay_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
        int err;
        long int new_delay;

#ifdef MPU3050_DEBUG
        printk("buf=%s\n", buf);
        printk("size=%u\n", size);
#endif

        err = strict_strtol(buf, 10, &new_delay);
        if ( err < 0 )
                return err;

	mpu3050_timeout = msecs_to_jiffies(new_delay/1000000);
	
#ifdef MPU3050_DEBUG
        printk("poll_delay_store called. new_delay=%ld\n", new_delay);
        printk("poll_delay_store called. mpu3050_timeout=%d\n", mpu3050_timeout);
#endif
        return size;
}
static ssize_t gyro_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{

#ifdef MPU3050_DEBUG
        printk("enabled=%d\n", ((mpu3050_timeout < 0) ? 0 : 1) );
#endif

        return snprintf(buf, PAGE_SIZE, "%d\n", ((mpu3050_timeout < 0) ? 0 : 1) );
}


static ssize_t gyro_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
        bool new_value;
        int ret = 0;

        if (sysfs_streq(buf, "1"))
                new_value = true;
        else if (sysfs_streq(buf, "0"))
                new_value = false;
        else {
                pr_err("%s: invalid value %d\n", __func__, *buf);
                return -EINVAL;
        }

#ifdef MPU3050_DEBUG
        printk("new_value = %d, old state = %d\n", new_value, ((mpu3050_timeout < 0) ? 0 : 1) );
#endif
        if (new_value && (mpu3050_timeout < 0) ) {

                ret = mpu3050_set_operation_mode(GYRO_POWER_ON);
                if (ret < 0) {
                        printk("GYRO POWER_UP failed!!\n");
                        ret = -EINVAL;
                }

        } else if (!new_value && (mpu3050_timeout > -1) ) {

                ret = mpu3050_set_operation_mode(GYRO_POWER_OFF);
                if (ret < 0) {
                        printk("GYRO POWER_DOWN failed!!\n");
                        ret = -EINVAL;
                }
        }

        (ret < 0) ? (int)NULL : (ret = size) ;

        return ret;
}


static struct device_attribute dev_attr_poll_delay =
        __ATTR(poll_delay, S_IRUGO | S_IWUGO | S_IXUGO,
               poll_delay_show, poll_delay_store);

static struct device_attribute dev_attr_gyro_enable =
        __ATTR(enable, S_IRUGO | S_IWUGO | S_IXUGO,
               gyro_enable_show, gyro_enable_store);


static struct attribute *gyro_sysfs_attrs[] = {
        &dev_attr_gyro_enable.attr,
        &dev_attr_poll_delay.attr,
        NULL
};

static struct attribute_group gyro_attribute_group = {
        .attrs = gyro_sysfs_attrs,
};

struct file_operations mpu_fops =
{
	.owner   = THIS_MODULE,
	.read    = mpu3050_read,
	.write   = mpu3050_write,
	.open    = mpu3050_open,
	.ioctl   = mpu3050_ioctl,
	.release = mpu3050_release,
};

#if CONFIG_PM
static int mpu3050_suspend(struct i2c_client * client, pm_message_t mesg)
{
	int ret = 0;

        #ifdef MPU3050_DEBUG
        printk(KERN_INFO MODULE_NAME ": suspending device...\n");
        #endif

       ret = mpu3050_set_operation_mode(GYRO_POWER_OFF);
       if (ret < 0) {
               printk("GYRO POWER_OFF failed!!\n");
               return -EINVAL;
       }

	if(mpu3050_timeout > -1) {

		shutdown_at_suspend = 1;
		timeout_at_suspend = mpu3050_timeout;

	}
	else {
		shutdown_at_suspend = 0;
	}

        return 0;
}

static int mpu3050_resume(struct i2c_client * client)
{
	int ret = 0;

       ret = mpu3050_set_operation_mode(GYRO_POWER_ON);
       if (ret < 0) {
               printk("GYRO POWER_ON failed!!\n");
               ret = -EINVAL;
       }

	if (shutdown_at_suspend) {

		#if MPU3050_DEBUG
		printk(KERN_INFO MODULE_NAME ": resuming device...\n");
		#endif

		shutdown_at_suspend = 0;

		mpu3050_timeout = timeout_at_suspend;
	}

	return ret;
}
#else

#define mpu3050_suspend NULL
#define mpu3050_resume NULL

#endif
                                               

static int mpu3050_probe(struct i2c_client *client,const struct i2c_device_id *id )
{
	int ret = 0;
	int result_gyro=-1;

	struct device *dev_gyro;
	mpu3050_client = client;
	
	//Allocate Gyro-device
	mpu3050_dev = kzalloc(sizeof *mpu3050_dev, GFP_KERNEL);
	if(mpu3050_dev < 0)
	{
		printk("[MPU-3050]:Not enough memory while allocating Device\n");
		return -ENOMEM;
	}

	//Initialize lock;
	mutex_init(&mpu3050_dev->lock);
	mpu3050_dev->client = client;
	i2c_set_clientdata(client, mpu3050_dev);
	mpu3050_client = client;
	#ifdef MPU3050_DEBUG
		printk(KERN_INFO MODULE_NAME"Registering [MPU-3050] Gyro-Driver at address:%d",mpu3050_client->addr);
		printk("--------%s %d\n",__func__,__LINE__);
	#endif
	
	//Register char-driver
	result_gyro = register_chrdev ( (int)NULL, GYRO_DEV_NAME, &mpu_fops);

	if(result_gyro <0)
	{
		printk(KERN_ERR "Error while registering[MPU-3050] the char-device\n");
		return result_gyro;
	}


	gyro_class = class_create(THIS_MODULE,GYRO_DEV_NAME);
	if(IS_ERR(gyro_class))
		{
			printk(KERN_ERR"Error while[MPU-3050] Creating the gyro_class\n");
			unregister_chrdev(result_gyro,GYRO_DEV_NAME);
			return PTR_ERR(gyro_class);
		}

	dev_gyro = device_create(gyro_class, NULL, MKDEV(result_gyro ,0),"%s",GYRO_DEV_NAME);

	if(IS_ERR(dev_gyro))
	{
		printk(KERN_ERR "Error while[MPU-3050] Creating the dev_gyro\n");
		return PTR_ERR(dev_gyro);
	}
	

	//Register input device
	gyro_input = input_allocate_device();

	if(NULL == gyro_input)
	{
		printk(KERN_ERR"\n[MPU3050-GYROSENSOR] Error while allocating input device");
		return -EIO;
	}

	mpu3050_dev->inputdev=gyro_input;
	set_bit(EV_REL, gyro_input->evbit);
	set_bit(REL_X, gyro_input->relbit);
	set_bit(REL_Y, gyro_input->relbit);
	set_bit(REL_Z, gyro_input->relbit);
	
	gyro_input->name="mpu3050_gyro";
	gyro_input->dev.parent = &mpu3050_client->dev;

	ret = input_register_device(gyro_input);
	if(ret)
	{
		printk("mpu3050 Gyro Sensor couldn't be registered:%d\n",ret);
		goto release_input_dev;
	}	
	 ret = sysfs_create_group(&gyro_input->dev.kobj, &gyro_attribute_group);
        if (ret) {
                pr_err("%s: could not create sysfs group\n", __func__);
                goto release_input_dev;
        }


	//Workqueue Initialisation
	mpu3050_work_q = create_singlethread_workqueue("gyro_sensor_mpu3050_work_queue");
	INIT_DELAYED_WORK( (struct delayed_work *)&mpu3050_delayed_work_q, mpu3050_work);

	mpu3050_dev->mpu3050_early_suspend.suspend = mpu3050_suspend;
        mpu3050_dev->mpu3050_early_suspend.resume = mpu3050_resume;
        register_early_suspend(&mpu3050_dev->mpu3050_early_suspend);


release_input_dev:
	input_free_device(gyro_input);
	return ret;
}
static int  mpu3050_remove(struct i2c_client *client)
{
	struct mpu3050_dev *mpu3050 = i2c_get_clientdata(client);
	 unregister_early_suspend(&mpu3050_dev->mpu3050_early_suspend);
	input_unregister_device(mpu3050->inputdev);
	input_free_device(mpu3050->inputdev);
	kfree(mpu3050);

	return 0;
}


static const struct i2c_device_id mpu3050_ids[] = {	
        { MODULE_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mpu3050_ids);

static struct i2c_driver mpu3050_driver = {
	.driver = {
		.name	= MODULE_NAME,
		.owner	= THIS_MODULE,
	},
	.id_table = mpu3050_ids,
	.probe = mpu3050_probe,
	.remove = mpu3050_remove,
#if CONFIG_PM
//	.suspend = mpu3050_suspend,
//	.resume = mpu3050_resume,
#endif
};


static int __init mpu3050_init(void)
{
	#ifdef MPU3050_DEBUG
	printk(KERN_INFO MODULE_NAME ": Registering MPU-3050 driver\n");
	#endif
	return i2c_add_driver(&mpu3050_driver);
}

static void __exit mpu3050_exit(void)
{
	#ifdef MPU3050_DEBUG
	printk(KERN_INFO MODULE_NAME ": Unregistered MPU-3050 driver\n");
	#endif
	i2c_del_driver(&mpu3050_driver);
}

module_init(mpu3050_init);
module_exit(mpu3050_exit);

MODULE_AUTHOR("Dharam Kumar");
MODULE_DESCRIPTION("MPU-3050 Driver");
MODULE_LICENSE("GPL");

