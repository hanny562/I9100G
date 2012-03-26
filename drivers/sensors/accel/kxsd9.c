/* kxsd9.c
 *
 * also acts as pedometer and free-fall detector
 * reports g-force as x,y,z
 * reports step count

    ^
 +y |
    ___
-x | -'| +x
<--|   |-->
   |___| / -z
   |_O_|/
 -y |  /
    v / +z

 *
 * 
 */


#include <asm/uaccess.h>
#include <asm/mach-types.h>
#include <asm/io.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>
#include <linux/slab.h>
#ifdef CONFIG_ANDROID_POWER
#include <linux/android_power.h>
#endif

#include "kxsd9.h"
#include "kxsd9_i2c.h"
#include "kxsd9_acc.h"

#define MODULE_NAME "kxsd9"
#define KXSD9_DEBUG  0
#define KXSD9_DUMP   0


struct kxsd9 *kxsd9;
static struct input_dev *kxsd9_input_dev;

// Used globally in current driver.
static int kxsd9_timeout = -1;

// Used in suspend/resume in current driver.
static int timeout_at_suspend = -1;
static int shutdown_at_suspend = 0;

// Workqueue related definitions
static struct workqueue_struct *kxsd9_work_q;
static struct delayed_work kxsd9_delayed_work_q;

#if 0

struct kxsd9_seq {
	int len;
	unsigned char *data;
} kxsd9_enable_seq[] = {
	{ 2, "\x0d""\xc0" },
	{ 1, "\x1e" },
	{ 2, "\x0a""\xca" },
	{ 2, "\x0b""\x60" },
	{ 2, "\x0c""\xe3" },
	{ 1, "\x00" },
}, kxsd9_disable_seq[] = {
	{ 2, "\x0d""\x00" },
};

static ktime_t kxsd9_poll_time = {.tv.nsec =  100 * NSEC_PER_MSEC };
static ktime_t kxsd9_seq_time = {.tv.nsec =  5 * NSEC_PER_MSEC };

static enum hrtimer_restart kxsd9_poll_timer(struct hrtimer *timer)
{
	struct kxsd9 *kxsd9;

	kxsd9 = container_of(timer, struct kxsd9, timer);

#ifdef CONFIG_ANDROID_POWER
	android_lock_suspend(&kxsd9->suspend_lock);
#endif

	schedule_work(&kxsd9->work.work);
	return HRTIMER_NORESTART;
}

int kxsd9_enable(struct kxsd9 *kxsd9,int on)
{
	if (!kxsd9->seq && !kxsd9->on) {
		hrtimer_start(&kxsd9->timer, kxsd9_seq_time, 
				HRTIMER_MODE_REL);
	}
	kxsd9->on = on;
	kxsd9->iseq = 0;
	if (on) {
		kxsd9->nseq = ARRAY_SIZE(kxsd9_enable_seq);
		kxsd9->seq = kxsd9_enable_seq;
	} else {
		kxsd9->nseq = ARRAY_SIZE(kxsd9_disable_seq);
		kxsd9->seq = kxsd9_disable_seq;
	}
	return 0;
}

#endif


static int kxsd9_i2c_read(struct i2c_client *client, unsigned id, char *buf, int len)
{
	int r;
	char outbuffer[2] = { 0, 0 };

	outbuffer[0] = id;
	// maejrep: Have to separate the "ask" and "read" chunks
	r = i2c_master_send(client, outbuffer, 1);
	if (r < 0) {
		printk(KERN_WARNING "%s: error asking for gsensor data at "
			"address %02x,%02x: %d\n",
			__func__, client->addr, id, r);
		return r;
	}
	mdelay(1);
	r = i2c_master_recv(client, buf, len);
	if (r < 0) {
		printk(KERN_ERR "%s: error reading gsensor data at "
			"address %02x,%02x: %d\n",
			__func__, client->addr, id, r);
		return r;
	}
	return 0;
}

static int kxsd9_i2c_write(struct i2c_client *client, unsigned id, char *buf)
{
	int r;
	char outbuffer[2] = { 0, 0 };

	outbuffer[0] = id ;
	outbuffer[1] = *buf ;

	printk("writing data=%02x at address %02x\n", outbuffer[1], outbuffer[0] );


	r = i2c_master_send(client, outbuffer, 2);
	if (r < 0) {
		printk(KERN_WARNING "%s: error writing gsensor data at "
			"address %02x,%02x: %d\n",
			__func__, client->addr, id, r);
		return r;
	}

	return 0;
}

int kxsd9_read_accel_xyz(struct kxsd9 *kxsd9, kxsd9acc_t * acc)
{
	int err;
	char buf[6];
	short x,y,z;

	err = kxsd9_i2c_read(kxsd9->client, 0, buf, 6);
	
	x = 0x8000 - 0x100 * buf[0] - buf[1];
	y = 0x8000 - 0x100 * buf[2] - buf[3];
	z = buf[4] * 0x100 + buf[5] - 0x8000 - 1000; // calib?

// Re-map the axes to account for different placement of sensor-IC
// in the different revisions of the device.
#if (CONFIG_SAMSUNG_OMAP4_TAB_REV <= 2)
	acc->x = x;
	acc->y = y;
	acc->z = z;
#elif (CONFIG_SAMSUNG_OMAP4_TAB_REV == 3)
	acc->x = y;
	acc->y = -x;
	acc->z = z;
#elif (CONFIG_SAMSUNG_OMAP4_TAB_REV == 4)
	acc->x = y;
	acc->y = x;
	acc->z = z;

#elif (CONFIG_SAMSUNG_OMAP4_TAB_REV == 5)
	acc->x = y;
	acc->y = -x;
	acc->z = z;
#endif

#if KXSD9_DEBUG
	printk("X:%d, Y:%d, Z:%d\n", acc->x, acc->y, acc->z);
#endif

	return err;
}

struct class *acc_class;


int kxsd9_open (struct inode *inode, struct file *filp)
{
	return 0;
}

ssize_t kxsd9_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
	return 0;
}

ssize_t kxsd9_write (struct file *filp, const char *buf, size_t count, loff_t *f_pos)
{
	return 0;
}

int kxsd9_release (struct inode *inode, struct file *filp)
{
	return 0;
}

void kxsd9_set_operation_mode(u8 mode)
{

	int ret;
	char tmp_buf;

	switch(mode)
	{
		//POWER_UP
		case 1 : 
		{	
			printk("KXSD9 Power-up\n");

			// Put KXSD9 in ON-mode
			tmp_buf = 0x40;
			ret = kxsd9_i2c_write(kxsd9->client, KXSD9_CTRL_REGB , &tmp_buf);
			if(ret<0) {
				printk("KXSD9 : POWER ON FAILED!!!! ret = %d\n", ret);
			}

			if(kxsd9_timeout < 0)
			{
				kxsd9_timeout=msecs_to_jiffies(200);
			}
		
			queue_delayed_work(kxsd9_work_q, &kxsd9_delayed_work_q, kxsd9_timeout);
			break;
		}

		//POWER_DOWN
		case 0 :
		{
			printk("KXSD( Power-down\n");
			kxsd9_timeout =-1;
			flush_workqueue(kxsd9_work_q);
		        cancel_delayed_work(&kxsd9_delayed_work_q);

			// Put KXSD9 in standby-mode
			tmp_buf = 0x00;
			ret = kxsd9_i2c_write(kxsd9->client, KXSD9_CTRL_REGB , &tmp_buf);

			break;
		}

	}

}

static int kxsd9_work(void)
{
	int ret =0;

	kxsd9acc_t data;
	ret = kxsd9_read_accel_xyz(kxsd9 , &data);

	if(ret < 0)
	{
		printk("kxsd9_read_accel_xyz FAILED\n");
		return (-1);
	}

#if KXSD9_DEBUG
	printk("KXSD9 : kxsd9_work called!! X=%d Y=%d Z=%d\n", data.x, data.y, data.z);
#endif

	input_report_abs(kxsd9_input_dev, ABS_X, data.y/16 );
	input_report_abs(kxsd9_input_dev, ABS_Y, data.x/16 );
	input_report_abs(kxsd9_input_dev, ABS_Z, data.z/16 );

	input_sync(kxsd9_input_dev);

	if(kxsd9_timeout > -1)
	{
		#if KXSD9_DEBUG
		printk("KXSD9 : queuing next at delay= %u\n", kxsd9_timeout);
		#endif
		queue_delayed_work(kxsd9_work_q, &kxsd9_delayed_work_q, kxsd9_timeout);
	}

	return 0;
}


/**************************sysfs-interface**************************/

static ssize_t poll_delay_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	long int delay_to_report;
	
	if (kxsd9_timeout<0)
	{
		delay_to_report = -1 ;
	}
	else
	{
		// millisecs to nanosecs
		delay_to_report = jiffies_to_msecs(kxsd9_timeout*1000000);
	}

#if KXSD9_DEBUG
	printk("delay_to_report=%ld\n", delay_to_report );
#endif
	return snprintf(buf, PAGE_SIZE, "%ld\n", delay_to_report);
}


static ssize_t poll_delay_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int err;
	long int new_delay;

#if KXSD9_DEBUG
	printk("buf=%s\n", buf);
	printk("size=%u\n", size);
#endif

	err = strict_strtol(buf, 10, &new_delay);
	if ( err < 0 )
		return err;

	// nanosecs to millisecs
	kxsd9_timeout = msecs_to_jiffies(new_delay/1000000);


#ifdef KXSD9_DEBUG
	printk("poll_delay_store called. new_delay=%ld\n", new_delay);
	printk("poll_delay_store called. kxsd9_timeout=%d\n", kxsd9_timeout);
#endif
	return size;
}


static ssize_t accel_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
#ifdef KXSD9_DEBUG
	printk("enabled=%d\n", ((kxsd9_timeout < 0) ? 0 : 1) );
#endif
	return snprintf(buf, PAGE_SIZE, "%d\n", ((kxsd9_timeout < 0) ? 0 : 1) );
}


static ssize_t accel_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
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

#if KXSD9_DEBUG
	printk("new_value = %d, old state = %d\n", new_value, ((kxsd9_timeout < 0) ? 0 : 1) );
#endif

	if (new_value && (kxsd9_timeout < 0) ) {
		kxsd9_set_operation_mode(1);

	} else if (!new_value && (kxsd9_timeout > -1) ) {
		kxsd9_set_operation_mode(0);
	}

	(ret < 0) ? (int)NULL : (ret = (int)size) ;

	return ret;
}


static struct device_attribute dev_attr_poll_delay =
	__ATTR(poll_delay, S_IRUGO | S_IWUGO | S_IXUGO,
	       poll_delay_show, poll_delay_store);

static struct device_attribute dev_attr_accel_enable =
	__ATTR(enable, S_IRUGO | S_IWUGO | S_IXUGO,
	       accel_enable_show, accel_enable_store);


static struct attribute *accel_sysfs_attrs[] = {
	&dev_attr_accel_enable.attr,
	&dev_attr_poll_delay.attr,
	NULL
};

static struct attribute_group accel_attribute_group = {
	.attrs = accel_sysfs_attrs,
};



/*****************************************************************/


#if CONFIG_PM

static int kxsd9_suspend(struct i2c_client * client, pm_message_t mesg)
{
	kxsd9_set_operation_mode(0);

	if (kxsd9_timeout > -1) {

		#if KXSD9_DEBUG
		printk(KERN_INFO MODULE_NAME ": suspending device...\n");
		#endif

		// Preserve the current kxsd9_timeout value
		// This is needed to re-initialise the workqueue
		// at the same poll_delay rate after resuming.
		timeout_at_suspend = kxsd9_timeout;

		shutdown_at_suspend = 1;
	}
	else {
			shutdown_at_suspend = 0;
	}

	return 0;
}

static int kxsd9_resume(struct i2c_client * client)
{

	if (shutdown_at_suspend) {

		#if KXSD9_DEBUG
		printk(KERN_INFO MODULE_NAME ": resuming device...\n");
		#endif

		shutdown_at_suspend = 0;

		kxsd9_timeout = timeout_at_suspend;
		kxsd9_set_operation_mode(1);
	}
	return 0;
}

#else

#define kxsd9_suspend NULL
#define kxsd9_resume NULL

#endif


int kxsd9_ioctl(struct inode *inode, struct file *filp, unsigned int cmd,  unsigned long arg)
{
	int err = 0;
	unsigned char data[6];

	/* check cmd */
	if(_IOC_TYPE(cmd) != KXSD9_IOC_MAGIC)
	{
#if KXSD9_DEBUG      
		printk("cmd magic type error\n");
#endif
		return -ENOTTY;
	}
	if(_IOC_NR(cmd) > KXSD9_IOC_MAXNR)
	{
#if KXSD9_DEBUG
		printk("cmd number error\n");
#endif
		return -ENOTTY;
	}

	if(err)
	{
#if KXSD9_DEBUG
		printk("cmd access_ok error\n");
#endif
		return -EFAULT;
	}


	switch(cmd)
	{
		case KXSD9_READ_ACCEL_XYZ:
			err = kxsd9_read_accel_xyz(kxsd9 , (kxsd9acc_t*)data);
			if(copy_to_user((kxsd9acc_t*)arg,(kxsd9acc_t*)data,6)!=0)
			{
#if KXSD9_DEBUG
				printk("copy_to error\n");
#endif
				return -EFAULT;
			}
			return err;

		case KXSD9_SET_RANGE:
			if(copy_from_user(data,(unsigned char*)arg,1)!=0)
			{
#if KXSD9_DEBUG           
				printk("[KXSD9] copy_from_user error\n");
#endif
				return -EFAULT;
			}
			return err;
		
		case KXSD9_SET_MODE:
			if(copy_from_user(data,(unsigned char*)arg,1)!=0)
			{
#if KXSD9_DEBUG           
				printk("[KXSD9] copy_from_user error\n");
#endif
				return -EFAULT;
			}
			return err;

		case KXSD9_SET_BANDWIDTH:
			if(copy_from_user(data,(unsigned char*)arg,1)!=0)
			{
#if KXSD9_DEBUG
				printk("[KXSD9] copy_from_user error\n");
#endif
				return -EFAULT;
			}
			return err;
		
		default:
			return 0;
	}
}


struct file_operations acc_fops =
{
	.owner   = THIS_MODULE,
	.read    = kxsd9_read,
	.write   = kxsd9_write,
	.open    = kxsd9_open,
	.ioctl   = kxsd9_ioctl,
	.release = kxsd9_release,
};


static int kxsd9_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int result;

	struct device *dev_t;

#if KXSD9_DEBUG
	printk("Initializing Kionix KXSD9 driver "
		"at addr: 0x%02x\n", client->addr);
#endif

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		printk(KERN_ERR MODULE_NAME ": i2c bus not supported\n");
		return -EINVAL;
	}

	kxsd9 = kzalloc(sizeof *kxsd9, GFP_KERNEL);
	if (kxsd9 < 0) {
		printk(KERN_ERR MODULE_NAME ": Not enough memory\n");
		return -ENOMEM;
	}

	mutex_init(&kxsd9->lock);
	kxsd9->client = client;
	i2c_set_clientdata(client, kxsd9);

	result = register_chrdev( 0, ACC_DEV_NAME, &acc_fops);
	if (result < 0) {
		return result;
	}
	
        acc_class = class_create (THIS_MODULE, ACC_DEV_NAME);

        if (IS_ERR(acc_class))	{
		unregister_chrdev( result, ACC_DEV_NAME);
		return PTR_ERR( acc_class );
	}


	dev_t = device_create( acc_class, NULL, MKDEV(result, 0), "%s", ACC_DEV_NAME);
        if (IS_ERR(dev_t))
        {
                return PTR_ERR(dev_t);
        }

	//input-device initialisation
	kxsd9_input_dev = input_allocate_device();
	set_bit(EV_ABS, kxsd9_input_dev->evbit);
	set_bit(ABS_X, kxsd9_input_dev->absbit);
	set_bit(ABS_Y, kxsd9_input_dev->absbit);
	set_bit(ABS_Z, kxsd9_input_dev->absbit);

	kxsd9_input_dev->name = "kxsd9_accelerometer";

	input_set_abs_params(kxsd9_input_dev, ABS_X, -3000, 3000, 0, 0);
	input_set_abs_params(kxsd9_input_dev, ABS_Y, -3000, 3000, 0, 0);
	input_set_abs_params(kxsd9_input_dev, ABS_Z, -3000, 3000, 0, 0);
	
	result = input_register_device(kxsd9_input_dev);
  	if (result)
	{
    		printk("Accel Sensor couldn't be registered as input device: %d\n", result);
    		goto release_input_dev;
  	}

	result = sysfs_create_group(&kxsd9_input_dev->dev.kobj, &accel_attribute_group);
	if (result) {
		pr_err("%s: could not create sysfs group\n", __func__);
		goto release_input_dev;
	}

	kxsd9->kxsd9_early_suspend.suspend = kxsd9_suspend;
        kxsd9->kxsd9_early_suspend.resume = kxsd9_resume;
        register_early_suspend(&kxsd9->kxsd9_early_suspend);

	//Workqueue Initialisation
	kxsd9_work_q = create_singlethread_workqueue("accel_sensor_kxsd9_work_queue");
	INIT_DELAYED_WORK( (struct delayed_work *)&kxsd9_delayed_work_q, kxsd9_work);

	return 0;

release_input_dev:
	input_free_device(kxsd9_input_dev);
	return result;
}


static int kxsd9_remove(struct i2c_client * client)
{
	struct kxsd9 *kxsd9 = i2c_get_clientdata(client);
	unregister_early_suspend(&kxsd9->kxsd9_early_suspend);
	input_unregister_device(kxsd9->inputdev);
	input_free_device(kxsd9->inputdev);

#ifdef CONFIG_ANDROID_POWER
	android_uninit_suspend_lock(&kxsd9->suspend_lock);	
#endif
	
	kfree(kxsd9);
	return 0;
}

static const struct i2c_device_id kxsd9_ids[] = {
        { MODULE_NAME, 0 },
        { },
};
MODULE_DEVICE_TABLE(i2c, kxsd9_ids);

static struct i2c_driver kxsd9_driver = {
	.driver = {
		.name	= MODULE_NAME,
		.owner	= THIS_MODULE,
	},
	.id_table = kxsd9_ids,
	.probe = kxsd9_probe,
	.remove = kxsd9_remove,
#if CONFIG_PM
//	.suspend = kxsd9_suspend,
//	.resume = kxsd9_resume,
#endif
};


static int __init kxsd9_init(void)
{
#if KXSD9_DEBUG
	printk(KERN_INFO MODULE_NAME ": Registering Kionix KXSD9 driver\n");
#endif
	return i2c_add_driver(&kxsd9_driver);
}

static void __exit kxsd9_exit(void)
{
#if KXSD9_DEBUG
	printk(KERN_INFO MODULE_NAME ": Unregistered Kionix KXSD9 driver\n");
#endif
	i2c_del_driver(&kxsd9_driver);
}

MODULE_AUTHOR("Job Bolle");
MODULE_DESCRIPTION("Kionix-KXSD9 Driver");
MODULE_LICENSE("GPL");

module_init(kxsd9_init);
module_exit(kxsd9_exit);

