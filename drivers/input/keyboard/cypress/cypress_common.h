#ifndef __CYPRESS_DEBUG_H__
#define __CYPRESS_DEBUG_H__

struct i2c_touchkey_driver {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct early_suspend early_suspend;
};


//#define CONFIG_CYPRESS_TOUCHKEY_DEBUG

#ifdef CONFIG_CYPRESS_TOUCHKEY_DEBUG
#define dbg(args...)	{	\
				printk("[CYPRESS TOUCH] %s: ", __FUNCTION__);	\
				printk(args);	\
			}
#else
#define dbg(args...)
#endif

#endif /* __CYPRESS_DEBUG_H__ */

