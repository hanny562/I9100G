#include <linux/module.h>
#include <linux/pm.h>
#include <linux/i2c/twl.h>
#include <linux/reboot.h>

#include <plat/gpio.h>
#include <plat/mux.h>

#define PHOENIX_DEV_ON		0x06
#define DEV_OFF			0x07

static void twl6030_power_off(void)
{
	u8 val = 0;
	int err = 0, i = 0;

	val = DEV_OFF;
	err = twl_i2c_write_u8(TWL_MODULE_PM_MASTER, val, PHOENIX_DEV_ON);
	if(err){
		printk(KERN_ERR"\n Error while trying to write PHOENIX_DEV_ON register,err=%d\n",err);
		return;
	}
}

static void samsung_omap4_power_off(void)
{
	printk(KERN_ERR"\n Device Going to POWER OFF \n");
	twl6030_power_off();
	return;
}

static int __init twl6030_power_off_init(void)
{
	pm_power_off = &samsung_omap4_power_off;
	return 0;
}
module_init(twl6030_power_off_init);

static void __exit twl6030_power_off_exit(void)
{
	pm_power_off = NULL;
}
module_exit(twl6030_power_off_exit);

MODULE_ALIAS("TWL6030 POWER OFF");
MODULE_DESCRIPTION("PHOENIX DEVICE SHUT DOWN");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("SRINIDHI RAO srinidhi.rao@samsung.com");

