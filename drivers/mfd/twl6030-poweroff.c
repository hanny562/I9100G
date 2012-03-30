/*
 * /drivers/mfd/twl6030-poweroff.c
 *
 * Power off device
 *
 * Copyright (C) 2010 Texas Instruments Corporation
 *
 * Written by Rajeev Kulkarni <rajeevk@ti.com>
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License. See the file "COPYING" in the main directory of this
 * archive for more details.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/pm.h>
#include <linux/i2c/twl.h>
#include <linux/delay.h>
#include <mach/gpio.h>
#include <linux/i2c/twl.h>
#include <plat/mux.h>
#include <linux/reboot.h>


#define TWL6030_PHONIX_DEV_ON	0x25

#define CON_DEVON	(1<<4) 	 
#define APP_DEVOFF	(1<<0)
#define CON_DEVOFF	(1<<1)
#define MOD_DEVOFF	(1<<2)


 
void twl6030_poweroff(void)
{
	u8 uninitialized_var(val);
	int err;

	err = twl_i2c_read_u8(TWL6030_MODULE_ID0, &val,
				  TWL6030_PHONIX_DEV_ON);
	if (err) {
		pr_warning("I2C error %d reading PHONIX_DEV_ON\n", err);
		return;
	}

	val |= CON_DEVON;

	err = twl_i2c_write_u8(TWL6030_MODULE_ID0, val,
				   TWL6030_PHONIX_DEV_ON);
	mdelay(30);

	val |= APP_DEVOFF | CON_DEVOFF | MOD_DEVOFF;

	err = twl_i2c_write_u8(TWL6030_MODULE_ID0, val,
				   TWL6030_PHONIX_DEV_ON);

	if (err) {
		pr_warning("I2C error %d writing PHONIX_DEV_ON\n", err);
		return;
	}

	return;
}

static void board_poweroff(void)
{
	int otg_id_status = 1;

#if (CONFIG_SAMSUNG_REL_HW_REV >= 3)
	otg_id_status = gpio_get_value( OMAP_GPIO_USB_OTG_ID );
#endif

	printk("\nPower off routine - Board Shutdown!! TA = %d, JIG = %d, OTG_ID = %d\n", 
		   gpio_get_value( OMAP_GPIO_TA_NCONNECTED ), gpio_get_value( OMAP_GPIO_JIG_ON18 ),
		   otg_id_status);

	if ( otg_id_status &&
		 (!gpio_get_value( OMAP_GPIO_TA_NCONNECTED ) || !gpio_get_value( OMAP_GPIO_JIG_ON18 )) ){
		printk("Warmreset by TA or USB or Jtag - check 2 pins : JIG_ON18, TA_nConnected \n\n");

		preempt_disable();
		local_irq_disable();
		local_fiq_disable();

		machine_restart("ta_inserted");
		while(1);
	} else {
		while(1)
		{
			if (gpio_get_value(OMAP_GPIO_KEY_PWRON)) {
				printk("Power button is pressed\n\n");
			} else {
				printk("Power Off !\n\n");
				twl6030_poweroff();
			}
		}

	}
}

static int __init twl6030_poweroff_init(void)
{
	pm_power_off = board_poweroff;

	return 0;
}

static void __exit twl6030_poweroff_exit(void)
{
	pm_power_off = NULL;
}

module_init(twl6030_poweroff_init);
module_exit(twl6030_poweroff_exit);

MODULE_DESCRIPTION("Triton2 device power off");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Rajeev Kulkarni");
