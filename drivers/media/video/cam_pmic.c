/*
 * drivers/media/video/mt9p012.c
 *
 * mt9p012 sensor driver
 *
 *
 * Copyright (C) 2008 Texas Instruments.
 *
 * Leverage OV9640.c
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *****************************************************
 *****************************************************
 * modules/camera/cam_pmic.c
 *
 * Camera PMIC driver source file
 *
 * Modified by paladin in Samsung Electronics
 *
 * Modified by Srinidhi Rao (srinidhi.rao@samsung.com) to 
 * port it on to Samsung OMAP4 Universal board
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <plat/gpio.h>
#include <plat/hardware.h>

#include "cam_pmic.h"

#define CAM_DBG 0x00000400
#define CAM_INF 0x00000800
#define CAM_ERR 0x00000100
#define CAM_WRN 0x00000200
#if 1
#define dprintk(x, y...)        printk("["#x"] "y);
#else
#define dprintk(x, y...)        
#endif

static struct cam_pmic pmic;
static struct i2c_driver cam_pmic_i2c_driver;

int cam_pmic_read_reg(u8 reg, u8* val)
{
  struct cam_pmic * camera_pmic = &pmic;
  struct i2c_client *client = camera_pmic->i2c_client;

  int err;
  struct i2c_msg msg[1];
  unsigned char data[2];

  printk(KERN_ERR"\ncam_pmic_read_reg is called...\n");

  if (!client->adapter)
    return -ENODEV;

  msg->addr = client->addr;
  msg->flags =  0;
  msg->len = 1;
  msg->buf = data;

  /* high byte goes out first */
  data[0] = (u8) reg;
  err = i2c_transfer(client->adapter, msg, 1);
  if (err >= 0) {
    msg->len = 1;
    msg->flags = I2C_M_RD;
    err = i2c_transfer(client->adapter, msg, 1);
  }

  if (err >= 0) {
    *val = data[0];

    printk(KERN_ERR" SUCCESS Value from Reg:0x%x ==> 0x%x\n", reg, *val);
    return 0;
  }

  printk(KERN_ERR"ERROR read from Reg:0x%x error %d\n", reg, err);

  return err;
}
EXPORT_SYMBOL(cam_pmic_read_reg);

int cam_pmic_write_reg(u8 reg, u8 val)
{
  struct cam_pmic * camera_pmic = &pmic;
  struct i2c_client *client = camera_pmic->i2c_client;

  int err;
  struct i2c_msg msg[1];
  unsigned char data[2];
  int retry = 0;

  printk(KERN_ERR"\n\t\tcam_pmic_write_reg is called...\n");

  if (!client->adapter)
  {
    printk("no i2c client adapter!!");
    return -ENODEV;
  }

  again:
  msg->addr = client->addr;
  msg->flags = 0;
  msg->len = 2;
  msg->buf = data;

  /* high byte goes out first */
  data[0] = reg;
  data[1] = val;

  err = i2c_transfer(client->adapter, msg, 1);
  if (err >= 0)
  {
    printk(KERN_ERR"wrote 0x%x to offset 0x%x success!\n", val, reg);
    return 0;
  }

  printk(KERN_ERR"wrote 0x%x to offset 0x%x error %d\n", val, reg, err);
  if (retry <= CAM_PMIC_I2C_RETRY) {
    printk(KERN_ERR"retry ... %d\n", retry);	
    retry++;
    set_current_state(TASK_UNINTERRUPTIBLE);
    schedule_timeout(msecs_to_jiffies(20));
    goto again;
  }
  return err;
}
EXPORT_SYMBOL(cam_pmic_write_reg);

int camera_front_m5mo_on_off(int on_off)
{
	u32 val;
	void __iomem *frefclk1_base = NULL, *frefclk1_src = NULL;

	frefclk1_base = ioremap(0x4A30A314, 4);
	frefclk1_src = ioremap(0x4A008154, 4);

	if (on_off) {
		gpio_direction_output(43, 0);
		gpio_direction_output(139, 0);

		/* Buck voltage --> cam_isp_core_1.2v --> 1.2v core; Nodelay 1.2v */
		cam_pmic_write_reg(0x06, 0x09);
		cam_pmic_write_reg(0x07, 0x09);

		/* LDO3 vlotage --> cam_sensor_io_1.8v --> i_host_1.8v; 1Ts delay */
		cam_pmic_write_reg(0x03, 0x2C);

		/* Now power for the sensor */
		/* LDO1 voltage --> cam_sensor_core_1.2v --> s_core_1.2v; 1Ts delay */
		cam_pmic_write_reg(0x01, 0x20);

		/* LDO2 voltage 1.5; 1Ts delay*/
		cam_pmic_write_reg(0x02, 0x26);

		/* sensor 1.8 voltage is given by pmic_en; no need to further program */
		/* LDO4 voltage; cam_sensor_a2.8v --> s_vana_2.8v; 1Ts delay */
		cam_pmic_write_reg(0x04, 0x3E);

		/* LDO 5 voltage ; for AF; 1Ts delay */
		cam_pmic_write_reg(0x05, 0x39);

		/* Enable all the voltages */
		cam_pmic_write_reg(0x08, 0xBF);

		/* Enable the PMIC to output the voltages */
		gpio_direction_output(43, 1);
		mdelay(5);

		/* Set the source clock */
		val = __raw_readl(frefclk1_src) | (1 << 8);
		__raw_writel(val, frefclk1_src);
		val = __raw_readl(frefclk1_src);
		printk(KERN_ERR"SRC_CLK value is %x\n",val);

		val = __raw_readl(frefclk1_base) | ((0x1 << 8) | (0x1 << 2) | (0xF << 16));
		__raw_writel(val, frefclk1_base);
		val = __raw_readl(frefclk1_base);
		printk(KERN_ERR"MCLK AUXCLK1 is %x\n",val);
		/* Delay for mclk to settle down */
		mdelay(5);

		/* Release the reset */
		gpio_direction_output(139, 1);
		mdelay(5);
	} else {
		val = 0;
		gpio_direction_output(43, 1);
		gpio_direction_output(139, 1);
		/* Hold the reset low*/
		gpio_direction_output(139, 0);
		/* Disable mclk */
		__raw_writel(val, frefclk1_base);
		udelay(10);
		/* Turn off the supplies */
		cam_pmic_write_reg(0x08, 0x80);

		/* Disable PMIC */
		gpio_direction_output(43, 0);
	}

	iounmap(frefclk1_base);
	iounmap(frefclk1_src);

	return 0;
}

/**
 * cam_pmic_probe - sensor driver i2c probe handler
 * @client: i2c driver client device structure
 *
 * Register sensor as an i2c client device and V4L2
 * device.
 */
static int
cam_pmic_probe(struct i2c_client *client, const struct i2c_device_id *device)
{
  printk(KERN_ERR"\n\n\t\t ##### cam_pmic_probe is called...######\n");

  if (i2c_get_clientdata(client))
  {
    printk("can't get i2c client data!!\n");
    return -EBUSY;
  }

  pmic.i2c_client = client;
  i2c_set_clientdata(client, &pmic);

  return 0;
}

/**
 * cam_pmic_remove - sensor driver i2c remove handler
 * @client: i2c driver client device structure
 *
 * Unregister sensor as an i2c client device and V4L2
 * device.  Complement of m4mo_probe().
 */
static int __exit
cam_pmic_remove(struct i2c_client *client)
{
  dprintk(CAM_INF, "cam_pmic_remove is called...\n");
  
  if (!client->adapter)
  {
    printk("no i2c client adapter!!");
    return -ENODEV;
  }

  i2c_set_clientdata(client, NULL);

  return 0;
}

static const struct i2c_device_id cam_pmic_id[] = {
  { CAM_PMIC_DRIVER_NAME, 0 },
  { },
};
MODULE_DEVICE_TABLE(i2c, cam_pmic_id);

static struct i2c_driver cam_pmic_i2c_driver = {
  .driver = {
    .name = CAM_PMIC_DRIVER_NAME,
    .owner = THIS_MODULE,
  },
  .probe = cam_pmic_probe,
  .remove = __exit_p(cam_pmic_remove),
  .id_table = cam_pmic_id,
};



/**
 * cam_pmic_init - sensor driver module_init handler
 *
 * Registers driver as an i2c client driver.  Returns 0 on success,
 * error code otherwise.
 */
static int __init cam_pmic_init(void)
{
  int err;

  dprintk(CAM_INF, "cam_pmic_init is called...\n");

  err = i2c_add_driver(&cam_pmic_i2c_driver);
  if (err) {
    dprintk(CAM_ERR, CAM_PMIC_MOD_NAME "Failed to register" CAM_PMIC_DRIVER_NAME ".\n");
    return err;
  }
  return 0;
}
module_init(cam_pmic_init);

/**
 * cam_pmic_cleanup - sensor driver module_exit handler
 *
 * Unregisters/deletes driver as an i2c client driver.
 * Complement of m4mosensor_init.
 */
static void __exit cam_pmic_cleanup(void)
{
  i2c_del_driver(&cam_pmic_i2c_driver);
}
module_exit(cam_pmic_cleanup);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Camera PMIC Driver");
