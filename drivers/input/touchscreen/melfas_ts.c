/* drivers/input/touchscreen/melfas_ts.c
 *
 * Copyright (C) 2010 Melfas, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */


#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/melfas_ts.h>
#include <linux/gpio.h>	
#include <plat/control.h>

#define OMAP_MUX_MODE0      0
#define OMAP_MUX_MODE3      3
#define OMAP_PULL_ENA			(1 << 3)
#define OMAP_PULL_UP			(1 << 4)
#define OMAP_INPUT_EN			(1 << 8)
#define OMAP_PIN_INPUT_PULLUP		(OMAP_PULL_ENA | OMAP_INPUT_EN | OMAP_PULL_UP)
#define OMAP_PIN_INPUT_PULLDOWN		(OMAP_PULL_ENA | OMAP_INPUT_EN)

#define MELFAS_MAX_TOUCH       10
#define FW_VERSION             0x01
#define HW_VERSION			   0x00

#define TS_MAX_X_COORD         1023
#define TS_MAX_Y_COORD         599
#define TS_MAX_Z_TOUCH         255
#define TS_MAX_W_TOUCH         30

#define TS_READ_START_ADDR 	   0x10
#define TS_READ_VERSION_ADDR   0x53
#define TS_READ_REGS_LEN       56

#define I2C_RETRY_CNT			10

#define	SET_DOWNLOAD_BY_GPIO	1

#define PRESS_KEY				1
#define RELEASE_KEY				0

#define DEBUG_PRINT 			0


#if SET_DOWNLOAD_BY_GPIO
#include "melfas_download.h"
#endif // SET_DOWNLOAD_BY_GPIO

#define SET_TOUCH_I2C()				omap4_ctrl_pad_writel(((OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP) << 16) | (OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP), 0x012A)
#define SET_TOUCH_I2C_TO_GPIO()		omap4_ctrl_pad_writel(((OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP) << 16) | (OMAP_MUX_MODE3 | OMAP_PIN_INPUT_PULLUP), 0x012A)

struct muti_touch_info
{
    int state;
    int strength;
    int width;
    int posX;
    int posY;
};

struct melfas_ts_data
{
    uint16_t addr;
    struct i2c_client *client;
    struct input_dev *input_dev;
    struct work_struct  work;
    uint32_t flags;
    int (*power)(int on);
    struct early_suspend early_suspend;
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void melfas_ts_early_suspend(struct early_suspend *h);
static void melfas_ts_late_resume(struct early_suspend *h);
#endif

static struct muti_touch_info g_Mtouch_info[MELFAS_MAX_TOUCH];

static int melfas_init_panel(struct melfas_ts_data *ts)
{

//    ret = i2c_master_send(ts->client, &buf, 1);

//    if (ret < 0)
//    {
//        printk(KERN_ERR "melfas_ts_probe: i2c_master_send() failed\n [%d]", ret);
//        return 0;
//    }


    return true;
}

static void melfas_ts_work_func(struct work_struct *work)
{
    struct melfas_ts_data *ts = container_of(work, struct melfas_ts_data, work);
    int ret = 0, i;
    uint8_t buf[TS_READ_REGS_LEN];
    int touchNumber = 0, touchPosition = 0, posX = 0, posY = 0, width = 0, strength = 0;
    int keyEvent = 0, keyState = 0, keyID = 0, keystrength = 0;

#if DEBUG_PRINT
    printk(KERN_ERR "melfas_ts_work_func\n");

    if (ts == NULL)
        printk(KERN_ERR "melfas_ts_work_func : TS NULL\n");
#endif


    /**
    Simple send transaction:
    	S Addr Wr [A]  Data [A] Data [A] ... [A] Data [A] P
    Simple recv transaction:
    	S Addr Rd [A]  [Data] A [Data] A ... A [Data] NA P
    */

    buf[0] = TS_READ_START_ADDR;
    for (i = 0; i < I2C_RETRY_CNT; i++)
    {
        ret = i2c_master_send(ts->client, buf, 1);
#if DEBUG_PRINT
        printk(KERN_ERR "melfas_ts_work_func : i2c_master_send [%d]\n", ret);
#endif
        if (ret >= 0)
        {
            ret = i2c_master_recv(ts->client, buf, TS_READ_REGS_LEN);
#if DEBUG_PRINT
            printk(KERN_ERR "melfas_ts_work_func : i2c_master_recv [%d]\n", ret);
#endif
            if (ret >= 0)
                break; // i2c success
        }
    }


    if (ret < 0)
    {
        printk(KERN_ERR "melfas_ts_work_func: i2c failed\n");
        enable_irq(ts->client->irq);
        return ;
    }
    else // Five Multi Touch Interface
    {
        touchNumber = buf[0] & 0x0F;
        touchPosition = buf[1] & 0xFF;

        for (i = 0; i < MELFAS_MAX_TOUCH; i++)
        {
            g_Mtouch_info[i].posX = TS_MAX_X_COORD - (((buf[2 + 5*i] & 0x0F) << 8) + buf[4 + 5*i]);
            g_Mtouch_info[i].posY = ((buf[2 + 5*i] >> 4)   << 8) + buf[3 + 5*i];
            g_Mtouch_info[i].width = buf[5 + 5*i];
            g_Mtouch_info[i].strength = buf[6 + 5*i];

            if (g_Mtouch_info[i].width != 0)
            {
                g_Mtouch_info[i].state = 1;
            }
            else
            {
                g_Mtouch_info[i].state = 0;
            }
        }

        keyID = buf[5*MELFAS_MAX_TOUCH + 2] & 0x07;
        keyState = (buf[5*MELFAS_MAX_TOUCH + 2] >> 3) & 0x01;
        keyEvent = (buf[5*MELFAS_MAX_TOUCH + 2] >> 4) & 0x01;
        keystrength = (buf[5*MELFAS_MAX_TOUCH + 3]);

        if (touchNumber > MELFAS_MAX_TOUCH)
        {
#if DEBUG_PRINT
            printk(KERN_ERR "melfas_ts_work_func: Touch ID: %d\n",  touchNumber);
#endif
            enable_irq(ts->client->irq);
            return;
        }

        for (i = 0; i < MELFAS_MAX_TOUCH; i++)
        {
            //if ((g_Mtouch_info[i].posX == 0) || (g_Mtouch_info[i].posY == 0))
	if(g_Mtouch_info[i].strength== -1)
                continue;

            input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, i);
            input_report_abs(ts->input_dev, ABS_MT_POSITION_X, g_Mtouch_info[i].posX);
            input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, g_Mtouch_info[i].posY);
            input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, g_Mtouch_info[i].strength);
            input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, g_Mtouch_info[i].width);
            input_mt_sync(ts->input_dev);
#if DEBUG_PRINT
            printk(KERN_ERR "melfas_ts_work_func: Touch ID: %d, x: %d, y: %d, z: %d w: %d\n",
                   i, g_Mtouch_info[i].posX, g_Mtouch_info[i].posY, g_Mtouch_info[i].strength, g_Mtouch_info[i].width);
#endif
				if (g_Mtouch_info[i].strength == 0)
					g_Mtouch_info[i].strength = -1;
        }
        if (keyEvent)
        {
            if (keyID == 0x1)
                input_report_key(ts->input_dev, KEY_MENU, keyState ? PRESS_KEY : RELEASE_KEY);
            if (keyID == 0x2)
                input_report_key(ts->input_dev, KEY_HOME, keyState ? PRESS_KEY : RELEASE_KEY);
            if (keyID == 0x3)
                input_report_key(ts->input_dev, KEY_BACK, keyState ? PRESS_KEY : RELEASE_KEY);
            if (keyID == 0x4)
                input_report_key(ts->input_dev, KEY_SEARCH, keyState ? PRESS_KEY : RELEASE_KEY);
#if DEBUG_PRINT
            printk(KERN_ERR "melfas_ts_work_func: keyID : %d, keyState: %d\n", keyID, keyState);
#endif
        }

        input_sync(ts->input_dev);
    }

    enable_irq(ts->client->irq);
}

static irqreturn_t melfas_ts_irq_handler(int irq, void *handle)
{
    struct melfas_ts_data *ts = (struct melfas_ts_data *)handle;
#if DEBUG_PRINT
    printk(KERN_ERR "melfas_ts_irq_handler\n");
#endif

    disable_irq_nosync(ts->client->irq);
    schedule_work(&ts->work);

    return IRQ_HANDLED;
}

static int melfas_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct melfas_ts_data *ts;
    int ret = 0, i;

    uint8_t buf[2];

#if DEBUG_PRINT
    printk(KERN_ERR "kim ms : melfas_ts_probe\n");
#endif

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        printk(KERN_ERR "melfas_ts_probe: need I2C_FUNC_I2C\n");
        ret = -ENODEV;
        goto err_check_functionality_failed;
    }

    ts = kmalloc(sizeof(struct melfas_ts_data), GFP_KERNEL);
    if (ts == NULL)
    {
        printk(KERN_ERR "melfas_ts_probe: failed to create a state of melfas-ts\n");
        ret = -ENOMEM;
        goto err_alloc_data_failed;
    }

    INIT_WORK(&ts->work, melfas_ts_work_func);

    ts->client = client;
    i2c_set_clientdata(client, ts);
    ret = i2c_master_send(ts->client, &buf, 1);


#if DEBUG_PRINT
    printk(KERN_ERR "melfas_ts_probe: i2c_master_send() [%d], Add[%d]\n", ret, ts->client->addr);
#endif



#if SET_DOWNLOAD_BY_GPIO
    buf[0] = TS_READ_VERSION_ADDR;
    for (i = 0; i < I2C_RETRY_CNT; i++)
    {
        ret = i2c_master_send(ts->client, buf, 1);
        if (ret >= 0)
        {
            ret = i2c_master_recv(ts->client, buf, 2);

            if (ret >= 0)
            {
            	printk("melfas_ts_probe: i2c success\n");
            	printk("buf[0] = %x, buf[0] = %x\n", buf[0], buf[1]);
                break; // i2c success
            }
        }
    }

	if(i == I2C_RETRY_CNT) //VERSION READ Fail
	{
		 //ret = mcsdl_download_binary_file();
		SET_TOUCH_I2C_TO_GPIO();
		ret = mcsdl_download_binary_data();
		SET_TOUCH_I2C();
	}
	else {
		if (buf[0] == HW_VERSION && buf[1] < FW_VERSION)
		{
			//ret = mcsdl_download_binary_file();
			SET_TOUCH_I2C_TO_GPIO();
			ret = mcsdl_download_binary_data();
			SET_TOUCH_I2C();
		}
	}
#endif // SET_DOWNLOAD_BY_GPIO

    ts->input_dev = input_allocate_device();
    if (!ts->input_dev)
    {
        printk(KERN_ERR "melfas_ts_probe: Not enough memory\n");
        ret = -ENOMEM;
        goto err_input_dev_alloc_failed;
    }

    ts->input_dev->name = "melfas-ts" ;

    ts->input_dev->evbit[0] = BIT_MASK(EV_ABS) | BIT_MASK(EV_KEY);


    ts->input_dev->keybit[BIT_WORD(KEY_MENU)] |= BIT_MASK(KEY_MENU);
    ts->input_dev->keybit[BIT_WORD(KEY_HOME)] |= BIT_MASK(KEY_HOME);
    ts->input_dev->keybit[BIT_WORD(KEY_BACK)] |= BIT_MASK(KEY_BACK);
    ts->input_dev->keybit[BIT_WORD(KEY_SEARCH)] |= BIT_MASK(KEY_SEARCH);


//	__set_bit(BTN_TOUCH, ts->input_dev->keybit);
//	__set_bit(EV_ABS,  ts->input_dev->evbit);
//	ts->input_dev->evbit[0] =  BIT_MASK(EV_SYN) | BIT_MASK(EV_ABS) | BIT_MASK(EV_KEY);

    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, TS_MAX_X_COORD, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, TS_MAX_Y_COORD, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, TS_MAX_Z_TOUCH, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, MELFAS_MAX_TOUCH - 1, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, TS_MAX_W_TOUCH, 0, 0);
//	__set_bit(EV_SYN, ts->input_dev->evbit);
//	__set_bit(EV_KEY, ts->input_dev->evbit);


    ret = input_register_device(ts->input_dev);
    if (ret)
    {
        printk(KERN_ERR "melfas_ts_probe: Failed to register device\n");
        ret = -ENOMEM;
        goto err_input_register_device_failed;
    }

    if (ts->client->irq)
    {
#if DEBUG_PRINT
        printk(KERN_ERR "melfas_ts_probe: trying to request irq: %s-%d\n", ts->client->name, ts->client->irq);
#endif
        ret = request_irq(client->irq, melfas_ts_irq_handler, IRQF_TRIGGER_FALLING, ts->client->name, ts);
        if (ret > 0)
        {
            printk(KERN_ERR "melfas_ts_probe: Can't allocate irq %d, ret %d\n", ts->client->irq, ret);
            ret = -EBUSY;
            goto err_request_irq;
        }
    }

    schedule_work(&ts->work);

	for (i = 0; i < MELFAS_MAX_TOUCH ; i++)  /* _SUPPORT_MULTITOUCH_ */
		g_Mtouch_info[i].strength = -1;	

#if DEBUG_PRINT
    printk(KERN_ERR "melfas_ts_probe: succeed to register input device\n");
#endif

#if CONFIG_HAS_EARLYSUSPEND
    ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
    ts->early_suspend.suspend = melfas_ts_early_suspend;
    ts->early_suspend.resume = melfas_ts_late_resume;
    register_early_suspend(&ts->early_suspend);
#endif

#if DEBUG_PRINT
    printk(KERN_INFO "melfas_ts_probe: Start touchscreen. name: %s, irq: %d\n", ts->client->name, ts->client->irq);
#endif
    return 0;

err_request_irq:
    printk(KERN_ERR "melfas-ts: err_request_irq failed\n");
    free_irq(client->irq, ts);
err_input_register_device_failed:
    printk(KERN_ERR "melfas-ts: err_input_register_device failed\n");
    input_free_device(ts->input_dev);
err_input_dev_alloc_failed:
    printk(KERN_ERR "melfas-ts: err_input_dev_alloc failed\n");
err_alloc_data_failed:
    printk(KERN_ERR "melfas-ts: err_alloc_data failed_\n");
err_detect_failed:
    printk(KERN_ERR "melfas-ts: err_detect failed\n");
    kfree(ts);
err_check_functionality_failed:
    printk(KERN_ERR "melfas-ts: err_check_functionality failed_\n");

    return ret;
}

static int melfas_ts_remove(struct i2c_client *client)
{
    struct melfas_ts_data *ts = i2c_get_clientdata(client);

    unregister_early_suspend(&ts->early_suspend);
    free_irq(client->irq, ts);
    input_unregister_device(ts->input_dev);
    kfree(ts);
    return 0;
}

static int melfas_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
    int ret;
	int i;    
    struct melfas_ts_data *ts = i2c_get_clientdata(client);

	for (i = 0; i < MELFAS_MAX_TOUCH ; i++)
	{
		g_Mtouch_info[i].strength = -1;
		g_Mtouch_info[i].posX = 0;
		g_Mtouch_info[i].posY = 0;
		g_Mtouch_info[i].width = 0;
	}
    
    disable_irq(client->irq);

    ret = cancel_work_sync(&ts->work);
    if (ret) /* if work was pending disable-count is now 2 */
        enable_irq(client->irq);

//	ret = i2c_smbus_write_byte_data(client, 0x01, 0x00); /* deep sleep */
//	if (ret < 0)
//		printk(KERN_ERR "melfas_ts_suspend: i2c_smbus_write_byte_data failed\n");
	gpio_direction_output(OMAP_GPIO_TSP_EN, 0);

    return 0;
}

static int melfas_ts_resume(struct i2c_client *client)
{
    struct melfas_ts_data *ts = i2c_get_clientdata(client);

	gpio_direction_output(OMAP_GPIO_TSP_EN, 1);
	msleep(100);

    melfas_init_panel(ts);

	cancel_work_sync(&ts->work);      
       schedule_work(&ts->work);

	enable_irq(client->irq);

    return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void melfas_ts_early_suspend(struct early_suspend *h)
{
    struct melfas_ts_data *ts;
    ts = container_of(h, struct melfas_ts_data, early_suspend);
    melfas_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void melfas_ts_late_resume(struct early_suspend *h)
{
    struct melfas_ts_data *ts;
    ts = container_of(h, struct melfas_ts_data, early_suspend);
    melfas_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id melfas_ts_id[] =
{
    { MELFAS_TS_NAME, 0 },
    { }
};

static struct i2c_driver melfas_ts_driver =
{
    .driver = {
    .name = MELFAS_TS_NAME,
    },
    .id_table = melfas_ts_id,
    .probe = melfas_ts_probe,
    .remove = __devexit_p(melfas_ts_remove),
#ifndef CONFIG_HAS_EARLYSUSPEND
    .suspend = melfas_ts_suspend,
    .resume = melfas_ts_resume,
#endif
};

static int __devinit melfas_ts_init(void)
{
    return i2c_add_driver(&melfas_ts_driver);
}

static void __exit melfas_ts_exit(void)
{
    i2c_del_driver(&melfas_ts_driver);
}

MODULE_DESCRIPTION("Driver for Melfas MTSI Touchscreen Controller");
MODULE_AUTHOR("MinSang, Kim <kimms@melfas.com>");
MODULE_VERSION("0.1");
MODULE_LICENSE("GPL");

module_init(melfas_ts_init);
module_exit(melfas_ts_exit);
