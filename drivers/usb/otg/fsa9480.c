/*
 * fsa9480.c - FSA9480 micro USB switch device driver
 *
 * Copyright (C) 2009 Samsung Electronics
 * Wonguk Jeong <wonguk.jeong@samsung.com>
 * Minkyu Kang <mk7.kang@samsung.com>
 *
 * Modified by Sumeet Pawnikar <sumeet.p@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/fsa9480.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <plat/microusbic.h>
#include <linux/input.h>

#include <mach/omap4-common.h>

/* FSA9480 I2C registers */
#define FSA9480_REG_DEVID              0x01
#define FSA9480_REG_CTRL               0x02
#define FSA9480_REG_INT1               0x03
#define FSA9480_REG_INT2               0x04
#define FSA9480_REG_INT1_MASK          0x05
#define FSA9480_REG_INT2_MASK          0x06
#define FSA9480_REG_ADC                        0x07
#define FSA9480_REG_TIMING1            0x08
#define FSA9480_REG_TIMING2            0x09
#define FSA9480_REG_DEV_T1             0x0a
#define FSA9480_REG_DEV_T2             0x0b
#define FSA9480_REG_BTN1               0x0c
#define FSA9480_REG_BTN2               0x0d
#define FSA9480_REG_CK                 0x0e
#define FSA9480_REG_CK_INT1            0x0f
#define FSA9480_REG_CK_INT2            0x10
#define FSA9480_REG_CK_INTMASK1                0x11
#define FSA9480_REG_CK_INTMASK2                0x12
#define FSA9480_REG_MANSW1             0x13
#define FSA9480_REG_MANSW2             0x14
#define FSA9480_REG_ON_PDDETECT               0x20


/* MANSW1 */
#define VAUDIO                 0x90
#define UART                   0x6c
#define AUDIO                  0x48
#define DHOST                  0x24
#define AUTO                   0x0

/*FSA9485 MANSW1*/
#define VAUDIO_9485            0x93
#define AUDIO_9485             0x4B
#define DHOST_9485             0x27

/* Control */
#define SWITCH_OPEN            (1 << 4)
#define RAW_DATA               (1 << 3)
#define MANUAL_SWITCH          (1 << 2)
#define WAIT                   (1 << 1)
#define INT_MASK               (1 << 0)
#define CTRL_MASK              (SWITCH_OPEN | RAW_DATA | MANUAL_SWITCH | \
                                       WAIT | INT_MASK)
/* Device Type 1*/
#define DEV_USB_OTG            (1 << 7)
#define DEV_DEDICATED_CHG      (1 << 6)
#define DEV_USB_CHG            (1 << 5)
#define DEV_CAR_KIT            (1 << 4)
#define DEV_UART               (1 << 3)
#define DEV_USB                        (1 << 2)
#define DEV_AUDIO_2            (1 << 1)
#define DEV_AUDIO_1            (1 << 0)

#define FSA9480_DEV_T1_HOST_MASK               (DEV_USB_OTG)
#define FSA9480_DEV_T1_USB_MASK                (DEV_USB)
#define FSA9480_DEV_T1_UART_MASK       (DEV_UART)
#define FSA9480_DEV_T1_CHARGER_MASK    (DEV_DEDICATED_CHG | DEV_USB_CHG)
#define FSA9480_DEV_T1_AUDIO_MASK    (DEV_AUDIO_1 | DEV_AUDIO_2)

/* Device Type 2*/
#define DEV_AV                 (1 << 6)
#define DEV_TTY                        (1 << 5)
#define DEV_PPD                        (1 << 4)
#define DEV_JIG_UART_OFF       (1 << 3)
#define DEV_JIG_UART_ON                (1 << 2)
#define DEV_JIG_USB_OFF                (1 << 1)
#define DEV_JIG_USB_ON         (1 << 0)

#define FSA9480_DEV_T2_USB_MASK                (DEV_JIG_USB_OFF | DEV_JIG_USB_ON)
#define FSA9480_DEV_T2_UART_MASK       (DEV_JIG_UART_OFF | DEV_JIG_UART_ON)

#define FSA9480_DEV_T2_JIG_MASK                (DEV_JIG_USB_OFF | DEV_JIG_USB_ON | \
                                       DEV_JIG_UART_OFF | DEV_JIG_UART_ON)

#define DEV_MHL                 (DEV_AV)
#define FSA9480_DEV_T2_MHL_MASK         (DEV_MHL)

/* deskdock key */
#define FSA9480_KEY_VOL_UP		0x0B
#define FSA9480_KEY_VOL_DN		0x0A
#define FSA9480_KEY_VOL_BOTH		0x07
#define FSA9480_KEY_RELEASE		0x1A
#define FSA9480_KEY_DETACHED		0x1F

#define DOCK_VOL_UP_KEY_PRESSED		(1<<0)
#define DOCK_VOL_DN_KEY_PRESSED		(1<<1) 

static int key_press_state;
struct input_dev *input;

struct fsa9480_usbsw {
       struct i2c_client               *client;
       struct fsa9480_platform_data    *pdata;
       struct work_struct              work;
       int                             dev1;
       int                             dev2;
       int                             mansw;
	   u8                              id;
};

static struct fsa9480_usbsw *chip;

#ifdef CONFIG_VIDEO_MHL_V1
/*for MHL cable insertion*/
static int isMHLconnected=0;
#endif

static int isDeskdockconnected=0;

static int fsa9480_write_reg(struct i2c_client *client,        u8 reg, u8 data)
{
       int ret = 0;
       u8 buf[2];
       struct i2c_msg msg[1];

       buf[0] = reg;
       buf[1] = data;

       msg[0].addr = client->addr;
       msg[0].flags = 0;
       msg[0].len = 2;
       msg[0].buf = buf;

       ret = i2c_transfer(client->adapter, &msg, 1);
       if (ret != 1) {
               printk("\n [FSA9480] i2c Write Failed (ret=%d) \n", ret);
               return -1;
       }

      // printk("[FAS9480] i2c Write success\n");
       return ret;
}

static int fsa9480_read_reg(struct i2c_client *client, u8 reg, u8 *data)
{
       int ret = 0;
       u8 buf[1];
       struct i2c_msg msg[2];

       buf[0] = reg;

        msg[0].addr = client->addr;
        msg[0].flags = 0;
        msg[0].len = 1;
        msg[0].buf = buf;

        msg[1].addr = client->addr;
        msg[1].flags = I2C_M_RD;
        msg[1].len = 1;
        msg[1].buf = buf;

       ret = i2c_transfer(client->adapter, msg, 2);
       if (ret != 2) {
               printk("\n [FSA9480] i2c Read Failed (ret=%d) \n", ret);
               return -1;
       }
       *data = buf[0];

      // printk("[FSA9480] i2c Read success\n");
       return 0;
}

static void fsa9480_handle_deskdock_key(u8 adc_data)
{
	switch (adc_data) {
		case FSA9480_KEY_VOL_UP:
			if (key_press_state & DOCK_VOL_DN_KEY_PRESSED) {
				key_press_state &= ~DOCK_VOL_DN_KEY_PRESSED;
				input_event(input, EV_KEY, KEY_VOLUMEDOWN, 0);
#if defined(CONFIG_SAMSUNG_KERNEL_DEBUG_USER)
	               printk(KERN_DEBUG "[DOCK] VOL_DN release\n");
#endif
			}
			if (key_press_state & DOCK_VOL_UP_KEY_PRESSED) {
				break;
			}
			key_press_state |= DOCK_VOL_UP_KEY_PRESSED;
			input_event(input, EV_KEY, KEY_VOLUMEUP, 1);
#if defined(CONFIG_SAMSUNG_KERNEL_DEBUG_USER)
	               printk(KERN_DEBUG "[DOCK] VOL_UP press\n");
#endif
			break;
		case FSA9480_KEY_VOL_DN:
			if (key_press_state & DOCK_VOL_UP_KEY_PRESSED) {
				key_press_state &= ~DOCK_VOL_UP_KEY_PRESSED;
				input_event(input, EV_KEY, KEY_VOLUMEUP, 0);
#if defined(CONFIG_SAMSUNG_KERNEL_DEBUG_USER)
	               printk(KERN_DEBUG "[DOCK] VOL_UP release\n");
#endif
			}
			if (key_press_state & DOCK_VOL_DN_KEY_PRESSED) {
				break;
			}
			key_press_state |= DOCK_VOL_DN_KEY_PRESSED;
			input_event(input, EV_KEY, KEY_VOLUMEDOWN, 1);
#if defined(CONFIG_SAMSUNG_KERNEL_DEBUG_USER)
	               printk(KERN_DEBUG "[DOCK] VOL_DN press\n");
#endif
			break;
		case FSA9480_KEY_VOL_BOTH	:
			if (!(key_press_state & DOCK_VOL_UP_KEY_PRESSED)) {
				key_press_state |= DOCK_VOL_UP_KEY_PRESSED;
				input_event(input, EV_KEY, KEY_VOLUMEUP, 1);
#if defined(CONFIG_SAMSUNG_KERNEL_DEBUG_USER)
	               printk(KERN_DEBUG "[DOCK] VOL_UP press\n");
#endif
			}
			if (!(key_press_state & DOCK_VOL_DN_KEY_PRESSED)) {
				key_press_state |= DOCK_VOL_DN_KEY_PRESSED;
				input_event(input, EV_KEY, KEY_VOLUMEDOWN, 1);
#if defined(CONFIG_SAMSUNG_KERNEL_DEBUG_USER)
	               printk(KERN_DEBUG "[DOCK] VOL_DN press\n");
#endif
			}
			break;
		case FSA9480_KEY_RELEASE:
		case FSA9480_KEY_DETACHED:
			if (key_press_state & DOCK_VOL_UP_KEY_PRESSED) {
				key_press_state &= ~DOCK_VOL_UP_KEY_PRESSED;
				input_event(input, EV_KEY, KEY_VOLUMEUP, 0);
#if defined(CONFIG_SAMSUNG_KERNEL_DEBUG_USER)
	               printk(KERN_DEBUG "[DOCK] VOL_UP release\n");
#endif
			}
			if (key_press_state & DOCK_VOL_DN_KEY_PRESSED) {
				key_press_state &= ~DOCK_VOL_DN_KEY_PRESSED;
				input_event(input, EV_KEY, KEY_VOLUMEDOWN, 0);
#if defined(CONFIG_SAMSUNG_KERNEL_DEBUG_USER)
	               printk(KERN_DEBUG "[DOCK] VOL_DN release\n");
#endif
			}
			break;
		default:
			return;
	}
	input_sync(input);
}

static void fsa9480_read_adc_value(void)
{
	u8 adc=0;
    struct fsa9480_usbsw *usbsw = chip;
    struct i2c_client *client = usbsw->client;
	
	fsa9480_read_reg(client, FSA9480_REG_ADC, &adc);
	printk("[FSA9480] %s: adc is 0x%x\n",__func__,adc);
	fsa9480_handle_deskdock_key(adc);
}

static void DisableFSA9480Interrupts(void)
{
       struct fsa9480_usbsw *usbsw = chip;
       struct i2c_client *client = usbsw->client;
        printk ("DisableFSA9480Interrupts-2\n");

     fsa9480_write_reg(client, FSA9480_REG_INT1_MASK, 0xFF);
     fsa9480_write_reg(client, FSA9480_REG_INT2_MASK, 0x1F);
	 
} // DisableFSA9480Interrupts()

static void EnableFSA9480Interrupts(void)
{
       struct fsa9480_usbsw *usbsw = chip;
       struct i2c_client *client = usbsw->client;
        u8 controlReg = 0;
		u8 intr, intr2;
		
   printk ("EnableFSA9480Interrupts\n");

     /*clear interrupts*/
     fsa9480_read_reg(client, FSA9480_REG_INT1, &intr);
     fsa9480_read_reg(client, FSA9480_REG_INT2, &intr2);
	 
     fsa9480_write_reg(client, FSA9480_REG_INT1_MASK, 0x00);
     fsa9480_write_reg(client, FSA9480_REG_INT2_MASK, 0x00);

} //EnableFSA9480Interrupts()




static void fsa9480_id_open(void)
{
	struct fsa9480_usbsw *usbsw = chip;
	struct i2c_client *client = usbsw->client;

	pr_alert("fsa9480 id_open\n");
	fsa9480_write_reg(client, 0x1B, 1);
}

void fsa9480_set_switch(const char *buf)
{
       struct fsa9480_usbsw *usbsw = chip;
       struct i2c_client *client = usbsw->client;
       u8 value = 0;
       unsigned int path = 0;

       fsa9480_read_reg(client, FSA9480_REG_CTRL, &value);

       if (!strncmp(buf, "VAUDIO", 6)) {
	   	       if(usbsw->id == 0)
			   	path = VAUDIO_9485;
			   else
			   	path = VAUDIO;
               value &= ~MANUAL_SWITCH;
       } else if (!strncmp(buf, "UART", 4)) {
               path = UART;
               value &= ~MANUAL_SWITCH;
       } else if (!strncmp(buf, "AUDIO", 5)) {
               if(usbsw->id == 0)
			   	path = AUDIO_9485;
			   else
                path = AUDIO;
               value &= ~MANUAL_SWITCH;
       } else if (!strncmp(buf, "DHOST", 5)) {
               path = DHOST;
               value &= ~MANUAL_SWITCH;
       } else if (!strncmp(buf, "AUTO", 4)) {
               path = AUTO;
               value |= MANUAL_SWITCH;
       } else {
               printk(KERN_ERR "Wrong command\n");
               return;
       }

       usbsw->mansw = path;
       fsa9480_write_reg(client, FSA9480_REG_MANSW1, path);
       fsa9480_write_reg(client, FSA9480_REG_CTRL, value);
}
EXPORT_SYMBOL_GPL(fsa9480_set_switch);

ssize_t fsa9480_get_switch(char *buf)
{
struct fsa9480_usbsw *usbsw = chip;
       struct i2c_client *client = usbsw->client;
       u8 value;

       fsa9480_read_reg(client, FSA9480_REG_MANSW1, &value);

       if (value == VAUDIO)
               return sprintf(buf, "VAUDIO\n");
       else if (value == UART)
               return sprintf(buf, "UART\n");
       else if (value == AUDIO)
               return sprintf(buf, "AUDIO\n");
       else if (value == DHOST)
               return sprintf(buf, "DHOST\n");
       else if (value == AUTO)
               return sprintf(buf, "AUTO\n");
       else
               return sprintf(buf, "%x", value);
}
EXPORT_SYMBOL_GPL(fsa9480_get_switch);


#ifdef CONFIG_VIDEO_MHL_V1
#ifdef CONFIG_OMAP_PM
#include <plat/omap-pm.h>
static struct pm_qos_request_list *pm_qos_dpll_handle;
#endif

void FSA9480_EnableIntrruptByMHL(bool _bDo)
{
	struct fsa9480_platform_data *pdata = chip->pdata;
	struct i2c_client *client = chip->client;
	//char buf[16];
	int ret = 0;

	if(true == _bDo)
	{
		printk("FSA9480_EnableIntrruptByMHL detatch\n");
#ifdef CONFIG_OMAP_PM
                 ret = omap_pm_set_max_mpu_wakeup_lat(&pm_qos_dpll_handle, -1);
                 if (ret) {
                            printk("%s %d Error setting MPU Constrain : %d\n", __func__, __LINE__,ret);
                            return;
                 }
#endif
		fsa9480_write_reg(client,FSA9480_REG_CTRL, 0x1E);
		EnableFSA9480Interrupts();
		isMHLconnected = 0;
	}
	else
	{
		printk("FSA9480_EnableIntrruptByMHL attach \n");
		DisableFSA9480Interrupts();
#ifdef CONFIG_OMAP_PM
		ret = omap_pm_set_max_mpu_wakeup_lat(&pm_qos_dpll_handle, 7);
		if (ret) {
			printk("%s %d Error setting MPU Constrain : %d\n", __func__, __LINE__,ret);
			return;
		}
 #endif
	}

	//fsa9480_get_switch(buf);
	//printk("[%s] fsa switch status = %s\n",__func__, buf);
}

/*MHL call this function to change VAUDIO path*/
void FSA9480_CheckAndHookAudioDock(void)
{
   struct fsa9480_platform_data *pdata = chip->pdata;
   struct i2c_client *client = chip->client;

   printk("[FSA9480] %s: FSA9485 VAUDIO\n",__func__);
   
   isMHLconnected = 0;
   isDeskdockconnected = 1;
   
   if (pdata->mhl_cb)
   	       pdata->mhl_cb(FSA9480_DETACHED);

   EnableFSA9480Interrupts();

   if(chip->id == 0)
	chip->mansw = VAUDIO_9485;
   else
	chip->mansw = VAUDIO;

   /*make ID change report*/
   fsa9480_write_reg(client,FSA9480_REG_CTRL, 0x16);
   
   if(pdata->deskdock_cb)
           pdata->deskdock_cb(FSA9480_ATTACHED);   

}
EXPORT_SYBMOL_GPL(FSA9480_CheckAndHookAudioDock);
#endif


static ssize_t fsa9480_show_status(struct device *dev,
                                  struct device_attribute *attr,
                                  char *buf)
{
       struct fsa9480_usbsw *usbsw = dev_get_drvdata(dev);
       struct i2c_client *client = usbsw->client;
       u8 devid, ctrl, adc, dev1, dev2, intr;
       u8 intmask1, intmask2, time1, time2, mansw1;

       fsa9480_read_reg(client, FSA9480_REG_DEVID, &devid);
       fsa9480_read_reg(client, FSA9480_REG_CTRL, &ctrl);
       fsa9480_read_reg(client, FSA9480_REG_ADC, &adc);
       fsa9480_read_reg(client, FSA9480_REG_INT1_MASK, &intmask1);
       fsa9480_read_reg(client, FSA9480_REG_INT2_MASK, &intmask2);
       fsa9480_read_reg(client, FSA9480_REG_DEV_T1, &dev1);
       fsa9480_read_reg(client, FSA9480_REG_DEV_T2, &dev2);
       fsa9480_read_reg(client, FSA9480_REG_TIMING1, &time1);
       fsa9480_read_reg(client, FSA9480_REG_TIMING2, &time2);
       fsa9480_read_reg(client, FSA9480_REG_MANSW1, &mansw1);

       fsa9480_read_reg(client, FSA9480_REG_INT1, &intr);
       intr &= 0xffff;

       return sprintf(buf, "Device ID(%02x), CTRL(%02x)\n"
                       "ADC(%02x), DEV_T1(%02x), DEV_T2(%02x)\n"
                       "INT(%04x), INTMASK(%02x, %02x)\n"
                       "TIMING(%02x, %02x), MANSW1(%02x)\n",
                       devid, ctrl, adc, dev1, dev2, intr,
                       intmask1, intmask2, time1, time2, mansw1);
}

static ssize_t fsa9480_show_manualsw(struct device *dev,
               struct device_attribute *attr, char *buf)
{
       return fsa9480_get_switch(buf);

}

static ssize_t fsa9480_set_manualsw(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf, size_t count)
{
       fsa9480_set_switch(buf);
       return count;
}

static DEVICE_ATTR(status, 0664, fsa9480_show_status, NULL);
static DEVICE_ATTR(switch,  0664,
               fsa9480_show_manualsw, fsa9480_set_manualsw);
static struct attribute *fsa9480_attributes[] = {
       &dev_attr_status.attr,
       &dev_attr_switch.attr,
       NULL
};

static const struct attribute_group fsa9480_group = {
       .attrs = fsa9480_attributes,
};

static irqreturn_t fsa9480_irq_handler(int irq, void *data)
{
       struct fsa9480_usbsw *usbsw = data;

       if (!work_pending(&usbsw->work)) {
               disable_irq_nosync(irq);
               schedule_work(&usbsw->work);
       }

       return IRQ_HANDLED;
}

static void fsa9480_detect_dev(struct fsa9480_usbsw *usbsw, u8 intr)
{
       u8 val1, val2, ctrl,temp;
       struct fsa9480_platform_data *pdata = usbsw->pdata;
       struct i2c_client *client = usbsw->client;
       int ret = 0;

#if 0
       /*reset except CP USB and AV dock*/
	   if ((usbsw->mansw != AUDIO) && (usbsw->mansw != AUDIO_9485)
	   	   && (usbsw->mansw != VAUDIO) && (usbsw->mansw != VAUDIO_9485))
       	{
		  /*reset UIC when mansw is not set*/
		  printk("[FSA9480] %s: reset UIC mansw is 0x%x\n",__func__,usbsw->mansw);
          fsa9480_write_reg(client, FSA9480_REG_CTRL, 0x1E);
		  usbsw->mansw = AUTO;
       	} 
#endif

       fsa9480_read_reg(client, FSA9480_REG_DEV_T1, &val1);
       fsa9480_read_reg(client, FSA9480_REG_DEV_T2, &val2);
       fsa9480_read_reg(client, FSA9480_REG_CTRL, &ctrl);

       dev_info(&client->dev, "intr: 0x%x, dev1: 0x%x, dev2: 0x%x, ctrl: 0x%x\n",
                       intr, val1, val2,ctrl);

       /* Attached */
       if (intr & (1 << 0)) {
	   	       if (val1 & DEV_CAR_KIT) {

				fsa9480_write_reg(client,FSA9480_REG_CK, 0x02);
				fsa9480_read_reg(client, FSA9480_REG_CK_INT1, &temp);			  	

               }
               if (val1 & FSA9480_DEV_T1_USB_MASK ||
                       val2 & FSA9480_DEV_T2_USB_MASK) {
                       if (pdata->usb_cb)
                               pdata->usb_cb(FSA9480_ATTACHED);
#if 1
                       if (usbsw->mansw)
                       	{
                               fsa9480_write_reg(client,FSA9480_REG_MANSW1, usbsw->mansw);
							   fsa9480_write_reg(client,FSA9480_REG_CTRL, 0x1A);
                       	}
#endif
               }

               if (val1 & FSA9480_DEV_T1_UART_MASK ||
                       val2 & FSA9480_DEV_T2_UART_MASK) {
                       if (pdata->uart_cb)
                               pdata->uart_cb(FSA9480_ATTACHED);

                       if (!(ctrl & (1 << 2)))
                       	{
                               fsa9480_write_reg(client,FSA9480_REG_MANSW1, UART);
							   fsa9480_write_reg(client,FSA9480_REG_CTRL, 0x1A);
                       	}
					           
               }


               if (val1 & FSA9480_DEV_T1_CHARGER_MASK) {
                       if (pdata->charger_cb)
                               pdata->charger_cb(FSA9480_ATTACHED);
               }

#ifdef CONFIG_EXTRA_DOCK_SPEAKER	
               if ((val1 & FSA9480_DEV_T1_AUDIO_MASK) ||(val2 & DEV_JIG_UART_ON) ) {
			   	        printk("[FSA9480] %s : CAR DOCK connected\n",__func__);
			   	       /*set forcely to AV*/
                       if(usbsw->id == 0)
				           fsa9480_write_reg(client,FSA9480_REG_MANSW1, VAUDIO_9485);
					   else
					   	   fsa9480_write_reg(client,FSA9480_REG_MANSW1, VAUDIO);
				       fsa9480_write_reg(client,FSA9480_REG_CTRL, 0x1A);
					   
                       if (pdata->cardock_cb)
                               pdata->cardock_cb(FSA9480_ATTACHED);
               }			   
#endif

               if (val2 & FSA9480_DEV_T2_JIG_MASK) {
                       if (pdata->jig_cb)
                               pdata->jig_cb(FSA9480_ATTACHED);
               }
#ifdef CONFIG_VIDEO_MHL_V1
		if(val2 & FSA9480_DEV_T2_MHL_MASK) {
			if(isDeskdockconnected)
			{
				printk("[FSA9480] %s : Deskdock already inserted\n",__func__);
				fsa9480_write_reg(client,FSA9480_REG_CTRL, 0x1E);
				EnableFSA9480Interrupts();
				return;
			}

			FSA9480_EnableIntrruptByMHL(false);
			printk("[FSA9480] %s : MHL connected\n",__func__);

		    /*set open by manual switching*/
			if(usbsw->id == 0)
				fsa9480_write_reg(client,FSA9480_REG_MANSW1, 0x90);
			else
				fsa9480_write_reg(client,FSA9480_REG_MANSW1, 0x91);
			fsa9480_write_reg(client,FSA9480_REG_CTRL, 0x1A);

			if(pdata->mhl_cb) {
				isMHLconnected = 1;
				pdata->mhl_cb(FSA9480_ATTACHED);
			}
                }
#else
          if(val2 & DEV_AV ) {
		  	   if(pdata->deskdock_cb)
                       pdata->deskdock_cb(FSA9480_ATTACHED); 
			   usbsw->mansw = VAUDIO_9485;
			   printk("[FSA9480] %s: VAUDIO-ATTACHED\n",__func__);
          	}
#endif
       } else if (intr & (1 << 1)) {
               if (usbsw->dev1 & FSA9480_DEV_T1_USB_MASK ||
                       usbsw->dev2 & FSA9480_DEV_T2_USB_MASK) {
                       if (pdata->usb_cb)
                               pdata->usb_cb(FSA9480_DETACHED);
               }

               if (usbsw->dev1 & FSA9480_DEV_T1_UART_MASK ||
                       usbsw->dev2 & FSA9480_DEV_T2_UART_MASK) {
                       if (pdata->uart_cb)
                               pdata->uart_cb(FSA9480_DETACHED);
               }

               if (usbsw->dev1 & FSA9480_DEV_T1_CHARGER_MASK) {
                       if (pdata->charger_cb)
                               pdata->charger_cb(FSA9480_DETACHED);
               }

#ifdef CONFIG_EXTRA_DOCK_SPEAKER
               if ((usbsw->dev1 & FSA9480_DEV_T1_AUDIO_MASK)||(usbsw->dev2 & DEV_JIG_UART_ON)) {
                       if (pdata->cardock_cb)
                               pdata->cardock_cb(FSA9480_DETACHED);
		       fsa9480_write_reg(client,FSA9480_REG_CTRL, 0x1E);
               }
			   
#endif
               if (usbsw->dev2 & FSA9480_DEV_T2_JIG_MASK) {
                                                                                                                                                                 
                       if (pdata->jig_cb)
                               pdata->jig_cb(FSA9480_DETACHED);
               }
#ifdef CONFIG_VIDEO_MHL_V1
		if (usbsw->dev2 & FSA9480_DEV_T2_MHL_MASK) 
		{
			if(isDeskdockconnected == 0)//isMHLconnected)
			{
				printk("[FSA9480] %s : MHL disconnected\n",__func__);
				isMHLconnected = 0;
				// if (pdata->mhl_cb)
				//       pdata->mhl_cb(FSA9480_DETACHED);
			}
              }
#else
         if (usbsw->dev2 & DEV_AV) {
		 	     if (pdata->deskdock_cb)
                         pdata->deskdock_cb(FSA9480_DETACHED);
				 fsa9480_write_reg(client,FSA9480_REG_CTRL, 0x1E);
				 usbsw->mansw = AUTO;
         	}
#endif
       /*deskdock detach case*/
	if(isDeskdockconnected)
       	{
		printk("[FSA9480] %s : Deskdock disconnected\n",__func__);
		if (pdata->deskdock_cb)
		{
			pdata->deskdock_cb(FSA9480_DETACHED);
		}
			fsa9480_write_reg(client,FSA9480_REG_CTRL, 0x1E);
			isDeskdockconnected=0;
			usbsw->mansw = AUTO;
       	}
 
	   if ((usbsw->mansw != AUDIO) && (usbsw->mansw != AUDIO_9485)
	   	   && (usbsw->mansw != VAUDIO) && (usbsw->mansw != VAUDIO_9485))
	  	  usbsw->mansw = AUTO;
	

       }

	ret = get_real_usbic_state();
	if (ret > 0) {
			/*
			 * The DPLL sets a constraints to hold DPLL cascade entry
			 * When USB/MHL/TA module inserts.
			 */
			if (dpll_cascading_blocker_hold(&client->dev) < 0)
				dev_warn(&client->dev, "Error holding DPLL cascading constraint\n");
	} else {
			/*
			 * The DPLL release a constraints to enter DPLL cascade entry 
			 * for LowPower mode When USB/MHL/TA module removals.
			 */
			if (dpll_cascading_blocker_release(&client->dev) < 0)
				dev_info(&client->dev, "DPLL cascading constraint for this device does not exist\n");
	}

       usbsw->dev1 = val1;
       usbsw->dev2 = val2;

       chip->dev1 = val1;
       chip->dev2 = val2;

       fsa9480_read_reg(client, FSA9480_REG_CTRL, &ctrl);
       ctrl &= ~INT_MASK;
       fsa9480_write_reg(client, FSA9480_REG_CTRL, ctrl);
}

int get_real_usbic_state(void)
{
       struct fsa9480_usbsw *usbsw = chip;
       int ret = MICROUSBIC_NO_DEVICE ;
       int val1 = 0;
       int val2 = 0;

       /* read real usb ic state
       val1 = chip->dev1;
       val2 = chip->dev2;
       */
       struct i2c_client *client = usbsw->client;
       fsa9480_read_reg(client, FSA9480_REG_DEV_T1, &val1);
       fsa9480_read_reg(client, FSA9480_REG_DEV_T2, &val2);

       if (val1 & FSA9480_DEV_T1_USB_MASK)
               ret = MICROUSBIC_USB_CABLE;
       else if (val1 & FSA9480_DEV_T1_CHARGER_MASK)
               ret = MICROUSBIC_USB_CHARGER;
       else if (val1 & FSA9480_DEV_T1_UART_MASK)
               ret = MICROUSBIC_USB_CHARGER;
	   else if (val1 & FSA9480_DEV_T1_HOST_MASK)
		   	   ret = MICROUSBIC_HOST;

       if (ret ==  MICROUSBIC_NO_DEVICE) {
               if (val2 & DEV_JIG_USB_ON)
				   ret = MICROUSBIC_JIG_USB_ON;
			   else if (val2 & FSA9480_DEV_T2_MHL_MASK)
				   ret = MICROUSBIC_MHL_CHARGER;
       }

	if(ret ==  MICROUSBIC_NO_DEVICE && isMHLconnected == 1)
	{
		return MICROUSBIC_MHL_CHARGER;
	}

      return ret;
}

int get_usbic_state(void)
{
	struct fsa9480_usbsw *usbsw = chip;
	int ret = MICROUSBIC_NO_DEVICE ;
	int val1 = 0;
	int val2 = 0;

	if(isMHLconnected)
	{
		return MICROUSBIC_MHL_CHARGER;
	}

	val1 = chip->dev1;
	val2 = chip->dev2;

	if (val1 & FSA9480_DEV_T1_USB_MASK)
		ret = MICROUSBIC_USB_CABLE;
	else if (val1 & FSA9480_DEV_T1_CHARGER_MASK)
		ret = MICROUSBIC_USB_CHARGER;
	else if (val1 & FSA9480_DEV_T1_UART_MASK)
		ret = MICROUSBIC_USB_CHARGER;
	else if (val1 & FSA9480_DEV_T1_HOST_MASK)
		ret = MICROUSBIC_HOST;

	if (ret ==  MICROUSBIC_NO_DEVICE) {
		if (val2 & DEV_JIG_USB_ON)
			ret = MICROUSBIC_JIG_USB_ON;
		else if (val2 & FSA9480_DEV_T2_MHL_MASK)
			ret = MICROUSBIC_MHL_CHARGER;
	}
	return ret;
}

static void fsa9480_work_cb(struct work_struct *work)
{
       u8 intr, intr2, val1, val2;
       struct fsa9480_usbsw *usbsw =
               container_of(work, struct fsa9480_usbsw, work);
       struct i2c_client *client = usbsw->client;

       fsa9480_read_reg(client, FSA9480_REG_DEV_T1, &val1);
       fsa9480_read_reg(client, FSA9480_REG_DEV_T2, &val2);

	   /*check AV cable for volume up & down*/
	   if(val2 & DEV_AV )
	   	if( (usbsw->mansw != VAUDIO) && (usbsw->mansw != VAUDIO_9485))
	   	{
		DisableFSA9480Interrupts();
	   	}

       /* clear interrupt */
       fsa9480_read_reg(client, FSA9480_REG_INT1, &intr);
	   fsa9480_read_reg(client, FSA9480_REG_INT2, &intr2);
       intr &= 0xffff;

	   pr_alert("[FSA9480] %s: intr=0x%x, intr2=0x%x, dev1=0x%x, dev2=0x%x\n",__func__,intr,intr2,val1,val2);

#if 1
       if((val2 & DEV_AV) && (intr2 == 0x04))
	   	fsa9480_read_adc_value();
	   else
#endif
         /* device detection */
       fsa9480_detect_dev(usbsw, intr);

       enable_irq(client->irq);
}
static int fsa9480_irq_init(struct fsa9480_usbsw *usbsw)
{
       struct fsa9480_platform_data *pdata = usbsw->pdata;
       struct i2c_client *client = usbsw->client;
       int ret, irq = -1;
       u8 intr, intr2;
       u8 mansw1;
       unsigned int ctrl = CTRL_MASK;

       /* clear interrupt */
       fsa9480_read_reg(client, FSA9480_REG_INT1, &intr);
	   fsa9480_read_reg(client, FSA9480_REG_INT2, &intr2);
       intr &= 0xffff;

       /* unmask interrupt (attach/detach only) */
       ret = fsa9480_write_reg(client, FSA9480_REG_INT1_MASK, 0x00);
       if (ret < 0)
               return ret;

       ret = fsa9480_write_reg(client, FSA9480_REG_INT2_MASK, 0x00);
       if (ret < 0)
               return ret;

       fsa9480_read_reg(client, FSA9480_REG_MANSW1, &mansw1);
       usbsw->mansw = mansw1;

       ctrl &= ~INT_MASK;              /* Unmask Interrupt */

       if (usbsw->mansw)
               ctrl &= ~MANUAL_SWITCH; /* Manual Switching Mode */

       fsa9480_write_reg(client, FSA9480_REG_CTRL, ctrl);

       INIT_WORK(&usbsw->work, fsa9480_work_cb);

       ret = request_irq(client->irq, fsa9480_irq_handler,
                       IRQF_TRIGGER_LOW | IRQF_DISABLED,
                       "fsa9480 micro USB", usbsw);
       if (ret) {
               dev_err(&client->dev,
                       "fsa9480: Unable to get IRQ %d\n", irq);
               goto out;
       }

       return 0;
out:
       return ret;
}
static int __devinit fsa9480_probe(struct i2c_client *client,
                        const struct i2c_device_id *id)
{
       struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
       struct fsa9480_usbsw *usbsw;
       int ret = 0;

       printk("[FSA9480] PROBE ...\n");

       usbsw = kzalloc(sizeof(struct fsa9480_usbsw), GFP_KERNEL);
       if (!usbsw) {
               dev_err(&client->dev, "failed to allocate driver data\n");
               return -ENOMEM;
       }

       input = input_allocate_device();
       if(!input) {
              dev_err(&client->dev, "failed to allocate input device\n");
              return -ENOMEM;
       }

       usbsw->client = client;
       usbsw->pdata = client->dev.platform_data;

       chip = usbsw;

       i2c_set_clientdata(client, usbsw);

       ret = fsa9480_irq_init(usbsw);
       if (ret)
               goto fsa9480_probe_fail;

       ret = sysfs_create_group(&client->dev.kobj, &fsa9480_group);
       if (ret) {
               dev_err(&client->dev,
                               "Creating fsa9480 attribute group failed");
               goto fsa9480_probe_fail2;
       }

       input->name = "sec_deskdock";
       input->phys = "sec_deskdock/input0";
       input->id.bustype = BUS_HOST;
       input->id.vendor = 0x0001;
       input->id.product = 0x0001;
       input->id.version = 0x0001;

       input_set_capability(input, EV_KEY, KEY_VOLUMEUP);
       input_set_capability(input, EV_KEY, KEY_VOLUMEDOWN);

       ret = input_register_device(input);
       if (ret) {
              dev_err(&client->dev, "%s: Unable to register input device, "
              "error: %d\n", __func__, ret);
              goto fsa9480_probe_fail2;
       }

       fsa9480_write_reg(client, FSA9480_REG_TIMING1, 0x6);
	   fsa9480_read_reg(client, FSA9480_REG_DEVID, &usbsw->id);

       if (chip->pdata->reset_cb)
               chip->pdata->reset_cb();

	   chip->pdata->id_open_cb = fsa9480_id_open;

       /* device detection */
       fsa9480_detect_dev(usbsw, 1);

	   /*reset UIC*/
	   fsa9480_write_reg(client, FSA9480_REG_CTRL, 0x1E);

	   /*set timing1 to 50ms*/
	   fsa9480_write_reg(client, FSA9480_REG_TIMING1, 0x0);

	   /*set enable DM/DP pull-down detection*/
	   fsa9480_write_reg(client, FSA9480_REG_ON_PDDETECT, 0x0C);
	   
	   

       printk("[FSA9480] PROBE Done.\n");
       return 0;

fsa9480_probe_fail2:
       if (client->irq)
               free_irq(client->irq, NULL);
fsa9480_probe_fail:
       i2c_set_clientdata(client, NULL);
       input_free_device(input);
       kfree(usbsw);
       return ret;
}
static int __devexit fsa9480_remove(struct i2c_client *client)
{
       struct fsa9480_usbsw *usbsw = i2c_get_clientdata(client);
       
	/*
	 * The DPLL releases all constraints to enter Low power mode 
	 * (DPLL cascade state) When USB/MHL/TA module removals.
	 */
	dpll_cascading_blocker_release(&client->dev);
       
	if (client->irq)
               free_irq(client->irq, NULL);
       i2c_set_clientdata(client, NULL);

       sysfs_remove_group(&client->dev.kobj, &fsa9480_group);
       kfree(usbsw);
       return 0;
}

#ifdef CONFIG_PM
static int fsa9480_resume(struct i2c_client *client)
{
       struct fsa9480_usbsw *usbsw = i2c_get_clientdata(client);
       u8 intr;
       u8 val1, val2;

       /* for hibernation */
       fsa9480_read_reg(client, FSA9480_REG_DEV_T1, &val1);
       fsa9480_read_reg(client, FSA9480_REG_DEV_T2, &val2);

       if (val1 || val2)
               intr = 1 << 0;
       else
               intr = 1 << 1;

       /* device detection */
       fsa9480_detect_dev(usbsw, intr);

       return 0;
}
#else
#define fsa9480_resume         NULL
#endif

static const struct i2c_device_id fsa9480_id[] = {
       {"fsa9480", 0},
       {}
};
MODULE_DEVICE_TABLE(i2c, fsa9480_id);

static struct i2c_driver fsa9480_i2c_driver = {
       .driver = {
               .name = "fsa9480",
       },
       .probe = fsa9480_probe,
       .remove = __devexit_p(fsa9480_remove),
       .resume = fsa9480_resume,
       .id_table = fsa9480_id,
};

static int __init fsa9480_init(void)
{
       return i2c_add_driver(&fsa9480_i2c_driver);
}

module_init(fsa9480_init);

#ifdef CONFIG_CHARGER_DETECT_BOOT
charger_module_init(fsa9480_init);
#endif

static void __exit fsa9480_exit(void)
{
       i2c_del_driver(&fsa9480_i2c_driver);
}

module_exit(fsa9480_exit);

MODULE_AUTHOR("Wonguk.Jeong <wonguk.jeong@samsung.com>");
MODULE_DESCRIPTION("FSA9480 USB Switch driver");
MODULE_LICENSE("GPL");



