/*
 *  atmel_maxtouch.c - Atmel maXTouch Touchscreen Controller
 *
 *  Version 0.2a
 *
 *  An early alpha version of the maXTouch Linux driver.
 *
 *
 *  Copyright (C) 2010 Iiro Valkonen <iiro.valkonen@atmel.com>
 *  Copyright (C) 2009 Ulf Samuelsson <ulf.samuelsson@atmel.com>
 *  Copyright (C) 2009 Raphael Derosso Pereira <raphaelpereira@gmail.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/string.h>
#include <linux/proc_fs.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/earlysuspend.h>
#include <linux/firmware.h>
#include <linux/wakelock.h>
#include <linux/delay.h>
#include <linux/i2c.h>



#include <linux/atmel_mxt1386.h>
#include "atmel_mxt1386_cfg.h"

#define READ_FW_FROM_HEADER 1
#define MXT_CONFIRM_I2CWRITE 0

u8 firmware_latest[] = {
	#include "mxt1386_fw_ver09.h"
};

static gen_powerconfig_t7_config_t power_config = {0};
static gen_acquisitionconfig_t8_config_t acquisition_config = {0};
static touch_multitouchscreen_t9_config_t touchscreen_config = {0};
static procg_noisesuppression_t22_config_t noise_suppression_config = {0};
static spt_cteconfig_t28_config_t cte_config = {0};
static proci_palmsuppression_t41_config_t palmsupression_config = {0};

extern int mxt_debug;

int mxt_get_object_values(struct mxt_data *mxt, u8 *buf, int obj_type)
{
	u8 *obj=NULL;
	u16 obj_size=0;

	switch(obj_type)
	{
	case MXT_GEN_POWERCONFIG_T7:
		obj = (u8*)&power_config;
		obj_size = MXT_GET_SIZE(obj_type);
		break;		
	case MXT_GEN_ACQUIRECONFIG_T8:
		obj = (u8*)&acquisition_config;
		obj_size = MXT_GET_SIZE(obj_type);
		break;
	case MXT_TOUCH_MULTITOUCHSCREEN_T9:
		obj = (u8*)&touchscreen_config;
		obj_size = MXT_GET_SIZE(obj_type);
		break;
	case MXT_PROCG_NOISESUPPRESSION_T22:
		obj = (u8*)&noise_suppression_config;
		obj_size = MXT_GET_SIZE(obj_type);
		break;
	case MXT_SPT_CTECONFIG_T28:
		obj = (u8*)&cte_config;
		obj_size = MXT_GET_SIZE(obj_type);
		break;
	default:
		kloge("Not supporting object type (object type: %d)", obj_type);
		return -1;
	}
	klogi("obj type: %d, obj size: %d", obj_type, obj_size);

	if (memcpy(buf, obj, obj_size)==NULL) {
		kloge("memcpy failed!");
		return -1;
	}
	return 0;
}

int mxt_power_config(struct mxt_data *mxt)
{
	struct i2c_client *client = mxt->client;
	u16 obj_addr, obj_size;
	int error;

	obj_addr = MXT_BASE_ADDR(MXT_GEN_POWERCONFIG_T7);
	obj_size = MXT_GET_SIZE(MXT_GEN_POWERCONFIG_T7);
	//klogi("address: 0x%x, size: %d", obj_addr, obj_size);

	power_config.idleacqint = 32;  /* Set Idle Acquisition Interval to 32 ms. */
	power_config.actvacqint = 255;	
	power_config.actv2idleto = 50;  /* Set Active to Idle Timeout to 4 s (one unit = 200ms). */

	//mxt_write_block(client, 0x0135, 3, (u8*)&power_config);
	error = mxt_write_block(client, obj_addr, obj_size, (u8*)&power_config);
	if (error < 0) {
		kloge( "mxt_write_byte failed!\n");
		return -EIO;
	}
#if MXT_CONFIRM_I2CWRITE
	else {
		u8 buf;
		int ret;
		mxt_read_block(client, obj_addr, obj_size, (u8*)&buf); 
		if (memcmp((u8*)&power_config, (u8*)&buf, obj_size) == 0) {
			kloge("Verifying configuration values... Ok");
			ret = 0;
		}
		else {
			kloge("Verifying configuration values... Failed");
			ret = -1;
		}
		return ret;
	}
#else
	return 0;
#endif
}

int mxt_acquisition_config(struct mxt_data *mxt)
{
	struct i2c_client *client = mxt->client;
	u16 obj_addr, obj_size;
	int error;

	obj_addr = MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8);
	obj_size = MXT_GET_SIZE(MXT_GEN_ACQUIRECONFIG_T8);
	//klogi("address: 0x%x, size: %d", obj_addr, obj_size);

	acquisition_config.chrgtime = 10;  // Atmel's recommendation: 8 -> 10
	acquisition_config.reserved = 0;
	acquisition_config.tchdrift = 5; 
	acquisition_config.driftst = 0; 
	acquisition_config.tchautocal = 0; // infinite
	acquisition_config.sync = 0; // disabled

	acquisition_config.atchcalst = 9;
	acquisition_config.atchcalsthr = 40;
	acquisition_config.atchcalfrcthr= 50;  // Atmel's recommendation: 0 -> 50
	acquisition_config.atchcalfrcratio = 0;

	//mxt_write_block(client, 0x0138, 10, (u8*)&acquisition_config);
	error = mxt_write_block(client, obj_addr, obj_size, (u8*)&acquisition_config);
	if (error < 0) {
		kloge("mxt_write_byte failed!\n");
		return -EIO;
	}
#if MXT_CONFIRM_I2CWRITE
	else {
		u8 buf;
		int ret;
		mxt_read_block(client, obj_addr, obj_size, (u8*)&buf); 
		if (memcmp((u8*)&acquisition_config, (u8*)&buf, obj_size) == 0) {
			kloge("Verifying config values... Ok");
			ret = 0;
		}
		else {
			kloge("Verifying config values... Failed");
			ret = -1;
		}
		return ret;
	}
#else
	return 0;
#endif
}

int mxt_multitouch_config(struct mxt_data *mxt)
{
	struct i2c_client *client = mxt->client;
	u16 obj_addr, obj_size;
	int error;

	obj_addr = MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9);
	obj_size = MXT_GET_SIZE(MXT_TOUCH_MULTITOUCHSCREEN_T9);
	//klogi("address: 0x%x, size: %d", obj_addr, obj_size);

	//touchscreen_config.ctrl = 0x8B; // enable amplitude
	touchscreen_config.ctrl = 131; // enable + message-enable
	touchscreen_config.xorigin = 0;
	touchscreen_config.yorigin = 0;
	touchscreen_config.xsize = 27;
	touchscreen_config.ysize = 42;
	touchscreen_config.akscfg = 0;
	touchscreen_config.blen = 0x11;
	touchscreen_config.tchthr = 50;
	touchscreen_config.tchdi = 2;
	touchscreen_config.orient = 0;
	touchscreen_config.mrgtimeout = 0;
	touchscreen_config.movhysti = 3;
	touchscreen_config.movhystn = 1;
	touchscreen_config.movfilter = 0x20;
	touchscreen_config.numtouch= MXT_MAX_NUM_TOUCHES;  // _SUPPORT_MULTITOUCH_
	touchscreen_config.mrghyst = 10;
	touchscreen_config.mrgthr = 20;
	touchscreen_config.amphyst = 10;
	touchscreen_config.xrange = 799;  // width: 1280
	touchscreen_config.yrange = 1279;  // height: 800
	//touchscreen_config.xrange = 1279;  // width: 1280
	//touchscreen_config.yrange = 799;  // height: 800
	touchscreen_config.xloclip = 0;
	touchscreen_config.xhiclip = 0;
	touchscreen_config.yloclip = 0;
	touchscreen_config.yhiclip = 0;
	touchscreen_config.xedgectrl = 0;
	touchscreen_config.xedgedist = 0;
	touchscreen_config.yedgectrl = 0;
	touchscreen_config.yedgedist = 0;
	touchscreen_config.jumplimit = 18;

	//mxt_write_block(client, 0x0142, 31, (u8*)&touchscreen_config);
	error = mxt_write_block(client, obj_addr, obj_size, (u8*)&touchscreen_config);
	if (error < 0) {
		kloge("mxt_write_byte failed!\n");
		return -EIO;
	}
#if MXT_CONFIRM_I2CWRITE
	else {
		u8 buf;
		int ret;
		mxt_read_block(client, obj_addr, obj_size, (u8*)&buf); 
		if (memcmp((u8*)&touchscreen_config, (u8*)&buf, obj_size) == 0) {
			kloge("Verifying config values... Ok");
			ret = 0;
		}
		else {
			kloge("Verifying config values... Failed");
			ret = -1;
		}
		return ret;
	}
#else
	return 0;
#endif
}

int mxt_noise_suppression_config(struct mxt_data *mxt)
{
	struct i2c_client *client = mxt->client;
	u16 obj_addr, obj_size;
	int error;

	obj_addr = MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T22);
	obj_size = MXT_GET_SIZE(MXT_PROCG_NOISESUPPRESSION_T22);
	//klogi("address: 0x%x, size: %d", obj_addr, obj_size);

	noise_suppression_config.ctrl = 5;		
	noise_suppression_config.reserved = 0;
	noise_suppression_config.reserved1 = 0;
	noise_suppression_config.reserved2 = 0;
	noise_suppression_config.reserved3 = 0;
	noise_suppression_config.reserved4 = 0;
	noise_suppression_config.reserved5 = 0;
	noise_suppression_config.reserved6 = 0;
	noise_suppression_config.noisethr = 40;
	noise_suppression_config.reserved7 = 0;///1;
	noise_suppression_config.freq[0] = 10;
	noise_suppression_config.freq[1] = 15;
	noise_suppression_config.freq[2] = 20;
	noise_suppression_config.freq[3] = 25;
	noise_suppression_config.freq[4] = 30;
	noise_suppression_config.reserved8 = 3;

	//mxt_write_block(client, 0x0185, 17, (u8*)&noise_suppression_config);
	error = mxt_write_block(client, obj_addr, obj_size, (u8*)&noise_suppression_config);
	if (error < 0) {
		kloge("mxt_write_byte failed!\n");
		return -EIO;
	}
#if MXT_CONFIRM_I2CWRITE
	else {
		u8 buf;
		int ret;
		mxt_read_block(client, obj_addr, obj_size, (u8*)&buf); 
		if (memcmp((u8*)&noise_suppression_config, (u8*)&buf, obj_size) == 0) {
			kloge("Verifying config values... Ok");
			ret = 0;
		}
		else {
			kloge("Verifying config values... Failed");
			ret = -1;
		}
		return ret;
	}
#else
	return 0;
#endif
}

int mxt_cte_config(struct mxt_data *mxt)
{
	struct i2c_client *client = mxt->client;
	u16 obj_addr, obj_size;
	int error;

	obj_addr = MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T28);
	obj_size = MXT_GET_SIZE(MXT_SPT_CTECONFIG_T28);
	//klogi("address: 0x%x, size: %d", obj_addr, obj_size);

	/* Set CTE config */
	cte_config.ctrl = 0;
	cte_config.cmd = 0;	
	cte_config.mode = 0;
	cte_config.idlegcafdepth = 16;///4;
	cte_config.actvgcafdepth = 63;	//8;
	cte_config.voltage = 0x3c;

	//mxt_write_block(client, 0x01b0, 6, (u8*)&cte_config);
	error = mxt_write_block(client, obj_addr, obj_size, (u8*)&cte_config);
	if (error < 0) {
		kloge( "mxt_write_byte failed!\n");
		return -EIO;
	}
#if MXT_CONFIRM_I2CWRITE
	else {
		u8 buf;
		int ret;
		mxt_read_block(client, obj_addr, obj_size, (u8*)&buf); 
		if (memcmp((u8*)&cte_config, (u8*)&buf, obj_size) == 0) {
			kloge("Verifying config values... Ok");
			ret = 0;
		}
		else {
			kloge("Verifying config values... Failed");
			ret = -1;
		}
		return ret;
	}
#else
	return 0;
#endif
}

int mxt_palmsuppression_config(struct mxt_data *mxt)
{
	struct i2c_client *client = mxt->client;
	u16 obj_addr, obj_size;
	int error;

	obj_addr = MXT_BASE_ADDR(MXT_PROCI_PALMSUPPRESSION_T41);
	obj_size = MXT_GET_SIZE(MXT_PROCI_PALMSUPPRESSION_T41);
	//klogi("address: 0x%x, size: %d", obj_addr, obj_size);

	palmsupression_config.ctrl = 1; 
	palmsupression_config.reserved1 = 0;
	palmsupression_config.reserved2 = 0; 
	palmsupression_config.largeobjthr= 40; 
	palmsupression_config.distancethr= 5;
	palmsupression_config.supextto= 5;

	error = mxt_write_block(client, obj_addr, obj_size, (u8*)&palmsupression_config);
	if (error < 0) {
		kloge( "mxt_write_byte failed!\n");
		return -EIO;
	}

	return 0;
}

int mxt_config_settings(struct mxt_data *mxt)
{
	if (mxt_power_config(mxt)<0)
		return -1;
	if (mxt_acquisition_config(mxt)<0)
		return -1;
	if (mxt_multitouch_config(mxt)<0)
		return -1;
	if (mxt_noise_suppression_config(mxt)<0)
		return -1;
	if (mxt_cte_config(mxt)<0)
		return -1;
	if (mxt_palmsuppression_config(mxt)<0)
		return -1;

	/* backup to nv memory */
	backup_to_nv(mxt);
	/* forces a reset of the chipset */
	reset_chip(mxt, RESET_TO_NORMAL);

	mdelay(350);  // 100ms
	
	return 0;
}

/*
 * Bootloader functions
 */

static void bootloader_status(u8 value)
{
	u8 *str =NULL;

	switch(value) {
	case 0xC0:
		str = "WAITING_BOOTLOAD_CMD"; break;
	case 0x80:
		str = "WAITING_FRAME_DATA"; break;
	case 0x40:
		str = "APP_CRC_FAIL"; break;
	case 0x02:
		str = "FRAME_CRC_CHECK"; break;
	case 0x03:
		str = "FRAME_CRC_FAIL"; break;
	case 0x04:
		str = "FRAME_CRC_PASS"; break;
	default:
		str = "Unknown Status";
	}

	if (mxt_debug)
		printk("bootloader status: %s (0x%02X)\n", str, value);
}

static int check_bootloader(struct i2c_client *client, unsigned int status)
{
	u8 val=0;
	u16 retry=0;

	mdelay(10);  // recommendation from ATMEL

recheck:
	if (i2c_master_recv(client, &val, 1)) {
		kloge("i2c recv failed");
		return -EIO;
	}

	switch (status) {
	case WAITING_BOOTLOAD_COMMAND:
	case WAITING_FRAME_DATA:
		val &= ~BOOTLOAD_STATUS_MASK;
		bootloader_status(val);
		if (val == APP_CRC_FAIL) {
			klogi_if("We've got a APP_CRC_FAIL, so try again (count=%d)", ++retry);
			goto recheck;
		}
		break;

	case FRAME_CRC_PASS:
		bootloader_status(val);
		if (val == FRAME_CRC_CHECK) {
			goto recheck;
		}
		break;

	default:
		return -EINVAL;
	}

	if (val != status) {
		kloge_if("Invalid status: 0x%02X ", val);
		return -EINVAL;
	}
	
	return 0;
}

static int unlock_bootloader(struct i2c_client *client)
{
	u8 cmd[2]={0};
	//printk("(!) unlock_bootloader has been called!\n");

	cmd[0] = 0xdc;  //MXT_CMD_UNLOCK_BL_LSB
	cmd[1] = 0xaa;  //MXT_CMD_UNLOCK_BL_MSB

	return i2c_master_send(client, cmd, 2);
}

int mxt_load_firmware(struct device *dev, const char *fn)
{
	struct i2c_client    *client;
	struct mxt_data      *mxt;
	struct firmware *fw = NULL;

	unsigned int frame_size;
	unsigned int pos = 0;
	unsigned int retry;
	int ret;

	client	  = to_i2c_client(dev);
	mxt 	  = i2c_get_clientdata(client);

#if READ_FW_FROM_HEADER

	fw = kzalloc(sizeof(struct firmware), GFP_KERNEL);

	fw->data = firmware_latest;
	fw->size = sizeof(firmware_latest);
	//klogi("size of firmware: %d", fw->size);
#else
	const struct firmware *fw = NULL;

	ret = request_firmware(&fw, fn, dev);
	if (ret < 0) {
		dev_err(&client->dev, "Unable to open firmware %s\n", fn);
		return -ENOMEM;
	}
#endif

	/* set resets into bootloader mode */
	reset_chip(mxt, RESET_TO_BOOTLOADER);
	mdelay(200);  //mdelay(100);

	/* change to slave address of bootloader */
	/* [SHANKAR] this way of address change may not work, we may have to add dummy client */
	if (mxt->client->addr == MXT_I2C_APP_ADDR) {
		klogi("I2C address: 0x%02X --> 0x%02X", MXT_I2C_APP_ADDR, MXT_I2C_BOOTLOADER_ADDR);
		mxt->client->addr = MXT_I2C_BOOTLOADER_ADDR;
	}

	ret = check_bootloader(mxt->client, WAITING_BOOTLOAD_COMMAND);
	if (ret < 0) {
		kloge("... Waiting bootloader command: Failed");
		goto err_fw;
	}

	/* unlock bootloader */
	unlock_bootloader(mxt->client);
	mdelay(200);  //mdelay(100);

	/* reading the information of the firmware */
	klogi("Firmware info: version [0x%02X], build [0x%02X]", fw->data[0], fw->data[1]);
	printk("Updating progress: ");
	pos += 2;

	while (pos < fw->size) {
		retry=0;
		ret = check_bootloader(mxt->client, WAITING_FRAME_DATA);
		if (ret < 0) {
			kloge("... Waiting frame data: Failed");
			goto err_fw;
		}

		frame_size = ((*(fw->data + pos) << 8) | *(fw->data + pos + 1));

		/* We should add 2 at frame size as the the firmware data is not
		 * included the CRC bytes.
		 */
		frame_size += 2;

		/* write one frame to device */
try_to_resend_the_last_frame:
		i2c_master_send(mxt->client, (u8*)(fw->data + pos), frame_size);

		ret = check_bootloader(mxt->client, FRAME_CRC_PASS);
		if (ret < 0) {
			if (++retry < 10) {
				check_bootloader(mxt->client, WAITING_FRAME_DATA);  // recommendation from ATMEL
				klogi_if("We've got a FRAME_CRC_FAIL, so try again up to 10 times (count=%d)", retry);
				goto try_to_resend_the_last_frame;
			}
			kloge("... CRC on the frame failed after 10 trials!");
			goto err_fw;
		}

		pos += frame_size;

		printk("#");
		klogi_if("%zd / %zd (bytes) updated...", pos, fw->size);
	}
	printk("\nUpdating firmware completed!\n");
	printk("note: You may need to reset this target.\n");

err_fw:
	/* change to slave address of application */
	if (mxt->client->addr == MXT_I2C_BOOTLOADER_ADDR) {
		klogi("I2C address: 0x%02X --> 0x%02X", MXT_I2C_BOOTLOADER_ADDR, MXT_I2C_APP_ADDR);
		mxt->client->addr = MXT_I2C_APP_ADDR;
	}

#if READ_FW_FROM_HEADER
	kfree(fw);
#endif

	return ret;
}

