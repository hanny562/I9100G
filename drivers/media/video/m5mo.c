/*
 * Driver for M5MO (5MP Camera) from NEC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/vmalloc.h>
#include <plat/clock.h>
#include <media/v4l2-common.h>
#include <media/v4l2-subdev.h>
#include <media/m5mo_platform.h>

#include "m5mo.h"
#include "m5mo_firmware.h"

#define M5MO_FIRMWARE_SIZE 2064384
#define M5MO_DRIVER_NAME		"M5MO"
#define SDCARD_FW
#ifdef SDCARD_FW
#define M5MO_FW_PATH			"/sdcard/RS_M5LS.bin"
#else /* SDCARD_FW */
#define M5MO_FW_PATH			"m5mo/RS_M5LS.bin"
#endif /* SDCARD_FW */
#define M5MO_FW_VER_LEN			21
#define M5MO_FW_VER_FILE_CUR	0x16FF00

#define M5MO_FLASH_BASE_ADDR	0x10000000
#define M5MO_INT_RAM_BASE_ADDR	0x68000000

#define M5MO_I2C_RETRY			5
#define M5MO_I2C_VERIFY			100
#define M5MO_ISP_INT_TIMEOUT	1000

#define M5MO_JPEG_MAXSIZE		0x3A0000
#define M5MO_THUMB_MAXSIZE		0xFC00
#define M5MO_POST_MAXSIZE		0xBB800

#define m5mo_readb(c, g, b, v)		m5mo_read(c, 1, g, b, v)
#define m5mo_readw(c, g, b, v)		m5mo_read(c, 2, g, b, v)
#define m5mo_readl(c, g, b, v)		m5mo_read(c, 4, g, b, v)

#define m5mo_writeb(c, g, b, v)		m5mo_write(c, 1, g, b, v)
#define m5mo_writew(c, g, b, v)		m5mo_write(c, 2, g, b, v)
#define m5mo_writel(c, g, b, v)		m5mo_write(c, 4, g, b, v)

#define CHECK_ERR(x)   if ((x) < 0) { \
							cam_err("i2c falied, err %d\n", x); \
						}

#define M5MO_DEBUG

#ifdef M5MO_DEBUG
#define debug(fmt,arg...) printk(KERN_CRIT "--------" fmt "\n",## arg)
#else
#define debug(fmt,arg...)
#endif

#define error(fmt,arg...) printk(KERN_CRIT fmt "\n",## arg)
static struct i2c_client *m5mo_i2c_client;
static unsigned char *firmware_area = NULL;
extern int camera_front_m5mo_on_off(int on_off);

static inline struct m5mo_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct m5mo_state, sd);
}

static int m5mo_read(struct i2c_client *c,
	u8 len, u8 category, u8 byte, int *val)
{
	struct i2c_msg msg;
	unsigned char data[5];
	unsigned char recv_data[len + 1];
	int i, err = 0;

	if (!c->adapter)
		return -ENODEV;

	if (len != 0x01 && len != 0x02 && len != 0x04)
		return -EINVAL;

	msg.addr = c->addr;
	msg.flags = 0;
	msg.len = sizeof(data);
	msg.buf = data;

	/* high byte goes out first */
	data[0] = msg.len;
	data[1] = 0x01;			/* Read category parameters */
	data[2] = category;
	data[3] = byte;
	data[4] = len;

	for (i = M5MO_I2C_RETRY; i; i--) {
		err = i2c_transfer(c->adapter, &msg, 1);
		if (err == 1)
			break;
		msleep(20);
	}

	if (err != 1) {
		cam_err("category %#x, byte %#x\n", category, byte);
		return err;
	}

	msg.flags = I2C_M_RD;
	msg.len = sizeof(recv_data);
	msg.buf = recv_data;
	for(i = M5MO_I2C_RETRY; i; i--) {
		err = i2c_transfer(c->adapter, &msg, 1);
		if (err == 1)
			break;
		msleep(20);
	}

	if (err != 1) {
		cam_err("category %#x, byte %#x\n", category, byte);
		return err;
	}

	if (recv_data[0] != sizeof(recv_data))
		cam_warn("expected length %d, but return length %d\n",
				 sizeof(recv_data), recv_data[0]);

	if (len == 0x01)
		*val = recv_data[1];
	else if (len == 0x02)
		*val = recv_data[1] << 8 | recv_data[2];
	else
		*val = recv_data[1] << 24 | recv_data[2] << 16 |
				recv_data[3] << 8 | recv_data[4];

	cam_dbg("category %#02x, byte %#x, value %#x\n", category, byte, *val);
	return err;
}

static int m5mo_write(struct i2c_client *c,
	u8 len, u8 category, u8 byte, int val)
{
	struct i2c_msg msg;
	unsigned char data[len + 4];
	int i, err;

	if (!c->adapter)
		return -ENODEV;

	if (len != 0x01 && len != 0x02 && len != 0x04)
		return -EINVAL;

	msg.addr = c->addr;
	msg.flags = 0;
	msg.len = sizeof(data);
	msg.buf = data;

	data[0] = msg.len;
	data[1] = 0x02;			/* Write category parameters */
	data[2] = category;
	data[3] = byte;
	if (len == 0x01) {
		data[4] = val & 0xFF;
	} else if (len == 0x02) {
		data[4] = (val >> 8) & 0xFF;
		data[5] = val & 0xFF;
	} else {
		data[4] = (val >> 24) & 0xFF;
		data[5] = (val >> 16) & 0xFF;
		data[6] = (val >> 8) & 0xFF;
		data[7] = val & 0xFF;
	}

	cam_dbg("category %#x, byte %#x, value %#x\n", category, byte, val);

	for (i = M5MO_I2C_RETRY; i; i--) {
		err = i2c_transfer(c->adapter, &msg, 1);
		if (err == 1)
			break;
		msleep(20);
	}

	return err;
}

#if 0
static int m5mo_mem_read(struct i2c_client *c, u16 len, u32 addr, u8 *val)
{
	struct i2c_msg msg;
	unsigned char data[8];
	unsigned char recv_data[len + 3];
	int i, err = 0;
	u16 recv_len;

	if (!c->adapter)
		return -ENODEV;

	if (len <= 0)
		return -EINVAL;

	msg.addr = c->addr;
	msg.flags = 0;
	msg.len = sizeof(data);
	msg.buf = data;

	/* high byte goes out first */
	data[0] = 0x00;
	data[1] = 0x03;
	data[2] = (addr >> 24) & 0xFF;
	data[3] = (addr >> 16) & 0xFF;
	data[4] = (addr >> 8) & 0xFF;
	data[5] = addr & 0xFF;
	data[6] = (len >> 8) & 0xFF;
	data[7] = len & 0xFF;

	for (i = M5MO_I2C_RETRY; i; i--) {
		err = i2c_transfer(c->adapter, &msg, 1);
		if (err == 1)
			break;
		msleep(20);
	}

	if (err != 1)
		return err;

	msg.flags = I2C_M_RD;
	msg.len = sizeof(recv_data);
	msg.buf = recv_data;
	for(i = M5MO_I2C_RETRY; i; i--) {
		err = i2c_transfer(c->adapter, &msg, 1);
		if (err == 1)
			break;
		msleep(20);
	}

	if (err != 1)
		return err;

	if (recv_len != (recv_data[0] << 8 | recv_data[1]))
		cam_warn("expected length %d, but return length %d\n",
				 len, recv_len);

	memcpy(val, recv_data + 1 + sizeof(recv_len), len);

	return err;
}
#endif

static int m5mo_mem_write(struct i2c_client *c, u16 len, u32 addr, u8 *val)
{
	struct i2c_msg msg;
	unsigned char data[len + 8];
	int i, err = 0;

	if (!c->adapter)
		return -ENODEV;

	msg.addr = c->addr;
	msg.flags = 0;
	msg.len = sizeof(data);
	msg.buf = data;

	/* high byte goes out first */
	data[0] = 0x00;
	data[1] = 0x04;
	data[2] = (addr >> 24) & 0xFF;
	data[3] = (addr >> 16) & 0xFF;
	data[4] = (addr >> 8) & 0xFF;
	data[5] = addr & 0xFF;
	data[6] = (len >> 8) & 0xFF;
	data[7] = len & 0xFF;
	memcpy(data + 2 + sizeof(addr) + sizeof(len), val, len);

	for(i = M5MO_I2C_RETRY; i; i--) {
		err = i2c_transfer(c->adapter, &msg, 1);
		if (err == 1)
			break;
		msleep(20);
	}

	return err;
}

#if 0
static inline void m5mo_clear_interrupt(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct m5mo_state *state = to_state(sd);
	int int_factor;

	state->isp.issued = 0;
	m5mo_readb(client, M5MO_CATEGORY_SYS, M5MO_SYS_INT_FACTOR, &int_factor);
}

static irqreturn_t m5mo_isp_isr(int irq, void *dev_id)
{
	struct v4l2_subdev *sd = (struct v4l2_subdev *)dev_id;
	struct m5mo_state *state = to_state(sd);

	cam_dbg("**************** interrupt ****************\n");
	if (!state->isp.issued) {
		wake_up_interruptible(&state->isp.wait);
		state->isp.issued = 1;
	}

	return IRQ_HANDLED;
}

static u32 m5mo_wait_interrupt(struct v4l2_subdev *sd,
	unsigned int timeout)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct m5mo_state *state = to_state(sd);
	u32 int_factor = 0;
	cam_dbg("E\n");

	if (wait_event_interruptible_timeout(state->isp.wait,
		state->isp.issued == 1,
		msecs_to_jiffies(timeout)) == 0) {
		cam_err("timeout\n");
		return 0;
	}

	m5mo_readb(client, M5MO_CATEGORY_SYS, M5MO_SYS_INT_FACTOR, &int_factor);

	cam_dbg("X\n");
	return int_factor;
}

static int m5mo_set_mode(struct i2c_client *client, u32 mode, u32 *old_mode)
{
	int i, err;
	u32 val;
	cam_dbg("E\n");

	err = m5mo_readb(client, M5MO_CATEGORY_SYS, M5MO_SYS_MODE, &val);
	if (err < 0)
		return err;

	if (old_mode)
		*old_mode = val;

	if (val == mode)
		return 0;

	switch (val) {
	case M5MO_SYSINIT_MODE:
		cam_warn("sensor is initializing\n");
		err = -EBUSY;
		break;

	case M5MO_PARMSET_MODE:
		if (mode == M5MO_STILLCAP_MODE) {
			err = m5mo_writeb(client, M5MO_CATEGORY_SYS, M5MO_SYS_MODE, M5MO_MONITOR_MODE);
			if (err < 0)
				return err;
			for (i = M5MO_I2C_VERIFY; i; i--) {
				err = m5mo_readb(client, M5MO_CATEGORY_SYS, M5MO_SYS_MODE, &val);
				if (val == M5MO_MONITOR_MODE)
					break;
				msleep(10);
			}
		}
	case M5MO_MONITOR_MODE:
	case M5MO_STILLCAP_MODE:
		err = m5mo_writeb(client, M5MO_CATEGORY_SYS, M5MO_SYS_MODE, mode);
		break;

	default:
		cam_warn("current mode is unknown\n");
		err = -EINVAL;
	}

	if (err < 0)
		return err;

	for (i = M5MO_I2C_VERIFY; i; i--) {
		err = m5mo_readb(client, M5MO_CATEGORY_SYS, M5MO_SYS_MODE, &val);
		if (val == mode)
			break;
		msleep(10);
	}

	cam_dbg("X\n");
	return err;
}

/* 
 * Turn on(off) Flash led for C1. 
 */
static int m5mo_flash_led_en(bool onoff)
{
	struct regulator *regulator;
	int err = 0;
	
	if(onoff == TRUE) {
		/* Flash mode on */
		/* cam_info("%s: Flash On!!!\n", __func__); */
		regulator = regulator_get(NULL, "led_flash");
		if (IS_ERR(regulator))
			return ENXIO;
		regulator_set_current_limit(regulator, 500000, 500000);
		regulator_enable(regulator);
		regulator_put(regulator);
	} else {
		/* Flash mode off */
		/* cam_info("%s: Flash Off!!!\n", __func__); */
		regulator = regulator_get(NULL, "led_flash");
		if (IS_ERR(regulator))
			return ENXIO;
		regulator_disable(regulator);
		regulator_put(regulator);
	}

	return err;
}

/*
 * v4l2_subdev_video_ops
 */
static const struct m5mo_frmsizeenum *m5mo_get_frmsize(const struct m5mo_frmsizeenum *frmsizes,
	int num_entries, int index)
{
	int i;

	for (i = 0; i < num_entries; i++) {
		if (frmsizes[i].index == index)
			return &frmsizes[i];
	}

	return NULL;
}

static int m5mo_start_capture(struct v4l2_subdev *sd, int val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct m5mo_state *state = to_state(sd);
	int err, int_factor;
	int num, den;
	cam_dbg("E\n");

	m5mo_clear_interrupt(sd);

	err = m5mo_writeb(client, M5MO_CATEGORY_CAPCTRL, M5MO_CAPCTRL_TRANSFER, 0x01);
	int_factor = m5mo_wait_interrupt(sd, M5MO_ISP_INT_TIMEOUT);
	if (!(int_factor & M5MO_INT_CAPTURE)) {
		cam_warn("M5MO_INT_CAPTURE isn't issued\n");
		return -ETIMEDOUT;
	}

	err = m5mo_readl(client, M5MO_CATEGORY_CAPCTRL, M5MO_CAPCTRL_IMG_SIZE,
				&state->jpeg.main_size);
	CHECK_ERR(err);
	err = m5mo_readl(client, M5MO_CATEGORY_CAPCTRL, M5MO_CAPCTRL_THUMB_SIZE,
				&state->jpeg.thumb_size);
	CHECK_ERR(err);

	state->jpeg.main_offset = 0;
	state->jpeg.thumb_offset = M5MO_JPEG_MAXSIZE;
	state->jpeg.postview_offset = M5MO_JPEG_MAXSIZE + M5MO_THUMB_MAXSIZE;

	/* EXIF */
	err = m5mo_readl(client, M5MO_CATEGORY_EXIF, M5MO_EXIF_EXPTIME_NUM, &num);
	CHECK_ERR(err);
	err = m5mo_readl(client, M5MO_CATEGORY_EXIF, M5MO_EXIF_EXPTIME_DEN, &den);
	CHECK_ERR(err);
	state->exif.exptime = (u32)num*1000000/den;

	err = m5mo_readw(client, M5MO_CATEGORY_EXIF, M5MO_EXIF_FLASH, &num);
	CHECK_ERR(err);
	state->exif.flash = (u16)num;

	err = m5mo_readw(client, M5MO_CATEGORY_EXIF, M5MO_EXIF_ISO, &num);
	CHECK_ERR(err);
	state->exif.iso = (u16)num;

	err = m5mo_readl(client, M5MO_CATEGORY_EXIF, M5MO_EXIF_TV_NUM, &num);
	CHECK_ERR(err);
	err = m5mo_readl(client, M5MO_CATEGORY_EXIF, M5MO_EXIF_TV_DEN, &den);
	CHECK_ERR(err);
	state->exif.tv = num/den;

	err = m5mo_readl(client, M5MO_CATEGORY_EXIF, M5MO_EXIF_BV_NUM, &num);
	CHECK_ERR(err);
	err = m5mo_readl(client, M5MO_CATEGORY_EXIF, M5MO_EXIF_BV_DEN, &den);
	CHECK_ERR(err);
	state->exif.bv = num/den;

	err = m5mo_readl(client, M5MO_CATEGORY_EXIF, M5MO_EXIF_EBV_NUM, &num);
	CHECK_ERR(err);
	err = m5mo_readl(client, M5MO_CATEGORY_EXIF, M5MO_EXIF_EBV_DEN, &den);
	CHECK_ERR(err);
	state->exif.ebv = num/den;

	cam_dbg("X\n");
	return err;
}
#endif

static int
m5mo_program_fw(struct i2c_client *c, u8 *buf, u32 addr, u32 unit, u32 count)
{
	u32 val;
	u32 intram_unit = 0x800;
	int i, j, retries, err = 0;

	for (i = 0; i < count; i++) {
		/* Set Flash ROM memory address */
		err = m5mo_writel(c, M5MO_CATEGORY_FLASH, M5MO_FLASH_ADDR, addr);
		CHECK_ERR(err);

		/* Erase FLASH ROM entire memory */
		err = m5mo_writeb(c, M5MO_CATEGORY_FLASH, M5MO_FLASH_ERASE, 0x01);
		CHECK_ERR(err);
		/* Response while sector-erase is operating */
		retries = 0;
		do {
			mdelay(10);
			err = m5mo_readb(c, M5MO_CATEGORY_FLASH, M5MO_FLASH_ERASE, &val);
			CHECK_ERR(err);
		} while (val && retries++ < M5MO_I2C_VERIFY);

		/* Set FLASH ROM programming size */
		err = m5mo_writew(c, M5MO_CATEGORY_FLASH, M5MO_FLASH_BYTE,
									unit == SZ_64K ? 0 : unit);
		CHECK_ERR(err);

		/* Clear M-5MoLS internal RAM */
		err = m5mo_writeb(c, M5MO_CATEGORY_FLASH, M5MO_FLASH_RAM_CLEAR, 0x01);
		CHECK_ERR(err);

		/* Set Flash ROM programming address */
		err = m5mo_writel(c, M5MO_CATEGORY_FLASH, M5MO_FLASH_ADDR, addr);
		CHECK_ERR(err);

		/* Send programmed firmware */
		for (j = 0; j < unit; j += intram_unit) {
			err = m5mo_mem_write(c, intram_unit, M5MO_INT_RAM_BASE_ADDR + j,
													buf + (i * unit) + j);
			CHECK_ERR(err);
			mdelay(10);
		}

		/* Start Programming */
		err = m5mo_writeb(c, M5MO_CATEGORY_FLASH, M5MO_FLASH_WR, 0x01);
		CHECK_ERR(err);

		/* Confirm programming has been completed */
		retries = 0;
		do {
			mdelay(10);
			err = m5mo_readb(c, M5MO_CATEGORY_FLASH, M5MO_FLASH_WR, &val);
			CHECK_ERR(err);
		} while (val && retries++ < M5MO_I2C_VERIFY);

		/* Increase Flash ROM memory address */
		addr += unit;
	}
	return 0;
}

#if 0
static int m5mo_load_fw(struct i2c_client *client)
{	
	u8 *buf, val;
	int err;
#if 0
#ifdef SDCARD_FW
	struct file *fp;
	mm_segment_t old_fs;
	long fsize, nread;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	fp = filp_open(M5MO_FW_PATH, O_RDONLY, S_IRUSR);
	if (IS_ERR(fp)) {
		cam_err("failed to open %s\n", M5MO_FW_PATH);
		return -ENOENT;
	}

	fsize = fp->f_path.dentry->d_inode->i_size;
	cam_info("start, file path %s, size %ld Bytes\n", M5MO_FW_PATH, fsize);

	buf = vmalloc(fsize);
	if (!buf) {
		cam_err("failed to allocate memory\n");
		return -ENOMEM;
	}

	nread = vfs_read(fp, (char __user *)buf, fsize, &fp->f_pos);
	if (nread != fsize) {
		cam_err("failed to read firmware file, nread %ld Bytes\n", nread);
		return -EIO;
	}
#else /* SDCARD_FW */
	struct device *dev = &client->adapter->dev;
	const struct firmware *fentry;

	err = request_firmware(&fentry, M5MO_FW_PATH, dev);
	if (err != 0) {
		cam_err("request_firmware falied\n");
		return -EINVAL;
	}

	buf = (u8 *)fentry->data;
#endif /* SDCARD_FW */
#endif
	buf = (u8 *)firmware_area;
//	buf = (u8 *)RS_M5LS_bin;

	/* set pin */
	val = 0x7E;
	err = m5mo_mem_write(client, sizeof(val), 0x50000308, &val);
	CHECK_ERR(err);
	/* select flash memory */
	err = m5mo_writeb(client, M5MO_CATEGORY_FLASH, M5MO_FLASH_SEL, 0x01);
	CHECK_ERR(err);
	/* program FLSH ROM */
	err = m5mo_program_fw(client, buf, M5MO_FLASH_BASE_ADDR, SZ_64K, 31);
	CHECK_ERR(err);
	err = m5mo_program_fw(client, buf, M5MO_FLASH_BASE_ADDR + SZ_64K * 31, SZ_8K, 4);
	CHECK_ERR(err);

	cam_info("end\n");

#if 0
#ifdef SDCARD_FW
	vfree(buf);
	filp_close(fp, current->files);
	set_fs(old_fs);
#endif  /* SDCARD_FW */
#endif
	return 0;
}

static int m5mo_check_version(struct i2c_client *client)
{
	u32 ver_chip, ver_fw, ver_hw, ver_param, ver_awb;
	m5mo_readb(client, M5MO_CATEGORY_SYS, M5MO_SYS_PJT_CODE, &ver_chip);
	m5mo_readw(client, M5MO_CATEGORY_SYS, M5MO_SYS_VER_FW, &ver_fw);
	m5mo_readw(client, M5MO_CATEGORY_SYS, M5MO_SYS_VER_HW, &ver_hw);
	m5mo_readw(client, M5MO_CATEGORY_SYS, M5MO_SYS_VER_PARAM, &ver_param);
	m5mo_readw(client, M5MO_CATEGORY_SYS, M5MO_SYS_VER_AWB, &ver_awb);

	cam_info("****************************************\n");
	cam_info("Chip\tF/W\tH/W\tParam\tAWB\n");
	cam_info("----------------------------------------\n");
	cam_info("%#02x\t%#04x\t%#04x\t%#04x\t%#04x\n",
			 ver_chip, ver_fw, ver_hw, ver_param, ver_awb);
	cam_info("****************************************\n");

	return 0;
}

static int m5mo_init_param(struct i2c_client *client)
{	
	int i, err;
	cam_dbg("E\n");

	err = m5mo_writeb(client, M5MO_CATEGORY_PARM, M5MO_PARM_OUT_SEL, 0x02);
	CHECK_ERR(err);

	err = m5mo_writeb(client, M5MO_CATEGORY_CAPPARM, M5MO_CAPPARM_YUVOUT_MAIN, 0x21);
	CHECK_ERR(err);

	err = m5mo_writel(client, M5MO_CATEGORY_CAPPARM, M5MO_CAPPARM_THUMB_JPEG_MAX, M5MO_THUMB_MAXSIZE);
	CHECK_ERR(err);

	cam_dbg("X\n");
	return 0;
}

static int m5mo_init(struct i2c_client *client)
{	
	
	u32 int_factor;
	int err, cam_mode=0xFF,prm_ver=0xEE,hw_ver=0xEE,fw_ver=0xEE;	

	printk(KERN_ERR"\n\n In M5MO INIT \n");
	/* start camera program(parallel FLASH ROM) */
	err = m5mo_writeb(client, M5MO_CATEGORY_FLASH, M5MO_FLASH_CAM_START, 0x01);
	CHECK_ERR(err);

	#if 0
	int_factor = m5mo_wait_interrupt(sd, M5MO_ISP_INT_TIMEOUT);
	if (!(int_factor & M5MO_INT_MODE)) {
		cam_err("firmware was erased?\n");
		return -ETIMEDOUT;
	}
	#endif
	mdelay(1000);
	
	err = m5mo_readb(client, M5MO_CATEGORY_FLASH, M5MO_FLASH_CAM_START, &cam_mode);
	CHECK_ERR(err);

	printk(KERN_ERR"\n\n\n\n \t\t ################################################ \n\n");
	printk(KERN_ERR"\t\t\t\t ->->->->->->->-> CAM_MODE = %x <-<-<-<-<-<-<-<- ", cam_mode);
	printk(KERN_ERR"\n\n \t\t ################################################ \n\n");


	err = m5mo_readb(client, 0x00, 0x04, &hw_ver);
	CHECK_ERR(err);


	err = m5mo_readb(client, 0x00, 0x06, &prm_ver);
	CHECK_ERR(err);

	err = m5mo_readb(client, 0x00, 0x02, &fw_ver);
	CHECK_ERR(err);

	printk(KERN_ERR"\n\n\n\n \t\t ################################################ \n\n");
	printk(KERN_ERR"\t\t\t\t ->->->->->->->-> HW_VER=%x, PRM_VER=%x, FW_VER=%x <-<-<-<-<-<-<-<- ", hw_ver, prm_ver, fw_ver);
	printk(KERN_ERR"\n\n \t\t ################################################ \n\n");

	/* check up F/W version */
	//err = m5mo_check_version(client);
	//CHECK_ERR(err);

	m5mo_init_param(client);

	return 0;
}

/*
 * s_config subdev ops
 * With camera device, we need to re-initialize every single opening time therefor,
 * it is not necessary to be initialized on probe time. except for version checking
 * NOTE: version checking is optional
 */
static int m5mo_s_config(struct v4l2_subdev *sd, int irq, void *platform_data)
{
	struct m5mo_state *state = to_state(sd);
	struct m5mo_platform_data *pdata = (struct m5mo_platform_data *)platform_data;
	int err = 0;

	/* Default state values */
	state->initialized = 0;
	state->isp.issued = 1;

	state->colorspace = 0;
	state->frmsize = NULL;

//	state->sensor_mode = SENSOR_CAMERA;
//	state->flash_mode = FLASH_MODE_OFF;
	
	state->fps = 0;			/* auto */

	state->jpeg.main_size = 0;
	state->jpeg.main_offset = 0;
	state->jpeg.postview_offset = 0;

	/* Register ISP irq */
	if (!pdata) {
		cam_err("no platform data\n");
		return -ENODEV;
	}

#if 1 /* C1 */
	if (irq) {
		//if (pdata->config_isp_irq)
			//pdata->config_isp_irq();
#else /* U1 */
	if (pdata->config_gpio)
		pdata->config_gpio();
	
	if (irq) {
#endif
 		err = request_irq(irq, m5mo_isp_isr, IRQF_TRIGGER_RISING, "m5mo isp", sd);
		if (err) {
			cam_err("failed to request irq\n");
			return err;
		}
		state->isp.irq = irq;

		/* wait queue initialize */
		init_waitqueue_head(&state->isp.wait);
	}

	return 0;
}

static int m5mo_get_standard_fw_version(char *str, struct m5mo_fw_version *ver)
{	
	switch (ver->company) {
	case 'O':
		str[0] = '0';
		str[1] = 'P';
		break;

	case 'S':
		str[0] = 'S';
		str[1] = 'E';
		break;

	case 'T':
		str[0] = 'T';
		str[1] = 'E';
		break;
	}

	str[2] = 'S';
	str[3] = 'A';
	str[4] = ver->year;
	str[5] = ver->month;
	str[6] = ver->day[0];
	str[7] = ver->day[1];
	str[8] = '\0';

	return 0;	
}

static int m5mo_get_sensor_fw_version(struct i2c_client *client,char *str)
{	
	struct m5mo_fw_version ver;
	u8 buf[M5MO_FW_VER_LEN] = {0, };
	int i = 0, val, err;

	do {
		err = m5mo_readb(client, M5MO_CATEGORY_SYS, M5MO_SYS_USER_VER, &val);
		CHECK_ERR(err);
		buf[i++] = (char)val;
	} while ((char)val != '\0' && i < sizeof(buf));
	
	memcpy(&ver, buf, sizeof(ver));
	
	m5mo_get_standard_fw_version(str, &ver);
	
	cam_info("%s\n", str);
	return 0;
}
#endif

static int m5mo_load_fw(struct i2c_client *client)
{
	u8 *buf, val;
	int err;

	cam_info("firmware load start\n");
	buf = (u8 *)firmware_area;
//	buf = (u8 *)RS_M5LS_bin;

	/* set pin */
	val = 0x7E;
	err = m5mo_mem_write(client, sizeof(val), 0x50000308, &val);
	CHECK_ERR(err);

	/* select flash memory */
	err = m5mo_writeb(client, M5MO_CATEGORY_FLASH, M5MO_FLASH_SEL, 0x01);
	CHECK_ERR(err);

	/* program FLASH ROM */
	err = m5mo_program_fw(client, buf, M5MO_FLASH_BASE_ADDR, SZ_64K, 31);
	CHECK_ERR(err);
	err = m5mo_program_fw(client, buf, M5MO_FLASH_BASE_ADDR + SZ_64K * 31, SZ_8K, 4);
	CHECK_ERR(err);

	cam_info("firmware load end\n");
	return 0;
}
static void m5mo_test(struct i2c_client *client);
static ssize_t m5mo_firmware_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	camera_front_m5mo_on_off(1);
	m5mo_test(m5mo_i2c_client);
	msleep(20 * 1000);
	camera_front_m5mo_on_off(0);

	return 0;
}

static unsigned char *f_curr_p = NULL;
static unsigned int total_firmware_len = 0; /* counter for total length */
static ssize_t m5mo_firmware_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	if (firmware_area == NULL) {
		printk("Storing firmware started\n");

		/* Allocate 3MB to hold the firmware */
		firmware_area = vmalloc(3 * SZ_1M);

		/* Reset the variables */
		f_curr_p = firmware_area;

		total_firmware_len = 0;
	}

	total_firmware_len += count;
	printk("firmware size is now = %d\n", total_firmware_len);

	if (total_firmware_len > (3 * SZ_1M)) {
		printk(KERN_ERR "Invalid firmware length\n");
		return -EINVAL;
	}

	/* Copy the firmware */
	memcpy(f_curr_p, buf, count);

	f_curr_p = f_curr_p + count;

	/* Now start programming the sensor flash memory */
	if (total_firmware_len ==  M5MO_FIRMWARE_SIZE) {
		/* Power on the camera */
		camera_front_m5mo_on_off(1);

		/* Write the firmware */
		m5mo_load_fw(m5mo_i2c_client);

		/* Power off the camera */
		camera_front_m5mo_on_off(0);

		vfree(firmware_area);
		firmware_area = NULL;
	}
	return count;
}
static DEVICE_ATTR(m5mo_firmware, S_IWUSR | S_IRUGO, m5mo_firmware_show, m5mo_firmware_store);

static void m5mo_poll_int(void)
{
	int val = 0;

	if(gpio_request(138, "M5MO_INT") != 0)
	        printk(KERN_ERR"\n Failed to request GPIO 138 \n");

	gpio_direction_input(138);
	while(1) {
		val = gpio_get_value(138);
	        printk(KERN_ERR"Int pin value = %d\n", val);
		if (val == 1)
			break;
	}
	gpio_free(138);
}

static void m5mo_test(struct i2c_client *client)
{
	int err = 0;
	int reg_val;
	int prm_ver, hw_ver, fw_ver;

	cam_info("gonna start firware program\n");

	if (gpio_request(138, "M5MO_INT") != 0)
	        printk(KERN_ERR"\n Failed to request GPIO 138 \n");
	gpio_direction_output(138, 0);
	gpio_free(138);

	err = m5mo_writeb(client, M5MO_CATEGORY_FLASH, M5MO_FLASH_CAM_START, 0x01);
	CHECK_ERR(err);
	m5mo_poll_int();
	mdelay(1000);

	err = m5mo_readb(client, 0x00, 0x10, &reg_val);
	CHECK_ERR(err);
	printk(KERN_ERR"Interrupt status reg =%x \n", reg_val);

	err = m5mo_writeb(client, 0x01, 0x01, 0x07);
	CHECK_ERR(err);

	err = m5mo_writeb(client, 0x01, 0x1a, 0x01);
	CHECK_ERR(err);

	err = m5mo_writeb(client, 0x00, 0x11, 0x01);
	CHECK_ERR(err);

	err = m5mo_writeb(client, 0x00, 0x0b, 0x02);
	CHECK_ERR(err);

	m5mo_poll_int();
	err = m5mo_readb(client, 0x00, 0x10, &reg_val);
	        printk(KERN_ERR"Interrupt status reg =%x \n", reg_val);

	err = m5mo_readw(client, 0x00, 0x04, &hw_ver);
	CHECK_ERR(err);
	err = m5mo_readw(client, 0x00, 0x06, &prm_ver);
	CHECK_ERR(err);
	err = m5mo_readw(client, 0x00, 0x02, &fw_ver);
	CHECK_ERR(err);

	printk(KERN_ERR"\n\n\n\n \t\t ################################################ \n\n");
	printk(KERN_ERR"\t\t\t\t ->->->->->->->-> HW_VER=%x, PRM_VER=%x, FW_VER=%x <-<-<-<-<-<-<-<- ", hw_ver, prm_ver, fw_ver);
	printk(KERN_ERR"\n\n \t\t ################################################ \n\n");
}

static int m5mo_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;

	cam_dbg("\n$$$$$ IN M5MO PROBE  $$$$$$\n");

	/* Request all the needed gpios */
	if (gpio_request(43, "CAM_PMIC_EN") != 0)
		printk(KERN_ERR"Failed to request GPIO 43 \n");

	if (gpio_request(139, "CAM_NRST") != 0)
		printk(KERN_ERR"Failed to request GPIO 139 \n");

	/* Save the client details */
	m5mo_i2c_client = client;

	ret = sysfs_create_file(&client->dev.kobj, &dev_attr_m5mo_firmware.attr);
	if (ret)
		printk(KERN_ERR "M5MO: %s: sysfs create entry failed\n", __func__);

#if  0
	camera_front_m5mo_on_off(1);
	m5mo_test(client);
	m5mo_load_fw(client);
	camera_front_m5mo_on_off(0);
#endif
	return 0;
}

static int m5mo_remove(struct i2c_client *client)
{
	cam_dbg("m5mo i2c driver m5mo_remove called");
	sysfs_remove_file(&client->dev.kobj, &dev_attr_m5mo_firmware.attr);
	m5mo_i2c_client = NULL;
	gpio_free(43);
	gpio_free(139);
	return 0;
}

static const struct i2c_device_id m5mo_id[] = {
	{ M5MO_DRIVER_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, m5mo_id);

static struct i2c_driver m5mo_i2c_driver =
{
	.driver = {
		 .name = M5MO_DRIVER_NAME,
	},
	.probe = m5mo_probe,
	.remove = m5mo_remove,
	.id_table = m5mo_id,
};

int __init m5mo_driver_init(void)
{
	int ret = 0;
	cam_dbg("m5mo_driver_init called");

	/*Add the i2c driver*/
	ret = i2c_add_driver(&m5mo_i2c_driver);
	if (ret)
	       error("m5mo i2c_add_driver failed");

	cam_dbg("m5mo_driver_init successful");
	return ret;
}

void __exit m5mo_driver_exit(void)
{
	cam_dbg("m5mo_driver_exit called");
	/*Delete the i2c driver*/
	i2c_del_driver(&m5mo_i2c_driver);
}

module_init(m5mo_driver_init);
module_exit(m5mo_driver_exit);

MODULE_DESCRIPTION("Fujitsu M5MO LS 8MP ISP i2c driver");
MODULE_AUTHOR("uma.shankar <@samsung.com>");
MODULE_LICENSE("GPL");
