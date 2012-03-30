/* /driver/input/misc/bmp182.c
 * Copyright (C) 2011 Samsung Electronics. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/workqueue.h>

#define BMP182_CHIP_ID			0x55
#define BMP182_CALIBRATION_DATA_START	0xAA
#define BMP182_CALIBRATION_DATA_LENGTH	11
#define BMP182_CHIP_ID_REG		0xD0
#define BMP182_CTRL_REG			0xF4
#define BMP182_TEMP_MEASUREMENT		0x2E
#define BMP182_PRESSURE_MEASUREMENT	0x34
#define BMP182_CONVERSION_REGISTER_MSB	0xF6
#define BMP182_CONVERSION_REGISTER_LSB	0xF7
#define BMP182_CONVERSION_REGISTER_XLSB	0xF8
#define BMP182_TEMP_CONVERSION_TIME	5

struct bmp182_calibration_data {
	s16 AC1, AC2, AC3;
	u16 AC4, AC5, AC6;
	s16 B1, B2;
	s16 MB, MC, MD;
};

/* driver data */
struct bmp182_data {
	struct bmp182_calibration_data calibration;
	struct input_dev *pressure_input_dev;
	struct i2c_client *i2c_client;
	struct work_struct work_pressure;
	struct hrtimer timer;
	struct mutex lock;
	struct workqueue_struct *pressure_wq;
	struct bmp182_platform_data *pdata;
	bool enabled;
	int als_index_count;
	ktime_t poll_delay;
	u8 oversampling_setting;
	u32 raw_temperature;
	u32 raw_pressure;
	u32 temp_measurement_period;
	u32 last_temp_measurement;
	s32 b6; /* calculated temperature correction coefficient */
};

static s32 bmp182_read_calibration_data(struct bmp182_data *data)
{
	struct bmp182_calibration_data *cali = &(data->calibration);
	u16 tmp[BMP182_CALIBRATION_DATA_LENGTH];
	s32 ret = i2c_smbus_read_i2c_block_data(data->i2c_client,
				BMP182_CALIBRATION_DATA_START,
				BMP182_CALIBRATION_DATA_LENGTH*sizeof(u16),
				(u8 *)tmp);
	if (ret < 0) {
		pr_err("%s: failed to read calibration data by i2c\n",
								__func__);
		return ret;
	}

	if (ret != BMP182_CALIBRATION_DATA_LENGTH*sizeof(u16)) {
		pr_err("%s: wrong i2c data length = %d\n", __func__, ret);
		return -EIO;
	}

	cali->AC1 = be16_to_cpu(tmp[0]);
	cali->AC2 = be16_to_cpu(tmp[1]);
	cali->AC3 = be16_to_cpu(tmp[2]);
	cali->AC4 = be16_to_cpu(tmp[3]);
	cali->AC5 = be16_to_cpu(tmp[4]);
	cali->AC6 = be16_to_cpu(tmp[5]);
	cali->B1 = be16_to_cpu(tmp[6]);
	cali->B2 = be16_to_cpu(tmp[7]);
	cali->MB = be16_to_cpu(tmp[8]);
	cali->MC = be16_to_cpu(tmp[9]);
	cali->MD = be16_to_cpu(tmp[10]);

	return 0;
}

static s32 bmp182_update_raw_temperature(struct bmp182_data *data)
{
	u16 tmp;
	s32 ret;

	mutex_lock(&data->lock);
	ret = i2c_smbus_write_byte_data(data->i2c_client,
				BMP182_CTRL_REG, BMP182_TEMP_MEASUREMENT);
	if (ret != 0) {
		pr_err("%s: failed to write i2c\n", __func__);
		goto exit;
	}
	msleep(BMP182_TEMP_CONVERSION_TIME);

	ret = i2c_smbus_read_i2c_block_data(data->i2c_client,
		BMP182_CONVERSION_REGISTER_MSB, sizeof(tmp), (u8 *)&tmp);
	if (ret < 0) {
		pr_err("%s: failed to read i2c\n", __func__);
		goto exit;
	}
	if (ret != sizeof(tmp)) {
		pr_err("%s: wrong i2c data length = %d\n", __func__, ret);
		ret = -EIO;
		goto exit;
	}
	data->raw_temperature = be16_to_cpu(tmp);
	data->last_temp_measurement = jiffies;
	ret = 0; /* everything ok, return 0 */

exit:
	mutex_unlock(&data->lock);
	return ret;
}

static s32 bmp182_update_raw_pressure(struct bmp182_data *data)
{
	u32 tmp = 0;
	s32 ret;

	mutex_lock(&data->lock);
	ret = i2c_smbus_write_byte_data(data->i2c_client,
		BMP182_CTRL_REG, BMP182_PRESSURE_MEASUREMENT +
		(data->oversampling_setting << 6));
	if (ret != 0) {
		pr_err("%s: failed to write i2c\n", __func__);
		goto exit;
	}

	/* wait for the end of conversion */
	msleep(2 + (3 << data->oversampling_setting));

	/* copy data into a u32 (4 bytes), but skip the first byte. */
	ret = i2c_smbus_read_i2c_block_data(data->i2c_client,
			BMP182_CONVERSION_REGISTER_MSB, 3, ((u8 *)&tmp)+1);
	if (ret < 0) {
		pr_err("%s: failed to read i2c\n", __func__);
		goto exit;
	}
	if (ret != 3) {
		pr_err("%s: wrong i2c data length = %d\n", __func__, ret);
		ret = -EIO;
		goto exit;
	}
	data->raw_pressure = be32_to_cpu((tmp));
	data->raw_pressure >>= (8 - data->oversampling_setting);
	ret = 0; /* everything ok, return 0 */

exit:
	mutex_unlock(&data->lock);
	return ret;
}

/*
 * This function starts the temperature measurement and returns the value
 * in tenth of a degree celsius.
 */
static s32 bmp182_get_temperature(struct bmp182_data *data, int *temperature)
{
	struct bmp182_calibration_data *cali = &data->calibration;
	long x1, x2;
	int ret;

	ret = bmp182_update_raw_temperature(data);
	if (ret != 0)
		goto exit;

	x1 = ((data->raw_temperature - cali->AC6) * cali->AC5) >> 15;
	x2 = (cali->MC << 11) / (x1 + cali->MD);
	data->b6 = x1 + x2 - 4000;
	/* if NULL just update b6. Used for pressure only measurements */
	if (temperature != NULL)
		*temperature = (x1 + x2 + 8) >> 4;

exit:
	return ret;
}

/*
 * This function starts the pressure measurement and returns the value
 * in millibar. Since the pressure depends on the ambient temperature,
 * a temperature measurement is executed according to the given temperature
 * measurememt period (default is 1 sec boundary). This period could vary
 * and needs to be adjusted accoring to the sensor environment, i.e. if big
 * temperature variations then the temperature needs to be read out often.
 */
static s32 bmp182_get_pressure(struct bmp182_data *data, int *pressure)
{
	struct bmp182_calibration_data *cali = &data->calibration;
	int ret;
	s32 x1, x2, x3, b3;
	u32 b4, b7;
	s32 p;

	/* update the ambient temperature according to the given meas. period */
	if (data->last_temp_measurement +
			data->temp_measurement_period < jiffies) {
		ret = bmp182_get_temperature(data, NULL);
		if (ret != 0)
			goto exit;
	}

	ret = bmp182_update_raw_pressure(data);
	if (ret != 0)
		goto exit;

	x1 = (data->b6 * data->b6) >> 12;
	x1 *= cali->B2;
	x1 >>= 11;

	x2 = cali->AC2 * data->b6;
	x2 >>= 11;

	x3 = x1 + x2;

	b3 = (((((s32)cali->AC1) * 4 + x3) << data->oversampling_setting) + 2);
	b3 >>= 2;

	x1 = (cali->AC3 * data->b6) >> 13;
	x2 = (cali->B1 * ((data->b6 * data->b6) >> 12)) >> 16;
	x3 = (x1 + x2 + 2) >> 2;
	b4 = (cali->AC4 * (u32)(x3 + 32768)) >> 15;

	b7 = ((u32)data->raw_pressure - b3) *
					(50000 >> data->oversampling_setting);
	p = ((b7 < 0x80000000) ? ((b7 << 1) / b4) : ((b7 / b4) * 2));

	x1 = p >> 8;
	x1 *= x1;
	x1 = (x1 * 3038) >> 16;
	x2 = (-7357 * p) >> 16;
	p += (x1 + x2 + 3791) >> 4;

	*pressure = p;

exit:
	return ret;
}

/*
 * This function sets the chip-internal oversampling. Valid values are 0..3.
 * The chip will use 2^oversampling samples for internal averaging.
 * This influences the measurement time and the accuracy; larger values
 * increase both. The datasheet gives on overview on how measurement time,
 * accuracy and noise correlate.
 */
static void bmp182_set_oversampling(struct bmp182_data *data,
						unsigned char oversampling)
{
	if (oversampling > 3)
		oversampling = 3;
	data->oversampling_setting = oversampling;
}

static ssize_t oversampling_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct bmp182_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%u\n", data->oversampling_setting);
}

static ssize_t oversampling_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct bmp182_data *data = dev_get_drvdata(dev);
	unsigned long oversampling;
	int success = strict_strtoul(buf, 10, &oversampling);
	if (success == 0) {
		mutex_lock(&data->lock);
		bmp182_set_oversampling(data, oversampling);
		mutex_unlock(&data->lock);
		return count;
	}
	return success;
}

static void bmp182_enable(struct bmp182_data *bmp182)
{
	hrtimer_start(&bmp182->timer, bmp182->poll_delay,
						HRTIMER_MODE_REL);
}

static void bmp182_pressure_disable(struct bmp182_data *bmp182)
{
	hrtimer_cancel(&bmp182->timer);
	cancel_work_sync(&bmp182->work_pressure);
}

static ssize_t poll_delay_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct bmp182_data *bmp182 = dev_get_drvdata(dev);
	return sprintf(buf, "%lld\n", ktime_to_ns(bmp182->poll_delay));
}

static ssize_t poll_delay_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct bmp182_data *bmp182 = dev_get_drvdata(dev);
	int64_t new_delay;
	int err;

	err = strict_strtoll(buf, 10, &new_delay);
	if (err < 0)
		return err;

	mutex_lock(&bmp182->lock);
	if (new_delay != ktime_to_ns(bmp182->poll_delay)) {
		bmp182->poll_delay = ns_to_ktime(new_delay);
		if (bmp182->enabled) {
			bmp182_pressure_disable(bmp182);
			bmp182_enable(bmp182);
		}
	}
	mutex_unlock(&bmp182->lock);

	return size;
}

static ssize_t enable_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct bmp182_data *bmp182 = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", (bmp182->enabled) ? 1 : 0);
}

static ssize_t enable_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t size)
{
	struct bmp182_data *bmp182 = dev_get_drvdata(dev);
	bool new_value;

	if (sysfs_streq(buf, "1"))
		new_value = true;
	else if (sysfs_streq(buf, "0"))
		new_value = false;
	else {
		pr_err("%s: invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}

	mutex_lock(&bmp182->lock);
	if (new_value && !(bmp182->enabled)) {
		bmp182->enabled = 1;
		bmp182_enable(bmp182);
	} else if (!new_value && (bmp182->enabled)) {
		bmp182_pressure_disable(bmp182);
		bmp182->enabled = 0;
	}
	mutex_unlock(&bmp182->lock);
	return size;
}

static DEVICE_ATTR(oversampling, S_IRUGO | S_IWUSR | S_IWGRP,
			oversampling_show, oversampling_store);

static DEVICE_ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
			poll_delay_show, poll_delay_store);

static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
			enable_show, enable_store);

static struct attribute *pressure_sysfs_attrs[] = {
	&dev_attr_oversampling.attr,
	&dev_attr_enable.attr,
	&dev_attr_poll_delay.attr,
	NULL
};

static struct attribute_group pressure_attribute_group = {
	.attrs = pressure_sysfs_attrs,
};

static void bmp182_work_func_pressure(struct work_struct *work)
{
	int pressure = 0, temp = 0;
	struct bmp182_data *bmp182 = container_of(work, struct bmp182_data,
					      work_pressure);
	bmp182_get_pressure(bmp182, &pressure);
	input_report_abs(bmp182->pressure_input_dev, ABS_PRESSURE, pressure);
	bmp182_get_temperature(bmp182, &temp);
	input_report_abs(bmp182->pressure_input_dev, ABS_MISC, temp);
	input_sync(bmp182->pressure_input_dev);
}

static enum hrtimer_restart bmp182_timer_func(struct hrtimer *timer)
{
	struct bmp182_data *bmp182
			= container_of(timer, struct bmp182_data, timer);
	queue_work(bmp182->pressure_wq, &bmp182->work_pressure);
	hrtimer_forward_now(&bmp182->timer, bmp182->poll_delay);
	return HRTIMER_RESTART;
}

static int bmp182_setup(struct bmp182_data *data,
			      struct bmp182_platform_data *pdata)
{
	int ret = bmp182_read_calibration_data(data);
	if (ret != 0) {
		pr_err("%s: failed to read calibration data\n", __func__);
		goto exit;
	}
	data->last_temp_measurement = 0;
	data->temp_measurement_period = 1*HZ;
	data->oversampling_setting = 3;
exit:
	return ret;
}

static int bmp182_i2c_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct bmp182_data *bmp182;
	struct input_dev *input_dev;
	int ret = -ENODEV;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s: i2c functionality check failed\n", __func__);
		return -ENODEV;
	}

	if (i2c_smbus_read_byte_data(client,
			BMP182_CHIP_ID_REG) != BMP182_CHIP_ID) {
		pr_err("%s: chip_id check failed\n", __func__);
		return -ENODEV;
	}

	bmp182 = kzalloc(sizeof(struct bmp182_data), GFP_KERNEL);
	if (!bmp182) {
		pr_err("%s: failed to alloc memory for module data\n",
		       __func__);
		return -ENOMEM;
	}

	bmp182->pdata = client->dev.platform_data;
	bmp182->i2c_client = client;
	i2c_set_clientdata(client, bmp182);
	mutex_init(&bmp182->lock);

	/* initial setup */
	ret = bmp182_setup(bmp182, bmp182->pdata);
	if (ret != 0) {
		pr_err("%s: bmp182_setup() failed\n", __func__);
		goto err_setup;
	}

	/* timer settings. we poll for pressure values using a timer */
	hrtimer_init(&bmp182->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	bmp182->poll_delay = ns_to_ktime(200 * NSEC_PER_MSEC);
	bmp182->timer.function = bmp182_timer_func;

	bmp182->pressure_wq = create_singlethread_workqueue("bmp182_wq");
	if (!bmp182->pressure_wq) {
		ret = -ENOMEM;
		pr_err("%s: could not create pressure workqueue\n", __func__);
		goto err_create_pressure_workqueue;
	}

	INIT_WORK(&bmp182->work_pressure, bmp182_work_func_pressure);

	/* allocate pressure sensor input_device */
	input_dev = input_allocate_device();
	if (!input_dev) {
		pr_err("%s: could not allocate input device\n", __func__);
		ret = -ENOMEM;
		goto err_input_allocate_device_pressure;
	}
	input_set_drvdata(input_dev, bmp182);
	input_dev->name = "pressure_sensor";
	input_set_capability(input_dev, EV_ABS, ABS_PRESSURE);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, 110000, 0, 0);
	input_set_capability(input_dev, EV_ABS, ABS_MISC);
	input_set_abs_params(input_dev, ABS_MISC, 0, 120, 0, 0);

	ret = input_register_device(input_dev);
	if (ret < 0) {
		pr_err("%s: could not register input device\n", __func__);
		input_free_device(input_dev);
		goto err_input_register_device_pressure;
	}
	bmp182->pressure_input_dev = input_dev;
	ret = sysfs_create_group(&input_dev->dev.kobj,
				 &pressure_attribute_group);
	if (ret) {
		pr_err("%s: could not create sysfs group\n", __func__);
		goto err_sysfs_create_group_pressure;
	}

	goto done;

/* error, unwind it all */
err_sysfs_create_group_pressure:
	input_unregister_device(bmp182->pressure_input_dev);
err_input_register_device_pressure:
err_input_allocate_device_pressure:
	destroy_workqueue(bmp182->pressure_wq);
err_create_pressure_workqueue:
err_setup:
	mutex_destroy(&bmp182->lock);
	kfree(bmp182);
done:
	return ret;
}

static int bmp182_suspend(struct device *dev)
{
	return 0;
}

static int bmp182_resume(struct device *dev)
{
	return 0;
}

static int bmp182_i2c_remove(struct i2c_client *client)
{
	struct bmp182_data *bmp182 = i2c_get_clientdata(client);
	input_unregister_device(bmp182->pressure_input_dev);
	destroy_workqueue(bmp182->pressure_wq);
	mutex_destroy(&bmp182->lock);
	kfree(bmp182);
	return 0;
}

static const struct i2c_device_id bmp182_device_id[] = {
	{"bmp182", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, bmp182_device_id);

static const struct dev_pm_ops bmp182_pm_ops = {
	.suspend = bmp182_suspend,
	.resume = bmp182_resume
};

static struct i2c_driver bmp182_i2c_driver = {
	.driver = {
		.name = "bmp182",
		.owner = THIS_MODULE,
		.pm = &bmp182_pm_ops
	},
	.probe		= bmp182_i2c_probe,
	.remove		= bmp182_i2c_remove,
	.id_table	= bmp182_device_id,
};


static int __init bmp182_init(void)
{
	return i2c_add_driver(&bmp182_i2c_driver);
}

static void __exit bmp182_exit(void)
{
	i2c_del_driver(&bmp182_i2c_driver);
}

module_init(bmp182_init);
module_exit(bmp182_exit);

MODULE_AUTHOR("tim.sk.lee@samsung.com");
MODULE_DESCRIPTION("Barometer sensor driver for bmp182");
MODULE_LICENSE("GPL");
