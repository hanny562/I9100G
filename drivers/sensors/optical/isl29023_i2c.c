#include <linux/kernel.h>
#include <linux/i2c.h>

#include "isl29023.h"
#include "isl29023_func.h"

#define LIGHTSENSOR_I2C_ADDR	0x88

int isl29023_i2c_read(struct i2c_client *, u8 , u8 *);
int isl29023_i2c_write(struct i2c_client *, u8 , u8);

static int isl29023_probe(struct i2c_client *,const struct i2c_device_id *);
static int __devexit isl29023_remove(struct i2c_client *);
static int isl29023_suspend(struct i2c_client *, pm_message_t);
static int isl29023_resume(struct i2c_client *);

static struct i2c_client* isl29023_i2c_client;
static struct i2c_driver isl29023_i2c_driver;

static const struct i2c_device_id isl29023_ids[] = {
	{ "isl29023", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, isl29023_ids);

static struct i2c_driver isl29023_i2c_driver =
{
	.driver	= {
		.name	= "isl29023_driver",
		.owner	= THIS_MODULE,
	},
	.probe		= isl29023_probe,
	.remove		= __devexit_p(isl29023_remove),
	.suspend	= &isl29023_suspend,
	.resume		= &isl29023_resume,
	.id_table	= isl29023_ids,
};


static int isl29023_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	int ret = 0;
	printk("----- %s %d\n", __func__, __LINE__);

	isl29023_i2c_client = client;
	/*Device Initialisation*/
	ret = isl29023_dev_init(isl29023_i2c_client);
	return ret;
}

static int __devexit isl29023_remove(struct i2c_client *client)
{
	isl29023_set_operation_mode(POWER_DOWN);
	isl29023_i2c_client = NULL;
	isl29023_dev_exit();
	return 0;
}

#ifdef CONFIG_PM
static int isl29023_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret = 0;

	debug("%s called", __func__);
	if ((ret = isl29023_set_operation_mode(POWER_DOWN)) < 0)
		ret = -1;
	return ret;
}

static int isl29023_resume(struct i2c_client *client)
{
	int ret = 0;

	debug("%s called", __func__);
	if ((ret = isl29023_set_operation_mode(ALS_CONTINUOUS)) < 0)
		ret = -1;

	return ret;
}
#else
#define isl29023_suspend	NULL
#define	isl29023_resume		NULL
#endif

int isl29023_i2c_init(void)
{
	int ret = 0;
	debug("%s called", __func__);
	if ( (ret = i2c_add_driver(&isl29023_i2c_driver) < 0) ) {
		error("isl29023_i2c_init failed");
	}

	return ret;
}

void isl29023_i2c_exit(void)
{
	debug("%s called", __func__);
	i2c_del_driver(&isl29023_i2c_driver);
}

int isl29023_i2c_read(struct i2c_client *client, u8 reg, u8 *value)
{
	struct i2c_msg msg[2];
	int count =0;
	int ret = 0;

	msg[0].addr	= client->addr;
	msg[0].flags	= 0;
	msg[0].len	= 1;
	msg[0].buf	= &reg;
	count = i2c_transfer(client->adapter,msg,1);

	if(count==1) {
		msg[0].addr	= client->addr;
		msg[0].flags	= I2C_M_RD;
		msg[0].len	= 1;
		msg[0].buf	= value;
		count = i2c_transfer(client->adapter, msg, 1);
		if(count !=1)
			ret =-1;
	} else
		ret =-1;

	return ret;
}

int isl29023_i2c_write(struct i2c_client *client, u8 reg, u8 value)
{
	struct i2c_msg msg[1];
	u8 data[2];
	int count = 0;
	int ret = 0;

	data[0] = reg;
	data[1] = value;

	msg[0].addr	= client->addr;
	msg[0].flags	= 0;
	msg[0].len	= 2;
	msg[0].buf	= data;

	count = i2c_transfer(client->adapter, msg, 1);
	if(count !=1)
		ret =-1;

	return ret;
}

