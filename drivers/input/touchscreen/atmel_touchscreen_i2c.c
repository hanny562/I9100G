#include <linux/module.h>
#include <linux/kernel_stat.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/random.h>
#include <linux/syscalls.h>
#include <linux/kthread.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <asm/irq.h>
#include <asm/mach/irq.h>
#include <plat/gpio.h>
#include <plat/mux.h>
#include <mach/hardware.h>
#include <linux/types.h>

#include "atmel_touch.h"

#define ATMEL_TSP_SENSOR_ADDRESS	0x4A
#define I2C_DF_NOTIFY			0x01
#define I2C_MAX_SEND_LENGTH		300
#define I2C_M_WR	0

extern u32 hw_revision;

static int i2c_tsp_sensor_attach_adapter(struct i2c_adapter *adapter);
static int i2c_tsp_sensor_probe_client(struct i2c_adapter *adapter, int address, int kind);
static int i2c_tsp_sensor_detach_client(struct i2c_client *client);

struct i2c_driver tsp_sensor_driver = {
	.driver	= {
		.name	= "tsp_driver",
		.owner	= THIS_MODULE,
	},
	.attach_adapter	= &i2c_tsp_sensor_attach_adapter,
	.remove	= &i2c_tsp_sensor_detach_client,
};

static struct i2c_client *g_client;

int i2c_tsp_sensor_read(u16 reg, u8 *read_val, unsigned int len)
{
	int ret;
	struct i2c_msg msg[2];
	unsigned char data[2];

#ifdef ENABLE_DEBUG
	if ( (g_client == NULL) || (!g_client->adapter) ) {
		perr("I2C client is not initialized\n");
		return -ENODEV;
	}
#endif

	data[0] = reg & 0x00ff;
	data[1] = reg >> 8;
	msg[0].addr  = g_client->addr;
	msg[0].flags = I2C_M_WR;
	msg[0].len   = 2;
	msg[0].buf   = data;

	msg[1].addr  = g_client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len   = len;
	msg[1].buf   = read_val;

	ret = i2c_transfer(g_client->adapter, msg, 2);
	if (ret == 2) {
		return 0;
	}
	perr("Sensor I2C read failed");
	return ret;
}


int i2c_tsp_sensor_write(u16 reg, u8 *read_val, unsigned int len)
{
	int ret;
	struct i2c_msg msg[1];
	unsigned char data[I2C_MAX_SEND_LENGTH];
	int i ;

	if ( (g_client == NULL) || (!g_client->adapter) ) {
		perr("I2C client is not initialized\n");
		return -ENODEV;
	}

	if (len + 2 > I2C_MAX_SEND_LENGTH) {
		perr("Data length error\n");
		return -ENODEV;
	}

	msg->addr  = g_client->addr;
	msg->flags = I2C_M_WR;
	msg->len   = len + 2;
	msg->buf   = data;
	data[0] = reg & 0x00ff;
	data[1] = reg >> 8;
	for (i = 0; i < len; i++) {
		data[i + 2] = *(read_val + i);
	}

	ret = i2c_transfer(g_client->adapter, msg, 1);
	if (ret == 1)
		return 0;

	perr("Error writing to sensor register\n");
	return ret;
}


void i2c_tsp_sensor_write_reg(u8 address, int data)
{
	u8 i2cdata[1];

	i2cdata[0] = data;
	i2c_tsp_sensor_write(address, i2cdata, 1);
}


static int i2c_tsp_sensor_attach_adapter(struct i2c_adapter *adapter)
{
	if (adapter->nr == 3)
		return i2c_tsp_sensor_probe_client(adapter, ATMEL_TSP_SENSOR_ADDRESS, 0);
	return 0;
}


static int i2c_tsp_sensor_probe_client(struct i2c_adapter *adapter, int address, int kind)
{
	if ( !i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA) ) {
		perr("byte op is not permited.\n");
		return -EIO;
	}

	g_client = kzalloc(sizeof(struct i2c_client), GFP_KERNEL);
	if ( !g_client ) {
		perr("Failed to allocate memory\n");
		return -ENOMEM;
	}
	g_client->addr    = address;
	g_client->adapter = adapter;
	g_client->driver  = &tsp_sensor_driver;
	strlcpy(g_client->name, "tsp_driver", I2C_NAME_SIZE);
	return 0;
}


static int i2c_tsp_sensor_detach_client(struct i2c_client *client)
{
	i2c_set_clientdata(client, NULL);
	kfree(client);
	g_client = NULL;
	return 0;
}


int i2c_tsp_sensor_init(void)
{
	int ret;

	if ( (ret = i2c_add_driver(&tsp_sensor_driver)) ) {
		perr("I2C Driver registration failed!\n");
		return ret;
	}
	return 0;
}


uint8_t read_boot_state(u8 *data)
{
	struct i2c_msg rmsg;
	int ret;

	rmsg.addr  = QT602240_I2C_BOOT_ADDR ;
	rmsg.flags = I2C_M_RD;
	rmsg.len   = 1;
	rmsg.buf   = data;

	ret = i2c_transfer(g_client->adapter, &rmsg, 1);
	if (ret != 1) {
		perr("Fail!!!! ret = %d\n", ret);
		return READ_MEM_FAILED;
	}
	return READ_MEM_OK;
}


uint8_t boot_unlock(void)
{
	struct i2c_msg wmsg;
	int ret;
	unsigned char data[2];

	data[0] = 0xDC;
	data[1] = 0xAA;

	wmsg.addr  = QT602240_I2C_BOOT_ADDR ;
	wmsg.flags = I2C_M_WR;
	wmsg.len   = 2;
	wmsg.buf   = data;

	ret = i2c_transfer(g_client->adapter, &wmsg, 1);
	if( ret != 1) {
		perr("I2C transfer failed\n");
		return WRITE_MEM_FAILED;
	}
	return WRITE_MEM_OK;
}


uint8_t boot_write_mem(uint16_t ByteCount, unsigned char * Data)
{
	struct i2c_msg wmsg;
	int ret;

	wmsg.addr  = QT602240_I2C_BOOT_ADDR ;
	wmsg.flags = I2C_M_WR;
	wmsg.len   = ByteCount;
	wmsg.buf   = Data;

	ret = i2c_transfer(g_client->adapter, &wmsg, 1);
	if ( ret == 1) {
		return WRITE_MEM_OK;
	} else {
		perr("Fail!!!!\n");
		return WRITE_MEM_FAILED;
	}
}

