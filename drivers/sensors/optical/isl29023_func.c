#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/delay.h>

#include <plat/irqs.h>
#include <mach/gpio.h>
#include "isl29023.h"
#include "isl29023_i2c.h"
#include "isl29023_func.h"


int isl29023_set_operation_mode(u8);
int isl29023_set_lux_range(u32);
int isl29023_set_adc_resolution(u8);
int isl29023_set_interrupt_persist(u8);
int isl29023_clear_irq_status(void);
int isl29023_get_operation_mode(u8*);
int isl29023_get_lux_range(u32*);
int isl29023_get_adc_resolution(u8*);
int isl29023_get_interrupt_persist(u8*);
int isl29023_get_lux_value(u32*);

static isl29023_dev  dev;


int isl29023_dev_init(struct i2c_client *client)
{
	int reg;
	int ret = 0;

	debug("%s called", __func__);

	dev.client = client;

	/* Resetting the registers array */
	for(reg = 0; reg < 8; reg++)
		dev.registers[reg] = 0;

	/* Device intialisations */
	if((ret = isl29023_set_operation_mode(ALS_CONTINUOUS)) < 0) {
		debug("isl29023_set_operation_mode failed");
		goto EXIT;
	}

	if((ret = isl29023_set_lux_range(LUX_RANGE_4)) < 0) {
		debug("isl29023_set_lux_range failed");
		goto EXIT;
	}

	if((ret = isl29023_set_adc_resolution(ADC_RESOLUTION_16))<0) {
		debug("isl29023_set_adc_resolution failed");
		goto EXIT;
	}

	if((ret = isl29023_set_interrupt_persist(INTERRUPT_PERSIST_4)) < 0) {
		debug("isl29023_set_interrupt_persist failed");
		goto EXIT;
	}

EXIT:
	return ret;
}

void isl29023_dev_exit(void)
{
	dev.client = NULL;
}

void isl29023_dev_mutex_init(void)
{
	mutex_init(&(dev.mutex_lock));
}

int isl29023_set_operation_mode(u8 mode)
{
	int ret = 0;

	debug("%s called",__func__);
	mutex_lock(&(dev.mutex_lock));

	switch(mode) {
	case POWER_DOWN:
		debug("POWER_DOWN");
		COMMAND1_SET_POWER_DOWN(&(dev.registers[ISL_REG_COMMAND1]));
		dev.state.op_mode=POWER_DOWN;
		break;

	case ALS_ONCE:
		debug("ALS_ONCE");
		COMMAND1_SET_ALS_ONCE(&(dev.registers[ISL_REG_COMMAND1]));
		dev.state.op_mode=ALS_ONCE;
		break;

	case IR_ONCE:
		debug("IR_ONCE");
		COMMAND1_SET_IR_ONCE(&(dev.registers[ISL_REG_COMMAND1]));
		dev.state.op_mode=IR_ONCE;
		break;

	case ALS_CONTINUOUS:
		debug("ALS_CONTINUOUS");
		COMMAND1_SET_ALS_CONTINUOUS(&(dev.registers[ISL_REG_COMMAND1]));
		dev.state.op_mode=ALS_CONTINUOUS;
		break;

	case IR_CONTINUOUS:
		debug("IR_CONTINUOUS");
		COMMAND1_SET_IR_CONTINUOUS(&(dev.registers[ISL_REG_COMMAND1]));
		dev.state.op_mode=IR_CONTINUOUS;
		break;

	default:
		debug("Wrong Operation mode called");
		ret = -EINVAL;
		goto EXIT;
	}

	if ((ret = isl29023_i2c_write(dev.client, ISL_REG_COMMAND1, dev.registers[ISL_REG_COMMAND1])) < 0)
		debug("isl29023_i2c_write failed %s",__func__);

EXIT:
	mutex_unlock(&(dev.mutex_lock));
	return ret;
}

int isl29023_set_lux_range(u32 range)
{
	int ret = 0;

	debug("%s called", __func__);
	mutex_lock(&(dev.mutex_lock));

	switch(range) {
	case LUX_RANGE_1:
		debug("LUX_RANGE_1");
		COMMAND2_SET_LUX_RANGE_1(&(dev.registers[ISL_REG_COMMAND2]));
		dev.state.lux_range=LUX_RANGE_1;
		break;

	case LUX_RANGE_2:
		debug("LUX_RANGE_2");
		COMMAND2_SET_LUX_RANGE_2(&(dev.registers[ISL_REG_COMMAND2]));
		dev.state.lux_range=LUX_RANGE_2;
		break;

	case LUX_RANGE_3:
		debug("LUX_RANGE_3");
		COMMAND2_SET_LUX_RANGE_3(&(dev.registers[ISL_REG_COMMAND2]));
		dev.state.lux_range=LUX_RANGE_3;
		break;

	case LUX_RANGE_4:
		debug("LUX_RANGE_4");
		COMMAND2_SET_LUX_RANGE_4(&(dev.registers[ISL_REG_COMMAND2]));
		dev.state.lux_range=LUX_RANGE_4;
		break;

	default:
		debug("Wrong Lux Range value");
		ret = -EINVAL;
		goto EXIT;
	}

	if ((ret = isl29023_i2c_write(dev.client, ISL_REG_COMMAND2, dev.registers[ISL_REG_COMMAND2])) < 0)
		debug("isl29023_i2c_write failed %s",__func__);

EXIT:
	mutex_unlock(&(dev.mutex_lock));
	return ret;
}

int isl29023_set_adc_resolution(u8 resol)
{
	int ret = 0;

	debug("%s called",__func__);
	mutex_lock(&(dev.mutex_lock));

	switch(resol) {
	case ADC_RESOLUTION_4:
		debug("ADC_RESOLUTION_4");
		COMMAND2_SET_ADC_RESOLUTION_4(&(dev.registers[ISL_REG_COMMAND2]));
		dev.state.adc_resol=ADC_RESOLUTION_4;
		break;

	case ADC_RESOLUTION_8:
		debug("ADC_RESOLUTION_8");
		COMMAND2_SET_ADC_RESOLUTION_8(&(dev.registers[ISL_REG_COMMAND2]));
		dev.state.adc_resol=ADC_RESOLUTION_8;
		break;

	case ADC_RESOLUTION_12:
		debug("ADC_RESOLUTION_12");
		COMMAND2_SET_ADC_RESOLUTION_12(&(dev.registers[ISL_REG_COMMAND2]));
		dev.state.adc_resol=ADC_RESOLUTION_12;
		break;

	case ADC_RESOLUTION_16:
		debug("ADC_RESOLUTION_16");
		COMMAND2_SET_ADC_RESOLUTION_16(&(dev.registers[ISL_REG_COMMAND2]));
		dev.state.adc_resol=ADC_RESOLUTION_16;
		break;

	default:
		debug("Wrong ADC Resolution value");
		ret= -EINVAL;
		goto EXIT;
	}

	if ((ret = isl29023_i2c_write(dev.client, ISL_REG_COMMAND2, dev.registers[ISL_REG_COMMAND2])) < 0)
		debug("isl29023_i2c_write failed %s",__func__);

EXIT:
	mutex_unlock(&(dev.mutex_lock));
	return ret;
}

int isl29023_set_interrupt_persist(u8 persist)
{
	int ret = 0;

	debug("%s called",__func__);
	mutex_lock(&(dev.mutex_lock));

	switch(persist) {
	case INTERRUPT_PERSIST_1:
		debug("INTERRUPT_PERSIST_1");
		COMMAND1_SET_INTERRUPT_PERSIST_1(&(dev.registers[ISL_REG_COMMAND1]));
		dev.state.interrupt_persist=INTERRUPT_PERSIST_1;
		break;

	case INTERRUPT_PERSIST_4:
		debug("INTERRUPT_PERSIST_4");
		COMMAND1_SET_INTERRUPT_PERSIST_4(&(dev.registers[ISL_REG_COMMAND1]));
		dev.state.interrupt_persist=INTERRUPT_PERSIST_4;
		break;

	case INTERRUPT_PERSIST_8:
		debug("INTERRUPT_PERSIST_8");
		COMMAND1_SET_INTERRUPT_PERSIST_8(&(dev.registers[ISL_REG_COMMAND1]));
		dev.state.interrupt_persist=INTERRUPT_PERSIST_8;
		break;

	case INTERRUPT_PERSIST_16:
		debug("INTERRUPT_PERSIST_16");
		COMMAND1_SET_INTERRUPT_PERSIST_16(&(dev.registers[ISL_REG_COMMAND1]));
		dev.state.interrupt_persist=INTERRUPT_PERSIST_16;
		break;

	default:
		debug("Wrong Interrupt Persist value");
		ret = -EINVAL;
		goto EXIT;
	}

	if ((ret = isl29023_i2c_write(dev.client, ISL_REG_COMMAND1, dev.registers[ISL_REG_COMMAND1])) < 0)
		debug("isl29023_i2c_write failed %s",__func__);

EXIT:
	mutex_unlock(&(dev.mutex_lock));
	return ret;
}

int isl29023_clear_irq_status(void)
{
	int ret = 0;

	debug("%s called",__func__);
	mutex_lock(&(dev.mutex_lock));

	COMMAND1_CLEAR_INTERRUPT_STATUS(&dev.registers[ISL_REG_COMMAND1]);
	if ((ret = isl29023_i2c_write(dev.client, ISL_REG_COMMAND1, (dev.registers[ISL_REG_COMMAND1]))) < 0)
		debug("isl29023_i2c_write failed %s",__func__);

	mutex_unlock(&(dev.mutex_lock));
	return ret;
}

int isl29023_get_lux_value(u32* lux_value)
{
	int ret = 0;
	u8 adc_lsb;
	u8 adc_msb;
	u16 adc_data;

	debug("%s called",__func__);
	mutex_lock(&(dev.mutex_lock));

	msleep(100); /*Time for ADC Conversion*/
	if ((ret = isl29023_i2c_read(dev.client, ISL_REG_DATA_LSB, &adc_lsb)) < 0)
		debug("isl29023_i2c_read 1 failed %s",__func__);
	if ((ret = isl29023_i2c_read(dev.client, ISL_REG_DATA_MSB, &adc_msb)) < 0)
		debug("isl29023_i2c_read 2 failed %s",__func__);

	debug("msb=%x,lsb=%x", adc_msb, adc_lsb);
	adc_data = adc_msb<<8|adc_lsb;
	debug("adc_data=%x,adc_resol=%d,lux_range=%d", adc_msb<<8|adc_lsb,
			dev.state.adc_resol, dev.state.lux_range);
	*lux_value = (adc_data*dev.state.lux_range)>>dev.state.adc_resol;

	mutex_unlock(&(dev.mutex_lock));
	return ret;
}


int isl29023_get_operation_mode(u8* mode)
{
	int ret = 0;
	u8 temp;

	debug("%s called",__func__);
	mutex_lock(&(dev.mutex_lock));

	if ((ret = isl29023_i2c_read(dev.client, ISL_REG_COMMAND1, &temp)) < 0)
		debug("isl29023_i2c_read  failed %s",__func__);

	temp = (REG0_OPM_MASK&temp)>>REG0_OPM_SHIFT;
	//temp = (REG0_OPM_MASK&dev.registers[ISL_REG_COMMAND1])>>REG0_OPM_SHIFT;
	switch(temp) {
	case 0:
		debug("Device is in POWER_DOWN mode");
		*mode = POWER_DOWN;
		break;

	case 1:
		debug("Device is in ALS_ONCE mode");
		*mode = ALS_ONCE;
		break;

	case 2:
		debug("Device is in IR_ONCE mode");
		*mode = IR_ONCE;
		break;

	case 5:
		debug("Device is in ALS_CONTINUOUS mode");
		*mode = ALS_CONTINUOUS;
		break;

	case 6:
		debug("Device is in IR_CONTINUOUS mode");
		*mode = IR_CONTINUOUS;
		break;

	default:
		debug("Unknown Operation Mode");
		*mode = 0xFF;
		break;
	}

	debug("mode=%d",*mode);
	mutex_unlock(&(dev.mutex_lock));
	return ret;
}

int isl29023_get_lux_range(u32* range)
{
	int ret = 0;
	u8 temp;

	debug("%s called", __func__);
	mutex_lock(&(dev.mutex_lock));

	if ((ret = isl29023_i2c_read(dev.client, ISL_REG_COMMAND2, &temp)) < 0)
		debug("isl29023_i2c_read  failed %s",__func__);

	temp = (REG1_RANGE_MASK&temp) >> REG1_RANGE_SHIFT;
	//temp = (REG1_RANGE_MASK&dev.registers[ISL_REG_COMMAND2])>>REG1_RANGE_SHIFT;
	switch(temp) {
	case 0 :
		debug("Lux Range is 1000");
		*range = LUX_RANGE_1;
		break;

	case 1:
		debug("Lux Range is 4000");
		*range = LUX_RANGE_2;
		break;

	case 2:
		debug("Lux Range is 16000");
		*range = LUX_RANGE_3;
		break;

	case 3:
		debug("Lux Range is 64000");
		*range = LUX_RANGE_4;
		break;

	default:
		debug("Unknown Lux Range");
		*range = 0xFFFF;
		break;
	}

	debug("range=%d", *range);
	mutex_unlock(&(dev.mutex_lock));
	return ret;
}

int isl29023_get_adc_resolution(u8* resol)
{
	int ret = 0;
	u8 temp;

	debug("%s called", __func__);
	mutex_lock(&(dev.mutex_lock));

	if ((ret = isl29023_i2c_read(dev.client, ISL_REG_COMMAND2, &temp)) < 0)
		debug("isl29023_i2c_read  failed %s", __func__);

	temp = (REG1_ADC_MASK&temp) >> REG1_ADC_SHIFT;
	//temp = (REG1_ADC_MASK&dev.registers[ISL_REG_COMMAND2])>>REG1_ADC_SHIFT;
	switch(temp) {
	case 0 :
		debug("ADC Resolution is 16");
		*resol = ADC_RESOLUTION_16;
		break;

	case 1:
		debug("ADC Resolution is 12");
		*resol = ADC_RESOLUTION_12;
		break;

	case 2:
		debug("ADC Resolution is 8");
		*resol = ADC_RESOLUTION_8;
		break;

	case 3:
		debug("ADC Resolution is 4");
		*resol = ADC_RESOLUTION_4;
		break;

	default:
		debug("Unknown ADC Resolution");
		*resol = 0xFF;
		break;
	}

	debug("resol=%d", *resol);
	mutex_unlock(&(dev.mutex_lock));
	return ret;
}

int isl29023_get_interrupt_persist(u8* persist)
{
	int ret = 0;
	u8 temp;

	debug("%s called", __func__);
	mutex_lock(&(dev.mutex_lock));
	if ((ret = isl29023_i2c_read(dev.client, ISL_REG_COMMAND1, &temp)) < 0)
		debug("isl29023_i2c_read  failed %s",__func__);

	temp = (REG0_INT_PERSIST_MASK&temp)>>REG0_INT_PERSIST_SHIFT;
	//temp = (REG0_INT_PERSIST_MASK&dev.registers[ISL_REG_COMMAND1])>>REG0_INT_PERSIST_SHIFT;
	switch(temp) {
	case 0 :
		debug("Interrupt Persist is 1 integration cycle");
		*persist = INTERRUPT_PERSIST_1;
		break;

	case 1:
		debug("Interrupt Persist is 4 integration cycle");
		*persist = INTERRUPT_PERSIST_4;
		break;

	case 2:
		debug("Interrupt Persist is 8 integration cycle");
		*persist = INTERRUPT_PERSIST_8;
		break;

	case 3:
		debug("Interrupt Persist is 16 integration cycle");
		*persist = INTERRUPT_PERSIST_16;
		break;

	default:
		debug("Unknown Interrupt Persist value");
		*persist = 0xFF;
		break;
	}

	debug("persist=%d", *persist);
	mutex_unlock(&(dev.mutex_lock));
	return ret;
}

