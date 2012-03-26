#ifndef __ISL29023_H__
#define __ISL29023_H__

#include <linux/kernel.h>
#include <linux/types.h>

#define LIGHTSENSOR_IRQ 		IRQ_EINT2
//#define ISL29023_DEBUG

#define USE_ISL29023_IOCTLS

#define error(fmt,arg...) printk(fmt "\n",## arg)

#ifdef ISL29023_DEBUG
#define debug(fmt,arg...) printk("--------" fmt "\n",## arg)
#else
#define debug(fmt,arg...)
#endif

/* Light Sensor Power Control */
#define ON              1
#define OFF				0

#ifdef USE_ISL29023_IOCTLS
/*IOCTLS*/
/*magic no*/
#define ISL29023_LIGHT_IOC_MAGIC  		0xEE
/*max seq no*/
#define ISL29023_LIGHT_IOC_NR_MAX 		9

#define ISL29023_LIGHT_IOC_SET_OPERATION_MODE         		_IOW(ISL29023_LIGHT_IOC_MAGIC, 0,u8)
#define ISL29023_LIGHT_IOC_SET_LUX_RANGE         		_IOW(ISL29023_LIGHT_IOC_MAGIC, 1,u32)
#define ISL29023_LIGHT_IOC_SET_ADC_RESOLUTION       		_IOW(ISL29023_LIGHT_IOC_MAGIC, 2,u8)
#define ISL29023_LIGHT_IOC_SET_INTERRUPT_PERSIST      		_IOW(ISL29023_LIGHT_IOC_MAGIC, 3,u8)
#define ISL29023_LIGHT_IOC_GET_OPERATION_MODE         		_IOR(ISL29023_LIGHT_IOC_MAGIC, 4,u8)
#define ISL29023_LIGHT_IOC_GET_LUX_RANGE         		_IOR(ISL29023_LIGHT_IOC_MAGIC, 5,u32)
#define ISL29023_LIGHT_IOC_GET_ADC_RESOLUTION       		_IOR(ISL29023_LIGHT_IOC_MAGIC, 6,u8)
#define ISL29023_LIGHT_IOC_GET_INTERRUPT_PERSIST      		_IOR(ISL29023_LIGHT_IOC_MAGIC, 7,u8)
#define ISL29023_LIGHT_IOC_SET_POLL_DELAY	     		_IOR(ISL29023_LIGHT_IOC_MAGIC, 8, u32)

#endif

#endif
