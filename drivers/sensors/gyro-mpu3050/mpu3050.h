#ifndef __MPU3050_H__
#define __MPU3050_H__

#if 0
#define MPU3050_DEBUG	1
#endif

#include<linux/earlysuspend.h>

/*Debug Related Macros */
#define error(fmt,arg...) printk(fmt "\n",## arg)

#ifdef MPU3050_DEBUG
#define debug(fmt,arg...) printk("--------" fmt "\n",## arg)
#else
#define debug(fmt,arg...)
#endif

//#define GYRO_DEV_MAJOR 238
#define GYRO_DEV_NAME "gyroscope"

/* MPU3050 IOCTLs....*/
#define MPU3050_GYRO_IOC_MAGIC 				'M'

#define MPU3050_GYRO_IOC_READ_GYRO_XYZ		        _IOR(MPU3050_GYRO_IOC_MAGIC,0,u32)
#define MPU3050_GYRO_IOC_READ_ACCEL_XYZ			_IOR(MPU3050_GYRO_IOC_MAGIC,1,u32)
#define MPU3050_GYRO_IOC_READ_TEMP			_IOR(MPU3050_GYRO_IOC_MAGIC,2,u32)
#define MPU3050_GYRO_IOC_SET_POLL_DELAY			_IOW(MPU3050_GYRO_IOC_MAGIC,3,u32)

#define MPU3050_GYRO_IOC_GET_OFFSETS			_IOR(MPU3050_GYRO_IOC_MAGIC,4,u32)
#define MPU3050_GYRO_IOC_SET_OFFSETS			_IOW(MPU3050_GYRO_IOC_MAGIC,5,u32)

#define MPU3050_GYRO_IOC_GET_OPERATION_MODE		_IOR(MPU3050_GYRO_IOC_MAGIC,6,u8)
#define MPU3050_GYRO_IOC_SET_OPERATION_MODE		_IOW(MPU3050_GYRO_IOC_MAGIC,7,u8)

#define MPU3050_GYRO_IOC_GET_INTERRUPT_CONFIG		_IOR(MPU3050_GYRO_IOC_MAGIC,8,u8)
#define MPU3050_GYRO_IOC_SET_INTERRUPT_CONFIG		_IOW(MPU3050_GYRO_IOC_MAGIC,9,u8)


#define MPU3050_GYRO_IOC_MAXNR				10


#if 0
//Different Modes in which the driver could work.
#define GYRO_POWER_DOWN 0
#define GYRO_MASTER_ACCEL 1
#define GYRO_ONLY 2
#define GYRO_MASTER_ACCEL_FIFO 3
#define GYRO_ONLY_FIFO	4
#endif

// Mode numbers for different IOCTLs
#define GYRO_POWER_OFF	0
#define GYRO_POWER_ON	1

// MPU3050_USER_CTRL Register values for different modes
#define GYRO_MODE_STANDBY	0x78
#define GYRO_MODE_ALL_ON	0x00

struct mpu3050_dev_state
{
u8 mode;
u16 gyro_xyz[3];
u16 accel_xyz[3];
u16 temperature;
u16 offsets[3];
};

struct mpu3050_dev
{
struct i2c_client *client;
struct input_dev *inputdev;
struct mutex lock;
struct mpu3050_dev_state state;
struct early_suspend mpu3050_early_suspend;
};

// MPU-3050 details
#define MPU3050_WHO_AM_I	0x00
#define MPU3050_PRODUCT_ID	0X01

// Register for Offsets
#define MPU3050_XOFFS_USER_H 	0X0C
#define MPU3050_XOFFS_USER_L 	0X0D
#define MPU3050_YOFFS_USER_H 	0X0E
#define MPU3050_YOFFS_USER_L 	0X0F
#define MPU3050_ZOFFS_USER_H 	0X10
#define MPU3050_ZOFFS_USER_L 	0X11

// Registers for miscellanous configurations.
#define MPU3050_FIFO_EN		0X12
#define MPU3050_AUX_VDDIO	0X13
#define MPU3050_AUX_SLV_ADDR	0X14
#define MPU3050_SMPALRT_DIV	0X15
#define MPU3050_DLPF_FS_SYNC	0X16
#define MPU3050_INT_CFG		0X17
#define MPU3050_AUX_ADDR	0X18
#define MPU3050_INT_STATUS	0X1A

// Registers for Temperature; X,Y and Z gyro values
#define MPU3050_TEMP_OUT_H 	0X1B
#define MPU3050_TEMP_OUT_L 	0X1C
#define MPU3050_GYRO_XOUT_H	0X1D
#define MPU3050_GYRO_XOUT_L	0X1E
#define MPU3050_GYRO_YOUT_H	0X1F
#define MPU3050_GYRO_YOUT_L	0X20
#define MPU3050_GYRO_ZOUT_H	0X21
#define MPU3050_GYRO_ZOUT_L 	0X22
#define MPU3050_AUX_XOUT_H	0X23
#define MPU3050_AUX_XOUT_L	0X24
#define MPU3050_AUX_YOUT_H	0X25
#define MPU3050_AUX_YOUT_L	0X26
#define MPU3050_AUX_ZOUT_H	0X27
#define MPU3050_AUX_ZOUT_L	0X28

// Register for count of bytes occupied in FIFO Buffer(max =512)
#define MPU3050_FIFO_COUNTH	0X3A
#define MPU3050_FIFO_COUNTL	0X3B

// Register for FIFO-Buffer Read.
#define MPU3050_FIFO_R		0X3C

// Register for User Options
#define MPU3050_USER_CTRL	0X3D

// Register for Power-Management options.
#define MPU3050_PWR_MGMNT	0X3E


// Sampling Rate.Default value = 8kHz
#define DEFAULT_SMPLRTDIVIDER	0x00 		// Using Formula : 8kHz/(1)= 8kHz
#define DFLT_SMPL_CFG		0x00 		// 256Hz low pass sample filter

// All 4 modes assign DLPF_CFG so as to set 8kHz sampling-rate & LPF of 256Hz
// EXT_SYNC_SET ignored for time-being 
#define GYRO_RANGE_250 		0x00
#define GYRO_RANGE_500 		0x08
#define GYRO_RANGE_1000		0x10	
#define GYRO_RANGE_2000		0x18

// Operations for User control; For register MPU3050_USER_CTRL
#define ENABLE_FIFO_READ	(1 << 6)
#define ENABLE_ACCEL_READ	(1 << 5)
#define RESET_GYRO_FUNC		(1 << 0)
#define RESET_FIFO_FUNC		(1 << 1)
#define RESET_ACCEL_FUNC	(1 << 3)

// Power_control Operations; For Register: MPU3050_PWR_MGM
#define SLEEP			(1 << 6)
#define	H_RESET			(1 << 7)
#define STDBY_GYROX		(1 << 5)
#define STDBY_GYROY		(1 << 4)
#define STDBY_GYROZ		(1 << 3)

#define CLOCK_INTERNAL_OSCILLATOR 	0
#define CLOCK_EXT_32KHZ			(1 << 0)
#define CLOCK_EXT_19KHZ			(1 << 1)
#define CLOCK_RESET			((1 << 2)|(1 << 1)|(1 << 0))



// Interrupt-Related Operations..
// when will be the interrupts enabled.
// Need to initialize interrupt configuration.
#define INTERRUPT_WHEN_DATA 		0
#define INTERRUPT_WHEN_DMP		1
#define INTERRUPT_WHEN_MPU_READY 	2


// If FIFO_READ is enabled;we need to specify what values will go
// in Buffer. Register : MPU3050_FIFO_EN
#define FIFO_TEMP_ENABLE	(1<<7)
#define FIFO_GYROX_ENABLE	(1<<6)
#define FIFO_GYROY_ENABLE	(1<<5)
#define FIFO_GYROZ_ENABLE	(1<<4)
#define FIFO_ACCELX_ENABLE	(1<<3)
#define FIFO_ACCELY_ENABLE	(1<<2)
#define FIFO_ACCELZ_ENABLE	(1<<1)
#define FIFO_FOOTER_ENABLE	(1<<0)

#define DEFAULT_FIFO (FIFO_GYROX_ENABLE)|FIFO_GYROY_ENABLE|FIFO_GYROZ_ENABLE|FIFO_ACCELX_ENABLE \
						|FIFO_ACCELY_ENABLE|FIFO_ACCELZ_ENABLE|FIFO_FOOTER_ENABLE



#endif
