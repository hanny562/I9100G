#ifndef __KXSD9_H__
#define __KXSD9_H__

#include <linux/earlysuspend.h>

/* They used to connect i2c_acc_kxsd9_write. */
#define KXSD9_WR_FUNC_PTR int (* kxsd9_i2c_txdata)(char *, int  )

#define KXSD9_BUS_WRITE_FUNC(dev_addr, reg_addr, reg_data, len)\
		kxsd9_i2c_txdata(reg_data, len)

/* They used to connect i2c_acc_kxsd9_read. */
#define KXSD9_RD_FUNC_PTR int (* kxsd9_i2c_rxdata)( char *, int  )

#define KXSD9_BUS_READ_FUNC( dev_addr, reg_addr, reg_data, r_len )\
		kxsd9_i2c_rxdata(reg_data, r_len)


/* KXSD9 IOCTL */
#define KXSD9_IOC_MAGIC			'K'
#define KXSD9_SET_RANGE			_IOWR(KXSD9_IOC_MAGIC,4, unsigned char)
#define KXSD9_SET_MODE			_IOWR(KXSD9_IOC_MAGIC,6, unsigned char)
#define KXSD9_SET_BANDWIDTH		_IOWR(KXSD9_IOC_MAGIC,8, unsigned char)
#define KXSD9_READ_ACCEL_XYZ           _IOWR(KXSD9_IOC_MAGIC,46,short)
#define KXSD9_IOC_MAXNR            	48

//#define DEBUG				0

/* KXSD9 I2C Address */
#define KXSD9_I2C_ADDR			0x38

/* KXSD9 API error codes */
#define E_KXSD9_NULL_PTR	(char)-127
#define E_COMM_RES		(char)-1
#define E_OUT_OF_RANGE		(char)-2


/* register write and read delays */
#define MDELAY_DATA_TYPE	unsigned int
#define KXSD9_EE_W_DELAY	28	/* delay after EEP write is 28 msec */


struct kxsd9 {
	struct i2c_client *client;
	struct input_dev *inputdev;
	struct hrtimer timer;
	struct delayed_work work;
	struct mutex lock;
#ifdef CONFIG_ANDROID_POWER
	android_suspend_lock_t suspend_lock;
#endif
	int on;
	struct kxsd9_seq *seq;
	int iseq,nseq;

	int pedo_up,pedo_lim;
	unsigned short pedo_count;
	struct early_suspend kxsd9_early_suspend;
};

typedef struct  {
	short x,//< holds x-axis acceleration data sign extended. Range -512 to 511.
	y, 	//< holds y-axis acceleration data sign extended. Range -512 to 511.
	z; 	//< holds z-axis acceleration data sign extended. Range -512 to 511.
} kxsd9acc_t;


typedef struct  {
	unsigned char xout_h;
	unsigned char xout_l;
	unsigned char yout_h;
	unsigned char yout_l;
	unsigned char zout_h;
	unsigned char zout_l;
	unsigned char auxout_h;
	unsigned char auxout_l;
	unsigned char rsvd1;
	unsigned char rsvd2;
	unsigned char reset_write;
	unsigned char rsvd3;
	unsigned char ctrl_regc;
	unsigned char ctrl_regb;
	unsigned char ctrl_rega;
} kxsd9regs_t;



typedef struct {
	kxsd9regs_t * image;			/**< pointer to kxsd9regs_t structure not mandatory */
	unsigned char mode;			/**< save current KXSD9 operation mode */
	unsigned char chip_id,			/**< save KXSD9's chip id which has to be 0x02 after calling bma020_init() */
	ml_version, 				/**< holds the KXSD9 ML_version number */
	al_version;				/**< holds the KXSD9 AL_version number */
	unsigned char dev_addr;			/**< initializes KXSD9's I2C device address 0x38 */
	unsigned char int_mask;			/**< stores the current KXSD9 API generated interrupt mask */
	KXSD9_WR_FUNC_PTR;		  	/**< function pointer to the SPI/I2C write function */
	KXSD9_RD_FUNC_PTR;		  	/**< function pointer to the SPI/I2C read function */
	void (*delay_msec)( MDELAY_DATA_TYPE ); /**< function pointer to a pause in mili seconds function */
	//struct early_suspend early_suspend;	/**< suspend function */
} kxsd9_t;


/* bit slice positions in registers */

/* cond BITSLICE */
#define CHIP_ID__POS		0
#define CHIP_ID__MSK		0x07
#define CHIP_ID__LEN		3
#define CHIP_ID__REG		CHIP_ID_REG

#define ML_VERSION__POS		0
#define ML_VERSION__LEN		4
#define ML_VERSION__MSK		0x0F
#define ML_VERSION__REG		VERSION_REG

#define AL_VERSION__POS		4
#define AL_VERSION__LEN		4
#define AL_VERSION__MSK		0xF0
#define AL_VERSION__REG		VERSION_REG


/* BANDWIDTH dependend definitions */
#define BANDWIDTH__POS		0
#define BANDWIDTH__LEN		3
#define BANDWIDTH__MSK		0x07
#define BANDWIDTH__REG		RANGE_BWIDTH_REG


/* RANGE */
#define RANGE__POS		3
#define RANGE__LEN		2
#define RANGE__MSK		0x18
#define RANGE__REG		RANGE_BWIDTH_REG


/* WAKE UP */
#define WAKE_UP__POS		0
#define WAKE_UP__LEN		1
#define WAKE_UP__MSK		0x01
#define WAKE_UP__REG		KXSD9_CONF2_REG


#define WAKE_UP_PAUSE__POS	1
#define WAKE_UP_PAUSE__LEN	2
#define WAKE_UP_PAUSE__MSK	0x06
#define WAKE_UP_PAUSE__REG	KXSD9_CONF2_REG


#define KXSD9_GET_BITSLICE(regvar, bitname)\
			(regvar & bitname##__MSK) >> bitname##__POS


#define KXSD9_SET_BITSLICE(regvar, bitname, val)\
		  (regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK)


/** endcond */


/* CONSTANTS */


/* range and bandwidth */
//cvs
#define KXSD9_RANGE_2G			3 /**< sets range to 2G mode \see bma020_set_range() */
#define KXSD9_RANGE_4G			2 /**< sets range to 4G mode \see bma020_set_range() */
#define KXSD9_RANGE_8G			0 /**< sets range to 8G mode \see bma020_set_range() */


#define KXSD9_BW_25HZ		0	/**< sets bandwidth to 25HZ \see bma020_set_bandwidth() */
#define KXSD9_BW_50HZ		1	/**< sets bandwidth to 50HZ \see bma020_set_bandwidth() */
#define KXSD9_BW_100HZ		2	/**< sets bandwidth to 100HZ \see bma020_set_bandwidth() */
#define KXSD9_BW_190HZ		3	/**< sets bandwidth to 190HZ \see bma020_set_bandwidth() */
#define KXSD9_BW_375HZ		4	/**< sets bandwidth to 375HZ \see bma020_set_bandwidth() */
#define KXSD9_BW_750HZ		5	/**< sets bandwidth to 750HZ \see bma020_set_bandwidth() */
#define KXSD9_BW_1500HZ	6	/**< sets bandwidth to 1500HZ \see bma020_set_bandwidth() */

/* mode settings */

#define KXSD9_MODE_NORMAL      0
#define KXSD9_MODE_SLEEP       2
#define KXSD9_MODE_WAKE_UP     3

/* wake up */

#define KXSD9_WAKE_UP_PAUSE_20MS		0
#define KXSD9_WAKE_UP_PAUSE_80MS		1
#define KXSD9_WAKE_UP_PAUSE_320MS		2
#define KXSD9_WAKE_UP_PAUSE_2560MS		3


/* KXSD9 Defines */
#define KXSD9_XOUT_H		0x00
#define KXSD9_XOUT_L		0x01
#define KXSD9_YOUT_H		0x02
#define KXSD9_YOUT_L		0x03
#define KXSD9_ZOUT_H		0x04
#define KXSD9_ZOUT_L		0x05
#define KXSD9_AUXOUT_H		0x06
#define KXSD9_AUXOUT_L		0x07
#define KXSD9_RESET_WRITE	0x0A
#define KXSD9_CTRL_REGC		0x0C
#define KXSD9_CTRL_REGB		0x0D
#define KXSD9_CTRL_REGA		0x0E

#define KXSD9_RESET_KEY		0xCA

#define KXSD9_REGC_LP2		(1 << 7)
#define KXSD9_REGC_LP1		(1 << 6)
#define KXSD9_REGC_LP0		(1 << 5)
#define KXSD9_REGC_MOTLEV	(1 << 4)
#define KXSD9_REGC_MOTLAT	(1 << 3)
#define KXSD9_REGC_FS1		(1 << 1)
#define KXSD9_REGC_FS0		(1 << 0)

/*
    * 'CLKhld=1' makes KXSD9 hold SCL low during A/D conversions. This may
     * affect the operation of other I2C devices. Thus, I don't think it is
      * good to set CLKhld to 1.
       */
#define KXSD9_REGB_CLKHLD	(1 << 7)
#define KXSD9_REGB_ENABLE	(1 << 6)
#define KXSD9_REGB_MOTIEN	(1 << 2)

#define KXSD9_REGA_MOTI		(1 << 1)

/*
    * Default Full Scale Range: +/-2 g
     * 12-bit Sensitivity : 819 counts/g
      */
#define KXSD9_RANGE_DEFAULT	(KXSD9_REGC_FS1 | KXSD9_REGC_FS0)

/* Default Motion Wake Up Acceleration Threshold: +/-1 g */
#define KXSD9_THRESHOLD_DEFAULT	(KXSD9_REGC_MOTLEV | KXSD9_RANGE_DEFAULT)

/* Default Operational Bandwidth (Filter Corner Frequency): 50Mhz */
#define KXSD9_BW_DEFAULT	(KXSD9_REGC_LP0 | KXSD9_REGC_LP1 |\
						KXSD9_REGC_LP2)

#define MAX_12BIT		((1 << 12) - 1)

#endif   // __KXSD9_H__

