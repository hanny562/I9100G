#ifndef _A1026_I2C_DRV_H
#define _A1026_I2C_DRV_H

#define	GPIO_2MIC_RST		11	
#define	GPIO_2MIC_POWERDOWN	12		
#define	GPIO_2MIC_EN		169

#define	A1026_2MIC_CALL_ON	0x1<<0

extern int a1026_i2c_drv_init(void);
extern void a1026_i2c_drv_exit(void);


#endif

