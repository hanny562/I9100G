#ifndef __ISL29023_I2C_H
#define __ISL29023_I2C_H

extern int isl29023_i2c_read(struct i2c_client*, u8 , u8*);
extern int isl29023_i2c_write(struct i2c_client*, u8 , u8);
extern int isl29023_i2c_init(void);
extern void isl29023_i2c_exit(void);

#endif
