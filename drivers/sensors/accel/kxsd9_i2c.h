#ifndef __KXSD9_I2C_HEADER__
#define __KXSD9_I2C_HEADER__

int kxsd9_i2c_rxdata(char *rxData, int length);
int kxsd9_i2c_txdata(char *txData, int length);

int  i2c_acc_kxsd9_init(void);
void i2c_acc_kxsd9_exit(void);

#endif
