/*
 * modules/camera/cam_pmic.h
 *
 * Camera PMIC driver header file
 *
 * Written by paladin in Samsung Electronics
 *
 * Modified by Srinidhi Rao (srinidhi.rao@samsung.com) to port 
 * Cam PMIC driver to Samsung OMAP4 Universal Board
 */


#ifndef CAM_PMIC_H
#define CAM_PMIC_H

#define CAM_PMIC_DBG          0
#define CAM_PMIC_DRIVER_NAME  "cam_pmic"
#define CAM_PMIC_MOD_NAME     "CAM_PMIC: "
#define CAM_PMIC_I2C_RETRY    10

//#define CAM_PMIC_I2C_ADDR  0x7C
#define CAM_PMIC_I2C_ADDR	0x7D

struct cam_pmic {
  struct i2c_client *i2c_client;
};

#define CAM_PMIC_UNINIT_VALUE   0xFF

int cam_pmic_read_reg(u8 reg, u8* val);
int cam_pmic_write_reg(u8 reg, u8 val);
#endif /* ifndef CAM_PMIC_H */
