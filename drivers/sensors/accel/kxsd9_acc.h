#ifndef __KXSD9_ACC_HEADER__
#define __KXSD9_ACC_HEADER__

#include "kxsd9_i2c.h"
#include "kxsd9.h"

#define ACC_DEV_NAME		"accelerometer"
#define ACC_DEV_MAJOR		241

#define KXSD9_MAJOR		100

/* kxsd9 ioctl command label */
#define IOCTL_KXSD9_GET_ACC_VALUE		0
#define DCM_IOC_MAGIC				's'
#define IOC_SET_ACCELEROMETER			_IO (DCM_IOC_MAGIC, 0x64)


#define KXSD9_POWER_OFF		0
#define KXSD9_POWER_ON		1

#endif
