/*
 * SAMSUNG LPDDR2 timings.
 *
 * Copyright (C) 2010 Texas Instruments
 *
 * Aneesh V <aneesh@ti.com>
 * Santosh Shilimkar <santosh.shilimkar@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _LPDDR2_SEC_H
#define _LPDDR2_SEC_H

extern const struct lpddr2_timings timings_sec_200_mhz;
extern const struct lpddr2_timings timings_sec_333_mhz;
extern const struct lpddr2_timings timings_sec_400_mhz;
extern const struct lpddr2_min_tck min_tck_sec;
extern struct lpddr2_device_info sec_2G_S4;
extern struct lpddr2_device_info sec_4G_S4;

#endif
