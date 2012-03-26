/*
 * Driver for M5MO ISP
 *
 * Sensor can be attached up to resolution of 4096*1664
 * Embedded JPEG encoder and object recognition as well
 *
 * NOTICE: This driver is just for TEST ONLY.
 * working only with one resolution
 *
 * Copyright (C) 2009, Dongsoo Nathaniel Kim<dongsoo45.kim@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

enum m5mo_data_length {
	M5MO_8BIT	= 1,
	M5MO_16BIT	= 2,
	M5MO_32BIT	= 4,
};

enum m5mo_phy {
	M5MO_PAR	= 0,
	M5MO_HDMI	= 1,
	M5MO_MIPI	= 2,
};

struct m5mo_fw_info {
	const char *fw_name;	/* firmware filename */
	unsigned int fw_ver_vendor1;
	unsigned int fw_ver_vendor2;
	unsigned int fw_ver_y;
	unsigned int fw_ver_m;
	unsigned int fw_ver_d1;
	unsigned int fw_ver_d2;
};

struct m5mo_platform_data {
	struct v4l2_pix_format pix;
	unsigned int data_length;

	int freq;		/* MCLK in KHz */
	enum m5mo_phy phy_con;	/* using mipi interface */

	/* F/W information */
	struct m5mo_fw_info *op_fw_ver;	/* starts with "O" */
	struct m5mo_fw_info *se_fw_ver;	/* starts with "S" */
};

