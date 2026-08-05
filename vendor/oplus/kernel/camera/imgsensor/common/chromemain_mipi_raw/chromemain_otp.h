/*
 * Copyright (C) 2024 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 chromemain_otp.h
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 sensor otp header file
 *
 ****************************************************************************/
#ifndef __CHROMEMAIN_OTP_H
#define __CHROMEMAIN_OTP_H

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/of.h>
#include <linux/dma-mapping.h>
#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_errcode.h"
#include "oplus-adaptor-subdrv-ctrl.h"
#include "adaptor-subdrv-ctrl.h"
#include "adaptor-i2c.h"
#include "adaptor.h"

#define OTP_I2C_ADDR 0x20
#define CHROMEMAIN_OTP_RET_FAIL -1
#define CHROMEMAIN_OTP_RET_SUCCESS 0
#define CHROMEMAIN_TXD_MAIN_OTP_MODULE_LENS 23
#define CHROMEMAIN_TXD_MAIN_OTP_AWB_LENS    20
#define CHROMEMAIN_TXD_main_OTP_AF_LENS    4
#define CHROMEMAIN_TXD_MAIN_OTP_LSC_LENS    1868

#if 0
// no lrc/sn data
#define CHROMEMAIN_TXD_MAIN_OTP_LRC_LENS    6
#define CHROMEMAIN_TXD_MAIN_OTP_SN_LENS 	 25
#endif

//flag addr
#define CHROMEMAIN_OTP_MODULE_FLAGADDR 0x827A
#define CHROMEMAIN_GROUP1_FLAG 0x01
#define CHROMEMAIN_GROUP2_FLAG 0x13
#define CHROMEMAIN_INVALID_FLAG 0x0


//data start addr
#define CHROMEMAIN_OTP_MODULE_GROUP1_STARTADDR 0x827B
#define CHROMEMAIN_OTP_AWB_GROUP1_STARTADDR 0x82AC
#define CHROMEMAIN_OTP_AF_GROUP1_STARTADDR 0x82D7

#define CHROMEMAIN_OTP_LRC_GROUP1_STARTADDR 0x82C3
#define CHROMEMAIN_OTP_SN_GROUP1_STARTADDR 0x82CF

#define CHROMEMAIN_OTP_MODULE_GROUP2_STARTADDR 0x8293
#define CHROMEMAIN_OTP_AWB_GROUP2_STARTADDR 0x82C1
#define CHROMEMAIN_OTP_AF_GROUP2_STARTADDR 0x82DC

#if 0
// no lrc/sn data
#define CHROMEMAIN_OTP_LRC_GROUP2_STARTADDR 0x82C9
#define CHROMEMAIN_OTP_SN_GROUP2_STARTADDR 0x82E9
#endif

enum chromemain_sensor_otp_page{
	page_0 = 0,
	page_1,
	page_2,
	page_3,
	page_4,
	page_5,
	page_6,
	page_7,
	page_8,
	page_9,
	page_10,
	page_11,
	page_12,
	page_max
};

struct chromemain_txd_main_otp_struct {
	UINT8 ModuleFlag;
	UINT8 module_info[CHROMEMAIN_TXD_MAIN_OTP_MODULE_LENS];
	UINT8 awb_data[CHROMEMAIN_TXD_MAIN_OTP_AWB_LENS];
	UINT8 af_data[CHROMEMAIN_TXD_main_OTP_AF_LENS];
	#if 0
	UINT8 lrc_data[CHROMEMAIN_TXD_MAIN_OTP_LRC_LENS];
	UINT8 sn_data[CHROMEMAIN_TXD_MAIN_OTP_SN_LENS];
	#endif
	UINT8 lsc_data[CHROMEMAIN_TXD_MAIN_OTP_LSC_LENS];
};

extern struct chromemain_txd_main_otp_struct chromemain_txd_main_otp;
extern int chromemain_sensor_otp_read_all_data(struct i2c_client *client);

#endif
