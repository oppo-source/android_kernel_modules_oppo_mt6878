/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2023 OPLUS Inc.
 */

#define PFX "CAM_CAL_LYCHEEF4UWIDE"
#define pr_fmt(fmt) PFX "[%s] " fmt, __func__

#include <linux/kernel.h>
#include "cam_cal_list.h"
#include "eeprom_i2c_common_driver.h"
#include "eeprom_i2c_custom_driver.h"
#include "cam_cal_config.h"
#include "oplus_kd_imgsensor.h"

#define READ_4000K 0
#define DEBUG_CALIBRATION_LOAD

static unsigned int do_single_lsc_lycheef4uwide(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData);
static unsigned int do_2a_gain_lycheef4uwide(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData);
static unsigned int do_lens_id_lycheef4uwide(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData);
static unsigned int do_pdaf_lycheef4uwide(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData);

static struct STRUCT_CALIBRATION_LAYOUT_STRUCT cal_layout_table = {
	0x00000006, 0x016F0131, CAM_CAL_SINGLE_EEPROM_DATA,
	{
		{0x00000001, 0x00000000, 0x00000002, do_module_version},
		{0x00000001, 0x00000000, 0x00000011, do_part_number},
		{0x00000001, 0x00000B00, 0x0000074C, do_single_lsc_lycheef4uwide},
		{0x00000001, 0x00000020, 0x0000000E, do_2a_gain_lycheef4uwide}, //Start address, block size is useless
		{0x00000000, 0x00001300, 0x000005DC, do_pdaf_lycheef4uwide},
		{0x00000000, 0x00000FAE, 0x00000550, do_stereo_data},
		{0x00000001, 0x00000000, 0x00002000, do_dump_all},
		{0x00000001, 0x00000008, 0x00000002, do_lens_id_lycheef4uwide}
	}
};

struct STRUCT_CAM_CAL_CONFIG_STRUCT lycheef4uwide_op_eeprom = {
	.name = "lycheef4uwide_op_eeprom",
	.check_layout_function = layout_check,
	.read_function = Common_read_region,
	.layout = &cal_layout_table,
	.sensor_id = LYCHEEF4UWIDE_SENSOR_ID,
	.i2c_write_id = 0xA2,
	.max_size = 0x2000,
	.enable_preload = 1,
	.preload_size = 0x2000,
	.has_stored_data = 1,
};

static unsigned int do_single_lsc_lycheef4uwide(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData)
{
	struct STRUCT_CAM_CAL_DATA_STRUCT *pCamCalData =
				(struct STRUCT_CAM_CAL_DATA_STRUCT *)pGetSensorCalData;

	int read_data_size;
	unsigned int err = CamCalReturnErr[pCamCalData->Command];
	unsigned short table_size;

	if (pCamCalData->DataVer >= CAM_CAL_TYPE_NUM) {
		err = CAM_CAL_ERR_NO_DEVICE;
		error_log("Read Failed\n");
		show_cmd_error_log(pCamCalData->Command);
		return err;
	}
	if (block_size != CAM_CAL_SINGLE_LSC_SIZE)
		error_log("block_size(%d) is not match (%d)\n",
				block_size, CAM_CAL_SINGLE_LSC_SIZE);

	pCamCalData->SingleLsc.LscTable.MtkLcsData.MtkLscType = 2;//mtk type
	pCamCalData->SingleLsc.LscTable.MtkLcsData.PixId = 8;

	table_size = 1868;

	pr_debug("lsc table_size %d\n", table_size);
	pCamCalData->SingleLsc.LscTable.MtkLcsData.TableSize = table_size;
	if (table_size > 0) {
		pCamCalData->SingleLsc.TableRotation = 0; // flip
		debug_log("u4Offset=%d u4Length=%d", start_addr, table_size);
		read_data_size = read_data(pdata,
			pCamCalData->sensorID, pCamCalData->deviceID,
			start_addr, table_size, (unsigned char *)
			&pCamCalData->SingleLsc.LscTable.MtkLcsData.SlimLscType);
		if (table_size == read_data_size)
			err = CAM_CAL_ERR_NO_ERR;
		else {
			error_log("Read Failed\n");
			err = CamCalReturnErr[pCamCalData->Command];
			show_cmd_error_log(pCamCalData->Command);
		}
	}
	#ifdef DEBUG_CALIBRATION_LOAD
	pr_debug("======================SingleLsc Data==================\n");
	pr_debug("[1st] = %x, %x, %x, %x\n",
		pCamCalData->SingleLsc.LscTable.Data[0],
		pCamCalData->SingleLsc.LscTable.Data[1],
		pCamCalData->SingleLsc.LscTable.Data[2],
		pCamCalData->SingleLsc.LscTable.Data[3]);
	pr_debug("[1st] = SensorLSC(1)?MTKLSC(2)?  %x\n",
		pCamCalData->SingleLsc.LscTable.MtkLcsData.MtkLscType);
	pr_debug("CapIspReg =0x%x, 0x%x, 0x%x, 0x%x, 0x%x",
		pCamCalData->SingleLsc.LscTable.MtkLcsData.CapIspReg[0],
		pCamCalData->SingleLsc.LscTable.MtkLcsData.CapIspReg[1],
		pCamCalData->SingleLsc.LscTable.MtkLcsData.CapIspReg[2],
		pCamCalData->SingleLsc.LscTable.MtkLcsData.CapIspReg[3],
		pCamCalData->SingleLsc.LscTable.MtkLcsData.CapIspReg[4]);
	pr_debug("RETURN = 0x%x\n", err);
	pr_debug("======================SingleLsc Data==================\n");
	#endif

	return err;
}

static unsigned int do_2a_gain_lycheef4uwide(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData)
{
	struct STRUCT_CAM_CAL_DATA_STRUCT *pCamCalData =
				(struct STRUCT_CAM_CAL_DATA_STRUCT *)pGetSensorCalData;
	int read_data_size;
	unsigned int err = CamCalReturnErr[pCamCalData->Command];

	long long cal_gain, fac_gain, cal_value;
	unsigned char awb_afconfig = 0xf;

	unsigned short af_inf, af_macro, af_50cm;
	int tempmax = 0;
	int cal_r = 1, cal_gr = 1, cal_gb = 1, cal_g = 1, cal_b = 1;
	int fac_r = 1, fac_gr = 1, fac_gb = 1, fac_g = 1, fac_b = 1;
	int rgcal_value = 1, bgcal_value = 1;
	unsigned int awb_offset, af_offset;

	(void) start_addr;
	(void) block_size;

	pr_debug("In %s: sensor_id=%x\n", __func__, pCamCalData->sensorID);
	memset((void *)&pCamCalData->Single2A, 0, sizeof(struct STRUCT_CAM_CAL_SINGLE_2A_STRUCT));
	/* Check rule */
	if (pCamCalData->DataVer >= CAM_CAL_TYPE_NUM) {
		err = CAM_CAL_ERR_NO_DEVICE;
		error_log("Read Failed\n");
		show_cmd_error_log(pCamCalData->Command);
		return err;
	}
	/* Check AWB & AF enable bit */
	pCamCalData->Single2A.S2aVer = 0x01;
	pCamCalData->Single2A.S2aBitEn = (0x03 & awb_afconfig);
	pCamCalData->Single2A.S2aAfBitflagEn = (0x0C & awb_afconfig);
	debug_log("S2aBitEn=0x%02x", pCamCalData->Single2A.S2aBitEn);
	/* AWB Calibration Data*/
	if (0x1 & awb_afconfig) {
		pCamCalData->Single2A.S2aAwb.rGainSetNum = 0x02;
		awb_offset = 0x60;
		read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
				awb_offset, 8, (unsigned char *)&cal_value);
		if (read_data_size > 0)	{
			debug_log("Read cal_value OK\n");
			rgcal_value  = cal_value & 0xFFFF;
			bgcal_value = (cal_value >> 16) & 0xFFFF;
			debug_log("Light source calibration 5100K value R/G:%d, B/G:%d", rgcal_value, bgcal_value);
			err = CAM_CAL_ERR_NO_ERR;
		} else {
			pCamCalData->Single2A.S2aBitEn = CAM_CAL_NONE_BITEN;
			error_log("Read cal_gain Failed\n");
			show_cmd_error_log(pCamCalData->Command);
		}
		/* AWB Unit Gain (5000K) */
		debug_log("5000K AWB\n");
		awb_offset = 0x0020;
		read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
				awb_offset, 8, (unsigned char *)&cal_gain);
		if (read_data_size > 0)	{
			debug_log("Read cal_gain OK %x\n", read_data_size);
			cal_r  = cal_gain & 0xFFFF;
			cal_gr = (cal_gain >> 16) & 0xFFFF;
			cal_gb = (cal_gain >> 32) & 0xFFFF;
			cal_g  = ((cal_gr + cal_gb) + 1) >> 1;
			cal_b  = (cal_gain >> 48) & 0xFFFF;
			debug_log("cal_r:%d, cal_g:%d, cal_b:%d", cal_r, cal_g, cal_b);
			cal_r  = (cal_r * rgcal_value + 500) / 1000;
			cal_b  = (cal_b * bgcal_value + 500) / 1000;
			if (cal_r > cal_g)
				/* R > G */
				if (cal_r > cal_b)
					tempmax = cal_r;
				else
					tempmax = cal_b;
			else
				/* G > R */
				if (cal_g > cal_b)
					tempmax = cal_g;
				else
					tempmax = cal_b;
			debug_log("UnitR:%d, UnitG:%d, UnitB:%d, New Unit Max=%d",
					cal_r, cal_g, cal_b, tempmax);
			err = CAM_CAL_ERR_NO_ERR;
		} else {
			pCamCalData->Single2A.S2aBitEn = CAM_CAL_NONE_BITEN;
			error_log("Read cal_gain Failed\n");
			show_cmd_error_log(pCamCalData->Command);
		}
		if (cal_gain != 0x0000000000000000 &&
			cal_gain != 0xFFFFFFFFFFFFFFFF &&
			cal_r    != 0x00000000 &&
			cal_g    != 0x00000000 &&
			cal_b    != 0x00000000) {
			pCamCalData->Single2A.S2aAwb.rGainSetNum = 1;
			pCamCalData->Single2A.S2aAwb.rUnitGainu4R =
					(unsigned int)((tempmax * 512 + (cal_r >> 1)) / cal_r);
			pCamCalData->Single2A.S2aAwb.rUnitGainu4G =
					(unsigned int)((tempmax * 512 + (cal_g >> 1)) / cal_g);
			pCamCalData->Single2A.S2aAwb.rUnitGainu4B =
					(unsigned int)((tempmax * 512 + (cal_b >> 1)) / cal_b);
		} else {
			pr_debug("There are something wrong on EEPROM, plz contact module vendor!!\n");
			pr_debug("Unit R=%d G=%d B=%d!!\n", cal_r, cal_g, cal_b);
		}
		/* AWB Golden Gain (5000K) */
		awb_offset = 0x0028;
		read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
				awb_offset, 8, (unsigned char *)&fac_gain);
		if (read_data_size > 0)	{
			debug_log("Read fac_gain OK\n");
			fac_r  = fac_gain & 0xFFFF;
			fac_gr = (fac_gain >> 16) & 0xFFFF;
			fac_gb = (fac_gain >> 32) & 0xFFFF;
			fac_g  = ((fac_gr + fac_gb) + 1) >> 1;
			fac_b  = (fac_gain >> 48) & 0xFFFF;
			if (fac_r > fac_g)
				if (fac_r > fac_b)
					tempmax = fac_r;
				else
					tempmax = fac_b;
			else
				if (fac_g > fac_b)
					tempmax = fac_g;
				else
					tempmax = fac_b;
			debug_log("GoldenR:%d, GoldenG:%d, GoldenB:%d, New Golden Max=%d",
					fac_r, fac_g, fac_b, tempmax);
			err = CAM_CAL_ERR_NO_ERR;
		} else {
			pCamCalData->Single2A.S2aBitEn = CAM_CAL_NONE_BITEN;
			error_log("Read fac_gain Failed\n");
			show_cmd_error_log(pCamCalData->Command);
		}
		if (fac_gain != 0x0000000000000000 &&
			fac_gain != 0xFFFFFFFFFFFFFFFF &&
			fac_r    != 0x00000000 &&
			fac_g    != 0x00000000 &&
			fac_b    != 0x00000000)	{
			pCamCalData->Single2A.S2aAwb.rGoldGainu4R =
					(unsigned int)((tempmax * 512 + (fac_r >> 1)) / fac_r);
			pCamCalData->Single2A.S2aAwb.rGoldGainu4G =
					(unsigned int)((tempmax * 512 + (fac_g >> 1)) / fac_g);
			pCamCalData->Single2A.S2aAwb.rGoldGainu4B =
					(unsigned int)((tempmax * 512 + (fac_b >> 1)) / fac_b);
		} else {
			pr_debug("There are something wrong on EEPROM, plz contact module vendor!!");
			pr_debug("Golden R=%d G=%d B=%d\n", fac_r, fac_g, fac_b);
		}
		/* Set AWB to 3A Layer */
		pCamCalData->Single2A.S2aAwb.rValueR   = cal_r;
		pCamCalData->Single2A.S2aAwb.rValueGr  = cal_gr;
		pCamCalData->Single2A.S2aAwb.rValueGb  = cal_gb;
		pCamCalData->Single2A.S2aAwb.rValueB   = cal_b;
		pCamCalData->Single2A.S2aAwb.rGoldenR  = fac_r;
		pCamCalData->Single2A.S2aAwb.rGoldenGr = fac_gr;
		pCamCalData->Single2A.S2aAwb.rGoldenGb = fac_gb;
		pCamCalData->Single2A.S2aAwb.rGoldenB  = fac_b;
		#ifdef DEBUG_CALIBRATION_LOAD
		pr_debug("======================AWB CAM_CAL==================\n");
		pr_debug("AWB Calibration @5100K\n");
		pr_debug("[cal_gain] = %llu\n", cal_gain);
		pr_debug("[fac_gain] = %llu\n", fac_gain);
		pr_debug("[rcal_gain.u4R] = %d\n", pCamCalData->Single2A.S2aAwb.rUnitGainu4R);
		pr_debug("[rcal_gain.u4G] = %d\n", pCamCalData->Single2A.S2aAwb.rUnitGainu4G);
		pr_debug("[rcal_gain.u4B] = %d\n", pCamCalData->Single2A.S2aAwb.rUnitGainu4B);
		pr_debug("[rfac_gain.u4R] = %d\n", pCamCalData->Single2A.S2aAwb.rGoldGainu4R);
		pr_debug("[rfac_gain.u4G] = %d\n", pCamCalData->Single2A.S2aAwb.rGoldGainu4G);
		pr_debug("[rfac_gain.u4B] = %d\n", pCamCalData->Single2A.S2aAwb.rGoldGainu4B);
		#endif
		/* AWB Unit Gain (4000K) */
		#if READ_4000K //zemin.lai@CamTuning delet ,module not support 4000k otp 20220421
		cal_r = 0; cal_gr = 0; cal_gb = 0; cal_g = 0; cal_b = 0;
		tempmax = 0;
		debug_log("4000K AWB\n");
		awb_offset = 0x0032;
		read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
				awb_offset, 8, (unsigned char *)&cal_gain);
		if (read_data_size > 0)	{
			debug_log("Read cal_gain OK %x\n", read_data_size);
			cal_r  = cal_gain & 0xFFFF;
			cal_gr = (cal_gain >> 16) & 0xFFFF;
			cal_gb = (cal_gain >> 32) & 0xFFFF;
			cal_g  = ((cal_gr + cal_gb) + 1) >> 1;
			cal_b  = (cal_gain >> 48) & 0xFFFF;
			if (cal_r > cal_g)
				/* R > G */
				if (cal_r > cal_b)
					tempmax = cal_r;
				else
					tempmax = cal_b;
			else
				/* G > R */
				if (cal_g > cal_b)
					tempmax = cal_g;
				else
					tempmax = cal_b;
			debug_log(
					"UnitR:%d, UnitG:%d, UnitB:%d, New Unit Max=%d",
					cal_r, cal_g, cal_b, tempmax);
			err = CAM_CAL_ERR_NO_ERR;
		} else {
			pCamCalData->Single2A.S2aBitEn = CAM_CAL_NONE_BITEN;
			error_log("Read cal_gain Failed\n");
			show_cmd_error_log(pCamCalData->Command);
		}
		if (cal_gain != 0x0000000000000000 &&
			cal_gain != 0xFFFFFFFFFFFFFFFF &&
			cal_r    != 0x00000000 &&
			cal_g    != 0x00000000 &&
			cal_b    != 0x00000000) {
			pCamCalData->Single2A.S2aAwb.rUnitGainu4R_mid =
				(unsigned int)((tempmax * 512 + (cal_r >> 1)) / cal_r);
			pCamCalData->Single2A.S2aAwb.rUnitGainu4G_mid =
				(unsigned int)((tempmax * 512 + (cal_g >> 1)) / cal_g);
			pCamCalData->Single2A.S2aAwb.rUnitGainu4B_mid =
				(unsigned int)((tempmax * 512 + (cal_b >> 1)) / cal_b);
		} else {
			pr_debug("There are something wrong on EEPROM, plz contact module vendor!!\n");
			pr_debug("Unit R=%d G=%d B=%d!!\n", cal_r, cal_g, cal_b);
		}
		/* AWB Golden Gain (4000K) */
		fac_r = 0; fac_gr = 0; fac_gb = 0; fac_g = 0; fac_b = 0;
		tempmax = 0;
		awb_offset = 0x003A;
		read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
				awb_offset, 8, (unsigned char *)&fac_gain);
		if (read_data_size > 0)	{
			debug_log("Read fac_gain OK\n");
			fac_r  = fac_gain & 0xFFFF;
			fac_gr = (fac_gain >> 16) & 0xFFFF;
			fac_gb = (fac_gain >> 32) & 0xFFFF;
			fac_g  = ((fac_gr + fac_gb) + 1) >> 1;
			fac_b  = (fac_gain >> 48) & 0xFFFF;
			if (fac_r > fac_g)
				if (fac_r > fac_b)
					tempmax = fac_r;
				else
					tempmax = fac_b;
			else
				if (fac_g > fac_b)
					tempmax = fac_g;
				else
					tempmax = fac_b;
			debug_log("GoldenR:%d, GoldenG:%d, GoldenB:%d, New Golden Max=%d",
					fac_r, fac_g, fac_b, tempmax);
			err = CAM_CAL_ERR_NO_ERR;
		} else {
			pCamCalData->Single2A.S2aBitEn = CAM_CAL_NONE_BITEN;
			error_log("Read fac_gain Failed\n");
			show_cmd_error_log(pCamCalData->Command);
		}
		if (fac_gain != 0x0000000000000000 &&
			fac_gain != 0xFFFFFFFFFFFFFFFF &&
			fac_r    != 0x00000000 &&
			fac_g    != 0x00000000 &&
			fac_b    != 0x00000000)	{
			pCamCalData->Single2A.S2aAwb.rGoldGainu4R_mid =
				(unsigned int)((tempmax * 512 + (fac_r >> 1)) / fac_r);
			pCamCalData->Single2A.S2aAwb.rGoldGainu4G_mid =
				(unsigned int)((tempmax * 512 + (fac_g >> 1)) / fac_g);
			pCamCalData->Single2A.S2aAwb.rGoldGainu4B_mid =
				(unsigned int)((tempmax * 512 + (fac_b >> 1)) / fac_b);
		} else {
			pr_debug("There are something wrong on EEPROM, plz contact module vendor!!");
			pr_debug("Golden R=%d G=%d B=%d\n", fac_r, fac_g, fac_b);
		}
		#ifdef DEBUG_CALIBRATION_LOAD
		pr_debug("AWB Calibration @4000K\n");
		pr_debug("[cal_gain] = %llu\n", cal_gain);
		pr_debug("[fac_gain] = %llu\n", fac_gain);
		pr_debug("[rcal_gain.u4R] = %d\n", pCamCalData->Single2A.S2aAwb.rUnitGainu4R_mid);
		pr_debug("[rcal_gain.u4G] = %d\n", pCamCalData->Single2A.S2aAwb.rUnitGainu4G_mid);
		pr_debug("[rcal_gain.u4B] = %d\n", pCamCalData->Single2A.S2aAwb.rUnitGainu4B_mid);
		pr_debug("[rfac_gain.u4R] = %d\n", pCamCalData->Single2A.S2aAwb.rGoldGainu4R_mid);
		pr_debug("[rfac_gain.u4G] = %d\n", pCamCalData->Single2A.S2aAwb.rGoldGainu4G_mid);
		pr_debug("[rfac_gain.u4B] = %d\n", pCamCalData->Single2A.S2aAwb.rGoldGainu4B_mid);
		#endif
		#endif
		/* AWB Unit Gain (2850K) */
		cal_r = 0; cal_gr = 0; cal_gb = 0; cal_g = 0; cal_b = 0;
		tempmax = 0;
		debug_log("2850K AWB\n");
		debug_log("3100K AWB\n");
		rgcal_value = 0; bgcal_value = 0;
		awb_offset = 0x6C;
		read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
				awb_offset, 8, (unsigned char *)&cal_value);
		if (read_data_size > 0)	{
			debug_log("Read cal_value OK\n");
			rgcal_value  = cal_value & 0xFFFF;
			bgcal_value = (cal_value >> 16) & 0xFFFF;
			debug_log("Light source calibration value 3100 R/G:%d, B/G:%d", rgcal_value, bgcal_value);
			err = CAM_CAL_ERR_NO_ERR;
		} else {
			pCamCalData->Single2A.S2aBitEn = CAM_CAL_NONE_BITEN;
			error_log("Read cal_gain Failed\n");
			show_cmd_error_log(pCamCalData->Command);
		}
		awb_offset = 0x0044;
		read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
				awb_offset, 8, (unsigned char *)&cal_gain);
		if (read_data_size > 0)	{
			debug_log("Read cal_gain OK %x\n", read_data_size);
			cal_r  = cal_gain & 0xFFFF;
			cal_gr = (cal_gain >> 16) & 0xFFFF;
			cal_gb = (cal_gain >> 32) & 0xFFFF;
			cal_g  = ((cal_gr + cal_gb) + 1) >> 1;
			cal_b  = (cal_gain >> 48) & 0xFFFF;
			debug_log("cal_r:%d, cal_g:%d, cal_b:%d", cal_r, cal_g, cal_b);
			cal_r  = (cal_r * rgcal_value + 500) / 1000;
			cal_b  = (cal_b * bgcal_value + 500) / 1000;
			if (cal_r > cal_g)
				/* R > G */
				if (cal_r > cal_b)
					tempmax = cal_r;
				else
					tempmax = cal_b;
			else
				/* G > R */
				if (cal_g > cal_b)
					tempmax = cal_g;
				else
					tempmax = cal_b;
			debug_log("UnitR:%d, UnitG:%d, UnitB:%d, New Unit Max=%d",
					cal_r, cal_g, cal_b, tempmax);
			err = CAM_CAL_ERR_NO_ERR;
		} else {
			pCamCalData->Single2A.S2aBitEn = CAM_CAL_NONE_BITEN;
			error_log("Read cal_gain Failed\n");
			show_cmd_error_log(pCamCalData->Command);
		}
		if (cal_gain != 0x0000000000000000 &&
			cal_gain != 0xFFFFFFFFFFFFFFFF &&
			cal_r    != 0x00000000 &&
			cal_g    != 0x00000000 &&
			cal_b    != 0x00000000) {
			pCamCalData->Single2A.S2aAwb.rGainSetNum = 2;
			pCamCalData->Single2A.S2aAwb.rUnitGainu4R_low =
				(unsigned int)((tempmax * 512 + (cal_r >> 1)) / cal_r);
			pCamCalData->Single2A.S2aAwb.rUnitGainu4G_low =
				(unsigned int)((tempmax * 512 + (cal_g >> 1)) / cal_g);
			pCamCalData->Single2A.S2aAwb.rUnitGainu4B_low =
				(unsigned int)((tempmax * 512 + (cal_b >> 1)) / cal_b);
		} else {
			pr_debug("There are something wrong on EEPROM, plz contact module vendor!!\n");
			pr_debug("Unit R=%d G=%d B=%d!!\n", cal_r, cal_g, cal_b);
		}
		/* AWB Golden Gain (3100K) */
		fac_r = 0; fac_gr = 0; fac_gb = 0; fac_g = 0; fac_b = 0;
		tempmax = 0;
		awb_offset = 0x004C;
		read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
				awb_offset, 8, (unsigned char *)&fac_gain);
		if (read_data_size > 0)	{
			debug_log("Read fac_gain OK\n");
			fac_r  = fac_gain & 0xFFFF;
			fac_gr = (fac_gain >> 16) & 0xFFFF;
			fac_gb = (fac_gain >> 32) & 0xFFFF;
			fac_g  = ((fac_gr + fac_gb) + 1) >> 1;
			fac_b  = (fac_gain >> 48) & 0xFFFF;
			if (fac_r > fac_g)
				if (fac_r > fac_b)
					tempmax = fac_r;
				else
					tempmax = fac_b;
			else
				if (fac_g > fac_b)
					tempmax = fac_g;
				else
					tempmax = fac_b;
			debug_log("GoldenR:%d, GoldenG:%d, GoldenB:%d, New Golden Max=%d",
					fac_r, fac_g, fac_b, tempmax);
			err = CAM_CAL_ERR_NO_ERR;
		} else {
			pCamCalData->Single2A.S2aBitEn = CAM_CAL_NONE_BITEN;
			error_log("Read fac_gain Failed\n");
			show_cmd_error_log(pCamCalData->Command);
		}
		if (fac_gain != 0x0000000000000000 &&
			fac_gain != 0xFFFFFFFFFFFFFFFF &&
			fac_r    != 0x00000000 &&
			fac_g    != 0x00000000 &&
			fac_b    != 0x00000000)	{
			pCamCalData->Single2A.S2aAwb.rGoldGainu4R_low =
				(unsigned int)((tempmax * 512 + (fac_r >> 1)) / fac_r);
			pCamCalData->Single2A.S2aAwb.rGoldGainu4G_low =
				(unsigned int)((tempmax * 512 + (fac_g >> 1)) / fac_g);
			pCamCalData->Single2A.S2aAwb.rGoldGainu4B_low =
				(unsigned int)((tempmax * 512 + (fac_b >> 1)) / fac_b);
		} else {
			pr_debug("There are something wrong on EEPROM, plz contact module vendor!!");
			pr_debug("Golden R=%d G=%d B=%d\n", fac_r, fac_g, fac_b);
		}
		#ifdef DEBUG_CALIBRATION_LOAD
		pr_debug("AWB Calibration @3100K\n");
		pr_debug("[cal_gain] = %llu\n", cal_gain);
		pr_debug("[fac_gain] = %llu\n", fac_gain);
		pr_debug("[rcal_gain.u4R] = %d\n", pCamCalData->Single2A.S2aAwb.rUnitGainu4R_low);
		pr_debug("[rcal_gain.u4G] = %d\n", pCamCalData->Single2A.S2aAwb.rUnitGainu4G_low);
		pr_debug("[rcal_gain.u4B] = %d\n", pCamCalData->Single2A.S2aAwb.rUnitGainu4B_low);
		pr_debug("[rfac_gain.u4R] = %d\n", pCamCalData->Single2A.S2aAwb.rGoldGainu4R_low);
		pr_debug("[rfac_gain.u4G] = %d\n", pCamCalData->Single2A.S2aAwb.rGoldGainu4G_low);
		pr_debug("[rfac_gain.u4B] = %d\n", pCamCalData->Single2A.S2aAwb.rGoldGainu4B_low);
		pr_debug("======================AWB CAM_CAL==================\n");
		#endif
	}
	/* AF Calibration Data*/
	if (0x2 & awb_afconfig) {
		// AF 50cm
		af_offset = 0x0096;
		read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
				af_offset, 2, (unsigned char *)&af_50cm);
		if (read_data_size > 0)
			err = CAM_CAL_ERR_NO_ERR;
		else {
			pCamCalData->Single2A.S2aBitEn = CAM_CAL_NONE_BITEN;
			error_log("Read Failed\n");
			show_cmd_error_log(pCamCalData->Command);
		}

		// AF Inf
		af_offset = 0x0094;
		read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
				af_offset, 2, (unsigned char *)&af_inf);
		if (read_data_size > 0)
			err = CAM_CAL_ERR_NO_ERR;
		else {
			pCamCalData->Single2A.S2aBitEn = CAM_CAL_NONE_BITEN;
			error_log("Read Failed\n");
			show_cmd_error_log(pCamCalData->Command);
		}

		// AF Macro
		af_offset = 0x0092;
		read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
				af_offset, 2, (unsigned char *)&af_macro);
		if (read_data_size > 0)
			err = CAM_CAL_ERR_NO_ERR;
		else {
			pCamCalData->Single2A.S2aBitEn = CAM_CAL_NONE_BITEN;
			error_log("Read Failed\n");
			show_cmd_error_log(pCamCalData->Command);
		}
		/*11bit Conversion 10bit*/
		af_inf = af_inf >> 1;
		af_macro = af_macro >> 1;
		af_50cm = af_50cm >> 1;

		pCamCalData->Single2A.S2aAf[0] = af_inf;
		pCamCalData->Single2A.S2aAf[1] = af_macro;
		pCamCalData->Single2A.S2aAf[2] = af_50cm;

		////Only AF Gathering <////
		#ifdef DEBUG_CALIBRATION_LOAD
		pr_debug("======================AF CAM_CAL==================\n");
		pr_debug("[af_inf] = %d\n", af_inf);
		pr_debug("[af_macro] = %d\n", af_macro);
		pr_debug("[af_50cm] = %d\n", af_50cm);
		pr_debug("======================AF CAM_CAL==================\n");
		#endif
	}
	return err;
}

static unsigned int do_lens_id_lycheef4uwide(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData)
{
	return do_lens_id_base(pdata, start_addr, block_size, pGetSensorCalData);
}

#define LYCHEEF4UWIDE_HVBIN_PDAF_PROC1_SIZE  (496)
#define LYCHEEF4UWIDE_HVBIN_PDAF_PROC2_SIZE  (1004)
#define LYCHEEF4UWIDE_HVBIN_PDAF_PROC1_ADDR  (0x1300)
#define LYCHEEF4UWIDE_HVBIN_PDAF_PROC2_ADDR  (0x1500)
static unsigned int do_pdaf_lycheef4uwide(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData)
{
	struct STRUCT_CAM_CAL_DATA_STRUCT *pCamCalData =
				(struct STRUCT_CAM_CAL_DATA_STRUCT *)pGetSensorCalData;

	int read_data_size;
	int err =  CamCalReturnErr[pCamCalData->Command];
	unsigned char is_valid = 0;

	pCamCalData->PDAF.Size_of_PDAF = LYCHEEF4UWIDE_HVBIN_PDAF_PROC1_SIZE + LYCHEEF4UWIDE_HVBIN_PDAF_PROC2_SIZE;
	debug_log("PDAF start_addr =%x table_size=%d\n", start_addr, block_size);

//proc1
	read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
			LYCHEEF4UWIDE_HVBIN_PDAF_PROC1_ADDR + LYCHEEF4UWIDE_HVBIN_PDAF_PROC1_SIZE,
			1, (unsigned char *)&is_valid);
	if(is_valid != 1) {
		debug_log("[%s]  proc1 unvalid\n", __FUNCTION__);
		err =  CamCalReturnErr[pCamCalData->Command];
		return err;
	}
	read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
			LYCHEEF4UWIDE_HVBIN_PDAF_PROC1_ADDR, LYCHEEF4UWIDE_HVBIN_PDAF_PROC1_SIZE,
			(unsigned char *)&pCamCalData->PDAF.Data[0]);
	if (read_data_size > 0)
		err = CAM_CAL_ERR_NO_ERR;

//proc2
	is_valid = 0;
	read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
			LYCHEEF4UWIDE_HVBIN_PDAF_PROC2_ADDR + LYCHEEF4UWIDE_HVBIN_PDAF_PROC2_SIZE,
			1, (unsigned char *)&is_valid);
	if(is_valid != 1) {
		debug_log("[%s]  proc2 unvalid\n", __FUNCTION__);
		err =  CamCalReturnErr[pCamCalData->Command];
		return err;
	}
	read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
			LYCHEEF4UWIDE_HVBIN_PDAF_PROC2_ADDR, LYCHEEF4UWIDE_HVBIN_PDAF_PROC2_SIZE,
			(unsigned char *)&pCamCalData->PDAF.Data[LYCHEEF4UWIDE_HVBIN_PDAF_PROC1_SIZE]);
	if (read_data_size > 0)
		err = CAM_CAL_ERR_NO_ERR;

	debug_log("======================PDAF Data==================\n");
	debug_log("First five %x, %x, %x, %x, %x\n",
		pCamCalData->PDAF.Data[0],
		pCamCalData->PDAF.Data[1],
		pCamCalData->PDAF.Data[2],
		pCamCalData->PDAF.Data[3],
		pCamCalData->PDAF.Data[4]);
	debug_log("RETURN = 0x%x\n", err);
	debug_log("======================PDAF Data==================\n");
	return err;
}
