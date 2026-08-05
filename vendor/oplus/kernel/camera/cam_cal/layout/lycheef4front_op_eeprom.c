// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#define PFX "CAM_CAL_LYCHEEF4FRONT"
#define pr_fmt(fmt) PFX "[%s] " fmt, __func__

#include <linux/kernel.h>
#include "cam_cal_list.h"
#include "eeprom_i2c_common_driver.h"
#include "eeprom_i2c_custom_driver.h"
#include "cam_cal_config.h"
#include "oplus_kd_imgsensor.h"

#define READ_4000K 0

static unsigned int do_single_lsc_lycheef4front(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData);
static unsigned int do_2a_gain_lycheef4front(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData);
static unsigned int do_lens_id_lycheef4front(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData);
static unsigned int do_pdaf_lycheef4front(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData);

static struct STRUCT_CALIBRATION_LAYOUT_STRUCT cal_layout_table = {
	0x00000006, 0x01CA0136, CAM_CAL_SINGLE_EEPROM_DATA,
	{
		{0x00000001, 0x00000000, 0x00000000, do_module_version},
		{0x00000001, 0x00000000, 0x00000002, do_part_number},
		{0x00000001, 0x00000C00, 0x0000074C, do_single_lsc_lycheef4front},
		{0x00000001, 0x00000007, 0x0000000E, do_2a_gain_lycheef4front}, //Start address, block size is useless
		{0x00000001, 0x00001400, 0x000001F0, do_pdaf_lycheef4front},
		{0x00000000, 0x00000FAE, 0x00000550, do_stereo_data},
		{0x00000001, 0x00000000, 0x00004000, do_dump_all},
		{0x00000001, 0x00000008, 0x00000002, do_lens_id_lycheef4front}
	}
};

struct STRUCT_CAM_CAL_CONFIG_STRUCT lycheef4front_op_eeprom = {
	.name = "lycheef4front_op_eeprom",
	.check_layout_function = layout_check,
	.read_function = Common_read_region,
	.layout = &cal_layout_table,
	.sensor_id = LYCHEEF4FRONT_SENSOR_ID,
	.i2c_write_id = 0xA8,
	.max_size = 0x4000,
	.enable_preload = 1,
	.preload_size = 0x4000,
};

#define LYCHEEF4FRONT_HVBIN_PDAF_PROC1_SIZE  (0x15F0 - 0x1400) //496      0x1F0
#define LYCHEEF4FRONT_HVBIN_PDAF_PROC2_SIZE  (0x19EC - 0x1600) //1004     0X3EC
#define LYCHEEF4FRONT_HVBIN_PDAF_PROC1_ADDR  (0x1400)
#define LYCHEEF4FRONT_HVBIN_PDAF_PROC2_ADDR  (0x1600)

//partial PD
/*
#define LYCHEEF4FRONT_PARTIAL_PD_PROC1_SIZE  (0x3590-0x33A0) //496       0X1F0
#define LYCHEEF4FRONT_PARTIAL_PD_PROC2_SIZE  (0x398C-0x35A0) //1004      1500   0x5DC
#define LYCHEEF4FRONT_PARTIAL_PD_PROC1_ADDR  (0x33A0)
#define LYCHEEF4FRONT_PARTIAL_PD_PROC2_ADDR  (0x35A0)
*/
unsigned int do_pdaf_lycheef4front(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData)
{
	struct STRUCT_CAM_CAL_DATA_STRUCT *pCamCalData =
				(struct STRUCT_CAM_CAL_DATA_STRUCT *)pGetSensorCalData;

	int read_data_size;
	int err =  CamCalReturnErr[pCamCalData->Command];
	unsigned char isvalid = 0;
	int bios = 0;
	//bool partial_pd_proc1_flag = false;
	//bool partial_pd_proc2_flag = false;
	bool qpd_proc1_flag = false;
	bool qpd_proc2_flag = false;

	pCamCalData->PDAF.Size_of_PDAF = 0;
	debug_log("======================PDAF Data==================\n");
	debug_log("[%s] QPD proc1 start_addr =%x table_size=%d\n", __FUNCTION__, LYCHEEF4FRONT_HVBIN_PDAF_PROC1_ADDR, LYCHEEF4FRONT_HVBIN_PDAF_PROC1_SIZE);
	debug_log("[%s] QPD proc2 start_addr =%x table_size=%d\n", __FUNCTION__, LYCHEEF4FRONT_HVBIN_PDAF_PROC2_ADDR, LYCHEEF4FRONT_HVBIN_PDAF_PROC2_SIZE);
	//debug_log("[%s] partial PD proc1 start_addr =%x table_size=%d\n", __FUNCTION__, LYCHEEF4FRONT_PARTIAL_PD_PROC1_ADDR, LYCHEEF4FRONT_PARTIAL_PD_PROC1_SIZE);
	//debug_log("[%s] partial PD proc2 start_addr =%x table_size=%d\n", __FUNCTION__, LYCHEEF4FRONT_PARTIAL_PD_PROC2_ADDR, LYCHEEF4FRONT_PARTIAL_PD_PROC2_SIZE);

// QPD
// QPD proc1
	read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
			LYCHEEF4FRONT_HVBIN_PDAF_PROC1_ADDR + LYCHEEF4FRONT_HVBIN_PDAF_PROC1_SIZE,
			1, (unsigned char *)&isvalid);
	if(isvalid != 1) {
		debug_log("[%s] QPD proc1 unvalid\n", __FUNCTION__);
	} else {
		read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
				LYCHEEF4FRONT_HVBIN_PDAF_PROC1_ADDR, LYCHEEF4FRONT_HVBIN_PDAF_PROC1_SIZE,
				(unsigned char *)&pCamCalData->PDAF.Data[bios]);
		if (read_data_size > 0) {
			debug_log("[%s] QDP proc1 First five %x, %x, %x, %x, %x\n",
				__FUNCTION__,
				pCamCalData->PDAF.Data[bios],
				pCamCalData->PDAF.Data[bios + 1],
				pCamCalData->PDAF.Data[bios + 2],
				pCamCalData->PDAF.Data[bios + 3],
				pCamCalData->PDAF.Data[bios + 4]);
			bios += LYCHEEF4FRONT_HVBIN_PDAF_PROC1_SIZE;
			qpd_proc1_flag = true;
			debug_log("[%s] bios = %d  0x%x\n", __FUNCTION__, bios, bios);
		}
	}

// QPD proc2
	if(qpd_proc1_flag) {
		isvalid = 0;
		read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
				LYCHEEF4FRONT_HVBIN_PDAF_PROC2_ADDR + LYCHEEF4FRONT_HVBIN_PDAF_PROC2_SIZE,
				1, (unsigned char *)&isvalid);
		if(isvalid != 1) {
			debug_log("[%s] QPD proc2 unvalid\n", __FUNCTION__);
		}
		read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
				LYCHEEF4FRONT_HVBIN_PDAF_PROC2_ADDR, LYCHEEF4FRONT_HVBIN_PDAF_PROC2_SIZE,
				(unsigned char *)&pCamCalData->PDAF.Data[bios]);
		if (read_data_size > 0) {
			debug_log("[%s] QDP proc2 First five %x, %x, %x, %x, %x\n",
				__FUNCTION__,
				pCamCalData->PDAF.Data[bios],
				pCamCalData->PDAF.Data[bios + 1],
				pCamCalData->PDAF.Data[bios + 2],
				pCamCalData->PDAF.Data[bios + 3],
				pCamCalData->PDAF.Data[bios + 4]);
			bios += LYCHEEF4FRONT_HVBIN_PDAF_PROC2_SIZE;
			qpd_proc2_flag = true;
			debug_log("[%s] bios = %d  0x%x\n", __FUNCTION__, bios, bios);
		}
	}

	if(qpd_proc1_flag && qpd_proc2_flag) {
		pCamCalData->PDAF.Size_of_PDAF = bios;
		err = CAM_CAL_ERR_NO_ERR;
	} else {
		bios = pCamCalData->PDAF.Size_of_PDAF;
		debug_log("[%s] QPD eeprom error", __FUNCTION__);
	}
	if(pCamCalData->PDAF.Size_of_PDAF == 0) {
		err =  CamCalReturnErr[pCamCalData->Command];
	}

	debug_log("[%s] pCamCalData->PDAF.Size_of_PDAF= %d  0x%x\n", __FUNCTION__,
		pCamCalData->PDAF.Size_of_PDAF, pCamCalData->PDAF.Size_of_PDAF);
	debug_log("RETURN = 0x%x\n", err);
	debug_log("======================PDAF Data==================\n");

	return err;
}

static unsigned int do_single_lsc_lycheef4front(struct EEPROM_DRV_FD_DATA *pdata,
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
		pCamCalData->SingleLsc.TableRotation = 0;
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

static unsigned int do_2a_gain_lycheef4front(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData)
{
	struct STRUCT_CAM_CAL_DATA_STRUCT *pCamCalData =
				(struct STRUCT_CAM_CAL_DATA_STRUCT *)pGetSensorCalData;
	int read_data_size;
	unsigned int err = CamCalReturnErr[pCamCalData->Command];

	long long calgain, facgain, calvalue;
	unsigned char awbafconfig = 0xf;

	unsigned short afinf, afmacro, af_50cm;
	int tempmax = 0;
	int cal_r = 1, cal_gr = 1, cal_gb = 1, cal_g = 1, cal_b = 1;
	int fac_r = 1, fac_gr = 1, fac_gb = 1, fac_g = 1, fac_b = 1;
	int rgcalvalue = 1, bgcalvalue = 1;
	unsigned int awb_offset;

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
	pCamCalData->Single2A.S2aBitEn = (0x03 & awbafconfig);
	pCamCalData->Single2A.S2aAfBitflagEn = (0x0C & awbafconfig);
	debug_log("S2aBitEn=0x%02x", pCamCalData->Single2A.S2aBitEn);
	/* AWB Calibration Data*/
	if (0x1 & awbafconfig) {
		pCamCalData->Single2A.S2aAwb.rGainSetNum = 0x02;
		awb_offset = 0x60;
		read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
				awb_offset, 4, (unsigned char *)&calvalue);
		if (read_data_size > 0)	{
			debug_log("Read calvalue OK\n");
			rgcalvalue  = calvalue & 0xFFFF;
			bgcalvalue = (calvalue >> 16) & 0xFFFF;
			debug_log("Light source calibration 5100K value R/G:%d, B/G:%d", rgcalvalue, bgcalvalue);
			err = CAM_CAL_ERR_NO_ERR;
		} else {
			pCamCalData->Single2A.S2aBitEn = CAM_CAL_NONE_BITEN;
			error_log("Read calgain Failed\n");
			show_cmd_error_log(pCamCalData->Command);
		}
		/* AWB Unit Gain (5000K) */
		debug_log("5000K AWB\n");
		awb_offset = 0x20;
		read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
				awb_offset, 8, (unsigned char *)&calgain);
		if (read_data_size > 0)	{
			debug_log("Read calgain OK %x\n", read_data_size);
			cal_r  = calgain & 0xFFFF;
			cal_gr = (calgain >> 16) & 0xFFFF;
			cal_gb = (calgain >> 32) & 0xFFFF;
			cal_g  = ((cal_gr + cal_gb) + 1) >> 1;
			cal_b  = (calgain >> 48) & 0xFFFF;
			debug_log("cal_r:%d, cal_g:%d, cal_b:%d", cal_r, cal_g, cal_b);
			cal_r  = (cal_r * rgcalvalue + 500) / 1000;
			cal_b  = (cal_b * bgcalvalue + 500) / 1000;
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
			error_log("Read calgain Failed\n");
			show_cmd_error_log(pCamCalData->Command);
		}
		if (calgain != 0x0000000000000000 &&
			calgain != 0xFFFFFFFFFFFFFFFF &&
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
		awb_offset = 0x28;
		read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
				awb_offset, 8, (unsigned char *)&facgain);
		if (read_data_size > 0)	{
			debug_log("Read facgain OK\n");
			fac_r  = facgain & 0xFFFF;
			fac_gr = (facgain >> 16) & 0xFFFF;
			fac_gb = (facgain >> 32) & 0xFFFF;
			fac_g  = ((fac_gr + fac_gb) + 1) >> 1;
			fac_b  = (facgain >> 48) & 0xFFFF;
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
			error_log("Read facgain Failed\n");
			show_cmd_error_log(pCamCalData->Command);
		}
		if (facgain != 0x0000000000000000 &&
			facgain != 0xFFFFFFFFFFFFFFFF &&
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
		pr_debug("[calgain] = 0x%x\n", calgain);
		pr_debug("[facgain] = 0x%x\n", facgain);
		pr_debug("[rcalgain.u4R] = %d\n", pCamCalData->Single2A.S2aAwb.rUnitGainu4R);
		pr_debug("[rcalgain.u4G] = %d\n", pCamCalData->Single2A.S2aAwb.rUnitGainu4G);
		pr_debug("[rcalgain.u4B] = %d\n", pCamCalData->Single2A.S2aAwb.rUnitGainu4B);
		pr_debug("[rfacgain.u4R] = %d\n", pCamCalData->Single2A.S2aAwb.rGoldGainu4R);
		pr_debug("[rfacgain.u4G] = %d\n", pCamCalData->Single2A.S2aAwb.rGoldGainu4G);
		pr_debug("[rfacgain.u4B] = %d\n", pCamCalData->Single2A.S2aAwb.rGoldGainu4B);
		#endif
		/* AWB Unit Gain (4000K) */
		#if 0
		#if READ_4000K //zemin.lai@CamTuning delet ,module not support 4000k otp 20220421
		cal_r  = 0;
		cal_gr = 0;
		cal_gb = 0;
		cal_g  = 0;
		cal_b  = 0;
		tempmax = 0;
		debug_log("4000K AWB\n");
		awb_offset = 0x32;
		read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
				awb_offset, 8, (unsigned char *)&calgain);
		if (read_data_size > 0)	{
			debug_log("Read calgain OK %x\n", read_data_size);
			cal_r  = calgain & 0xFFFF;
			cal_gr = (calgain >> 16) & 0xFFFF;
			cal_gb = (calgain >> 32) & 0xFFFF;
			cal_g  = ((cal_gr + cal_gb) + 1) >> 1;
			cal_b  = (calgain >> 48) & 0xFFFF;
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
			error_log("Read calgain Failed\n");
			show_cmd_error_log(pCamCalData->Command);
		}
		if (calgain != 0x0000000000000000 &&
			calgain != 0xFFFFFFFFFFFFFFFF &&
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
		fac_r  = 0;
		fac_gr = 0;
		fac_gb = 0;
		fac_g  = 0;
		fac_b  = 0;
		tempmax = 0;
		awb_offset = 0x3A;
		read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
				awb_offset, 8, (unsigned char *)&facgain);
		if (read_data_size > 0)	{
			debug_log("Read facgain OK\n");
			fac_r  = facgain & 0xFFFF;
			fac_gr = (facgain >> 16) & 0xFFFF;
			fac_gb = (facgain >> 32) & 0xFFFF;
			fac_g  = ((fac_gr + fac_gb) + 1) >> 1;
			fac_b  = (facgain >> 48) & 0xFFFF;
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
			error_log("Read facgain Failed\n");
			show_cmd_error_log(pCamCalData->Command);
		}
		if (facgain != 0x0000000000000000 &&
			facgain != 0xFFFFFFFFFFFFFFFF &&
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
		pr_debug("[calgain] = 0x%x\n", calgain);
		pr_debug("[facgain] = 0x%x\n", facgain);
		pr_debug("[rcalgain.u4R] = %d\n", pCamCalData->Single2A.S2aAwb.rUnitGainu4R_mid);
		pr_debug("[rcalgain.u4G] = %d\n", pCamCalData->Single2A.S2aAwb.rUnitGainu4G_mid);
		pr_debug("[rcalgain.u4B] = %d\n", pCamCalData->Single2A.S2aAwb.rUnitGainu4B_mid);
		pr_debug("[rfacgain.u4R] = %d\n", pCamCalData->Single2A.S2aAwb.rGoldGainu4R_mid);
		pr_debug("[rfacgain.u4G] = %d\n", pCamCalData->Single2A.S2aAwb.rGoldGainu4G_mid);
		pr_debug("[rfacgain.u4B] = %d\n", pCamCalData->Single2A.S2aAwb.rGoldGainu4B_mid);
		#endif
		#endif
		#endif
		/* AWB Unit Gain (3100K) */
		cal_r  = 0;
		cal_gr = 0;
		cal_gb = 0;
		cal_g  = 0;
		cal_b  = 0;
		tempmax = 0;
		debug_log("2850K AWB\n");
		rgcalvalue = 0;
		bgcalvalue = 0;
		awb_offset = 0x6C;
		read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
				awb_offset, 4, (unsigned char *)&calvalue);
		if (read_data_size > 0)	{
			debug_log("Read calvalue OK\n");
			rgcalvalue  = calvalue & 0xFFFF;
			bgcalvalue = (calvalue >> 16) & 0xFFFF;
			debug_log("Light source calibration value 3100 R/G:%d, B/G:%d", rgcalvalue, bgcalvalue);
			err = CAM_CAL_ERR_NO_ERR;
		} else {
			pCamCalData->Single2A.S2aBitEn = CAM_CAL_NONE_BITEN;
			error_log("Read calgain Failed\n");
			show_cmd_error_log(pCamCalData->Command);
		}
		awb_offset = 0x44;
		read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
				awb_offset, 8, (unsigned char *)&calgain);
		if (read_data_size > 0)	{
			debug_log("Read calgain OK %x\n", read_data_size);
			cal_r  = calgain & 0xFFFF;
			cal_gr = (calgain >> 16) & 0xFFFF;
			cal_gb = (calgain >> 32) & 0xFFFF;
			cal_g  = ((cal_gr + cal_gb) + 1) >> 1;
			cal_b  = (calgain >> 48) & 0xFFFF;
			debug_log("cal_r:%d, cal_g:%d, cal_b:%d", cal_r, cal_g, cal_b);
			cal_r  = (cal_r * rgcalvalue + 500) / 1000;
			cal_b  = (cal_b * bgcalvalue + 500) / 1000;
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
			error_log("Read calgain Failed\n");
			show_cmd_error_log(pCamCalData->Command);
		}
		if (calgain != 0x0000000000000000 &&
			calgain != 0xFFFFFFFFFFFFFFFF &&
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
		fac_r  = 0;
		fac_gr = 0;
		fac_gb = 0;
		fac_g  = 0;
		fac_b  = 0;
		tempmax = 0;
		awb_offset = 0x4C;
		read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
				awb_offset, 8, (unsigned char *)&facgain);
		if (read_data_size > 0)	{
			debug_log("Read facgain OK\n");
			fac_r  = facgain & 0xFFFF;
			fac_gr = (facgain >> 16) & 0xFFFF;
			fac_gb = (facgain >> 32) & 0xFFFF;
			fac_g  = ((fac_gr + fac_gb) + 1) >> 1;
			fac_b  = (facgain >> 48) & 0xFFFF;
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
			error_log("Read facgain Failed\n");
			show_cmd_error_log(pCamCalData->Command);
		}
		if (facgain != 0x0000000000000000 &&
			facgain != 0xFFFFFFFFFFFFFFFF &&
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
		pr_debug("[calgain] = 0x%x\n", calgain);
		pr_debug("[facgain] = 0x%x\n", facgain);
		pr_debug("[rcalgain.u4R] = %d\n", pCamCalData->Single2A.S2aAwb.rUnitGainu4R_low);
		pr_debug("[rcalgain.u4G] = %d\n", pCamCalData->Single2A.S2aAwb.rUnitGainu4G_low);
		pr_debug("[rcalgain.u4B] = %d\n", pCamCalData->Single2A.S2aAwb.rUnitGainu4B_low);
		pr_debug("[rfacgain.u4R] = %d\n", pCamCalData->Single2A.S2aAwb.rGoldGainu4R_low);
		pr_debug("[rfacgain.u4G] = %d\n", pCamCalData->Single2A.S2aAwb.rGoldGainu4G_low);
		pr_debug("[rfacgain.u4B] = %d\n", pCamCalData->Single2A.S2aAwb.rGoldGainu4B_low);
		pr_debug("======================AWB CAM_CAL==================\n");
		#endif
	}
	/* AF Calibration Data*/
	if (0x2 & awbafconfig) {
		read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
				0x96, 2, (unsigned char *)&af_50cm);
		if (read_data_size > 0)
			err = CAM_CAL_ERR_NO_ERR;
		else {
			pCamCalData->Single2A.S2aBitEn = CAM_CAL_NONE_BITEN;
			error_log("Read Failed\n");
			show_cmd_error_log(pCamCalData->Command);
		}

		read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
				0x94, 2, (unsigned char *)&afinf);
		if (read_data_size > 0)
			err = CAM_CAL_ERR_NO_ERR;
		else {
			pCamCalData->Single2A.S2aBitEn = CAM_CAL_NONE_BITEN;
			error_log("Read Failed\n");
			show_cmd_error_log(pCamCalData->Command);
		}

		read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
				0x92, 2, (unsigned char *)&afmacro);
		if (read_data_size > 0)
			err = CAM_CAL_ERR_NO_ERR;
		else {
			pCamCalData->Single2A.S2aBitEn = CAM_CAL_NONE_BITEN;
			error_log("Read Failed\n");
			show_cmd_error_log(pCamCalData->Command);
		}

/* 		afinf = afinf >> 2;
		afmacro = afmacro >> 2;
		af_50cm = af_50cm >> 2; */

		pCamCalData->Single2A.S2aAf[0] = afinf;
		pCamCalData->Single2A.S2aAf[1] = afmacro;
		pCamCalData->Single2A.S2aAf[2] = af_50cm;

		////Only AF Gathering <////
		#ifdef DEBUG_CALIBRATION_LOAD
		pr_debug("======================AF CAM_CAL==================\n");
		pr_debug("[afinf] = %d\n", afinf);
		pr_debug("[afmacro] = %d\n", afmacro);
		pr_debug("[af_50cm] = %d\n", af_50cm);
		pr_debug("======================AF CAM_CAL==================\n");
		#endif
	}
	return err;
}

static unsigned int do_lens_id_lycheef4front(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData)
{
	return do_lens_id_base(pdata, start_addr, block_size, pGetSensorCalData);
}
