// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#define PFX "CAM_CAL_GREENLANDMAIN"
#define pr_fmt(fmt) PFX "[%s] " fmt, __func__

#include <linux/kernel.h>
#include "cam_cal_list.h"
#include "eeprom_i2c_common_driver.h"
#include "eeprom_i2c_custom_driver.h"
#include "cam_cal_config.h"
#include "oplus_kd_imgsensor.h"

#define READ_4000K 0

static unsigned int do_single_lsc_greenlandmain(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData);
static unsigned int do_2a_gain_greenlandmain(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData);
static unsigned int do_lens_id_greenlandmain(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData);
static unsigned int do_part_number_greenlandmain(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData);

static struct STRUCT_CALIBRATION_LAYOUT_STRUCT cal_layout_table = {
	0x0000000B, 0x02020202, CAM_CAL_SINGLE_EEPROM_DATA,
	{
		{0x00000000, 0x00000003, 0x00000001, do_module_version},
		{0x00000001, 0x00000774, 0x00000018, do_part_number_greenlandmain},
		{0x00000001, 0x00000026, 0x0000074C, do_single_lsc_greenlandmain},
		{0x00000001, 0x00000014, 0x00000010, do_2a_gain_greenlandmain}, //Start address, block size is useless
		{0x00000000, 0x00000000, 0x0000079F, do_dump_all},
		{0x00000001, 0x0000000D, 0x00000001, do_lens_id_greenlandmain}
	}
};

struct STRUCT_CAM_CAL_CONFIG_STRUCT greenlandmain_op_eeprom = {
	.name = "greenlandmain_op_eeprom",
	.check_layout_function = layout_check,
	.read_function = Common_read_region,
	.layout = &cal_layout_table,
	.sensor_id = GREENLANDMAIN_SENSOR_ID,
	.i2c_write_id = 0xB0,
	.max_size = 0x1000,
	.enable_preload = 1,
	.preload_size = 0x1000,
};

static unsigned int do_single_lsc_greenlandmain(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData)
{
	debug_log("[greenlandmain_debug] do_single_lsc_greenlandmain");
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

static unsigned int do_part_number_greenlandmain(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData)
{
	struct STRUCT_CAM_CAL_DATA_STRUCT *pCamCalData =
				(struct STRUCT_CAM_CAL_DATA_STRUCT *)pGetSensorCalData;
	unsigned int err = CamCalReturnErr[pCamCalData->Command];
	unsigned int size_limit = sizeof(pCamCalData->PartNumber);

	memset(&pCamCalData->PartNumber[0], 0, size_limit);

	if (block_size > size_limit) {
		error_log("part number size can't larger than %u\n", size_limit);
		return err;
	}

	if (read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
			start_addr, block_size, (unsigned char *)&pCamCalData->PartNumber[0]) > 0)
		err = CAM_CAL_ERR_NO_ERR;
	else {
		error_log("Read Failed\n");
		show_cmd_error_log(pCamCalData->Command);
	}

	debug_log("======================Part Number==================\n");
	debug_log("[Part Number] = %x %x %x %x\n",
			pCamCalData->PartNumber[0], pCamCalData->PartNumber[1],
			pCamCalData->PartNumber[2], pCamCalData->PartNumber[3]);
	debug_log("[Part Number] = %x %x %x %x\n",
			pCamCalData->PartNumber[4], pCamCalData->PartNumber[5],
			pCamCalData->PartNumber[6], pCamCalData->PartNumber[7]);
	debug_log("[Part Number] = %x %x %x %x\n",
			pCamCalData->PartNumber[8], pCamCalData->PartNumber[9],
			pCamCalData->PartNumber[10], pCamCalData->PartNumber[11]);
	debug_log("======================Part Number==================\n");

	return err;
}

static unsigned int do_2a_gain_greenlandmain(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData)
{
	debug_log("[greenlandmain_debug] do_2a_gain_greenlandmain");
	struct STRUCT_CAM_CAL_DATA_STRUCT *pCamCalData =
				(struct STRUCT_CAM_CAL_DATA_STRUCT *)pGetSensorCalData;
	int read_data_size;
	unsigned int err = CamCalReturnErr[pCamCalData->Command];

	long long CalGain, FacGain;
	unsigned char AWBAFConfig = 0xf;

	unsigned short AFInf, AFMacro;
	int tempMax = 0;
	int CalR = 1, CalGr = 1, CalGb = 1, CalG = 1, CalB = 1;
	int FacR = 1, FacGr = 1, FacGb = 1, FacG = 1, FacB = 1;
	// int rgCalValue = 1, bgCalValue = 1;
	unsigned int awb_offset;

	(void) start_addr;
	(void) block_size;

	pr_debug("In %s: sensor_id=%x\n", __func__, pCamCalData->sensorID);
	memset((void *)&pCamCalData->Single2A, 0, sizeof(struct STRUCT_CAM_CAL_SINGLE_2A_STRUCT));
	/* Check AWB & AF enable bit */
	pCamCalData->Single2A.S2aVer = 0x01;
	pCamCalData->Single2A.S2aBitEn = (0x03 & AWBAFConfig);
	pCamCalData->Single2A.S2aAfBitflagEn = (0x0C & AWBAFConfig);
	debug_log("S2aBitEn=0x%02x", pCamCalData->Single2A.S2aBitEn);
	/* AWB Calibration Data*/

	pCamCalData->Single2A.S2aAwb.rGainSetNum = 0x02;
	/* AWB Current Gain (5100K) */
	debug_log("5000K AWB\n");
	awb_offset = 0x14;
	read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
			awb_offset, 8, (unsigned char *)&CalGain);
	if (read_data_size > 0)	{
		debug_log("Read CalGain OK %x\n", read_data_size);
		CalR  = CalGain & 0xFFFF;
		CalR  = ((CalR & 0xFF00) >> 8) | ((CalR & 0x00FF) << 8);
		CalGr = (CalGain >> 16) & 0xFFFF;
		CalGr  = ((CalGr & 0xFF00) >> 8) | ((CalGr & 0x00FF) << 8);
		CalGb = (CalGain >> 32) & 0xFFFF;
		CalGb  = ((CalGb & 0xFF00) >> 8) | ((CalGb & 0x00FF) << 8);
		CalG  = ((CalGr + CalGb) + 1) >> 1;
		CalB  = (CalGain >> 48) & 0xFFFF;
		CalB  = ((CalB & 0xFF00) >> 8) | ((CalB & 0x00FF) << 8);
		if (CalR > CalG)
			/* R > G */
			if (CalR > CalB)
				tempMax = CalR;
			else
				tempMax = CalB;
		else
			/* G > R */
			if (CalG > CalB)
				tempMax = CalG;
			else
				tempMax = CalB;
		debug_log("UnitR:%d, UnitG:%d, UnitB:%d, New Unit Max=%d",
				CalR, CalG, CalB, tempMax);
		err = CAM_CAL_ERR_NO_ERR;
	} else {
		pCamCalData->Single2A.S2aBitEn = CAM_CAL_NONE_BITEN;
		error_log("Read CalGain Failed\n");
		show_cmd_error_log(pCamCalData->Command);
	}
	if (CalGain != 0x0000000000000000 &&
		CalGain != 0xFFFFFFFFFFFFFFFF &&
		CalR    != 0x00000000 &&
		CalG    != 0x00000000 &&
		CalB    != 0x00000000) {
		pCamCalData->Single2A.S2aAwb.rGainSetNum = 1;
		pCamCalData->Single2A.S2aAwb.rUnitGainu4R =
				(unsigned int)((tempMax * 512 + (CalR >> 1)) / CalR);
		pCamCalData->Single2A.S2aAwb.rUnitGainu4G =
				(unsigned int)((tempMax * 512 + (CalG >> 1)) / CalG);
		pCamCalData->Single2A.S2aAwb.rUnitGainu4B =
				(unsigned int)((tempMax * 512 + (CalB >> 1)) / CalB);
	} else {
		pr_debug("There are something wrong on EEPROM, plz contact module vendor!!\n");
		pr_debug("Unit R=%d G=%d B=%d!!\n", CalR, CalG, CalB);
	}
	/* AWB Golden Gain (5100K) */
	awb_offset = 0x1C;
	read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
			awb_offset, 8, (unsigned char *)&FacGain);
	if (read_data_size > 0)	{
		debug_log("Read FacGain OK\n");
		FacR  = FacGain & 0xFFFF;
		FacR  = ((FacR & 0xFF00) >> 8) | ((FacR & 0x00FF) << 8);
		FacGr = (FacGain >> 16) & 0xFFFF;
		FacGr  = ((FacGr & 0xFF00) >> 8) | ((FacGr & 0x00FF) << 8);
		FacGb = (FacGain >> 32) & 0xFFFF;
		FacGb  = ((FacGb & 0xFF00) >> 8) | ((FacGb & 0x00FF) << 8);
		FacG  = ((FacGr + FacGb) + 1) >> 1;
		FacB  = (FacGain >> 48) & 0xFFFF;
		FacB  = ((FacB & 0xFF00) >> 8) | ((FacB & 0x00FF) << 8);
		if (FacR > FacG)
			if (FacR > FacB)
				tempMax = FacR;
			else
				tempMax = FacB;
		else
			if (FacG > FacB)
				tempMax = FacG;
			else
				tempMax = FacB;
		debug_log("GoldenR:%d, GoldenG:%d, GoldenB:%d, New Golden Max=%d",
				FacR, FacG, FacB, tempMax);
		err = CAM_CAL_ERR_NO_ERR;
	} else {
		pCamCalData->Single2A.S2aBitEn = CAM_CAL_NONE_BITEN;
		error_log("Read FacGain Failed\n");
		show_cmd_error_log(pCamCalData->Command);
	}
	if (FacGain != 0x0000000000000000 &&
		FacGain != 0xFFFFFFFFFFFFFFFF &&
		FacR    != 0x00000000 &&
		FacG    != 0x00000000 &&
		FacB    != 0x00000000)	{
		pCamCalData->Single2A.S2aAwb.rGoldGainu4R =
				(unsigned int)((tempMax * 512 + (FacR >> 1)) / FacR);
		pCamCalData->Single2A.S2aAwb.rGoldGainu4G =
				(unsigned int)((tempMax * 512 + (FacG >> 1)) / FacG);
		pCamCalData->Single2A.S2aAwb.rGoldGainu4B =
				(unsigned int)((tempMax * 512 + (FacB >> 1)) / FacB);
	} else {
		pr_debug("There are something wrong on EEPROM, plz contact module vendor!!");
		pr_debug("Golden R=%d G=%d B=%d\n", FacR, FacG, FacB);
	}
	/* Set AWB to 3A Layer */
	pCamCalData->Single2A.S2aAwb.rValueR   = CalR;
	pCamCalData->Single2A.S2aAwb.rValueGr  = CalGr;
	pCamCalData->Single2A.S2aAwb.rValueGb  = CalGb;
	pCamCalData->Single2A.S2aAwb.rValueB   = CalB;
	pCamCalData->Single2A.S2aAwb.rGoldenR  = FacR;
	pCamCalData->Single2A.S2aAwb.rGoldenGr = FacGr;
	pCamCalData->Single2A.S2aAwb.rGoldenGb = FacGb;
	pCamCalData->Single2A.S2aAwb.rGoldenB  = FacB;
	#ifdef DEBUG_CALIBRATION_LOAD
	pr_debug("======================AWB CAM_CAL==================\n");
	pr_debug("AWB Calibration @5100K\n");
	pr_debug("[CalGain] = 0x%x\n", CalGain);
	pr_debug("[FacGain] = 0x%x\n", FacGain);
	pr_debug("[rCalGain.u4R] = %d\n", pCamCalData->Single2A.S2aAwb.rUnitGainu4R);
	pr_debug("[rCalGain.u4G] = %d\n", pCamCalData->Single2A.S2aAwb.rUnitGainu4G);
	pr_debug("[rCalGain.u4B] = %d\n", pCamCalData->Single2A.S2aAwb.rUnitGainu4B);
	pr_debug("[rFacGain.u4R] = %d\n", pCamCalData->Single2A.S2aAwb.rGoldGainu4R);
	pr_debug("[rFacGain.u4G] = %d\n", pCamCalData->Single2A.S2aAwb.rGoldGainu4G);
	pr_debug("[rFacGain.u4B] = %d\n", pCamCalData->Single2A.S2aAwb.rGoldGainu4B);
	#endif
	/* AWB Current Gain (4100K) */
	debug_log("4100K AWB\n");
	awb_offset = 0x14;
	read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
			awb_offset, 8, (unsigned char *)&CalGain);
	if (read_data_size > 0)	{
		debug_log("Read CalGain OK %x\n", read_data_size);
		CalR  = CalGain & 0xFFFF;
		CalR  = ((CalR & 0xFF00) >> 8) | ((CalR & 0x00FF) << 8);
		CalGr = (CalGain >> 16) & 0xFFFF;
		CalGr  = ((CalGr & 0xFF00) >> 8) | ((CalGr & 0x00FF) << 8);
		CalGb = (CalGain >> 32) & 0xFFFF;
		CalGb  = ((CalGb & 0xFF00) >> 8) | ((CalGb & 0x00FF) << 8);
		CalG  = ((CalGr + CalGb) + 1) >> 1;
		CalB  = (CalGain >> 48) & 0xFFFF;
		CalB  = ((CalB & 0xFF00) >> 8) | ((CalB & 0x00FF) << 8);
		if (CalR > CalG)
			/* R > G */
			if (CalR > CalB)
				tempMax = CalR;
			else
				tempMax = CalB;
		else
			/* G > R */
			if (CalG > CalB)
				tempMax = CalG;
			else
				tempMax = CalB;
		debug_log("UnitR:%d, UnitG:%d, UnitB:%d, New Unit Max=%d",
				CalR, CalG, CalB, tempMax);
		err = CAM_CAL_ERR_NO_ERR;
	} else {
		pCamCalData->Single2A.S2aBitEn = CAM_CAL_NONE_BITEN;
		error_log("Read CalGain Failed\n");
		show_cmd_error_log(pCamCalData->Command);
	}
	if (CalGain != 0x0000000000000000 &&
		CalGain != 0xFFFFFFFFFFFFFFFF &&
		CalR    != 0x00000000 &&
		CalG    != 0x00000000 &&
		CalB    != 0x00000000) {
		pCamCalData->Single2A.S2aAwb.rGainSetNum = 2;
		pCamCalData->Single2A.S2aAwb.rUnitGainu4R_mid =
				(unsigned int)((tempMax * 512 + (CalR >> 1)) / CalR);
		pCamCalData->Single2A.S2aAwb.rUnitGainu4G_mid =
				(unsigned int)((tempMax * 512 + (CalG >> 1)) / CalG);
		pCamCalData->Single2A.S2aAwb.rUnitGainu4B_mid =
				(unsigned int)((tempMax * 512 + (CalB >> 1)) / CalB);
	} else {
		pr_debug("There are something wrong on EEPROM, plz contact module vendor!!\n");
		pr_debug("Unit R=%d G=%d B=%d!!\n", CalR, CalG, CalB);
	}
	/* AWB Golden Gain (4100K) */
	awb_offset = 0x1C;
	read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
			awb_offset, 8, (unsigned char *)&FacGain);
	if (read_data_size > 0)	{
		debug_log("Read FacGain OK\n");
		FacR  = FacGain & 0xFFFF;
		FacR  = ((FacR & 0xFF00) >> 8) | ((FacR & 0x00FF) << 8);
		FacGr = (FacGain >> 16) & 0xFFFF;
		FacGr  = ((FacGr & 0xFF00) >> 8) | ((FacGr & 0x00FF) << 8);
		FacGb = (FacGain >> 32) & 0xFFFF;
		FacGb  = ((FacGb & 0xFF00) >> 8) | ((FacGb & 0x00FF) << 8);
		FacG  = ((FacGr + FacGb) + 1) >> 1;
		FacB  = (FacGain >> 48) & 0xFFFF;
		FacB  = ((FacB & 0xFF00) >> 8) | ((FacB & 0x00FF) << 8);
		if (FacR > FacG)
			if (FacR > FacB)
				tempMax = FacR;
			else
				tempMax = FacB;
		else
			if (FacG > FacB)
				tempMax = FacG;
			else
				tempMax = FacB;
		debug_log("GoldenR:%d, GoldenG:%d, GoldenB:%d, New Golden Max=%d",
				FacR, FacG, FacB, tempMax);
		err = CAM_CAL_ERR_NO_ERR;
	} else {
		pCamCalData->Single2A.S2aBitEn = CAM_CAL_NONE_BITEN;
		error_log("Read FacGain Failed\n");
		show_cmd_error_log(pCamCalData->Command);
	}
	if (FacGain != 0x0000000000000000 &&
		FacGain != 0xFFFFFFFFFFFFFFFF &&
		FacR    != 0x00000000 &&
		FacG    != 0x00000000 &&
		FacB    != 0x00000000)	{
		pCamCalData->Single2A.S2aAwb.rGoldGainu4R_mid =
				(unsigned int)((tempMax * 512 + (FacR >> 1)) / FacR);
		pCamCalData->Single2A.S2aAwb.rGoldGainu4G_mid =
				(unsigned int)((tempMax * 512 + (FacG >> 1)) / FacG);
		pCamCalData->Single2A.S2aAwb.rGoldGainu4B_mid =
				(unsigned int)((tempMax * 512 + (FacB >> 1)) / FacB);
	} else {
		pr_debug("There are something wrong on EEPROM, plz contact module vendor!!");
		pr_debug("Golden R=%d G=%d B=%d\n", FacR, FacG, FacB);
	}
	/* AWB Current Gain (3000K) */
	debug_log("3000K AWB\n");
	awb_offset = 0x14;
	read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
			awb_offset, 8, (unsigned char *)&CalGain);
	if (read_data_size > 0)	{
		debug_log("Read CalGain OK %x\n", read_data_size);
		CalR  = CalGain & 0xFFFF;
		CalR  = ((CalR & 0xFF00) >> 8) | ((CalR & 0x00FF) << 8);
		CalGr = (CalGain >> 16) & 0xFFFF;
		CalGr  = ((CalGr & 0xFF00) >> 8) | ((CalGr & 0x00FF) << 8);
		CalGb = (CalGain >> 32) & 0xFFFF;
		CalGb  = ((CalGb & 0xFF00) >> 8) | ((CalGb & 0x00FF) << 8);
		CalG  = ((CalGr + CalGb) + 1) >> 1;
		CalB  = (CalGain >> 48) & 0xFFFF;
		CalB  = ((CalB & 0xFF00) >> 8) | ((CalB & 0x00FF) << 8);
		if (CalR > CalG)
			/* R > G */
			if (CalR > CalB)
				tempMax = CalR;
			else
				tempMax = CalB;
		else
			/* G > R */
			if (CalG > CalB)
				tempMax = CalG;
			else
				tempMax = CalB;
		debug_log("UnitR:%d, UnitG:%d, UnitB:%d, New Unit Max=%d",
				CalR, CalG, CalB, tempMax);
		err = CAM_CAL_ERR_NO_ERR;
	} else {
		pCamCalData->Single2A.S2aBitEn = CAM_CAL_NONE_BITEN;
		error_log("Read CalGain Failed\n");
		show_cmd_error_log(pCamCalData->Command);
	}
	if (CalGain != 0x0000000000000000 &&
		CalGain != 0xFFFFFFFFFFFFFFFF &&
		CalR    != 0x00000000 &&
		CalG    != 0x00000000 &&
		CalB    != 0x00000000) {
		pCamCalData->Single2A.S2aAwb.rGainSetNum = 3;
		pCamCalData->Single2A.S2aAwb.rUnitGainu4R_low =
				(unsigned int)((tempMax * 512 + (CalR >> 1)) / CalR);
		pCamCalData->Single2A.S2aAwb.rUnitGainu4G_low =
				(unsigned int)((tempMax * 512 + (CalG >> 1)) / CalG);
		pCamCalData->Single2A.S2aAwb.rUnitGainu4B_low =
				(unsigned int)((tempMax * 512 + (CalB >> 1)) / CalB);
	} else {
		pr_debug("There are something wrong on EEPROM, plz contact module vendor!!\n");
		pr_debug("Unit R=%d G=%d B=%d!!\n", CalR, CalG, CalB);
	}
	/* AWB Golden Gain (3000K) */
	awb_offset = 0x1C;
	read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
			awb_offset, 8, (unsigned char *)&FacGain);
	if (read_data_size > 0)	{
		debug_log("Read FacGain OK\n");
		FacR  = FacGain & 0xFFFF;
		FacR  = ((FacR & 0xFF00) >> 8) | ((FacR & 0x00FF) << 8);
		FacGr = (FacGain >> 16) & 0xFFFF;
		FacGr  = ((FacGr & 0xFF00) >> 8) | ((FacGr & 0x00FF) << 8);
		FacGb = (FacGain >> 32) & 0xFFFF;
		FacGb  = ((FacGb & 0xFF00) >> 8) | ((FacGb & 0x00FF) << 8);
		FacG  = ((FacGr + FacGb) + 1) >> 1;
		FacB  = (FacGain >> 48) & 0xFFFF;
		FacB  = ((FacB & 0xFF00) >> 8) | ((FacB & 0x00FF) << 8);
		if (FacR > FacG)
			if (FacR > FacB)
				tempMax = FacR;
			else
				tempMax = FacB;
		else
			if (FacG > FacB)
				tempMax = FacG;
			else
				tempMax = FacB;
		debug_log("GoldenR:%d, GoldenG:%d, GoldenB:%d, New Golden Max=%d",
				FacR, FacG, FacB, tempMax);
		err = CAM_CAL_ERR_NO_ERR;
	} else {
		pCamCalData->Single2A.S2aBitEn = CAM_CAL_NONE_BITEN;
		error_log("Read FacGain Failed\n");
		show_cmd_error_log(pCamCalData->Command);
	}
	if (FacGain != 0x0000000000000000 &&
		FacGain != 0xFFFFFFFFFFFFFFFF &&
		FacR    != 0x00000000 &&
		FacG    != 0x00000000 &&
		FacB    != 0x00000000)	{
		pCamCalData->Single2A.S2aAwb.rGoldGainu4R_low =
				(unsigned int)((tempMax * 512 + (FacR >> 1)) / FacR);
		pCamCalData->Single2A.S2aAwb.rGoldGainu4G_low =
				(unsigned int)((tempMax * 512 + (FacG >> 1)) / FacG);
		pCamCalData->Single2A.S2aAwb.rGoldGainu4B_low =
				(unsigned int)((tempMax * 512 + (FacB >> 1)) / FacB);
	} else {
		pr_debug("There are something wrong on EEPROM, plz contact module vendor!!");
		pr_debug("Golden R=%d G=%d B=%d\n", FacR, FacG, FacB);
	}

	/* AF Calibration Data*/
	read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
			0x793, 2, (unsigned char *)&AFInf);
	AFInf = ((AFInf & 0xFF00) >> 8) | ((AFInf & 0x00FF) << 8);
	if (read_data_size > 0)
		err = CAM_CAL_ERR_NO_ERR;
	else {
		pCamCalData->Single2A.S2aBitEn = CAM_CAL_NONE_BITEN;
		error_log("Read Failed\n");
		show_cmd_error_log(pCamCalData->Command);
	}

	read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
			0x791, 2, (unsigned char *)&AFMacro);
	AFMacro = ((AFMacro & 0xFF00) >> 8) | ((AFMacro & 0x00FF) << 8);
	if (read_data_size > 0)
		err = CAM_CAL_ERR_NO_ERR;
	else {
		pCamCalData->Single2A.S2aBitEn = CAM_CAL_NONE_BITEN;
		error_log("Read Failed\n");
		show_cmd_error_log(pCamCalData->Command);
	}

	pCamCalData->Single2A.S2aAf[0] = AFInf;
	pCamCalData->Single2A.S2aAf[1] = AFMacro;

	////Only AF Gathering <////
	pr_debug("======================AF CAM_CAL==================\n");
	pr_debug("[AFInf] = %d\n", AFInf);
	pr_debug("[AFMacro] = %d\n", AFMacro);
	pr_debug("======================AF CAM_CAL==================\n");

	return err;
}

static unsigned int do_lens_id_greenlandmain(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData)
{
	debug_log("[greenlandmain_debug] do_lens_id_greenlandmain");
	return do_lens_id_base(pdata, start_addr, block_size, pGetSensorCalData);
}
