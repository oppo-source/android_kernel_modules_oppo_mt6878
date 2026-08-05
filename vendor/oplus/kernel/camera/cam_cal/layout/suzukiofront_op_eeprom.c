// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#define PFX "CAM_CAL_SUZUKIOFRONT"
#define pr_fmt(fmt) PFX "[%s] " fmt, __func__

#include <linux/kernel.h>
#include "cam_cal_list.h"
#include "eeprom_i2c_common_driver.h"
#include "eeprom_i2c_custom_driver.h"
#include "cam_cal_config.h"
#include "oplus_kd_imgsensor.h"

#define READ_4000K 0

static unsigned int do_single_lsc_suzukiofront(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData);
static unsigned int do_2a_gain_suzukiofront(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData);
static unsigned int do_lens_id_suzukiofront(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData);
unsigned int layout_check_suzukiofront(struct EEPROM_DRV_FD_DATA *pdata,
				unsigned int sensorID);

static struct STRUCT_CALIBRATION_LAYOUT_STRUCT cal_layout_table = {
	0x00000006, 0x581c, CAM_CAL_SINGLE_EEPROM_DATA,
	{
		{0x00000001, 0x00000001, 0x00000008, do_module_version},
		{0x00000000, 0x00000000, 0x00000002, do_part_number},
		{0x00000001, 0x00000019, 0x0000074C, do_single_lsc_suzukiofront},
		{0x00000001, 0x0000000B, 0x0000000C, do_2a_gain_suzukiofront},  //Start address, block size is useless
		{0x00000000, 0x00000000, 0x00000000, do_pdaf},
		{0x00000000, 0x00000000, 0x00000000, do_stereo_data},
		{0x00000001, 0x00000000, 0x00002000, do_dump_all},
		{0x00000001, 0x00000007, 0x00000001, do_lens_id_suzukiofront}
	}
};



struct STRUCT_CAM_CAL_CONFIG_STRUCT suzukiofront_op_eeprom = {
	.name = "suzukiofront_op_eeprom",
	.check_layout_function = layout_check_suzukiofront,
	.read_function = Common_read_region,
	.layout = &cal_layout_table,
	.sensor_id = SUZUKIOFRONT_SENSOR_ID,
	.i2c_write_id = 0xA0,
	.max_size = 0x2000,
	.enable_preload = 1,
	.preload_size = 0x2000,
};

unsigned int layout_check_suzukiofront(struct EEPROM_DRV_FD_DATA *pdata,
				unsigned int sensorID)
{
	unsigned int header_offset = suzukiofront_op_eeprom.layout->header_addr;
	unsigned int check_id = 0x00000000;
	unsigned int result = CAM_CAL_ERR_NO_DEVICE;

	if (suzukiofront_op_eeprom.sensor_id == sensorID)
		printk("%s  sensor_id matched\n", suzukiofront_op_eeprom.name);
	else {
		printk("%s  sensor_id not matched,sensorID=0x%x\n", suzukiofront_op_eeprom.name,sensorID);
		return result;
	}

	if (read_data_region(pdata, (u8 *)&check_id, header_offset, 2) != 2) {
		printk(" header_id read failed\n");
		return result;
	}

 	if (check_id == suzukiofront_op_eeprom.layout->header_id) {
 		printk(" header_id matched 0x%08x 0x%08x\n",
 			check_id, suzukiofront_op_eeprom.layout->header_id);
 		result = CAM_CAL_ERR_NO_ERR;
 	} else
 		printk(" header_id not matched 0x%08x 0x%08x\n",
 			check_id, suzukiofront_op_eeprom.layout->header_id);

 	return result;
 }

static unsigned int do_single_lsc_suzukiofront(struct EEPROM_DRV_FD_DATA *pdata,
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

	printk(" lsc table_size %d\n", table_size);
	pCamCalData->SingleLsc.LscTable.MtkLcsData.TableSize = table_size;
	if (table_size > 0) {
		pCamCalData->SingleLsc.TableRotation = 0;
		printk(" u4Offset=%d u4Length=%d", start_addr, table_size);
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
// #ifdef DEBUG_CALIBRATION_LOAD
	printk("======================SingleLsc Data==================\n");
	printk(" [1st] = 0x%x, 0x%x, 0x%x, 0x%x\n",
		pCamCalData->SingleLsc.LscTable.Data[0],
		pCamCalData->SingleLsc.LscTable.Data[1],
		pCamCalData->SingleLsc.LscTable.Data[2],
		pCamCalData->SingleLsc.LscTable.Data[3]);
	printk(" [1st] = SensorLSC(1)?MTKLSC(2)?  %x\n",
		pCamCalData->SingleLsc.LscTable.MtkLcsData.MtkLscType);
	printk(" CapIspReg =0x%x, 0x%x, 0x%x, 0x%x, 0x%x",
		pCamCalData->SingleLsc.LscTable.MtkLcsData.CapIspReg[0],
		pCamCalData->SingleLsc.LscTable.MtkLcsData.CapIspReg[1],
		pCamCalData->SingleLsc.LscTable.MtkLcsData.CapIspReg[2],
		pCamCalData->SingleLsc.LscTable.MtkLcsData.CapIspReg[3],
		pCamCalData->SingleLsc.LscTable.MtkLcsData.CapIspReg[4]);
	printk(" RETURN = 0x%x\n", err);
	printk(" ======================SingleLsc Data==================\n");
// #endif

	return err;
}

static unsigned int do_2a_gain_suzukiofront(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData)
{
	struct STRUCT_CAM_CAL_DATA_STRUCT *pCamCalData =
				(struct STRUCT_CAM_CAL_DATA_STRUCT *)pGetSensorCalData;
	int read_data_size;
	unsigned int err = CamCalReturnErr[pCamCalData->Command];
	u8 rgbGain[12];
	int tempMax = 0;
	int CalR = 1, CalGr = 1, CalGb = 1, CalG = 1, CalB = 1;
	int FacR = 1, FacGr = 1, FacGb = 1, FacG = 1, FacB = 1;
	int rgCalValue = 1, bgCalValue = 1;
	long long CalValue;
	printk(" In %s: sensor_id=%x\n", __func__, pCamCalData->sensorID);
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
	pCamCalData->Single2A.S2aBitEn = 0x03;
	pCamCalData->Single2A.S2aAfBitflagEn = 0x0C;
	printk(" S2aBitEn=0x%02x", pCamCalData->Single2A.S2aBitEn);
	/* AWB Calibration Data*/
	{
		pCamCalData->Single2A.S2aAwb.rGainSetNum = 0x01;
		/* AWB Unit Gain (5000K) */
		printk("5000K AWB\n");

		read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
				0x0767, 4, (unsigned char *)&CalValue);
		if (read_data_size > 0)	{
			printk( "Read CalValue OK\n");
			rgCalValue  = CalValue & 0xFFFF;
			bgCalValue = (CalValue >> 16) & 0xFFFF;
			printk("Light source calibration 5100K value R/G:%d, B/G:%d",rgCalValue, bgCalValue);
			err = CAM_CAL_ERR_NO_ERR;
		} else {
			pCamCalData->Single2A.S2aBitEn = CAM_CAL_NONE_BITEN;
			error_log("Read CalGain Failed\n");
			show_cmd_error_log(pCamCalData->Command);
		}
		read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
				start_addr, 12, (u8*)&rgbGain[0]);
		if (read_data_size > 0)	{
			printk(" Read CalGain OK %d\n", read_data_size);
			CalR  = ((rgbGain[0]<<8) | rgbGain[1]);
            CalB  = ((rgbGain[2]<<8) | rgbGain[3]);
            CalGb = ((rgbGain[4]<<8) | rgbGain[5]);
            CalGr = ((rgbGain[4]<<8) | rgbGain[5]);
            CalG = ((CalGr + CalGb) + 1) >> 1;
            CalR  = CalR * rgCalValue / 1000;
            CalB  = CalB * bgCalValue / 1000;

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
			printk(" UnitR:0x%x, UnitG:0x%x, UnitB:0x%x, New Unit Max=0x%x",
					CalR, CalG, CalB, tempMax);
			err = CAM_CAL_ERR_NO_ERR;
		} else {
			pCamCalData->Single2A.S2aBitEn = CAM_CAL_NONE_BITEN;
			error_log("Read CalGain Failed\n");
			show_cmd_error_log(pCamCalData->Command);
		}
		if (CalR!=0 && CalG!=0 && CalB!=0 ) {
			pCamCalData->Single2A.S2aAwb.rGainSetNum = 1;
			pCamCalData->Single2A.S2aAwb.rUnitGainu4R =
					(unsigned int)((tempMax * 512 + (CalR >> 1)) / CalR);
			pCamCalData->Single2A.S2aAwb.rUnitGainu4G =
					(unsigned int)((tempMax * 512 + (CalG >> 1)) / CalG);
			pCamCalData->Single2A.S2aAwb.rUnitGainu4B =
					(unsigned int)((tempMax * 512 + (CalB >> 1)) / CalB);

			pCamCalData->Single2A.S2aAwb.rGainSetNum = 2;
			pCamCalData->Single2A.S2aAwb.rUnitGainu4R_mid =
					(unsigned int)((tempMax * 512 + (CalR >> 1)) / CalR);
			pCamCalData->Single2A.S2aAwb.rUnitGainu4G_mid =
					(unsigned int)((tempMax * 512 + (CalG >> 1)) / CalG);
			pCamCalData->Single2A.S2aAwb.rUnitGainu4B_mid =
					(unsigned int)((tempMax * 512 + (CalB >> 1)) / CalB);

			pCamCalData->Single2A.S2aAwb.rGainSetNum = 3;
			pCamCalData->Single2A.S2aAwb.rUnitGainu4R_low =
					(unsigned int)((tempMax * 512 + (CalR >> 1)) / CalR);
			pCamCalData->Single2A.S2aAwb.rUnitGainu4G_low =
					(unsigned int)((tempMax * 512 + (CalG >> 1)) / CalG);
			pCamCalData->Single2A.S2aAwb.rUnitGainu4B_low =
					(unsigned int)((tempMax * 512 + (CalB >> 1)) / CalB);
		} else {
			printk(" There are something wrong on EEPROM, plz contact module vendor!!\n");
			printk(" Unit R=0x%x G=0x%x B=0x%x!!\n", CalR, CalG, CalB);
		}

		if (read_data_size > 0)	{
			printk("Read FacGain OK\n");
			FacR  = ((rgbGain[6]<<8) | rgbGain[7]);
            FacB  = ((rgbGain[8]<<8) | rgbGain[9]);
            FacGb = ((rgbGain[10]<<8) | rgbGain[11]);
            FacGr = ((rgbGain[10]<<8) | rgbGain[11]);
            FacG = (((FacGr + FacGb) + 1) >> 1);
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
			printk(" GoldenR:0x%x, GoldenG:0x%x, GoldenB:0x%x, New Golden Max=0x%x",
					FacR, FacG, FacB, tempMax);
			err = CAM_CAL_ERR_NO_ERR;
		} else {
			pCamCalData->Single2A.S2aBitEn = CAM_CAL_NONE_BITEN;
			error_log("Read FacGain Failed\n");
			show_cmd_error_log(pCamCalData->Command);
		}
		if (FacR!=0 && FacG!=0 && FacB!=0) {
			pCamCalData->Single2A.S2aAwb.rGoldGainu4R =
					(unsigned int)((tempMax * 512 + (FacR >> 1)) / FacR);
			pCamCalData->Single2A.S2aAwb.rGoldGainu4G =
					(unsigned int)((tempMax * 512 + (FacG >> 1)) / FacG);
			pCamCalData->Single2A.S2aAwb.rGoldGainu4B =
					(unsigned int)((tempMax * 512 + (FacB >> 1)) / FacB);

			pCamCalData->Single2A.S2aAwb.rGoldGainu4R_mid =
					(unsigned int)((tempMax * 512 + (FacR >> 1)) / FacR);
			pCamCalData->Single2A.S2aAwb.rGoldGainu4G_mid =
					(unsigned int)((tempMax * 512 + (FacG >> 1)) / FacG);
			pCamCalData->Single2A.S2aAwb.rGoldGainu4B_mid =
					(unsigned int)((tempMax * 512 + (FacB >> 1)) / FacB);

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
		/* Set AWB to 3A Layer */
		pCamCalData->Single2A.S2aAwb.rValueR   = CalR;
		pCamCalData->Single2A.S2aAwb.rValueGr  = CalGr;
		pCamCalData->Single2A.S2aAwb.rValueGb  = CalGb;
		pCamCalData->Single2A.S2aAwb.rValueB   = CalB;
		pCamCalData->Single2A.S2aAwb.rGoldenR  = FacR;
		pCamCalData->Single2A.S2aAwb.rGoldenGr = FacGr;
		pCamCalData->Single2A.S2aAwb.rGoldenGb = FacGb;
		pCamCalData->Single2A.S2aAwb.rGoldenB  = FacB;
// #ifdef DEBUG_CALIBRATION_LOAD
		printk(" ======================AWB CAM_CAL==================\n");
		printk(" AWB Calibration @5100K\n");
		printk(" [rCalGain.u4R] = 0x%x\n", pCamCalData->Single2A.S2aAwb.rUnitGainu4R);
		printk(" [rCalGain.u4G] = 0x%x\n", pCamCalData->Single2A.S2aAwb.rUnitGainu4G);
		printk(" [rCalGain.u4B] = 0x%x\n", pCamCalData->Single2A.S2aAwb.rUnitGainu4B);
		printk(" [rFacGain.u4R] = 0x%x\n", pCamCalData->Single2A.S2aAwb.rGoldGainu4R);
		printk(" [rFacGain.u4G] = 0x%x\n", pCamCalData->Single2A.S2aAwb.rGoldGainu4G);
		printk(" [rFacGain.u4B] = 0x%x\n", pCamCalData->Single2A.S2aAwb.rGoldGainu4B);
// #endif
	}
	return err;
}

static unsigned int do_lens_id_suzukiofront(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData)
{
	return do_lens_id_base(pdata, start_addr, block_size, pGetSensorCalData);
}