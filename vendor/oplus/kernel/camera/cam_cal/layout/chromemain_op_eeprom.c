// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#define PFX "CAM_CAL_CHROMEMAIN"
#define pr_fmt(fmt) PFX "[%s] " fmt, __func__

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include "cam_cal_list.h"
#include "eeprom_i2c_common_driver.h"
#include "eeprom_i2c_custom_driver.h"
#include "cam_cal_config.h"
#include "oplus_kd_imgsensor.h"

#define OTP_I2C_ADDR 0x20
#define CHROMEMAIN_OTP_RET_FAIL -1
#define CHROMEMAIN_OTP_RET_SUCCESS 0
#define CHROMEMAIN_TXD_main_OTP_MODULE_LENS 23
#define CHROMEMAIN_TXD_main_OTP_AWB_LENS 20
#define CHROMEMAIN_TXD_main_OTP_AF_LENS 4
#define CHROMEMAIN_TXD_main_OTP_LSC_LENS 1868



//flag addr
#define CHROMEMAIN_OTP_MODULE_FLAGADDR 0x827A
#define CHROMEMAIN_GROUP1_FLAG 0x01
#define CHROMEMAIN_GROUP2_FLAG 0x13
#define CHROMEMAIN_INVALID_FLAG 0x0

//data start addr
#define CHROMEMAIN_OTP_MODULE_GROUP1_STARTADDR 0x827B
#define CHROMEMAIN_OTP_AWB_GROUP1_STARTADDR 0x82AC
#define CHROMEMAIN_OTP_AF_GROUP1_STARTADDR 0x82D7


#define CHROMEMAIN_OTP_MODULE_GROUP2_STARTADDR 0x8293
#define CHROMEMAIN_OTP_AWB_GROUP2_STARTADDR 0x82C1
#define CHROMEMAIN_OTP_AF_GROUP2_STARTADDR 0x82DC

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
	unsigned char ModuleFlag;
	unsigned char module_info[CHROMEMAIN_TXD_main_OTP_MODULE_LENS];
	unsigned char awb_data[CHROMEMAIN_TXD_main_OTP_AWB_LENS];
	unsigned char af_data[CHROMEMAIN_TXD_main_OTP_AF_LENS];
	unsigned char lsc_data[CHROMEMAIN_TXD_main_OTP_LSC_LENS];
};

struct chromemain_txd_main_otp_struct chromemain_txd_main_otp = {
	.ModuleFlag = 0
};
EXPORT_SYMBOL(chromemain_txd_main_otp);

int chromemain_i2c_rd_u8(struct i2c_client *i2c_client,
		u16 addr, u16 reg, u8 *val)
{
	int ret;
	u8 buf[2];
	struct i2c_msg msg[2];

	if (i2c_client == NULL)
		return -ENODEV;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	msg[0].addr = addr;
	msg[0].flags = i2c_client->flags;
	msg[0].buf = buf;
	msg[0].len = sizeof(buf);

	msg[1].addr = addr;
	msg[1].flags = i2c_client->flags | I2C_M_RD;
	msg[1].buf = buf;
	msg[1].len = 1;

	ret = i2c_transfer(i2c_client->adapter, msg, 2);
	if (ret < 0) {
		dev_info(&i2c_client->dev, "i2c transfer failed (%d), r - reg = 0x%04x, val = 0x%02x\n", ret, reg, *val);
		return ret;
	}

	*val = buf[0];

	return 0;
}

int chromemain_i2c_wr_u8(struct i2c_client *i2c_client,
		u16 addr, u16 reg, u8 val)
{
	int ret;
	u8 buf[3];
	struct i2c_msg msg;

	if (i2c_client == NULL)
		return -ENODEV;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;
	buf[2] = val;

	msg.addr = addr;
	msg.flags = i2c_client->flags;
	msg.buf = buf;
	msg.len = sizeof(buf);

	ret = i2c_transfer(i2c_client->adapter, &msg, 1);
	if (ret < 0)
		dev_info(&i2c_client->dev, "i2c transfer failed (%d), w - reg = 0x%04x, val = 0x%02x\n", ret, reg, val);

	return ret;
}

#define subdrv_i2c_rd_u8(i2c_client, reg) \
({ \
	u8 __val = 0xff; \
	chromemain_i2c_rd_u8(i2c_client, \
		i2c_client->addr >> 1, reg, &__val); \
	__val; \
})

#define subdrv_i2c_wr_u8(i2c_client, reg, val) \
	chromemain_i2c_wr_u8(i2c_client, \
		i2c_client->addr >> 1, reg, val)

static unsigned int do_part_number_chromemain(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData);
static unsigned int do_single_lsc_chromemain(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData);
static unsigned int do_2a_gain_chromemain(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData);
static unsigned int do_lens_id_chromemain(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData);
static unsigned int layout_check_chromemain(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int sensorID);

static struct STRUCT_CALIBRATION_LAYOUT_STRUCT cal_layout_table = {
	0x00000004, 0x00215319, CAM_CAL_SINGLE_EEPROM_DATA,
	{
		{0x00000001, 0x00000000, 0x00000000, do_module_version},
		{0x00000001, 0x00000000, 0x00000002, do_part_number_chromemain},
		{0x00000001, 0x00000530, 0x0000074C, do_single_lsc_chromemain},
		{0x00000001, 0x00000007, 0x0000000E, do_2a_gain_chromemain}, //Start address, block size is useless
		{0x00000000, 0x00000000, 0x00000000, do_pdaf},
		{0x00000000, 0x00000000, 0x00000000, do_stereo_data},
		{0x00000001, 0x00000000, 0x00002000, do_dump_all},
		{0x00000001, 0x00000008, 0x00000002, do_lens_id_chromemain}
	}
};

struct STRUCT_CAM_CAL_CONFIG_STRUCT chromemain_op_eeprom = {
	.name = "chromemain_op_eeprom",
	.check_layout_function = layout_check_chromemain,
	.read_function = Common_read_region,
	.layout = &cal_layout_table,
	.sensor_id = CHROMEMAIN_SENSOR_ID,
	.i2c_write_id = OTP_I2C_ADDR,
	.max_size = 0x1FFF,
	.enable_preload = 1,
	.preload_size = 0x1FFF,
};
static struct STRUCT_CAM_CAL_CONFIG_STRUCT *cam_cal_config = &chromemain_op_eeprom;

static unsigned int do_part_number_chromemain(struct EEPROM_DRV_FD_DATA *pdata,
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

	memcpy(&pCamCalData->PartNumber[0], &chromemain_txd_main_otp.module_info[0], block_size);
	debug_log("partNumber[0] = 0x%x, partNumber[1] = 0x%x\n", pCamCalData->PartNumber[0], pCamCalData->PartNumber[1]);

	return CAM_CAL_ERR_NO_ERR;
}

int memcpy_test(unsigned char *dsc, unsigned char *src, int length)
{
	memcpy(dsc, src, length);
	return 0;
}

static unsigned int do_single_lsc_chromemain(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData)
{
	struct STRUCT_CAM_CAL_DATA_STRUCT *pCamCalData =
				(struct STRUCT_CAM_CAL_DATA_STRUCT *)pGetSensorCalData;

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

	table_size = block_size;

	debug_log("lsc table_size %d\n", table_size);
	pCamCalData->SingleLsc.LscTable.MtkLcsData.TableSize = table_size;
	if (table_size > 0) {
		pCamCalData->SingleLsc.TableRotation = 2;
		debug_log("u4Offset=%d u4Length=%d", start_addr, table_size);
		memcpy_test(&pCamCalData->SingleLsc.LscTable.Data[4], &chromemain_txd_main_otp.lsc_data[0], block_size);
		err = CAM_CAL_ERR_NO_ERR;
	}
	#ifdef DEBUG_CALIBRATION_LOAD
	debug_log("======================SingleLsc Data==================\n");
	debug_log("[1st] = %x, %x, %x, %x\n",
		pCamCalData->SingleLsc.LscTable.Data[0],
		pCamCalData->SingleLsc.LscTable.Data[1],
		pCamCalData->SingleLsc.LscTable.Data[2],
		pCamCalData->SingleLsc.LscTable.Data[3]);
	debug_log("[1st] = SensorLSC(1)?MTKLSC(2)?  %x\n",
		pCamCalData->SingleLsc.LscTable.MtkLcsData.MtkLscType);
	debug_log("CapIspReg =0x%x, 0x%x, 0x%x, 0x%x, 0x%x",
		pCamCalData->SingleLsc.LscTable.MtkLcsData.CapIspReg[0],
		pCamCalData->SingleLsc.LscTable.MtkLcsData.CapIspReg[1],
		pCamCalData->SingleLsc.LscTable.MtkLcsData.CapIspReg[2],
		pCamCalData->SingleLsc.LscTable.MtkLcsData.CapIspReg[3],
		pCamCalData->SingleLsc.LscTable.MtkLcsData.CapIspReg[4]);
	debug_log("RETURN = 0x%x\n", err);
	debug_log("======================SingleLsc Data==================\n");
	#endif

	return err;
}


#define MAX(a, b, c) ((a > b) ? ((a > c) ? a : c) : ((b > c) ? b : c))

static unsigned int do_2a_gain_chromemain(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData)
{
	struct STRUCT_CAM_CAL_DATA_STRUCT *pCamCalData =
				(struct STRUCT_CAM_CAL_DATA_STRUCT *)pGetSensorCalData;
	unsigned int err = CamCalReturnErr[pCamCalData->Command];

	unsigned char AWBAFConfig = 0xf;
	unsigned char awbdata[12];

	int tempMax = 0;
	int CalR = 1, CalGr = 1, CalGb = 1, CalG = 1, CalB = 1;
	int FacR = 1, FacGr = 1, FacGb = 1, FacG = 1, FacB = 1;
	int CalGb2Gr = 1, CalR2G = 1, CalB2G = 1;
	int FacGb2Gr = 1, FacR2G = 1, FacB2G = 1;

	unsigned short AFInf = 0, AFMacro = 0;

	(void) start_addr;
	(void) block_size;

	debug_log("In %s: sensor_id=%x\n", __func__, pCamCalData->sensorID);
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
	pCamCalData->Single2A.S2aBitEn = (0x03 & AWBAFConfig);
	pCamCalData->Single2A.S2aAfBitflagEn = (0x0C & AWBAFConfig);
	debug_log("S2aBitEn=0x%02x", pCamCalData->Single2A.S2aBitEn);

	err = CAM_CAL_ERR_NO_ERR;

	/* AWB Calibration Data*/
	if (0x1 & AWBAFConfig) {
		//pCamCalData->Single2A.S2aAwb.rGainSetNum = 0x02;

		memcpy(&awbdata[0], &chromemain_txd_main_otp.awb_data[4], 6);
		memcpy(&awbdata[6], &chromemain_txd_main_otp.awb_data[14], 6);

		debug_log("Read UINT AWB\n");

		CalR2G   = ((awbdata[0]<<8)|awbdata[1]);
		CalB2G   = ((awbdata[2]<<8)|awbdata[3]);
		CalGb2Gr = ((awbdata[4]<<8)|awbdata[5]);

		CalGr = 255;
		CalGb = (CalGb2Gr * CalGr)/1024;

		CalG  = ((CalGr + CalGb)*10000) >> 1;
		CalR  = (CalR2G * CalG)/1024;
		CalB  = (CalB2G * CalG)/1024;

        tempMax = MAX(CalR, CalB, CalG);
		debug_log("UnitR:%d, UnitG:%d, UnitB:%d, New Unit Max=%d", CalR, CalG, CalB, tempMax);
		err = CAM_CAL_ERR_NO_ERR;

		if (CalR != 0 && CalG != 0 && CalB != 0) {
			pCamCalData->Single2A.S2aAwb.rGainSetNum = 1;
			pCamCalData->Single2A.S2aAwb.rUnitGainu4R = (unsigned int)((tempMax * 512 + (CalR >> 1)) / CalR);
			pCamCalData->Single2A.S2aAwb.rUnitGainu4G = (unsigned int)((tempMax * 512 + (CalG >> 1)) / CalG);
			pCamCalData->Single2A.S2aAwb.rUnitGainu4B = (unsigned int)((tempMax * 512 + (CalB >> 1)) / CalB);

			pCamCalData->Single2A.S2aAwb.rGainSetNum++;
			pCamCalData->Single2A.S2aAwb.rUnitGainu4R_low = (unsigned int)((tempMax * 512 + (CalR >> 1)) / CalR);
			pCamCalData->Single2A.S2aAwb.rUnitGainu4G_low = (unsigned int)((tempMax * 512 + (CalG >> 1)) / CalG);
			pCamCalData->Single2A.S2aAwb.rUnitGainu4B_low = (unsigned int)((tempMax * 512 + (CalB >> 1)) / CalB);
		} else {
			error_log("There are something wrong on EEPROM, plz contact module vendor!!\n");
			error_log("Unit R=%d G=%d B=%d!!\n", CalR, CalG, CalB);
		}

		debug_log("Read Golden AWB\n");

		FacR2G   = ((awbdata[6]<<8)|awbdata[7]);
		FacB2G   = ((awbdata[8]<<8)|awbdata[9]);
		FacGb2Gr = ((awbdata[10]<<8)|awbdata[11]);

		FacGr = 255;
		FacGb = (FacGb2Gr * FacGr)/1024;

		FacG  = ((FacGr + FacGb)*10000) >> 1;
		FacR  = (FacR2G * FacG)/1024;
		FacB  = (FacB2G * FacG)/1024;

        tempMax = MAX(FacR, FacB, FacG);
		debug_log("GoldenR:%d, GoldenG:%d, GoldenB:%d, New Golden Max=%d", FacR, FacG, FacB, tempMax);
		err = CAM_CAL_ERR_NO_ERR;

		if (FacR != 0 && FacG != 0 && FacB != 0)	{
			pCamCalData->Single2A.S2aAwb.rGoldGainu4R = (unsigned int)((tempMax * 512 + (FacR >> 1)) / FacR);
			pCamCalData->Single2A.S2aAwb.rGoldGainu4G = (unsigned int)((tempMax * 512 + (FacG >> 1)) / FacG);
			pCamCalData->Single2A.S2aAwb.rGoldGainu4B = (unsigned int)((tempMax * 512 + (FacB >> 1)) / FacB);

			pCamCalData->Single2A.S2aAwb.rGoldGainu4R_low= (unsigned int)((tempMax * 512 + (FacR >> 1)) / FacR);
			pCamCalData->Single2A.S2aAwb.rGoldGainu4G_low = (unsigned int)((tempMax * 512 + (FacG >> 1)) / FacG);
			pCamCalData->Single2A.S2aAwb.rGoldGainu4B_low = (unsigned int)((tempMax * 512 + (FacB >> 1)) / FacB);
		} else {
			error_log("There are something wrong on EEPROM, plz contact module vendor!!");
			error_log("Golden R=%d G=%d B=%d\n", FacR, FacG, FacB);
		}

		/* Set AWB to 3A Layer */
		pCamCalData->Single2A.S2aAwb.rValueR   = CalR/10000;
		pCamCalData->Single2A.S2aAwb.rValueGr  = CalGr;
		pCamCalData->Single2A.S2aAwb.rValueGb  = CalGb;
		pCamCalData->Single2A.S2aAwb.rValueB   = CalB/10000;
		pCamCalData->Single2A.S2aAwb.rGoldenR  = FacR/10000;
		pCamCalData->Single2A.S2aAwb.rGoldenGr = FacGr;
		pCamCalData->Single2A.S2aAwb.rGoldenGb = FacGb;
		pCamCalData->Single2A.S2aAwb.rGoldenB  = FacB/10000;

		#ifdef DEBUG_CALIBRATION_LOAD
		debug_log("======================AWB CAM_CAL==================\n");
		debug_log("[rCalGain.u4R] = %d\n", pCamCalData->Single2A.S2aAwb.rUnitGainu4R);
		debug_log("[rCalGain.u4G] = %d\n", pCamCalData->Single2A.S2aAwb.rUnitGainu4G);
		debug_log("[rCalGain.u4B] = %d\n", pCamCalData->Single2A.S2aAwb.rUnitGainu4B);
		debug_log("[rFacGain.u4R] = %d\n", pCamCalData->Single2A.S2aAwb.rGoldGainu4R);
		debug_log("[rFacGain.u4G] = %d\n", pCamCalData->Single2A.S2aAwb.rGoldGainu4G);
		debug_log("[rFacGain.u4B] = %d\n", pCamCalData->Single2A.S2aAwb.rGoldGainu4B);
		#endif
	}

	if (0x2 & AWBAFConfig) {
		AFMacro = chromemain_txd_main_otp.af_data[0] << 8 | chromemain_txd_main_otp.af_data[1];
		AFInf = chromemain_txd_main_otp.af_data[2] << 8 | chromemain_txd_main_otp.af_data[3];
		pCamCalData->Single2A.S2aAf[0] = AFInf;
		pCamCalData->Single2A.S2aAf[1] = AFMacro;

		#ifdef DEBUG_CALIBRATION_LOAD
		debug_log("======================AF CAM_CAL==================\n");
		debug_log("[AFInf] = %d\n", AFInf);
		debug_log("[AFMacro] = %d\n", AFMacro);
		debug_log("======================AF CAM_CAL==================\n");
		#endif
	}

	return err;
}

static unsigned int do_lens_id_chromemain(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData)
{
	struct STRUCT_CAM_CAL_DATA_STRUCT *pCamCalData =
				(struct STRUCT_CAM_CAL_DATA_STRUCT *)pGetSensorCalData;

	memcpy(&pCamCalData->LensDrvId[0], &chromemain_txd_main_otp.module_info[5], 1);
	return CAM_CAL_ERR_NO_ERR;
}

static int chromemain_checksum(struct i2c_client *client, u16 checkAddr, unsigned char *checkArray, int length)
{
	int checksum = subdrv_i2c_rd_u8(client, checkAddr);
	int sum = 0;
	for (int i = 0; i < length; i++) {
		sum += checkArray[i];
	}
	debug_log("sum = %d, sum %% 255 + 1 = 0x%02x, checksum = 0x%02x\n", sum, sum % 255 + 1, checksum);

	if ((sum % 255 + 1) != checksum)
		return CHROMEMAIN_OTP_RET_FAIL;
	return CHROMEMAIN_OTP_RET_SUCCESS;
}

static void chromemain_read(struct i2c_client *client, unsigned int start, unsigned int end, unsigned char *pinputdata)
{
	u16 addr = start;
	for (int i = 0; i < end - start + 1; i++) {
		pinputdata[i] = subdrv_i2c_rd_u8(client, addr++);
	}
}

int chromemain_set_threshold(struct i2c_client *client, u8 threshold) //set thereshold
{
	u8 threshold_reg1[3] = { 0x48, 0x48, 0x48 };
	u8 threshold_reg2[3] = { 0x38, 0x18, 0x58 };
	u8 threshold_reg3[3] = { 0x41, 0x41, 0x41 };

	if (threshold < 3 && threshold >= 0) {
		subdrv_i2c_wr_u8(client, 0x36b0, threshold_reg1[threshold]);
		subdrv_i2c_wr_u8(client, 0x36b1, threshold_reg2[threshold]);
		subdrv_i2c_wr_u8(client, 0x36b2, threshold_reg3[threshold]);
		debug_log("chromemain_otp set_threshold %d\n", threshold);
	} else {
		pr_err("chromemain_otp set invalid threshold %d\n", threshold);

		return CHROMEMAIN_OTP_RET_FAIL;
	}

	return CHROMEMAIN_OTP_RET_SUCCESS;
}

int chromemain_set_page_and_load_data(struct i2c_client *client, int page) //set page
{
	u16 Startaddress = 0;
	u16 EndAddress = 0;
	int delay = 0;
	int pag = 0;

	Startaddress = page * 0x200 + 0x7E00; //set start address in page
	EndAddress = Startaddress + 0x1ff; //set end address in page
	pag = page * 2 - 1; //change page
	subdrv_i2c_wr_u8(client, 0x4408, (Startaddress >> 8) & 0xff);
	subdrv_i2c_wr_u8(client, 0x4409, Startaddress & 0xff);
	subdrv_i2c_wr_u8(client, 0x440a, (EndAddress >> 8) & 0xff);
	subdrv_i2c_wr_u8(client, 0x440b, EndAddress & 0xff);

	subdrv_i2c_wr_u8(client, 0x4401, 0x13); // address set finished
	subdrv_i2c_wr_u8(client, 0x4412, pag & 0xff); // set page
	subdrv_i2c_wr_u8(client, 0x4407, 0x00); // set page finished
	subdrv_i2c_wr_u8(client, 0x4400, 0x11); // manual load begin
	while ((subdrv_i2c_rd_u8(client, 0x4420) & 0x01) == 0x01) {
		delay++;
		debug_log("chromemain_otp set_page[%d] waitting, OTP is still busy for loading %d times\n", page, delay);
		if (delay == 10) {
			pr_err("chromemain_otp set_page fail, load timeout!!!\n");

			return CHROMEMAIN_OTP_RET_FAIL;
		}
		mdelay(10);
	}
	debug_log("chromemain_otp set_page success\n");

	return CHROMEMAIN_OTP_RET_SUCCESS;
}

static int chromemain_sensor_otp_read_data(struct i2c_client *client, u16 ui4_offset,
					unsigned int ui4_length, unsigned char *pinputdata)
{
	int i = 0, sum = 0, rd_sum = 0xFF;

	for (i = 0; i < ui4_length; i++) {
		pinputdata[i] = subdrv_i2c_rd_u8(client, ui4_offset + i);
		sum += pinputdata[i];
	}

	rd_sum = subdrv_i2c_rd_u8(client, ui4_offset + i); // i == ui4_length
	debug_log("sum = %x, calc_sum = %x, rd_sum = %x\n", sum, sum % 255 + 1, rd_sum);

	if ((sum % 255 + 1) != rd_sum)
		return -1;

	return 0;
}

static int chromemain_sensor_otp_read_module_info(struct i2c_client *client, unsigned char *pinputdata)
{
	int ret = CHROMEMAIN_OTP_RET_FAIL;

	chromemain_txd_main_otp.ModuleFlag = subdrv_i2c_rd_u8(client, CHROMEMAIN_OTP_MODULE_FLAGADDR);
	debug_log("chromemain_otp Read ModuleFlag addr :0x%x, data:0x%x\n",
			CHROMEMAIN_OTP_MODULE_FLAGADDR,
			chromemain_txd_main_otp.ModuleFlag);

	if (chromemain_txd_main_otp.ModuleFlag == CHROMEMAIN_GROUP1_FLAG) {
		ret = chromemain_sensor_otp_read_data(client,
			CHROMEMAIN_OTP_MODULE_GROUP1_STARTADDR,
			CHROMEMAIN_TXD_main_OTP_MODULE_LENS, pinputdata);
		debug_log("chromemain_otp group1 ret = %d!\n", ret);
	} else if (chromemain_txd_main_otp.ModuleFlag == CHROMEMAIN_GROUP2_FLAG) {
		ret = chromemain_sensor_otp_read_data(client,
			CHROMEMAIN_OTP_MODULE_GROUP2_STARTADDR,
			CHROMEMAIN_TXD_main_OTP_MODULE_LENS, pinputdata);
		debug_log("chromemain_otp group2 ret = %d!\n", ret);
	} else {
		pr_err("chromemain_otp invalid flag :0x%x\n",
				chromemain_txd_main_otp.ModuleFlag);
	}

	return ret;
}

static int chromemain_sensor_otp_read_awb_info(struct i2c_client *client, unsigned char *pinputdata)
{
	int ret = CHROMEMAIN_OTP_RET_FAIL;

	if(chromemain_txd_main_otp.ModuleFlag == CHROMEMAIN_GROUP1_FLAG){
		ret = chromemain_sensor_otp_read_data(client,
			CHROMEMAIN_OTP_AWB_GROUP1_STARTADDR,
			CHROMEMAIN_TXD_main_OTP_AWB_LENS, pinputdata);
		debug_log("chromemain_otp group1 ret = %d!\n", ret);
	} else if(chromemain_txd_main_otp.ModuleFlag == CHROMEMAIN_GROUP2_FLAG) {
		ret = chromemain_sensor_otp_read_data(client,
			CHROMEMAIN_OTP_AWB_GROUP2_STARTADDR,
			CHROMEMAIN_TXD_main_OTP_AWB_LENS, pinputdata);
		debug_log("chromemain_otp group2 ret = %d!\n", ret);
	} else
		pr_err("chromemain_otp invalid!\n");

	return ret;
}

static int chromemain_sensor_otp_read_af_info(struct i2c_client *client, unsigned char *pinputdata)
{
	int ret = CHROMEMAIN_OTP_RET_FAIL;

	if(chromemain_txd_main_otp.ModuleFlag == CHROMEMAIN_GROUP1_FLAG){
		ret = chromemain_sensor_otp_read_data(client,
			CHROMEMAIN_OTP_AF_GROUP1_STARTADDR,
			CHROMEMAIN_TXD_main_OTP_AF_LENS, pinputdata);
		debug_log("chromemain_otp group1 ret = %d!\n", ret);
	} else if(chromemain_txd_main_otp.ModuleFlag == CHROMEMAIN_GROUP2_FLAG) {
		ret = chromemain_sensor_otp_read_data(client,
			CHROMEMAIN_OTP_AF_GROUP2_STARTADDR,
			CHROMEMAIN_TXD_main_OTP_AF_LENS, pinputdata);
		debug_log("chromemain_otp group2 ret = %d!\n", ret);
	} else
		pr_err("chromemain_otp invalid!\n");

	return ret;
}

static int chromemain_sensor_otp_read_lsc_info(struct i2c_client *client, unsigned char *pinputdata)
{
	int ret = CHROMEMAIN_OTP_RET_FAIL;

	if (chromemain_txd_main_otp.ModuleFlag == CHROMEMAIN_GROUP1_FLAG) {
		// lsc 1:
		chromemain_set_page_and_load_data(client, page_2);
		chromemain_read(client, 0x82E2, 0x83FF, &chromemain_txd_main_otp.lsc_data[0]);

		// lsc 2:
		chromemain_set_page_and_load_data(client, page_3);
		chromemain_read(client, 0x847A, 0x85FF, &chromemain_txd_main_otp.lsc_data[0x11E]);

		// lsc 3:
		chromemain_set_page_and_load_data(client, page_4);
		chromemain_read(client, 0x867A, 0x87FF, &chromemain_txd_main_otp.lsc_data[0x2A4]);

		// lsc 4:
		chromemain_set_page_and_load_data(client, page_5);
		chromemain_read(client, 0x887A, 0x89FF, &chromemain_txd_main_otp.lsc_data[0x42A]);

		// lsc 5:
		chromemain_set_page_and_load_data(client, page_6);
		chromemain_read(client, 0x8A7A, 0x8BFF, &chromemain_txd_main_otp.lsc_data[0x5B0]);

		// lsc 6:
		chromemain_set_page_and_load_data(client, page_7);
		chromemain_read(client, 0x8C7A, 0x8C8F, &chromemain_txd_main_otp.lsc_data[0x736]);

		// checksum
		ret = chromemain_checksum(client, 0x8C90, &chromemain_txd_main_otp.lsc_data[0], 1868);
	} else if (chromemain_txd_main_otp.ModuleFlag == CHROMEMAIN_GROUP2_FLAG) {
		// lsc 1:
		chromemain_set_page_and_load_data(client, page_7);
		chromemain_read(client, 0x8C91, 0x8DFF, &chromemain_txd_main_otp.lsc_data[0]);

		// lsc 2:
		chromemain_set_page_and_load_data(client, page_8);
		chromemain_read(client, 0x8E7A, 0x8FFF, &chromemain_txd_main_otp.lsc_data[0x16F]);

		// lsc 3:
		chromemain_set_page_and_load_data(client, page_9);
		chromemain_read(client, 0x907A, 0x91FF, &chromemain_txd_main_otp.lsc_data[0x2F5]);

		// lsc 4:
		chromemain_set_page_and_load_data(client, page_10);
		chromemain_read(client, 0x927A, 0x93FF, &chromemain_txd_main_otp.lsc_data[0x47B]);

		// lsc 5:
		chromemain_set_page_and_load_data(client, page_11);
		chromemain_read(client, 0x947A, 0x95C4, &chromemain_txd_main_otp.lsc_data[0x601]);

		// checksum
		ret = chromemain_checksum(client, 0x95C5, &chromemain_txd_main_otp.lsc_data[0], 1868);
	} else {
		pr_err("dbgmsg - read lsc error");
		// err
	}

	return ret;
}

static int chromemain_sensor_otp_read_by_group(struct i2c_client *client)
{
	int threshold = 0;
	int ret = CHROMEMAIN_OTP_RET_FAIL;

	for (threshold = 0; threshold < 3; threshold++) {
		chromemain_set_threshold(client, threshold);
		chromemain_set_page_and_load_data(client, page_2);
		ret = chromemain_sensor_otp_read_module_info(client,
			chromemain_txd_main_otp.module_info);
		if (ret == CHROMEMAIN_OTP_RET_FAIL) {
			chromemain_txd_main_otp.ModuleFlag = CHROMEMAIN_INVALID_FLAG;
			pr_err("chromemain_otp read module info in threshold R%d fail\n", threshold);
			continue;
		}

		ret = chromemain_sensor_otp_read_awb_info(client,
			chromemain_txd_main_otp.awb_data);
		if (ret == CHROMEMAIN_OTP_RET_FAIL) {
			pr_err("chromemain_otp read awb info in threshold R%d fail\n", threshold);
			continue;
		}

		ret = chromemain_sensor_otp_read_af_info(client,
			chromemain_txd_main_otp.af_data);
		if (ret == CHROMEMAIN_OTP_RET_FAIL) {
			pr_err("chromemain_otp read awb info in threshold R%d fail\n", threshold);
			continue;
		}

		ret = chromemain_sensor_otp_read_lsc_info(client,
			chromemain_txd_main_otp.lsc_data);
		if (ret == CHROMEMAIN_OTP_RET_FAIL) {
			pr_err("chromemain_otp read lsc info in threshold R%d fail\n", threshold);
			continue;
		}

		debug_log("chromemain_otp read all otp data in threshold R%d success\n", threshold);
		break;
	}
	if (ret == CHROMEMAIN_OTP_RET_FAIL) {
		pr_err("chromemain_otp read otp data  in threshold R1 R2 R3 all failed!\n");
	}

	return ret;
}

int chromemain_sensor_otp_read_all_data(struct i2c_client *client)
{
	int ret = CHROMEMAIN_OTP_RET_FAIL;
	int delay = 0;
	client->addr = OTP_I2C_ADDR;

	if (subdrv_i2c_rd_u8(client, 0x3107) == 0xff) {
		pr_err("i2c tran err!\n");
		return ret;
	}

	ret = chromemain_sensor_otp_read_by_group(client);
	if (ret == CHROMEMAIN_OTP_RET_FAIL) {
		pr_err("chromemain_otp read lsc info in threshold R1 R2 R3 all failed!!!\n");

		return ret;
	}
	pr_info("chromemain_otp read otp data success\n");

	subdrv_i2c_wr_u8(client, 0x4408, 0x80);
	subdrv_i2c_wr_u8(client, 0x4409, 0x00);
	subdrv_i2c_wr_u8(client, 0x440a, 0x81);
	subdrv_i2c_wr_u8(client, 0x440b, 0xff);

	subdrv_i2c_wr_u8(client, 0x4401, 0x13);
	subdrv_i2c_wr_u8(client, 0x4412, 0x1);
	subdrv_i2c_wr_u8(client, 0x4407, 0x0e);
	subdrv_i2c_wr_u8(client, 0x4400, 0x11);

	while ((subdrv_i2c_rd_u8(client, 0x4420) & 0x01) == 0x01) {
		delay++;
		debug_log("chromemain_otp read otp is waitting, OTP is still busy for loading %d times\n", delay);
		if (delay == 10) {
			pr_err("chromemain_otp read otp data fail, load timeout!\n");

			return CHROMEMAIN_OTP_RET_FAIL;
		}
		mdelay(10);
	}

	return ret;
}
EXPORT_SYMBOL(chromemain_sensor_otp_read_all_data);

static unsigned int layout_check_chromemain(struct EEPROM_DRV_FD_DATA *pdata, unsigned int sensorID)
{
	unsigned int header_offset = cam_cal_config->layout->header_addr;
	unsigned int check_id = 0x00000000;
	unsigned int result = CAM_CAL_ERR_NO_DEVICE;
	struct i2c_client *client;

	if (cam_cal_config->sensor_id == sensorID)
		pr_info("%s sensor_id matched\n", cam_cal_config->name);
	else {
		pr_info("%s sensor_id not matched\n", cam_cal_config->name);
		return result;
	}

	if (pdata->pdrv->pi2c_client != NULL) {
		client = pdata->pdrv->pi2c_client;
	} else {
		client = NULL;
		pr_err("pdata client is null!\n");
		return result;
	}

	if (chromemain_txd_main_otp.ModuleFlag == 0) {
		debug_log("read sensor otp!\n");
		chromemain_sensor_otp_read_all_data(client);
	}

	memcpy(&check_id, &chromemain_txd_main_otp.module_info[header_offset], 4);

	if (check_id == cam_cal_config->layout->header_id) {	// hearder id on OTP guide
		pr_info("header_id matched 0x%08x\n", check_id);
		result = CAM_CAL_ERR_NO_ERR;
	} else{
		pr_info("header_id not matched 0x%08x\n", check_id);
	}
	return result;
}
