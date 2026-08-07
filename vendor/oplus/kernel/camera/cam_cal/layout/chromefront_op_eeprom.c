// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#define PFX "CAM_CAL"
#define pr_fmt(fmt) PFX "[%s] " fmt, __func__
#define MAX_TEMP(a,b,c)  (a)>(b)?((a)>(c)?(a):(c)):((b)>(c)?(b):(c))

#include <linux/kernel.h>
#include <linux/delay.h>
#include "cam_cal_list.h"
#include "eeprom_i2c_common_driver.h"
#include "eeprom_i2c_custom_driver.h"
#include "cam_cal_config.h"
#include "oplus_kd_imgsensor.h"

#define CHROMEFRONT_OTP_TOTAL_SIZE 1900

#define CHROMEFRONT_MOUDLE 0
#define MODULE_GROUP_FLAG 0x827A
#define MODULE_GROUP1_INFO_PAGE 2
#define MODULE_GROUP1_INFO_ADDR 0x827D
#define MODULE_GROUP1_CHECKSUM 0x8286
#define MODULE_GROUP2_INFO_PAGE 7
#define MODULE_GROUP2_INFO_ADDR 0x8C7D
#define MODULE_GROUP2_CHECKSUM 0x8C86
#define MODULE_INFO_LENGTH 9

#define CHROMEFRONT_AWB 10
#define CHROMEFRONT_AWB_LENGTH 20
#define CHROMEFRONT_AWB_CHECKSUM 30
#define AWB_GROUP_FLAG 0x827B
#define AWB_GROUP1_INFO_PAGE 2
#define AWB_GROUP1_INFO_ADDR 0x8287
#define AWB_GROUP1_CHECKSUM 0x829B
#define AWB_GROUP2_INFO_PAGE 7
#define AWB_GROUP2_INFO_ADDR 0x8C87
#define AWB_GROUP2_CHECKSUM 0x8C9B

#define CHROMEFRONT_LSC  31
#define CHROMEFRONT_LSC_LENGTH 1868
#define CHROMEFRONT_LSC_CHECKSUM 1899
#define LSC_GROUP_FLAG 0x827C
#define LSC_GROUP1_PART1_INFO_PAGE 2
#define LSC_GROUP1_PART1_INFO_ADDR 0x829C
#define LSC_GROUP1_PART1_INFO_LENGTH 356
#define LSC_GROUP1_PART2_INFO_PAGE 3
#define LSC_GROUP1_PART2_INFO_ADDR 0x847A
#define LSC_GROUP1_PART2_INFO_LENGTH 390
#define LSC_GROUP1_PART3_INFO_PAGE 4
#define LSC_GROUP1_PART3_INFO_ADDR 0x867A
#define LSC_GROUP1_PART3_INFO_LENGTH 390
#define LSC_GROUP1_PART4_INFO_PAGE 5
#define LSC_GROUP1_PART4_INFO_ADDR 0x887A
#define LSC_GROUP1_PART4_INFO_LENGTH 390
#define LSC_GROUP1_PART5_INFO_PAGE 6
#define LSC_GROUP1_PART5_INFO_ADDR 0x8A7A
#define LSC_GROUP1_PART5_INFO_LENGTH 342
#define LSC_GROUP1_CHECKSUM 0x8BD0
#define LSC_GROUP2_PART1_INFO_PAGE 7
#define LSC_GROUP2_PART1_INFO_ADDR 0x8C9C
#define LSC_GROUP2_PART1_INFO_LENGTH 356
#define LSC_GROUP2_PART2_INFO_PAGE 8
#define LSC_GROUP2_PART2_INFO_ADDR 0x8E7A
#define LSC_GROUP2_PART2_INFO_LENGTH 390
#define LSC_GROUP2_PART3_INFO_PAGE 9
#define LSC_GROUP2_PART3_INFO_ADDR 0x907A
#define LSC_GROUP2_PART3_INFO_LENGTH 390
#define LSC_GROUP2_PART4_INFO_PAGE 10
#define LSC_GROUP2_PART4_INFO_ADDR 0x927A
#define LSC_GROUP2_PART4_INFO_LENGTH 390
#define LSC_GROUP2_PART5_INFO_PAGE 11
#define LSC_GROUP2_PART5_INFO_ADDR 0x947A
#define LSC_GROUP2_PART5_INFO_LENGTH 342
#define LSC_GROUP2_CHECKSUM 0x95D0
#define LSC_INFO_LENGTH 1868

#define OTP_I2C_ADDR 0x6c

struct chromefront_otp_t {
    u8  module_param[9];
    u8  module_checksum;
    u8  awb_param[20];
    u8  awb_checksum;
    u8  lsc_param[1868];
    u8  lsc_checksum;
};

 struct STRUCT_CHROMEFRONT_CAM_CAL_MODULE_INFO {
     unsigned char module_id;
     unsigned char position_id;
     unsigned char lens_id;
	 unsigned char reserved[3];
     unsigned char year;
     unsigned char month;
     unsigned char day;
 };

struct chromefront_otp_t chromefront_otp_info;
static bool chromefront_otp_read_tag = 0;

static int iReadRegI2C(struct i2c_client *client,
		u8 *a_pSendData, u16 a_sizeSendData,
		u8 *a_pRecvData, u16 a_sizeRecvData)
{
	int  i4RetValue = 0;
	struct i2c_msg msg[2];

	msg[0].addr = client->addr;
	msg[0].flags = client->flags & I2C_M_TEN;
	msg[0].len = a_sizeSendData;
	msg[0].buf = a_pSendData;

	msg[1].addr = client->addr;
	msg[1].flags = client->flags & I2C_M_TEN;
	msg[1].flags |= I2C_M_RD;
	msg[1].len = a_sizeRecvData;
	msg[1].buf = a_pRecvData;

	i4RetValue = i2c_transfer(client->adapter, msg, 2);

	if (i4RetValue != 2) {
		pr_err("chromefront I2C read failed!! addr = 0x%x\n",client->addr);
		return -1;
	}
	return 0;

}

static int iWriteRegI2C(struct i2c_client *client,
			u8 *a_pSendData, u16 a_sizeSendData)
{
	int  i4RetValue = 0;
  	struct i2c_msg msg;

  	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = a_sizeSendData;
	msg.buf = a_pSendData;

	i4RetValue = i2c_transfer(client->adapter, &msg, 1);

	if (i4RetValue != 1) {
		pr_err("chromefront I2C write failed!! addr = 0x%x\n",client->addr);
		return -1;
	}
	return 0;
}

static u16 read_otp(struct i2c_client *client, u32 addr)
{
	u16 get_byte = 0;
	char pu_send_cmd[2] = { (char)((addr >> 8) & 0xFF),
		(char)(addr & 0xFF) };

	iReadRegI2C(client, pu_send_cmd, 2, (u8 *)&get_byte, 1);

	return get_byte;
}

static void write_otp(struct i2c_client *client, u16 addr, u8 para)
{
	char pu_send_cmd[3] = { (char)((addr >> 8) & 0xFF),
		(char)(addr & 0xFF), (char)(para & 0xFF) };

	iWriteRegI2C(client, pu_send_cmd, 3);
}

static void chromefront_sensor_init(struct i2c_client *client)
{

    write_otp(client,0x36e9,0x80);
    write_otp(client,0x37f9,0x80);
    write_otp(client,0x36e9,0x24);
    write_otp(client,0x37f9,0x24);
    write_otp(client,0x301f,0x01);
    write_otp(client,0x3205,0xc7);
    write_otp(client,0x3211,0x04);
    write_otp(client,0x3270,0x00);
    write_otp(client,0x3271,0x00);
    write_otp(client,0x3272,0x00);
    write_otp(client,0x3273,0x03);
    write_otp(client,0x3301,0x08);
    write_otp(client,0x3303,0x10);
    write_otp(client,0x3306,0x84);
    write_otp(client,0x3309,0x88);
    write_otp(client,0x330a,0x01);
    write_otp(client,0x330b,0x0c);
    write_otp(client,0x330c,0x10);
    write_otp(client,0x330d,0x18);
    write_otp(client,0x330e,0x30);
    write_otp(client,0x3314,0x15);
    write_otp(client,0x331f,0x79);
    write_otp(client,0x3326,0x0e);
    write_otp(client,0x3327,0x0a);
    write_otp(client,0x3329,0x0b);
    write_otp(client,0x3333,0x10);
    write_otp(client,0x3334,0x40);
    write_otp(client,0x335d,0x60);
    write_otp(client,0x3364,0x56);
    write_otp(client,0x336c,0xce);
    write_otp(client,0x3390,0x08);
    write_otp(client,0x3391,0x09);
    write_otp(client,0x3392,0x0f);
    write_otp(client,0x3393,0x10);
    write_otp(client,0x3394,0x20);
    write_otp(client,0x3395,0x28);
    write_otp(client,0x33ad,0x3c);
    write_otp(client,0x33af,0x70);
    write_otp(client,0x33b2,0x70);
    write_otp(client,0x33b3,0x40);
    write_otp(client,0x349f,0x1e);
    write_otp(client,0x34a6,0x09);
    write_otp(client,0x34a7,0x0f);
    write_otp(client,0x34a8,0x30);
    write_otp(client,0x34a9,0x20);
    write_otp(client,0x34f8,0x1f);
    write_otp(client,0x34f9,0x08);
    write_otp(client,0x3637,0x43);
    write_otp(client,0x363c,0x8d);
    write_otp(client,0x3670,0x4a);
    write_otp(client,0x3674,0xf6);
    write_otp(client,0x3675,0xdc);
    write_otp(client,0x3676,0xcc);
    write_otp(client,0x367c,0x09);
    write_otp(client,0x367d,0x0f);
    write_otp(client,0x3690,0x34);
    write_otp(client,0x3691,0x44);
    write_otp(client,0x3692,0x55);
    write_otp(client,0x3698,0x86);
    write_otp(client,0x3699,0x8d);
    write_otp(client,0x369a,0x99);
    write_otp(client,0x369b,0xb7);
    write_otp(client,0x369c,0x0f);
    write_otp(client,0x369d,0x1f);
    write_otp(client,0x36a2,0x09);
    write_otp(client,0x36a3,0x0b);
    write_otp(client,0x36a4,0x0f);
    write_otp(client,0x36b0,0x48);
    write_otp(client,0x36b1,0x38);
    write_otp(client,0x36b2,0x41);
    write_otp(client,0x370f,0x01);
    write_otp(client,0x3724,0xc1);
    write_otp(client,0x3771,0x07);
    write_otp(client,0x3772,0x03);
    write_otp(client,0x3773,0x63);
    write_otp(client,0x377a,0x08);
    write_otp(client,0x377b,0x0f);
    write_otp(client,0x3901,0x04);
    write_otp(client,0x3903,0xa0);
    write_otp(client,0x3905,0x8d);
    write_otp(client,0x391d,0x01);
    write_otp(client,0x3926,0x23);
    write_otp(client,0x393f,0x80);
    write_otp(client,0x3940,0x00);
    write_otp(client,0x3941,0x00);
    write_otp(client,0x3942,0x00);
    write_otp(client,0x3943,0x63);
    write_otp(client,0x3944,0x5f);
    write_otp(client,0x3e00,0x01);
    write_otp(client,0x3e01,0x38);
    write_otp(client,0x3e02,0x00);
    write_otp(client,0x4401,0x13);
    write_otp(client,0x4402,0x03);
    write_otp(client,0x4403,0x0e);
    write_otp(client,0x4404,0x28);
    write_otp(client,0x4405,0x34);
    write_otp(client,0x4407,0x0e);
    write_otp(client,0x440c,0x42);
    write_otp(client,0x440d,0x42);
    write_otp(client,0x440e,0x32);
    write_otp(client,0x440f,0x53);
    write_otp(client,0x4412,0x01);
    write_otp(client,0x4424,0x01);
    write_otp(client,0x442d,0x00);
    write_otp(client,0x442e,0x00);
    write_otp(client,0x4509,0x28);
    write_otp(client,0x450d,0x18);
    write_otp(client,0x451d,0xc8);
    write_otp(client,0x4526,0x09);
    write_otp(client,0x5000,0x0e);
    write_otp(client,0x550e,0x00);
    write_otp(client,0x550F,0xBC);
    write_otp(client,0x5780,0x66);
    write_otp(client,0x578d,0x40);
    write_otp(client,0x57aa,0xeb);
}

static u16 chromefront_otp_read_group(struct i2c_client *client,u16 page, u16 addr, u8 *data, u16 length, u8 times)
{
    bool re = TRUE;
    u8 def = 0x00, busy_flag = 0x01;

    u8 pRegData[390] = { 0 };
    u8 threshold[3][3] = { {0x48,0x38,0x41},{0x48,0x18,0x41},{0x48,0x58,0x41} };

    write_otp(client,0x36b0, threshold[times][0]);
    write_otp(client,0x36b1, threshold[times][1]);
    write_otp(client,0x36b2, threshold[times][2]);

    //read start addr
    write_otp(client,0x4408, 0x80 + (page - 1) * 0x02);
    write_otp(client,0x4409, 0x00);
    //read end addr
    write_otp(client,0x440a, 0x81 + (page - 1) * 0x02);
    write_otp(client,0x440b, 0xff);

    write_otp(client,0x4401, 0x13);

    //set Page
    write_otp(client,0x4412, 0x03 + (page - 2) * 0x02);
    write_otp(client,0x4407, 0x00);

    write_otp(client,0x4400, 0x11);
    mdelay(10);

    for (size_t loop_time = 0; loop_time < 1000; loop_time++)
    {
        mdelay(5);
        def = read_otp(client,0x4420);//[0]busy,0 ok//[1]otp,0 ok
        busy_flag = def & 0x1;
        if (0 == busy_flag) break;
    }
    pr_info("%d-busy_flag = %d(0 is over，1 is reading)", times + 1, busy_flag);

    if (busy_flag)
    {
        pr_err("read timeout 10s");
        re = FALSE;
        goto READ_CLOCK_END;
    }

    for (int i = 0; i < length; i++)
    {
        pRegData[i] = read_otp(client,addr+i);
    }


	memcpy(data, pRegData, length);
READ_CLOCK_END:
    return re;
}

static int chromefront_iReadData(struct i2c_client *client,u16 page, unsigned int ui4_offset, unsigned int ui4_length, unsigned char *pinputdata, u8 times)
{
    int i4RetValue = 0;
    int i4ResidueDataLength;
    u32 u4CurrentOffset;

    pr_info("ui4_offset = 0x%x, ui4_length = %d \n", ui4_offset, ui4_length);

    i4ResidueDataLength = (int)ui4_length;
    u4CurrentOffset = ui4_offset;
    i4RetValue = chromefront_otp_read_group(client,page, (u16) u4CurrentOffset, pinputdata, i4ResidueDataLength, times);
    if (i4RetValue != 1) {
        pr_err("I2C iReadData failed!!\n");
        return -1;
    }

    return 0;
}

static bool chromefront_param_checksum(u8 *buf, unsigned int size, u8 checksum)
{
    int i, sum = 0;

    for (i = 0; i < size; i++)
    {
        sum += buf[i];
    }

    if ((sum % 255+1) != checksum)
    {
        pr_err("checksum fail size = %d sum=%d sum-in-eeprom=%d", size, sum % 255+1, checksum);
        return false;
    }
    pr_info("checksum success size = %d sum=%d sum-in-eeprom=%d", size, sum % 255+1, checksum);
    return true;
}

static bool chromefront_read_module_info(struct i2c_client *client,u8 moduleflag)
{
    bool ret = false;
    pr_info("--------------chromefront module info read begin------------\n");
	for (int times = 0; times < 2; times++) {
		if (moduleflag == 1) {
			chromefront_iReadData(client,MODULE_GROUP1_INFO_PAGE, MODULE_GROUP1_INFO_ADDR, MODULE_INFO_LENGTH, &chromefront_otp_info.module_param[0],times);
			chromefront_iReadData(client,MODULE_GROUP1_INFO_PAGE, MODULE_GROUP1_CHECKSUM, 1, &chromefront_otp_info.module_checksum,times);
		} else if (moduleflag  == 2) {
			chromefront_iReadData(client,MODULE_GROUP2_INFO_PAGE, MODULE_GROUP2_INFO_ADDR, MODULE_INFO_LENGTH, &chromefront_otp_info.module_param[0],times);
			chromefront_iReadData(client,MODULE_GROUP2_INFO_PAGE, MODULE_GROUP2_CHECKSUM, 1, &chromefront_otp_info.module_checksum,times);
		} else {
			pr_err("--------------chromefront module info read failed------------\n");
		}
		ret = chromefront_param_checksum(&chromefront_otp_info.module_param[0], MODULE_INFO_LENGTH, chromefront_otp_info.module_checksum);
		if (ret) {
			pr_info("--------------chromefront module info checksum success------------\n");
			break;
		}
	}
    return ret;
}

static bool chromefront_read_awb_info(struct i2c_client *client,u8 moduleflag)
{
    bool ret = false;
    pr_info("--------------chromefront awb info read begin------------\n");
	for (int times = 0; times < 2; times++) {
		if (moduleflag  == 1) {
			chromefront_iReadData(client,AWB_GROUP1_INFO_PAGE, AWB_GROUP1_INFO_ADDR, CHROMEFRONT_AWB_LENGTH, &chromefront_otp_info.awb_param[0],times);
			chromefront_iReadData(client,AWB_GROUP1_INFO_PAGE, AWB_GROUP1_CHECKSUM, 1, &chromefront_otp_info.awb_checksum,times);
		} else if (moduleflag == 2) {
			chromefront_iReadData(client,AWB_GROUP2_INFO_PAGE, AWB_GROUP2_INFO_ADDR, CHROMEFRONT_AWB_LENGTH, &chromefront_otp_info.awb_param[0],times);
			chromefront_iReadData(client,AWB_GROUP2_INFO_PAGE, AWB_GROUP2_CHECKSUM, 1, &chromefront_otp_info.awb_checksum,times);
		} else {
			pr_err("--------------chromefront awb info read failed------------\n");
		}
		pr_info("--------------chromefront awb info read end------------\n");
		ret = chromefront_param_checksum(&chromefront_otp_info.awb_param[0], CHROMEFRONT_AWB_LENGTH, chromefront_otp_info.awb_checksum);
		if (ret) {
			pr_info("--------------chromefront awb info checksum success------------\n");
			break;
		}
	}
    return ret;
}

static bool chromefront_read_lsc_info(struct i2c_client *client,u8 moduleflag)
{
    bool ret = false;
    int idex = 0;
    u8 *pBuff;
    pr_info("--------------chromefront lsc info read begin------------\n");
	for (int times = 0; times < 2; times++) {
		if (moduleflag  == 1) {
			pr_err("--------------chromefront lsc info read part1 idex %d------------\n", idex);
			pBuff = &chromefront_otp_info.lsc_param[idex];
			chromefront_iReadData(client,LSC_GROUP1_PART1_INFO_PAGE, LSC_GROUP1_PART1_INFO_ADDR, LSC_GROUP1_PART1_INFO_LENGTH, pBuff,times);

			idex = idex + LSC_GROUP1_PART1_INFO_LENGTH;
			pBuff = &chromefront_otp_info.lsc_param[idex];
			pr_info("--------------chromefront lsc info read part2 idex %d ------------\n", idex);
			chromefront_iReadData(client,LSC_GROUP1_PART2_INFO_PAGE, LSC_GROUP1_PART2_INFO_ADDR, LSC_GROUP1_PART2_INFO_LENGTH,  pBuff/*&chromefront_otp_info.lsc_param[idex]*/,times);

			idex = idex + LSC_GROUP1_PART2_INFO_LENGTH;
			pr_info("--------------chromefront lsc info read part3 idex %d ------------\n", idex);
			pBuff = &chromefront_otp_info.lsc_param[idex];
			chromefront_iReadData(client,LSC_GROUP1_PART3_INFO_PAGE, LSC_GROUP1_PART3_INFO_ADDR, LSC_GROUP1_PART3_INFO_LENGTH, pBuff/*&chromefront_otp_info.lsc_param[idex]*/,times);

			idex = idex + LSC_GROUP1_PART3_INFO_LENGTH;
			pr_info("--------------chromefront lsc info read part4 idex %d ------------\n", idex);
			pBuff = &chromefront_otp_info.lsc_param[idex];
			chromefront_iReadData(client,LSC_GROUP1_PART4_INFO_PAGE, LSC_GROUP1_PART4_INFO_ADDR, LSC_GROUP1_PART4_INFO_LENGTH, pBuff/*&chromefront_otp_info.lsc_param[idex]*/,times);

			idex = idex + LSC_GROUP1_PART4_INFO_LENGTH;
			pr_info("--------------chromefront lsc info read part5 idex %d ------------\n", idex);
			pBuff = &chromefront_otp_info.lsc_param[idex];
			chromefront_iReadData(client,LSC_GROUP1_PART5_INFO_PAGE, LSC_GROUP1_PART5_INFO_ADDR, LSC_GROUP1_PART5_INFO_LENGTH, pBuff/*&chromefront_otp_info.lsc_param[idex]*/,times);
			pr_info("--------------chromefront lsc info read checksum ------------\n");
			chromefront_iReadData(client,LSC_GROUP1_PART5_INFO_PAGE, LSC_GROUP1_CHECKSUM, 1, &chromefront_otp_info.lsc_checksum,times);
		} else if (moduleflag  == 2) {
			pr_info("--------------chromefront lsc info read part1 ------------\n");
			chromefront_iReadData(client,LSC_GROUP2_PART1_INFO_PAGE, LSC_GROUP2_PART1_INFO_ADDR, LSC_GROUP2_PART1_INFO_LENGTH, &chromefront_otp_info.lsc_param[idex],times);
			idex += LSC_GROUP2_PART1_INFO_LENGTH;
			pr_info("--------------chromefront lsc info read part2 ------------\n");
			chromefront_iReadData(client,LSC_GROUP2_PART2_INFO_PAGE, LSC_GROUP2_PART2_INFO_ADDR, LSC_GROUP2_PART2_INFO_LENGTH, &chromefront_otp_info.lsc_param[idex],times);
			idex += LSC_GROUP2_PART2_INFO_LENGTH;
			pr_info("--------------chromefront lsc info read part3 ------------\n");
			chromefront_iReadData(client,LSC_GROUP2_PART3_INFO_PAGE, LSC_GROUP2_PART3_INFO_ADDR, LSC_GROUP2_PART3_INFO_LENGTH, &chromefront_otp_info.lsc_param[idex],times);
			idex += LSC_GROUP2_PART3_INFO_LENGTH;
			pr_info("--------------chromefront lsc info read part4 ------------\n");
			chromefront_iReadData(client,LSC_GROUP2_PART4_INFO_PAGE, LSC_GROUP2_PART4_INFO_ADDR, LSC_GROUP2_PART4_INFO_LENGTH, &chromefront_otp_info.lsc_param[idex],times);
			idex += LSC_GROUP2_PART4_INFO_LENGTH;
			pr_info("--------------chromefront lsc info read part5 ------------\n");
			chromefront_iReadData(client,LSC_GROUP2_PART5_INFO_PAGE, LSC_GROUP2_PART5_INFO_ADDR, LSC_GROUP2_PART5_INFO_LENGTH, &chromefront_otp_info.lsc_param[idex],times);
			pr_info("--------------chromefront lsc info read checksum ------------\n");
			chromefront_iReadData(client,LSC_GROUP2_PART5_INFO_PAGE, LSC_GROUP2_CHECKSUM, 1, &chromefront_otp_info.lsc_checksum,times);
		} else {
			pr_err("--------------chromefront lsc info read failed------------\n");
		}
		pr_info("--------------chromefront lsc info read end------------\n");
		ret = chromefront_param_checksum(&chromefront_otp_info.lsc_param[0], LSC_INFO_LENGTH, chromefront_otp_info.lsc_checksum);
		if (ret) {
			pr_info("--------------chromefront lsc info checksum success------------\n");
			break;
		}
	}
    return ret;
}

static void chromefront_read_finish(struct i2c_client *client){

        write_otp(client,0x4408, 0x00);
        write_otp(client,0x4409, 0x00);
        write_otp(client,0x440a, 0x01);
        write_otp(client,0x440b, 0xff);
        write_otp(client,0x4401, 0x13);
        write_otp(client,0x4407, 0x0e);
        write_otp(client,0x4400, 0x11);
        mdelay(10);

}

static void read_chromefront_otp_data(struct i2c_client *client)
{
    u8 moduleflag =0;
    u8 value =0;
    bool checksum_module = false;
    bool checksum_awb = false;
    bool checksum_lsc = false;

    chromefront_sensor_init(client);
    pr_err("chromefront moduleflag read begin");
    chromefront_iReadData(client,MODULE_GROUP1_INFO_PAGE, MODULE_GROUP_FLAG, 1, &value, 1);
    if (value == 1) {
         moduleflag = 1;
    } else if(value == 0x13){
         moduleflag = 2;
    } else{
        pr_err("chromefront module flag is error!");
    }

    pr_info("chromefront group1 flag = 0x%x", value);
    pr_info("chromefront moduleflag = 0x%x end", moduleflag);


    if (moduleflag != 1 && moduleflag != 2) {
        pr_err("chromefront module invalid moduleflag = 0x%x", moduleflag);
        return;
    }

    checksum_module = chromefront_read_module_info(client,moduleflag);

    chromefront_iReadData(client,AWB_GROUP1_INFO_PAGE, AWB_GROUP_FLAG, 1, &value,1);

    if (value == 1) {
         moduleflag = 1;
    } else if(value == 0x13){
         moduleflag = 2;
    } else{
        pr_err("chromefront awb flag is error!");
    }

    pr_info("chromefront awb flag = 0x%x", value);
    pr_info("chromefront awbflag = 0x%x end", moduleflag);


    if (moduleflag != 1 && moduleflag != 2) {
        pr_err("chromefront awb invalid moduleflag = 0x%x", moduleflag);
        return;
    }
    checksum_awb = chromefront_read_awb_info(client,moduleflag);

    chromefront_iReadData(client,LSC_GROUP1_PART1_INFO_PAGE, LSC_GROUP_FLAG, 1, &value,1);

    if (value == 1) {
         moduleflag = 1;
    } else if(value == 0x13){
         moduleflag = 2;
    } else{
        pr_err("chromefront lsc flag is error!");
    }

    pr_info("chromefront lsc flag = 0x%x", value);
    pr_info("chromefront lscflag = 0x%x end", moduleflag);


    if (moduleflag != 1 && moduleflag != 2) {
        pr_err("chromefront lsc invalid moduleflag = 0x%x", moduleflag);
        return;
    }


    checksum_lsc = chromefront_read_lsc_info(client,moduleflag);

    if (true == (checksum_module & checksum_awb & checksum_lsc))
    {
        pr_err("----------------chromefront otp info check success----------------");
        chromefront_otp_read_tag = 1;
    }
    else
    {
        pr_err("----------------chromefront otp info check fail-------------------");
    }
    chromefront_read_finish(client);
}

unsigned int chromefront_read_region(struct i2c_client *client, unsigned int addr,
                                unsigned char *data, unsigned int size)
{

    unsigned char *dataTmp = data;
    pr_info("chromefront otp region addr = 0x%x, size = %d\n", addr, size);

    if(chromefront_otp_read_tag==0 ){
        read_chromefront_otp_data(client);
    }
    if (addr == 0x827B && size == 4) {//check
        *(u32 *)data = 0x46;
    } else if (addr == 0x0 && size == CHROMEFRONT_OTP_TOTAL_SIZE) {
        unsigned int totalSize = sizeof(chromefront_otp_info.module_param) +
                                 sizeof(chromefront_otp_info.module_checksum) +
                                 sizeof(chromefront_otp_info.awb_param) +
                                 sizeof(chromefront_otp_info.awb_checksum) +
                                 sizeof(chromefront_otp_info.lsc_param) +
                                 sizeof(chromefront_otp_info.lsc_checksum);

        pr_info("chromefront otp region addr = 0x%x, size = %d  totalSize=%d \n", addr, size, totalSize);
        if (size == totalSize) {
            memcpy(dataTmp, chromefront_otp_info.module_param, sizeof(chromefront_otp_info.module_param));
            dataTmp += sizeof(chromefront_otp_info.module_param);
            data[9] = chromefront_otp_info.module_checksum;
            dataTmp += sizeof(chromefront_otp_info.module_checksum);
            memcpy(dataTmp, chromefront_otp_info.awb_param, sizeof(chromefront_otp_info.awb_param));
            dataTmp += sizeof(chromefront_otp_info.awb_param);
            data[30] = chromefront_otp_info.awb_checksum;
            dataTmp += sizeof(chromefront_otp_info.awb_checksum);
            memcpy(dataTmp, chromefront_otp_info.lsc_param, sizeof(chromefront_otp_info.lsc_param));
            dataTmp += sizeof(chromefront_otp_info.lsc_param);
            data[totalSize - 1] = chromefront_otp_info.lsc_checksum;
        } else {
            pr_err("chromefront otp size != totalSize");
        }
    } else if (addr == CHROMEFRONT_MOUDLE && size == MODULE_INFO_LENGTH) { //read single module_param
        memcpy(data,(chromefront_otp_info.module_param), size);
        pr_info("add = 0x%x, read module_param\n",addr);
    } else if (addr == CHROMEFRONT_AWB && size == CHROMEFRONT_AWB_LENGTH) { //read single awb data
        memcpy(data,(chromefront_otp_info.awb_param), size);
        pr_info("add = 0x%x, read awb\n",addr);
    } else if (addr == CHROMEFRONT_LSC && size == CHROMEFRONT_LSC_LENGTH) {//lsc_param
        memcpy(data, chromefront_otp_info.lsc_param, size);
        pr_info("add = 0x%x, read lsc\n",addr);
    } else if (addr == CHROMEFRONT_LSC_CHECKSUM && size == 1) {//lsc_checksum
        *(u32 *)data = chromefront_otp_info.lsc_checksum;
        pr_info("add = 0x%x, read lsc_checksum = %x\n",addr, *(u32 *)data);
    } else if (addr == CHROMEFRONT_AWB_CHECKSUM && size == 1) {//awb_checksum
        *(u32 *)data = chromefront_otp_info.awb_checksum;
        pr_info("add = 0x%x, read awb_checksum = %x\n",addr, *(u32 *)data);
    } else{
        pr_err("chromefront otp addr = 0x%x, size = %d ,read error !!!\n",addr,size);
    }

    return size;
}

unsigned char otp_check_sum_chromefront(unsigned char *data, unsigned int length){
  	int i = 0;
  	int sum = 0;
  	unsigned char checksum;

  	if (data == NULL){
  		error_log("data null");
  		return 1;
  	}

  	for(i = 0; i < length;i++){
  		sum += *(data+i);
  	}
  	checksum = sum % 255 + 1;
  	return checksum;
}


unsigned int do_module_info_chromefront(struct EEPROM_DRV_FD_DATA *pdata,
 		 unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData)
{
 	struct STRUCT_CAM_CAL_DATA_STRUCT *pCamCalData =
 				 (struct STRUCT_CAM_CAL_DATA_STRUCT *)pGetSensorCalData;
 	struct STRUCT_CHROMEFRONT_CAM_CAL_MODULE_INFO  Module_info;
 	int read_data_size;
	pr_info("======================do_module_info_chromefront start==================\n");

 	read_data_size = read_data(pdata, pCamCalData->sensorID, pCamCalData->deviceID,
 					CHROMEFRONT_MOUDLE,
 					MODULE_INFO_LENGTH,
 					(unsigned char *)&Module_info);
 	pr_info("[MODULE_INFO] read_data_size: %d\n", read_data_size);
 	if(read_data_size <= 0) {
 		error_log("%s: %d, Read Failed\n",__func__,__LINE__);
 		show_cmd_error_log(pCamCalData->Command);
 		return CamCalReturnErr[pCamCalData->Command];
 	}

 	pr_info("======================MODULE_INFO==================\n");
 	pr_info("[MODULE_INFO]module_id:0x%x,position_id:0x%x,lens_id:0x%x,date:%d-%d-%d",
				Module_info.module_id,
				Module_info.position_id,
				Module_info.lens_id,
				Module_info.year,
				Module_info.month,
				Module_info.day);
 	pr_info("======================MODULE_INFO==================\n");
  	pr_info("======================do_module_info_chromefront end==================\n");
  	return CAM_CAL_ERR_NO_DEVICE;
}

unsigned int do_2a_gain_chromefront(struct EEPROM_DRV_FD_DATA *pdata,
  		 unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData)
{

    struct STRUCT_CAM_CAL_DATA_STRUCT *pCamCalData =
  						  (struct STRUCT_CAM_CAL_DATA_STRUCT *)pGetSensorCalData;
    int ioctlerr;
    int err = CamCalReturnErr[pCamCalData->Command];
    u8 AWBAFConfig;
    u8 awbCheckSum = 0;
    u8 calchecksum = 0;
    u8 awbdata[20];

    int tempMax  = 0;
    int  CalGb2Gr = 1, CalR2G = 1, CalB2G = 1, CalGr = 1, CalGb = 1, CalG = 1, CalR = 1, CalB = 1;
    int  FacGb2Gr = 1, FacR2G = 1, FacB2G = 1, FacGr = 1, FacGb = 1, FacG = 1, FacR = 1, FacB = 1;
    pr_info("======================do_2a_gain_chromefront start==================\n");
    pr_info("DoCamCal2AGain is enter..BlockSize=%d SensorID=%x\n", block_size, pCamCalData->sensorID);

    memset((void*)&pCamCalData->Single2A, 0, sizeof(struct STRUCT_CAM_CAL_SINGLE_2A_STRUCT));//To set init value
    {
        AWBAFConfig = 0x01;
        pCamCalData->Single2A.S2aVer = 0x01;
        pCamCalData->Single2A.S2aBitEn = (0x03 & AWBAFConfig);
        pCamCalData->Single2A.S2aAfBitflagEn = (0x0C & AWBAFConfig);// //Bit: step 0(inf.), 1(marco), 2, 3, 4,5,6,7
        if(0x01&AWBAFConfig){
            ////AWB////
            read_data(pdata,pCamCalData->sensorID, pCamCalData->deviceID,
                    CHROMEFRONT_AWB, CHROMEFRONT_AWB_LENGTH, (u8 *)&awbdata);

            calchecksum = otp_check_sum_chromefront((unsigned char *)&awbdata,CHROMEFRONT_AWB_LENGTH);

            ioctlerr = read_data(pdata,pCamCalData->sensorID, pCamCalData->deviceID,
                    CHROMEFRONT_AWB_CHECKSUM, 1, (u8 *)&awbCheckSum);

            pr_info("calchecksum = 0x%x", calchecksum);
            pr_info("awbCheckSum = 0x%x", awbCheckSum);
            if(calchecksum == awbCheckSum && calchecksum != 0 && awbCheckSum != 0)
            {
                if(ioctlerr>0)
                {
                    // Get min gain
                    CalR2G  = ((awbdata[8]<<8)|awbdata[9]);
                    CalB2G = ((awbdata[10]<<8)|awbdata[11]);
                    CalGb2Gr = ((awbdata[12]<<8)|awbdata[13]);

                    CalGr = 255;
                    CalGb = (CalGb2Gr * CalGr)/1024;
                    CalG  = ((CalGr + CalGb) * 10000) >> 1;

                    CalR  = (CalR2G * CalG)/1024;
                    CalB  = (CalB2G * CalG)/1024;

                    tempMax = MAX_TEMP(CalR, CalG, CalB);
                    pr_debug("UnitR:%d, UnitG:%d, UnitB:%d, New Unit Max=%d", CalR, CalG, CalB, tempMax);
                    err = CAM_CAL_ERR_NO_ERR;
                }
                else
                {
                    pCamCalData->Single2A.S2aBitEn = CAM_CAL_NONE_BITEN;
                    error_log("ioctl err\n");
                }
            }
            else{
                error_log("CHROMEFRONT awb checksum failed!");
            }
            if (CalR!=0 && CalG!=0 && CalB!=0 )
            {
                pCamCalData->Single2A.S2aAwb.rGainSetNum = 1;
                pCamCalData->Single2A.S2aAwb.rUnitGainu4R = (u32)((tempMax*512 + (CalR >> 1))/CalR);
                pCamCalData->Single2A.S2aAwb.rUnitGainu4G = (u32)((tempMax*512 + (CalG >> 1))/CalG);
                pCamCalData->Single2A.S2aAwb.rUnitGainu4B  = (u32)((tempMax*512 + (CalB >> 1))/CalB);

                pCamCalData->Single2A.S2aAwb.rGainSetNum++;
                pCamCalData->Single2A.S2aAwb.rUnitGainu4R_low = (u32)((tempMax*512 + (CalR >> 1))/CalR);
                pCamCalData->Single2A.S2aAwb.rUnitGainu4G_low = (u32)((tempMax*512 + (CalG >> 1))/CalG);
                pCamCalData->Single2A.S2aAwb.rUnitGainu4B_low  = (u32)((tempMax*512 + (CalB >> 1))/CalB);
            }
            else
            {
                error_log("There are something wrong on EEPROM, plz contact module vendor R=%d G=%d B=%d!!\n", CalR, CalG, CalB);
            }

            if(ioctlerr>0)
            {
                // Get min gain
                FacR2G  = ((awbdata[14]<<8)|awbdata[15]);
                FacB2G = ((awbdata[16]<<8)|awbdata[17]);
                FacGb2Gr = ((awbdata[18]<<8)|awbdata[19]);

                FacGr = 255;
                FacGb = (FacGb2Gr * FacGr)/1024;
                FacG  = ((FacGr + FacGb) * 10000) >> 1;

                FacR  = (FacR2G * FacG)/1024;
                FacB  = (FacB2G * FacG)/1024;
                pr_info("Extract golden Gain OK\n");

                if(FacR > FacG) {
                    /* R > G */
                    if(FacR > FacB)
                        tempMax = FacR;
                    else
                        tempMax = FacB;
                }
                else {
                    /* G > R */
                    if(FacG > FacB)
                        tempMax = FacG;
                    else
                        tempMax = FacB;
                }
                pr_debug("GoldenR:%d, GoldenG:%d, GoldenB:%d, New Golden Max=%d", FacR, FacG, FacB, tempMax);
                err = CAM_CAL_ERR_NO_ERR;
            }
            else
            {
                pCamCalData->Single2A.S2aBitEn = CAM_CAL_NONE_BITEN;
                error_log("ioctl err\n");
            }
            pr_info("Start assign value\n");
            if ( FacR!=0 && FacG!=0 && FacB!=0 )
            {
                pCamCalData->Single2A.S2aAwb.rGoldGainu4R = (u32)((tempMax * 512 + (FacR >> 1)) /FacR);
                pCamCalData->Single2A.S2aAwb.rGoldGainu4G = (u32)((tempMax * 512 + (FacG >> 1)) /FacG);
                pCamCalData->Single2A.S2aAwb.rGoldGainu4B  = (u32)((tempMax * 512 + (FacB >> 1)) /FacB);

                pCamCalData->Single2A.S2aAwb.rGoldGainu4R_low = (u32)((tempMax * 512 + (FacR >> 1)) /FacR);
                pCamCalData->Single2A.S2aAwb.rGoldGainu4G_low = (u32)((tempMax * 512 + (FacG >> 1)) /FacG);
                pCamCalData->Single2A.S2aAwb.rGoldGainu4B_low  = (u32)((tempMax * 512 + (FacB >> 1)) /FacB);
            }
            else
            {
                error_log("There are something wrong on EEPROM, plz contact module vendor!! Golden R=%d G=%d B=%d\n", FacR, FacG, FacB);
            }
            // Set original data to 3A Layer
            pCamCalData->Single2A.S2aAwb.rValueR = CalR/10000;
            pCamCalData->Single2A.S2aAwb.rValueGr = CalGr;
            pCamCalData->Single2A.S2aAwb.rValueGb = CalGb;
            pCamCalData->Single2A.S2aAwb.rValueB = CalB/10000;
            pCamCalData->Single2A.S2aAwb.rGoldenR = FacR/10000;
            pCamCalData->Single2A.S2aAwb.rGoldenGr = FacGr;
            pCamCalData->Single2A.S2aAwb.rGoldenGb = FacGb;
            pCamCalData->Single2A.S2aAwb.rGoldenB = FacB/10000;
            ////Only AWB Gain Gathering <////
            pr_info( "======================CHROMEFRONT_AWB CAM_CAL==================\n");
            pr_info( "[UnitGain] R=%d G=%d B=%d\n",
                        pCamCalData->Single2A.S2aAwb.rUnitGainu4R,
                        pCamCalData->Single2A.S2aAwb.rUnitGainu4G,
                        pCamCalData->Single2A.S2aAwb.rUnitGainu4B);
            pr_info( "[GoldenGain] R=%d G=%d B=%d\n",
                        pCamCalData->Single2A.S2aAwb.rGoldGainu4R,
                        pCamCalData->Single2A.S2aAwb.rGoldGainu4G,
                        pCamCalData->Single2A.S2aAwb.rGoldGainu4B);
            pr_info( "======================CHROMEFRONT_AWB CAM_CAL==================\n");
        }
    }
	pr_info("======================do_2a_gain_chromefront end==================\n");
    return 0;
}

unsigned int do_single_lsc_chromefront(struct EEPROM_DRV_FD_DATA *pdata,
  		 unsigned int start_addr, unsigned int block_size, unsigned int *pGetSensorCalData)
{
    struct STRUCT_CAM_CAL_DATA_STRUCT *pCamCalData =
  						  (struct STRUCT_CAM_CAL_DATA_STRUCT *)pGetSensorCalData;
    int ioctlerr;
    int err = CamCalReturnErr[pCamCalData->Command];
    u16 table_size;
    u8 lscCheckSum = 0;
    u8 calchecksum = 0;

    pr_info("======================do_single_lsc_chromefront start==================\n");

    pCamCalData->SingleLsc.LscTable.MtkLcsData.MtkLscType = 2;//mtk type
    pCamCalData->SingleLsc.LscTable.MtkLcsData.PixId = 8;//hardcode.... need to fix

    table_size = 1868;

    ioctlerr = read_data(pdata,pCamCalData->sensorID, pCamCalData->deviceID,
            CHROMEFRONT_LSC_CHECKSUM, 1, (u8 *)&lscCheckSum); //lscchecksum
    pr_info("cam_cal get lscchecksum =  0x%x\n",lscCheckSum);
    pCamCalData->SingleLsc.LscTable.MtkLcsData.TableSize = CHROMEFRONT_LSC_LENGTH;

    pCamCalData->SingleLsc.TableRotation=1;
    ioctlerr = read_data(pdata,pCamCalData->sensorID, pCamCalData->deviceID,
            CHROMEFRONT_LSC, CHROMEFRONT_LSC_LENGTH, (u8 *)&pCamCalData->SingleLsc.LscTable.MtkLcsData.SlimLscType);
    if(CHROMEFRONT_LSC_LENGTH == ioctlerr)
    {
        err = CAM_CAL_ERR_NO_ERR;
    }
    else
    {
        error_log("ioctl err\n");
        err = CamCalReturnErr[pCamCalData->Command];
    }

    calchecksum = otp_check_sum_chromefront((unsigned char *)&pCamCalData->SingleLsc.LscTable.MtkLcsData.SlimLscType,CHROMEFRONT_LSC_LENGTH);

    pr_info( "CHROMEFRONT calchecksum = 0x%x", calchecksum);
    if (calchecksum == lscCheckSum)
    {
        pr_info( "CHROMEFRONT lsc checksum pass!");
    }
    else if(calchecksum == 0 && lscCheckSum == 0)
    {
        memset(pCamCalData->SingleLsc.LscTable.Data, 0, sizeof(pCamCalData->SingleLsc.LscTable.Data));
        error_log( "CHROMEFRONT lsc checksum error, clean all lsc data");
    }

    pr_info( "======================CHROMEFRONT SingleLsc Data==================\n");
    pr_info( "[1st] = %x, %x, %x, %x \n", pCamCalData->SingleLsc.LscTable.Data[0],
                                             pCamCalData->SingleLsc.LscTable.Data[1],
                                             pCamCalData->SingleLsc.LscTable.Data[2],
                                             pCamCalData->SingleLsc.LscTable.Data[3]);
    pr_info( "[1st] = SensorLSC(1)?MTKLSC(2)?  %x \n", pCamCalData->SingleLsc.LscTable.MtkLcsData.MtkLscType);
    pr_info( "CapIspReg =0x%x, 0x%x, 0x%x, 0x%x, 0x%x",
         pCamCalData->SingleLsc.LscTable.MtkLcsData.CapIspReg[0],
         pCamCalData->SingleLsc.LscTable.MtkLcsData.CapIspReg[1],
         pCamCalData->SingleLsc.LscTable.MtkLcsData.CapIspReg[2],
         pCamCalData->SingleLsc.LscTable.MtkLcsData.CapIspReg[3],
         pCamCalData->SingleLsc.LscTable.MtkLcsData.CapIspReg[4]);
    pr_info( "RETURN = 0x%x \n", err);
    pr_info( "======================CHROMEFRONT SingleLsc Data==================\n");

    return err;
}

static struct STRUCT_CALIBRATION_LAYOUT_STRUCT cal_layout_table = {
  	0x0000827B, 0x00000046, CAM_CAL_SINGLE_EEPROM_DATA,
  	{
  		{0x00000001, 0x0000827D, 0x00000009, do_module_info_chromefront},
  		{0x00000000, 0x00000000, 0x00000000, NULL},
  		{0x00000001, 0x000082D6, 0x0000074C, do_single_lsc_chromefront},
  		{0x00000001, 0x000082B5, 0x00000014, do_2a_gain_chromefront},
  		{0x00000000, 0x00000000, 0x00000000, NULL},
  		{0x00000000, 0x00000000, 0x00000000, NULL},
  		{0x00000000, 0x00008280, 0x00000001, NULL}
  	}
};

static unsigned int layout_check_chromefront(struct EEPROM_DRV_FD_DATA *pdata,
		unsigned int sensorID);
struct STRUCT_CAM_CAL_CONFIG_STRUCT chromefront_op_eeprom = {
  	.name = "chromefront_op_eeprom",
  	.check_layout_function = layout_check_chromefront,
  	.read_function = chromefront_read_region,
  	.layout = &cal_layout_table,
  	.sensor_id = CHROMEFRONT_SENSOR_ID,
  	.i2c_write_id = OTP_I2C_ADDR,
  	.max_size = 0x9800,
  	.enable_preload = 1,
  	.preload_size = 0x076C,
  	.has_stored_data = 1,
};
static struct STRUCT_CAM_CAL_CONFIG_STRUCT *cam_cal_config = &chromefront_op_eeprom;
static unsigned int layout_check_chromefront(struct EEPROM_DRV_FD_DATA *pdata, unsigned int sensorID)
{
	unsigned int header_offset = cam_cal_config->layout->header_addr;
	unsigned int check_id = 0x00000000;
	unsigned int result = CAM_CAL_ERR_NO_DEVICE;

	if (cam_cal_config->sensor_id == sensorID)
		pr_info("%s sensor_id matched\n", cam_cal_config->name);
	else {
		pr_info("%s sensor_id not matched\n", cam_cal_config->name);
		return result;
	}

	if (read_data_region(pdata, (u8 *)&check_id, header_offset, 4) != 4) {
		pr_info("header_id read failed\n");
		return result;
	}

	if (check_id == cam_cal_config->layout->header_id) {
		pr_info("header_id matched 0x%08x 0x%08x\n",
			check_id, cam_cal_config->layout->header_id);
		result = CAM_CAL_ERR_NO_ERR;
	} else
		pr_err("header_id not matched 0x%08x 0x%08x\n",
			check_id, cam_cal_config->layout->header_id);

	return result;
}
