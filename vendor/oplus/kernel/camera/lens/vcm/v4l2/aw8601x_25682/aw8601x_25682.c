/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2025 AWNIC Inc.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_runtime.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#define DRIVER_NAME			"aw8601x_25682"
#define AW8601xAF_I2C_SLAVE_ADDR	(0x18) /* 0x0c: 0x18 >> 1 */

#ifndef _AW8601x_H_
#define _AW8601x_H_

#define RELEASE_FADE_OUT

/* i2c transfer */
#define AW_DATA_BYTE_1		(1)
#define AW_DATA_BYTE_2		(2)
#define AW_DATA_BYTE_3		(3)

#define AW_SUCCESS		(0)
#define AW_ERROR		(-1)
#define AW_EOOR_LOOP		(5)

/* reset */
#define AW_SHUTDOWN		(0x01)
#define AW_WAKEUP		(0x00)
#define AW_RESET_DELAY_MAX	(1200)
#define AW_RESET_DELAY_MIN	(1000)

#define AW_INIT_ERROR_DELAY_MAX	(1000)
#define AW_INIT_ERROR_DELAY_MIN	(500)

/* Maximum limit position */
#define AW_LIMITPOS_MAX		(1023)
#define AW_LIMITPOS_MIN		(0)

/* Register */
#define AW_REG_CHIP_ID		(0x00)
#define AW_REG_IC_VER		(0x01)
#define AW_REG_CONTROL		(0x02)
#define AW_REG_CODE_H		(0x03)
#define AW_REG_CODE_L		(0x04)
#define AW_REG_STATUS		(0x05)
#define AW_REG_ALG_MODE		(0x06)
#define AW_REG_DIV		(0x07)
#define AW_REG_VRCT		(0x08)
#define AW_REG_PRESET	(0x09)
#define AW_REG_NRC		(0x0A)
#define AW_REG_IMAX		(0x10)
#define AW_REG_SWT		(0x11)

#define AW_PD_MODE_EN		(0x01)

/* Init Position */
#define AW_BOTTOM_INIT_POS_H	(0x01)
#define AW_BOTTOM_INIT_POS_L	(0x2c)

#define AW_MID_INIT_POS_H	(0x02)
#define AW_MID_INIT_POS_L	(0x00)

/* Current gears config */
/* the maximum output current gear is defined by product requirements */
#define AW_CURRENT_GEARS_CFG_EN (1)
#define AW86017_CURRENT_120MA	(0x00) /* default */
#define AW86017_CURRENT_150MA	(0x10)
#define AW86017_CURRENT_200MA	(0x20)
#define AW86017_CURRENT_100MA	(0x30)

/* Chip id  */
/* Mid-mounted motor */
#define AW8601_CHIPID		(0x01)
#define AW86016_CHIPID		(0x15)
/* Bottom motor */
#define AW86014_CHIPID		(0x41)
#define AW86017_CHIPID		(0x03)
/* motor type */
#define AW_MID_MOUNTED_MOTOR	(0)
#define AW_BOTTOM_MOTOR		(1)

/* Algo parameter config */
#define AW8601_RING		(0x01) /* Non-direct mode */
#define AW8601_ALGO_MODE	(0x00)
#define AW8601_DIV_H		(0x00)
#define AW8601_DIV_L		(0x00)
#define AW8601_VRCT		(0x00)

#define AW86016_RING		(0x01) /* Non-direct mode */
#define AW86016_ALGO_MODE	(0x00)
#define AW86016_DIV_H		(0x00)
#define AW86016_DIV_L		(0x00)
#define AW86016_VRCT		(0x00)

#define AW86014_RING		(0x01) /* Non-direct mode */
#define AW86014_ALGO_MODE	(0x00)
#define AW86014_DIV		(0x00)
#define AW86014_VRCT		(0x00)

#define AW86017_RING		(0x01) /* Non-direct mode */
#define AW86017_ALGO_MODE	(0x84)
#define AW86017_DIV		(0x01)
#define AW86017_VRCT		(0x38)

//#define AW86016_MIDDLE_TO_BUTTOM
#endif

/* Log Format */
#define AW_LOGI(format, ...) \
	pr_info("[%s][%04d]%s: " format "\n", DRIVER_NAME, __LINE__, __func__, ##__VA_ARGS__)
#define AW_LOGD(format, ...) \
	pr_debug("[%s][%04d]%s: " format "\n", DRIVER_NAME, __LINE__, __func__, ##__VA_ARGS__)
#define AW_LOGE(format, ...) \
	pr_err("[%s][%04d]%s: " format "\n", DRIVER_NAME, __LINE__, __func__, ##__VA_ARGS__)


#define AW8601xAF_ORIGIN_FOCUS_POS	0
/*
 * This sets the minimum granularity for the focus positions.
 * A value of 1 gives maximum accuracy for a desired focus position
 */
#define AW8601xAF_FOCUS_STEPS		1
#define AW8601xAF_CMD_DELAY		0xff
#define AW8601xAF_CTRL_DELAY_US		5000
#define AW8601xAF_POS_CTRL_DELAY_US     1000
/*
 * This acts as the minimum granularity of lens movement.
 * Keep this value power of 2, so the control steps can be
 * uniformly adjusted for gradual lens movement, with desired
 * number of control steps.
 */
#define AW8601xAF_MOVE_STEPS		30
#define AW8601xAF_MOVE_DELAY_US		1000

static int g_last_pos = AW8601xAF_ORIGIN_FOCUS_POS;

/* aw8601xaf device structure */
struct aw8601xaf_device {
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_subdev sd;
	struct v4l2_ctrl *focus;
	struct regulator *vin;
	struct regulator *vdd;
	struct pinctrl *vcamaf_pinctrl;
	struct pinctrl_state *vcamaf_on;
	struct pinctrl_state *vcamaf_off;
};

/*******************************************************************************
 * I2c read/write
 ******************************************************************************/
static int AW8601xAF_WriteRegs(struct aw8601xaf_device *aw_dev, unsigned char a_uAddr, \
	unsigned char *a_uData, unsigned int len)
{
	int ret = 0;
	unsigned char *buf = NULL;
	struct i2c_client *client = v4l2_get_subdevdata(&aw_dev->sd);

	buf = kmalloc(len + 1, GFP_KERNEL);
	if (buf == NULL) {
		AW_LOGE("allocate memory error.");
		return -ENOMEM;
	}
	memset(buf, 0, len + 1);
	buf[0] = a_uAddr;
	memcpy(&buf[1], a_uData, len);
	ret = i2c_master_send(client, buf, len + 1);
	if (ret < 0)
		AW_LOGE("Send data err, ret = %d.", ret);

	kfree(buf);
	buf = NULL;

	return ret;
}

static int AW8601xAF_ReadRegs(struct aw8601xaf_device *aw_dev, unsigned char a_uAddr, \
	unsigned char *a_puData, unsigned int data_len)
{
	int ret = 0;
	struct i2c_msg msg[2];
	int msg_num = 0;
	struct i2c_client *client = v4l2_get_subdevdata(&aw_dev->sd);

	msg_num = ARRAY_SIZE(msg);

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = sizeof(unsigned char);
	msg[0].buf = &a_uAddr;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = data_len;
	msg[1].buf = a_puData;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0) {
		AW_LOGE("i2c transfer error, ret: %d.", ret);
		return ret;
	} else if (ret != msg_num) {
		AW_LOGE("i2c transfer error(size error), ret: %d.", ret);
		return -EAGAIN;
	}

	return AW_SUCCESS;
}

static inline struct aw8601xaf_device *to_aw8601xaf_vcm(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct aw8601xaf_device, ctrls);
}

static inline struct aw8601xaf_device *sd_to_aw8601xaf_vcm(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct aw8601xaf_device, sd);
}

struct regval_list {
	unsigned char reg_num;
	unsigned char value;
};

static int aw8601xaf_set_position(struct aw8601xaf_device *aw_dev, u16 val)
{
	int ret = 0;
	unsigned char pos[3] = { 0 };

	struct i2c_client *client = v4l2_get_subdevdata(&aw_dev->sd);

	pos[0] = AW_REG_CODE_H; /* AW_REG_CODE_H: 0x03 */
	pos[1] = (unsigned char)((val >> 8) & 0x03);
	pos[2] = (unsigned char)(val & 0xff);

	AW_LOGD("Target Position = 0x%04d(%d).", val, val);

	//ret = AW8601xAF_WriteRegs(aw_dev, AW_REG_CONTROL, &pos[0], AW_DATA_BYTE_3);
	ret = i2c_master_send(client, &pos[0], AW_DATA_BYTE_3);
	if (ret < 0) {
		AW_LOGE("Set position err, ret = %d.", ret);
		return ret;
	}

	return AW_SUCCESS;
}

static int aw8601xaf_goto_last_pos(struct aw8601xaf_device *aw_dev)
{
	int ret, val = AW8601xAF_ORIGIN_FOCUS_POS, diff_dac, nStep_count, i;

	diff_dac = g_last_pos - AW8601xAF_ORIGIN_FOCUS_POS;
	if (diff_dac == 0) {
		return 0;
	}
	nStep_count = (diff_dac < 0 ? (diff_dac*(-1)) : diff_dac) /
			AW8601xAF_MOVE_STEPS;
	for (i = 0; i < nStep_count; ++i) {
		val += (diff_dac < 0 ? (AW8601xAF_MOVE_STEPS*(-1)) : AW8601xAF_MOVE_STEPS);
		ret = aw8601xaf_set_position(aw_dev, val);
		if (ret) {
			AW_LOGE("I2C failure: %d", ret);
			return ret;
		}
		usleep_range(AW8601xAF_MOVE_DELAY_US, AW8601xAF_MOVE_DELAY_US + 1000);
	}

	return 0;
}

static int aw8601xaf_release(struct aw8601xaf_device *aw_dev)
{
	int ret;
	int val;
	uint8_t algo_cfg[3] = { 0 };
	uint8_t val_off;

	AW_LOGI("Start");

	algo_cfg[0] = 0x80;
	algo_cfg[1] = 0;
	algo_cfg[2] = 0;

	ret = AW8601xAF_WriteRegs(aw_dev, AW_REG_ALG_MODE, algo_cfg, AW_DATA_BYTE_3);
	if (ret < 0) {
		AW_LOGE("Set algo_cfg error, ret: %d", ret);
		return -EAGAIN;
	}

	val = aw_dev->focus->val;
	if (val > 300) {
		ret = aw8601xaf_set_position(aw_dev, 300);
		if (ret != AW_SUCCESS)
			AW_LOGE("Set AF target position failed, ret: %d", ret);
		usleep_range(5000,6000);
		val = 300;
	}
	while (val > 10) {
		val -= 10;
		ret = aw8601xaf_set_position(aw_dev, val);
		if (ret != AW_SUCCESS)
			AW_LOGE("Set AF target position failed, ret: %d", ret);
		usleep_range(1000,2000);
	}

	// last step to origin
	ret = aw8601xaf_set_position(aw_dev, AW8601xAF_ORIGIN_FOCUS_POS);
	if (ret) {
		AW_LOGE("I2C failure: %d", ret);
		return ret;
	}

	val_off = AW_WAKEUP;
	ret = AW8601xAF_WriteRegs(aw_dev, AW_REG_CONTROL, &val_off, AW_DATA_BYTE_1);
	if (ret < 0) {
		AW_LOGE("Wake up error, ret: %d.", ret);
		return -EAGAIN;
	}

	AW_LOGI("end");

	return 0;
}

static inline int AW8601xAF_SoftReset(struct aw8601xaf_device *aw_dev)
{
	uint8_t val = 0;
	int ret = 0;

	val = AW_SHUTDOWN;
	ret = AW8601xAF_WriteRegs(aw_dev, AW_REG_CONTROL, &val, AW_DATA_BYTE_1);
	if (ret < 0) {
		AW_LOGE("Shut down error, ret: %d.", ret);
		return -EAGAIN;
	}
	val = AW_WAKEUP;
	ret = AW8601xAF_WriteRegs(aw_dev, AW_REG_CONTROL, &val, AW_DATA_BYTE_1);
	if (ret < 0) {
		AW_LOGE("Wake up error, ret: %d.", ret);
		return -EAGAIN;
	}

	usleep_range(AW_RESET_DELAY_MIN, AW_RESET_DELAY_MAX);

	return ret;
}

static inline int AW8601xAF_CfgCurrentGears(struct aw8601xaf_device *aw_dev, unsigned char chip_id)
{
	uint8_t gears = 0;

	if (chip_id == AW86017_CHIPID) {
		/* the maximum output current gear is defined by product requirements */
		gears = AW86017_CURRENT_120MA;
		AW_LOGD("Current gears config 150mA, val: 0x%02x", gears);
		AW8601xAF_WriteRegs(aw_dev, AW_REG_IMAX, &gears, AW_DATA_BYTE_1);
	} else {
		AW_LOGD("This chip is not support current gears config.");
	}

	AW_LOGD("Current gears config OK, 0x%02x", gears);

	return AW_SUCCESS;
}

static int aw8601xaf_init(struct aw8601xaf_device *aw_dev)
{
	int ret = 0;
	//uint8_t val = 0;
	uint8_t mode_cmd[] = { 0xed, 0xab };
	uint8_t chip_id = 0;
	uint8_t algo_mode = 0;
	uint8_t div = 0;
	uint8_t vrct = 0;
	uint8_t type = 0;
	uint8_t ctl_val = 0;
	uint8_t algo_cfg[3] = { 0 };
	//struct i2c_client *client = v4l2_get_subdevdata(&aw_dev->sd);

	AW_LOGI("Start.");
	/* enter advance mode */
	ret = AW8601xAF_WriteRegs(aw_dev, mode_cmd[0], &mode_cmd[1], AW_DATA_BYTE_1);
	if (ret < 0) {
		AW_LOGE("Enter advance mode failed, ret: %d.", ret);
		return -EAGAIN;
	}

	/* Identification IC */
	ret = AW8601xAF_ReadRegs(aw_dev, AW_REG_CHIP_ID, &chip_id, AW_DATA_BYTE_1);
    AW_LOGI("chip_id = 0x%x",chip_id);
	if (ret < 0) {
		AW_LOGE("Read chipid error, ret: %d.", ret);
		return -EAGAIN;
	}

	switch (chip_id) {
	case AW8601_CHIPID: /* Mid-mounted motor */
		type = AW_MID_MOUNTED_MOTOR;
		algo_mode = AW8601_ALGO_MODE;
		div = (AW8601_DIV_H << 2) | AW8601_DIV_L;
		vrct = AW8601_VRCT;
		break;
	case AW86016_CHIPID: /* Mid-mounted motor */
		type = AW_MID_MOUNTED_MOTOR;
		algo_mode = AW86016_ALGO_MODE;
		div = (AW86016_DIV_H << 2) | AW86016_DIV_L;
		vrct = AW86016_VRCT;
		break;
	case AW86014_CHIPID: /* Bottom motor */
		type = AW_BOTTOM_MOTOR;
		algo_mode = AW86014_ALGO_MODE;
		div = AW86014_DIV;
		vrct = AW86014_VRCT;
		break;
	case AW86017_CHIPID: /* Bottom motor */
		type = AW_BOTTOM_MOTOR;
		algo_mode = AW86017_ALGO_MODE;
		div = AW86017_DIV;
		vrct = AW86017_VRCT;
		break;
	default:
		AW_LOGE("Chip id match failed, Chip ID: 0x%02x", chip_id);
		break;
	}

	/* SoftReset */
	ret = AW8601xAF_SoftReset(aw_dev);
	if (ret < 0) {
		AW_LOGE("Soft reset error, ret: %d.", ret);
		return -EAGAIN;
	}

	if (type == AW_BOTTOM_MOTOR) {
		algo_cfg[0] = (AW86014_RING << 7) | algo_mode;
		algo_cfg[1] = div;
		algo_cfg[2] = vrct;

		ret = AW8601xAF_WriteRegs(aw_dev, AW_REG_ALG_MODE, algo_cfg, AW_DATA_BYTE_3);
		if (ret < 0) {
			AW_LOGE("Set algo_cfg error, ret: %d", ret);
			return -EAGAIN;
		}
	} else if (type == AW_MID_MOUNTED_MOTOR) {
		ret = AW8601xAF_ReadRegs(aw_dev, AW_REG_CONTROL, &ctl_val, AW_DATA_BYTE_1);
		if (ret < 0) {
			AW_LOGE("Read ctl_val error, ret: %d", ret);
			return -EAGAIN;
		}
		ctl_val &= 0xfd; /* clear RING flag */
		ctl_val |= (AW86016_RING << 1);
		ret = AW8601xAF_WriteRegs(aw_dev, AW_REG_CONTROL, &ctl_val, AW_DATA_BYTE_1);
		if (ret < 0) {
			AW_LOGE("Set ctl_val error, ret: %d", ret);
			return -EAGAIN;
		}
		algo_cfg[0] = (algo_mode << 6) | (div >> 2);
		algo_cfg[1] = (div << 6) | vrct;

		ret = AW8601xAF_WriteRegs(aw_dev, AW_REG_ALG_MODE, algo_cfg, AW_DATA_BYTE_2);
		if (ret < 0) {
			AW_LOGE("Set algo_cfg error, ret: %d", ret);
			return -EAGAIN;
		}
	}

#if AW_CURRENT_GEARS_CFG_EN
	AW8601xAF_CfgCurrentGears(aw_dev,chip_id);
#endif

	aw8601xaf_goto_last_pos(aw_dev);
	AW_LOGI("End.");

	return ret;
}

/* Power handling */
static int aw8601xaf_power_off(struct aw8601xaf_device *aw_dev)
{
	int ret;

	AW_LOGI("entry");

	ret = aw8601xaf_release(aw_dev);
	if (ret)
		AW_LOGI("aw8601xaf release failed!\n");

	ret = regulator_disable(aw_dev->vin);
	if (ret)
		return ret;

	ret = regulator_disable(aw_dev->vdd);
	if (ret)
		return ret;

	if (aw_dev->vcamaf_pinctrl && aw_dev->vcamaf_off)
		ret = pinctrl_select_state(aw_dev->vcamaf_pinctrl,
					aw_dev->vcamaf_off);

	AW_LOGI("end");

	return ret;
}

static int aw8601xaf_power_on(struct aw8601xaf_device *aw_dev)
{
	int ret;

	AW_LOGI("entry");
	ret = regulator_enable(aw_dev->vin);
	if (ret < 0)
		return ret;

	ret = regulator_enable(aw_dev->vdd);
	if (ret < 0)
		return ret;

	if (aw_dev->vcamaf_pinctrl && aw_dev->vcamaf_on)
		ret = pinctrl_select_state(aw_dev->vcamaf_pinctrl,
					aw_dev->vcamaf_on);

	if (ret < 0)
		return ret;

	/*
	 * TODO:Confirm hardware requirements and adjust/remove the delay.
	 */
	usleep_range(AW8601xAF_CTRL_DELAY_US, AW8601xAF_CTRL_DELAY_US + 100);

	ret = aw8601xaf_init(aw_dev);
	if (ret < 0)
		goto fail;

	AW_LOGI("end");
	return 0;

fail:
	regulator_disable(aw_dev->vin);
	regulator_disable(aw_dev->vdd);
	if (aw_dev->vcamaf_pinctrl && aw_dev->vcamaf_off) {
		pinctrl_select_state(aw_dev->vcamaf_pinctrl,
				aw_dev->vcamaf_off);
	}

	return ret;
}

static int aw8601xaf_set_ctrl(struct v4l2_ctrl *ctrl)
{
	int ret = 0;
	int loop_time = 0;
	uint8_t status = 0;
	struct aw8601xaf_device *aw_dev = to_aw8601xaf_vcm(ctrl);

	if (ctrl->id == V4L2_CID_FOCUS_ABSOLUTE) {
		/*wait for I2C bus idle*/
		while (loop_time < 20)
		{
			ret = AW8601xAF_ReadRegs(aw_dev, AW_REG_STATUS, &status, AW_DATA_BYTE_1);
			if (ret < 0) {
				AW_LOGE("Read chipid error, ret: %d.", ret);
				return -EAGAIN;
			}
			status = status & 0x10;
			AW_LOGD("aw8601xaf 0x05 status:%x", status);
			if(status == 0){
				break;
			}
			loop_time++;
			usleep_range(AW8601xAF_POS_CTRL_DELAY_US, AW8601xAF_POS_CTRL_DELAY_US + 100);
		}
		AW_LOGI("pos(%d)\n", ctrl->val);
		ret = aw8601xaf_set_position(aw_dev, ctrl->val);
		if (ret) {
			AW_LOGI("I2C failure: %d", ret);
			return ret;
		}
		g_last_pos = ctrl->val;
	}
	return 0;
}

static const struct v4l2_ctrl_ops aw8601xaf_vcm_ctrl_ops = {
	.s_ctrl = aw8601xaf_set_ctrl,
};

static int aw8601xaf_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	int ret;
	struct aw8601xaf_device *aw_dev = sd_to_aw8601xaf_vcm(sd);

	ret = aw8601xaf_power_on(aw_dev);
	if (ret < 0) {
		AW_LOGE("power on fail, ret = %d.", ret);
		return ret;
	}

	return 0;
}

static int aw8601xaf_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct aw8601xaf_device *aw_dev = sd_to_aw8601xaf_vcm(sd);

	aw8601xaf_power_off(aw_dev);

	return 0;
}

static const struct v4l2_subdev_internal_ops aw8601xaf_v4l2_int_ops = {
	.open = aw8601xaf_open,
	.close = aw8601xaf_close,
};

static const struct v4l2_subdev_ops aw8601xaf_ops = { };

static void aw8601xaf_subdev_cleanup(struct aw8601xaf_device *aw_dev)
{
	v4l2_async_unregister_subdev(&aw_dev->sd);
	v4l2_ctrl_handler_free(&aw_dev->ctrls);
#if IS_ENABLED(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&aw_dev->sd.entity);
#endif
}

static int aw8601xaf_init_controls(struct aw8601xaf_device *aw_dev)
{
	struct v4l2_ctrl_handler *hdl = &aw_dev->ctrls;
	const struct v4l2_ctrl_ops *ops = &aw8601xaf_vcm_ctrl_ops;

	v4l2_ctrl_handler_init(hdl, 1);

	aw_dev->focus = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FOCUS_ABSOLUTE,
			  0, AW_LIMITPOS_MAX, AW8601xAF_FOCUS_STEPS, 0);

	if (hdl->error)
		return hdl->error;

	aw_dev->sd.ctrl_handler = hdl;

	return 0;
}

static int aw8601xaf_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct aw8601xaf_device *aw_dev;
	int ret = 0;

	AW_LOGI("entry");
	aw_dev = devm_kzalloc(dev, sizeof(*aw_dev), GFP_KERNEL);
	if (!aw_dev) {
		AW_LOGE("devm_kzalloc failed");
		return -ENOMEM;
	}

	aw_dev->vin = devm_regulator_get(dev, "vin");
	if (IS_ERR(aw_dev->vin)) {
		ret = PTR_ERR(aw_dev->vin);
		if (ret != -EPROBE_DEFER)
			AW_LOGE("cannot get vin regulator.");
		return ret;
	}

	aw_dev->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(aw_dev->vdd)) {
		ret = PTR_ERR(aw_dev->vdd);
		if (ret != -EPROBE_DEFER)
			AW_LOGE("cannot get vdd regulator.");
		return ret;
	}

	aw_dev->vcamaf_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(aw_dev->vcamaf_pinctrl)) {
		ret = PTR_ERR(aw_dev->vcamaf_pinctrl);
		aw_dev->vcamaf_pinctrl = NULL;
		AW_LOGE("cannot get pinctrl.");
	} else {
		aw_dev->vcamaf_on = pinctrl_lookup_state(
			aw_dev->vcamaf_pinctrl, "vcamaf_on");

		if (IS_ERR(aw_dev->vcamaf_on)) {
			ret = PTR_ERR(aw_dev->vcamaf_on);
			aw_dev->vcamaf_on = NULL;
			AW_LOGE("cannot get vcamaf_on pinctrl.");
		}

		aw_dev->vcamaf_off = pinctrl_lookup_state(
			aw_dev->vcamaf_pinctrl, "vcamaf_off");

		if (IS_ERR(aw_dev->vcamaf_off)) {
			ret = PTR_ERR(aw_dev->vcamaf_off);
			aw_dev->vcamaf_off = NULL;
			AW_LOGE("cannot get vcamaf_off pinctrl.");
		}
	}
	AW_LOGI("init pinctrl end");

	// init client and addr
	client->addr = (AW8601xAF_I2C_SLAVE_ADDR >> 1);

	/* init v4l2 */
	v4l2_i2c_subdev_init(&aw_dev->sd, client, &aw8601xaf_ops);
	aw_dev->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	aw_dev->sd.internal_ops = &aw8601xaf_v4l2_int_ops;
	AW_LOGI("init aw_dev->sd end");

	ret = aw8601xaf_init_controls(aw_dev);
	if (ret)
		goto err_cleanup;
	AW_LOGI("init init_controls end");

#if IS_ENABLED(CONFIG_MEDIA_CONTROLLER)
	ret = media_entity_pads_init(&aw_dev->sd.entity, 0, NULL);
	if (ret < 0)
		goto err_cleanup;

	aw_dev->sd.entity.function = MEDIA_ENT_F_LENS;
	AW_LOGI("aw_dev->sd.entity.function");
#endif
	ret = v4l2_async_register_subdev(&aw_dev->sd);
	if (ret < 0)
		goto err_cleanup;
	AW_LOGI("init v4l2 end");
	AW_LOGI("exit");
	return 0;
err_cleanup:
	AW_LOGE("err_cleanup");
	aw8601xaf_subdev_cleanup(aw_dev);
	AW_LOGE("aw8601xaf_subdev_cleanup");
	return ret;
}

static void aw8601xaf_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct aw8601xaf_device *aw_dev = sd_to_aw8601xaf_vcm(sd);

	AW_LOGI("entry");
	aw8601xaf_subdev_cleanup(aw_dev);
	AW_LOGI("end");
}

static const struct i2c_device_id aw8601xaf_id_table[] = {
	{ DRIVER_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, aw8601xaf_id_table);

static const struct of_device_id aw8601xaf_of_table[] = {
	{ .compatible = "oplus,aw8601x_25682" },
	{ },
};
MODULE_DEVICE_TABLE(of, aw8601xaf_of_table);

static struct i2c_driver aw8601xaf_i2c_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = aw8601xaf_of_table,
	},
	.probe_new  = aw8601xaf_probe,
	.remove = aw8601xaf_remove,
	.id_table = aw8601xaf_id_table,
};

module_i2c_driver(aw8601xaf_i2c_driver);

MODULE_AUTHOR("XXX");
MODULE_DESCRIPTION("AW8601xAF VCM driver");
MODULE_LICENSE("GPL v2");
