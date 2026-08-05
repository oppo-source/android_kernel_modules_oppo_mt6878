// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2019 MediaTek Inc.

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_runtime.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include "oplus_cam_monitor_define.h"

#define DRIVER_NAME "dw9827c_26232"


#define LOG_INF(format, args...)                                               \
	pr_info(DRIVER_NAME " [%s] " format, __func__, ##args)

#define dw9827c_26232_NAME				"dw9827c_26232"
#define dw9827c_26232_MAX_FOCUS_POS			4095
#define dw9827c_26232_ORIGIN_FOCUS_POS		0
/*
 * This sets the minimum granularity for the focus positions.
 * A value of 1 gives maximum accuracy for a desired focus position
 */
#define dw9827c_26232_FOCUS_STEPS			1
#define dw9827c_26232_SET_POSITION_ADDR		0x00

#define dw9827c_26232_CMD_DELAY			0xff
#define dw9827c_26232_CTRL_DELAY_US			10000
/*
 * This acts as the minimum granularity of lens movement.
 * Keep this value power of 2, so the control steps can be
 * uniformly adjusted for gradual lens movement, with desired
 * number of control steps.
 */
#define dw9827c_26232_MOVE_STEPS			400
#define dw9827c_26232_MOVE_DELAY_US			1000
#define dw9827c_26232_INIT_DELAY_US			3000

#define dw9827c_26232_MOVE_INIT_POS			230

#define dw9827c_26232_CONTROL_REG           0x02

#define dw9827c_26232_MIDDLE_CODE           512

#ifndef OPLUS_FEATURE_CAMERA_COMMON
#define OPLUS_FEATURE_CAMERA_COMMON
#endif /* OPLUS_FEATURE_CAMERA_COMMON */

/* dw9827c_26232 device structure */
struct dw9827c_26232_device {
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_subdev sd;
	struct v4l2_ctrl *focus;
	struct regulator *vin;
	struct regulator *vdd;
	struct pinctrl *vcamaf_pinctrl;
	struct pinctrl_state *vcamaf_on;
	struct pinctrl_state *vcamaf_off;
	/* active or standby mode */
	bool active;
};

struct regval_list {
	unsigned char reg_num;
	unsigned char value;
	unsigned char delay;
};

static struct regval_list dw9827c_26232_init_regs[] = {
	{0x02, 0x40, 3},
	{0x04, 0x01, 3},
	{0x00, ((dw9827c_26232_MOVE_INIT_POS << 6) >> 8) & 0xff, 0},
	{0x01, (dw9827c_26232_MOVE_INIT_POS << 6) & 0xff       , 0},
	{0x02, 0x00, 3},
};

#define VCM_IOC_POWER_ON         _IO('V', BASE_VIDIOC_PRIVATE + 3)
#define VCM_IOC_POWER_OFF        _IO('V', BASE_VIDIOC_PRIVATE + 4)

static int resume_pos = 0;

static inline struct dw9827c_26232_device *to_dw9827c_26232_vcm(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct dw9827c_26232_device, ctrls);
}

static inline struct dw9827c_26232_device *sd_to_dw9827c_26232_vcm(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct dw9827c_26232_device, sd);
}

static int dw9827c_26232_set_position(struct dw9827c_26232_device *dw9827c_26232, u16 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&dw9827c_26232->sd);
	int retry = 3;
	int ret;

	LOG_INF("dw9827c_26232 Set postition:%u.", val);
	LOG_INF("[test] client->addr(0x%x)", client->addr);
	while (--retry > 0) {
        ret = i2c_smbus_write_word_data(client, dw9827c_26232_SET_POSITION_ADDR,
                                        swab16((val << 4)));
		if (ret < 0) {
			usleep_range(dw9827c_26232_MOVE_DELAY_US,
				     dw9827c_26232_MOVE_DELAY_US + 1000);
			#ifdef OPLUS_FEATURE_CAMERA_COMMON
			LOG_INF("dw9827c_26232 Set postition:%u fail.", val);
			#endif /*OPLUS_FEATURE_CAMERA_COMMON*/
		} else {
			break;
		}
	}
	return ret;
}

static int dw9827c_26232_release(struct dw9827c_26232_device *dw9827c_26232)
{
	int ret, val;
	int diff_dac = 0;
	int nstep_count = 0;
	int i = 0;
	struct i2c_client *client = v4l2_get_subdevdata(&dw9827c_26232->sd);

	if (!dw9827c_26232->active) {
		return 0;
	}
	pr_info("%s +\n", __func__);
	diff_dac = dw9827c_26232_ORIGIN_FOCUS_POS - dw9827c_26232->focus->val;

	nstep_count = (diff_dac < 0 ? (diff_dac*(-1)) : diff_dac) /
		dw9827c_26232_MOVE_STEPS;

	val = dw9827c_26232->focus->val;

	for (i = 0; i < nstep_count; ++i) {
		val += (diff_dac < 0 ? (dw9827c_26232_MOVE_STEPS*(-1)) : dw9827c_26232_MOVE_STEPS);

		ret = dw9827c_26232_set_position(dw9827c_26232, val);
		if (ret) {
			LOG_INF("%s I2C failure: %d",
				__func__, ret);
			return ret;
		}
		usleep_range(dw9827c_26232_MOVE_DELAY_US,
			     dw9827c_26232_MOVE_DELAY_US + 1000);
	}

	// last step to origin
	ret = dw9827c_26232_set_position(dw9827c_26232, dw9827c_26232_ORIGIN_FOCUS_POS);
	if (ret) {
		LOG_INF("%s I2C failure: %d",
			__func__, ret);
		return ret;
	}

	i2c_smbus_write_byte_data(client, 0x02, 0x20);
	dw9827c_26232->active = false;

	usleep_range(5000, 5000 + 1000);

	pr_info("%s -\n", __func__);

	return 0;
}


static int dw9827c_26232_write_array(struct dw9827c_26232_device *dw9827c_26232,
			      struct regval_list *vals, u32 len)
{
	unsigned int i;
	int ret;
	struct i2c_client *client = v4l2_get_subdevdata(&dw9827c_26232->sd);

	for (i = 0; i < len; i++) {
		LOG_INF("Init write [0x%x, 0x%x, %d]", vals[i].reg_num, vals[i].value, vals[i].delay);
		ret = i2c_smbus_write_byte_data(client, vals[i].reg_num, vals[i].value);
		if (ret < 0)
			return ret;
		if(vals[i].delay) {
			usleep_range(vals[i].delay * 1000, vals[i].delay * 1000 + 1000);
		}
	}

	return 0;
}


static int dw9827c_26232_init(struct dw9827c_26232_device *dw9827c_26232)
{
	int ret = 0;
	if (dw9827c_26232->active) {
		return 0;
	}
	pr_info("%s +\n", __func__);
	ret = dw9827c_26232_write_array(dw9827c_26232, dw9827c_26232_init_regs,
                    ARRAY_SIZE(dw9827c_26232_init_regs));
	usleep_range(1000, 1000 + 1000);
	LOG_INF("dw9827c_26232_init position: %d,ret=%d", dw9827c_26232->focus->val, ret);
	ret = dw9827c_26232_set_position(dw9827c_26232, dw9827c_26232->focus->val);

	dw9827c_26232->active = true;

	pr_info("%s -,ret=%d\n", __func__, ret);

	return ret;
}

/* Power handling */
static int dw9827c_26232_power_off(struct dw9827c_26232_device *dw9827c_26232)
{
	int ret;
	struct i2c_client *client = v4l2_get_subdevdata(&dw9827c_26232->sd);
	char module_name[NAME_STRING_MAX_LENGTH];
	scnprintf(module_name, sizeof(module_name), "%d-%04x", client->adapter->nr, client->addr);

	LOG_INF("+\n");

	ret = dw9827c_26232_release(dw9827c_26232);
	if (ret)
		LOG_INF("dw9827c_26232 release failed!\n");

	ret = regulator_disable(dw9827c_26232->vin);
	if (ret)
		return ret;

	oplus_cam_monitor_regulator_notify(module_name, "vcm_vin", CAM_MONITOR_DISABLE, 0, 0);

	ret = regulator_disable(dw9827c_26232->vdd);
	if (ret)
		return ret;

	oplus_cam_monitor_regulator_notify(module_name, "vcm_vdd", CAM_MONITOR_DISABLE, 0, 0);

	if (dw9827c_26232->vcamaf_pinctrl && dw9827c_26232->vcamaf_off)
		ret = pinctrl_select_state(dw9827c_26232->vcamaf_pinctrl,
					dw9827c_26232->vcamaf_off);

	LOG_INF("-\n");

	return ret;
}

static int dw9827c_26232_power_on(struct dw9827c_26232_device *dw9827c_26232)
{
	int ret;
	struct i2c_client *client = v4l2_get_subdevdata(&dw9827c_26232->sd);
	char module_name[NAME_STRING_MAX_LENGTH];
	scnprintf(module_name, sizeof(module_name), "%d-%04x", client->adapter->nr, client->addr);

	LOG_INF("+\n");

	ret = regulator_set_voltage(dw9827c_26232->vin, 2800000, 2800000);
	if (ret < 0) {
		LOG_INF("%s regulator_set_voltage vin failure: %d", __func__, ret);
		return ret;
	}

	ret = regulator_enable(dw9827c_26232->vin);
	if (ret < 0) {
		LOG_INF("%s regulator_enable vin failure: %d", __func__, ret);
		return ret;
	}

	oplus_cam_monitor_regulator_notify(module_name, "vcm_vin", CAM_MONITOR_ENABLE, 2800000, 2800000);

	ret = regulator_enable(dw9827c_26232->vdd);
	if (ret < 0) {
		LOG_INF("%s regulator_enable vdd failure: %d", __func__, ret);
		return ret;
	}

	oplus_cam_monitor_regulator_notify(module_name, "vcm_vdd", CAM_MONITOR_ENABLE, 2800000, 2800000);

	if (dw9827c_26232->vcamaf_pinctrl && dw9827c_26232->vcamaf_on)
		ret = pinctrl_select_state(dw9827c_26232->vcamaf_pinctrl,
					dw9827c_26232->vcamaf_on);

	if (ret < 0) {
		LOG_INF("%s pinctrl_select_state failure: %d", __func__, ret);
		return ret;
	}

	/*
	 * TODO(b/139784289): Confirm hardware requirements and adjust/remove
	 * the delay.
	 */
	usleep_range(dw9827c_26232_CTRL_DELAY_US, dw9827c_26232_CTRL_DELAY_US + 100);

	ret = dw9827c_26232_init(dw9827c_26232);
	if (ret < 0)
		goto fail;

	LOG_INF("-\n");

	return 0;

fail:
	regulator_disable(dw9827c_26232->vin);
	regulator_disable(dw9827c_26232->vdd);

	oplus_cam_monitor_regulator_notify(module_name, "vcm_vin", CAM_MONITOR_DISABLE, 0, 0);
	oplus_cam_monitor_regulator_notify(module_name, "vcm_vdd", CAM_MONITOR_DISABLE, 0, 0);

	if (dw9827c_26232->vcamaf_pinctrl && dw9827c_26232->vcamaf_off) {
		pinctrl_select_state(dw9827c_26232->vcamaf_pinctrl,
				dw9827c_26232->vcamaf_off);
	}

	return ret;
}

static int dw9827c_26232_set_ctrl(struct v4l2_ctrl *ctrl)
{
	int ret = 0;
	struct dw9827c_26232_device *dw9827c_26232 = to_dw9827c_26232_vcm(ctrl);

	if (ctrl->id == V4L2_CID_FOCUS_ABSOLUTE) {
		LOG_INF("pos(%d)\n", ctrl->val);
		ret = dw9827c_26232_set_position(dw9827c_26232, ctrl->val);
		if (ret) {
			LOG_INF("%s I2C failure: %d",
				__func__, ret);
			return ret;
		}
	}
	return 0;
}

static const struct v4l2_ctrl_ops dw9827c_26232_vcm_ctrl_ops = {
	.s_ctrl = dw9827c_26232_set_ctrl,
};

static int dw9827c_26232_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	int ret;
	struct dw9827c_26232_device *dw9827c_26232 = sd_to_dw9827c_26232_vcm(sd);

	ret = dw9827c_26232_power_on(dw9827c_26232);
	if (ret < 0) {
		LOG_INF("power on fail, ret = %d\n", ret);
		return ret;
	}

	return 0;
}

static int dw9827c_26232_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct dw9827c_26232_device *dw9827c_26232 = sd_to_dw9827c_26232_vcm(sd);

	dw9827c_26232_power_off(dw9827c_26232);

	return 0;
}

static int dw9827c_26232_vcm_resume(struct dw9827c_26232_device *dw9827c_26232)
{
    int ret = 0, val = dw9827c_26232_MIDDLE_CODE, move_step = 0;
	struct i2c_client *client = v4l2_get_subdevdata(&dw9827c_26232->sd);
	if (dw9827c_26232->active)
		return 0;
	ret = i2c_smbus_write_byte_data(client, dw9827c_26232_CONTROL_REG, 0x00);
	if (ret) {
		LOG_INF("I2C failure!!!\n");
		return ret;
	}

	LOG_INF("cur pos(%d), resume pos(%d)\n", val, resume_pos);
	move_step = resume_pos > val ? dw9827c_26232_MOVE_STEPS : -dw9827c_26232_MOVE_STEPS;

	while (resume_pos - val > dw9827c_26232_MOVE_STEPS || val - resume_pos > dw9827c_26232_MOVE_STEPS) {
		val += move_step;
		ret = dw9827c_26232_set_position(dw9827c_26232, val);
		if (ret)
			LOG_INF("I2C failure: %d", ret);
		usleep_range(dw9827c_26232_MOVE_DELAY_US, dw9827c_26232_MOVE_DELAY_US + 10);
	}
	ret = dw9827c_26232_set_position(dw9827c_26232, resume_pos);

	dw9827c_26232->active = true;
	LOG_INF("exit stand by mode\n");
	return ret;
}

static int dw9827c_26232_vcm_suspend(struct dw9827c_26232_device *dw9827c_26232)
{
    int ret = 0;
    resume_pos = dw9827c_26232->focus->val; // save suspend code
    ret = dw9827c_26232_release(dw9827c_26232);
    pr_info("%s entry stand by mode, active:%d\n", __func__, dw9827c_26232->active);
    return ret;
}

static long dw9827c_26232_ops_core_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	int ret = 0;
	struct dw9827c_26232_device *dw9827c_26232 = sd_to_dw9827c_26232_vcm(sd);
	pr_info("%s +\n", __func__);

	switch (cmd) {
	case VCM_IOC_POWER_ON:
        ret = dw9827c_26232_vcm_resume(dw9827c_26232);
        pr_info("%s VCM_IOC_POWER_ON, cmd:%d, ret: %d\n", __func__, cmd, ret);
        break;
	case VCM_IOC_POWER_OFF:
        ret = dw9827c_26232_vcm_suspend(dw9827c_26232);
        pr_info("%s VCM_IOC_POWER_OFF, cmd:%d, ret:%d\n", __func__, cmd, ret);
        break;
	break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}
	pr_info("%s -\n", __func__);
	return ret;
}

static const struct v4l2_subdev_internal_ops dw9827c_26232_int_ops = {
	.open = dw9827c_26232_open,
	.close = dw9827c_26232_close,
};

static struct v4l2_subdev_core_ops dw9827c_26232_ops_core = {
	.ioctl = dw9827c_26232_ops_core_ioctl,
};

static const struct v4l2_subdev_ops dw9827c_26232_ops = {
	.core = &dw9827c_26232_ops_core,
};

static void dw9827c_26232_subdev_cleanup(struct dw9827c_26232_device *dw9827c_26232)
{
	v4l2_async_unregister_subdev(&dw9827c_26232->sd);
	v4l2_ctrl_handler_free(&dw9827c_26232->ctrls);
#if IS_ENABLED(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&dw9827c_26232->sd.entity);
#endif
}

static int dw9827c_26232_init_controls(struct dw9827c_26232_device *dw9827c_26232)
{
	struct v4l2_ctrl_handler *hdl = &dw9827c_26232->ctrls;
	const struct v4l2_ctrl_ops *ops = &dw9827c_26232_vcm_ctrl_ops;

	v4l2_ctrl_handler_init(hdl, 1);

	dw9827c_26232->focus = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FOCUS_ABSOLUTE,
			  0, dw9827c_26232_MAX_FOCUS_POS, dw9827c_26232_FOCUS_STEPS, 0);

	if (hdl->error)
		return hdl->error;

	dw9827c_26232->sd.ctrl_handler = hdl;

	return 0;
}

static int dw9827c_26232_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct dw9827c_26232_device *dw9827c_26232;
	int ret;

	LOG_INF("+\n");

	dw9827c_26232 = devm_kzalloc(dev, sizeof(*dw9827c_26232), GFP_KERNEL);
	if (!dw9827c_26232)
		return -ENOMEM;

	dw9827c_26232->vin = devm_regulator_get(dev, "vin");
	if (IS_ERR(dw9827c_26232->vin)) {
		ret = PTR_ERR(dw9827c_26232->vin);
		if (ret != -EPROBE_DEFER)
			LOG_INF("cannot get vin regulator\n");
		return ret;
	}

	dw9827c_26232->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(dw9827c_26232->vdd)) {
		ret = PTR_ERR(dw9827c_26232->vdd);
		if (ret != -EPROBE_DEFER)
			LOG_INF("cannot get vdd regulator\n");
		return ret;
	}

	dw9827c_26232->vcamaf_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(dw9827c_26232->vcamaf_pinctrl)) {
		ret = PTR_ERR(dw9827c_26232->vcamaf_pinctrl);
		dw9827c_26232->vcamaf_pinctrl = NULL;
		LOG_INF("cannot get pinctrl\n");
	} else {
		dw9827c_26232->vcamaf_on = pinctrl_lookup_state(
			dw9827c_26232->vcamaf_pinctrl, "vcamaf_on");

		if (IS_ERR(dw9827c_26232->vcamaf_on)) {
			ret = PTR_ERR(dw9827c_26232->vcamaf_on);
			dw9827c_26232->vcamaf_on = NULL;
			LOG_INF("cannot get vcamaf_on pinctrl\n");
		}

		dw9827c_26232->vcamaf_off = pinctrl_lookup_state(
			dw9827c_26232->vcamaf_pinctrl, "vcamaf_off");

		if (IS_ERR(dw9827c_26232->vcamaf_off)) {
			ret = PTR_ERR(dw9827c_26232->vcamaf_off);
			dw9827c_26232->vcamaf_off = NULL;
			LOG_INF("cannot get vcamaf_off pinctrl\n");
		}
	}

	v4l2_i2c_subdev_init(&dw9827c_26232->sd, client, &dw9827c_26232_ops);
	dw9827c_26232->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	dw9827c_26232->sd.internal_ops = &dw9827c_26232_int_ops;

	ret = dw9827c_26232_init_controls(dw9827c_26232);
	if (ret)
		goto err_cleanup;

#if IS_ENABLED(CONFIG_MEDIA_CONTROLLER)
	ret = media_entity_pads_init(&dw9827c_26232->sd.entity, 0, NULL);
	if (ret < 0)
		goto err_cleanup;

	dw9827c_26232->sd.entity.function = MEDIA_ENT_F_LENS;
#endif

	ret = v4l2_async_register_subdev(&dw9827c_26232->sd);
	if (ret < 0)
		goto err_cleanup;
{
	int value = 0x00;
	dw9827c_26232_power_on(dw9827c_26232);

	client->addr = 0x0c;  //0x18
	LOG_INF("[test] client->addr(0x%x)", client->addr);

	i2c_smbus_write_byte_data(client, 0x02, 0x40);//standby
	usleep_range(3 * 1000, 3 * 1000 + 1000);
	value = 0x00;
	value = i2c_smbus_read_byte_data(client, 0x7D);

	if((client->addr == 0x0c) && (value < 0)) {   //0x18 addr error
		LOG_INF("[test] i2c 0x18  error  value(%d)", value);
		client->addr = 0x0f;   //0x1E
		value = i2c_smbus_read_byte_data(client, 0x7D);
		if(value >= 0) {
			LOG_INF("[test] i2c 0x1E  correct  value(%d)", value);

		i2c_smbus_write_byte_data(client, 0x02, 0x40);//standby
		usleep_range(5 * 1000, 5 * 1000 + 1000);
		i2c_smbus_write_byte_data(client, 0x34, 0x85);//pt off
		i2c_smbus_write_byte_data(client, 0x7D, 0x00);// States standby  // change i2c addr to 0x18
//		i2c_smbus_write_byte_data(client, 0x95, 0x00);// SAL OFF
		i2c_smbus_write_byte_data(client, 0x03, 0x01);// store
		usleep_range(10 * 1000, 10 * 1000 + 1000);
		i2c_smbus_write_byte_data(client, 0x34, 0x52);// pt on
		i2c_smbus_write_byte_data(client, 0x04, 0x01);// sw reset
		usleep_range(5 * 1000, 5 * 1000 + 1000);

		value = i2c_smbus_read_byte_data(client, 0x03);
		LOG_INF("[test] 0x18 product_id: %x\n", value);

		client->addr = 0x0c;   //0x18
		value = i2c_smbus_read_byte_data(client, 0x03);
		LOG_INF("[test] 0x18 product_id: %x\n", value);
		} else {
			client->addr = 0x0c;
			LOG_INF("[test] i2c 0x1E  error   value(%d)", value);
		}
	}

	value = i2c_smbus_read_byte_data(client, 0x03);
	LOG_INF("[test] 0x18 product_id: %x\n", value);
	dw9827c_26232_power_off(dw9827c_26232);
}

	LOG_INF("-\n");

	return 0;

err_cleanup:
	dw9827c_26232_subdev_cleanup(dw9827c_26232);
	return ret;
}

static void dw9827c_26232_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct dw9827c_26232_device *dw9827c_26232 = sd_to_dw9827c_26232_vcm(sd);

	LOG_INF("+\n");

	dw9827c_26232_subdev_cleanup(dw9827c_26232);

	LOG_INF("-\n");
	//return 0;
}

static const struct i2c_device_id dw9827c_26232_id_table[] = {
	{ dw9827c_26232_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, dw9827c_26232_id_table);

static const struct of_device_id dw9827c_26232_of_table[] = {
	{.compatible = "oplus,dw9827c_26232" },
	{ },
};
MODULE_DEVICE_TABLE(of, dw9827c_26232_of_table);

static struct i2c_driver dw9827c_26232_i2c_driver = {
	.driver = {
		.name = dw9827c_26232_NAME,
		.of_match_table = dw9827c_26232_of_table,
	},
	.probe_new  = dw9827c_26232_probe,
	.remove = dw9827c_26232_remove,
	.id_table = dw9827c_26232_id_table,
};

module_i2c_driver(dw9827c_26232_i2c_driver);

MODULE_AUTHOR("Po-Hao Huang <Po-Hao.Huang@mediatek.com>");
MODULE_DESCRIPTION("dw9827c_26232 VCM driver");
MODULE_LICENSE("GPL v2");
