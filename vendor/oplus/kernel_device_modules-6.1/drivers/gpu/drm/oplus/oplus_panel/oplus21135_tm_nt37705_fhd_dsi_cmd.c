// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/backlight.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <drm/drm_modes.h>
#include <linux/delay.h>
#include <drm/drm_connector.h>
#include <drm/drm_device.h>
#include <linux/of_graph.h>

#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
//#include <soc/oplus/device_info.h>
//#include <mt-plat/mtk_boot_common.h>

#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../../mediatek/mediatek_v2/mtk_panel_ext.h"
#include "../../mediatek/mediatek_v2/mtk_drm_graphics_base.h"
#endif

#include "oplus21135_tm_nt37705_fhd_dsi_cmd.h"

#ifdef OPLUS_FEATURE_DISPLAY
#include "../../oplus/oplus_drm_disp_panel.h"
#endif /* OPLUS_FEATURE_DISPLAY */

#ifdef OPLUS_FEATURE_DISPLAY_ADFR
#include "../../oplus/oplus_adfr_ext.h"
#endif

#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
#include "../../oplus/oplus_display_onscreenfingerprint.h"
#include "../../mediatek/mediatek_v2/mtk-cmdq-ext.h"
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */

#define SILKY_MAX_NORMAL_BRIGHTNESS   8191
#define BRIGHTNESS_MAX    4095
#define BRIGHTNESS_HALF   2047
#define MAX_NORMAL_BRIGHTNESS   3515
#define LCM_BRIGHTNESS_TYPE 2
#define DRM_PANEL_EVENT_PWM_TURBO  0x14
DEFINE_MUTEX(oplus_pwm_lock);

static struct regulator *vmc_ldo;
static struct regulator *vrfio18_aif;

//static unsigned int oplus_init_fps = 0;
static bool pwm_en = 0;
unsigned int oplus_bl_record = 0;
static unsigned int last_backlight = 0;
static unsigned int bl_record = 0;
static bool pwm_power_on = false;

extern unsigned int oplus_display_brightness;
extern unsigned int oplus_max_normal_brightness;
unsigned int lcm_id1 = 0;
unsigned int lcm_id2 = 0;

static int panel_send_pack_hs_cmd(void *dsi, struct LCM_setting_table *table, unsigned int lcm_cmd_count, dcs_write_gce_pack cb, void *handle);
//static int oplus_panel_osc_change(void *dsi, dcs_write_gce cb, void *handle, bool en);
//extern void lcdinfo_notify(unsigned long val, void *v);

enum oplus_adfr_manual_tianma_min_fps_value {
  	OPLUS_ADFR_MANAUL_MIN_FPS_MAX = 0x00,
  	OPLUS_ADFR_MANAUL_MIN_FPS_60HZ = 0x01,
  	OPLUS_ADFR_MANAUL_MIN_FPS_40HZ = 0x02,
  	OPLUS_ADFR_MANAUL_MIN_FPS_30HZ = 0x03,
  	OPLUS_ADFR_MANAUL_MIN_FPS90_45HZ = 0x01,
  	OPLUS_ADFR_MANAUL_MIN_FPS90_30HZ = 0x02,
};

struct lcm_pmic_info {
	struct regulator *reg_vrfio18_aif;
	struct regulator *reg_vmc3p0;
};

struct lcm {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *bias_pos, *bias_neg;
	struct gpio_desc *bias_gpio;
	struct gpio_desc *vddr1p2_enable_gpio;
	struct gpio_desc *vddr_aod_enable_gpio;
	struct gpio_desc *te_switch_gpio,*te_out_gpio;
	/* ADFR:save display mode for panel init */
	struct drm_display_mode *m;
	bool prepared;
	bool enabled;
	int error;
};

#define lcm_dcs_write_seq(ctx, seq...)                                         \
	({                                                                     \
		const u8 d[] = { seq };                                        \
		BUILD_BUG_ON_MSG(ARRAY_SIZE(d) > 128,                          \
				 "DCS sequence too big for stack");            \
		lcm_dcs_write(ctx, d, ARRAY_SIZE(d));                          \
	})

#define lcm_dcs_write_seq_static(ctx, seq...)                                  \
	({                                                                     \
		static const u8 d[] = { seq };                                 \
		lcm_dcs_write(ctx, d, ARRAY_SIZE(d));                          \
	})

static inline struct lcm *panel_to_lcm(struct drm_panel *panel)
{
	return container_of(panel, struct lcm, panel);
}

static int get_panel_es_ver(void)
{
	int ret = 0;

	if (lcm_id1 == 0x78 && lcm_id2 == 0x01) {
		ret = ES_UDON;
	} else if ((lcm_id1 == 0x70 && lcm_id2 == 0x02)
		|| (lcm_id1 == 0x70 && lcm_id2 == 0x01)) {
		ret = ES1;
	} else if (lcm_id1 == 0x70 && lcm_id2 == 0x03) {
		ret = ES2;
	} else if (lcm_id1 == 0x70 && lcm_id2 == 0x04) {
		ret = ES3;
	} else if (lcm_id1 == 0x70 && lcm_id2 == 0x05) {
		ret = ES4;
	} else if (lcm_id1 == 0x70 && lcm_id2 >= 0x06) {
		ret = ES5;
	}

	return ret;
}

static bool lcm_es1(void)
{
	pr_info("%s lcm_id1=0x%x,lcm_id2=0x%x\n", __func__, lcm_id1, lcm_id2);
	if (get_panel_es_ver() == ES1) {
		return true;
	} else {
		return false;
	}
}

static bool get_pwm_status(void)
{
	if ((get_panel_es_ver() == ES_UDON
		|| get_panel_es_ver() >= ES4) && pwm_en) {
		return true;
	} else {
		return false;
	}
}

#ifdef PANEL_SUPPORT_READBACK
static int lcm_dcs_read(struct lcm *ctx, u8 cmd, void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;

	if (ctx->error < 0)
		return 0;

	ret = mipi_dsi_dcs_read(dsi, cmd, data, len);
	if (ret < 0) {
		dev_info(ctx->dev, "error %d reading dcs seq:(%#x)\n", ret,
			 cmd);
		ctx->error = ret;
	}

	return ret;
}
static void lcm_panel_get_data(struct lcm *ctx)
{
	u8 buffer[3] = { 0 };
	static int ret;

	pr_info("%s+\n", __func__);

	if (ret == 0) {
		ret = lcm_dcs_read(ctx, 0x0A, buffer, 1);
		pr_info("%s  0x%08x\n", __func__, buffer[0] | (buffer[1] << 8));
		dev_info(ctx->dev, "return %d data(0x%08x) to dsi engine\n",
			ret, buffer[0] | (buffer[1] << 8));
	}
}
#endif

static void lcm_dcs_write(struct lcm *ctx, const void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;
	char *addr;

	if (ctx->error < 0)
		return;

	addr = (char *)data;

	if ((int)*addr < 0xB0)
		ret = mipi_dsi_dcs_write_buffer(dsi, data, len);
	else
		ret = mipi_dsi_generic_write(dsi, data, len);

	if (ret < 0) {
		dev_info(ctx->dev, "error %zd writing seq: %ph\n", ret, data);
		ctx->error = ret;
	}
}

static void push_table(struct lcm *ctx, struct LCM_setting_table *table,
		unsigned int count)
{
	unsigned int i;
	unsigned int cmd;

	for (i = 0; i < count; i++) {
		cmd = table[i].cmd;
		switch (cmd) {
		case REGFLAG_DELAY:
			msleep(table[i].count);
			break;
		case REGFLAG_UDELAY:
			usleep_range(table[i].count, table[i].count + 1000);
			break;
		case REGFLAG_END_OF_TABLE:
			break;
		default:
			lcm_dcs_write(ctx, table[i].para_list, 
				table[i].count);
			break;
		}
	}
}

static int lcm_panel_vrfio18_aif_regulator_init(struct device *dev)
{
        static int regulator_vufs_inited;
        int ret = 0;

        if (regulator_vufs_inited)
                return ret;
        pr_err("[LCM] lcm_panel_vrfio18_aif_regulator_init\n");

        /* please only get regulator once in a driver */
        vrfio18_aif = devm_regulator_get(dev, "1p8");
        if (IS_ERR(vrfio18_aif)) { /* handle return value */
                ret = PTR_ERR(vrfio18_aif);
                pr_err("get vrfio18_aif fail, error: %d\n", ret);
                //return ret;
        }
        regulator_vufs_inited = 1;
        return ret; /* must be 0 */

}

static int lcm_panel_vrfio18_aif_enable(struct device *dev)
{
        int ret = 0;
        int retval = 0;

        lcm_panel_vrfio18_aif_regulator_init(dev);

        /* set voltage with min & max*/
	if (!IS_ERR_OR_NULL(vrfio18_aif)) {
		ret = regulator_set_voltage(vrfio18_aif, 1800000, 1800000);
		if (ret < 0)
			pr_err("[LCM] set voltage vrfio18_aif fail, ret = %d\n", ret);
		retval |= ret;
	}
        /* enable regulator */
	if (!IS_ERR_OR_NULL(vrfio18_aif)) {
		ret = regulator_enable(vrfio18_aif);
		if (ret < 0)
			pr_err("[LCM] enable regulator vrfio18_aif fail, ret = %d\n", ret);
		retval |= ret;
	}
	pr_err("[LCM] lcm_panel_vrfio18_aif_enable\n");

        return retval;
}

static int lcm_panel_vrfio18_aif_disable(struct device *dev)
{
	int ret = 0;
	int retval = 0;

	lcm_panel_vrfio18_aif_regulator_init(dev);

	if (IS_ERR_OR_NULL(vrfio18_aif)) {
		pr_err("[LCM] disable regulator fail, vrfio18_aif is null\n");
	}

	if (!IS_ERR_OR_NULL(vrfio18_aif)) {
		ret = regulator_disable(vrfio18_aif);
		pr_err("[LCM] disable regulator vrfio18_aif 1.8v\n");
		if (ret < 0)
			pr_err("[LCM] disable regulator vrfio18_aif fail, ret = %d\n", ret);
		retval |= ret;
	}
	return ret;
}

static int lcm_panel_vmc_ldo_regulator_init(struct device *dev)
{
	static int regulator_vmc_inited;
	int ret = 0;

	if (regulator_vmc_inited)
		return ret;
	pr_err("[LCM] lcm_panel_vmc_ldo_regulator_init\n");

	/* please only get regulator once in a driver */
	vmc_ldo = devm_regulator_get(dev, "3p0");
	if (IS_ERR(vmc_ldo)) { /* handle return value */
		ret = PTR_ERR(vmc_ldo);
		pr_err("[LCM] vmc_ldo fail, error: %d\n", ret);
		//return ret;
	}
	regulator_vmc_inited = 1;
	return ret; /* must be 0 */
}

static int lcm_panel_vmc_ldo_enable(struct device *dev)
{
	int ret = 0;
	int retval = 0;

	lcm_panel_vmc_ldo_regulator_init(dev);

        /* set voltage with min & max*/
	if (!IS_ERR_OR_NULL(vmc_ldo)) {
		ret = regulator_set_voltage(vmc_ldo, 3000000, 3000000);
		if (ret < 0)
			pr_err("[LCM] voltage vmc_ldo fail, ret = %d\n", ret);
		retval |= ret;
	}
        /* enable regulator */
	if (!IS_ERR_OR_NULL(vmc_ldo)) {
		ret = regulator_enable(vmc_ldo);
		if (ret < 0)
			pr_err("[LCM] enable regulator vmc_ldo fail, ret = %d\n", ret);
		retval |= ret;
	}
	pr_err("[LCM] lcm_panel_vmc_ldo_enable\n");

        return retval;
}

static int lcm_panel_vmc_ldo_disable(struct device *dev)
{
	int ret = 0;
	int retval = 0;

	lcm_panel_vmc_ldo_regulator_init(dev);

	if (IS_ERR_OR_NULL(vmc_ldo)) {
		pr_err("[LCM] disable regulator fail, vmc_ldo is null\n");
	}

	if (!IS_ERR_OR_NULL(vmc_ldo)) {
		ret = regulator_disable(vmc_ldo);
		pr_err("[LCM] disable regulator vmc_ldo 3v\n");
		if (ret < 0)
			pr_err("[LCM] disable regulator vmc_ldo fail, ret = %d\n", ret);
		retval |= ret;
	}
	return ret;
}

int oplus_set_dbv_frame(void *dsi, dcs_write_gce_pack cb, void *handle, bool enable)
{
	u8 avdd_base=72;
	u8 avdd_out=0;
	u8 avdd_shift=0;
	u8 elvss_target=0;
	u32 bl_lvl=0;

	mutex_lock(&oplus_pwm_lock);
	bl_lvl = oplus_display_brightness;
	if (enable == true){
		avdd_base=78;
	}
	if(bl_lvl<0x731)
		elvss_target=20;
	else if(bl_lvl<0x81E)
		elvss_target=21;
	else if(bl_lvl<0x90B)
		elvss_target=22;
	else if(bl_lvl==0x90B)
		elvss_target=23;
	else if(bl_lvl<0xA9B)
		elvss_target=24;
	else if(bl_lvl<0xB63)
		elvss_target=25;
	else if(bl_lvl<0xC2B)
		elvss_target=26;
	else if(bl_lvl<0xCF3)
		elvss_target=27;
	else if(bl_lvl<0xDBB)
		elvss_target=28;
	else if(bl_lvl==0xDBB)
		elvss_target=29;
	else if(bl_lvl<0xE56)
		elvss_target=33;
	else if(bl_lvl<0xE75)
		elvss_target=35;
	else if(bl_lvl<0xEB5)
		elvss_target=38;
	else if(bl_lvl<0xF15)
		elvss_target=42;
	else if(bl_lvl <= 0xFFF)
		elvss_target=49;

	avdd_shift = elvss_target-20;
	avdd_out = avdd_base + avdd_shift;
	dsi_switch_avdd[2].para_list[3] = avdd_out;
	pr_info("pwm_turbo oplus_set_dbv_frame, dsi_switch_avdd[2].para_list[3]=%d\n", avdd_out);
	panel_send_pack_hs_cmd(dsi, dsi_switch_avdd, sizeof(dsi_switch_avdd) / sizeof(struct LCM_setting_table), cb, handle);

	mutex_unlock(&oplus_pwm_lock);

	return 0;
}

int oplus_display_panel_set_elvss(void *dsi, dcs_write_gce_pack cb, void *handle, bool en_h_pwm)
{
	pr_info("pwm_turbo oplus_display_panel_set_elvss en_h_pwm:%d\n", en_h_pwm);
	mutex_lock(&oplus_pwm_lock);
	if(en_h_pwm == true)
		panel_send_pack_hs_cmd(dsi, dsi_high_f_switch_120Hz, sizeof(dsi_high_f_switch_120Hz) / sizeof(struct LCM_setting_table), cb, handle);
	else
		panel_send_pack_hs_cmd(dsi, dsi_low_f_switch_120Hz, sizeof(dsi_low_f_switch_120Hz) / sizeof(struct LCM_setting_table), cb, handle);
	usleep_range(8200, 8300);

	if (en_h_pwm) {
		if (oplus_display_brightness <= 0x643) {
			pr_info("pwm_turbo dsi_high_12plus backlight level:%d\n", oplus_display_brightness);
			dsi_switch_elvss[5].para_list[1] = 0x4B;
			dsi_switch_elvss[8].para_list[1] = 0xd2;
		} else if (oplus_display_brightness > 0x643) {
			pr_info("pwm_turbo dsi_low_3plus backlight level:%d\n", oplus_display_brightness);
			dsi_switch_elvss[5].para_list[1] = 0x42;
			dsi_switch_elvss[8].para_list[1] = 0xb2;
		}
	} else {
		pr_info("pwm_turbo closed, set dsi_low_3plus backlight level:%d\n", oplus_display_brightness);
		dsi_switch_elvss[5].para_list[1] = 0x42;
		dsi_switch_elvss[8].para_list[1] = 0xb2;
	}

	dsi_switch_elvss[9].para_list[1] = oplus_display_brightness >> 8;
	dsi_switch_elvss[9].para_list[2] = oplus_display_brightness & 0xFF;
	panel_send_pack_hs_cmd(dsi, dsi_switch_elvss, sizeof(dsi_switch_elvss) / sizeof(struct LCM_setting_table), cb, handle);

	pr_info("pwm_turbo lcdinfo_notify 0x14\n");
	//lcdinfo_notify(DRM_PANEL_EVENT_PWM_TURBO, &en_h_pwm);
	pwm_en = en_h_pwm;
	mutex_unlock(&oplus_pwm_lock);

	return 0;
}

static int get_mode_enum(struct drm_display_mode *m)
{
	int ret = 0;
	int m_vrefresh = 0;

	if (m == NULL) {
    	pr_info("get_mode_enum drm_display_mode *m is null, default 120fps\n");
    	ret = FHD_SDC120;
    	return ret;
	}

	m_vrefresh = drm_mode_vrefresh(m);

    if (m_vrefresh == 60 && m->hskew == SDC_MFR) {
    	ret = FHD_SDC60;
    } else if (m_vrefresh == 90 && m->hskew == SDC_ADFR) {
    	ret = FHD_SDC90;
    } else if (m_vrefresh == 120 && m->hskew == SDC_ADFR) {
    	ret = FHD_SDC120;
    } else if (m_vrefresh == 120 && m->hskew == OPLUS_ADFR) {
    	ret = FHD_OPLUS120;
    }

	return ret;
}

static void lcm_panel_init(struct lcm *ctx)
{
	int mode_id = -1;
	struct drm_display_mode *m = ctx->m;

	mode_id = get_mode_enum(m);

	usleep_range(10000, 10100);
	switch (mode_id) {
	case FHD_SDC60:
		if (get_pwm_status()) {
			pr_info("fhd_dsi_on_cmd_high_pwm_sdc60\n");
			push_table(ctx, dsi_on_cmd_high_pwm_sdc60, sizeof(dsi_on_cmd_high_pwm_sdc60) / sizeof(struct LCM_setting_table));
		} else {
			pr_info("fhd_dsi_on_cmd_sdc60\n");
			if (lcm_es1()) {
				push_table(ctx, dsi_on_cmd_sdc60_id02, sizeof(dsi_on_cmd_sdc60_id02) / sizeof(struct LCM_setting_table));
			} else {
				push_table(ctx, dsi_on_cmd_sdc60_id03, sizeof(dsi_on_cmd_sdc60_id03) / sizeof(struct LCM_setting_table));
			}
		}
		break;
	case FHD_SDC90:
		pr_info("fhd_dsi_on_cmd_sdc90\n");
		if (lcm_es1()) {
			push_table(ctx, dsi_on_cmd_sdc90_id02, sizeof(dsi_on_cmd_sdc90_id02) / sizeof(struct LCM_setting_table));
		} else {
			push_table(ctx, dsi_on_cmd_sdc90_id03, sizeof(dsi_on_cmd_sdc90_id03) / sizeof(struct LCM_setting_table));
		}
		break;
	case FHD_SDC120:
		if (get_pwm_status()) {
			pr_info("fhd_dsi_on_cmd_high_pwm_sdc120\n");
			push_table(ctx, dsi_on_cmd_high_pwm_sdc120, sizeof(dsi_on_cmd_high_pwm_sdc120) / sizeof(struct LCM_setting_table));
		} else {
			pr_info("fhd_dsi_on_cmd_sdc120\n");
			if (lcm_es1()) {
				push_table(ctx, dsi_on_cmd_sdc120_id02, sizeof(dsi_on_cmd_sdc120_id02) / sizeof(struct LCM_setting_table));
			} else {
				push_table(ctx, dsi_on_cmd_sdc120_id03, sizeof(dsi_on_cmd_sdc120_id03) / sizeof(struct LCM_setting_table));
			}
		}
		break;
	case FHD_OPLUS120:
		if (get_pwm_status()) {
			pr_info("default mode_id dsi_on_cmd_high_pwm_oa120\n");
			push_table(ctx, dsi_on_cmd_high_pwm_oa120, sizeof(dsi_on_cmd_high_pwm_oa120) / sizeof(struct LCM_setting_table));
		} else {
			pr_info("fhd_dsi_on_cmd_oplus120\n");
			if (lcm_es1()) {
				push_table(ctx, dsi_on_cmd_oa120_id02, sizeof(dsi_on_cmd_oa120_id02) / sizeof(struct LCM_setting_table));
			} else {
				push_table(ctx, dsi_on_cmd_oa120_id03, sizeof(dsi_on_cmd_oa120_id03) / sizeof(struct LCM_setting_table));
			}
		}
		break;
	default:
		if (get_pwm_status()) {
			pr_info("default mode_id dsi_on_cmd_high_pwm_sdc120\n");
			push_table(ctx, dsi_on_cmd_high_pwm_sdc120, sizeof(dsi_on_cmd_high_pwm_sdc120) / sizeof(struct LCM_setting_table));
		} else {
			pr_info(" default mode_id\n");
			if (lcm_es1()) {
				push_table(ctx, dsi_on_cmd_sdc120_id02, sizeof(dsi_on_cmd_sdc120_id02) / sizeof(struct LCM_setting_table));
			} else {
				push_table(ctx, dsi_on_cmd_sdc120_id03, sizeof(dsi_on_cmd_sdc120_id03) / sizeof(struct LCM_setting_table));
			}
		}
		break;
	}

	pr_info("%s,successful--- mode_id=%d\n", __func__, mode_id);
}

static int lcm_disable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (!ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = false;

	return 0;
}

static int lcm_unprepare(struct drm_panel *panel)
{
	unsigned int vrefresh_rate = 0;
	struct lcm *ctx = panel_to_lcm(panel);
	pr_err("[LCM]%s+\n", __func__);

	if (!ctx->prepared)
		return 0;

	if (!ctx->m) {
		vrefresh_rate = 120;
		// OFP_INFO("default refresh rate is 120hz\n");
	} else {
		vrefresh_rate = drm_mode_vrefresh(ctx->m);
	}

	if (oplus_ofp_get_aod_state()) {
		if (get_pwm_status()) {
			if (vrefresh_rate == 60) {
				push_table(ctx, aod_off_cmd_hpwm_60hz, sizeof(aod_off_cmd_hpwm_60hz) / sizeof(struct LCM_setting_table));
			} else {
				push_table(ctx, aod_off_cmd_hpwm_120hz, sizeof(aod_off_cmd_hpwm_120hz) / sizeof(struct LCM_setting_table));
			}

			if (oplus_display_brightness <= 0x643) {
				pr_info("pwm_turbo dsi_high_12plus bl_lvl=%u\n", oplus_display_brightness);
				push_table(ctx, dsi_high_12plus, sizeof(dsi_high_12plus) / sizeof(struct LCM_setting_table));
			} else if (oplus_display_brightness > 0x643) {
				pr_info("pwm_turbo dsi_low_3plus bl_lvl=%u\n", oplus_display_brightness);
				push_table(ctx, dsi_low_3plus, sizeof(dsi_low_3plus) / sizeof(struct LCM_setting_table));
			}
		} else {
			if (vrefresh_rate == 60) {
				push_table(ctx, aod_off_cmd_lpwm_60hz, sizeof(aod_off_cmd_lpwm_60hz) / sizeof(struct LCM_setting_table));
			} else if (vrefresh_rate == 120) {
				push_table(ctx, aod_off_cmd_lpwm_120hz, sizeof(aod_off_cmd_lpwm_120hz) / sizeof(struct LCM_setting_table));
			} else {
				push_table(ctx, aod_off_cmd_lpwm_90hz, sizeof(aod_off_cmd_lpwm_90hz) / sizeof(struct LCM_setting_table));
			}

		}
		usleep_range(9000, 9100);
		// OFP_INFO("send aod off cmd\n");
	}

	usleep_range(5000, 5100);
	lcm_dcs_write_seq_static(ctx, MIPI_DCS_SET_DISPLAY_OFF);
	usleep_range(25000, 25100);
	lcm_dcs_write_seq_static(ctx, MIPI_DCS_ENTER_SLEEP_MODE);
	usleep_range(125000, 125100);

	ctx->error = 0;
	ctx->prepared = false;
	pr_err("[LCM]%s-\n", __func__);

	return 0;
}

static int lcm_prepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	pr_err("[LCM]%s +\n", __func__);
	if (ctx->prepared)
		return 0;

	lcm_panel_init(ctx);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	ctx->prepared = true;
#ifdef PANEL_SUPPORT_READBACK
	lcm_panel_get_data(ctx);
#endif

	pr_info("%s-\n", __func__);
	return ret;
}

static int lcm_enable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = true;

	return 0;
}

static const struct drm_display_mode display_mode[MODE_NUM] = {
	//sdc_120_mode
	{
		.clock = 436550,
		.hdisplay = 1240,
		.hsync_start = 1240 + 9,//HFP
		.hsync_end = 1240 + 9 + 2,//HSA
		.htotal = 1240 + 9 + 2 + 21,//HBP
		.vdisplay = 2772,
		.vsync_start = 2772 + 52,//VFP
		.vsync_end = 2772 + 52 + 14,//VSA
		.vtotal = 2772 + 52+ 14 + 22,//VBP
		.hskew = SDC_ADFR,
	},
	//sdc_60_mode
	{
		.clock = 218275,
		.hdisplay = 1240,
		.hsync_start = 1240 + 9,//HFP
		.hsync_end = 1240 + 9 + 2,//HSA
		.htotal = 1240 + 9 + 2 + 21,//HBP
		.vdisplay = 2772,
		.vsync_start = 2772 + 52,//VFP
		.vsync_end = 2772 + 52 + 14,//VSA
		.vtotal = 2772 + 52+ 14 + 22,//VBP
		.hskew = SDC_MFR,
	},
	//sdc_90_mode
	{
		.clock = 327412,
		.hdisplay = 1240,
		.hsync_start = 1240 + 9,//HFP
		.hsync_end = 1240 + 9 + 2,//HSA
		.htotal = 1240 + 9 + 2 + 21,//HBP
		.vdisplay = 2772,
		.vsync_start = 2772 + 52,//VFP
		.vsync_end = 2772 + 52 + 14,//VSA
		.vtotal = 2772 + 52+ 14 + 22,//VBP
		.hskew = SDC_ADFR,
	},
	//oa_120_mode
	{
		.clock = 436550,
		.hdisplay = 1240,
		.hsync_start = 1240 + 9,//HFP
		.hsync_end = 1240 + 9 + 2,//HSA
		.htotal = 1240 + 9 + 2 + 21,//HBP
		.vdisplay = 2772,
		.vsync_start = 2772 + 52,//VFP
		.vsync_end = 2772 + 52 + 14,//VSA
		.vtotal = 2772 + 52+ 14 + 22,//VBP
		.hskew = OPLUS_ADFR,
	},
};

#if defined(CONFIG_MTK_PANEL_EXT)
static struct mtk_panel_params ext_params[MODE_NUM] = {
	//fhd_sdc_120_mode
	{
	.pll_clk = 556,
	.phy_timcon = {
		.hs_trail = 14,
		.clk_trail = 15,
	},
	.cust_esd_check = 0,
	.esd_check_enable = 0,
	//.esd_check_multi = 0,
	.lcm_esd_check_table[0] = {
                .cmd = 0x0A, .count = 1, .para_list[0] = 0x9c,
        },
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 0,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size_l = sizeof(top_rc_pattern_l),
	.corner_pattern_lt_addr_l = (void *)top_rc_pattern_l,
	.corner_pattern_tp_size_r =  sizeof(top_rc_pattern_r),
	.corner_pattern_lt_addr_r = (void *)top_rc_pattern_r,
#endif
	.cmd_null_pkt_en = 1,
	.cmd_null_pkt_len = 0,
#if 0
	//.color_vivid_status = true,
	.color_srgb_status = true,
	.color_softiris_status = false,
	.color_dual_panel_status = false,
	.color_dual_brightness_status = true,
	.color_oplus_calibrate_status = true,
	.vendor = "Tianma_NT37705",
	.manufacture = "Tianma4095",
#endif
	.lane_swap_en = 0,
	.lane_swap[0][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
	.lane_swap[0][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
	.lane_swap[0][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
	.lane_swap[0][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
	.lane_swap[0][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
	.lane_swap[0][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
	.lane_swap[1][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
	.lane_swap[1][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
	.lane_swap[1][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
	.lane_swap[1][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
	.lane_swap[1][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
	.lane_swap[1][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.dsc_params = {
			.enable = 1,
			.ver = 18,
			.slice_mode = 1,
			.rgb_swap = 0,
			.dsc_cfg = 40,
			.rct_on = 1,
			.bit_per_channel = 10,
			.dsc_line_buf_depth = 11,
			.bp_enable = 1,
			.bit_per_pixel = 128,
			.pic_height = 2772,
			.pic_width = 1240,
			.slice_height = 12,
			.slice_width = 620,
			.chunk_size = 620,
			.xmit_delay = 512,
			.dec_delay = 593,
			.scale_value = 32,
			.increment_interval = 294,
			.decrement_interval = 8,
			.line_bpg_offset = 13,
			.nfl_bpg_offset = 2421,
			.slice_bpg_offset = 1887,
			.initial_offset = 6144,
			.final_offset = 4336,
			.flatness_minqp = 7,
			.flatness_maxqp = 16,
			.rc_model_size = 8192,
			.rc_edge_factor = 6,
			.rc_quant_incr_limit0 = 15,
			.rc_quant_incr_limit1 = 15,
			.rc_tgt_offset_hi = 3,
			.rc_tgt_offset_lo = 3,
		},
	.data_rate = 1112,
#if 0
	.oplus_mode_switch_hs = 1,
	.oplus_serial_para0 = 0xD7,

	.oplus_fakeframe_cfg = 0,
	.oplus_fakeframe_deferred_time = 0,
	.oplus_autoon_cfg = 0,
	.oplus_autooff_cfg = 0,
	.oplus_minfps0_cfg = 1,
	.oplus_minfps1_cfg = 0,
#endif
#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	.oplus_ofp_need_keep_apart_backlight = true,
	.oplus_ofp_hbm_on_delay = 8,
	.oplus_ofp_pre_hbm_off_delay = 3,
	.oplus_ofp_hbm_off_delay = 0,
	.oplus_ofp_need_to_sync_data_in_aod_unlocking = true,
	.oplus_ofp_aod_off_insert_black = 1,
	.oplus_ofp_aod_off_black_frame_total_time = 42,
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
	.dyn_fps = {
		.switch_en = 1, .vact_timing_fps = 120,
		},
	//.prefetch_offset = 233,
	},
	//fhd_sdc_60_mode
	{
	.pll_clk = 556,
	.phy_timcon = {
		.hs_trail = 14,
		.clk_trail = 15,
	},
	.cust_esd_check = 0,
	.esd_check_enable = 0,
	//.esd_check_multi = 0,
	.lcm_esd_check_table[0] = {
                .cmd = 0x0A, .count = 1, .para_list[0] = 0x9c,
        },
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 0,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size_l = sizeof(top_rc_pattern_l),
	.corner_pattern_lt_addr_l = (void *)top_rc_pattern_l,
	.corner_pattern_tp_size_r =  sizeof(top_rc_pattern_r),
	.corner_pattern_lt_addr_r = (void *)top_rc_pattern_r,
#endif
	.cmd_null_pkt_en = 1,
	.cmd_null_pkt_len = 400,
#if 0
	//.color_vivid_status = true,
	.color_srgb_status = true,
	.color_softiris_status = false,
	.color_dual_panel_status = false,
	.color_dual_brightness_status = true,
	.color_oplus_calibrate_status = true,
	.vendor = "Tianma_NT37705",
	.manufacture = "Tianma4095",
#endif
	.lane_swap_en = 0,
	.lane_swap[0][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
	.lane_swap[0][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
	.lane_swap[0][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
	.lane_swap[0][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
	.lane_swap[0][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
	.lane_swap[0][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
	.lane_swap[1][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
	.lane_swap[1][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
	.lane_swap[1][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
	.lane_swap[1][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
	.lane_swap[1][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
	.lane_swap[1][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.dsc_params = {
			.enable = 1,
			.ver = 18,
			.slice_mode = 1,
			.rgb_swap = 0,
			.dsc_cfg = 40,
			.rct_on = 1,
			.bit_per_channel = 10,
			.dsc_line_buf_depth = 11,
			.bp_enable = 1,
			.bit_per_pixel = 128,
			.pic_height = 2772,
			.pic_width = 1240,
			.slice_height = 12,
			.slice_width = 620,
			.chunk_size = 620,
			.xmit_delay = 512,
			.dec_delay = 593,
			.scale_value = 32,
			.increment_interval = 294,
			.decrement_interval = 8,
			.line_bpg_offset = 13,
			.nfl_bpg_offset = 2421,
			.slice_bpg_offset = 1887,
			.initial_offset = 6144,
			.final_offset = 4336,
			.flatness_minqp = 7,
			.flatness_maxqp = 16,
			.rc_model_size = 8192,
			.rc_edge_factor = 6,
			.rc_quant_incr_limit0 = 15,
			.rc_quant_incr_limit1 = 15,
			.rc_tgt_offset_hi = 3,
			.rc_tgt_offset_lo = 3,
		},
	.data_rate = 1112,
#if 0
	.oplus_mode_switch_hs = 1,
	.oplus_serial_para0 = 0xD7,
	.oplus_fakeframe_cfg = 0,
	.oplus_fakeframe_deferred_time = 0,
	.oplus_autoon_cfg = 0,
	.oplus_autooff_cfg = 0,
	.oplus_minfps0_cfg = 1,
	.oplus_minfps1_cfg = 0,
#endif
#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	.oplus_ofp_need_keep_apart_backlight = false,
	.oplus_ofp_hbm_on_delay = 0,
	.oplus_ofp_pre_hbm_off_delay = 2,
	.oplus_ofp_hbm_off_delay = 0,
	.oplus_ofp_need_to_sync_data_in_aod_unlocking = true,
	.oplus_ofp_aod_off_insert_black = 2,
	.oplus_ofp_aod_off_black_frame_total_time = 59,
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
	.dyn_fps = {
		.switch_en = 1, .vact_timing_fps = 60,
		},
	//.prefetch_offset = 466,
	},
	//fhd 90hz
    {
	.pll_clk = 414,
	.phy_timcon = {
		.hs_trail = 14,
		.clk_trail = 15,
	},
	.cust_esd_check = 0,
	.esd_check_enable = 0,
	//.esd_check_multi = 0,
	.lcm_esd_check_table[0] = {
                .cmd = 0x0A, .count = 1, .para_list[0] = 0x9c,
        },
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 0,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size_l = sizeof(top_rc_pattern_l),
	.corner_pattern_lt_addr_l = (void *)top_rc_pattern_l,
	.corner_pattern_tp_size_r =  sizeof(top_rc_pattern_r),
	.corner_pattern_lt_addr_r = (void *)top_rc_pattern_r,
#endif
	.cmd_null_pkt_en = 1,
	.cmd_null_pkt_len = 0,
#if 0
	//.color_vivid_status = true,
	.color_srgb_status = true,
	.color_softiris_status = false,
	.color_dual_panel_status = false,
	.color_dual_brightness_status = true,
	.color_oplus_calibrate_status = true,
	.vendor = "Tianma_NT37705",
	.manufacture = "Tianma4095",
#endif
	.lane_swap_en = 0,
	.lane_swap[0][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
	.lane_swap[0][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
	.lane_swap[0][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
	.lane_swap[0][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
	.lane_swap[0][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
	.lane_swap[0][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
	.lane_swap[1][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
	.lane_swap[1][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
	.lane_swap[1][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
	.lane_swap[1][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
	.lane_swap[1][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
	.lane_swap[1][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.dsc_params = {
			.enable = 1,
			.ver = 18,
			.slice_mode = 1,
			.rgb_swap = 0,
			.dsc_cfg = 40,
			.rct_on = 1,
			.bit_per_channel = 10,
			.dsc_line_buf_depth = 11,
			.bp_enable = 1,
			.bit_per_pixel = 128,
			.pic_height = 2772,
			.pic_width = 1240,
			.slice_height = 12,
			.slice_width = 620,
			.chunk_size = 620,
			.xmit_delay = 512,
			.dec_delay = 593,
			.scale_value = 32,
			.increment_interval = 294,
			.decrement_interval = 8,
			.line_bpg_offset = 13,
			.nfl_bpg_offset = 2421,
			.slice_bpg_offset = 1887,
			.initial_offset = 6144,
			.final_offset = 4336,
			.flatness_minqp = 7,
			.flatness_maxqp = 16,
			.rc_model_size = 8192,
			.rc_edge_factor = 6,
			.rc_quant_incr_limit0 = 15,
			.rc_quant_incr_limit1 = 15,
			.rc_tgt_offset_hi = 3,
			.rc_tgt_offset_lo = 3,
		},
	.data_rate = 828,
#if 0
	.oplus_mode_switch_hs = 1,
	.oplus_serial_para0 = 0xD7,
	.oplus_fakeframe_cfg = 0,
	.oplus_fakeframe_deferred_time = 0,
	.oplus_autoon_cfg = 0,
	.oplus_autooff_cfg = 0,
	.oplus_minfps0_cfg = 1,
	.oplus_minfps1_cfg = 0,
#endif
#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	.oplus_ofp_need_keep_apart_backlight = true,
	.oplus_ofp_hbm_on_delay = 11,
	.oplus_ofp_pre_hbm_off_delay = 5,
	.oplus_ofp_hbm_off_delay = 0,
	.oplus_ofp_need_to_sync_data_in_aod_unlocking = true,
	.oplus_ofp_aod_off_insert_black = 1,
	.oplus_ofp_aod_off_black_frame_total_time = 45,
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
	.dyn_fps = {
		.switch_en = 1, .vact_timing_fps = 90,
		},
	//.prefetch_offset = 211,
	},
	//fhd_oa_120_mode
	{
	.pll_clk = 556,
	.phy_timcon = {
		.hs_trail = 14,
		.clk_trail = 15,
	},
	.cust_esd_check = 0,
	.esd_check_enable = 0,
	//.esd_check_multi = 0,
	.lcm_esd_check_table[0] = {
                .cmd = 0x0A, .count = 1, .para_list[0] = 0x9c,
        },
#ifdef CONFIG_MTK_ROUND_CORNER_SUPPORT
	.round_corner_en = 0,
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_height_bot = ROUND_CORNER_H_BOT,
	.corner_pattern_tp_size_l = sizeof(top_rc_pattern_l),
	.corner_pattern_lt_addr_l = (void *)top_rc_pattern_l,
	.corner_pattern_tp_size_r =  sizeof(top_rc_pattern_r),
	.corner_pattern_lt_addr_r = (void *)top_rc_pattern_r,
#endif
	.cmd_null_pkt_en = 1,
	.cmd_null_pkt_len = 0,
#if 0
	//.color_vivid_status = true,
	.color_srgb_status = true,
	.color_softiris_status = false,
	.color_dual_panel_status = false,
	.color_dual_brightness_status = true,
	.color_oplus_calibrate_status = true,
	.vendor = "Tianma_NT37705",
	.manufacture = "Tianma4095",
#endif
	.lane_swap_en = 0,
	.lane_swap[0][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
	.lane_swap[0][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
	.lane_swap[0][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
	.lane_swap[0][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
	.lane_swap[0][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
	.lane_swap[0][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
	.lane_swap[1][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
	.lane_swap[1][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
	.lane_swap[1][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
	.lane_swap[1][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
	.lane_swap[1][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
	.lane_swap[1][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.dsc_params = {
			.enable = 1,
			.ver = 18,
			.slice_mode = 1,
			.rgb_swap = 0,
			.dsc_cfg = 40,
			.rct_on = 1,
			.bit_per_channel = 10,
			.dsc_line_buf_depth = 11,
			.bp_enable = 1,
			.bit_per_pixel = 128,
			.pic_height = 2772,
			.pic_width = 1240,
			.slice_height = 12,
			.slice_width = 620,
			.chunk_size = 620,
			.xmit_delay = 512,
			.dec_delay = 593,
			.scale_value = 32,
			.increment_interval = 294,
			.decrement_interval = 8,
			.line_bpg_offset = 13,
			.nfl_bpg_offset = 2421,
			.slice_bpg_offset = 1887,
			.initial_offset = 6144,
			.final_offset = 4336,
			.flatness_minqp = 7,
			.flatness_maxqp = 16,
			.rc_model_size = 8192,
			.rc_edge_factor = 6,
			.rc_quant_incr_limit0 = 15,
			.rc_quant_incr_limit1 = 15,
			.rc_tgt_offset_hi = 3,
			.rc_tgt_offset_lo = 3,
		},
	.data_rate = 1112,
#if 0
	.oplus_mode_switch_hs = 1,
	.oplus_serial_para0 = 0xD7,
	.oplus_fakeframe_cfg = 0,
	.oplus_fakeframe_deferred_time = 0,
	.oplus_autoon_cfg = 0,
	.oplus_autooff_cfg = 0,
	.oplus_minfps0_cfg = 1,
	.oplus_minfps1_cfg = 0,
#endif
#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	.oplus_ofp_need_keep_apart_backlight = true,
	.oplus_ofp_hbm_on_delay = 8,
	.oplus_ofp_pre_hbm_off_delay = 3,
	.oplus_ofp_hbm_off_delay = 0,
	.oplus_ofp_need_to_sync_data_in_aod_unlocking = true,
	.oplus_ofp_aod_off_insert_black = 1,
	.oplus_ofp_aod_off_black_frame_total_time = 42,
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
	.dyn_fps = {
		.switch_en = 1, .vact_timing_fps = 120,
		},
	},
};

static int panel_ata_check(struct drm_panel *panel)
{
	/* Customer test by own ATA tool */
	return 1;
}

void oplus_display_panel_set_frequency_pwm(void *dsi, dcs_write_gce cb, void *handle, unsigned int bl_lvl)
{
	unsigned int i = 0;
	if ((bl_lvl <= 0x643 && bl_record > 0x643) || (pwm_power_on == true && bl_lvl <= 0x643)) {
		pr_info("pwm_turbo dsi_high_12plus backlight level=%d, bl_record=%d\n", bl_lvl, bl_record);
		for (i = 0; i < sizeof(dsi_high_12plus)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, dsi_high_12plus[i].para_list, dsi_high_12plus[i].count);
		}
		bl_record = bl_lvl;
		pwm_power_on = false;
	} else if ((bl_lvl > 0x643 && bl_record <= 0x643) || (pwm_power_on == true && bl_lvl > 0x643)) {
		pr_info("pwm_turbo dsi_low_3plus backlight level=%d, bl_record=%d\n", bl_lvl, bl_record);
		for (i = 0; i < sizeof(dsi_low_3plus)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, dsi_low_3plus[i].para_list, dsi_low_3plus[i].count);
		}
		bl_record = bl_lvl;
		pwm_power_on = false;
	}
}

#ifndef OPLUS_FEATURE_DISPLAY_HPWM
static int oplus_display_panel_set_pwm_turbo(struct drm_panel *panel, void *dsi, dcs_write_gce_pack cb, void *handle, bool en_h_pwm)
{
	unsigned int dbv = 0;
	dbv = oplus_display_brightness;

	pr_info("pwm_turbo backlight level=%d, pwm_en=%d\n", dbv, en_h_pwm);
	oplus_set_dbv_frame(dsi, cb, handle, en_h_pwm);
	return 0;
}


static int oplus_display_panel_set_pwm_fps(void *dsi, dcs_write_gce_pack cb, void *handle, int fps, bool en_h_pwm)
{
	pr_info("pwm_turbo oplus_display_panel_set_pwm_fps fps=%d, pwm_en=%d\n", fps, en_h_pwm);
	if (en_h_pwm) {
		if (fps == 60) {
			cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(300), CMDQ_GPR_R06);
			panel_send_pack_hs_cmd(dsi, timing_switch_cmd_high_pwm_sdc60_part1, sizeof(timing_switch_cmd_high_pwm_sdc60_part1) / sizeof(struct LCM_setting_table), cb, handle);
			cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(8500), CMDQ_GPR_R06);
			panel_send_pack_hs_cmd(dsi, timing_switch_cmd_high_pwm_sdc60, sizeof(timing_switch_cmd_high_pwm_sdc60) / sizeof(struct LCM_setting_table), cb, handle);
		} else {
			panel_send_pack_hs_cmd(dsi, timing_switch_cmd_high_pwm_sdc120_part1, sizeof(timing_switch_cmd_high_pwm_sdc120_part1) / sizeof(struct LCM_setting_table), cb, handle);
			cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(8500), CMDQ_GPR_R06);
			panel_send_pack_hs_cmd(dsi, timing_switch_cmd_high_pwm_sdc120, sizeof(timing_switch_cmd_high_pwm_sdc120) / sizeof(struct LCM_setting_table), cb, handle);
		}
	} else {
		if (fps == 60) {
			cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(300), CMDQ_GPR_R06);
			panel_send_pack_hs_cmd(dsi, timing_switch_cmd_sdc60_part1, sizeof(timing_switch_cmd_sdc60_part1) / sizeof(struct LCM_setting_table), cb, handle);
			cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(8500), CMDQ_GPR_R06);
			panel_send_pack_hs_cmd(dsi, timing_switch_cmd_sdc60, sizeof(timing_switch_cmd_sdc60) / sizeof(struct LCM_setting_table), cb, handle);
		} else {
			panel_send_pack_hs_cmd(dsi, timing_switch_cmd_sdc120_part1, sizeof(timing_switch_cmd_sdc120_part1) / sizeof(struct LCM_setting_table), cb, handle);
			cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(8500), CMDQ_GPR_R06);
			panel_send_pack_hs_cmd(dsi, timing_switch_cmd_sdc120, sizeof(timing_switch_cmd_sdc120) / sizeof(struct LCM_setting_table), cb, handle);
		}
	}
	return 0;
}

static int oplus_display_panel_set_pwm_plus_bl(void *dsi, dcs_write_gce_pack cb, void *handle, unsigned int bl_lvl)
{
	if (bl_lvl <= 0x643) {
		dsi_high_12plus_bl[6].para_list[1] = bl_lvl >> 8;
		dsi_high_12plus_bl[6].para_list[2] = bl_lvl & 0xFF;
		pr_info("pwm_turbo dsi_high_12plus_bl backlight level=%d, last_backlight=%d\n", bl_lvl, last_backlight);
		panel_send_pack_hs_cmd(dsi, dsi_high_12plus_bl, sizeof(dsi_high_12plus_bl) / sizeof(struct LCM_setting_table), cb, handle);
	} else if (bl_lvl > 0x643) {
		dsi_low_3plus_bl[6].para_list[1]  = bl_lvl >> 8;
		dsi_low_3plus_bl[6].para_list[2]  = bl_lvl & 0xFF;
		pr_info("pwm_turbo dsi_low_3plus_bl backlight level=%d, last_backlight=%d\n", bl_lvl, last_backlight);
		panel_send_pack_hs_cmd(dsi, dsi_low_3plus_bl, sizeof(dsi_low_3plus_bl) / sizeof(struct LCM_setting_table), cb, handle);
	}
	return 0;
}
#endif

static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb, void *handle, unsigned int level)
{
	unsigned int mapped_level = 0;
	unsigned char bl_level[] = {0x51, 0x03, 0xFF};

	pr_debug("[DISP][DEBUG][%s:%d]start\n", __func__, __LINE__);

	if (!dsi || !cb) {
		return -EINVAL;
	}

	last_backlight = level;

	if (level == 1) {
		pr_info("[DISP][INFO][%s:%d]filter backlight %u setting\n", __func__, __LINE__, level);
		return 0;
	} else if (level > 4095) {
		level = 4095;
	}

	//if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT && level > 0){
	//	level = 2047;
	//}

	pr_info("[DISP][INFO][%s:%d]backlight lvl:%u\n", __func__, __LINE__, level);

	mapped_level = level;
	if (mapped_level > 1) {
		//lcdinfo_notify(LCM_BRIGHTNESS_TYPE, &mapped_level);
	}

	bl_level[1] = level >> 8;
	bl_level[2] = level & 0xFF;
	cb(dsi, handle, bl_level, ARRAY_SIZE(bl_level));

	pr_debug("[DISP][DEBUG][%s:%d]end\n", __func__, __LINE__);

	return 0;
}

static int oplus_esd_backlight_recovery(void *dsi, dcs_write_gce cb, void *handle)
{
	unsigned int level = oplus_display_brightness;
	unsigned char bl_level[] = {0x51, 0x03, 0xFF};

	pr_debug("[DISP][DEBUG][%s:%d]start\n", __func__, __LINE__);

	if (!dsi || !cb) {
		return -EINVAL;
	}

	if (level == 1) {
		pr_info("[DISP][INFO][%s:%d]filter backlight %u setting\n", __func__, __LINE__, level);
		return 0;
	} else if (level > 4095) {
		level = 4095;
	}

	pr_info("[DISP][INFO][%s:%d]backlight lvl:%u\n", __func__, __LINE__, level);

	bl_level[1] = oplus_display_brightness >> 8;
	bl_level[2] = oplus_display_brightness & 0xFF;
	cb(dsi, handle, bl_level, ARRAY_SIZE(bl_level));

	pr_debug("[DISP][DEBUG][%s:%d]end\n", __func__, __LINE__);

	return 0;
}

#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
static int lcm_set_hbm(void *dsi, dcs_write_gce cb,
		void *handle, unsigned int hbm_mode)
{
	unsigned int i = 0;

	OFP_DEBUG("start\n");

	if (!dsi || !cb) {
		OFP_ERR("Invalid params\n");
		return -EINVAL;
	}

	// OFP_INFO("hbm_mode:%u,bl_lvl:%u\n", hbm_mode, oplus_display_brightness);

	if(hbm_mode == 1) {
		for (i = 0; i < sizeof(hbm_on_cmd_60hz)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, hbm_on_cmd_60hz[i].para_list, hbm_on_cmd_60hz[i].count);
		}
	} else if (hbm_mode == 0) {
		for (i = 0; i < sizeof(hbm_off_cmd_60hz)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, hbm_off_cmd_60hz[i].para_list, hbm_off_cmd_60hz[i].count);
		}
		lcm_setbacklight_cmdq(dsi, cb, handle, oplus_display_brightness);

		if (get_pwm_status() && (oplus_display_brightness != 1)) {
			pwm_power_on = true;
			oplus_display_panel_set_frequency_pwm(dsi, cb, handle, oplus_display_brightness);
		}
	}

	OFP_DEBUG("end\n");

	return 0;
}

static int panel_hbm_set_cmdq(struct drm_panel *panel, void *dsi,
		dcs_write_gce cb, void *handle, bool en)
{
	unsigned int i = 0;
	unsigned int reg_count = 0;
	unsigned int vrefresh_rate = 0;
	struct lcm *ctx = NULL;
	struct LCM_setting_table *hbm_cmd = NULL;

	OFP_DEBUG("start\n");

	if (!panel || !dsi || !cb) {
		OFP_ERR("Invalid input params\n");
		return -EINVAL;
	}

	ctx = panel_to_lcm(panel);
	if (!ctx) {
		OFP_ERR("Invalid lcm params\n");
	}

	if (!ctx->m) {
		vrefresh_rate = 120;
		// OFP_INFO("default refresh rate is 120hz\n");
	} else {
		vrefresh_rate = drm_mode_vrefresh(ctx->m);
	}

	// OFP_INFO("hbm_en:%u,bl_lvl:%u,refresh_rate:%u\n", en, oplus_display_brightness, vrefresh_rate);

	if (vrefresh_rate == 60) {
		if (en) {
			hbm_cmd = hbm_on_cmd_60hz;
			reg_count = sizeof(hbm_on_cmd_60hz) / sizeof(struct LCM_setting_table);
		} else {
			hbm_cmd = hbm_off_cmd_60hz;
			reg_count = sizeof(hbm_off_cmd_60hz) / sizeof(struct LCM_setting_table);
		}
	} else if (vrefresh_rate == 90) {
		if (en) {
			hbm_cmd = hbm_on_cmd_90hz;
			reg_count = sizeof(hbm_on_cmd_90hz) / sizeof(struct LCM_setting_table);
		} else {
			hbm_cmd = hbm_off_cmd_90hz;
			reg_count = sizeof(hbm_off_cmd_90hz) / sizeof(struct LCM_setting_table);
		}
	} else if (vrefresh_rate == 120) {
		if (en) {
			hbm_cmd = hbm_on_cmd_120hz;
			reg_count = sizeof(hbm_on_cmd_120hz) / sizeof(struct LCM_setting_table);
		} else {
			hbm_cmd = hbm_off_cmd_120hz;
			reg_count = sizeof(hbm_off_cmd_120hz) / sizeof(struct LCM_setting_table);
		}
	}

	for (i = 0; i < reg_count; i++) {
		unsigned int cmd;
		cmd = hbm_cmd[i].cmd;

		switch (cmd) {
			case REGFLAG_DELAY:
				if (!handle) {
					usleep_range(hbm_cmd[i].count * 1000, hbm_cmd[i].count * 1000 + 100);
				} else {
					cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(hbm_cmd[i].count * 1000), CMDQ_GPR_R14);
				}
				break;
			case REGFLAG_UDELAY:
				if (!handle) {
					usleep_range(hbm_cmd[i].count, hbm_cmd[i].count + 100);
				} else {
					cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(hbm_cmd[i].count), CMDQ_GPR_R14);
				}
				break;
			case REGFLAG_END_OF_TABLE:
				break;
			default:
				cb(dsi, handle, hbm_cmd[i].para_list, hbm_cmd[i].count);
		}
	}

	if (!en) {
		lcm_setbacklight_cmdq(dsi, cb, handle, oplus_display_brightness);

		if (get_pwm_status() && (oplus_display_brightness != 1)) {
			pwm_power_on = true;
			oplus_display_panel_set_frequency_pwm(dsi, cb, handle, oplus_display_brightness);
		}
	}

	OFP_DEBUG("end\n");

	return 0;
}

static int panel_doze_disable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	unsigned int i = 0;
	unsigned int reg_count = 0;
	int vrefresh_rate = 0;
	struct lcm *ctx = NULL;
	struct LCM_setting_table *aod_off_cmd = NULL;

	if (!panel || !dsi) {
		OFP_ERR("Invalid dsi params\n");
	}

	ctx = panel_to_lcm(panel);
	if (!ctx) {
		OFP_ERR("Invalid lcm params\n");
	}

	if (!ctx->m) {
		vrefresh_rate = 120;
		// OFP_INFO("default refresh rate is 120hz\n");
	} else {
		vrefresh_rate = drm_mode_vrefresh(ctx->m);
	}

	if (get_pwm_status()) {
		if (vrefresh_rate == 60) {
			aod_off_cmd = aod_off_cmd_hpwm_60hz;
			reg_count = sizeof(aod_off_cmd_hpwm_60hz) / sizeof(struct LCM_setting_table);
		} else {
			aod_off_cmd = aod_off_cmd_hpwm_120hz;
			reg_count = sizeof(aod_off_cmd_hpwm_120hz) / sizeof(struct LCM_setting_table);
		}
	} else {
		if (vrefresh_rate == 60) {
			aod_off_cmd = aod_off_cmd_lpwm_60hz;
			reg_count = sizeof(aod_off_cmd_lpwm_60hz) / sizeof(struct LCM_setting_table);
		} else if (vrefresh_rate == 120) {
			aod_off_cmd = aod_off_cmd_lpwm_120hz;
			reg_count = sizeof(aod_off_cmd_lpwm_120hz) / sizeof(struct LCM_setting_table);
		} else {
			aod_off_cmd = aod_off_cmd_lpwm_90hz;
			reg_count = sizeof(aod_off_cmd_lpwm_90hz) / sizeof(struct LCM_setting_table);
		}
	}

	for (i = 0; i < reg_count; i++) {
		unsigned int cmd;
		cmd = aod_off_cmd[i].cmd;

		switch (cmd) {
			case REGFLAG_DELAY:
				if (handle == NULL) {
					usleep_range(aod_off_cmd[i].count * 1000, aod_off_cmd[i].count * 1000 + 100);
				} else {
					cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(aod_off_cmd[i].count * 1000), CMDQ_GPR_R14);
				}
				break;
			case REGFLAG_UDELAY:
				if (handle == NULL) {
					usleep_range(aod_off_cmd[i].count, aod_off_cmd[i].count + 100);
				} else {
					cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(aod_off_cmd[i].count), CMDQ_GPR_R14);
				}
				break;
			case REGFLAG_END_OF_TABLE:
				break;
			default:
				cb(dsi, handle, aod_off_cmd[i].para_list, aod_off_cmd[i].count);
		}
	}

	lcm_setbacklight_cmdq(dsi, cb, handle, last_backlight);

	if (get_pwm_status()) {
		pwm_power_on = true;
		oplus_display_panel_set_frequency_pwm(dsi, cb, handle, oplus_display_brightness);
	}

	//OFP_INFO("send aod off cmd\n");

	return 0;
}


static int panel_doze_enable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	unsigned int i = 0;

	for (i = 0; i < (sizeof(aod_on_cmd)/sizeof(struct LCM_setting_table)); i++) {
		unsigned int cmd;
		cmd = aod_on_cmd[i].cmd;

		switch (cmd) {
			case REGFLAG_DELAY:
				usleep_range(aod_on_cmd[i].count * 1000, aod_on_cmd[i].count * 1000 + 100);
				break;
			case REGFLAG_UDELAY:
				usleep_range(aod_on_cmd[i].count, aod_on_cmd[i].count + 100);
				break;
			case REGFLAG_END_OF_TABLE:
				break;
			default:
				cb(dsi, handle, aod_on_cmd[i].para_list, aod_on_cmd[i].count);
		}
	}

	// OFP_INFO("send aod on cmd\n");

	return 0;
}

static int panel_set_aod_light_mode(void *dsi, dcs_write_gce cb, void *handle, unsigned int level)
{
	int i = 0;

	if (level == 0) {
		for (i = 0; i < sizeof(aod_high_mode)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, aod_high_mode[i].para_list, aod_high_mode[i].count);
		}
	} else {
		for (i = 0; i < sizeof(aod_low_mode)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, aod_low_mode[i].para_list, aod_low_mode[i].count);
		}
	}
	// OFP_INFO("level = %d\n", level);

	return 0;
}
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */

static int panel_ext_reset(struct drm_panel *panel, int on)
{
	struct lcm *ctx = panel_to_lcm(panel);

	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_info(ctx->dev, "%s: cannot get reset_gpio %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	gpiod_set_value(ctx->reset_gpio, on);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	return 0;
}

static int lcm_panel_poweron(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	if (ctx->prepared)
		return 0;

	pr_err("[LCM]debug %s, ctx->prepared %d\n", __func__, ctx->prepared);

	lcm_panel_vrfio18_aif_enable(ctx->dev);
	usleep_range(5000, 5010);
	ctx->vddr1p2_enable_gpio = devm_gpiod_get(ctx->dev, "pm-enable", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->vddr1p2_enable_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->vddr1p2_enable_gpio);
	usleep_range(5000, 5010);
	lcm_panel_vmc_ldo_enable(ctx->dev);
	usleep_range(21000, 21100);


	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	usleep_range(1000, 1100);
	return 0;
}

static int lcm_panel_poweroff(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	if (ctx->prepared)
		return 0;

	pr_err("[LCM]debug lcm %s  ctx->prepared %d \n", __func__,ctx->prepared);
	usleep_range(10000, 10100);
	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	usleep_range(5000, 5100);
	/* set vddi 3.0v */
	lcm_panel_vmc_ldo_disable(ctx->dev);
	usleep_range(10000, 10100);;
	ctx->vddr1p2_enable_gpio = devm_gpiod_get(ctx->dev, "pm-enable", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->vddr1p2_enable_gpio, 0);
	devm_gpiod_put(ctx->dev, ctx->vddr1p2_enable_gpio);
	usleep_range(10000, 10100);;
	lcm_panel_vrfio18_aif_disable(ctx->dev);
	usleep_range(2000, 2100);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	usleep_range(50000, 50100);
	return 0;
}

static int lcm_panel_reset(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (ctx->prepared)
               return 0;

	pr_info("[LCM]debug for lcd reset :%s, ctx->prepared:%d\n", __func__, ctx->prepared);

	usleep_range(5000, 5100);
	gpiod_set_value(ctx->reset_gpio,1);
	usleep_range(3000, 3100);
	gpiod_set_value(ctx->reset_gpio,0);
	usleep_range(5000, 5100);
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(15000, 15100);

	return 0;
}

struct drm_display_mode *get_mode_by_id(struct drm_connector *connector,
	unsigned int mode)
{
	struct drm_display_mode *m;
	unsigned int i = 0;

	list_for_each_entry(m, &connector->modes, head) {
		if (i == mode)
			return m;
		i++;
	}
	return NULL;
}

static int mtk_panel_ext_param_get(struct drm_panel *panel,
		struct drm_connector *connector,
		struct mtk_panel_params **ext_param,
		unsigned int id)
{
	int ret = 0;
	int m_vrefresh = 0;
	struct drm_display_mode *m = get_mode_by_id(connector, id);

	m_vrefresh = drm_mode_vrefresh(m);
	pr_err("[LCM] mode:%d,m_vrefresh:%d\n",id,m_vrefresh);

	if (m_vrefresh == 60 && m->hskew == SDC_MFR) {
		*ext_param = &ext_params[1];
	}
	else if (m_vrefresh == 120 && m->hskew == SDC_ADFR) {
		*ext_param = &ext_params[0];
	} else if (m_vrefresh == 90 && m->hskew == SDC_ADFR) {
		*ext_param = &ext_params[2];
	} else if (m_vrefresh == 120 && m->hskew == OPLUS_ADFR) {
		*ext_param = &ext_params[3];
	} else {
		*ext_param = &ext_params[0];
	}

	if (*ext_param)
		pr_err("[LCM] data_rate:%d\n", (*ext_param)->data_rate);
	else
		pr_err("[LCM] ext_param is NULL;\n");

	return ret;
}

static int mtk_panel_ext_param_set(struct drm_panel *panel,
			struct drm_connector *connector, unsigned int mode)
{
	struct mtk_panel_ext *ext = find_panel_ext(panel);
	int ret = 0;
	int m_vrefresh = 0;
	struct drm_display_mode *m = get_mode_by_id(connector, mode);

	m_vrefresh = drm_mode_vrefresh(m);
	pr_err("%s, mode=%d, vrefresh=%d, hskew=%d\n", __func__, mode, drm_mode_vrefresh(m), m->hskew);

	if (m_vrefresh == 60 && m->hskew == SDC_MFR) {
		ext->params = &ext_params[1];
	} else if (m_vrefresh == 120 && m->hskew == SDC_ADFR) {
		ext->params = &ext_params[0];
	} else if (m_vrefresh == 90 && m->hskew == SDC_ADFR) {
		ext->params = &ext_params[2];
	} else if (m_vrefresh == 120 && m->hskew == OPLUS_ADFR) {
		ext->params = &ext_params[3];
	} else {
		ext->params = &ext_params[0];
	}

	return ret;
}

static int panel_send_pack_hs_cmd(void *dsi, struct LCM_setting_table *table, unsigned int lcm_cmd_count, dcs_write_gce_pack cb, void *handle)
{
	unsigned int i = 0;
	struct mtk_ddic_dsi_cmd send_cmd_to_ddic;

	if(lcm_cmd_count > MAX_TX_CMD_NUM_PACK) {
		pr_err("out of mtk_ddic_dsi_cmd \n");
		return 0;
	}

	for (i = 0; i < lcm_cmd_count; i++) {
		send_cmd_to_ddic.mtk_ddic_cmd_table[i].cmd_num = table[i].count;
		send_cmd_to_ddic.mtk_ddic_cmd_table[i].para_list = table[i].para_list;
	}
	send_cmd_to_ddic.is_hs = 1;
	send_cmd_to_ddic.is_package = 1;
	send_cmd_to_ddic.cmd_count = lcm_cmd_count;
	cb(dsi, handle, &send_cmd_to_ddic);

	return 0;
}

static int mode_switch_hs(struct drm_panel *panel, struct drm_connector *connector,
		void *dsi_drv, unsigned int cur_mode, unsigned int dst_mode,
			enum MTK_PANEL_MODE_SWITCH_STAGE stage, dcs_write_gce_pack cb)
{
	int ret = 0;
	int m_vrefresh = 0;
	int src_vrefresh = 0;
	unsigned int lcm_cmd_count = 0;
	/* lk mipi setting is 830 */
	static int last_data_rate = 900;
	struct mtk_panel_ext *ext = find_panel_ext(panel);
	struct drm_display_mode *m = get_mode_by_id(connector, dst_mode);
	struct drm_display_mode *src_m = get_mode_by_id(connector, cur_mode);
	struct lcm *ctx = panel_to_lcm(panel);

	pr_info("%s cur_mode = %d dst_mode %d\n", __func__, cur_mode, dst_mode);
	if (cur_mode == dst_mode)
		return ret;

	if (stage == BEFORE_DSI_POWERDOWN) {
		m_vrefresh = drm_mode_vrefresh(m);
		src_vrefresh = drm_mode_vrefresh(src_m);
		pr_info("mode_switch_hs,cur_mode:%d,dst_mode:%d,hdisplay:%d->%d,vrefresh:%d->%d,hskew:%d->%d\n",
			cur_mode, dst_mode, src_m->hdisplay, m->hdisplay, src_vrefresh, m_vrefresh, src_m->hskew, m->hskew);

		if (m->hskew == SDC_MFR || m->hskew == SDC_ADFR) {
			if (m_vrefresh == 60) {
				if (get_pwm_status()) {
					if (src_vrefresh == 120) {
						usleep_range(200, 300);
					}
					pr_info("timing switch to high pwm sdc60\n");
					lcm_cmd_count = sizeof(timing_switch_cmd_high_pwm_sdc60_part1) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_high_pwm_sdc60_part1, lcm_cmd_count, cb, NULL);
					if(src_vrefresh == 90) {
						usleep_range(3500, 3600);
					} else if (src_vrefresh == 120) {
						usleep_range(8400, 8500);
					}
					lcm_cmd_count = sizeof(timing_switch_cmd_high_pwm_sdc60) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_high_pwm_sdc60, lcm_cmd_count, cb, NULL);
				} else {
					if (src_vrefresh == 120) {
						usleep_range(200, 300);
					}
					pr_info("timing switch to sdc60\n");
					lcm_cmd_count = sizeof(timing_switch_cmd_sdc60_part1) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc60_part1, lcm_cmd_count, cb, NULL);
					if(src_vrefresh == 90) {
						usleep_range(3500, 3600);
					} else if (src_vrefresh == 120) {
						usleep_range(8400, 8500);
					}
					lcm_cmd_count = sizeof(timing_switch_cmd_sdc60) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc60, lcm_cmd_count, cb, NULL);
				}
			} else if (m_vrefresh == 90) {
				if (src_vrefresh == 120) {
					usleep_range(200, 300);
				}
				pr_info("timing switch to sdc90\n");
				lcm_cmd_count = sizeof(timing_switch_cmd_sdc90_part1) / sizeof(struct LCM_setting_table);
				panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc90_part1, lcm_cmd_count, cb, NULL);
				usleep_range(8400, 8500);
				lcm_cmd_count = sizeof(timing_switch_cmd_sdc90) / sizeof(struct LCM_setting_table);
				panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc90, lcm_cmd_count, cb, NULL);
			} else if (m_vrefresh == 120) {
				if (src_vrefresh == 90) {
						usleep_range(500, 600);
				}
				if (get_pwm_status()) {
					pr_info("timing switch to high pwm sdc120\n");
					lcm_cmd_count = sizeof(timing_switch_cmd_high_pwm_sdc120_part1) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_high_pwm_sdc120_part1, lcm_cmd_count, cb, NULL);
					if((src_vrefresh == 60) || (src_vrefresh == 120)) {
						usleep_range(8400, 8500);
					} else if (src_vrefresh == 90) {
						usleep_range(3800, 3900);
					}
					lcm_cmd_count = sizeof(timing_switch_cmd_high_pwm_sdc120) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_high_pwm_sdc120, lcm_cmd_count, cb, NULL);
				} else {
					if (src_vrefresh == 90) {
						usleep_range(500, 600);
					}
					pr_info("timing switch to sdc120\n");
					lcm_cmd_count = sizeof(timing_switch_cmd_sdc120_part1) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc120_part1, lcm_cmd_count, cb, NULL);
					if((src_vrefresh == 60) || (src_vrefresh == 120)) {
						usleep_range(8400, 8500);
					} else if (src_vrefresh == 90) {
						usleep_range(3800, 3900);
					}
					lcm_cmd_count = sizeof(timing_switch_cmd_sdc120) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_sdc120, lcm_cmd_count, cb, NULL);
				}
			}
			oplus_adfr_set_multite_state(false);
		} else if (m->hskew == OPLUS_MFR || m->hskew == OPLUS_ADFR) {
			if (m_vrefresh == 120) {
				if (get_pwm_status()) {
					pr_info("timing switch to high pwm oa120\n");
					if (src_vrefresh == 120) {
						usleep_range(200, 300);
					}
					if (src_vrefresh == 90) {
						usleep_range(800, 900);
					}
					lcm_cmd_count = sizeof(timing_switch_cmd_high_pwm_oa120_part1) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_high_pwm_oa120_part1, lcm_cmd_count, cb, NULL);
					if((src_vrefresh == 60) || (src_vrefresh == 120)) {
						usleep_range(8400, 8500);
					} else if (src_vrefresh == 90) {
						usleep_range(5500, 5600);
					}
					lcm_cmd_count = sizeof(timing_switch_cmd_high_pwm_oa120) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_high_pwm_oa120, lcm_cmd_count, cb, NULL);
				} else {
					/* send OA120 timing-switch cmd */
					pr_info("timing switch to oa120\n");
					if (src_vrefresh == 120) {
						usleep_range(200, 300);
					}
					if (src_vrefresh == 90) {
						usleep_range(800, 900);
					}
					lcm_cmd_count = sizeof(timing_switch_cmd_oa120_part1) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_oa120_part1, lcm_cmd_count, cb, NULL);
					if((src_vrefresh == 60) || (src_vrefresh == 120)) {
						usleep_range(8400, 8500);
					} else if (src_vrefresh == 90) {
						usleep_range(5500, 5600);
					}
					lcm_cmd_count = sizeof(timing_switch_cmd_oa120) / sizeof(struct LCM_setting_table);
					panel_send_pack_hs_cmd(dsi_drv, timing_switch_cmd_oa120, lcm_cmd_count, cb, NULL);
				}
			}
		}
	} else if (stage == AFTER_DSI_POWERON) {
		ctx->m = m;
	}

	if (ext->params->data_rate != last_data_rate) {
		ret = 1;
		pr_info("need to change mipi clk, data_rate=%d,last_data_rate=%d\n",ext->params->data_rate,last_data_rate);
		last_data_rate = ext->params->data_rate;
	}

	return ret;
}

static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.panel_poweron = lcm_panel_poweron,
	.panel_poweroff = lcm_panel_poweroff,
	.panel_reset = lcm_panel_reset,
	.ata_check = panel_ata_check,
	.ext_param_get = mtk_panel_ext_param_get,
	.ext_param_set = mtk_panel_ext_param_set,
	.mode_switch_hs = mode_switch_hs,
	.esd_backlight_recovery = oplus_esd_backlight_recovery,
#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	.set_hbm = lcm_set_hbm,
	.hbm_set_cmdq = panel_hbm_set_cmdq,
	.doze_disable = panel_doze_disable,
	.doze_enable = panel_doze_enable,
	.set_aod_light_mode = panel_set_aod_light_mode,
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */
};
#endif

static int lcm_get_modes(struct drm_panel *panel,
					struct drm_connector *connector)
{
	struct drm_display_mode *mode[MODE_NUM];
	int i = 0;

	mode[0] = drm_mode_duplicate(connector->dev, &display_mode[0]);
	if (!mode[0]) {
		pr_err("failed to add mode %ux%ux@%u\n",
			display_mode[0].hdisplay, display_mode[0].vdisplay,
			 drm_mode_vrefresh(&display_mode[0]));
		return -ENOMEM;
	}

	drm_mode_set_name(mode[0]);
	mode[0]->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode[0]);
	//pr_info("en=%u, clock=%d,htotal=%d,vtotal=%d,hskew=%d,vrefresh=%d\n",mode[0], mode[0]->clock, mode[0]->htotal,
	//	mode[0]->vtotal, mode[0]->hskew, drm_mode_vrefresh(mode[0]));

	for (i = 1; i < MODE_NUM; i++) {
		mode[i] = drm_mode_duplicate(connector->dev, &display_mode[i]);
		//pr_info(" en=%u,clock=%d\n",mode[i], mode[i]->clock);
		if (!mode[i]) {
			pr_info("not enough memory\n");
			return -ENOMEM;
		}

		drm_mode_set_name(mode[i]);
		mode[i]->type = DRM_MODE_TYPE_DRIVER;
		drm_mode_probed_add(connector, mode[i]);
	}

	connector->display_info.width_mm = 70;
	connector->display_info.height_mm = 157;

	return 1;
}

static const struct drm_panel_funcs lcm_drm_funcs = {
	.disable = lcm_disable,
	.unprepare = lcm_unprepare,
	.prepare = lcm_prepare,
	.enable = lcm_enable,
	.get_modes = lcm_get_modes,
};

static int lcm_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct device_node *dsi_node, *remote_node = NULL, *endpoint = NULL;
	struct lcm *ctx;
	struct device_node *backlight;
	int ret;
	int rc = 0;
	u32 config = 0;

	pr_info("[LCM] nt37705 %s START\n", __func__);

	dsi_node = of_get_parent(dev->of_node);
	if (dsi_node) {
		endpoint = of_graph_get_next_endpoint(dsi_node, NULL);

		if (endpoint) {
			remote_node = of_graph_get_remote_port_parent(endpoint);
			if (!remote_node) {
				pr_info("No panel connected,skip probe lcm\n");
				return -ENODEV;
			}
			pr_info("device node name:%s\n", remote_node->name);
		}
	}
	if (remote_node != dev->of_node) {
		pr_info("%s+ skip probe due to not current lcm\n", __func__);
		return -ENODEV;
	}

	usleep_range(1000000, 1000100);

	ctx = devm_kzalloc(dev, sizeof(struct lcm), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);
	pr_err("[LCM]%s %d\n",__func__,__LINE__);
	ctx->dev = dev;
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_LPM | MIPI_DSI_CLOCK_NON_CONTINUOUS;

	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		ctx->backlight = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!ctx->backlight)
			return -EPROBE_DEFER;
	}

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_info(dev, "cannot get reset-gpios %ld\n",
			 PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	devm_gpiod_put(dev, ctx->reset_gpio);

	lcm_panel_vrfio18_aif_enable(ctx->dev);
	usleep_range(5000, 5100);
	ctx->vddr1p2_enable_gpio = devm_gpiod_get(dev, "pm-enable", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->vddr1p2_enable_gpio)) {
		dev_err(dev, "%s: cannot get vddr1p2_enable_gpio %ld\n",
			__func__, PTR_ERR(ctx->vddr1p2_enable_gpio));
		return PTR_ERR(ctx->vddr1p2_enable_gpio);
	}
	devm_gpiod_put(dev, ctx->vddr1p2_enable_gpio);

	usleep_range(5000, 5100);

    lcm_panel_vmc_ldo_enable(ctx->dev);

	ctx->prepared = true;
	ctx->enabled = true;
	drm_panel_init(&ctx->panel, dev, &lcm_drm_funcs, DRM_MODE_CONNECTOR_DSI);

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&ctx->panel);

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_handle_reg(&ctx->panel);
	ret = mtk_panel_ext_create(dev, &ext_params[0], &ext_funcs, &ctx->panel);
	if (ret < 0)
		return ret;

#endif
	rc = of_property_read_u32(dev->of_node, "oplus-adfr-config", &config);
	if (rc == 0) {
		oplus_adfr_config = config;
		pr_info("config=%d, adfrconfig=%d\n", config, oplus_adfr_config);
	} else {
		oplus_adfr_config = 0;
		pr_info("adfrconfig=%d\n", oplus_adfr_config);
	}
	oplus_adfr_config = 0;
	pr_info("adfrconfig=%d\n", oplus_adfr_config);
	//register_device_proc("lcd", "Tianma_NT37705", "Tianma4095");
	oplus_max_normal_brightness = SILKY_MAX_NORMAL_BRIGHTNESS;

#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
	oplus_ofp_init(dev);
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */

	pr_info("[LCM] %s- lcm,nt37705,END\n", __func__);
#ifdef OPLUS_FEATURE_DISPLAY
	pwm_power_on = true;
#endif

	return ret;
}

static void lcm_remove(struct mipi_dsi_device *dsi)
{
	struct lcm *ctx = mipi_dsi_get_drvdata(dsi);
#if defined(CONFIG_MTK_PANEL_EXT)
	struct mtk_panel_ctx *ext_ctx = find_panel_ctx(&ctx->panel);
#endif

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_detach(ext_ctx);
	mtk_panel_remove(ext_ctx);
#endif
}

static const struct of_device_id lcm_of_match[] = {
	{
	    .compatible = "oplus21135,tm,nt37705,cmd",
	},
	{}
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "oplus21135_tm_nt37705_fhd_dsi_cmd",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

module_mipi_dsi_driver(lcm_driver);

MODULE_AUTHOR("LiPing-m <liping-m@mediatek.com>");
MODULE_DESCRIPTION("lcm NT37705 CMD 120HZ AMOLED Panel Driver");
MODULE_LICENSE("GPL v2");
