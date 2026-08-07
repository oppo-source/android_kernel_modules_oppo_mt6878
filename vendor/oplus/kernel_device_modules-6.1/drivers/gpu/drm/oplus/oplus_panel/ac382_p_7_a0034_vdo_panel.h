#ifndef PANEL_AC382_P_7_A0034_H
#define PANEL_AC382_P_7_A0034_H

#define REGFLAG_CMD				0xFFFA
#define REGFLAG_DELAY			0xFFFC
#define REGFLAG_UDELAY			0xFFFB
#define REGFLAG_END_OF_TABLE	0xFFFD

#define BRIGHTNESS_HALF         3515
#define BRIGHTNESS_MAX          4094

enum MODE_ID {
	FHD_SDC60 = 0,
	FHD_SDC90 = 1,
	FHD_SDC120 = 2,
	FHD_SDC144 = 3,
	FHD_SDC30 = 4,
};

/* Mode Config */
#define MODE_NUM                    5
#define RES_NUM                     (2)
#define MODE_MAPPING_RULE(x)        ((x) % (MODE_NUM))
static enum RES_SWITCH_TYPE res_switch_type = RES_SWITCH_NO_USE;

struct ba {
	u32 brightness;
	u32 alpha;
};
enum SEED_MODE_ID {
	EXPERT = 101,
	NATURAL = 102,
	UIR_ON_EXPERT = 111,
	UIR_ON_NATURAL = 112,
	UIR_OFF_EXPERT = 121,
	UIR_OFF_NATURAL = 122,
	UIR_DIM_ENABLE = 134,
	UIR_DIM_DISABLE = 135,
};

struct LCM_setting_table {
	unsigned int cmd;
	unsigned int count;
	unsigned char para_list[256];
};

/* -------------------------doze mode setting start------------------------- */
struct LCM_setting_table lcm_lhbm_on_setting[] = {
	{REGFLAG_CMD, 4, {0xFF, 0x5A, 0xA5, 0x00}},
	{REGFLAG_CMD, 2, {0xBA, 0x03}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

struct LCM_setting_table lcm_lhbm_off_setting[] = {
	{REGFLAG_CMD, 4, {0xFF, 0x5A, 0xA5, 0x00}},
	{REGFLAG_CMD, 2, {0xBA, 0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

struct LCM_setting_table AOD_off_setting[] = {
	{REGFLAG_CMD, 4, {0xFF, 0x5A, 0xA5, 0x00}},
	{REGFLAG_CMD, 1, {0x38}},
	{REGFLAG_CMD, 4, {0xFF, 0x5A, 0xA5, 0x22}},
	{REGFLAG_CMD, 2, {0xDD, 0x12}},
	{REGFLAG_CMD, 4, {0xFF, 0x5A, 0xA5, 0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

struct LCM_setting_table AOD_on_setting[] = {
	{REGFLAG_CMD, 4, {0xFF, 0x5A, 0xA5, 0x00}},
	{REGFLAG_CMD, 1, {0x39}},
	{REGFLAG_CMD, 4, {0xFF, 0x5A, 0xA5, 0x22}},
	{REGFLAG_CMD, 2, {0xDD, 0x02}},
	{REGFLAG_CMD, 4, {0xFF, 0x5A, 0xA5, 0x00}},
	{REGFLAG_CMD, 3, {0xFD, 0x00, 0x80}},
	{REGFLAG_CMD, 3, {0x51, 0x03, 0xD4}},
	{REGFLAG_CMD, 3, {0xFD, 0x00, 0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

struct LCM_setting_table aod_high_bl_level[] = {
	{REGFLAG_CMD, 4, {0xFF, 0x5A, 0xA5, 0x00}},
	{REGFLAG_CMD, 3, {0xFD, 0x00, 0x80}},
	{REGFLAG_CMD, 3, {0x51, 0x03, 0xD4}},
	{REGFLAG_CMD, 3, {0xFD, 0x00, 0x00}},
};

struct LCM_setting_table aod_low_bl_level[] = {
	{REGFLAG_CMD, 4, {0xFF, 0x5A, 0xA5, 0x00}},
	{REGFLAG_CMD, 3, {0xFD, 0x00, 0x80}},
	{REGFLAG_CMD, 3, {0x51, 0x01, 0xB3}},
	{REGFLAG_CMD, 3, {0xFD, 0x00, 0x00}},
};
/* -------------------------doze mode setting end------------------------- */

/* -------------------------demura setting start---------------------- */
//static struct LCM_setting_table dsi_demura0_bl[] = {
//    /* demura0 level <= 0x0481(70nit_PWM) */
//	// brs reload DC(BRS0,1,2)
//    {REGFLAG_CMD, 6, {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x04}},
//    {REGFLAG_CMD, 2, {0x6F, 0x01}},
//    {REGFLAG_CMD, 4, {0xB5, 0x00, 0x00, 0x07}},
//    {REGFLAG_CMD, 2, {0x9D, 0xAA}},
//};

//static struct LCM_setting_table dsi_demura1_bl[] = {
//    /* demura1 0x0482(70.1nit_DC) <= level <= 0x0FFE(1400nit) */
//	// brs reload DC(BRS0,1,3)
//    {REGFLAG_CMD, 6, {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x04}},
//    {REGFLAG_CMD, 2, {0x6F, 0x01}},
//    {REGFLAG_CMD, 4, {0xB5, 0x00, 0x00, 0x0B}},
//    {REGFLAG_CMD, 2, {0x9D, 0xAA}},
//};
///* -------------------------demura setting end------------------------- */
///* ---------------panel seed setting --------------- */
/* ---------------Loading on 110% --------------- */
struct LCM_setting_table dsi_set_seed_natural[] = {
    {REGFLAG_CMD, 4, {0xFF, 0x5A, 0xA5, 0x76}},
    {REGFLAG_CMD, 2, {0x80, 0xA9}},
    {REGFLAG_CMD, 2, {0x81, 0x00}},
    {REGFLAG_CMD, 3, {0x85, 0x08, 0x10}},
    {REGFLAG_CMD, 2, {0x9D, 0x01}},
    {REGFLAG_CMD, 4, {0xFF, 0x5A, 0xA5, 0x00}},
    {REGFLAG_CMD, 2, {0xB4, 0x00}},
    {REGFLAG_CMD, 4, {0xFF, 0x5A, 0xA5, 0x00}},
};
/* ---------------Loading off 100% --------------- */
struct LCM_setting_table dsi_set_seed_expert[] = {
    {REGFLAG_CMD, 4, {0xFF, 0x5A, 0xA5, 0x76}},
    {REGFLAG_CMD, 2, {0x80, 0xA9}},
    {REGFLAG_CMD, 2, {0x81, 0x00}},
    {REGFLAG_CMD, 3, {0x85, 0x08, 0x10}},
    {REGFLAG_CMD, 2, {0x9D, 0x01}},
    {REGFLAG_CMD, 4, {0xFF, 0x5A, 0xA5, 0x00}},
    {REGFLAG_CMD, 2, {0xB4, 0x00}},
    {REGFLAG_CMD, 4, {0xFF, 0x5A, 0xA5, 0x00}},
};
struct LCM_setting_table dsi_set_uir_on_natural[] = {
    {REGFLAG_CMD, 4, {0xFF, 0x5A, 0xA5, 0x76}},
    {REGFLAG_CMD, 2, {0x80, 0xA9}},
    {REGFLAG_CMD, 2, {0x81, 0x11}},
    {REGFLAG_CMD, 2, {0x88, 0x0B}},
    {REGFLAG_CMD, 4, {0xA8, 0x00, 0x12, 0x0E}},
    {REGFLAG_CMD, 3, {0x85, 0x08, 0x00}},
    {REGFLAG_CMD, 2, {0x9D, 0x1A}},
    {REGFLAG_CMD, 4, {0xFF, 0x5A, 0xA5, 0x00}},
    {REGFLAG_CMD, 2, {0xB4, 0x20}},
};
struct LCM_setting_table dsi_set_uir_on_expert[] = {
    {REGFLAG_CMD, 4, {0xFF, 0x5A, 0xA5, 0x76}},
    {REGFLAG_CMD, 2, {0x80, 0xA9}},
    {REGFLAG_CMD, 2, {0x81, 0x11}},
    {REGFLAG_CMD, 2, {0x88, 0x0B}},
    {REGFLAG_CMD, 4, {0xA8, 0x00, 0x12, 0x0E}},
    {REGFLAG_CMD, 3, {0x85, 0x08, 0x00}},
    {REGFLAG_CMD, 2, {0x9D, 0x1A}},
    {REGFLAG_CMD, 4, {0xFF, 0x5A, 0xA5, 0x00}},
    {REGFLAG_CMD, 2, {0xB4, 0x20}},
};
struct LCM_setting_table dsi_set_uir_off_natural[] = {
    {REGFLAG_CMD, 4, {0xFF, 0x5A, 0xA5, 0x76}},
    {REGFLAG_CMD, 2, {0x80, 0xA9}},
    {REGFLAG_CMD, 2, {0x81, 0x11}},
    {REGFLAG_CMD, 2, {0x88, 0x0B}},
    {REGFLAG_CMD, 4, {0xA8, 0x00, 0x12, 0x0E}},
    {REGFLAG_CMD, 3, {0x85, 0x08, 0x10}},
    {REGFLAG_CMD, 2, {0x9D, 0x01}},
    {REGFLAG_CMD, 4, {0xFF, 0x5A, 0xA5, 0x00}},
    {REGFLAG_CMD, 2, {0xB4, 0x00}},
    {REGFLAG_CMD, 4, {0xFF, 0x5A, 0xA5, 0x00}},
};
struct LCM_setting_table dsi_set_uir_off_expert[] = {
    {REGFLAG_CMD, 4, {0xFF, 0x5A, 0xA5, 0x76}},
    {REGFLAG_CMD, 2, {0x80, 0xA9}},
    {REGFLAG_CMD, 2, {0x81, 0x11}},
    {REGFLAG_CMD, 2, {0x88, 0x0B}},
    {REGFLAG_CMD, 4, {0xA8, 0x00, 0x12, 0x0E}},
    {REGFLAG_CMD, 3, {0x85, 0x08, 0x10}},
    {REGFLAG_CMD, 2, {0x9D, 0x01}},
    {REGFLAG_CMD, 4, {0xFF, 0x5A, 0xA5, 0x00}},
    {REGFLAG_CMD, 2, {0xB4, 0x00}},
    {REGFLAG_CMD, 4, {0xFF, 0x5A, 0xA5, 0x00}},
};
struct LCM_setting_table dsi_set_uir_dim_enable[] = {
    {REGFLAG_CMD, 4, {0xFF, 0x5A, 0xA5, 0x76}},
    {REGFLAG_CMD, 2, {0x81, 0x11}},
    {REGFLAG_CMD, 2, {0x88, 0x0B}},
    {REGFLAG_CMD, 4, {0xA8, 0x00, 0x12, 0x0E}},
};
struct LCM_setting_table dsi_set_uir_dim_disable[] = {
    {REGFLAG_CMD, 4, {0xFF, 0x5A, 0xA5, 0x76}},
    {REGFLAG_CMD, 2, {0x81, 0x00}},
};
#endif
