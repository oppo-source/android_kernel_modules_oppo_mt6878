#ifndef __S6E8FC3_FHDP_DSI_VDO_SAMSUNG_AMS667FK03__
#define __S6E8FC3_FHDP_DSI_VDO_SAMSUNG_AMS667FK03__

#define REGFLAG_CMD				0xFFFA
#define REGFLAG_DELAY			0xFFFC
#define REGFLAG_UDELAY			0xFFFB
#define REGFLAG_END_OF_TABLE	0xFFFD

#define BRIGHTNESS_HALF         2047
#define BRIGHTNESS_MAX          4095

enum MODE_ID {
	FHD_SDC60 = 0,
	FHD_SDC90 = 1,
	FHD_SDC120 = 2,
};

struct LCM_setting_table {
	unsigned int cmd;
	unsigned int count;
	unsigned char para_list[256];
};

static struct LCM_setting_table lcm_setbrightness_normal[] = {
	{REGFLAG_CMD,3, {0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD,4, {0xB0, 0x00, 0x0C, 0xB2}},
	{REGFLAG_CMD,2, {0xB2, 0x30}},
	{REGFLAG_CMD,3, {0xF0, 0xA5, 0xA5}},

	{REGFLAG_CMD,2, {0x53, 0x20}},
	{REGFLAG_CMD,3, {0x51, 0x00, 0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_setbrightness_hbm[] = {
	{REGFLAG_CMD,2, {0x53, 0xE0}},
	{REGFLAG_CMD,3, {0x51, 0x00, 0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_finger_HBM_on_setting[] = {
	{REGFLAG_CMD,3, {0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD,4, {0xB0, 0x00, 0x0C, 0xB2}},
	{REGFLAG_CMD,2, {0xB2, 0x00}},
	{REGFLAG_CMD,3, {0xF0, 0x0D, 0x55}},
	/*HBM ON*/
	// {REGFLAG_CMD,3, {0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD,2, {0x53, 0xE0}},
	{REGFLAG_CMD,3, {0x51, 0x0A, 0x00}},
	// {REGFLAG_CMD,3, {0xF0, 0xA5, 0xA5}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_normal_HBM_on_setting[] = {
	{REGFLAG_CMD,2, {0x53, 0xE0}},
	{REGFLAG_CMD,3, {0x51, 0x0F, 0xFF}},
};

/* -------------------------doze mode setting start------------------------- */
static struct LCM_setting_table AOD_off_setting[] = {
	// {REGFLAG_CMD, 4, {0xFF, 0x78, 0x38, 0x00}},
	// {REGFLAG_CMD, 1, {0x38}},
	// {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table AOD_on_setting[] = {
	{REGFLAG_CMD, 1, {0x11}},
	{REGFLAG_DELAY, 20, {}},

	{REGFLAG_CMD, 2, {0x35, 0x00}},

	{REGFLAG_CMD, 3, {0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD, 2, {0x60, 0x21}},
	{REGFLAG_CMD, 3, {0xF7, 0x0B}},
	{REGFLAG_CMD, 3, {0xF0, 0xA5, 0xA5}},

	{REGFLAG_CMD, 3, {0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD, 2, {0x53, 0x28}},
	{REGFLAG_CMD, 3, {0x51, 0x03, 0xFF}},
	{REGFLAG_CMD, 3, {0xF0, 0xA5, 0xA5}},
	{REGFLAG_DELAY, 120,{}},
	{REGFLAG_CMD, 1, {0x29}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table aod_high_bl_level[] = {
	{REGFLAG_CMD, 3, {0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD, 2, {0x53, 0x28}},
	{REGFLAG_CMD, 3, {0x51, 0x03, 0xFF}},
	{REGFLAG_CMD, 3, {0xF0, 0xA5, 0xA5}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table aod_low_bl_level[] = {
	{REGFLAG_CMD, 3, {0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD, 2, {0x53, 0x28}},
	{REGFLAG_CMD, 3, {0x51, 0x01, 0xFF}},
	{REGFLAG_CMD, 3, {0xF0, 0xA5, 0xA5}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
/* -------------------------doze mode setting end------------------------- */

/* -------------------------frame mode switch start------------------------- */
static struct LCM_setting_table mode_switch_to_60[] = {
    {REGFLAG_CMD, 3, {0xF0,0x5A,0x5A}},
    {REGFLAG_CMD, 2, {0x60,0x21}},
    {REGFLAG_CMD, 2, {0xF7,0x0B}},
    {REGFLAG_CMD, 3, {0xF0,0xA5,0xA5}},
};

static struct LCM_setting_table mode_switch_to_120[] = {
    {REGFLAG_CMD, 3, {0xF0,0x5A,0x5A}},
    {REGFLAG_CMD, 2, {0x60,0x01}},
    {REGFLAG_CMD, 2, {0xF7,0x0B}},
    {REGFLAG_CMD, 3, {0xF0,0xA5,0xA5}},
};
/* -------------------------frame mode switch end------------------------- */
#endif
