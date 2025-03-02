#ifndef __OPLUS_MMS_GAUGE_H__
#define __OPLUS_MMS_GAUGE_H__

#include <oplus_mms.h>

#define GAUGE_INVALID_TEMP	(-400)
#define OPLUS_BATTERY_TYPE_LEN 16

enum gauge_topic_item {
	GAUGE_ITEM_SOC,
	GAUGE_ITEM_VOL,
	GAUGE_ITEM_VOL_MAX,
	GAUGE_ITEM_VOL_MIN,
	GAUGE_ITEM_GAUGE_VBAT,
	GAUGE_ITEM_CURR,
	GAUGE_ITEM_TEMP,
	GAUGE_ITEM_FCC,
	GAUGE_ITEM_CC,
	GAUGE_ITEM_SOH,
	GAUGE_ITEM_RM,
	GAUGE_ITEM_BATT_EXIST,
	GAUGE_ITEM_ERR_CODE,
	GAUGE_ITEM_RESUME,
	GAUGE_ITEM_HMAC,
	GAUGE_ITEM_AUTH,
	GAUGE_ITEM_REAL_TEMP,
	GAUGE_ITEM_SUBBOARD_TEMP_ERR,
	GAUGE_ITEM_VBAT_UV,
	GAUGE_ITEM_DEEP_SUPPORT,
	GAUGE_ITEM_REG_INFO,
	GAUGE_ITEM_CALIB_TIME,
	GAUGE_ITEM_UV_INC,
	GAUGE_ITEM_FCC_COEFF,
	GAUGE_ITEM_SOH_COEFF,
};

enum gauge_type_id {
	DEVICE_BQ27541,
	DEVICE_BQ27411,
	DEVICE_BQ28Z610,
	DEVICE_ZY0602,
	DEVICE_ZY0603,
};

#define OPLUS_BATT_SERIAL_NUM_SIZE 20
struct battery_manufacture_info {
	char batt_serial_num[OPLUS_BATT_SERIAL_NUM_SIZE];
};

int oplus_gauge_get_batt_mvolts(void);
int oplus_gauge_get_batt_fc(void);
int oplus_gauge_get_batt_qm(void);
int oplus_gauge_get_batt_pd(void);
int oplus_gauge_get_batt_rcu(void);
int oplus_gauge_get_batt_rcf(void);
int oplus_gauge_get_batt_fcu(void);
int oplus_gauge_get_batt_fcf(void);
int oplus_gauge_get_batt_sou(void);
int oplus_gauge_get_batt_do0(void);
int oplus_gauge_get_batt_doe(void);
int oplus_gauge_get_batt_trm(void);
int oplus_gauge_get_batt_pc(void);
int oplus_gauge_get_batt_qs(void);
int oplus_gauge_get_batt_mvolts_2cell_max(void);
int oplus_gauge_get_batt_mvolts_2cell_min(void);

int oplus_gauge_get_batt_soc(void);
int oplus_gauge_get_batt_current(void);
int oplus_gauge_get_remaining_capacity(void);
int oplus_gauge_get_device_type(void);
int oplus_gauge_get_device_type_for_vooc(void);

int oplus_gauge_get_batt_fcc(void);

int oplus_gauge_get_batt_cc(void);
int oplus_gauge_get_batt_soh(void);
bool oplus_gauge_get_batt_hmac(void);
bool oplus_gauge_get_batt_authenticate(void);
void oplus_gauge_set_batt_full(bool);
bool oplus_gauge_check_chip_is_null(void);
bool oplus_gauge_is_exist(struct oplus_mms *topic);

int oplus_gauge_update_battery_dod0(void);
int oplus_gauge_update_soc_smooth_parameter(void);
int oplus_gauge_get_battery_cb_status(void);
int oplus_gauge_get_i2c_err(void);
void oplus_gauge_clear_i2c_err(void);
int oplus_gauge_get_passedchg(int *val);
int oplus_gauge_dump_register(void);
int oplus_gauge_lock(void);
int oplus_gauge_unlock(void);
bool oplus_gauge_is_locked(void);
int oplus_gauge_get_batt_num(void);
int oplus_gauge_get_batt_capacity_mah(struct oplus_mms *topic);

int oplus_gauge_get_dod0(struct oplus_mms *mms, int index, int *val);
int oplus_gauge_get_dod0_passed_q(struct oplus_mms *mms, int index, int *val);
int oplus_gauge_get_qmax(struct oplus_mms *mms, int index, int *val);
int oplus_gauge_get_qmax_passed_q(struct oplus_mms *mms, int index, int *val);
int oplus_gauge_get_volt(struct oplus_mms *mms, int index, int *val);
int oplus_gauge_get_gauge_type(struct oplus_mms *mms, int index, int *val);
int oplus_gauge_get_bcc_parameters(char *buf);
int oplus_gauge_fastchg_update_bcc_parameters(char *buf);
int oplus_gauge_get_prev_bcc_parameters(char *buf);
int oplus_gauge_set_bcc_parameters(const char *buf);

int oplus_gauge_protect_check(void);
bool oplus_gauge_afi_update_done(void);

bool oplus_gauge_check_reset_condition(void);
bool oplus_gauge_reset(void);
bool is_support_parallel_battery(struct oplus_mms *topic);
void oplus_gauge_set_deep_dischg_count(struct oplus_mms *topic, int count);
int oplus_gauge_show_deep_dischg_count(struct oplus_mms *topic);
void oplus_gauge_set_deep_count_cali(struct oplus_mms *topic, int val);
int oplus_gauge_get_deep_count_cali(struct oplus_mms *topic);
void oplus_gauge_set_deep_dischg_ratio_thr(struct oplus_mms *topic, int ratio);
int oplus_gauge_get_deep_dischg_ratio_thr(struct oplus_mms *topic);
int oplus_gauge_get_battery_type_str(char *type);
struct device_node *oplus_get_node_by_type(struct device_node *father_node);
int oplus_gauge_get_battinfo_sn(struct oplus_mms *topic, char *sn_buff, int size_buffer);
#endif /* __OPLUS_MMS_GAUGE_H__ */
