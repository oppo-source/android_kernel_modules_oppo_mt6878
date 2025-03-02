/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MediaTek Inc.
 */
#ifndef __CONN_HOST_CSR_TOP_REGS_H__
#define __CONN_HOST_CSR_TOP_REGS_H__

#define CONN_HOST_CSR_TOP_BASE                                 0x18060000

#define CONN_HOST_CSR_TOP_GPS_LPCTL_ADDR                       (CONN_HOST_CSR_TOP_BASE + 0x0040)
#define CONN_HOST_CSR_TOP_CONN_INFRA_WAKEPU_GPS_ADDR           (CONN_HOST_CSR_TOP_BASE + 0x01AC)
#define CONN_HOST_CSR_TOP_BGF_MONFLG_ON_OUT_ADDR               (CONN_HOST_CSR_TOP_BASE + 0x0C00)
#define CONN_HOST_CSR_TOP_CR_HOSTCSR2BGF_ON_DBG_SEL_ADDR       (CONN_HOST_CSR_TOP_BASE + 0x0C04)

#define CONN_HOST_CSR_TOP_GPS_LPCTL_GPS_AP_HOST_CLR_FW_OWN_HS_PULSE_ADDR \
	CONN_HOST_CSR_TOP_GPS_LPCTL_ADDR
#define CONN_HOST_CSR_TOP_GPS_LPCTL_GPS_AP_HOST_CLR_FW_OWN_HS_PULSE_MASK 0x00000002
#define CONN_HOST_CSR_TOP_GPS_LPCTL_GPS_AP_HOST_CLR_FW_OWN_HS_PULSE_SHFT 1

#define CONN_HOST_CSR_TOP_CONN_INFRA_WAKEPU_GPS_CONN_INFRA_WAKEPU_GPS_ADDR CONN_HOST_CSR_TOP_CONN_INFRA_WAKEPU_GPS_ADDR
#define CONN_HOST_CSR_TOP_CONN_INFRA_WAKEPU_GPS_CONN_INFRA_WAKEPU_GPS_MASK 0x00000001
#define CONN_HOST_CSR_TOP_CONN_INFRA_WAKEPU_GPS_CONN_INFRA_WAKEPU_GPS_SHFT 0

#endif /* __CONN_HOST_CSR_TOP_REGS_H__ */

