/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2026 Oplus.
 */

#ifndef __OPLUS_REVERSE_USB_ENUM_H__
#define __OPLUS_REVERSE_USB_ENUM_H__

#include <linux/types.h>
#include <linux/usb.h>

enum reverse_usb_enum_type {
	REVERSE_USB_ENUM_UNKNOWN = 0,
	REVERSE_USB_ENUM_GAMEPAD,
	REVERSE_USB_ENUM_MAX,
};

struct oplus_reverse_usb_enum_result {
	int type;
	u16 vid;
	u16 pid;
};

struct oplus_reverse_usb_enum_mgr;

struct oplus_reverse_usb_enum_mgr * oplus_reverse_usb_enum_mgr_create(void);
void oplus_reverse_usb_enum_mgr_destroy(struct oplus_reverse_usb_enum_mgr *mgr);
int oplus_reverse_usb_enum_mgr_refresh(struct oplus_reverse_usb_enum_mgr *mgr);
bool oplus_reverse_usb_enum_mgr_is_gamepad(struct oplus_reverse_usb_enum_mgr *mgr);

#endif /* __OPLUS_REVERSE_USB_ENUM_H__ */
