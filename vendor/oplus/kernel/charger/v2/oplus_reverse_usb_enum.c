/***********************************************************
** Copyright (C), 2026 Oplus. All rights reserved.
** File: oplus_reverse_usb_enum.c
** Description: reverse charge usb enum classify
****************************************************************/

#define pr_fmt(fmt) "[REVERSE_DETECT_GAMEPAD]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/notifier.h>
#include <linux/mutex.h>
#include <linux/usb.h>
#include <linux/hid.h>
#include <linux/list.h>
#include <oplus_chg.h>
#include <oplus_reverse_usb_enum.h>

struct reverse_usb_gamepad_dev {
	struct list_head list;
	u8 busnum;
	u8 devnum;
	u16 vid;
	u16 pid;
};

struct oplus_reverse_usb_enum_mgr {
	struct notifier_block usb_nb;
	bool usb_nb_registered;
	struct mutex lock;
	struct list_head gamepad_devs;
	int gamepad_num;
	struct oplus_reverse_usb_enum_result result;
};

struct reverse_usb_enum_profile {
	int intf_count;
	bool has_hid_gamepad;
};

static const char *oplus_reverse_usb_enum_type_str(int type)
{
	switch (type) {
	case REVERSE_USB_ENUM_GAMEPAD:
		return "gamepad";
	default:
		return "unknown";
	}
}

static bool oplus_reverse_usb_enum_should_skip(struct usb_device *udev)
{
	if (!udev || udev->state != USB_STATE_CONFIGURED)
		return true;

	return false;
}

static bool oplus_reverse_usb_enum_is_hid_gamepad_intf(
	const struct usb_interface_descriptor *intf_desc)
{
	if (!intf_desc)
		return false;

	if (intf_desc->bInterfaceClass != USB_CLASS_HID)
		return false;

	/*
	 * Keep reverse decision strict:
	 * only treat non-keyboard/non-mouse HID as gamepad-like.
	 */
	return intf_desc->bInterfaceProtocol != USB_INTERFACE_PROTOCOL_KEYBOARD &&
	       intf_desc->bInterfaceProtocol != USB_INTERFACE_PROTOCOL_MOUSE;
}

static void oplus_reverse_usb_enum_collect_profile(struct usb_device *udev,
						   struct reverse_usb_enum_profile *profile)
{
	struct usb_host_config *config;
	struct usb_interface_cache *intf_cache;
	struct usb_interface_descriptor *intf_desc;
	int i, j;

	config = udev->config;
	if (!config)
		return;

	for (i = 0; i < udev->descriptor.bNumConfigurations; i++, config++) {
		if (!config)
			continue;
		for (j = 0; j < config->desc.bNumInterfaces; j++) {
			intf_cache = config->intf_cache[j];
			if (!intf_cache)
				continue;
			if (intf_cache->num_altsetting <= 0) {
				chg_debug("skip intf bus=%u dev=%u cfg=%d intf=%d reason=no_altsetting\n",
					  udev->bus ? udev->bus->busnum : 0, udev->devnum, i, j);
				continue;
			}
			intf_desc = &intf_cache->altsetting[0].desc;
			profile->intf_count++;
			if (oplus_reverse_usb_enum_is_hid_gamepad_intf(intf_desc))
				profile->has_hid_gamepad = true;
		}
	}
}

static int oplus_reverse_usb_enum_classify(const struct reverse_usb_enum_profile *profile,
					   const char **reason)
{
	/*
	 * Only classify USB gamepad-like devices.
	 * HID keyboard/mouse are excluded in collect_profile().
	 */
	if (profile->has_hid_gamepad) {
		*reason = "usb_gamepad_profile";
		return REVERSE_USB_ENUM_GAMEPAD;
	}

	*reason = "unknown";
	return REVERSE_USB_ENUM_UNKNOWN;
}

static int oplus_reverse_usb_enum_detect_gamepad(struct usb_device *udev,
						 struct oplus_reverse_usb_enum_result *result)
{
	struct reverse_usb_enum_profile profile = { 0 };
	const char *classify_reason = "unknown";
	u16 vid, pid;
	int type;

	if (!udev || !result) {
		chg_err("invalid arg udev=%p result=%p\n", udev, result);
		return -EINVAL;
	}

	vid = le16_to_cpu(udev->descriptor.idVendor);
	pid = le16_to_cpu(udev->descriptor.idProduct);
	result->type = REVERSE_USB_ENUM_UNKNOWN;
	result->vid = 0;
	result->pid = 0;

	if (oplus_reverse_usb_enum_should_skip(udev)) {
		chg_debug("detect skip state=%u vid=0x%04x pid=0x%04x\n", udev->state, vid, pid);
		return 0;
	}

	oplus_reverse_usb_enum_collect_profile(udev, &profile);
	type = oplus_reverse_usb_enum_classify(&profile, &classify_reason);

	if (type == REVERSE_USB_ENUM_GAMEPAD) {
		result->type = type;
		result->vid = vid;
		result->pid = pid;
		chg_info("usb gamepad classify: vid=0x%04x pid=0x%04x intf_count=%d reason=%s\n",
			 vid, pid, profile.intf_count, classify_reason);
	}

	chg_debug("detect summary: state=%u type=%s vid=0x%04x pid=0x%04x intf_count=%d\n",
		  udev->state, oplus_reverse_usb_enum_type_str(result->type),
		  result->vid, result->pid, profile.intf_count);
	return 0;
}

static u8 oplus_reverse_usb_dev_get_busnum(struct usb_device *udev)
{
	return (udev && udev->bus) ? udev->bus->busnum : 0;
}

static void oplus_reverse_usb_enum_update_cached_result_locked(struct oplus_reverse_usb_enum_mgr *mgr)
{
	struct reverse_usb_gamepad_dev *dev;

	if (!mgr)
		return;

	if (mgr->gamepad_num > 0 && !list_empty(&mgr->gamepad_devs)) {
		dev = list_first_entry(&mgr->gamepad_devs, struct reverse_usb_gamepad_dev, list);
		mgr->result.type = REVERSE_USB_ENUM_GAMEPAD;
		mgr->result.vid = dev->vid;
		mgr->result.pid = dev->pid;
	} else {
		mgr->result.type = REVERSE_USB_ENUM_UNKNOWN;
		mgr->result.vid = 0;
		mgr->result.pid = 0;
	}
}

static bool oplus_reverse_usb_enum_add_gamepad_device_list(struct oplus_reverse_usb_enum_mgr *mgr,
							    struct usb_device *udev,
							    u16 vid, u16 pid)
{
	struct reverse_usb_gamepad_dev *dev;
	u8 busnum = oplus_reverse_usb_dev_get_busnum(udev);
	u8 devnum;

	if (!udev) {
		chg_err("add gamepad device invalid udev=NULL\n");
		return false;
	}
	devnum = udev->devnum;
	list_for_each_entry(dev, &mgr->gamepad_devs, list) {
		if (dev->busnum == busnum && dev->devnum == devnum) {
			dev->vid = vid;
			dev->pid = pid;
			return true;
		}
	}

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		chg_err("kzalloc gamepad_devs failed");
		return false;
	}
	dev->busnum = busnum;
	dev->devnum = devnum;
	dev->vid = vid;
	dev->pid = pid;
	list_add_tail(&dev->list, &mgr->gamepad_devs);
	mgr->gamepad_num++;
	return true;
}

static bool oplus_reverse_usb_enum_track_gamepad_remove_list(struct oplus_reverse_usb_enum_mgr *mgr,
							      struct usb_device *udev)
{
	struct reverse_usb_gamepad_dev *dev, *tmp;
	u8 busnum;
	u8 devnum;

	if (!udev) {
		chg_err("remove gamepad device invalid udev=NULL\n");
		return false;
	}

	busnum = oplus_reverse_usb_dev_get_busnum(udev);
	devnum = udev->devnum;

	list_for_each_entry_safe(dev, tmp, &mgr->gamepad_devs, list) {
		if (dev->busnum == busnum && dev->devnum == devnum) {
			list_del(&dev->list);
			kfree(dev);
			mgr->gamepad_num--;
			return true;
		}
	}
	return false;
}

static int oplus_reverse_usb_enum_mgr_notifier_call(struct notifier_block *nb,
						    unsigned long action, void *data)
{
	struct oplus_reverse_usb_enum_mgr *mgr =
		container_of(nb, struct oplus_reverse_usb_enum_mgr, usb_nb);
	struct usb_device *udev = data;
	struct oplus_reverse_usb_enum_result result = { 0 };
	int old_type, rc;
	u16 vid = 0, pid = 0;
	bool removed = false;

	if (!mgr || !udev) {
		chg_err("notifier invalid arg mgr=%p udev=%p action=%lu\n", mgr, udev, action);
		return NOTIFY_DONE;
	}

	vid = le16_to_cpu(udev->descriptor.idVendor);
	pid = le16_to_cpu(udev->descriptor.idProduct);
	if (action == USB_DEVICE_ADD) {
		rc = oplus_reverse_usb_enum_detect_gamepad(udev, &result);
		if (rc < 0)
			return NOTIFY_DONE;
		if (result.type != REVERSE_USB_ENUM_GAMEPAD)
			return NOTIFY_OK;

		mutex_lock(&mgr->lock);
		old_type = mgr->result.type;
		if (!oplus_reverse_usb_enum_add_gamepad_device_list(mgr, udev, result.vid, result.pid)) {
			mutex_unlock(&mgr->lock);
			chg_err("track gamepad add failed bus=%u dev=%u\n",
				oplus_reverse_usb_dev_get_busnum(udev), udev->devnum);
			return NOTIFY_DONE;
		}
		oplus_reverse_usb_enum_update_cached_result_locked(mgr);
		if (old_type != mgr->result.type)
			chg_info("usb enum type %s ----> %s (vid=0x%04x pid=0x%04x)\n",
				 oplus_reverse_usb_enum_type_str(old_type),
				 oplus_reverse_usb_enum_type_str(mgr->result.type),
				 mgr->result.vid, mgr->result.pid);
		mutex_unlock(&mgr->lock);
		return NOTIFY_OK;
	}

	if (action != USB_DEVICE_REMOVE)
		return NOTIFY_DONE;

	mutex_lock(&mgr->lock);
	old_type = mgr->result.type;
	removed = oplus_reverse_usb_enum_track_gamepad_remove_list(mgr, udev);
	if (removed) {
		oplus_reverse_usb_enum_update_cached_result_locked(mgr);
		if (old_type != mgr->result.type)
			chg_info("usb enum type %s ----> %s (vid=0x%04x pid=0x%04x)\n",
				 oplus_reverse_usb_enum_type_str(old_type),
				 oplus_reverse_usb_enum_type_str(mgr->result.type),
				 mgr->result.vid, mgr->result.pid);
	}
	mutex_unlock(&mgr->lock);
	if (removed)
		chg_info("gamepad unplugged, clear tracked type: vid=0x%04x pid=0x%04x\n", vid, pid);
	return NOTIFY_OK;
}

struct oplus_reverse_usb_enum_mgr * oplus_reverse_usb_enum_mgr_create(void)
{
	struct oplus_reverse_usb_enum_mgr *mgr;

	mgr = kzalloc(sizeof(*mgr), GFP_KERNEL);
	if (!mgr) {
		chg_err("alloc usb enum mgr failed\n");
		return NULL;
	}

	mutex_init(&mgr->lock);
	INIT_LIST_HEAD(&mgr->gamepad_devs);
	mgr->gamepad_num = 0;
	mgr->result.type = REVERSE_USB_ENUM_UNKNOWN;
	mgr->usb_nb.notifier_call = oplus_reverse_usb_enum_mgr_notifier_call;
	usb_register_notify(&mgr->usb_nb);
	mgr->usb_nb_registered = true;
	chg_info("usb enum mgr create done, notifier_registered=%d mgr=%p\n",
		 mgr->usb_nb_registered, mgr);

	return mgr;
}

void oplus_reverse_usb_enum_mgr_destroy(struct oplus_reverse_usb_enum_mgr *mgr)
{
	struct reverse_usb_gamepad_dev *dev, *tmp;

	if (!mgr)
		return;

	chg_info("usb enum mgr destroy start, notifier_registered=%d mgr=%p\n",
		 mgr->usb_nb_registered, mgr);
	if (mgr->usb_nb_registered) {
		usb_unregister_notify(&mgr->usb_nb);
		mgr->usb_nb_registered = false;
	}
	mutex_lock(&mgr->lock);
	list_for_each_entry_safe(dev, tmp, &mgr->gamepad_devs, list) {
		list_del(&dev->list);
		kfree(dev);
	}
	mgr->gamepad_num = 0;
	mgr->result.type = REVERSE_USB_ENUM_UNKNOWN;
	mgr->result.vid = 0;
	mgr->result.pid = 0;
	mutex_unlock(&mgr->lock);
	chg_info("usb enum mgr destroy done, mgr=%p\n", mgr);
	kfree(mgr);
}

int oplus_reverse_usb_enum_mgr_refresh(struct oplus_reverse_usb_enum_mgr *mgr)
{
	if (!mgr) {
		chg_err("refresh invalid mgr=NULL\n");
		return -EINVAL;
	}

	/*
	 * notifier-only mode has no full device walk.
	 * Keep current cached type and wait for add/remove notifier events.
	 */
	chg_info("usb enum proactive refresh (notifier-only), keep current type\n");
	return 0;
}

bool oplus_reverse_usb_enum_mgr_is_gamepad(struct oplus_reverse_usb_enum_mgr *mgr)
{
	bool is_gamepad = false;

	if (!mgr)
		return false;

	mutex_lock(&mgr->lock);
	is_gamepad = (mgr->result.type == REVERSE_USB_ENUM_GAMEPAD);
	mutex_unlock(&mgr->lock);

	return is_gamepad;
}
