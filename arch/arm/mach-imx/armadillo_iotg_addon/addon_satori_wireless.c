/*
 * Copyright (C) 2017 Atmark Techno, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_gpio.h>

#include "armadillo_iotg_std_addon.h"

extern_dtb(addon_satori_wireless_iotg_g3_intf1);
extern_dtb(addon_satori_wireless_iotg_g3_intf2);
extern_dtb(addon_satori_wireless_x1_intf1);

#define PIN_RESET	(42)
#define PIN_WAKEUP	(43)
#define PIN_RING	(32)

int addon_setup_satori_wireless(struct addon_device *adev)
{
	struct armadillo_iotg_addon *addon = to_addon(adev);
	void *begin;
	size_t size;
	int gpio_reset;
	int gpio_wakeup;
	int gpio_ring;
	char label[64];

	switch (adev->intf) {
	case ADDON_INTERFACE1:
		switch (addon->type) {
		case ADDON_BOARD_TYPE_IOTG:
			begin = dtb_begin(addon_satori_wireless_iotg_g3_intf1);
			size = dtb_size(addon_satori_wireless_iotg_g3_intf1);
			break;
		case ADDON_BOARD_TYPE_X1:
			begin = dtb_begin(addon_satori_wireless_x1_intf1);
			size = dtb_size(addon_satori_wireless_x1_intf1);
			break;
		default:
			return -ENODEV;
		};
		break;
	case ADDON_INTERFACE2:
		switch (addon->type) {
		case ADDON_BOARD_TYPE_IOTG:
			begin = dtb_begin(addon_satori_wireless_iotg_g3_intf2);
			size = dtb_size(addon_satori_wireless_iotg_g3_intf2);
			break;
		default:
			return -ENODEV;
		};
		break;
	default:
		BUG();
	};
	armadillo_iotg_addon_dt_overlay(addon->dev, begin, size);

	gpio_reset = adev->gpios[PIN_RESET - 1];
	gpio_wakeup = adev->gpios[PIN_WAKEUP - 1];
	gpio_ring = adev->gpios[PIN_RING - 1];

	if (!gpio_is_valid(gpio_reset)) {
		dev_warn(addon->dev, "gpio_reset is invalid\n");
		return -EINVAL;
	}
	if (devm_gpio_request(addon->dev, gpio_reset, NULL)) {
		dev_warn(addon->dev, "gpio_reset request failed\n");
		return -EINVAL;
	}

	if (!gpio_is_valid(gpio_wakeup)) {
		dev_warn(addon->dev, "gpio_wakeup is invalid\n");
		return -EINVAL;
	}
	if (devm_gpio_request(addon->dev, gpio_wakeup, NULL)) {
		dev_warn(addon->dev, "gpio_wakeup request failed\n");
		return -EINVAL;
	}

	if (!gpio_is_valid(gpio_ring)) {
		dev_warn(addon->dev, "gpio_ring is invalid\n");
		return -EINVAL;
	}
	if (devm_gpio_request(addon->dev, gpio_ring, NULL)) {
		dev_warn(addon->dev, "gpio_ring request failed\n");
		return -EINVAL;
	}

	gpio_direction_output(gpio_reset, 1);	 /* always high */
	gpio_direction_output(gpio_wakeup, 1);	 /* always high */
	gpio_direction_output(gpio_ring, 1);	 /* always high */

	gpio_export(gpio_reset, false);
	gpio_export(gpio_wakeup, false);
	gpio_export(gpio_ring, false);

	snprintf(label, sizeof(label), "RESET_INTF%d", adev->intf);
	gpio_export_link(addon->dev, label, gpio_reset);
	snprintf(label, sizeof(label), "WAKEUP_INTF%d", adev->intf);
	gpio_export_link(addon->dev, label, gpio_wakeup);
	snprintf(label, sizeof(label), "RING_INTF%d", adev->intf);
	gpio_export_link(addon->dev, label, gpio_ring);

	return 0;
}
