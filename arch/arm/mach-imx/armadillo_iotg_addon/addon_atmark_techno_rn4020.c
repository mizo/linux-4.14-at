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

extern_dtb(addon_atmark_techno_rn4020_iotg_g3_intf1);
extern_dtb(addon_atmark_techno_rn4020_iotg_g3_intf2);
extern_dtb(addon_atmark_techno_rn4020_x1_intf1);

#define PIN_CMD_MLDP	(46)
#define PIN_WAKE_SW	(43)
#define PIN_WAKE_HW	(42)

#define RN4020_COMMAND_MODE	(1)
#define RN4020_MLDP_MODE	(0)
#define RN4020_WAKE_SW_ASSERT	(1)
#define RN4020_WAKE_SW_DEASSERT	(0)
#define RN4020_WAKE_HW_ASSERT	(1)
#define RN4020_WAKE_HW_DEASSERT	(0)

int addon_setup_atmark_techno_rn4020(struct addon_device *adev)
{
	struct armadillo_iotg_addon *addon = to_addon(adev);
	void *begin;
	size_t size;
	int gpio_cmd_mldp;
	int gpio_wake_sw;
	int gpio_wake_hw;
	char label[64];

	switch (adev->intf) {
	case ADDON_INTERFACE1:
		switch (addon->type) {
		case ADDON_BOARD_TYPE_IOTG:
			begin = dtb_begin(addon_atmark_techno_rn4020_iotg_g3_intf1);
			size = dtb_size(addon_atmark_techno_rn4020_iotg_g3_intf1);
			break;
		case ADDON_BOARD_TYPE_X1:
			begin = dtb_begin(addon_atmark_techno_rn4020_x1_intf1);
			size = dtb_size(addon_atmark_techno_rn4020_x1_intf1);
			break;
		default:
			return -ENODEV;
		};
		break;
	case ADDON_INTERFACE2:
		switch (addon->type) {
		case ADDON_BOARD_TYPE_IOTG:
			begin = dtb_begin(addon_atmark_techno_rn4020_iotg_g3_intf2);
			size = dtb_size(addon_atmark_techno_rn4020_iotg_g3_intf2);
			break;
		default:
			return -ENODEV;
		};
		break;
	default:
		BUG();
	};
	armadillo_iotg_addon_dt_overlay(addon->dev, begin, size);

	gpio_cmd_mldp = adev->gpios[PIN_CMD_MLDP - 1];
	gpio_wake_sw = adev->gpios[PIN_WAKE_SW - 1];
	gpio_wake_hw = adev->gpios[PIN_WAKE_HW - 1];

	if (!gpio_is_valid(gpio_cmd_mldp)) {
		dev_warn(addon->dev, "cmd_mldp is invalid\n");
		return -EINVAL;
	}
	if (devm_gpio_request(addon->dev, gpio_cmd_mldp, NULL)) {
		dev_warn(addon->dev, "cmd_mldp request failed\n");
		return -EINVAL;
	}

	if (!gpio_is_valid(gpio_wake_sw)) {
		dev_warn(addon->dev, "gpio_wake_sw is invalid\n");
		return -EINVAL;
	}
	if (devm_gpio_request(addon->dev, gpio_wake_sw, NULL)) {
		dev_warn(addon->dev, "gpio_wake_sw request failed\n");
		return -EINVAL;
	}

	if (!gpio_is_valid(gpio_wake_hw)) {
		dev_warn(addon->dev, "gpio_wake_hw is invalid\n");
		return -EINVAL;
	}
	if (devm_gpio_request(addon->dev, gpio_wake_hw, NULL)) {
		dev_warn(addon->dev, "gpio_wake_hw request failed\n");
		return -EINVAL;
	}

	gpio_direction_output(gpio_wake_sw, RN4020_WAKE_SW_ASSERT);
	gpio_direction_output(gpio_wake_hw, RN4020_WAKE_HW_DEASSERT);
	gpio_direction_output(gpio_cmd_mldp, RN4020_COMMAND_MODE);

	gpio_export(gpio_wake_sw, false);
	gpio_export(gpio_wake_hw, false);
	gpio_export(gpio_cmd_mldp, false);

	snprintf(label, sizeof(label), "RN4020_WAKE_SW_INTF%d", adev->intf);
	gpio_export_link(addon->dev, label, gpio_wake_sw);
	snprintf(label, sizeof(label), "RN4020_WAKE_HW_INTF%d", adev->intf);
	gpio_export_link(addon->dev, label, gpio_wake_hw);
	snprintf(label, sizeof(label), "RN4020_CMDMLDP_INTF%d", adev->intf);
	gpio_export_link(addon->dev, label, gpio_cmd_mldp);

	return 0;
}
