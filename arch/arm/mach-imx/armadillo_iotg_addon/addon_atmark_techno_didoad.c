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

extern_dtb(addon_atmark_techno_didoad_iotg_g3_intf1);
extern_dtb(addon_atmark_techno_didoad_iotg_g3_intf2);
extern_dtb(addon_atmark_techno_didoad_x1_intf1);

#define PIN_ADUM1401_VE1	(43)
#define PIN_DI1			(48)
#define PIN_DI2			(47)
#define PIN_DO1			(24)
#define PIN_DO2			(25)

int addon_setup_atmark_techno_didoad(struct addon_device *adev)
{
	struct armadillo_iotg_addon *addon = to_addon(adev);
	void *begin;
	size_t size;
	int gpio_adum1401_ve1;
	int gpio_di1;
	int gpio_di2;
	int gpio_do1;
	int gpio_do2;
	char label[64];

	switch (adev->intf) {
	case ADDON_INTERFACE1:
		switch (addon->type) {
		case ADDON_BOARD_TYPE_IOTG:
			begin = dtb_begin(addon_atmark_techno_didoad_iotg_g3_intf1);
			size = dtb_size(addon_atmark_techno_didoad_iotg_g3_intf1);
			break;
		case ADDON_BOARD_TYPE_X1:
			begin = dtb_begin(addon_atmark_techno_didoad_x1_intf1);
			size = dtb_size(addon_atmark_techno_didoad_x1_intf1);
			break;
		default:
			return -ENODEV;
		};
		break;
	case ADDON_INTERFACE2:
		switch (addon->type) {
		case ADDON_BOARD_TYPE_IOTG:
			begin = dtb_begin(addon_atmark_techno_didoad_iotg_g3_intf2);
			size = dtb_size(addon_atmark_techno_didoad_iotg_g3_intf2);
			break;
		default:
			return -ENODEV;
		};
		break;
	default:
		BUG();
	};
	armadillo_iotg_addon_dt_overlay(addon->dev, begin, size);

	gpio_adum1401_ve1 = adev->gpios[PIN_ADUM1401_VE1 - 1];
	gpio_di1 = adev->gpios[PIN_DI1 - 1];
	gpio_di2 = adev->gpios[PIN_DI2 - 1];
	gpio_do1 = adev->gpios[PIN_DO1 - 1];
	gpio_do2 = adev->gpios[PIN_DO2 - 1];

	if (!gpio_is_valid(gpio_adum1401_ve1)) {
		dev_warn(addon->dev, "gpio_adum1401_ve1 is invalid\n");
		return -EINVAL;
	}
	if (devm_gpio_request(addon->dev, gpio_adum1401_ve1, NULL)) {
		dev_warn(addon->dev, "gpio_adum1401_ve1 request failed\n");
		return -EINVAL;
	}

	if (!gpio_is_valid(gpio_di1)) {
		dev_warn(addon->dev, "gpio_di1 is invalid\n");
		return -EINVAL;
	}
	if (devm_gpio_request(addon->dev, gpio_di1, NULL)) {
		dev_warn(addon->dev, "gpio_di1 request failed\n");
		return -EINVAL;
	}

	if (!gpio_is_valid(gpio_di2)) {
		dev_warn(addon->dev, "gpio_di2 is invalid\n");
		return -EINVAL;
	}
	if (devm_gpio_request(addon->dev, gpio_di2, NULL)) {
		dev_warn(addon->dev, "gpio_di2 request failed\n");
		return -EINVAL;
	}

	if (!gpio_is_valid(gpio_do1)) {
		dev_warn(addon->dev, "gpio_do1 is invalid\n");
		return -EINVAL;
	}
	if (devm_gpio_request(addon->dev, gpio_do1, NULL)) {
		dev_warn(addon->dev, "gpio_do1 request failed\n");
		return -EINVAL;
	}

	if (!gpio_is_valid(gpio_do2)) {
		dev_warn(addon->dev, "gpio_do2 is invalid\n");
		return -EINVAL;
	}
	if (devm_gpio_request(addon->dev, gpio_do2, NULL)) {
		dev_warn(addon->dev, "gpio_do2 request failed\n");
		return -EINVAL;
	}

	gpio_direction_output(gpio_adum1401_ve1, 1);
	gpio_direction_input(gpio_di1);
	gpio_direction_input(gpio_di2);
	gpio_direction_output(gpio_do1, 0);
	gpio_direction_output(gpio_do2, 0);

	gpio_export(gpio_adum1401_ve1, false);
	gpio_export(gpio_di1, false);
	gpio_export(gpio_di2, false);
	gpio_export(gpio_do1, false);
	gpio_export(gpio_do2, false);

	snprintf(label, sizeof(label), "ADUM1401_VE1_INTF%d", adev->intf);
	gpio_export_link(addon->dev, label, gpio_adum1401_ve1);
	snprintf(label, sizeof(label), "DI1_INTF%d", adev->intf);
	gpio_export_link(addon->dev, label, gpio_di1);
	snprintf(label, sizeof(label), "DI2_INTF%d", adev->intf);
	gpio_export_link(addon->dev, label, gpio_di2);
	snprintf(label, sizeof(label), "DO1_INTF%d", adev->intf);
	gpio_export_link(addon->dev, label, gpio_do1);
	snprintf(label, sizeof(label), "DO2_INTF%d", adev->intf);
	gpio_export_link(addon->dev, label, gpio_do2);

	return 0;
}
