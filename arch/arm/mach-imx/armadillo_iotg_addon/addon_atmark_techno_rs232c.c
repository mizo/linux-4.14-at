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

extern_dtb(addon_atmark_techno_rs232c_iotg_g3_intf1);
extern_dtb(addon_atmark_techno_rs232c_iotg_g3_intf2);
extern_dtb(addon_atmark_techno_rs232c_x1_intf1);

#define PIN_FORCEOFF	(42)
#define PIN_RI		(46)
#define PIN_DCD		(47)
#define PIN_DSR		(48)
#define PIN_DTR		(49)

int addon_setup_atmark_techno_rs232c(struct addon_device *adev)
{
	struct armadillo_iotg_addon *addon = to_addon(adev);
	void *begin;
	size_t size;
	int gpio_forceoff;
	int gpio_ri;
	int gpio_dcd;
	int gpio_dsr;
	int gpio_dtr;
	char label[64];

	switch (adev->intf) {
	case ADDON_INTERFACE1:
		switch (addon->type) {
		case ADDON_BOARD_TYPE_IOTG:
			begin = dtb_begin(addon_atmark_techno_rs232c_iotg_g3_intf1);
			size = dtb_size(addon_atmark_techno_rs232c_iotg_g3_intf1);
			break;
		case ADDON_BOARD_TYPE_X1:
			begin = dtb_begin(addon_atmark_techno_rs232c_x1_intf1);
			size = dtb_size(addon_atmark_techno_rs232c_x1_intf1);
			break;
		default:
			return -ENODEV;
		};
		break;
	case ADDON_INTERFACE2:
		switch (addon->type) {
		case ADDON_BOARD_TYPE_IOTG:
			begin = dtb_begin(addon_atmark_techno_rs232c_iotg_g3_intf2);
			size = dtb_size(addon_atmark_techno_rs232c_iotg_g3_intf2);
			break;
		default:
			return -ENODEV;
		};
		break;
	default:
		BUG();
	};
	armadillo_iotg_addon_dt_overlay(addon->dev, begin, size);

	gpio_forceoff = adev->gpios[PIN_FORCEOFF - 1];
	gpio_ri = adev->gpios[PIN_RI - 1];
	gpio_dcd = adev->gpios[PIN_DCD - 1];
	gpio_dsr = adev->gpios[PIN_DSR - 1];
	gpio_dtr = adev->gpios[PIN_DTR - 1];

	if (!gpio_is_valid(gpio_forceoff)) {
		dev_warn(addon->dev, "forceoff is invalid\n");
		return -EINVAL;
	}
	if (devm_gpio_request(addon->dev, gpio_forceoff, NULL)) {
		dev_warn(addon->dev, "forceoff request failed\n");
		return -EINVAL;
	}

	if (!gpio_is_valid(gpio_ri)) {
		dev_warn(addon->dev, "ri is invalid\n");
		return -EINVAL;
	}
	if (devm_gpio_request(addon->dev, gpio_ri, NULL)) {
		dev_warn(addon->dev, "ri request failed\n");
		return -EINVAL;
	}

	if (!gpio_is_valid(gpio_dcd)) {
		dev_warn(addon->dev, "dcd is invalid\n");
		return -EINVAL;
	}
	if (devm_gpio_request(addon->dev, gpio_dcd, NULL)) {
		dev_warn(addon->dev, "dcd request failed\n");
		return -EINVAL;
	}

	if (!gpio_is_valid(gpio_dsr)) {
		dev_warn(addon->dev, "dsr is invalid\n");
		return -EINVAL;
	}
	if (devm_gpio_request(addon->dev, gpio_dsr, NULL)) {
		dev_warn(addon->dev, "dsr request failed\n");
		return -EINVAL;
	}

	if (!gpio_is_valid(gpio_dtr)) {
		dev_warn(addon->dev, "dtr is invalid\n");
		return -EINVAL;
	}
	if (devm_gpio_request(addon->dev, gpio_dtr, NULL)) {
		dev_warn(addon->dev, "dtr request failed\n");
		return -EINVAL;
	}

	gpio_direction_output(gpio_forceoff, 1);
	gpio_direction_input(gpio_ri);
	gpio_direction_input(gpio_dcd);
	gpio_direction_input(gpio_dsr);
	gpio_direction_output(gpio_dtr, 0);

	gpio_export(gpio_forceoff, false);
	gpio_export(gpio_ri, false);
	gpio_export(gpio_dcd, false);
	gpio_export(gpio_dsr, false);
	gpio_export(gpio_dtr, false);

	snprintf(label, sizeof(label), "FORCEOFF_INTF%d", adev->intf);
	gpio_export_link(addon->dev, label, gpio_forceoff);
	snprintf(label, sizeof(label), "RI_INTF%d", adev->intf);
	gpio_export_link(addon->dev, label, gpio_ri);
	snprintf(label, sizeof(label), "DCD_INTF%d", adev->intf);
	gpio_export_link(addon->dev, label, gpio_dcd);
	snprintf(label, sizeof(label), "DSR_INTF%d", adev->intf);
	gpio_export_link(addon->dev, label, gpio_dsr);
	snprintf(label, sizeof(label), "DTR_INTF%d", adev->intf);
	gpio_export_link(addon->dev, label, gpio_dtr);

	return 0;
}
