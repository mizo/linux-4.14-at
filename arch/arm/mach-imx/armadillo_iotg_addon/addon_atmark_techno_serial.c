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

extern_dtb(addon_atmark_techno_serial_iotg_g3_intf1);
extern_dtb(addon_atmark_techno_serial_iotg_g3_intf1_rs485);
extern_dtb(addon_atmark_techno_serial_iotg_g3_intf2);
extern_dtb(addon_atmark_techno_serial_iotg_g3_intf2_rs485);
extern_dtb(addon_atmark_techno_serial_x1_intf1);
extern_dtb(addon_atmark_techno_serial_x1_intf1_rs485);

#define PIN_XR3160_MODE	(42)
#define PIN_ADUM1402_VE1	(43)

#define XR3160_MODE_RS232C	(0)
#define XR3160_MODE_RS485	(1)

int addon_setup_atmark_techno_serial(struct addon_device *adev)
{
	struct armadillo_iotg_addon *addon = to_addon(adev);
	void *begin;
	size_t size;
	int gpio_xr3160_mode;
	int gpio_adum1402_ve1;
	char label[64];
	int mode;

	switch (adev->intf) {
	case ADDON_INTERFACE1:
		switch (addon->type) {
		case ADDON_BOARD_TYPE_IOTG:
			begin = dtb_begin(addon_atmark_techno_serial_iotg_g3_intf1);
			size = dtb_size(addon_atmark_techno_serial_iotg_g3_intf1);
			break;
		case ADDON_BOARD_TYPE_X1:
			begin = dtb_begin(addon_atmark_techno_serial_x1_intf1);
			size = dtb_size(addon_atmark_techno_serial_x1_intf1);
			break;
		default:
			return -ENODEV;
		};
		break;
	case ADDON_INTERFACE2:
		switch (addon->type) {
		case ADDON_BOARD_TYPE_IOTG:
			begin = dtb_begin(addon_atmark_techno_serial_iotg_g3_intf2);
			size = dtb_size(addon_atmark_techno_serial_iotg_g3_intf2);
			break;
		default:
			return -ENODEV;
		};
		break;
	default:
		BUG();
	};
	armadillo_iotg_addon_dt_overlay(addon->dev, begin, size);

	gpio_xr3160_mode = adev->gpios[PIN_XR3160_MODE - 1];
	gpio_adum1402_ve1 = adev->gpios[PIN_ADUM1402_VE1 - 1];

	if (!gpio_is_valid(gpio_xr3160_mode)) {
		dev_warn(addon->dev, "gpio_xr3160_mode is invalid\n");
		return -EINVAL;
	}
	if (devm_gpio_request(addon->dev, gpio_xr3160_mode, NULL)) {
		dev_warn(addon->dev, "gpio_xr3160_mode request failed\n");
		return -EINVAL;
	}

	if (!gpio_is_valid(gpio_adum1402_ve1)) {
		dev_warn(addon->dev, "gpio_adum1402_ve1 is invalid\n");
		return -EINVAL;
	}
	if (devm_gpio_request(addon->dev, gpio_adum1402_ve1, NULL)) {
		dev_warn(addon->dev, "gpio_adum1402_ve1 request failed\n");
		return -EINVAL;
	}

	gpio_direction_input(gpio_xr3160_mode);
	gpio_direction_output(gpio_adum1402_ve1, 1);

	gpio_export(gpio_xr3160_mode, false);

	snprintf(label, sizeof(label), "XR3160_MODE_INTF%d", adev->intf);
	gpio_export_link(addon->dev, label, gpio_xr3160_mode);

	mode = gpio_get_value(gpio_xr3160_mode);
	if (mode == XR3160_MODE_RS485) {
		switch (adev->intf) {
		case ADDON_INTERFACE1:
			switch (addon->type) {
			case ADDON_BOARD_TYPE_IOTG:
				begin = dtb_begin(addon_atmark_techno_serial_iotg_g3_intf1_rs485);
				size = dtb_size(addon_atmark_techno_serial_iotg_g3_intf1_rs485);
				break;
			case ADDON_BOARD_TYPE_X1:
				begin = dtb_begin(addon_atmark_techno_serial_x1_intf1_rs485);
				size = dtb_size(addon_atmark_techno_serial_x1_intf1_rs485);
				break;
			default:
				BUG();
			};
			break;
		case ADDON_INTERFACE2:
			switch (addon->type) {
			case ADDON_BOARD_TYPE_IOTG:
				begin = dtb_begin(addon_atmark_techno_serial_iotg_g3_intf2_rs485);
				size = dtb_size(addon_atmark_techno_serial_iotg_g3_intf2_rs485);
				break;
			default:
				BUG();
			};
			break;
		default:
			BUG();
		};
		armadillo_iotg_addon_dt_overlay(addon->dev, begin, size);
	}

	return 0;
}
