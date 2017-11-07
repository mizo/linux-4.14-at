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

extern_dtb(addon_atmark_techno_wi_sun_iotg_g3_intf1);
extern_dtb(addon_atmark_techno_wi_sun_iotg_g3_intf2);
extern_dtb(addon_atmark_techno_wi_sun_x1_intf1);

#define PIN_RESET	(42)
#define PIN_NMIX	(43)

#define BP35A1_RESET_ASSERT	(0)
#define BP35A1_RESET_DEASSERT	(1)

int addon_setup_atmark_techno_wi_sun(struct addon_device *adev)
{
	struct armadillo_iotg_addon *addon = to_addon(adev);
	void *begin;
	size_t size;
	int gpio_reset;
	int gpio_nmix;

	switch (adev->intf) {
	case ADDON_INTERFACE1:
		switch (addon->type) {
		case ADDON_BOARD_TYPE_IOTG:
			begin = dtb_begin(addon_atmark_techno_wi_sun_iotg_g3_intf1);
			size = dtb_size(addon_atmark_techno_wi_sun_iotg_g3_intf1);
			break;
		case ADDON_BOARD_TYPE_X1:
			begin = dtb_begin(addon_atmark_techno_wi_sun_x1_intf1);
			size = dtb_size(addon_atmark_techno_wi_sun_x1_intf1);
			break;
		default:
			return -ENODEV;
		};
		break;
	case ADDON_INTERFACE2:
		switch (addon->type) {
		case ADDON_BOARD_TYPE_IOTG:
			begin = dtb_begin(addon_atmark_techno_wi_sun_iotg_g3_intf2);
			size = dtb_size(addon_atmark_techno_wi_sun_iotg_g3_intf2);
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
	gpio_nmix = adev->gpios[PIN_NMIX - 1];

	if (!gpio_is_valid(gpio_reset)) {
		dev_warn(addon->dev, "gpio_reset is invalid\n");
		return -EINVAL;
	}
	if (devm_gpio_request(addon->dev, gpio_reset, NULL)) {
		dev_warn(addon->dev, "gpio_reset request failed\n");
		return -EINVAL;
	}

	if (!gpio_is_valid(gpio_nmix)) {
		dev_warn(addon->dev, "gpio_nmix is invalid\n");
		return -EINVAL;
	}
	if (devm_gpio_request(addon->dev, gpio_nmix, NULL)) {
		dev_warn(addon->dev, "gpio_nmix request failed\n");
		return -EINVAL;
	}

	/* refered: reference circuit */
	gpio_direction_output(gpio_nmix, 1); /* always high */

	/* reset */
	gpio_direction_output(gpio_reset, BP35A1_RESET_ASSERT);
	ndelay(500);
	gpio_direction_output(gpio_reset, BP35A1_RESET_DEASSERT);

	return 0;
}
