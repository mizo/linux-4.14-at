/*
 * Copyright (C) 2019 Atmark Techno, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307	 USA
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

extern_dtb(addon_advaly_dio_iotg_g3_intf1);
extern_dtb(addon_advaly_dio_iotg_g3_intf2);
extern_dtb(addon_advaly_dio_x1_intf1);

#define PIN_DIO_SEL0		(43)
#define PIN_DIO_SEL1		(42)
#define PIN_DI0			(39)
#define PIN_DI1			(38)
#define PIN_DI2			(37)
#define PIN_DI3			(36)
#define PIN_DI4			(35)
#define PIN_DI5			(34)
#define PIN_DI6			(33)
#define PIN_DI7			(32)
#define PIN_DO0			(41)
#define PIN_DO1			(40)

enum sw_mode {
	SW_MODE_DO,
	SW_MODE_DI,
};

enum dio_index {
	INDEX_DIO_SEL0,
	INDEX_DIO_SEL1,
};

struct armadillo_iotg_addon *addon;

static int advaly_dio_setup_sw(struct gpio agpio,
				 struct addon_device *adev)
{
	int ret;

	ret = devm_gpio_request_one(addon->dev,
				    agpio.gpio,
				    agpio.flags,
				    agpio.label);
	if (ret) {
		dev_err(addon->dev,
			"failed to request gpio %s\n",
			agpio.label);
		return ret;
	}

	return 0;
}

static int advaly_dio_setup_gpio(struct gpio agpio,
				 struct addon_device *adev)
{
	char label[64];
	int ret;

	ret = devm_gpio_request_one(addon->dev,
				    agpio.gpio,
				    agpio.flags,
				    agpio.label);
	if (ret) {
		dev_err(addon->dev,
			"failed to request gpio %s\n",
			agpio.label);
		return ret;
	}

	snprintf(label, sizeof(label), "%s_INTF%d", agpio.label, adev->intf);
	gpio_export_link(addon->dev, label, agpio.gpio);

	return 0;
}

static inline void advaly_dio_teardown_sw(struct gpio agpio)
{
	devm_gpio_free(addon->dev, agpio.gpio);
}

int addon_setup_advaly_dio(struct addon_device *adev)
{
	void *begin;
	size_t size;
	const struct gpio sw_gpios[] = {
		{ adev->gpios[PIN_DIO_SEL0 - 1], GPIOF_DIR_IN, NULL },
		{ adev->gpios[PIN_DIO_SEL1 - 1], GPIOF_DIR_IN, NULL },
	};
	const struct gpio certainly_gpios[] = {
		{ adev->gpios[PIN_DI0 - 1], GPIOF_DIR_IN | GPIOF_EXPORT, "DI0" },
		{ adev->gpios[PIN_DI1 - 1], GPIOF_DIR_IN | GPIOF_EXPORT, "DI1" },
		{ adev->gpios[PIN_DI2 - 1], GPIOF_DIR_IN | GPIOF_EXPORT, "DI2" },
		{ adev->gpios[PIN_DI3 - 1], GPIOF_DIR_IN | GPIOF_EXPORT, "DI3" },
	};
	const struct gpio sw1_di_gpios[] = {
		{ adev->gpios[PIN_DI4 - 1], GPIOF_DIR_IN | GPIOF_EXPORT, "DI4" },
		{ adev->gpios[PIN_DI5 - 1], GPIOF_DIR_IN | GPIOF_EXPORT, "DI5" },
	};
	const struct gpio sw1_do_gpio = {
		adev->gpios[PIN_DO0 - 1], GPIOF_OUT_INIT_LOW | GPIOF_EXPORT, "DO0",
	};
	const struct gpio sw2_di_gpios[] = {
		{ adev->gpios[PIN_DI6 - 1], GPIOF_DIR_IN | GPIOF_EXPORT, "DI6" },
		{ adev->gpios[PIN_DI7 - 1], GPIOF_DIR_IN | GPIOF_EXPORT, "DI7" },
	};
	const struct gpio sw2_do_gpio = {
		adev->gpios[PIN_DO1 - 1], GPIOF_OUT_INIT_LOW | GPIOF_EXPORT, "DO1",
	};
	int ret;
	int i;

	addon = to_addon(adev);

	switch (adev->intf) {
	case ADDON_INTERFACE1:
		switch (addon->type) {
		case ADDON_BOARD_TYPE_IOTG:
			begin = dtb_begin(addon_advaly_dio_iotg_g3_intf1);
			size = dtb_size(addon_advaly_dio_iotg_g3_intf1);
			break;
		case ADDON_BOARD_TYPE_X1:
			begin = dtb_begin(addon_advaly_dio_x1_intf1);
			size = dtb_size(addon_advaly_dio_x1_intf1);
			break;
		default:
			return -ENODEV;
		};
		break;
	case ADDON_INTERFACE2:
		switch (addon->type) {
		case ADDON_BOARD_TYPE_IOTG:
			begin = dtb_begin(addon_advaly_dio_iotg_g3_intf2);
			size = dtb_size(addon_advaly_dio_iotg_g3_intf2);
			break;
		default:
			return -ENODEV;
		};
		break;
	default:
		BUG();
	};
	armadillo_iotg_addon_dt_overlay(addon->dev, begin, size);

	ret = advaly_dio_setup_sw(sw_gpios[INDEX_DIO_SEL0], adev);
	if (ret)
		goto err;

	ret = gpio_get_value(sw_gpios[INDEX_DIO_SEL0].gpio);
	if (ret == SW_MODE_DO) {
		ret = advaly_dio_setup_gpio(sw1_do_gpio, adev);
		if (ret)
			goto err_free_sw1;
	} else if (ret == SW_MODE_DI) {
		for (i = 0; i < ARRAY_SIZE(sw1_di_gpios); i++) {
			ret = advaly_dio_setup_gpio(sw1_di_gpios[i],
						    adev);
			if (ret)
				goto err_free_sw1;
		}
	} else {
		dev_err(addon->dev, "can not get value of sw1\n");
		goto err_free_sw1;
	}

	ret = advaly_dio_setup_sw(sw_gpios[INDEX_DIO_SEL1], adev);
	if (ret)
		goto err;

	ret = gpio_get_value(sw_gpios[INDEX_DIO_SEL1].gpio);
	if (ret == SW_MODE_DO) {
		ret = advaly_dio_setup_gpio(sw2_do_gpio, adev);
		if (ret)
			goto err_free_sw2;
	} else if (ret == SW_MODE_DI) {
		for (i = 0; i < ARRAY_SIZE(sw2_di_gpios); i++) {
			ret = advaly_dio_setup_gpio(sw2_di_gpios[i],
						    adev);
			if (ret)
				goto err_free_sw2;
		}
	} else {
		dev_err(addon->dev, "can not get value of sw2\n");
		goto err_free_sw2;
	}

	for (i = 0; i < ARRAY_SIZE(certainly_gpios); i++) {
		ret = advaly_dio_setup_gpio(certainly_gpios[i],
					    adev);
		if (ret)
			goto err;
	}

err_free_sw2:
	advaly_dio_teardown_sw(sw_gpios[INDEX_DIO_SEL1]);

err_free_sw1:
	advaly_dio_teardown_sw(sw_gpios[INDEX_DIO_SEL0]);

err:
	return ret;
}
