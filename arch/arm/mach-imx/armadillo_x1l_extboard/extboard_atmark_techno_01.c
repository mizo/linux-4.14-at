/*
 * Copyright (C) 2018 Atmark Techno, Inc. All Rights Reserved.
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

#include "armadillo_x1l_extboard.h"

#define extboard01_dt_overlay(dev, name)			\
	armadillo_x1l_extboard_dt_overlay(dev,			\
		dtb_begin(extboard_atmark_techno_01_##name),	\
		dtb_size(extboard_atmark_techno_01_##name))

extern_dtb(extboard_atmark_techno_01_base);
extern_dtb(extboard_atmark_techno_01_els31);
extern_dtb(extboard_atmark_techno_01_wl1837);

#define PIN_AOM_PWREN	(52)
#define PIN_NMIX	(5)
#define PIN_RESET	(6)
#define PIN_LTE_GPIO12_3	(50)
#define PIN_LTE_VUSB		(14)
#define PIN_LTE_SYS_REST_N	(46)
#define PIN_WLAN_PWR_EN		(54)
#define PIN_WLAN_IRQ		(13)

#define BP35A1_RESET_ASSERT	(0)
#define BP35A1_RESET_DEASSERT	(1)

static bool have_els31(u16 product_id)
{
	if ((product_id == EXTBOARD_PRODUCT_ID_ATMARK_TECHNO_01) ||
	    (product_id == EXTBOARD_PRODUCT_ID_ATMARK_TECHNO_01_ELS31))
		return true;
	return false;
}

static bool have_wl1837(u16 product_id)
{
	if ((product_id == EXTBOARD_PRODUCT_ID_ATMARK_TECHNO_01) ||
	    (product_id == EXTBOARD_PRODUCT_ID_ATMARK_TECHNO_01_WL1837))
		return true;
	return false;
}

static int extboard_gpio_request(struct device *dev, unsigned gpio,
				 const char *label)
{
	if (!gpio_is_valid(gpio)) {
		dev_warn(dev, "%s gpio is invalid\n", label);
		return -EINVAL;
	}
	if (devm_gpio_request(dev, gpio, NULL)) {
		dev_warn(dev, "%s gpio request failed\n", label);
		return -EINVAL;
	}

	return 0;
}

int extboard_setup_atmark_techno_01(struct extboard_device *edev)
{
	struct extboard_device_descriptor *desc = &edev->desc;
	u16 product_id = be16_to_cpu(desc->product_id);
	int gpio_aom_pwren;
	int gpio_reset;
	int gpio_nmix;
	int ret;

	extboard01_dt_overlay(edev->dev, base);

	if (have_els31(product_id)) {
		int gpio_lte_gpio12_3;
		int gpio_lte_vusb;
		int gpio_lte_sys_rest_n;

		extboard01_dt_overlay(edev->dev, els31);

		gpio_lte_gpio12_3 = edev->gpios[PIN_LTE_GPIO12_3 - 1];;
		gpio_lte_vusb = edev->gpios[PIN_LTE_VUSB - 1];;
		gpio_lte_sys_rest_n = edev->gpios[PIN_LTE_SYS_REST_N - 1];;

		ret = extboard_gpio_request(edev->dev, gpio_lte_gpio12_3,
					    "ELS31_GPIO12_3");
		if (ret)
			return ret;
		gpio_direction_output(gpio_lte_gpio12_3, 1);

		ret = extboard_gpio_request(edev->dev, gpio_lte_vusb,
					    "ELS31_LTE_VUSB");
		if (ret)
			return ret;
		gpio_direction_output(gpio_lte_vusb, 1);

		ret = extboard_gpio_request(edev->dev, gpio_lte_sys_rest_n,
					    "ELS31_SYS_REST_N");
		if (ret)
			return ret;
		gpio_direction_input(gpio_lte_sys_rest_n);
	}


	if (have_wl1837(product_id)) {
		int gpio_wlan_pwr_en;
		int gpio_wlan_irq;

		extboard01_dt_overlay(edev->dev, wl1837);

		gpio_wlan_pwr_en = edev->gpios[PIN_WLAN_PWR_EN - 1];;
		gpio_wlan_irq = edev->gpios[PIN_WLAN_IRQ - 1];;

		ret = extboard_gpio_request(edev->dev, gpio_wlan_pwr_en,
					    "WL1837_WLAN_PWR_EN");
		if (ret)
			return ret;
		gpio_direction_output(gpio_wlan_pwr_en, 1);

		ret = extboard_gpio_request(edev->dev, gpio_wlan_irq,
					    "WL1837_WLAN_IRQ");
		if (ret)
			return ret;
		gpio_direction_input(gpio_wlan_irq);
	}

	gpio_aom_pwren = edev->gpios[PIN_AOM_PWREN - 1];
	gpio_nmix = edev->gpios[PIN_NMIX - 1];
	gpio_reset = edev->gpios[PIN_RESET - 1];

	ret = extboard_gpio_request(edev->dev, gpio_aom_pwren, "AOM_PWREN");
	if (ret)
		return ret;
	gpio_direction_output(gpio_aom_pwren, 1);
	mdelay(3); /* power-ramp-delay */

	ret = extboard_gpio_request(edev->dev, gpio_nmix, "BP35A1_NMIX");
	if (ret)
		return ret;
	gpio_direction_output(gpio_nmix, 1);

	ret = extboard_gpio_request(edev->dev, gpio_reset, "BP35A1_RESET");
	if (ret)
		return ret;
	/* reset */
	gpio_direction_output(gpio_reset, BP35A1_RESET_ASSERT);
	ndelay(500);
	gpio_direction_output(gpio_reset, BP35A1_RESET_DEASSERT);

	return 0;
}
