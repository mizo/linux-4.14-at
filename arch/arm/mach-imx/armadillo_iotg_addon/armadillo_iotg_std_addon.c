/*
 * Copyright (C) 2017-2018 Atmark Techno, Inc. All Rights Reserved.
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
#define DEBUG

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_fdt.h>
#include <linux/of_gpio.h>

#include "armadillo_iotg_std_addon.h"

/* EEPROM I2C slave address */
#define ADDON_EEPROM_ADDR1	(0x50) /* Add-On Module I/F 1 */
#define ADDON_EEPROM_ADDR2	(0x51) /* Add-On Module I/F 2 */

struct addon_vendor_name
{
        u16 vendor;
        const char *name;
};
#define VENDOR_NAME(v, n) { ADDON_VENDOR_ID_##v, n }

static struct addon_vendor_name vendor_names[] = {
	VENDOR_NAME(ATMARK_TECHNO, "Atmark Techno"),
	VENDOR_NAME(SATORI, "Satori"),
	VENDOR_NAME(ADVALY, "Advaly"),
};

static const char *unknownvendorname = "Unknown Vendor";

struct addon_product_name
{
        u16 vendor;
        u16 product;
        const char *name;
};
#define PRODUCT_NAME(v, p, n) { ADDON_VENDOR_ID_##v,	\
			ADDON_PRODUCT_ID_##v##_##p, n }

static struct addon_product_name product_names[] = {
	PRODUCT_NAME(ATMARK_TECHNO, WI_SUN, "Wi-SUN"),
	PRODUCT_NAME(ATMARK_TECHNO, EN_OCEAN, "EnOcean"),
	PRODUCT_NAME(ATMARK_TECHNO, SERIAL, "RS485/RS422/RS232C"),
	PRODUCT_NAME(ATMARK_TECHNO, DIDOAD, "DI/DO/AD"),
	PRODUCT_NAME(ATMARK_TECHNO, RN4020, "RN4020"),
	PRODUCT_NAME(ATMARK_TECHNO, CAN, "Can"),
	PRODUCT_NAME(ATMARK_TECHNO, ZIGBEE, "ZigBee"),
	PRODUCT_NAME(ATMARK_TECHNO, RS232C, "RS232C"),
	PRODUCT_NAME(ATMARK_TECHNO, RS485, "RS485"),
	PRODUCT_NAME(ATMARK_TECHNO, SD, "SD"),
	PRODUCT_NAME(SATORI, B_ROUTE, "B_ROUTE"),
	PRODUCT_NAME(SATORI, 920M, "920M"),
	PRODUCT_NAME(SATORI, LOW_POWER, "LOW_POWER"),
	PRODUCT_NAME(ADVALY, USBLAN, "USB/LAN"),
};

static const char *unknownproductname = "Unknown Product";

int armadillo_iotg_addon_dt_overlay(struct device *dev, void *begin, size_t size)
{
	static void *overlay_data;
	struct device_node *overlay;
	int ret;

	/*
	 * Must create permanent copy of FDT because of_fdt_unflatten_tree()
	 * will create pointers to the passed in FDT in the EDT.
	 */
	overlay_data = kmemdup(begin, size, GFP_KERNEL);
	if (overlay_data == NULL) {
		dev_err(dev, "Failed to allocate memory\n");
		ret = -ENOMEM;
		goto err;
	}

	of_fdt_unflatten_tree(overlay_data, NULL, &overlay);
	if (overlay == NULL) {
		dev_err(dev, "No tree to attach\n");
		ret = -EINVAL;
		goto err_free_overlay_data;
	}

	of_node_set_flag(overlay, OF_DETACHED);
	ret = of_resolve_phandles(overlay);
	if (ret != 0) {
		dev_err(dev, "Failed to resolve phandles\n");
		goto err_free_overlay_data;
	}

	ret = of_overlay_create(overlay);
	if (ret < 0) {
		dev_err(dev, "Failed to creating overlay\n");
		goto err_free_overlay_data;
	}

err_free_overlay_data:
	kfree(overlay_data);

err:
	return ret;
}

static const char *addon_get_vendor_name(u16 vendor)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(vendor_names); i++)
                if (vendor_names[i].vendor == vendor)
                        return vendor_names[i].name;

        return unknownvendorname;
}

static const char *addon_get_product_name(u16 vendor, u16 product)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(product_names); i++) {
                if ((product_names[i].vendor == vendor) &&
		    (product_names[i].product == product))
                        return product_names[i].name;
	}

        return unknownproductname;
}

static int addon_setup(struct addon_device *adev)
{
	struct addon_device_descriptor *desc = &adev->desc;
	struct armadillo_iotg_addon *addon = to_addon(adev);
	u16 vendor_id = be16_to_cpu(desc->vendor_id);
	u16 product_id = be16_to_cpu(desc->product_id);
	u16 revision = be16_to_cpu(desc->revision);
	u32 serial_no = be32_to_cpu(desc->serial_no);
	int ret = -ENODEV;

	dev_info(addon->dev, "%s %s board detected at "
		 "Add-On Module I/F %d(Rev %d, SerialNumber=%d).\n",
		 addon_get_vendor_name(vendor_id),
		 addon_get_product_name(vendor_id, product_id),
		 adev->intf, revision, serial_no);

	switch (vendor_id) {
	case ADDON_VENDOR_ID_ATMARK_TECHNO:
		switch (product_id) {
		case ADDON_PRODUCT_ID_ATMARK_TECHNO_WI_SUN:
			ret = addon_setup_atmark_techno_wi_sun(adev);
			break;
		case ADDON_PRODUCT_ID_ATMARK_TECHNO_EN_OCEAN:
			ret = addon_setup_atmark_techno_en_ocean(adev);
			break;
		case ADDON_PRODUCT_ID_ATMARK_TECHNO_SERIAL:
			ret = addon_setup_atmark_techno_serial(adev);
			break;
		case ADDON_PRODUCT_ID_ATMARK_TECHNO_DIDOAD:
			ret = addon_setup_atmark_techno_didoad(adev);
			break;
		case ADDON_PRODUCT_ID_ATMARK_TECHNO_RN4020:
			ret = addon_setup_atmark_techno_rn4020(adev);
			break;
		case ADDON_PRODUCT_ID_ATMARK_TECHNO_CAN:
			break;
		case ADDON_PRODUCT_ID_ATMARK_TECHNO_ZIGBEE:
			break;
		case ADDON_PRODUCT_ID_ATMARK_TECHNO_RS232C:
			ret = addon_setup_atmark_techno_rs232c(adev);
			break;
		case ADDON_PRODUCT_ID_ATMARK_TECHNO_RS485:
			ret = addon_setup_atmark_techno_rs485(adev);
			break;
		case ADDON_PRODUCT_ID_ATMARK_TECHNO_SD:
			ret = addon_setup_atmark_techno_sd(adev);
			break;
		default:
			break;
		}
		break;
	case ADDON_VENDOR_ID_SATORI:
		switch (product_id) {
		case ADDON_PRODUCT_ID_SATORI_B_ROUTE:
		case ADDON_PRODUCT_ID_SATORI_920M:
		case ADDON_PRODUCT_ID_SATORI_LOW_POWER:
			ret = addon_setup_satori_wireless(adev);
			break;
		default:
			break;
		}
		break;
	case ADDON_VENDOR_ID_ADVALY:
		switch (product_id) {
		case ADDON_PRODUCT_ID_ADVALY_USBLAN:
			ret = addon_setup_advaly_usblan(adev);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}

	if (ret) {
		dev_err(addon->dev, "Failed to initialize Add-On Module I/F %d.\n",
			adev->intf);
	}

	return ret;
}

static int addon_get_descriptor(struct addon_device_descriptor *desc,
				struct i2c_adapter *adap, u16 addr)
{
	union i2c_smbus_data data;
	unsigned char *p = (unsigned char *)desc;
	int i;
	int ret;

	for (i = 0; i < sizeof(struct addon_device_descriptor); i++) {
		ret = i2c_smbus_xfer(adap, addr, 0, I2C_SMBUS_READ,
				     i, I2C_SMBUS_BYTE_DATA, &data);
		if (ret)
			goto out;
		*(p + i) = data.byte;
	}

out:
	i2c_put_adapter(adap);

	return ret;
}

static void addon_get_gpios(struct addon_device *adev)
{
	struct armadillo_iotg_addon *addon = to_addon(adev);
	struct device_node *np = addon->dev->of_node;
	const char *name;
	int i;

	switch (adev->intf) {
	case ADDON_INTERFACE1:
		name = "intf1-gpios";
		break;
	case ADDON_INTERFACE2:
		name = "intf2-gpios";
		break;
	default:
		BUG();
	}

	for (i = 0; i < NR_ADDON_PINS; i++)
		adev->gpios[i] = of_get_named_gpio(np, name, i);
}

#define __ADDON_ATTR_RO(_name, _idx) {					\
	.attr	= { .name = __stringify(_name), .mode = S_IRUGO },	\
	.show	= _name##_idx##_show,					\
}

#define ADDON_DEVICE_ATTR_RO(_name, _idx)			\
	struct device_attribute dev_attr_##_name##_idx =	\
				__ADDON_ATTR_RO(_name, _idx)

#define ADDON_SHOW_BE16(_name, _idx)					\
static ssize_t _name##_idx##_show(struct device *dev,			\
			    struct device_attribute *attr, char *buf)	\
{									\
	struct armadillo_iotg_addon *addon = dev_get_drvdata(dev);	\
	return sprintf(buf, "%d\n",					\
		       be16_to_cpu(addon->adev[_idx].desc._name));	\
}									\
static ADDON_DEVICE_ATTR_RO(_name, _idx)

#define ADDON_SHOW_BE32(_name, _idx)					\
static ssize_t _name##_idx##_show(struct device *dev,			\
			    struct device_attribute *attr, char *buf)	\
{									\
	struct armadillo_iotg_addon *addon = dev_get_drvdata(dev);	\
	return sprintf(buf, "%d\n",					\
		       be32_to_cpu(addon->adev[_idx].desc._name));	\
}									\
static ADDON_DEVICE_ATTR_RO(_name, _idx)

ADDON_SHOW_BE16(vendor_id, 1);
ADDON_SHOW_BE16(vendor_id, 2);
ADDON_SHOW_BE16(product_id, 1);
ADDON_SHOW_BE16(product_id, 2);
ADDON_SHOW_BE16(revision, 1);
ADDON_SHOW_BE16(revision, 2);
ADDON_SHOW_BE32(serial_no, 1);
ADDON_SHOW_BE32(serial_no, 2);

static struct attribute *addon_attrs1[] = {
	&dev_attr_vendor_id1.attr,
	&dev_attr_product_id1.attr,
	&dev_attr_revision1.attr,
	&dev_attr_serial_no1.attr,
	NULL
};

static struct attribute *addon_attrs2[] = {
	&dev_attr_vendor_id2.attr,
	&dev_attr_product_id2.attr,
	&dev_attr_revision2.attr,
	&dev_attr_serial_no2.attr,
	NULL
};

static struct attribute_group addon_attr_group1 = {
	.name = "interface1",
	.attrs = addon_attrs1,
};

static struct attribute_group addon_attr_group2 = {
	.name = "interface2",
	.attrs = addon_attrs2,
};

static int addon_detect(struct addon_device *adev)
{
	u16 addr;
	const struct attribute_group *grp;
	struct armadillo_iotg_addon *addon;
	int ret;

	switch (adev->intf) {
	case ADDON_INTERFACE1:
		addr = ADDON_EEPROM_ADDR1;
		grp = &addon_attr_group1;
		break;
	case ADDON_INTERFACE2:
		addr = ADDON_EEPROM_ADDR2;
		grp = &addon_attr_group2;
		break;
	default:
		BUG();
	}

	addon = to_addon(adev);
	ret = addon_get_descriptor(&adev->desc, addon->adap, addr);
	if (!ret) {
		addon_get_gpios(adev);
		ret = addon_setup(adev);
		if (!ret) {
			ret = sysfs_create_group(&addon->dev->kobj, grp);
			if (ret) {
				dev_err(addon->dev,
					"Unable to create sysfs group\n");
				return ret;
			}
		}
	} else
		dev_info(addon->dev,
			 "No add-on expansion board detected at "
			 "Add-On Module I/F %d.\n", adev->intf);


	return 0;
}

#if defined(CONFIG_OF)
static const struct of_device_id armadillo_iotg_addon_dt_ids[] = {
	{ .compatible = "armadillo_iotg_addon", .data = (void *)ADDON_BOARD_TYPE_IOTG },
	{ .compatible = "armadillo_x1_addon", .data = (void *)ADDON_BOARD_TYPE_X1 },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, armadillo_iotg_addon_dt_ids);
#endif

static int armadillo_iotg_addon_probe(struct platform_device *pdev)
{
	struct device_node *node;
	const struct of_device_id *of_id;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct armadillo_iotg_addon *addon;
	u32 delay = 0;
	int ret;

	dev_dbg(dev, "probe\n");

	addon = devm_kzalloc(dev, sizeof(*addon), GFP_KERNEL);
	if (!addon) {
		ret = -ENOMEM;
		goto err;
	}

	addon->power_gpio = of_get_named_gpio(np, "power-gpio", 0);
	if (gpio_is_valid(addon->power_gpio)) {
		ret = devm_gpio_request_one(dev, addon->power_gpio,
					    GPIOF_OUT_INIT_HIGH, "AOM_PWREN");
		if (ret) {
			dev_err(dev, "Unable to get power gpio\n");
			goto err;
		}

		of_property_read_u32(np, "power-ramp-delay", &delay);
		mdelay(delay);
	}

	node = of_parse_phandle(dev->of_node, "addon-i2c-bus", 0);
	if (!node) {
		dev_err(dev, "No addon-i2c-bus node\n");
		ret = -EINVAL;
		goto err_power_off;
	}

	addon->adap = of_find_i2c_adapter_by_node(node);
	of_node_put(node);
	if (!addon->adap) {
		dev_err(dev, "Failed to find i2c adapter\n");
		ret = -ENODEV;
		goto err_power_off;
	}

	addon->dev = dev;
	of_id = of_match_device(armadillo_iotg_addon_dt_ids, dev);
	if (!of_id) {
		dev_err(dev, "Failed to find device match data\n");
		ret = -ENODEV;
		goto err_power_off;
	}
	addon->type = (enum addon_board_type)of_id->data;

	addon->adev[ADDON_INTERFACE1].intf = ADDON_INTERFACE1;
	ret = addon_detect(&addon->adev[ADDON_INTERFACE1]);
	if (ret)
		goto err_power_off;

	addon->adev[ADDON_INTERFACE2].intf = ADDON_INTERFACE2;
	ret = addon_detect(&addon->adev[ADDON_INTERFACE2]);
	if (ret)
		goto err_power_off;

	dev_set_drvdata(dev, addon);

	return 0;

err_power_off:
	if (gpio_is_valid(addon->power_gpio))
		gpio_set_value(addon->power_gpio, 0);
err:
	return ret;
}

static int armadillo_iotg_addon_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct armadillo_iotg_addon *addon = dev_get_drvdata(dev);

	dev_dbg(dev, "remove\n");

	if (gpio_is_valid(addon->power_gpio))
		gpio_set_value(addon->power_gpio, 0);


	return 0;
}

static struct platform_driver armadillo_iotg_addon_driver = {
	.driver		= {
		.name	= "armadillo_iotg_addon",
		.owner	= THIS_MODULE,
		.of_match_table	= of_match_ptr(armadillo_iotg_addon_dt_ids),
	},
	.probe		= armadillo_iotg_addon_probe,
	.remove		= armadillo_iotg_addon_remove,
};

static int __init armadillo_iotg_addon_init(void)
{
	int ret;

	ret = platform_driver_register(&armadillo_iotg_addon_driver);
	if (ret)
		printk(KERN_ERR "armadillo_iotg_addon: probe failed: %d\n", ret);

	return 0;
}
subsys_initcall_sync(armadillo_iotg_addon_init);

static void __exit armadillo_iotg_addon_exit(void)
{
	platform_driver_unregister(&armadillo_iotg_addon_driver);
}
module_exit(armadillo_iotg_addon_exit);

MODULE_AUTHOR("Atmark Techno, Inc.");
MODULE_DESCRIPTION("Add-On Module Auto Detect");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:armadillo_iotg_addon");
