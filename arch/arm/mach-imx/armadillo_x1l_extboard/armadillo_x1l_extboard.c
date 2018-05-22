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
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_fdt.h>
#include <linux/of_gpio.h>

#include "armadillo_x1l_extboard.h"

/* EEPROM I2C slave address */
#define EXTBOARD_EEPROM_ADDR	(0x50)

struct extboard_vendor_name
{
        u16 vendor;
        const char *name;
};
#define VENDOR_NAME(v, n) { EXTBOARD_VENDOR_ID_##v, n }

static struct extboard_vendor_name vendor_names[] = {
	VENDOR_NAME(ATMARK_TECHNO, "Atmark Techno"),
};

static const char *unknownvendorname = "Unknown Vendor";

struct extboard_product_name
{
        u16 vendor;
        u16 product;
        const char *name;
};
#define PRODUCT_NAME(v, p, n) { EXTBOARD_VENDOR_ID_##v,	\
			EXTBOARD_PRODUCT_ID_##v##_##p, n }

static struct extboard_product_name product_names[] = {
	PRODUCT_NAME(ATMARK_TECHNO, 01,
		     "ExtBoard01"),
	PRODUCT_NAME(ATMARK_TECHNO, 01_ELS31,
		     "ExtBoard01 without WL1837MOD"),
	PRODUCT_NAME(ATMARK_TECHNO, 01_WL1837,
		     "ExtBoard01 without ELS31-J"),
	PRODUCT_NAME(ATMARK_TECHNO, 01_NONE,
		     "ExtBoard01 without WL1837MOD/ELS31-J"),
};

static const char *unknownproductname = "Unknown Product";

struct armadillo_x1l_extboard {
	struct i2c_adapter *adap;
};

int armadillo_x1l_extboard_dt_overlay(struct device *dev, void *begin,
				      size_t size)
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

static const char *extboard_get_vendor_name(u16 vendor)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(vendor_names); i++)
                if (vendor_names[i].vendor == vendor)
                        return vendor_names[i].name;

        return unknownvendorname;
}

static const char *extboard_get_product_name(u16 vendor, u16 product)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(product_names); i++) {
                if ((product_names[i].vendor == vendor) &&
		    (product_names[i].product == product))
                        return product_names[i].name;
	}

        return unknownproductname;
}

static void extboard_setup(struct extboard_device *edev)
{
	struct extboard_device_descriptor *desc = &edev->desc;
	u16 vendor_id = be16_to_cpu(desc->vendor_id);
	u16 product_id = be16_to_cpu(desc->product_id);
	u16 revision = be16_to_cpu(desc->revision);
	u32 serial_no = be32_to_cpu(desc->serial_no);
	int ret = -ENODEV;

	dev_info(edev->dev, "%s %s board detected "
		 " (Rev %d, SerialNumber=%d).\n",
		 extboard_get_vendor_name(vendor_id),
		 extboard_get_product_name(vendor_id, product_id),
		 revision, serial_no);

	switch (vendor_id) {
	case EXTBOARD_VENDOR_ID_ATMARK_TECHNO:
		switch (product_id) {
		case EXTBOARD_PRODUCT_ID_ATMARK_TECHNO_01:
		case EXTBOARD_PRODUCT_ID_ATMARK_TECHNO_01_ELS31:
		case EXTBOARD_PRODUCT_ID_ATMARK_TECHNO_01_WL1837:
		case EXTBOARD_PRODUCT_ID_ATMARK_TECHNO_01_NONE:
			ret = extboard_setup_atmark_techno_01(edev);
			break;
		default:
			break;
		}
	default:
		break;
	}

	if (ret)
		dev_err(edev->dev, "Failed to initialize Extension Board.\n");
}

static int extboard_get_descriptor(struct extboard_device_descriptor *desc,
				struct i2c_adapter *adap)
{
	union i2c_smbus_data data;
	unsigned char *p = (unsigned char *)desc;
	int i;
	int ret;

	for (i = 0; i < sizeof(struct extboard_device_descriptor); i++) {
		ret = i2c_smbus_xfer(adap, EXTBOARD_EEPROM_ADDR, 0,
				     I2C_SMBUS_READ, i, I2C_SMBUS_BYTE_DATA,
				     &data);
		if (ret)
			goto out;
		*(p + i) = data.byte;
	}

out:
	i2c_put_adapter(adap);

	return ret;
}

static void extboard_get_gpios(struct device_node *np,
			    struct extboard_device *edev)
{
	int i;

	for (i = 0; i < NR_EXTBOARD_PINS; i++)
		edev->gpios[i] = of_get_named_gpio(np, "gpios", i);
}

#if defined(CONFIG_OF)
static const struct of_device_id armadillo_x1l_extboard_dt_ids[] = {
	{ .compatible = "armadillo_x1l_extboard" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, armadillo_x1l_extboard_dt_ids);
#endif

static int armadillo_x1l_extboard_probe(struct platform_device *pdev)
{
	struct device_node *node;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct armadillo_x1l_extboard *extboard;
	struct extboard_device edev;
	int ret;

	dev_dbg(dev, "probe\n");

	extboard = devm_kzalloc(dev, sizeof(*extboard), GFP_KERNEL);
	if (!extboard)
		return -ENOMEM;

	node = of_parse_phandle(dev->of_node, "extboard-i2c-bus", 0);
	if (!node) {
		dev_err(dev, "No extboard-i2c-bus node\n");
		return -EINVAL;
	}

	extboard->adap = of_find_i2c_adapter_by_node(node);
	of_node_put(node);
	if (!extboard->adap) {
		dev_err(dev, "Failed to find i2c adapter\n");
		return -ENODEV;
	}

	edev.dev = dev;
	ret = extboard_get_descriptor(&edev.desc, extboard->adap);
	if (ret) {
		dev_info(dev, "No extension board detected.\n");
		return 0;
	}

	extboard_get_gpios(np, &edev);
	extboard_setup(&edev);

	return 0;
}

static int armadillo_x1l_extboard_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	dev_dbg(dev, "remove\n");

	return 0;
}

static struct platform_driver armadillo_x1l_extboard_driver = {
	.driver		= {
		.name	= "armadillo_x1l_extboard",
		.owner	= THIS_MODULE,
		.of_match_table	= of_match_ptr(armadillo_x1l_extboard_dt_ids),
	},
	.probe		= armadillo_x1l_extboard_probe,
	.remove		= armadillo_x1l_extboard_remove,
};

static int __init armadillo_x1l_extboard_init(void)
{
	int ret;

	ret = platform_driver_register(&armadillo_x1l_extboard_driver);
	if (ret)
		printk(KERN_ERR "armadillo_x1l_extboard: probe failed: %d\n", ret);

	return 0;
}
subsys_initcall_sync(armadillo_x1l_extboard_init);

static void __exit armadillo_x1l_extboard_exit(void)
{
	platform_driver_unregister(&armadillo_x1l_extboard_driver);
}
module_exit(armadillo_x1l_extboard_exit);

MODULE_AUTHOR("Atmark Techno, Inc.");
MODULE_DESCRIPTION("Extension Board Auto Detect for Armadillo-X1L");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:armadillo_x1l_extboard");
