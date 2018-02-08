/*
 * Copyright (C) 2018 Atmark Techno, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/slab.h>

#define BMIC_REGULATOR_MAJOR_VERSION(v)	((v >> 8) & 0xff)
#define BMIC_REGULATOR_MINOR_VERSION(v)	(v & 0xff)

/* Register definitions */
#define REG_VERSION	0x01
#define REG_VAL		0x04

struct bmic_regulator {
	struct i2c_client *i2c;
	struct regulator_desc desc;
	struct regulator_dev *rdev;
};

static int bmic_regulator_get_voltage(struct regulator_dev *dev)
{
	struct bmic_regulator *bmic = rdev_get_drvdata(dev);
	u32 volt;
	int ret;

	ret = i2c_smbus_read_i2c_block_data(bmic->i2c, REG_VAL,
					    sizeof(u32), (u8 *)&volt);
	if (ret < 0)
		return ret;

	return volt;
}

static struct regulator_ops bmic_regulator_ops = {
	.get_voltage = bmic_regulator_get_voltage,
};

static int bmic_regulator_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct bmic_regulator *bmic;
	struct regulator_dev *rdev;
	struct regulator_config config = { };
	int ver;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_WORD_DATA))
		return -ENODEV;

	ver = i2c_smbus_read_word_data(client, REG_VERSION);
	dev_info(&client->dev, "version: %d.%d\n",
		 BMIC_REGULATOR_MAJOR_VERSION(ver),
		 BMIC_REGULATOR_MINOR_VERSION(ver));

	bmic = devm_kzalloc(&client->dev, sizeof(bmic), GFP_KERNEL);
	if (!bmic) {
		dev_err(&client->dev, "Memory allocation failed\n");
		return -ENOMEM;
	}

	bmic->desc.name = id->name;
	bmic->desc.id = -1;
	bmic->desc.type = REGULATOR_VOLTAGE;
	bmic->desc.owner = THIS_MODULE;
	bmic->desc.ops = &bmic_regulator_ops;

	config.init_data = of_get_regulator_init_data(&client->dev,
						      client->dev.of_node,
						      &bmic->desc);
	if (!config.init_data)
		return -EINVAL;

	bmic->i2c = client;

	/* Register the regulators */
	config.dev = &client->dev;
	config.driver_data = bmic;
	config.of_node = client->dev.of_node;

	rdev = devm_regulator_register(&client->dev, &bmic->desc, &config);
	if (IS_ERR(rdev)) {
		dev_err(&client->dev, "Regulator register failed\n");
		return PTR_ERR(rdev);
	}

	bmic->rdev = rdev;

	return 0;
}

static const struct i2c_device_id bmic_regulator_id[] = {
	{ "bmic_regulator", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, bmic_regulator_id);

#ifdef CONFIG_OF
static const struct of_device_id bmic_regulator_of_match[] = {
	{ .compatible = "at,bmic_regulator" },
	{},
};
MODULE_DEVICE_TABLE(of, bmic_regulator_of_match);
#endif

static struct i2c_driver bmic_regulator_i2c_driver = {
	.driver = {
		.name = "bmic_regulator",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(bmic_regulator_of_match),
	},
	.probe = bmic_regulator_probe,
	.id_table = bmic_regulator_id,
};

static int __init bmic_regulator_init(void)
{
	return i2c_add_driver(&bmic_regulator_i2c_driver);
}
subsys_initcall(bmic_regulator_init);

static void __exit bmic_regulator_cleanup(void)
{
	i2c_del_driver(&bmic_regulator_i2c_driver);
}
module_exit(bmic_regulator_cleanup);

MODULE_AUTHOR("Atmark Techno, Inc.");
MODULE_DESCRIPTION("Atmark Techno BMIC ADC driver");
MODULE_LICENSE("GPL v2");
