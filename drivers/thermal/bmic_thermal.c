/*
 * Copyright (C) 2018 Atmark Techno, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/thermal.h>
#include <linux/regulator/consumer.h>

#define BMIC_THERMAL_MAJOR_VERSION(v)	((v >> 8) & 0xff)
#define BMIC_THERMAL_MINOR_VERSION(v)	(v & 0xff)

#define BMIC_THERMAL_RESOLUTION 12
#define BMIC_THERMAL_VALUE_MASK ((1<<BMIC_THERMAL_RESOLUTION)-1)

#define REG_VERSION	0x01
#define REG_INT_STAT	0x02
#define		REG_INT_STAT_VALID	(1<<0)
#define REG_INT_CONF	0x03
#define		REG_INT_CONF_ACFGT_PORT1	(1<<3)
#define		REG_INT_CONF_ACREN_PORT1	(1<<2)
#define		REG_INT_CONF_ENABLE_PORT1	(1<<0)
#define REG_CONV	0x04
#define REG_CV1		0x05
#define REG_CV2		0x06


struct bmic_thermal {
	struct i2c_client *i2c;
	struct regulator *ref;
	struct thermal_zone_device *tz;
};

#define TYP_M		(1620) /* uV/mCelsius */
#define TYP_VTEMP25	(716) /* mV */
static int bmic_adc_to_temp(struct bmic_thermal *thermal, int adc)
{
	int temp;
	int m = TYP_M;
	int vtemp25 = TYP_VTEMP25;
	int vtemp;
	int volt;

	volt = regulator_get_voltage(thermal->ref);
	volt /= 1000; /* uV to mV */

	vtemp = adc * volt / (1<<BMIC_THERMAL_RESOLUTION);

	temp = 25000 - ((vtemp - vtemp25) * 1000 * 1000 / m);

	return temp;
}

static int bmic_get_temp(struct thermal_zone_device *tz, int *temp)
{
	struct bmic_thermal *thermal = tz->devdata;
	u32 conv;
	int ret;

	ret = i2c_smbus_read_i2c_block_data(thermal->i2c, REG_CONV,
					    sizeof(conv), (u8 *)&conv);
	if (ret < 0)
		return ret;

	if (conv & ~BMIC_THERMAL_VALUE_MASK)
		return -EINVAL;

	*temp = bmic_adc_to_temp(thermal, conv);

	return 0;
}

static struct thermal_zone_device_ops ops = {
	.get_temp = bmic_get_temp,
};

static int bmic_thermal_probe(struct i2c_client *client,
			      const struct i2c_device_id *id)
{
	struct bmic_thermal *thermal;
	int ver;
	int ret;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_WORD_DATA))
		return -ENODEV;

	ver = i2c_smbus_read_word_data(client, REG_VERSION);
	dev_info(&client->dev, "version: %d.%d\n",
		 BMIC_THERMAL_MAJOR_VERSION(ver),
		 BMIC_THERMAL_MINOR_VERSION(ver));

	thermal = devm_kzalloc(&client->dev, sizeof(*thermal), GFP_KERNEL);
	if (!thermal)
		return -ENOMEM;

	thermal->i2c = client;

	thermal->ref = devm_regulator_get(&client->dev, "vref");
	if (IS_ERR(thermal->ref)) {
		dev_err(&client->dev, "can't get vref regulator\n");
		return PTR_ERR(thermal->ref);
	}

	ret = regulator_enable(thermal->ref);
	if (ret < 0) {
		dev_err(&client->dev, "can't enable vref regulator\n");
		return ret;
	}

	thermal->tz = thermal_zone_device_register("bmic_thermal", 0, 0,
						   thermal, &ops, NULL, 0, 0);
	if (IS_ERR(thermal->tz)) {
		dev_err(&client->dev,
			"Failed to register thermal zone device\n");
		ret = PTR_ERR(thermal->tz);
		goto regulator_disable;
	}

	i2c_set_clientdata(client, thermal);

	return 0;

regulator_disable:
	regulator_disable(thermal->ref);

	return ret;
}

static int bmic_thermal_remove(struct i2c_client *client)
{
	struct bmic_thermal *thermal = i2c_get_clientdata(client);

	regulator_disable(thermal->ref);
	thermal_zone_device_unregister(thermal->tz);

	return 0;
}

static const struct i2c_device_id bmic_thermal_id[] = {
	{ "bmic_thermal", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, bmic_thermal_id);

#ifdef CONFIG_OF
static const struct of_device_id bmic_thermal_of_match[] = {
	{ .compatible = "at,bmic_thermal" },
	{ }
};
MODULE_DEVICE_TABLE(of, bmic_thermal_of_match);
#endif

static struct i2c_driver bmic_thermal_driver = {
	.driver = {
		.name = "bmic_thermal",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(bmic_thermal_of_match),
	},
	.probe = bmic_thermal_probe,
	.remove = bmic_thermal_remove,
	.id_table = bmic_thermal_id,
};
module_i2c_driver(bmic_thermal_driver);

MODULE_AUTHOR("Atmark Techno, Inc.");
MODULE_DESCRIPTION("Atmark Techno BMIC thermal driver");
MODULE_LICENSE("GPL v2");
