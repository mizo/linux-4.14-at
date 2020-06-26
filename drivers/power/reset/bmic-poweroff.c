/*
 * Copyright (C) 2019 Atmark Techno, Inc. All Rights Reserved.
 *
 * Based on: gpio-poweroff.c
 *   Copyright (C) 2012 Jamie Lentin
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/reboot.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/i2c.h>

#define BMIC_POWEROFF_MAJOR_VERSION(v)	((v >> 8) & 0xff)
#define BMIC_POWEROFF_MINOR_VERSION(v)	(v & 0xff)

#define MINIMUM_RESTART_DELAY (1)

#define REG_VERSION	0x01
#define REG_TIME	0x04

struct bmic_poweroff {
	struct i2c_client *client;
};

struct bmic_restart {
	struct notifier_block restart_handler;
	struct i2c_client *client;
	u32 delay;
};

static struct gpio_desc *reset_gpio;

static void bmic_poweroff_do_poweroff(void)
{
	BUG_ON(!reset_gpio);

	gpiod_direction_output(reset_gpio, 1);

	/* give it some time */
	mdelay(3000);

	WARN_ON(1);
}

static int bmic_poweroff_set_turn_on_delay(struct i2c_client *client, u32 delay)
{
	int ret;

	ret = i2c_smbus_write_i2c_block_data(client, REG_TIME,
					     sizeof(delay), (u8 *)&delay);
	if (ret)
		dev_err(&client->dev, "failed to set turn-on-delay\n");

	return ret;
}

static int bmic_poweroff_get_turn_on_delay(struct i2c_client *client, u32 *delay)
{
	int ret;

	ret = i2c_smbus_read_i2c_block_data(client, REG_TIME,
					    sizeof(*delay), (u8 *)delay);
	if (ret < sizeof(delay)) {
		dev_err(&client->dev, "failed to get turn-on-delay\n");
		if (ret >= 0)
			ret = -EIO;
	}

	return ret;
}

static ssize_t turn_on_delay_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct bmic_poweroff *poweroff = dev_get_drvdata(dev);
	u32 val;
	int ret;

	ret = bmic_poweroff_get_turn_on_delay(poweroff->client, &val);
	if (ret != sizeof(val))
		return ret;

	return sprintf(buf, "%u\n", val);
}

static ssize_t turn_on_delay_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct bmic_poweroff *poweroff = dev_get_drvdata(dev);
	u32 val;
	int ret;

	ret = kstrtou32(buf, 0, &val);
	if (ret)
		return ret;

	ret = bmic_poweroff_set_turn_on_delay(poweroff->client, val);
	if (ret)
		return ret;

	return count;
}

static int bmic_restart_notify(struct notifier_block *this,
			       unsigned long mode, void *cmd)
{
	int ret;
	struct bmic_restart *bmic_restart =
		container_of(this, struct bmic_restart, restart_handler);

	/* override turn-on-delay */
	ret = bmic_poweroff_set_turn_on_delay(bmic_restart->client,
					      bmic_restart->delay);
	if (ret)
		return NOTIFY_BAD;

	bmic_poweroff_do_poweroff();

	return NOTIFY_DONE;
}

static DEVICE_ATTR(turn_on_delay, S_IRUGO | S_IWUSR,
		   turn_on_delay_show, turn_on_delay_store);

static struct attribute *bmic_poweroff_attrs[] = {
	&dev_attr_turn_on_delay.attr,
	NULL
};

static struct attribute_group bmic_poweroff_attr_group = {
	.name = NULL, /* put in device directory */
	.attrs = bmic_poweroff_attrs,
};

static int bmic_poweroff_probe(struct i2c_client *client,
			       const struct i2c_device_id *id)
{
	struct bmic_poweroff *poweroff;
	struct device *dev = &client->dev;
	struct device_node *of_node = dev->of_node;
	struct bmic_restart *bmic_restart;
	u32 delay;
	int ver;
	int ret;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_WORD_DATA |
				     I2C_FUNC_SMBUS_BLOCK_DATA))
		return -ENODEV;

	ver = i2c_smbus_read_word_data(client, REG_VERSION);
	if (ver < 0 || ver == 0xffff) {
		dev_err(dev, "failed to obtain version\n");
		return -ENODEV;
	}
	dev_info(dev, "version: %d.%d\n",
		 BMIC_POWEROFF_MAJOR_VERSION(ver),
		 BMIC_POWEROFF_MINOR_VERSION(ver));

	poweroff = devm_kzalloc(dev, sizeof(*poweroff), GFP_KERNEL);
	if (!poweroff)
		return -ENOMEM;
	poweroff->client = client;

	reset_gpio = devm_gpiod_get(dev, NULL, GPIOD_OUT_LOW);
	if (IS_ERR(reset_gpio))
		return PTR_ERR(reset_gpio);

	ret = of_property_read_u32(of_node, "turn-on-delay", &delay);
	if (!ret) {
		ret = bmic_poweroff_set_turn_on_delay(client, delay);
		if (ret)
			return ret;
	}

	ret = sysfs_create_group(&dev->kobj, &bmic_poweroff_attr_group);
	if (ret) {
		dev_err(dev, "unable to create sysfs group\n");
		return ret;
	}

	dev_set_drvdata(dev, poweroff);

	if (pm_power_off != NULL)
		dev_info(dev, "overrided pm_power_off function");
	pm_power_off = &bmic_poweroff_do_poweroff;

	bmic_restart = devm_kzalloc(dev, sizeof(*bmic_restart),
				    GFP_KERNEL);
	if (!bmic_restart)
		return -ENOMEM;

	ret = of_property_read_u32(of_node, "restart-delay", &delay);
	if (ret || !delay) {
		dev_warn(dev, "restart-delay uses the default value 1\n");
		delay = MINIMUM_RESTART_DELAY;
	}

	bmic_restart->restart_handler.notifier_call = bmic_restart_notify;
	bmic_restart->restart_handler.priority = 192;
	bmic_restart->delay = delay;
	bmic_restart->client = client;

	ret = register_restart_handler(&bmic_restart->restart_handler);
	if (ret) {
		dev_err(dev, "cannot register restart handler\n");
		return -ENODEV;
	}

	return 0;
}

static int bmic_poweroff_remove(struct i2c_client *client)
{
	struct bmic_restart *bmic_restart = dev_get_drvdata(&client->dev);
	int ret;

	ret = unregister_restart_handler(&bmic_restart->restart_handler);
	if (ret)
		dev_warn(&client->dev,"cannot unregister restart handler\n");

	if (pm_power_off == &bmic_poweroff_do_poweroff)
		pm_power_off = NULL;

	return 0;
}

static const struct i2c_device_id bmic_poweroff_id[] = {
	{ "bmic_poweroff", 0, },
	{ }
};
MODULE_DEVICE_TABLE(i2c, bmic_poweroff_id);

#ifdef CONFIG_OF
static const struct of_device_id bmic_poweroff_dt_ids[] = {
	{ .compatible = "at,bmic_poweroff", },
	{ }
};
MODULE_DEVICE_TABLE(of, bmic_poweroff_dt_ids);
#endif

static struct i2c_driver bmic_poweroff_driver = {
	.driver = {
		.name	= "bmic_poweroff",
		.of_match_table = of_match_ptr(bmic_poweroff_dt_ids),
	},
	.probe		= bmic_poweroff_probe,
	.remove		= bmic_poweroff_remove,
	.id_table	= bmic_poweroff_id,
};

static int __init bmic_poweroff_init(void)
{
	return i2c_add_driver(&bmic_poweroff_driver);
}
late_initcall(bmic_poweroff_init);

static void __exit bmic_poweroff_exit(void)
{
	i2c_del_driver(&bmic_poweroff_driver);
}
module_exit(bmic_poweroff_exit);

MODULE_AUTHOR("Atmark Techno, Inc.");
MODULE_DESCRIPTION("Atmark Techno BMIC poweroff driver");
MODULE_LICENSE("GPL v2");
