/*
 * Cold Reset Driver
 *
 * Copyright (c) 2017 Atmark Techno, Inc. All Rights Reserved.
 *
 * Based on: gpio-reset.c
 *   Copyright 2013 Philipp Zabel, Pengutronix
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/reset-controller.h>
#include <linux/regulator/consumer.h>

struct cold_reset_data {
	struct reset_controller_dev rcdev;
	struct reset_control *reset;
	struct regulator *power;

	int num_gpios;
	int *gpios;
};

#define to_cold_reset_data(_rcdev) \
	container_of(_rcdev, struct cold_reset_data, rcdev)

static int cold_reset_assert(struct reset_controller_dev *rcdev,
			     unsigned long id)
{
	struct cold_reset_data *drvdata = to_cold_reset_data(rcdev);
	int i;

	for (i = 0; i < drvdata->num_gpios; i++)
		gpio_direction_output(drvdata->gpios[i], 0);

	return regulator_disable(drvdata->power);
}

static int cold_reset_deassert(struct reset_controller_dev *rcdev,
			       unsigned long id)
{
	struct cold_reset_data *drvdata = to_cold_reset_data(rcdev);
	int i;
	int ret;

	ret = regulator_enable(drvdata->power);
	if (ret)
		return ret;

	for (i = 0; i < drvdata->num_gpios; i++)
		gpio_direction_output(drvdata->gpios[i], 1);

	if (drvdata->reset)
		ret = reset_control_reset(drvdata->reset);

	return ret;
}

static int cold_reset(struct reset_controller_dev *rcdev, unsigned long id)
{
	int ret;

	ret = cold_reset_assert(rcdev, id);
	if (ret)
		return ret;

	return cold_reset_deassert(rcdev, id);
}

static struct reset_control_ops cold_reset_ops = {
	.reset = cold_reset,
	.assert = cold_reset_assert,
	.deassert = cold_reset_deassert,
};


static ssize_t cold_reset_reset_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct platform_device *pdev= to_platform_device(dev);
	struct cold_reset_data *drvdata = platform_get_drvdata(pdev);
	int val, ret;

	ret = kstrtoint(buf, 0, &val);
	if (ret)
		return ret;

	if (val)
		ret = cold_reset(&drvdata->rcdev, 0);

	return ret ? : count;
}
static DEVICE_ATTR(reset, S_IWUSR, NULL, cold_reset_reset_store);

static int of_cold_reset_xlate(struct reset_controller_dev *rcdev,
			       const struct of_phandle_args *reset_spec)
{
	if (WARN_ON(reset_spec->args_count != 0))
		return -EINVAL;

	return 0;
}

static int cold_reset_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct cold_reset_data *drvdata;
	int *gpios;
	bool initially_in_reset;
	int i;
	int ret;

	drvdata = devm_kzalloc(&pdev->dev, sizeof(*drvdata), GFP_KERNEL);
	if (drvdata == NULL)
		return -ENOMEM;

	drvdata->reset = devm_reset_control_get(&pdev->dev, NULL);
	if (IS_ERR(drvdata->reset)) {
		if (PTR_ERR(drvdata->reset) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		drvdata->reset = NULL;
	}

	drvdata->power = devm_regulator_get(&pdev->dev, "power");
	if (IS_ERR(drvdata->power)) {
		dev_err(&pdev->dev, "Cannot get the regulator\n");
		return PTR_ERR(drvdata->power);
	}

	drvdata->num_gpios = of_gpio_named_count(np, "high-fixed-gpios");
	if (drvdata->num_gpios < 0)
		drvdata->num_gpios = 0;

	gpios = devm_kzalloc(&pdev->dev, sizeof(int) * drvdata->num_gpios,
			     GFP_KERNEL);
	if (gpios == NULL)
		return -ENOMEM;

	for (i = 0; i < drvdata->num_gpios; i++) {
		gpios[i] = of_get_named_gpio(np, "high-fixed-gpios", i);
		if (!gpio_is_valid(gpios[i])) {
			dev_err(&pdev->dev, "Invalid gpio %d\n", i);
			return gpios[i];
		}

		ret = devm_gpio_request(&pdev->dev, gpios[i], NULL);
		if (ret) {
			dev_err(&pdev->dev, "Failed to request gpio%d\n",
				gpios[i]);
			return ret;
		}
	}

	platform_set_drvdata(pdev, drvdata);

	drvdata->gpios = gpios;

	drvdata->rcdev.of_node = np;
	drvdata->rcdev.owner = THIS_MODULE;
	drvdata->rcdev.nr_resets = 1;
	drvdata->rcdev.ops = &cold_reset_ops;
	drvdata->rcdev.of_xlate = of_cold_reset_xlate;
	reset_controller_register(&drvdata->rcdev);

	initially_in_reset = of_property_read_bool(np, "initially-in-reset");
	if (initially_in_reset)
		cold_reset_assert(&drvdata->rcdev, 0);
	else
		cold_reset_deassert(&drvdata->rcdev, 0);

	ret = device_create_file(&pdev->dev, &dev_attr_reset);
	if (ret < 0)
		return ret;

	return 0;
}

static int cold_reset_remove(struct platform_device *pdev)
{
	struct cold_reset_data *drvdata = platform_get_drvdata(pdev);

	reset_controller_unregister(&drvdata->rcdev);

	return 0;
}

static struct of_device_id cold_reset_dt_ids[] = {
	{ .compatible = "cold-reset" },
	{ }
};

static struct platform_driver cold_reset_driver = {
	.probe = cold_reset_probe,
	.remove = cold_reset_remove,
	.driver = {
		.name = "cold-reset",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(cold_reset_dt_ids),
	},
};

static int __init cold_reset_init(void)
{
	return platform_driver_register(&cold_reset_driver);
}
subsys_initcall_sync(cold_reset_init);

static void __exit cold_reset_exit(void)
{
	platform_driver_unregister(&cold_reset_driver);
}
module_exit(cold_reset_exit);

MODULE_AUTHOR("Daisuke Mizobuchi <mizo@atmark-techno.com>");
MODULE_DESCRIPTION("Cold Reset Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:cold-reset");
MODULE_DEVICE_TABLE(of, cold_reset_dt_ids);
