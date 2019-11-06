/*
 * GPIO-based Reset Driver for Quectel EC25
 *
 * Copyright (c) 2018 Atmark Techno, Inc. All Rights Reserved.
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
#include <linux/reset-controller.h>

struct ec25_reset_data {
	struct reset_controller_dev rcdev;

	unsigned int gpio_pwrkey;
	unsigned int gpio_reset;
	unsigned int gpio_status;

	bool active_low_pwrkey;
	bool active_low_reset;

	bool abort_safe_reset;

	struct work_struct work;
};

#define EC25_PWRKEY_POWER_ON_ASSERT_TIME_MS	(500)
#define EC25_PWRKEY_POWER_OFF_ASSERT_TIME_MS	(650)
#define EC25_RESET_ASSERT_TIME_MS		(460)
#define EC25_POWER_OFF_WAIT_TIME_MIN_MS		(29500)

#define to_ec25_reset_data(_rcdev) \
	container_of(_rcdev, struct ec25_reset_data, rcdev)

static void ec25_reset(unsigned int gpio, bool active_low, unsigned long delay_ms)
{
	gpio_set_value_cansleep(gpio, !active_low);
	mdelay(delay_ms);
	gpio_set_value_cansleep(gpio, active_low);
}

static void ec25_safe_reset(struct reset_controller_dev *rcdev)
{
	struct ec25_reset_data *drvdata = to_ec25_reset_data(rcdev);

	/* If power is already on */
	if (!gpio_get_value(drvdata->gpio_status)) {
		ec25_reset(drvdata->gpio_pwrkey,
				drvdata->active_low_pwrkey, EC25_PWRKEY_POWER_OFF_ASSERT_TIME_MS);

		/* We need to wait for power off at least 29.5s. */
		msleep(EC25_POWER_OFF_WAIT_TIME_MIN_MS);

		/* But max time is not defined. */
		while (!gpio_get_value(drvdata->gpio_status)) {
			/* Aborted by force reset */
			if (drvdata->abort_safe_reset)
				return;

			msleep(1);
		}
	}

	ec25_reset(drvdata->gpio_pwrkey,
			drvdata->active_low_pwrkey, EC25_PWRKEY_POWER_ON_ASSERT_TIME_MS);
}

static void ec25_safe_reset_work(struct work_struct *ws)
{
	struct ec25_reset_data *drvdata =
		container_of(ws, struct ec25_reset_data, work);

	ec25_safe_reset(&drvdata->rcdev);
}

static int ec25_force_reset(struct reset_controller_dev *rcdev, unsigned long id)
{
	struct ec25_reset_data *drvdata = to_ec25_reset_data(rcdev);

	if (!drvdata->abort_safe_reset) {
		/* Abort safe reset */
		drvdata->abort_safe_reset = true;
		cancel_work_sync(&drvdata->work);
	}

	ec25_reset(drvdata->gpio_reset,
			drvdata->active_low_reset, EC25_RESET_ASSERT_TIME_MS);

	ec25_reset(drvdata->gpio_pwrkey,
			drvdata->active_low_pwrkey, EC25_PWRKEY_POWER_ON_ASSERT_TIME_MS);

	return 0;
}

static struct reset_control_ops ec25_reset_ops = {
	.reset = ec25_force_reset,
};

static ssize_t ec25_reset_reset_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct platform_device *pdev= to_platform_device(dev);
	struct ec25_reset_data *drvdata = platform_get_drvdata(pdev);
	int val, ret;

	ret = kstrtoint(buf, 0, &val);
	if (ret)
		return ret;

	if (val)
		ret = ec25_force_reset(&drvdata->rcdev, 0);

	return ret ? : count;
}
static DEVICE_ATTR(reset, S_IWUSR, NULL, ec25_reset_reset_store);

static int of_ec25_reset_xlate(struct reset_controller_dev *rcdev,
				const struct of_phandle_args *reset_spec)
{
	if (WARN_ON(reset_spec->args_count != 0))
		return -EINVAL;

	return 0;
}

static int ec25_reset_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct ec25_reset_data *drvdata;
	enum of_gpio_flags flags;
	unsigned long gpio_flags;
	int ret;

	drvdata = devm_kzalloc(&pdev->dev, sizeof(*drvdata), GFP_KERNEL);
	if (drvdata == NULL)
		return -ENOMEM;

	if (of_gpio_named_count(np, "gpio-pwrkey") != 1) {
		dev_err(&pdev->dev,
			"gpio-pwrkey property missing, or not a single gpio\n");
		return -EINVAL;
	}

	drvdata->gpio_pwrkey = of_get_named_gpio_flags(np, "gpio-pwrkey", 0, &flags);
	if (drvdata->gpio_pwrkey == -EPROBE_DEFER) {
		return drvdata->gpio_pwrkey;
	} else if (!gpio_is_valid(drvdata->gpio_pwrkey)) {
		dev_err(&pdev->dev, "invalid pwrkey gpio: %d\n", drvdata->gpio_pwrkey);
		return drvdata->gpio_pwrkey;
	}

	drvdata->active_low_pwrkey = flags & OF_GPIO_ACTIVE_LOW;

	if (of_gpio_named_count(np, "gpio-reset") != 1) {
		dev_err(&pdev->dev,
			"gpio-reset property missing, or not a single gpio\n");
		return -EINVAL;
	}

	drvdata->gpio_reset = of_get_named_gpio_flags(np, "gpio-reset", 0, &flags);
	if (drvdata->gpio_reset == -EPROBE_DEFER) {
		return drvdata->gpio_reset;
	} else if (!gpio_is_valid(drvdata->gpio_reset)) {
		dev_err(&pdev->dev, "invalid reset gpio: %d\n", drvdata->gpio_reset);
		return drvdata->gpio_reset;
	}

	drvdata->active_low_reset = flags & OF_GPIO_ACTIVE_LOW;

	if (of_gpio_named_count(np, "gpio-status") != 1) {
		dev_err(&pdev->dev,
			"gpio-status property missing, or not a single gpio\n");
		return -EINVAL;
	}

	drvdata->gpio_status = of_get_named_gpio_flags(np, "gpio-status", 0, &flags);
	if (drvdata->gpio_status == -EPROBE_DEFER) {
		return drvdata->gpio_status;
	} else if (!gpio_is_valid(drvdata->gpio_status)) {
		dev_err(&pdev->dev, "invalid status gpio: %d\n", drvdata->gpio_status);
		return drvdata->gpio_status;
	}

	if (drvdata->active_low_pwrkey)
		gpio_flags = GPIOF_OUT_INIT_HIGH;
	else
		gpio_flags = GPIOF_OUT_INIT_LOW;

	ret = devm_gpio_request_one(&pdev->dev, drvdata->gpio_pwrkey, gpio_flags, NULL);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to request gpio %d: %d\n",
			drvdata->gpio_pwrkey, ret);
		return ret;
	}

	if (drvdata->active_low_reset)
		gpio_flags = GPIOF_OUT_INIT_HIGH;
	else
		gpio_flags = GPIOF_OUT_INIT_LOW;

	ret = devm_gpio_request_one(&pdev->dev, drvdata->gpio_reset, gpio_flags, NULL);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to request gpio %d: %d\n",
			drvdata->gpio_reset, ret);
		return ret;
	}

	ret = devm_gpio_request_one(&pdev->dev, drvdata->gpio_status, GPIOF_IN, NULL);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to request gpio %d: %d\n",
			drvdata->gpio_status, ret);
		return ret;
	}

	gpio_export(drvdata->gpio_status, false);
	gpio_export_link(&pdev->dev, "ec25_status", drvdata->gpio_status);

	platform_set_drvdata(pdev, drvdata);

	drvdata->abort_safe_reset = false;
	INIT_WORK(&drvdata->work, &ec25_safe_reset_work);
	schedule_work(&drvdata->work);

	drvdata->rcdev.of_node = np;
	drvdata->rcdev.owner = THIS_MODULE;
	drvdata->rcdev.nr_resets = 1;
	drvdata->rcdev.ops = &ec25_reset_ops;
	drvdata->rcdev.of_xlate = of_ec25_reset_xlate;
	reset_controller_register(&drvdata->rcdev);

	return device_create_file(&pdev->dev, &dev_attr_reset);
}

static int ec25_reset_remove(struct platform_device *pdev)
{
	struct ec25_reset_data *drvdata = platform_get_drvdata(pdev);

	drvdata->abort_safe_reset = true;
	cancel_work_sync(&drvdata->work);

	gpio_unexport(drvdata->gpio_status);

	reset_controller_unregister(&drvdata->rcdev);

	return 0;
}

static struct of_device_id ec25_reset_dt_ids[] = {
	{ .compatible = "ec25-reset" },
	{ }
};

static struct platform_driver ec25_reset_driver = {
	.probe = ec25_reset_probe,
	.remove = ec25_reset_remove,
	.driver = {
		.name = "ec25-reset",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ec25_reset_dt_ids),
	},
};

static int __init ec25_reset_init(void)
{
	return platform_driver_register(&ec25_reset_driver);
}
arch_initcall(ec25_reset_init);

static void __exit ec25_reset_exit(void)
{
	platform_driver_unregister(&ec25_reset_driver);
}
module_exit(ec25_reset_exit);

MODULE_AUTHOR("Takumi Ando <takumi.ando@atmark-techno.com>");
MODULE_DESCRIPTION("Quectel EC25 Reset Controller");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:ec25-reset");
MODULE_DEVICE_TABLE(of, ec25_reset_dt_ids);
