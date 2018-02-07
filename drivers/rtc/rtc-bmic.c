/*
 * Copyright (C) 2018 Atmark Techno, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/rtc.h>
#include <linux/of.h>
#include <linux/of_gpio.h>

#define BMIC_RTC_MAJOR_VERSION(v)	((v >> 8) & 0xff)
#define BMIC_RTC_MINOR_VERSION(v)	(v & 0xff)

struct bmic_rtc {
	struct i2c_client *i2c;
	struct rtc_device *rtc;
	int irq_gpio;
};

#define REG_VERSION	0x01
#define REG_INT_STAT	0x02
#define		REG_INT_STAT_ALARM	(1<<0)
#define REG_INT_CONF	0x03
#define		REG_INT_CONF_ENABLE	(1<<0)
#define REG_TIME	0x04
#define REG_ALARM	0x05


static int bmic_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct bmic_rtc *rtc = dev_get_drvdata(dev);
	u32 time;
	int ret;

	ret = i2c_smbus_read_i2c_block_data(rtc->i2c, REG_TIME,
					    sizeof(u32), (u8 *)&time);
	if (ret < 0)
		return ret;

	rtc_time_to_tm(time, tm);

	return 0;
}

static int bmic_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct bmic_rtc *rtc = dev_get_drvdata(dev);
	unsigned long time;

	rtc_tm_to_time(tm, &time);

	return i2c_smbus_write_i2c_block_data(rtc->i2c, REG_TIME,
					      sizeof(u32), (u8 *)&time);
}

static void bmic_rtc_clear_interrupt(struct bmic_rtc *rtc)
{
	/* disable interrupt */
	i2c_smbus_write_byte_data(rtc->i2c, REG_INT_CONF, 0);

	/* clear interrupt status */
	i2c_smbus_write_byte_data(rtc->i2c, REG_INT_STAT, REG_INT_STAT_ALARM);
}

static irqreturn_t bmic_rtc_interrupt(int irq, void *private)
{
	struct bmic_rtc *rtc = private;
	int ret;

	ret = i2c_smbus_read_byte_data(rtc->i2c, REG_INT_STAT);
	if (ret < 0)
		goto out;

	if (ret & REG_INT_STAT_ALARM) {
		rtc_update_irq(rtc->rtc, 1, RTC_IRQF | RTC_AF);
		goto out;
	} else {
		/* this irq is shared ... */
		return IRQ_NONE;
	}

out:
	bmic_rtc_clear_interrupt(rtc);

	return IRQ_HANDLED;
}

static int bmic_rtc_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	struct bmic_rtc *rtc = dev_get_drvdata(dev);
	u8 val;

	if (enabled)
		val = REG_INT_CONF_ENABLE;
	else
		val = 0;

	return i2c_smbus_write_byte_data(rtc->i2c, REG_INT_CONF, val);
}

static int bmic_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	struct bmic_rtc *rtc = dev_get_drvdata(dev);
	u32 time;
	int ret;

	ret = i2c_smbus_read_i2c_block_data(rtc->i2c, REG_ALARM,
					    sizeof(u32), (u8 *)&time);
	if (ret < 0)
		return ret;
	rtc_time_to_tm(time, &alm->time);

	ret = i2c_smbus_read_byte_data(rtc->i2c, REG_INT_CONF);
	if (ret < 0)
		return ret;
	alm->enabled = ret & REG_INT_CONF_ENABLE;

	return 0;
}

static int bmic_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	struct bmic_rtc *rtc = dev_get_drvdata(dev);
	unsigned long time;
	int ret;

	rtc_tm_to_time(&alm->time, &time);

	ret = i2c_smbus_write_i2c_block_data(rtc->i2c, REG_ALARM,
					     sizeof(u32), (u8 *)&time);
	if (ret < 0)
		return ret;

	return bmic_rtc_alarm_irq_enable(dev, alm->enabled);
}

static struct rtc_class_ops bmic_rtc_ops = {
	.read_time	= bmic_rtc_read_time,
	.set_time	= bmic_rtc_set_time,
	.read_alarm	= bmic_rtc_read_alarm,
	.set_alarm	= bmic_rtc_set_alarm,
	.alarm_irq_enable = bmic_rtc_alarm_irq_enable,
};

static int bmic_rtc_remove(struct i2c_client *client)
{
	return 0;
}

static int bmic_rtc_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct bmic_rtc *rtc;
	struct device_node *np = client->dev.of_node;
	int ver;
	int ret;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_WORD_DATA))
		return -ENODEV;

	ver = i2c_smbus_read_word_data(client, REG_VERSION);
	dev_info(&client->dev, "version: %d.%d\n",
		 BMIC_RTC_MAJOR_VERSION(ver),
		 BMIC_RTC_MINOR_VERSION(ver));

	rtc = devm_kzalloc(&client->dev, sizeof(*rtc), GFP_KERNEL);
	if (!rtc)
		return -ENOMEM;

	rtc->i2c = client;

	rtc->irq_gpio = of_get_named_gpio(np, "irq-gpio", 0);
	if (!gpio_is_valid(rtc->irq_gpio)) {
		dev_err(&client->dev, "can't get irq-gpio\n");
		return -ENODEV;
	}

	ret = devm_request_threaded_irq(&client->dev,
					gpio_to_irq(rtc->irq_gpio),
					NULL,
					bmic_rtc_interrupt,
					IRQF_TRIGGER_FALLING |
					IRQF_ONESHOT | IRQF_SHARED,
					"bmic_rtc_irq", rtc);
	if (ret < 0) {
		dev_err(&client->dev, "can't get IRQ %d\n",
			gpio_to_irq(rtc->irq_gpio));
		return ret;
	}
	device_init_wakeup(&client->dev, true);

	dev_set_drvdata(&client->dev, rtc);

	rtc->rtc = devm_rtc_device_register(&client->dev, client->name,
					    &bmic_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc->rtc))
		return PTR_ERR(rtc->rtc);

	return 0;
}

static const struct i2c_device_id bmic_rtc_id[] = {
	{ "bmic_rtc", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, bmic_rtc_id);

#ifdef CONFIG_OF
static const struct of_device_id bmic_rtc_of_match[] = {
	{ .compatible = "at,bmic_rtc", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, bmic_rtc_of_match);
#endif

static struct i2c_driver bmic_rtc_driver = {
	.driver		= {
		.name	= "bmic_rtc",
		.of_match_table = of_match_ptr(bmic_rtc_of_match),
	},
	.probe		= bmic_rtc_probe,
	.remove		= bmic_rtc_remove,
	.id_table	= bmic_rtc_id,
};
module_i2c_driver(bmic_rtc_driver);

MODULE_AUTHOR("Atmark Techno, Inc.");
MODULE_DESCRIPTION("Atmark Techno BMIC RTC driver");
MODULE_LICENSE("GPL v2");
