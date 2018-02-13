/*
 * Copyright (C) 2018 Atmark Techno, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>

#include <linux/iio/iio.h>
#include <linux/iio/events.h>
#include <linux/regulator/consumer.h>

struct bmic_adc {
	struct i2c_client *i2c;
	struct regulator *ref;
	unsigned int irq_gpio;
};

#define BMIC_ADC_MAJOR_VERSION(v)	((v >> 8) & 0xff)
#define BMIC_ADC_MINOR_VERSION(v)	(v & 0xff)

enum {
	BMIC_ADC_PORT1 = 0,
	BMIC_ADC_PORT2,
	BMIC_ADC_PORT_NR,
};

#define BMIC_ADC_RESOLUTION 12
#define BMIC_ADC_VALUE_MASK ((1<<BMIC_ADC_RESOLUTION)-1)

#define REG_VERSION	0x01
#define REG_INT_STAT	0x02
#define		REG_INT_STAT_PORT2	(1<<1)
#define		REG_INT_STAT_PORT1	(1<<0)
#define REG_INT_CONF	0x03
#define		REG_INT_CONF_ACFGT_PORT2	(1<<5)
#define		REG_INT_CONF_ACREN_PORT2	(1<<4)
#define		REG_INT_CONF_ACFGT_PORT1	(1<<3)
#define		REG_INT_CONF_ACREN_PORT1	(1<<2)
#define		REG_INT_CONF_ENABLE_PORT2	(1<<1)
#define		REG_INT_CONF_ENABLE_PORT1	(1<<0)
#define REG_CONV_PORT1	0x04
#define REG_CONV_PORT2	0x05
#define REG_CV1_PORT1	0x06
#define REG_CV2_PORT1	0x07
#define REG_CV1_PORT2	0x08
#define REG_CV2_PORT2	0x09

static int bmic_adc_get_event_direction(struct bmic_adc *adc, int port)
{
	u8 acfgt, acren;
	int ret;

	switch (port) {
	case BMIC_ADC_PORT1:
		acfgt = REG_INT_CONF_ACFGT_PORT1;
		acren = REG_INT_CONF_ACREN_PORT1;
		break;
	case BMIC_ADC_PORT2:
		acfgt = REG_INT_CONF_ACFGT_PORT2;
		acren = REG_INT_CONF_ACREN_PORT2;
		break;
	default:
		return -EINVAL;
	}

	ret = i2c_smbus_read_byte_data(adc->i2c, REG_INT_CONF);
	if (ret < 0)
		return ret;

	if ((ret & acfgt) && !(ret & acren))
		return IIO_EV_DIR_RISING;
	if (!(ret & acfgt) && !(ret & acren))
		return IIO_EV_DIR_FALLING;

	return -EINVAL;
}

static int bmic_adc_set_event_direction(struct bmic_adc *adc, int port,
					enum iio_event_direction dir)
{
	u8 acfgt, acren;
	u8 val;
	int ret;

	switch (port) {
	case BMIC_ADC_PORT1:
		acfgt = REG_INT_CONF_ACFGT_PORT1;
		acren = REG_INT_CONF_ACREN_PORT1;
		break;
	case BMIC_ADC_PORT2:
		acfgt = REG_INT_CONF_ACFGT_PORT2;
		acren = REG_INT_CONF_ACREN_PORT2;
		break;
	default:
		return -EINVAL;
	}

	switch (dir) {
	case IIO_EV_DIR_FALLING:
		val = 0;
		break;
	case IIO_EV_DIR_RISING:
		val = acfgt;
		break;
	default:
		return -EINVAL;
	}

	ret = i2c_smbus_read_byte_data(adc->i2c, REG_INT_CONF);
	ret &= ~(acfgt | acren);
	ret |= val;

	return i2c_smbus_write_byte_data(adc->i2c, REG_INT_CONF, val);
}

static int bmic_adc_read_raw(struct iio_dev *iio,
			     struct iio_chan_spec const *chan,
			     int *val, int *val2, long mask)
{
	struct bmic_adc *adc = iio_priv(iio);
	u8 cmd;
	u32 conv;
	int volt;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		switch (chan->address) {
		case BMIC_ADC_PORT1:
			cmd = REG_CONV_PORT1;
			break;
		case BMIC_ADC_PORT2:
			cmd = REG_CONV_PORT2;
			break;
		default:
			return -EINVAL;
		}
		ret = i2c_smbus_read_i2c_block_data(adc->i2c, cmd,
						    sizeof(u32), (u8 *)&conv);
		if (ret < 0)
			return ret;

		*val = conv & BMIC_ADC_VALUE_MASK;
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		volt = regulator_get_voltage(adc->ref);
		if (volt < 0)
			return volt;

		*val = volt / 1000;
		*val2 = BMIC_ADC_RESOLUTION;

		return IIO_VAL_FRACTIONAL_LOG2;

	default:
		break;
	}

	return -EINVAL;
}

static int bmic_adc_read_event_config(struct iio_dev *iio,
				      const struct iio_chan_spec *chan,
				      enum iio_event_type type,
				      enum iio_event_direction dir)
{
	struct bmic_adc *adc = iio_priv(iio);
	u8 enable;
	int conf;

	switch (chan->address) {
	case BMIC_ADC_PORT1:
		enable = REG_INT_CONF_ENABLE_PORT1;
		break;
	case BMIC_ADC_PORT2:
		enable = REG_INT_CONF_ENABLE_PORT2;
		break;
	default:
		return -EINVAL;
	}

	conf = i2c_smbus_read_byte_data(adc->i2c, REG_INT_CONF);
	if (conf < 0)
		return conf;

	return !!(conf & enable);
}

static int bmic_adc_write_event_config(struct iio_dev *iio,
				       const struct iio_chan_spec *chan,
				       enum iio_event_type type,
				       enum iio_event_direction dir,
				       int state)
{
	struct bmic_adc *adc = iio_priv(iio);
	u8 enable;
	int conf;
	int ret;

	ret = bmic_adc_set_event_direction(adc, chan->address, dir);
	if (ret < 0)
		return ret;

	switch (chan->address) {
	case BMIC_ADC_PORT1:
		enable = REG_INT_CONF_ENABLE_PORT1;
		break;
	case BMIC_ADC_PORT2:
		enable = REG_INT_CONF_ENABLE_PORT2;
		break;
	default:
		return -EINVAL;
	}

	conf = i2c_smbus_read_byte_data(adc->i2c, REG_INT_CONF);
	if (conf < 0)
		return conf;

	if (state)
		conf |= enable;
	else
		conf &= ~enable;

	return i2c_smbus_write_byte_data(adc->i2c, REG_INT_CONF, conf);
}

static int bmic_adc_read_event_value(struct iio_dev *iio,
				     const struct iio_chan_spec *chan,
				     enum iio_event_type type,
				     enum iio_event_direction dir,
				     enum iio_event_info info,
				     int *val, int *val2)
{
	struct bmic_adc *adc = iio_priv(iio);
	u8 cmd;
	u32 conv;
	int ret;

	switch (chan->address) {
	case BMIC_ADC_PORT1:
		cmd = REG_CV1_PORT1;
		break;
	case BMIC_ADC_PORT2:
		cmd = REG_CV1_PORT2;
		break;
	default:
		return -EINVAL;
	}

	ret = i2c_smbus_read_i2c_block_data(adc->i2c, cmd,
					    sizeof(u32), (u8 *)&conv);
	if (ret < 0)
		return ret;

	*val = conv & BMIC_ADC_VALUE_MASK;

	return IIO_VAL_INT;
}

static int bmic_adc_write_event_value(struct iio_dev *iio,
				      const struct iio_chan_spec *chan,
				      enum iio_event_type type,
				      enum iio_event_direction dir,
				      enum iio_event_info info,
				      int val, int val2)
{
	struct bmic_adc *adc = iio_priv(iio);
	u8 cmd;

	switch (chan->address) {
	case BMIC_ADC_PORT1:
		cmd = REG_CV1_PORT1;
		break;
	case BMIC_ADC_PORT2:
		cmd = REG_CV1_PORT2;
		break;
	default:
		return -EINVAL;
	}

	if (val & ~BMIC_ADC_VALUE_MASK)
		return -EINVAL;

	return i2c_smbus_write_i2c_block_data(adc->i2c, cmd,
					      sizeof(u32), (u8 *)&val);
}

static int bmic_adc_get_event(struct bmic_adc *adc)
{
	int stat;

	stat = i2c_smbus_read_byte_data(adc->i2c, REG_INT_STAT);
	if (stat < 0)
		return stat;

	if (stat & REG_INT_STAT_PORT1)
		return BMIC_ADC_PORT1;
	if (stat & REG_INT_STAT_PORT2)
		return BMIC_ADC_PORT2;

	return BMIC_ADC_PORT_NR;
}

static void bmic_adc_clear_event(struct bmic_adc *adc, u8 port)
{
	u8 enable, stat;
	int conf;

	switch (port) {
	case BMIC_ADC_PORT1:
		enable = REG_INT_CONF_ENABLE_PORT1;
		stat = REG_INT_STAT_PORT1;
		break;
	case BMIC_ADC_PORT2:
		enable = REG_INT_CONF_ENABLE_PORT2;
		stat = REG_INT_STAT_PORT2;
		break;
	default:
		return;
	}

	/* disable interrupt */
	conf = i2c_smbus_read_byte_data(adc->i2c, REG_INT_CONF);
	conf &= ~enable;
	i2c_smbus_write_byte_data(adc->i2c, REG_INT_CONF, conf);

	/* clear interrupt status */
	i2c_smbus_write_byte_data(adc->i2c, REG_INT_STAT, stat);
}

static void bmic_adc_clear_event_all(struct bmic_adc *adc)
{
	int i;
	for (i = 0; i < BMIC_ADC_PORT_NR; i++)
		bmic_adc_clear_event(adc, BIT(i));
}

static irqreturn_t bmic_adc_event_handler(int irq, void *private)
{
	struct iio_dev *iio = private;
	struct bmic_adc *adc = iio_priv(iio);
	enum iio_event_direction ev_dir;
	u64 ev_code;
	int port;
	int handled = 0;
	int i;

	for (i = 0; i < BMIC_ADC_PORT_NR; i++) {
		port = bmic_adc_get_event(adc);
		if (port < 0)
			goto out;
		if (port == BMIC_ADC_PORT_NR)
			break;

		ev_dir = bmic_adc_get_event_direction(adc, port);
		if (ev_dir < 0)
			goto out;

		ev_code = IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE, port,
					       IIO_EV_TYPE_THRESH,
					       ev_dir);
		iio_push_event(iio, ev_code, iio_get_time_ns(iio));

		bmic_adc_clear_event(adc, port);

		handled = 1;
	}

	if (!handled) {
		/* this irq is shared ... */
		return IRQ_NONE;
	}

	return IRQ_HANDLED;

out:
	bmic_adc_clear_event_all(adc);

	return IRQ_HANDLED;
}

#define BMIC_ADC_THRESH_EVENT(di, sp)		\
	{					\
		.type = IIO_EV_TYPE_THRESH,	\
		.dir = di,			\
		.mask_separate = sp,		\
	}


static const struct iio_event_spec bmic_adc_events[] = {
	BMIC_ADC_THRESH_EVENT(IIO_EV_DIR_FALLING,
			      BIT(IIO_EV_INFO_VALUE) |
			      BIT(IIO_EV_INFO_ENABLE)),
	BMIC_ADC_THRESH_EVENT(IIO_EV_DIR_RISING,
			      BIT(IIO_EV_INFO_VALUE) |
			      BIT(IIO_EV_INFO_ENABLE)),
};

#define BMIC_ADC_VOLTAGE_CHANNEL(idx, num, res)			\
	{							\
		.type = IIO_VOLTAGE,				\
		.indexed = 1,					\
		.channel = num,					\
		.address = num,					\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),	\
		.info_mask_shared_by_type =			\
			BIT(IIO_CHAN_INFO_SCALE),		\
		.scan_index = idx,				\
		.event_spec = bmic_adc_events,			\
		.num_event_specs = ARRAY_SIZE(bmic_adc_events),	\
		.scan_type = {					\
				.sign = 'u',			\
				.realbits = res,		\
				.storagebits = 32,		\
				.shift = 0,			\
		},						\
	}

static const struct iio_chan_spec bmic_adc_channels[] = {
	BMIC_ADC_VOLTAGE_CHANNEL(0, BMIC_ADC_PORT1, BMIC_ADC_RESOLUTION),
	BMIC_ADC_VOLTAGE_CHANNEL(1, BMIC_ADC_PORT2, BMIC_ADC_RESOLUTION),
	IIO_CHAN_SOFT_TIMESTAMP(2),
};

static const struct iio_info bmic_adc_info = {
	.read_raw = bmic_adc_read_raw,
	.read_event_config = bmic_adc_read_event_config,
	.write_event_config = bmic_adc_write_event_config,
	.read_event_value = bmic_adc_read_event_value,
	.write_event_value = bmic_adc_write_event_value,
	.driver_module = THIS_MODULE,
};

static int bmic_adc_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct iio_dev *iio;
	struct bmic_adc *adc;
	struct device_node *np = client->dev.of_node;
	int ver;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_WORD_DATA))
		return -ENODEV;

	ver = i2c_smbus_read_word_data(client, REG_VERSION);
	dev_info(&client->dev, "version: %d.%d\n",
		 BMIC_ADC_MAJOR_VERSION(ver),
		 BMIC_ADC_MINOR_VERSION(ver));

	iio = devm_iio_device_alloc(&client->dev, sizeof(*adc));
	if (!iio)
		return -ENOMEM;

	adc = iio_priv(iio);
	adc->i2c = client;

	adc->ref = devm_regulator_get(&client->dev, "vref");
	if (IS_ERR(adc->ref)) {
		dev_err(&client->dev, "can't get vref regulator\n");
		return PTR_ERR(adc->ref);
	}

	ret = regulator_enable(adc->ref);
	if (ret) {
		dev_err(&client->dev, "can't enable vref regulator\n");
		return ret;
	}

	adc->irq_gpio = of_get_named_gpio(np, "irq-gpio", 0);
	if (!gpio_is_valid(adc->irq_gpio)) {
		dev_err(&client->dev, "can't get irq-gpio\n");
		goto regulator_disable;
	}

	ret = devm_request_threaded_irq(&client->dev,
					gpio_to_irq(adc->irq_gpio),
					NULL,
					bmic_adc_event_handler,
					IRQF_TRIGGER_FALLING |
					IRQF_ONESHOT | IRQF_SHARED,
					"bmic_adc_irq", iio);
	if (ret) {
		dev_err(&client->dev, "can't get IRQ %d\n",
			gpio_to_irq(adc->irq_gpio));
		goto regulator_disable;
	}

	iio->dev.parent = &client->dev;
	iio->name = dev_name(&client->dev);
	iio->modes = INDIO_DIRECT_MODE;

	iio->info = &bmic_adc_info;
	iio->channels = bmic_adc_channels;

	iio->num_channels = ARRAY_SIZE(bmic_adc_channels);

	ret = iio_device_register(iio);
	if (ret) {
		dev_err(&client->dev, "can't register iio device\n");
		goto regulator_disable;
	}

	i2c_set_clientdata(client, iio);

	return 0;

regulator_disable:
	regulator_disable(adc->ref);

	return ret;
}

static int bmic_adc_remove(struct i2c_client *client)
{
	struct iio_dev *iio = i2c_get_clientdata(client);
	struct bmic_adc *adc = iio_priv(iio);

	iio_device_unregister(iio);
	regulator_disable(adc->ref);

	return 0;
}

static const struct i2c_device_id bmic_adc_id[] = {
	{ "bmic_adc", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, bmic_adc_id);

#ifdef CONFIG_OF
static const struct of_device_id bmic_adc_of_match[] = {
	{ .compatible = "at,bmic_adc" },
	{ }
};
MODULE_DEVICE_TABLE(of, bmic_adc_of_match);
#endif

static struct i2c_driver bmic_adc_driver = {
	.driver = {
		.name = "bmic_adc",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(bmic_adc_of_match),
	},
	.probe = bmic_adc_probe,
	.remove = bmic_adc_remove,
	.id_table = bmic_adc_id,
};
module_i2c_driver(bmic_adc_driver);

MODULE_AUTHOR("Atmark Techno, Inc.");
MODULE_DESCRIPTION("Atmark Techno BMIC ADC driver");
MODULE_LICENSE("GPL v2");
