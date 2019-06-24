#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/regulator/consumer.h>
#include "common.h"

static int imx6_vbus_sel_probe(struct platform_device *pdev)
{
	imx_anatop_3p0_vbus_sel(1);

	return 0;
}

static struct of_device_id imx6_vbus_sel_dt_ids[] = {
	{ .compatible = "imx6-vbus-sel" },
	{ }
};

static struct platform_driver imx6_vbus_sel_driver = {
	.driver = {
		.name = "imx6-vbus-sel",
		.of_match_table = imx6_vbus_sel_dt_ids,
	},
	.probe = imx6_vbus_sel_probe,
};

static int __init imx6_vbus_sel_init(void)
{
	return platform_driver_register(&imx6_vbus_sel_driver);
}
subsys_initcall_sync(imx6_vbus_sel_init);
