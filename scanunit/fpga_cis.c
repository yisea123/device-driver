/*
 * GWI FPGA-controlled CIS (Contact Image Sensor) driver
 *
 * Copyright 2016 Hunan GreatWall Information Financial Equipment Co., Ltd.
 *
 * Licensed under the GPL-2.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_device.h>

#include "../fpga.h"
#include "../fpga_io.h"


#include "imagesensor.h"


struct isensor_dev {
	struct imagesensor sensor; 
	void __iomem	*ctrl_base;
	void __iomem	*mmio_base;
	u32 mask;
};

#define to_isensor_dev(sensor)	container_of(sensor, struct isensor_dev, sensor)


static int fpga_cis_enable(struct imagesensor *sensor)
{
	struct isensor_dev *sensordev = to_isensor_dev(sensor);
	int rs;
	u32 mask;

	if (!sensordev)
		return -EINVAL;
	mask = FPGA_REG_CIS_SCAN_ENABLE; 
	rs = fpga_update_lbits(sensordev->ctrl_base + FPGA_REG_CIS_CONTROL, mask, mask);
	return rs;
}


static void fpga_cis_disable(struct imagesensor *sensor)
{
	struct isensor_dev *sensordev = to_isensor_dev(sensor);
	u32 mask;

	if (!sensordev)
		return;
	mask = FPGA_REG_CIS_SCAN_ENABLE;
	fpga_update_lbits(sensordev->ctrl_base + FPGA_REG_CIS_CONTROL, mask, 0);
}


int fpga_cis_get_config(struct imagesensor *sensor, struct scanunit_config *config)
{
	struct isensor_dev *sensordev = to_isensor_dev(sensor);
	struct scan_reg_config *regconfig;
	int i;

	if (!sensor || !config)
		return -EINVAL;
	if (config->regcount <= 0 || !config->regconfig) 
		return -EINVAL;

	regconfig = config->regconfig;
	for (i=0; i<config->regcount; i++) {
		u32 value, mask = regconfig[i].mask;
		fpga_readl(&value, sensordev->mmio_base + regconfig[i].address);
		if (mask != 0)
			value &= mask;
		regconfig[i].value = value;
	}
	return 0;
}


int fpga_cis_set_config(struct imagesensor *sensor, const struct scanunit_config *config)
{
	struct isensor_dev *sensordev = to_isensor_dev(sensor);
	struct scan_reg_config *regconfig;
	int i;

	if (!sensor || !config)
		return -EINVAL;
	if (config->regcount <= 0 || !config->regconfig) 
		return -EINVAL;

	regconfig = config->regconfig;
	for (i=0; i<config->regcount; i++) {
		u32 value, mask = regconfig[i].mask;
		if (mask != 0) {	// bit operation
			fpga_readl(&value, sensordev->mmio_base + regconfig[i].address);
			value &= ~mask;
			value |= regconfig[i].value & mask;
		}
		else
			value = regconfig[i].value;
		fpga_writel(value, sensordev->mmio_base + regconfig[i].address);
	}
	return 0;
}


static int fpga_cis_probe(struct platform_device *pdev)
{
	struct isensor_dev *sensordev;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *np_parent;
	int ret = 0;
	u32 reg[3];
	u32 fpga_cs;

	sensordev = devm_kzalloc(&pdev->dev, sizeof(*sensordev), GFP_KERNEL);
	if (sensordev == NULL)
		return -ENOMEM;

	np_parent = of_get_parent(np);
	if (!np_parent) {
		pr_err("no parent node found in devicetree");
		return -ENODEV; 
	}

	ret = of_property_read_u32_array(np_parent, "ctrl-reg", reg, 3);
	if (IS_ERR_VALUE(ret)) {
		dev_err(&pdev->dev, "invalid FPGA CIS CTRL registers memory definition\n");
		return ret;
	}
	sensordev->ctrl_base = fpga_io_get(reg[0]);
	if (IS_ERR(sensordev->ctrl_base))
		return PTR_ERR(sensordev->ctrl_base);

	sensordev->ctrl_base += reg[1];


	ret = of_property_read_u32_array(np, "reg", reg, 3);
	if (IS_ERR_VALUE(ret)) {
		dev_err(&pdev->dev, "invalid FPGA CIS registers memory definition\n");
		return ret;
	}

	fpga_cs = reg[0];
	sensordev->mmio_base = fpga_io_get(fpga_cs);
	if (IS_ERR(sensordev->mmio_base))
		return PTR_ERR(sensordev->mmio_base);

	sensordev->mmio_base += reg[1];
	dev_dbg(&pdev->dev, "mmio_base = %08X.\n", (u32)sensordev->mmio_base);

	sensordev->sensor.dev = &pdev->dev; 
	sensordev->sensor.enable = fpga_cis_enable; 
	sensordev->sensor.disable = fpga_cis_disable;
	sensordev->sensor.get_config = fpga_cis_get_config;
	sensordev->sensor.set_config = fpga_cis_set_config;
	ret = imagesensor_add(&sensordev->sensor);
	if (ret < 0)
		return ret;

	platform_set_drvdata(pdev, sensordev);
	return 0;
}

static int fpga_cis_remove(struct platform_device *pdev)
{
	struct isensor_dev *sensordev = platform_get_drvdata(pdev);
	imagesensor_remove(&sensordev->sensor);
	return 0;
}


static const struct of_device_id fpga_cis_match[] = {
	{ .compatible = "gwi,fpga-cis", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, fpga_cis_match);

static struct platform_driver fpga_cis_driver = {
	.driver		= {
		.name	= "fpga-cis",
		.of_match_table = fpga_cis_match,
	},
	.probe		= fpga_cis_probe,
	.remove		= fpga_cis_remove,
};

module_platform_driver(fpga_cis_driver);

MODULE_AUTHOR("Zhang Xudong <zhangxudong@gwi.com.cn>");
MODULE_DESCRIPTION("GWI FPGA-controlled CIS (Contact Image Sensor) driver");
MODULE_LICENSE("GPL v2");
