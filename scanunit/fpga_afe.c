/*
 * GWI FPGA-controlled Image Analog Front End (AFE) driver
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

#include "imagedigitiser.h"


struct afe_dev {
	struct imagedigitiser afe; 
	void __iomem	*ctrl_base;
	void __iomem	*mmio_base;
	u32 mask;
};

#define to_afe_dev(afe)	container_of(afe, struct afe_dev, afe)


static int fpga_afe_enable(struct imagedigitiser *afe)
{
	struct afe_dev *afedev = to_afe_dev(afe);
	int rs;
	u32 mask;

	if (!afe)
		return -EINVAL;
	mask = afedev->mask;
	rs = fpga_update_lbits(afedev->ctrl_base + FPGA_REG_IMGADC_CONTROL, mask, mask);
	return rs;
}


static void fpga_afe_disable(struct imagedigitiser *afe)
{
	struct afe_dev *afedev = to_afe_dev(afe);
	u32 mask;

	if (!afe)
		return;
	mask = afedev->mask;
	fpga_update_lbits(afedev->ctrl_base + FPGA_REG_IMGADC_CONTROL, mask, 0);
}


static int fpga_afe_get_config(struct imagedigitiser *afe, struct scanunit_config *config)
{
	struct afe_dev *afedev = to_afe_dev(afe);
	struct scan_reg_config *regconfig;
	int i;

	if (!afe || !config)
		return -EINVAL;
	if (config->regcount <= 0 || !config->regconfig) 
		return -EINVAL;

	regconfig = config->regconfig;
	for (i=0; i<config->regcount; i++) {
		u32 value, mask = regconfig[i].mask;
		fpga_readl(&value, afedev->mmio_base + regconfig[i].address);
		if (mask != 0)
			value &= mask;
		regconfig[i].value = value;
	}
	return 0;
}


static int fpga_afe_set_config(struct imagedigitiser *afe, const struct scanunit_config *config)
{
	struct afe_dev *afedev = to_afe_dev(afe);
	struct scan_reg_config *regconfig;
	int i;

	if (!afe || !config)
		return -EINVAL;
	if (config->regcount <= 0 || !config->regconfig) 
		return -EINVAL;

	regconfig = config->regconfig;
	for (i=0; i<config->regcount; i++) {
		fpga_update_lbits(afedev->mmio_base + regconfig[i].address, regconfig[i].mask, regconfig[i].value);
	}
	return 0;
}


static int fpga_afe_probe(struct platform_device *pdev)
{
	struct afe_dev *afedev;
	struct device_node *np = pdev->dev.of_node;
	struct device_node *np_parent;
	int ret = 0;
	u32 reg[3];
	u32 fpga_cs;

	afedev = devm_kzalloc(&pdev->dev, sizeof(*afedev), GFP_KERNEL);
	if (afedev == NULL)
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
	afedev->ctrl_base  = fpga_io_get(reg[0]);
	if (IS_ERR(afedev->ctrl_base ))
		return PTR_ERR(afedev->ctrl_base );

	afedev->ctrl_base  += reg[1];

	ret = of_property_read_u32_array(np, "reg", reg, 3);
	if (IS_ERR_VALUE(ret)) {
		dev_err(&pdev->dev, "invalid FPGA AFE registers memory definition\n");
		return ret;
	}
	fpga_cs = reg[0];
	afedev->mmio_base = fpga_io_get(fpga_cs);
	if (IS_ERR(afedev->mmio_base))
		return PTR_ERR(afedev->mmio_base);

	afedev->mmio_base += reg[1];
	dev_dbg(&pdev->dev, "mmio_base = %08X.\n", (u32)afedev->mmio_base);

	ret = of_property_read_u32(np, "mask", &afedev->mask); 
	if (ret){
		dev_err(&pdev->dev, "Failed to parse chip mask of FPGA AFE\n");
		return ret;
	}

	afedev->afe.dev = &pdev->dev;
	afedev->afe.enable = fpga_afe_enable;
	afedev->afe.disable = fpga_afe_disable;
	afedev->afe.get_config = fpga_afe_get_config;
	afedev->afe.set_config = fpga_afe_set_config;
	ret = imagedigitiser_add(&afedev->afe);
	if (ret < 0)
		return ret;

	platform_set_drvdata(pdev, afedev);
	return 0;
}

static int fpga_afe_remove(struct platform_device *pdev)
{
	struct afe_dev *afedev = platform_get_drvdata(pdev);
	imagedigitiser_remove(&afedev->afe);
	return 0;
}


static const struct of_device_id fpga_afe_match[] = {
	{ .compatible = "gwi,fpga-afe", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, fpga_afe_match);

static struct platform_driver fpga_afe_driver = {
	.driver		= {
		.name	= "fpga-afe",
		.of_match_table = fpga_afe_match,
	},
	.probe		= fpga_afe_probe,
	.remove		= fpga_afe_remove,
};

module_platform_driver(fpga_afe_driver);

MODULE_AUTHOR("Zhang Xudong <zhangxudong@gwi.com.cn>");
MODULE_DESCRIPTION("GWI FPGA-controlled Image Analog Front End (AFE) driver");
MODULE_LICENSE("GPL v2");
