/*
 * GWI FPGA-controlled PWM driver
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
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/pwm.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include "../fpga.h"
#include "../fpga_io.h"


struct fpga_pwm_channel_info {
	u32 reg_period;
	u32 reg_pulsewidth;
	u32 cell_mask;
};

struct fpga_pwm_chip {
	struct pwm_chip	chip;
	void __iomem *mmio_base;
	u32 ctrl_reg;
	u32 clk_period;
	struct fpga_pwm_channel_info *fpga_pwm_chans;
};

#define to_fpga_chip(chip)	container_of(chip, struct fpga_pwm_chip, chip)

static int fpga_pwm_config(struct pwm_chip *chip,
		struct pwm_device *pwm, int duty_ns, int period_ns) 
{
	struct fpga_pwm_chip *fpgapwm = to_fpga_chip(chip);
	u32 value;
	int rs;

	//pr_debug("fpga_pwm_config duty_ns=%d period_ns=%d clk_period=%d\n", duty_ns, period_ns, fpgapwm->clk_period);
	value = (duty_ns/fpgapwm->clk_period) & 0xffff;
	//pr_debug("fpga_pwm_config—— reg_pulsewidth:reg=%x addr=%x  value=%x\n", (u32)fpgapwm->fpga_pwm_chans[pwm->hwpwm].reg_pulsewidth, (u32)(fpgapwm->mmio_base + fpgapwm->fpga_pwm_chans[pwm->hwpwm].reg_pulsewidth), value);
	rs = fpga_writel(value, fpgapwm->mmio_base + fpgapwm->fpga_pwm_chans[pwm->hwpwm].reg_pulsewidth); 
	if (rs)
		return rs;


	value = (period_ns/fpgapwm->clk_period) & 0xffff;
	//pr_debug("fpga_pwm_config—— reg_period:reg=%x addr=%x  value=%x\n", (u32)fpgapwm->fpga_pwm_chans[pwm->hwpwm].reg_period, (u32)(fpgapwm->mmio_base + fpgapwm->fpga_pwm_chans[pwm->hwpwm].reg_period), value);
	rs = fpga_writel(value, fpgapwm->mmio_base + fpgapwm->fpga_pwm_chans[pwm->hwpwm].reg_period);
     
	return rs;
}

static int fpga_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct fpga_pwm_chip *fpgapwm = to_fpga_chip(chip);
	int rs;
	u32 cellmask;

	cellmask = fpgapwm->fpga_pwm_chans[pwm->hwpwm].cell_mask;
	//pr_debug("fpga_pwm_enable:reg=%x addr=%x  mask=%x\n", (u32)fpgapwm->ctrl_reg, (u32)(fpgapwm->mmio_base + fpgapwm->ctrl_reg), cellmask);
	rs = fpga_update_lbits(fpgapwm->mmio_base + fpgapwm->ctrl_reg, cellmask, cellmask);
	return rs;
}

static void fpga_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct fpga_pwm_chip *fpgapwm = to_fpga_chip(chip);
	int rs;
	u32 cellmask;

	//pr_debug("fpga_pwm_disable:reg=%x addr=%x  mask=%x\n", (u32)fpgapwm->ctrl_reg, (u32)(fpgapwm->mmio_base + fpgapwm->ctrl_reg), cellmask);
	cellmask = fpgapwm->fpga_pwm_chans[pwm->hwpwm].cell_mask;
	rs = fpga_update_lbits(fpgapwm->mmio_base + fpgapwm->ctrl_reg, cellmask, 0);
}

static struct pwm_ops fpga_pwm_ops = {
	.enable = fpga_pwm_enable,
	.disable = fpga_pwm_disable,
	.config = fpga_pwm_config,
	.owner = THIS_MODULE,
};


static int fpga_pwm_probe(struct platform_device *pdev)
{
	struct fpga_pwm_chip *fpgapwm;
	struct device_node *np = pdev->dev.of_node;
	struct resource *r;
	int i, ret = 0;
	u32 fpga_cs, pwm_count, fpga_pwm_chansize = 0;
	struct device_node *child;

	fpgapwm = devm_kzalloc(&pdev->dev, sizeof(*fpgapwm), GFP_KERNEL);
	if (fpgapwm == NULL)
		return -ENOMEM;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ret = of_property_read_u32(np, "fpga-cs", &fpga_cs);
	dev_dbg(&pdev->dev, "fpga_cs of FPGA PWM is %d\n", fpga_cs);
	if (ret){
		dev_err(&pdev->dev, "Failed to parse chip number of FPGA PWM\n");
		return ret;
	}
	fpgapwm->mmio_base = fpga_io_get(fpga_cs);
	if (IS_ERR(fpgapwm->mmio_base))
		return PTR_ERR(fpgapwm->mmio_base);

	ret = of_property_read_u32(np, "clk_period", &fpgapwm->clk_period); 
	dev_dbg(&pdev->dev, "FPGA PWM clock period is %d\n", fpgapwm->clk_period);
	if (ret){
		dev_err(&pdev->dev, "Failed to parse FPGA PWM clock period\n");
		return ret;
	}

	ret = of_property_read_u32(np, "ctrl-reg", &fpgapwm->ctrl_reg); 
	dev_dbg(&pdev->dev, "FPGA PWM ctrl_reg is %d\n", fpgapwm->ctrl_reg);
	if (ret){
		dev_err(&pdev->dev, "Failed to parse FPGA PWM ctrl_reg\n");
		return ret;
	}

	pwm_count = 0;
	for_each_child_of_node(pdev->dev.of_node, child){
		pwm_count++;
	}
	fpga_pwm_chansize = pwm_count * sizeof(struct fpga_pwm_channel_info);//sizeof(*fpga_pwm_chans);
	fpgapwm->fpga_pwm_chans = devm_kzalloc(&pdev->dev, fpga_pwm_chansize, GFP_KERNEL);
	if (fpgapwm->fpga_pwm_chans == NULL)
		return -ENOMEM;

	i = 0;
	for_each_child_of_node(pdev->dev.of_node, child){
		ret = of_property_read_u32(child, "period", &fpgapwm->fpga_pwm_chans[i].reg_period); 
		if (ret){
			dev_err(&pdev->dev, "Failed to parse reg_period of pwm %d\n", i + 1);
			return ret;
		}
		pr_debug("fpga_pwm_chans[%d].reg_period = %x\n", i, fpgapwm->fpga_pwm_chans[i].reg_period);
		ret = of_property_read_u32(child, "pulsewidth", &fpgapwm->fpga_pwm_chans[i].reg_pulsewidth); 
		if (ret){
			dev_err(&pdev->dev, "Failed to parse reg_pulsewidth of pwm %d\n", i + 1);
			return ret;
		}
		pr_debug("fpga_pwm_chans[%d].reg_pulsewidth= %x\n", i, fpgapwm->fpga_pwm_chans[i].reg_pulsewidth);
		ret = of_property_read_u32(child, "mask", &fpgapwm->fpga_pwm_chans[i].cell_mask); 
		if (ret){
			dev_err(&pdev->dev, "Failed to parse cell_mask of pwm %d\n", i + 1);
			return ret;
		}
		pr_debug("fpga_pwm_chans[%d].cell_mask = %x\n", i, fpgapwm->fpga_pwm_chans[i].cell_mask);
		i++;
	}	

	fpgapwm->chip.ops = &fpga_pwm_ops;
	fpgapwm->chip.dev = &pdev->dev;
	fpgapwm->chip.base = -1;
	fpgapwm->chip.npwm = pwm_count;
	fpgapwm->chip.can_sleep = true;

	ret = pwmchip_add(&fpgapwm->chip);
	if (ret < 0)
		return ret;

	platform_set_drvdata(pdev, fpgapwm);
	return 0;
}

static int fpga_pwm_remove(struct platform_device *pdev)
{
	struct fpga_pwm_chip *fpgapwm = platform_get_drvdata(pdev);

	return pwmchip_remove(&fpgapwm->chip);
}


static const struct of_device_id fpga_pwm_match[] = {
	{ .compatible = "gwi,fpga-pwm", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, fpga_pwm_match);

static struct platform_driver fpga_pwm_driver = {
	.driver		= {
		.name	= "fpga-pwm",
		.of_match_table = fpga_pwm_match,
	},
	.probe		= fpga_pwm_probe,
	.remove		= fpga_pwm_remove,
};

module_platform_driver(fpga_pwm_driver);

MODULE_AUTHOR("Zhang Xudong <zhangxudong@gwi.com.cn>");
MODULE_DESCRIPTION("GWI FPGA-controlled PWM driver");
MODULE_LICENSE("GPL v2");
