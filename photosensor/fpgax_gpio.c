/*
 * GWI FPGA-controlled GPIO driver
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
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include "../fpga.h"
#include "../fpga_io.h"
#include "../fpgax_io.h"


struct fpga_gpio_chip {
	struct gpio_chip chip;
	void __iomem	*mmio_base;
};

#define to_fpga_chip(chip)	container_of(chip, struct fpga_gpio_chip, chip)

static int fpga_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct fpga_gpio_chip *fpgachip = to_fpga_chip(chip);
	u32 value;
	int rs;
	rs = fpgax_readl(&value, fpgachip->mmio_base + FPGA_REG_SENSOR_SADC_IN); 
	if (IS_ERR_VALUE(rs))
		return rs;

	return (bool)(value & BIT(offset));
}

static int fpga_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	return 0;
}

static int fpga_gpio_direction_output(struct gpio_chip *chip, unsigned offset, int value)
{
	return -EPERM;
}


static int fpga_gpio_probe(struct platform_device *pdev)
{
	struct fpga_gpio_chip *fpgagpio;
	struct device_node *np = pdev->dev.of_node;
	int ret = 0,gpio_num;
	u32 fpga_cs, reg[3];

	fpgagpio = devm_kzalloc(&pdev->dev, sizeof(*fpgagpio), GFP_KERNEL);
	if (fpgagpio == NULL)
		return -ENOMEM;

	ret = of_property_read_u32_array(np, "reg", reg, 3);
	if (IS_ERR_VALUE(ret)) {
		dev_err(&pdev->dev, "Failed to parse chip number of FPGA GPIO\n");
		return ret;
	}
	fpga_cs = reg[0];
	fpgagpio->mmio_base = fpgax_io_get(fpga_cs);
	if (IS_ERR(fpgagpio->mmio_base))
		return PTR_ERR(fpgagpio->mmio_base);

	gpio_num = 0;
	ret = of_property_read_u32(np, "gpio-num", &gpio_num);

	fpgagpio->chip.ngpio = gpio_num;
	fpgagpio->chip.base = -1;
	fpgagpio->chip.get = fpga_gpio_get;
	fpgagpio->chip.direction_input = fpga_gpio_direction_input;
	fpgagpio->chip.direction_output = fpga_gpio_direction_output;
	fpgagpio->chip.dev = &pdev->dev;
	fpgagpio->chip.owner = THIS_MODULE; 
	ret = gpiochip_add(&fpgagpio->chip);
	if (ret < 0)
		return ret;

	platform_set_drvdata(pdev, fpgagpio);
	return 0;
}

static int fpga_gpio_remove(struct platform_device *pdev)
{
	struct fpga_gpio_chip *fpgagpio = platform_get_drvdata(pdev);

	gpiochip_remove(&fpgagpio->chip);
	return 0;
}


static const struct of_device_id fpga_gpio_match[] = {
	{ .compatible = "gwi,fpga-gpio", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, fpga_gpio_match);

static struct platform_driver fpga_gpio_driver = {
	.driver		= {
		.name	= "fpga-gpio",
		.of_match_table = fpga_gpio_match,
	},
	.probe		= fpga_gpio_probe,
	.remove		= fpga_gpio_remove,
};

module_platform_driver(fpga_gpio_driver);

MODULE_AUTHOR("Zhang Xudong <zhangxudong@gwi.com.cn>");
MODULE_DESCRIPTION("GWI FPGA-controlled GPIO driver");
MODULE_LICENSE("GPL v2");
