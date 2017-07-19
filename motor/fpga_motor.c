/*
 * FPGA-controlled motor interrupts handle
 *
 * Copyright 2017 Hunan GreatWall Information Financial Equipment Co., Ltd.
 *
 * Licensed under the GPL-2.
 */
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>


#include "../fpga_io.h"
#include "../fpga.h"


static int irq;
static void __iomem *reg_base;



struct motor_irq_entry
{
	irq_handler_t handler;
	void *dev_id;
	u32 mask;
};

#define MAX_FPGA_MOTORS		16

struct motor_irq_entry motor_irq_table[MAX_FPGA_MOTORS];


int motorint_handler_register(u32 mask, irq_handler_t handler, void * dev_id)
{
	u32 bitmask;
	int i;

	if (!mask || !handler || !dev_id)
		return -EINVAL;

	bitmask = 1;
	for (i=0; i<MAX_FPGA_MOTORS; i++)
	{
		if (bitmask == mask)
		    break;
		bitmask <<= 1;
	}
	if (i >= MAX_FPGA_MOTORS)
		return -EINVAL;
	if (motor_irq_table[i].handler != NULL)
		return -EBUSY;

	/* register motor interrupt handler */
	motor_irq_table[i].handler = handler;
	motor_irq_table[i].dev_id = dev_id;
	motor_irq_table[i].mask = mask;

	return 0;
}
EXPORT_SYMBOL_GPL(motorint_handler_register);


int motorint_handler_unregister(irq_handler_t handler)
{
	int i;

	if (!handler)
		return -EINVAL;

	for (i=0; i<MAX_FPGA_MOTORS; i++)
	{
		if (motor_irq_table[i].handler == handler)
		    break;
	}
	if (i >= MAX_FPGA_MOTORS)
		return -EINVAL;

	/* unregister the motor interrupt handler */
	memset(&motor_irq_table[i], 0, sizeof(motor_irq_table[i]));

	return 0;
}
EXPORT_SYMBOL_GPL(motorint_handler_unregister);


static irqreturn_t fpga_motor_isr(int irq, void *dev_id)
{
	u32 int_status, mask;
	irqreturn_t rs;
	int i;
	
	// get FPGA motor interrupt status
	fpga_readl(&int_status, reg_base + FPGA_REG_MOTOR_INT_STATUS);
	if (!int_status)
		return IRQ_NONE;

	// clear FPGA motor interrupt flags
	fpga_writel(int_status, reg_base + FPGA_REG_MOTOR_INT_CLEAR);

	mask = 1;
	// call motor interrupt handlers
	for (i=0; i<MAX_FPGA_MOTORS; i++)
	{
		if ((int_status & mask) && motor_irq_table[i].handler)
			rs = motor_irq_table[i].handler(mask, motor_irq_table[i].dev_id);
		int_status &= ~mask;
		if (int_status == 0)
			break;
		mask <<= 1;
	}

	return IRQ_HANDLED;
}




static int fpga_motor_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int ret;
	u32 reg[3];
	int irq, fpga_cs;

	ret = of_property_read_u32_array(np, "reg", reg, 3);
	if (IS_ERR_VALUE(ret)) {
		dev_err(&pdev->dev, "invalid FPGA DC-motor registers memory definition\n");
		return ret;
	}
	fpga_cs = reg[0];
	reg_base = fpga_io_get(fpga_cs);
	if (IS_ERR(reg_base)) {
		ret = PTR_ERR(reg_base);
		dev_err(&pdev->dev, "Failed to remap FPGA motor interrupt registers, err = %d\n", ret);
		return ret;
	}
	reg_base += reg[1];

	ret = of_property_read_u32(np, "fpga-irq", &irq);
	dev_dbg(&pdev->dev, "irq of FPGA motor is %d\n", irq);
	if (IS_ERR_VALUE(ret)) {
		dev_err(&pdev->dev, "Failed to parse FPGA interrupt number of motors\n");
		return ret;
	}
	ret = fpga_request_irq(irq, fpga_motor_isr, NULL);
	if (IS_ERR_VALUE(ret)) {
		dev_err(&pdev->dev, "Failed to request FPGA irq of motors\n");
		return ret;
	}
	return 0;
}


static int fpga_motor_remove(struct platform_device *pdev)
{
	fpga_free_irq(irq);
	return 0;
}



static const struct of_device_id fpga_motor_match[] = {
	{ .compatible = "gwi,fpga-motor", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, fpga_motor_match);

static struct platform_driver fpga_motor_driver = {
	.probe          = fpga_motor_probe,
	.remove         = fpga_motor_remove,
	.driver         = {
		.name   = "fpga-motor",
		.of_match_table = fpga_motor_match,
	},
};

module_platform_driver(fpga_motor_driver);

MODULE_AUTHOR("Zhang Xudong <zhangxudong@gwi.com.cn>");
MODULE_DESCRIPTION("FPGA-controlled motor driver");
MODULE_LICENSE("GPL v2");
