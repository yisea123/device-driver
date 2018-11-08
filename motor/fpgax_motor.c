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
#include "../fpgax_io.h"
#include "../fpga.h"


static int irq;
static int irqx;
static void __iomem *reg_base;
static void __iomem *regx_base;


struct motor_irq_entry
{
	irq_handler_t handler;
	void *dev_id;
	u32 mask;
};

#define MAX_FPGA_MOTORS		16

struct motor_irq_entry motor_irq_table[MAX_FPGA_MOTORS];
struct motor_irq_entry motorx_irq_table[MAX_FPGA_MOTORS];


int motorint_handler_register(u32 mask, irq_handler_t handler, void * dev_id, u32 fpgabus)
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

        if (fpgabus == MOTION_EIM_INTERFACE) {
                printk("\nmotorint_handler_register");
                if (motor_irq_table[i].handler != NULL) 
        		return -EBUSY;
                printk(" MOTION_EIM_INTERFACE\n");
        	/* register motor interrupt handler */
        	motor_irq_table[i].handler = handler;
        	motor_irq_table[i].dev_id = dev_id;
        	motor_irq_table[i].mask = mask;
                printk("\n\nmotor_irq_table[i].handler = %x, dev_id=%x, i=%d, mask=%d\n", motor_irq_table[i].handler, motor_irq_table[i].dev_id, i, mask);
        }
        else if (fpgabus == MOTION_SPI_INTERFACE) {
                printk("\nmotorint_handler_register");
                if (motorx_irq_table[i].handler != NULL) 
        		return -EBUSY;
                printk(" MOTION_SPI_INTERFACE\n");
        	/* register motor interrupt handler */
        	motorx_irq_table[i].handler = handler;
        	motorx_irq_table[i].dev_id = dev_id;
        	motorx_irq_table[i].mask = mask;
                printk("\n\nmotorx_irq_table[i].handler = %x, dev_id=%x, i=%d, mask=%d\n", motorx_irq_table[i].handler, motorx_irq_table[i].dev_id, i, mask);
        }

	return 0;
}
EXPORT_SYMBOL_GPL(motorint_handler_register);


int motorint_handler_unregister(irq_handler_t handler, u32 fpgabus)
{
	int i;

	if (!handler)
		return -EINVAL;



        if (fpgabus == MOTION_EIM_INTERFACE) {

        	for (i=0; i<MAX_FPGA_MOTORS; i++)
        	{
        		if (motor_irq_table[i].handler == handler)
        		    break;
        	}
        	if (i >= MAX_FPGA_MOTORS)
        		return -EINVAL;
        	/* unregister the motor interrupt handler */
        	memset(&motor_irq_table[i], 0, sizeof(motor_irq_table[i]));
        }
        else if (fpgabus == MOTION_SPI_INTERFACE) {

        	for (i=0; i<MAX_FPGA_MOTORS; i++)
        	{
        		if (motorx_irq_table[i].handler == handler)
        		    break;
        	}
        	if (i >= MAX_FPGA_MOTORS)
        		return -EINVAL;
        	/* unregister the motor interrupt handler */
        	memset(&motorx_irq_table[i], 0, sizeof(motorx_irq_table[i]));
        }
	return 0;
}
EXPORT_SYMBOL_GPL(motorint_handler_unregister);


static irqreturn_t fpga_motor_isr(int irq, void *dev_id)
{
	u32 int_status, mask;
	irqreturn_t rs;
	int i;
	//printk("fpga_motor_isr\n");
	// get FPGA motor interrupt status
	fpga_readl(&int_status, reg_base + FPGA_REG_MOTOR_INT_STATUS);
        
        //printk("reg_base=%x, int_status=%d\n", (u32)(reg_base + FPGA_REG_MOTOR_INT_STATUS), int_status);
        
	if (!int_status)
		return IRQ_NONE;
        //printk("int_status=%d, reg=%x\n", int_status, (u32)(reg_base + FPGA_REG_MOTOR_INT_STATUS));
	// clear FPGA motor interrupt flags
	fpga_writel(int_status, reg_base + FPGA_REG_MOTOR_INT_CLEAR);

	mask = 1;
	// call motor interrupt handlers
	for (i=0; i<MAX_FPGA_MOTORS; i++)
	{
                //printk("motor_irq_table[i].handler=%x\n",motor_irq_table[i].handler);
		if ((int_status & mask) && motor_irq_table[i].handler)
                {
                        //printk("good\n");
			rs = motor_irq_table[i].handler(mask, motor_irq_table[i].dev_id);
                }
		int_status &= ~mask;
		if (int_status == 0)
			break;
		mask <<= 1;
	}

	return IRQ_HANDLED;
        
}

static irqreturn_t fpgax_motor_isr(int irq, void *dev_id)
{
	u32 int_status, mask;
	irqreturn_t rs;
	int i;
        //printk("fpga_motor_isr\n");
	// get FPGA motor interrupt status
	fpgax_readl(&int_status, regx_base + FPGA_REG_MOTOR_INT_STATUS);

        printk("regx_base=%x, int_status=%d\n", (u32)(regx_base + FPGA_REG_MOTOR_INT_STATUS), int_status);

	if (!int_status)
		return IRQ_NONE;

	// clear FPGA motor interrupt flags
	fpgax_writel(int_status, regx_base + FPGA_REG_MOTOR_INT_CLEAR);

	mask = 1;
	// call motor interrupt handlers
	for (i=0; i<MAX_FPGA_MOTORS; i++)
	{
                printk("motorx_irq_table[i].handler=%x\n",motorx_irq_table[i].handler);
		if ((int_status & mask) && motorx_irq_table[i].handler)
                {
                        printk("good\n");
			rs = motorx_irq_table[i].handler(mask, motorx_irq_table[i].dev_id);
                }
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
	u32 reg[3], regSPI[3];
	int irq, irqx, fpga_cs, fpgax_cs;

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

        /******************************parse SPI register**************************************************************/
	ret = of_property_read_u32_array(np, "regspi", regSPI, 3);
	if (IS_ERR_VALUE(ret)) {
		dev_err(&pdev->dev, "invalid FPGAX DC-motor registers memory definition\n");
		return ret;
	}
	fpgax_cs = regSPI[0];
	regx_base = fpgax_io_get(fpgax_cs);
	if (IS_ERR(regx_base)) {
		ret = PTR_ERR(regx_base);
		dev_err(&pdev->dev, "Failed to remap FPGAX motor interrupt registers, err = %d\n", ret);
		return ret;
	}
	regx_base += regSPI[1];


	ret = of_property_read_u32(np, "fpga-irq", &irq);
	dev_dbg(&pdev->dev, "irq of FPGA motor is %d\n", irq);
	if (IS_ERR_VALUE(ret)) {
		dev_err(&pdev->dev, "Failed to parse FPGA interrupt number of motors\n");
		return ret;
	}

	ret = of_property_read_u32(np, "fpgax-irq", &irqx);
	dev_dbg(&pdev->dev, "irq of FPGAX motor is %d\n", irqx);
	if (IS_ERR_VALUE(ret)) {
		dev_err(&pdev->dev, "Failed to parse FPGAX interrupt number of motors\n");
		return ret;
	}

	ret = fpga_request_irq(irq, fpga_motor_isr, NULL);
	if (IS_ERR_VALUE(ret)) {
		dev_err(&pdev->dev, "Failed to request FPGA irq of motors\n");
		return ret;
	}

	ret = fpgax_request_irq(irqx, fpgax_motor_isr, NULL);
	if (IS_ERR_VALUE(ret)) {
		dev_err(&pdev->dev, "Failed to request FPGAX irq of motors\n");
		return ret;
	}
	return 0;
}


static int fpga_motor_remove(struct platform_device *pdev)
{
	fpga_free_irq(irq);
        fpgax_free_irq(irqx);
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
