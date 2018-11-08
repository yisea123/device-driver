
/*
 * FPGA-controlled DC Motor driver
 *
 * Copyright 2016-2017 Hunan GreatWall Information Financial Equipment Co., Ltd.
 *
 * Licensed under the GPL-2.
 */
#include <linux/completion.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/delay.h>

#include "dcmotor.h"
#include "../fpga_io.h"
#include "../fpgax_io.h"
#include "../fpga.h"


extern int motorint_handler_register(u32 mask, irq_handler_t handler, void * dev_id, u32 fpgabus);
extern int motorint_handler_unregister(irq_handler_t handler, u32 fpgabus);


struct motor_dev {
	struct dcmotor motor;
	void __iomem *mmio_base;
        u32 cominterface;           // communicate interface
	u32 mask;
};

#define to_motor_dev(motor)	container_of(motor, struct motor_dev, motor)


static int fpga_dcmotor_start(struct dcmotor *motor)
{
	struct motor_dev *motordev = to_motor_dev(motor);
	int rs;
        u32 val=0;
	if (!motor)
		return -EINVAL;

        if (motordev->cominterface == MOTION_EIM_INTERFACE) {
            rs = fpga_update_lbits(motordev->mmio_base + FPGA_REG_DCMOTOR_CONTROL, FPGA_REG_MOTOR_RUN, 0);
            rs = fpga_update_lbits(motordev->mmio_base + FPGA_REG_DCMOTOR_CONTROL, FPGA_REG_MOTOR_RUN, FPGA_REG_MOTOR_RUN);           
        }
        else if (motordev->cominterface == MOTION_SPI_INTERFACE) {
            rs = fpgax_update_lbits(motordev->mmio_base + FPGA_REG_DCMOTOR_CONTROL, FPGA_REG_MOTOR_RUN, 0);
            rs = fpgax_update_lbits(motordev->mmio_base + FPGA_REG_DCMOTOR_CONTROL, FPGA_REG_MOTOR_RUN, FPGA_REG_MOTOR_RUN);
        }
        else{
            return -EINVAL;
        }


	motor->status |= DCMOTOR_RUNNING;
	return rs;
}


static void fpga_dcmotor_stop(struct dcmotor *motor)
{
	struct motor_dev *motordev = to_motor_dev(motor);

	if (!motor)
		return;

        if (motordev->cominterface == MOTION_EIM_INTERFACE) {
            fpga_update_lbits(motordev->mmio_base + FPGA_REG_DCMOTOR_CONTROL, FPGA_REG_MOTOR_RUN, 0);
        }
        else if (motordev->cominterface == MOTION_SPI_INTERFACE) {
            fpgax_update_lbits(motordev->mmio_base + FPGA_REG_DCMOTOR_CONTROL, FPGA_REG_MOTOR_RUN, 0);
        }
        else{
            return;
        }

	motor->status &= ~DCMOTOR_RUNNING;
}


static inline int _fpga_dcmotor_status(struct dcmotor *motor)
{
	struct motor_dev *motordev = to_motor_dev(motor);
	int rs, ret;
	u32 status;

	if (!motor)
		return -EINVAL;

        if (motordev->cominterface == MOTION_EIM_INTERFACE) {
            rs = fpga_readl(&status, motordev->mmio_base + FPGA_REG_DCMOTOR_STATUS);
        }
        else if (motordev->cominterface == MOTION_SPI_INTERFACE) {
            rs = fpgax_readl(&status, motordev->mmio_base + FPGA_REG_DCMOTOR_STATUS);
        }
        else{
            return -EINVAL;
        }

	if (rs)
		return 0;

	ret = 0;
	if (status & FPGA_REG_DCMOTOR_RUNNING)
		ret |= DCMOTOR_RUNNING;
	if (status & FPGA_REG_DCMOTOR_STOPPED_BY_SENSOR)
		ret |= DCMOTOR_STOPPED_BY_SENSOR;
	return ret;
}


static int fpga_dcmotor_status(struct dcmotor *motor)
{
	return _fpga_dcmotor_status(motor);
}


static int fpga_dcmotor_config(struct dcmotor *motor, const struct dcmotor_config *config)
{
	struct motor_dev *motordev = to_motor_dev(motor);
	int rs;
	u32 val;

	if (!motor || !config)
		return -EINVAL;

	val = (config->dir == MOTION_CLOCKWISE) ? 0 : FPGA_REG_MOTOR_DIRECTION;
        if (motordev->cominterface == MOTION_EIM_INTERFACE) {
            rs = fpga_update_lbits(motordev->mmio_base + FPGA_REG_DCMOTOR_CONTROL, FPGA_REG_MOTOR_DIRECTION, val);
        }
        else if (motordev->cominterface == MOTION_SPI_INTERFACE) {
            rs = fpgax_update_lbits(motordev->mmio_base + FPGA_REG_DCMOTOR_CONTROL, FPGA_REG_MOTOR_DIRECTION, val);
           // printk("motordev->mmio_base=%x, and val=%d\n", (u32)motordev->mmio_base, val);
        }
        else{
            return -EINVAL;
        }
	return rs;
}


static irqreturn_t fpga_dcmotor_isr(int irq, void *dev_id)
{
	struct motor_dev *motordev;
	struct dcmotor *motor;
	int status;

	if (!dev_id)
		return -EINVAL;

	motordev = (struct motor_dev *)dev_id;
	motor = &motordev->motor;
        //printk("\nfpga_dcmotor_isr\n");
        status = _fpga_dcmotor_status(motor); 
        if (motordev->cominterface == MOTION_EIM_INTERFACE) {
            if (status & DCMOTOR_STOPPED_BY_SENSOR)
    		fpga_update_lbits(motordev->mmio_base + FPGA_REG_DCMOTOR_CONTROL, FPGA_REG_MOTOR_RUN, 0);
        }
        else if (motordev->cominterface == MOTION_SPI_INTERFACE) {
            if (status & DCMOTOR_STOPPED_BY_SENSOR)
    		fpgax_update_lbits(motordev->mmio_base + FPGA_REG_DCMOTOR_CONTROL, FPGA_REG_MOTOR_RUN, 0);
        }
        else{
            return -EINVAL;
        }
        


	/* update dcmotor status */
	motor->status = status;

	if (motor->callback)
		motor->callback(motor, &motor->callbackdata);
	return IRQ_HANDLED; 
}


static struct dcmotor_ops fpga_dcmotor_ops = {
	.config = fpga_dcmotor_config,
	.status = fpga_dcmotor_status,
	.start = fpga_dcmotor_start,
	.stop = fpga_dcmotor_stop,
	.owner = THIS_MODULE,
};


static int fpga_dcmotor_probe(struct platform_device *pdev)
{
	struct motor_dev *motordev;
	struct device_node *np = pdev->dev.of_node;
	int ret;
	u32 reg[3];
	u32 fpga_cs;

	motordev = devm_kzalloc(&pdev->dev, sizeof(*motordev), GFP_KERNEL);
	if (motordev == NULL)
		return -ENOMEM;

	ret = of_property_match_string(np, "fpgabus", "EIM");
	if (ret >= 0){
		dev_dbg(&pdev->dev,"fpgabus:%x\n", motordev->cominterface);
                motordev->cominterface = MOTION_EIM_INTERFACE;


	} 
        else{
		ret = of_property_match_string(np, "fpgabus", "SPI");
		if (ret >= 0){
			dev_dbg(&pdev->dev, "fpgabus:%x\n", motordev->cominterface);
                        motordev->cominterface = MOTION_SPI_INTERFACE;
                } 
                else
                    return -EINVAL;
	}

	ret = of_property_read_u32_array(np, "reg", reg, 3);
	if (IS_ERR_VALUE(ret)) {
		dev_err(&pdev->dev, "invalid FPGA DC-motor registers memory definition\n");
		return ret;
	}
	fpga_cs = reg[0];

        if (motordev->cominterface == MOTION_EIM_INTERFACE) {
                motordev->mmio_base = fpga_io_get(fpga_cs); 
        }
        else if (motordev->cominterface == MOTION_SPI_INTERFACE) {
                motordev->mmio_base = fpgax_io_get(fpga_cs); 
        }
        else
            return -EINVAL;

	if (IS_ERR(motordev->mmio_base))
		return PTR_ERR(motordev->mmio_base);

	motordev->mmio_base += reg[1];
	dev_dbg(&pdev->dev, "mmio_base = %08X.\n", (u32)motordev->mmio_base);

	ret = of_property_read_u32(np, "mask", &motordev->mask); 
	if (ret) {
		dev_err(&pdev->dev, "Failed to parse interrupt mask of FPGA DC-motor\n");
		return ret;
	}

	ret = motorint_handler_register(motordev->mask,  fpga_dcmotor_isr, (void *)motordev, motordev->cominterface);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register FPGA DC-motor interrupt handler\n");
		return ret;
	}

	motordev->motor.dev = &pdev->dev;
	motordev->motor.ops = &fpga_dcmotor_ops;
	ret = dcmotor_add(&motordev->motor);
	if (ret < 0)
		return ret;

	platform_set_drvdata(pdev, motordev);
	return 0;
}


static int fpga_dcmotor_remove(struct platform_device *pdev)
{
	struct motor_dev *motordev = platform_get_drvdata(pdev);

	dcmotor_remove(&motordev->motor);
	motorint_handler_unregister(fpga_dcmotor_isr, motordev->cominterface);
	return 0;
}


static const struct of_device_id fpga_dcmotor_match[] = {
	{ .compatible = "gwi,fpga-dcmotor", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, fpga_dcmotor_match);

static struct platform_driver fpga_dcmotor_driver = {
	.probe          = fpga_dcmotor_probe,
	.remove         = fpga_dcmotor_remove,
	.driver         = {
		.name   = "dcmotor",
		.of_match_table = fpga_dcmotor_match,
	},
};

module_platform_driver(fpga_dcmotor_driver);

MODULE_AUTHOR("Zhang Xudong <zhangxudong@gwi.com.cn>");
MODULE_DESCRIPTION("FPGA-controlled DC motor driver");
MODULE_LICENSE("GPL v2");
