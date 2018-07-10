
/*
 * GPIO-drived DC Motor driver
 *
 * Copyright 2016 Hunan GreatWall Information Financial Equipment Co., Ltd.
 *
 * Licensed under the GPL-2.
 */

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#include "dcmotor.h"

#define GPIO_VALUE_HIGH 	1
#define GPIO_VALUE_LOW  	0
#define	DCMOTO_GPIO_NUM		2

struct motor_dev {
	struct dcmotor motor;
	int dcmoto_gpio_en1;
	int dcmoto_gpio_en2;
};
#define to_dcmoto_dev(motor)	container_of(motor, struct motor_dev, motor)

static int gpio_init_from_dts(struct device *dev,struct motor_dev *motordev)
{
	int ret;
	struct device_node *np = dev->of_node;
	
	motordev->dcmoto_gpio_en1 = of_get_named_gpio(np, "win1-gpio", 0);
	if (!gpio_is_valid(motordev->dcmoto_gpio_en1)) {
		dev_err(dev, "no win1-gpio pin available\n");
		goto err;
	}
	ret = gpio_request(motordev->dcmoto_gpio_en1, NULL);
	if(ret)
		return ret;

	motordev->dcmoto_gpio_en2 = of_get_named_gpio(np, "win2-gpio", 0); 
	if (!gpio_is_valid(motordev->dcmoto_gpio_en2)) {
		dev_err(dev, "no win2-gpio pin available\n");
		goto err;
	}
	ret = gpio_request(motordev->dcmoto_gpio_en2, NULL);
	if(ret)
		return ret;
err:
	  return 0;
}

static int gpio_dcmotor_start(struct dcmotor *motor)
{
	int ret=0;
	struct motor_dev *motordev = to_dcmoto_dev(motor);
	if (!motordev)
		return -EINVAL;
	gpio_direction_output(motordev->dcmoto_gpio_en1, GPIO_VALUE_LOW);
	gpio_direction_output(motordev->dcmoto_gpio_en2, GPIO_VALUE_LOW);
	gpio_direction_output((motordev->motor.config.dir ? motordev->dcmoto_gpio_en1 : motordev->dcmoto_gpio_en2), GPIO_VALUE_HIGH);
	motor->status |= DCMOTOR_RUNNING;
	printk(KERN_DEBUG "dcmotor start.\n");
	return ret;
}


static void gpio_dcmotor_stop(struct dcmotor *motor)
{
	struct motor_dev *motordev = to_dcmoto_dev(motor);
	if (!motordev)
		return;//return -EINVAL;
	if ((motor->status & DCMOTOR_RUNNING) == 0)
	{
		return;
	}
	gpio_direction_output(motordev->dcmoto_gpio_en1, GPIO_VALUE_HIGH);
	gpio_direction_output(motordev->dcmoto_gpio_en2, GPIO_VALUE_HIGH);
	mdelay(500);
	gpio_direction_output(motordev->dcmoto_gpio_en1, GPIO_VALUE_LOW);
	gpio_direction_output(motordev->dcmoto_gpio_en2, GPIO_VALUE_LOW);
	motor->status &= ~DCMOTOR_RUNNING;
	printk(KERN_DEBUG "dcmotor stop.\n");
	if (motor->callback)
	{
		motor->callback(motor, &(motor->callbackdata));
	}
	return;
}


static int gpio_dcmotor_config(struct dcmotor *motor, const struct dcmotor_config *config)
{
	int ret=0;
	struct motor_dev *motordev = to_dcmoto_dev(motor);
	if (!motordev || !config)
		return -EINVAL;
	
	motordev->motor.config.dir = config->dir; 
	printk(KERN_DEBUG "dcmotor config.dir = %d\n",motordev->motor.config.dir);
	return ret;
}

static struct dcmotor_ops dcmotor_gpio_ops = {
     .start = gpio_dcmotor_start,
     .stop = gpio_dcmotor_stop,
     .config = gpio_dcmotor_config,
     .owner = THIS_MODULE,
};

static int gpio_dcmotor_probe(struct platform_device *pdev)
{
	struct motor_dev *motordev;
	int ret;
	//struct device_node *np = pdev->dev.of_node;


	motordev = devm_kzalloc(&pdev->dev, sizeof(*motordev), GFP_KERNEL);
	if (motordev == NULL)
		return -ENOMEM;
	motordev->motor.dev = &pdev->dev;
	ret =  gpio_init_from_dts(&pdev->dev, motordev);
	if (ret)
	{
		dev_err(&pdev->dev, "Failed to init gpio from dts: %d\n", ret);
		return ret;
	}

	motordev->motor.ops = &dcmotor_gpio_ops;
	ret = dcmotor_add(&motordev->motor);
	if (ret < 0)
		return ret;
	platform_set_drvdata(pdev, motordev);
	return 0;
}

static int gpio_dcmotor_remove(struct platform_device *pdev)
{
	struct motor_dev *motordev = platform_get_drvdata(pdev);
	if (motordev == NULL)
		return -ENODEV;

	return dcmotor_remove(&motordev->motor);
}

static const struct of_device_id dcmotor_match[] = {
	{ .compatible = "gwi,gpio-dcmotor", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, dcmotor_match);

static struct platform_driver gpio_dcmotor_driver = {
	.probe          = gpio_dcmotor_probe,
	.remove         = gpio_dcmotor_remove,
	.driver         = {
		.name   = "gpio-dcmotor",
		.of_match_table = dcmotor_match,
	},
};

module_platform_driver(gpio_dcmotor_driver);

MODULE_AUTHOR("Zhang Xudong <zhangxudong@gwi.com.cn>");
MODULE_DESCRIPTION("GPIO-controlled DC motor driver");
MODULE_LICENSE("GPL v2");
