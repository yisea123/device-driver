
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
#include <linux/pwm.h>

#include "dcmotor.h"

#define DEFAULT_PERIOD				40000
#define DEFAULT_DUTY				0
#define pwm_set_duty(period, percent)		(period * percent / 100)	/* pwm占空比，百分比 */

struct motor_dev {
	struct dcmotor motor;
	struct pwm_device *dcmoto_pwm1;
	struct pwm_device *dcmoto_pwm2;
};

#define to_motor_dev(motor)	container_of(motor, struct motor_dev, motor)


static int pwm_dcmotor_start(struct dcmotor *motor)
{
	int ret = 0;
	struct motor_dev *motordev = to_motor_dev(motor);

	if (!motordev->motor.config.dir)
	{
		pwm_config(motordev->dcmoto_pwm1, DEFAULT_PERIOD, DEFAULT_PERIOD);
		pwm_config(motordev->dcmoto_pwm2, pwm_set_duty(DEFAULT_PERIOD, DEFAULT_DUTY), DEFAULT_PERIOD);
	}
	else
	{
		pwm_config(motordev->dcmoto_pwm1, pwm_set_duty(DEFAULT_PERIOD, DEFAULT_DUTY), DEFAULT_PERIOD);
		pwm_config(motordev->dcmoto_pwm2, DEFAULT_PERIOD, DEFAULT_PERIOD);
	}
	ret =pwm_enable(motordev->dcmoto_pwm1); 
	if (ret < 0)
	{
		printk("pwm_enable erro\n");
	}
	ret = pwm_enable(motordev->dcmoto_pwm2);
	if (ret < 0)
	{
		printk("pwm_enable erro\n");
	}
	motor->status |= DCMOTOR_RUNNING;
	return 0;
}


static void pwm_dcmotor_stop(struct dcmotor *motor)
{
	struct motor_dev *motordev = to_motor_dev(motor);
	if ((motor->status & DCMOTOR_RUNNING) == 0)
	{
		return;
	}
	pwm_config(motordev->dcmoto_pwm1, pwm_set_duty(DEFAULT_PERIOD, 100), DEFAULT_PERIOD);
	pwm_config(motordev->dcmoto_pwm2, pwm_set_duty(DEFAULT_PERIOD, 100), DEFAULT_PERIOD);
	mdelay(50);
	pwm_disable(motordev->dcmoto_pwm1); 
	pwm_disable(motordev->dcmoto_pwm2);
	motor->status &= ~DCMOTOR_RUNNING;
	if (motor->callback)
	{
		motor->status &= ~DCMOTOR_RUNNING;
		motor->callback(motor, &(motor->callbackdata));
	}
}

static int pwm_dcmotor_config(struct dcmotor *motor, const struct dcmotor_config *config)
{
	int ret=0;
	struct motor_dev *motordev = to_motor_dev(motor);
	if (!motordev || !config)
		return -EINVAL;
	
	motordev->motor.config.dir = config->dir; 
//	printk("dcmotor config.dir = %d\n",motordev->motor.config.dir);
	return ret;
}

static struct dcmotor_ops dcmotor_pwm_ops = {
     .start = pwm_dcmotor_start,
     .stop = pwm_dcmotor_stop,
     .config = pwm_dcmotor_config,
     .owner = THIS_MODULE,
};

static int pwm_dcmotor_probe(struct platform_device *pdev)
{
	struct motor_dev *motordev;
//	struct device_node *np = pdev->dev.of_node;
	struct device_node *child;
	int ret;

	motordev = devm_kzalloc(&pdev->dev, sizeof(*motordev), GFP_KERNEL);
	if (motordev == NULL)
	{
		printk("devm_kzalloc fail\n");
		return -ENOMEM;
	}
	
	motordev->motor.dev = &pdev->dev;

	child = of_find_node_by_name(NULL,"dcmotor_pwm1");
	if (!child)
		return -ENODEV;
	motordev->dcmoto_pwm1 = of_pwm_get(child, NULL);
	if (IS_ERR(motordev->dcmoto_pwm1))
	{
		printk("of_pwm_get fail\n");
		return -ENODEV;
	}
	child = of_find_node_by_name(NULL,"dcmotor_pwm2");
	if (!child)
		return -ENODEV;
	motordev->dcmoto_pwm2 = of_pwm_get(child, NULL);
	if (IS_ERR(motordev->dcmoto_pwm2))
	{
		printk("of_pwm_get fail\n");
		return -ENODEV;
	}
	motordev->motor.ops = &dcmotor_pwm_ops;

	ret = dcmotor_add(&motordev->motor);
	if (IS_ERR_VALUE(ret)) {
		dev_err(&pdev->dev, "Failed to register a dcmotor\n");
		return ret;
	}
	platform_set_drvdata(pdev, motordev);
	return 0;
}

static int pwm_dcmotor_remove(struct platform_device *pdev)
{
	struct motor_dev *motordev = platform_get_drvdata(pdev);
	if (motordev == NULL)
		return -ENODEV;

	return dcmotor_remove(&motordev->motor);
}

static const struct of_device_id dcmotor_match[] = {
	{ .compatible = "gwi,pwm-dcmotor", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, dcmotor_match);

static struct platform_driver pwm_dcmotor_driver = {
	.probe          = pwm_dcmotor_probe,
	.remove         = pwm_dcmotor_remove,
	.driver         = {
		.name   = "pwm-dcmotor",
		.of_match_table = dcmotor_match,
	},
};

module_platform_driver(pwm_dcmotor_driver);

MODULE_AUTHOR("Zhang Xudong <zhangxudong@gwi.com.cn>");
MODULE_DESCRIPTION("GPIO-controlled DC motor driver");
MODULE_LICENSE("GPL v2");
