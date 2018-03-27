/*
 * PWM-controlled Stepper Motor driver
 *
 * Copyright 2016-2017 Hunan GreatWall Information Financial Equipment Co., Ltd.
 *
 * Licensed under the GPL-2.
 */
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/completion.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/pwm.h>
#include <../arch/arm/mach-imx/hardware.h>
#include "pwm-gwi.h"
#include "steppermotor.h"
#include "steptable.h"

#define PWM_STEPMOTOR_SPEED_CTRL	1	//增加加减速控制

#if 1	//voucher dispenser
#define STPMOTO_CONFIG_EN 	0
#define STPMOTO_CONFIG_DIR 	1
#define GPIO_VALUE_HIGH 	1
#define GPIO_VALUE_LOW  	0
#define STEPMOTO_GPIO_NUM	2
#define DRIVER_IC_DUTY		100000	//>470ns
#define DEFAULT_PERIOD		2000000
#endif


//extern int motorint_handler_register(u32 mask, irq_handler_t handler, void * dev_id);
//extern int motorint_handler_unregister(irq_handler_t handler);


#define MAXIMUM_STEPS		0xffff

#define SPEED_TO_COUNT(speed,stepping,clock)	(((1000000000L / (speed)) / (stepping)) / (clock))


static DEFINE_MUTEX(stepmotor_mutex);

//步进电机运行状态指示
typedef enum
{
	STEPMOTOR_ST_INIT = 0,		//初始化状态
	STEPMOTOR_ST_HOLD,		//Holding
	STEPMOTOR_ST_ACC,		//加速
	STEPMOTOR_ST_CONST,		//匀速
	STEPMOTOR_ST_DEC,		//减速
	STEPMOTOR_ST_POSIT		//停止安置
}t_stepmotor_running_st;

struct stepmotor_running_info
{
	int step_total;		//总步数
	int step_lose;		//已走步数
	int step_left;		//剩余步数

	int * p_speed_cur;		//当前速度指针
	int * p_acc_ramp_table;		//加速表指针
	int acc_ramp_table_size;	//加速表长度
	int * p_dec_ramp_table;		//减速表指针
	int dec_ramp_table_size;	//减速表长度
	
	t_stepmotor_running_st running_st;	//运行当前状态
	spinlock_t running_info_lock;		//running info 自旋锁
};

struct motor_dev {
	struct steppermotor motor;
	struct pwm_device *pwm;
	int gpio_dir;
	int gpio_en;
	int gpio_hold;
	int stepping;			// motor stepping multiple (should be one of 1/2/4/8/16/32)
	int clock_period;		// motor unit clock period (in nanoseconds)
	struct list_head speedtable_list;
	struct ramp_info rampinfo;	// ramp table of supported speeds and speed-shifts
	struct stepmotor_running_info running_info;	//电机运动时信息结构体
};

#define to_motor_dev(motor)	container_of(motor, struct motor_dev, motor)


static int pwm_stepmotor_start(struct steppermotor *motor)
{
	struct motor_dev *motordev = to_motor_dev(motor);
	struct stepmotor_running_info * p_running_info;
	unsigned long irq_flags;
	int rs = 0;
	if (!motordev)
		return -EINVAL;
	p_running_info = &motordev->running_info;
	motor->status |= STEPPERMOTOR_RUNNING;
	spin_lock_irqsave(&p_running_info->running_info_lock, irq_flags);
	p_running_info->step_total = motor->config.steps_to_run * motordev->stepping;
	p_running_info->step_left = p_running_info->step_total;
	p_running_info->step_lose = 0;
	p_running_info->running_st = STEPMOTOR_ST_INIT;
	spin_unlock_irqrestore(&p_running_info->running_info_lock, irq_flags);
	if (motordev->gpio_hold)
	{
		gpio_direction_output(motordev->gpio_hold, GPIO_VALUE_LOW);
	}
	gpio_direction_output(motordev->gpio_en,GPIO_VALUE_HIGH);
	ndelay(200);
	pwm_enable(motordev->pwm); 
	return rs;
}


static void pwm_stepmotor_stop(struct steppermotor *motor)
{
	struct motor_dev *motordev = to_motor_dev(motor);
	struct stepmotor_running_info * p_running_info;
	unsigned long irq_flags;

	if (!motor)
		return;
	p_running_info = &motordev->running_info;
	motor->status &= ~STEPPERMOTOR_RUNNING;
	gpio_direction_output(motordev->gpio_en, GPIO_VALUE_LOW);
	pwm_disable(motordev->pwm);
	spin_lock_irqsave(&p_running_info->running_info_lock, irq_flags);
	p_running_info->running_st = STEPMOTOR_ST_POSIT;
	spin_unlock_irqrestore(&p_running_info->running_info_lock, irq_flags);
	printk(KERN_DEBUG "STOPED - running step = %d .\r\n", p_running_info->step_lose);
	if (motor->callback)	//add by hl for debug 20170523
	{
		motor->status = STEPPERMOTOR_STOPPED_BY_TOTAL_STEPS;
		motor->callback(motor, &(motor->callbackdata));
	}
}

static void pwm_stepmotor_emergencybrake(struct steppermotor *motor)
{
	struct motor_dev *motordev = to_motor_dev(motor);
	struct stepmotor_running_info * p_running_info;
	unsigned long irq_flags;

	if (!motor)
		return;
	p_running_info = &motordev->running_info;
	motor->status &= ~STEPPERMOTOR_RUNNING;
	gpio_direction_output(motordev->gpio_en, GPIO_VALUE_LOW);
	pwm_disable(motordev->pwm);
	spin_lock_irqsave(&p_running_info->running_info_lock, irq_flags);
	p_running_info->running_st = STEPMOTOR_ST_POSIT;
	spin_unlock_irqrestore(&p_running_info->running_info_lock, irq_flags);
	printk(KERN_DEBUG "STOPED - running step = %d .\r\n", p_running_info->step_lose);
	if (motor->callback)	//add by hl for debug 20170523
	{
		motor->status = STEPPERMOTOR_STOPPED_BY_TOTAL_STEPS;
		motor->callback(motor, &(motor->callbackdata));
	}
}

static int pwm_stepmotor_lock(struct steppermotor *motor)
{
	struct motor_dev * motordev = to_motor_dev(motor);
	struct stepmotor_running_info * p_running_info;
	p_running_info = &motordev->running_info;
	//enabe motor_hold(vref) to dec the VREF & motor_en to lock the motor
	if (motordev->gpio_hold)
	{
		pwm_disable(motordev->pwm);
		gpio_direction_output(motordev->gpio_hold, GPIO_VALUE_HIGH);
		gpio_direction_output(motordev->gpio_en, GPIO_VALUE_HIGH);
		p_running_info->running_st = STEPMOTOR_ST_HOLD;
	}
	return 0;
}

static int pwm_stepmotor_unlock(struct steppermotor *motor)
{
	struct motor_dev * motordev = to_motor_dev(motor);
	struct stepmotor_running_info * p_running_info;
	p_running_info = &motordev->running_info;
	//disable motor_hold(vref) to resume the VREF & disable motor_en to unlock the motor
	if (motordev->gpio_hold)
	{
		pwm_disable(motordev->pwm);
		gpio_direction_output(motordev->gpio_en, GPIO_VALUE_LOW);
		gpio_direction_output(motordev->gpio_hold, GPIO_VALUE_LOW);
		p_running_info->running_st = STEPMOTOR_ST_POSIT;
	}
	return 0;
}

static int pwm_stepmotor_status(struct steppermotor *motor)
{
	return motor->status;
}

static inline void __iomem * pwm_ram_load_ramptable(void __iomem *addr, void __iomem *limit, u32 flag, struct motor_speedtable *speedtable)
{
#if 0
	void __iomem *ptr = addr;
	int i;
	u32 val;

	for (i = 0; i < speedtable->ramp_size; i++) 
	{
		if (ptr >= limit) {
			printk(KERN_ERR "fpga_stepmoter: ramp table [@%08x] exceeds steppermotor RAM limit %08x\n",
				(u32)ptr, (u32)limit);
			return ERR_PTR(-EINVAL);	// exceed table limit, return error
		}
		val = speedtable->ramp_table[i] | flag;
		fpga_writel(val, ptr);
		ptr += sizeof(u32);
	}
	return ptr;		// return next loading address
#endif
	return 0;
}


static inline void __iomem * pwm_ram_load_value(void __iomem *addr, u32 value)
{
	return 0;
}

static int pwm_stepmotor_config(struct steppermotor *motor, const struct steppermotor_config *config)
{
	int ret=0;
	int duty, period;
	struct motor_dev *motordev = to_motor_dev(motor);
	struct speed_info *speedinfo;
	int steps, speedlevel;
	struct stepmotor_running_info * p_running_info;
#if PWM_STEPMOTOR_SPEED_CTRL
	struct motor_speedtable *speedtable;
	int index;
	int speed1;
#endif
	if (!motordev || !config)
		return -EINVAL;
	ret = steppermotor_check_config(motor, config);
	if (IS_ERR_VALUE(ret))
	{
		dev_err(motor->dev, "error in configuration of steppermotor");
		return -ret;
	}
	if (steppermotor_is_running(motor->status))
		return -EBUSY;
	p_running_info = &motordev->running_info;
	mutex_lock(&stepmotor_mutex);
	
#if PWM_STEPMOTOR_SPEED_CTRL
	speedinfo = config->speedinfo;
	steps = speedinfo->steps;
	speed1 = speedinfo->speed;
	index = lookup_speedtable(motor, speed1);
	// load acceleration table of speed 1
	speedtable = motordev->rampinfo.speeds[index].accel_table;
	p_running_info->p_acc_ramp_table = speedtable->ramp_table;
	p_running_info->acc_ramp_table_size = speedtable->ramp_size;
	p_running_info->p_speed_cur = p_running_info->p_acc_ramp_table;
	// load decel table
	speedtable = motordev->rampinfo.speeds[index].decel_table;
	speedlevel = motordev->rampinfo.speeds[index].step_ticks;
	p_running_info->p_dec_ramp_table = speedtable->ramp_table;
	p_running_info->dec_ramp_table_size = speedtable->ramp_size;
	
	period = *p_running_info->p_speed_cur;	//sconfig->speedinfo->speed;
	duty = DRIVER_IC_DUTY;
#else
	period = DEFAULT_PERIOD;	//sconfig->speedinfo->speed;
	duty = DRIVER_IC_DUTY;
#endif
	printk("stepmotor config.\n");
	pwm_config(motordev->pwm, duty, period);
	gpio_direction_output(motordev->gpio_dir, (config->dir ? GPIO_VALUE_LOW : GPIO_VALUE_HIGH)); 
	
	mutex_unlock(&stepmotor_mutex);
	return ret;
}

/* 
  运行状态下返回当前已经走过的步数(全步)
  停止状态下返回，当前已运行步数|0x10000
 * */
static int pwm_stepmotor_get_running_steps(struct steppermotor *motor)
{
	struct motor_dev *motordev = to_motor_dev(motor);
	u32 val;
	struct stepmotor_running_info * p_running_info;
	//unsigned long irq_flags;

	if (!motor)
		return -EINVAL;

	p_running_info = &motordev->running_info;
	//spin_lock_irqsave(&p_running_info->running_info_lock, irq_flags);
	val = p_running_info->step_lose / motordev->stepping;
	if((p_running_info->running_st == STEPMOTOR_ST_INIT)
	||(p_running_info->running_st == STEPMOTOR_ST_HOLD)
	||(p_running_info->running_st == STEPMOTOR_ST_POSIT))
	{
		val |= STEP_MOTOR_STEPS_STOP_FLAG;
	}
	//spin_unlock_irqrestore(&p_running_info->running_info_lock, irq_flags);

	return val;
}


static int pwm_stepmotor_get_medialength_in_steps(struct steppermotor *motor)
{
	return 0;
}


static int pwm_stepmotor_set_trigger_next(struct steppermotor *motor, const struct steppermotor_trigger *trigger)
{
	return 0;
}

int pwm_stepmotor_set_sensor_sel_mask(struct steppermotor *motor, int sensel)
{
	return 0;
}


static int pwm_stepmotor_set_skew_steps(struct steppermotor *motor, int steps)
{
	return 0;
}

irqreturn_t pwm_stepmotor_isr(int irq, void *dev_id)
{
	struct motor_dev *motordev;
	struct steppermotor *motor;
	struct stepmotor_running_info * p_running_info;
	int step_left_tmp;
	unsigned long irq_flags;
	int duty = DRIVER_IC_DUTY;

	if (!dev_id)
		return -EINVAL;

	motordev = (struct motor_dev *)dev_id;
	motor = &motordev->motor;
	p_running_info = &motordev->running_info;
#if 1//PWM_STEPMOTOR_SPEED_CTRL
	if(p_running_info->running_st == STEPMOTOR_ST_INIT)	//初始化模式，切换到加速
	{
		spin_lock_irqsave(&p_running_info->running_info_lock, irq_flags);
		p_running_info->running_st = STEPMOTOR_ST_ACC;
		p_running_info->p_speed_cur = p_running_info->p_acc_ramp_table;
		spin_unlock_irqrestore(&p_running_info->running_info_lock, irq_flags);
	}
	if(p_running_info->running_st == STEPMOTOR_ST_ACC)
	{
		if(p_running_info->p_speed_cur < p_running_info->p_acc_ramp_table 
		+ p_running_info->acc_ramp_table_size)
		{
			pwm_config(motordev->pwm, duty, *p_running_info->p_speed_cur);
			spin_lock_irqsave(&p_running_info->running_info_lock, irq_flags);
			p_running_info->p_speed_cur++;
			spin_unlock_irqrestore(&p_running_info->running_info_lock, irq_flags);
		}
		else	//加速完毕，切换到匀速模式
		{
			spin_lock_irqsave(&p_running_info->running_info_lock, irq_flags);
			p_running_info->running_st = STEPMOTOR_ST_CONST;
			spin_unlock_irqrestore(&p_running_info->running_info_lock, irq_flags);
		}
	}
	else if(p_running_info->running_st == STEPMOTOR_ST_CONST)
	{
		//剩余步数小于等于减速表长度，切换到减速模式
		if(p_running_info->step_left < (p_running_info->dec_ramp_table_size*2))
		{
			spin_lock_irqsave(&p_running_info->running_info_lock, irq_flags);
			p_running_info->running_st = STEPMOTOR_ST_DEC;
			p_running_info->p_speed_cur = p_running_info->p_dec_ramp_table + 1;
			spin_unlock_irqrestore(&p_running_info->running_info_lock, irq_flags);
		}
	}
	else if(p_running_info->running_st == STEPMOTOR_ST_DEC)
	{
		if(p_running_info->p_speed_cur < p_running_info->p_dec_ramp_table
		+ p_running_info->dec_ramp_table_size)
		{
			pwm_config(motordev->pwm, duty, *p_running_info->p_speed_cur);
			spin_lock_irqsave(&p_running_info->running_info_lock, irq_flags);
			//pwm_config(motordev->pwm, duty, *p_running_info->p_speed_cur);
			p_running_info->p_speed_cur++;
			spin_unlock_irqrestore(&p_running_info->running_info_lock, irq_flags);
		}
	}
	//printk("(%d_%d)", p_running_info->running_st, *(p_running_info->p_speed_cur - 1));
#endif
	spin_lock_irqsave(&p_running_info->running_info_lock, irq_flags);
	if(p_running_info->step_left > 0)
	{
		p_running_info->step_left--;
		p_running_info->step_lose++;
	}
	step_left_tmp = p_running_info->step_left;
	spin_unlock_irqrestore(&p_running_info->running_info_lock, irq_flags);
	if (step_left_tmp <= 0)
	{
		motor->status &= ~STEPPERMOTOR_RUNNING;
		motor->status = STEPPERMOTOR_STOPPED_BY_TOTAL_STEPS;
		gpio_direction_output(motordev->gpio_en, GPIO_VALUE_LOW);
		pwm_disable(motordev->pwm);
		spin_lock_irqsave(&p_running_info->running_info_lock, irq_flags);
		p_running_info->running_st = STEPMOTOR_ST_POSIT;
		spin_unlock_irqrestore(&p_running_info->running_info_lock, irq_flags);
		if(motor->callback)
		{
			motor->status = STEPPERMOTOR_STOPPED_BY_TOTAL_STEPS;
			motor->callback(motor, &motor->callbackdata);
		}
	}

	return IRQ_HANDLED;
}
EXPORT_SYMBOL_GPL(pwm_stepmotor_isr);

static struct steppermotor_ops pwm_stepmotor_ops = {
	.config			= pwm_stepmotor_config,
	.status			= pwm_stepmotor_status,
	.start			= pwm_stepmotor_start,
	.stop			= pwm_stepmotor_stop,
	.lock			= pwm_stepmotor_lock,
	.unlock			= pwm_stepmotor_unlock,
	.emergencybrake		= pwm_stepmotor_emergencybrake,
	.get_running_steps	= pwm_stepmotor_get_running_steps,
	.get_medialength_in_steps = pwm_stepmotor_get_medialength_in_steps,
	.set_sensor_sel_mask	= pwm_stepmotor_set_sensor_sel_mask,
	.set_trigger_next	= pwm_stepmotor_set_trigger_next,
	.set_skew_steps		= pwm_stepmotor_set_skew_steps,
	.owner = THIS_MODULE,
};

static int gpio_init_from_dts(struct device *dev,struct motor_dev *motordev)
{
	int ret;
	struct device_node *np = dev->of_node;
	
	motordev->gpio_en = of_get_named_gpio(np, "wen-gpio", 0); 
	if (!gpio_is_valid(motordev->gpio_en)) {
		dev_err(dev, "no wen-gpio pin available\n");
		goto err;
	}
	ret = gpio_request(motordev->gpio_en, NULL);
	if(ret)
		return ret;

	motordev->gpio_dir = of_get_named_gpio(np, "wdir-gpio", 0); 
	if (!gpio_is_valid(motordev->gpio_dir)) {
		dev_err(dev, "no wdir-gpio pin available\n");
		goto err;
	} 
	ret = gpio_request(motordev->gpio_dir, NULL);
	if(ret)
		return ret;
	//if hold gpio is valid, config. else set to NULL
	motordev->gpio_hold = of_get_named_gpio(np, "whold-gpio", 0); 
	if (gpio_is_valid(motordev->gpio_hold))
	{
		ret = gpio_request(motordev->gpio_hold, NULL);
		if(ret)
			return ret;
	}
	else
	{
		motordev->gpio_hold = 0;
	}
err:
	return 0;
}

static int pwm_stepmotor_hw_init(struct motor_dev *motordev)
{
	int ret = 0;
	
	gpio_direction_output(motordev->gpio_en, GPIO_VALUE_LOW);
	if (motordev->gpio_hold)
	{
		gpio_direction_output(motordev->gpio_hold, GPIO_VALUE_LOW);
	}
	
	return ret;
}

static int pwm_stepmotor_probe(struct platform_device *pdev)
{
	struct motor_dev *motordev;
	struct device_node *np = pdev->dev.of_node;
	int ret, val;
	struct stepmotor_running_info * p_running_info;

	motordev = devm_kzalloc(&pdev->dev, sizeof(*motordev), GFP_KERNEL);
	if (motordev == NULL)
		return -ENOMEM;
	motordev->motor.dev = &pdev->dev;
	motordev->pwm = of_pwm_get(np, NULL);
	if (IS_ERR(motordev->pwm))
		return -ENODEV;
	ret = gpio_init_from_dts(&pdev->dev, motordev);
	if (ret)
	{
		dev_err(&pdev->dev, "Failed to init gpio from dts: %d\n", ret);
		return ret;
	}
	ret = of_property_read_u32(np, "clock-period", &motordev->clock_period);
	if (ret) {
		dev_err(&pdev->dev, "Failed to parse FPGA steppermotor unit clock period\n");
		return ret;
	}
	ret = of_property_read_u32(np, "pullin-speed", &val);
	if (ret) {
		dev_err(&pdev->dev, "Failed to parse steppermotor pull-in speed\n");
		return ret;
	}
	if (val <= 0) {
		dev_err(&pdev->dev, "Invalid steppermotor pull-in speed: %d\n", val);
		return -EINVAL;
	}
	motordev->motor.feature.pullin_speed = val;

	ret = of_property_read_u32(np, "stepping", &val);
	if (ret) {
		dev_info(&pdev->dev, "Failed to parse FPGA steppermotor stepping multiple\n");
		return ret;
	}
	if (val!=1 && val!=2 && val!=4 && val!=8 && val!=16 && val!=32) {
		dev_err(&pdev->dev, "Invalid FPGA steppermotor stepping multiple: %d\n", val);
		return -EINVAL;
	}
	motordev->stepping = val;

	ret = steppermotor_speedtable_parse(&pdev->dev, &motordev->speedtable_list, motordev->stepping);
	if (IS_ERR_VALUE(ret)) {
		dev_err(&pdev->dev, "Failed to parse speedtable information of a steppermotor\n");
		return ret;
	}
	ret = steppermotor_speedtable_analysis(&pdev->dev, &motordev->speedtable_list, &motordev->motor.feature, &motordev->rampinfo); 
	if (IS_ERR_VALUE(ret)) {
		dev_err(&pdev->dev, "Failed to analysis speed feature information from speedtable\n");
		return ret;
	}
	ret = steppermotor_ramptable_convert(&motordev->rampinfo, motordev->clock_period); 
	if (IS_ERR_VALUE(ret)) {
		dev_err(&pdev->dev, "Failed to conver ramp table values to ticks count in speedtable\n");
		return ret;
	}
	pwm_stepmotor_hw_init(motordev);
	motordev->motor.ops = &pwm_stepmotor_ops;
	motordev->motor.feature.max_steps = MAXIMUM_STEPS / motordev->stepping; 
	imx_pwm_set_callback(motordev->pwm, pwm_stepmotor_isr, motordev);
	p_running_info = &motordev->running_info;
	p_running_info->step_total = 0;
	p_running_info->step_lose = 0;
	p_running_info->step_left = 0;
	p_running_info->running_st = STEPMOTOR_ST_POSIT;
	spin_lock_init(&p_running_info->running_info_lock);
	ret = steppermotor_add(&motordev->motor);
	if (IS_ERR_VALUE(ret)) {
		dev_err(&pdev->dev, "Failed to register a steppermotor\n");
		return ret;
	}

#ifdef DEBUG
	dev_info(&pdev->dev, "steppermotor features:\n");
	{
		int i;

		printk(KERN_INFO "maximum steps = %d\n", motordev->motor.feature.max_steps); 
		printk(KERN_INFO "pull-in speed = %d SPS\n", motordev->motor.feature.pullin_speed);
		printk(KERN_INFO "supports %d speeds:\n", motordev->motor.feature.num_speed);
		for (i = 0; i < motordev->motor.feature.num_speed; i++)
			printk(KERN_INFO "%d SPS, accel_steps = %d, decel_steps = %d\n", motordev->motor.feature.speeds[i].speed,
				motordev->motor.feature.speeds[i].accel_steps, motordev->motor.feature.speeds[i].decel_steps);
		printk(KERN_INFO "supports %d speed-shifts:\n", motordev->motor.feature.num_speedshift);
		for (i = 0; i < motordev->motor.feature.num_speedshift; i++)
			printk(KERN_INFO "%d -> %d SPS, shift-steps = %d\n", motordev->motor.feature.speedshifts[i].speed1,
				motordev->motor.feature.speedshifts[i].speed2, motordev->motor.feature.speedshifts[i].steps);
		printk(KERN_INFO "\n");
	}
#endif

	platform_set_drvdata(pdev, motordev);
	return 0;
}


static int pwm_stepmotor_remove(struct platform_device *pdev)
{
	struct motor_dev *motordev = platform_get_drvdata(pdev);
	steppermotor_remove(&motordev->motor);

	return 0;
}


static const struct of_device_id pwm_stepmotor_match[] = {
	{ .compatible = "gwi,pwm-steppermotor", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, pwm_stepmotor_match);

static struct platform_driver pwm_stepmotor_driver = {
	.probe          = pwm_stepmotor_probe,
	.remove         = pwm_stepmotor_remove,
	.driver         = {
		.name   = "pwm-stepmotor",
		.of_match_table = pwm_stepmotor_match,
	},
};

module_platform_driver(pwm_stepmotor_driver);

MODULE_AUTHOR("Zhang Xudong <zhangxudong@gwi.com.cn>");
MODULE_DESCRIPTION("PWM-controlled steppermotor driver");
MODULE_LICENSE("GPL v2");
