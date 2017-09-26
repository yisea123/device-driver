/*
 * FPGA-controlled Stepper Motor driver
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


#include "steppermotor.h"
#include "steptable.h"

#include "../fpga_io.h"
#include "../fpga.h"


extern int motorint_handler_register(u32 mask, irq_handler_t handler, void * dev_id);
extern int motorint_handler_unregister(irq_handler_t handler);

static inline int _fpga_stepmotor_status(struct steppermotor *motor);


#define MAXIMUM_STEPS		0xffff

#define SPEED_TO_COUNT(speed,stepping,clock)	(((1000000000L / (speed)) / (stepping)) / (clock))


#define TRIGGER_NEXT_CTRL_MASK	(FPGA_REG_MOTOR_STOP_AT_TRGSTEP_END | FPGA_REG_MOTOR_SENSOR_CHECK_MODE |	\
				FPGA_REG_MOTOR_SENSER_STOP_ENABLE | FPGA_REG_MOTOR_SENSER_CONTINUE_MODE |	\
				FPGA_REG_MOTOR_SENSER_STOP_MODE | FPGA_REG_MOTOR_EN_SKEW_STEPS)

#define MOTOR_TABLE_RAMP_LIMIT		(motordev->ram_base + FPGA_RAM_MOTOR_TABLE_RAMP + FPGA_RAM_MOTOR_TABLE_RAMP_SIZE)
#define MOTOR_TABLE_COUNT_LIMIT		(motordev->ram_base + FPGA_RAM_MOTOR_TABLE_COUNT + FPGA_RAM_MOTOR_TABLE_COUNT_SIZE)
#define MOTOR_TABLE_STOPHIGH_LIMIT	(motordev->ram_base + FPGA_RAM_MOTOR_TABLE_STOPHIGH + FPGA_RAM_MOTOR_TABLE_STOPHIGH_SIZE)
#define MOTOR_TABLE_STOPLOW_LIMIT	(motordev->ram_base + FPGA_RAM_MOTOR_TABLE_STOPLOW + FPGA_RAM_MOTOR_TABLE_STOPLOW_SIZE)


static DEFINE_MUTEX(stepmotor_mutex);


struct motor_dev {
	struct steppermotor motor;
	void __iomem *mmio_base;	// registers base address
	void __iomem *ram_base;		// RAM base address
	int ram_size;			// RAM size (in bytes)
	u32 mask;			// motor interrupt bit mask
	int stepping;			// motor stepping multiple (should be one of 1/2/4/8/16/32)
	int clock_period;		// motor unit clock period (in nanoseconds)
	struct list_head speedtable_list;
	struct ramp_info rampinfo;	// ramp table of supported speeds and speed-shifts
};

#define to_motor_dev(motor)	container_of(motor, struct motor_dev, motor)


static int fpga_stepmotor_start(struct steppermotor *motor)
{
	struct motor_dev *motordev = to_motor_dev(motor);
	int rs;

	if (!motor)
		return -EINVAL;

	rs = fpga_update_lbits(motordev->mmio_base + FPGA_REG_MOTOR_CONTROL, FPGA_REG_MOTOR_RUN|FPGA_REG_MOTOR_STOP|FPGA_REG_MOTOR_EMERGENCY_BRAKE, 0);
	rs = fpga_update_lbits(motordev->mmio_base + FPGA_REG_MOTOR_CONTROL, FPGA_REG_MOTOR_RUN, FPGA_REG_MOTOR_RUN);

	motor->status |= STEPPERMOTOR_RUNNING;
	return rs;
}


static void fpga_stepmotor_stop(struct steppermotor *motor)
{
	struct motor_dev *motordev = to_motor_dev(motor);

	if (!motor)
		return;

	fpga_update_lbits(motordev->mmio_base + FPGA_REG_MOTOR_CONTROL, FPGA_REG_MOTOR_RUN|FPGA_REG_MOTOR_STOP|FPGA_REG_MOTOR_EMERGENCY_BRAKE, FPGA_REG_MOTOR_STOP);
}


static void fpga_stepmotor_emergencybrake(struct steppermotor *motor)
{
	struct motor_dev *motordev = to_motor_dev(motor);

	if (!motor)
		return;

	fpga_update_lbits(motordev->mmio_base + FPGA_REG_MOTOR_CONTROL, FPGA_REG_MOTOR_RUN|FPGA_REG_MOTOR_STOP|FPGA_REG_MOTOR_EMERGENCY_BRAKE, FPGA_REG_MOTOR_EMERGENCY_BRAKE);

	motor->status = _fpga_stepmotor_status(motor);
}


static inline int _fpga_stepmotor_status(struct steppermotor *motor)
{
	struct motor_dev *motordev = to_motor_dev(motor);
	int rs, ret;
	u32 status;

	if (!motor)
		return -EINVAL;

	rs = fpga_readl(&status, motordev->mmio_base + FPGA_REG_MOTOR_STATUS);
	if (rs)
		return 0;

	ret = 0;
	if (status & FPGA_REG_MOTOR_STOPPED)
	{
		if (status & FPGA_REG_MOTOR_STOPPED_BY_SENSOR) 
			ret |= STEPPERMOTOR_STOPPED_BY_SENSOR;
		else if (status & FPGA_REG_MOTOR_STOPPED_BY_TRIGGERSTEP) 
			ret |= STEPPERMOTOR_STOPPED_BY_TRIGGER;
		else if (status & FPGA_REG_MOTOR_STOPPED_BY_SKEW)
			ret |= STEPPERMOTOR_STOPPED_BY_SKEW;
		else
			ret |= STEPPERMOTOR_STOPPED_BY_TOTAL_STEPS;
	}
	else
		ret |= STEPPERMOTOR_RUNNING;

	if (status & FPGA_REG_MOTOR_MEDIALENGTH_OK) 
		ret |= STEPPERMOTOR_MEDIALENGTH_OK;
	if (status & FPGA_REG_MOTOR_TRIGGERSTEP_DONE)
		ret |= STEPPERMOTOR_TRIGGER_STEPS_DONE;
	if (status & FPGA_REG_MOTOR_FAULT)
		ret |= STEPPERMOTOR_FAULT;

	return ret;
}


static int fpga_stepmotor_status(struct steppermotor *motor)
{
	return _fpga_stepmotor_status(motor);
}


static inline void fpga_ram_clear(void __iomem *addr, int size)
{
	int i;
	for (i = 0; i < size;)
	{
		fpga_writel(0, addr);
		addr += sizeof(u32);
		i += sizeof(u32);
	}
}


static int fpga_stepmotor_hw_init(struct motor_dev *motordev)
{
	int ret;

	mutex_lock(&stepmotor_mutex);

	// clear motor control register
	ret = fpga_writel(0, motordev->mmio_base + FPGA_REG_MOTOR_CONTROL);

	// clear entire RAM of this steppermotor
	fpga_ram_clear(motordev->ram_base, motordev->ram_size);

	mutex_unlock(&stepmotor_mutex);
	return 0;
}


static inline void __iomem * fpga_ram_load_ramptable(void __iomem *addr, void __iomem *limit, u32 flag, struct motor_speedtable *speedtable)
{
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
}


static inline void __iomem * fpga_ram_load_value(void __iomem *addr, u32 value)
{
	fpga_writel(value, addr);
	addr += sizeof(u32);
	return addr;		// return next loading address
}


static int fpga_stepmotor_config(struct steppermotor *motor, const struct steppermotor_config *config)
{
	struct motor_dev *motordev = to_motor_dev(motor);
	struct speed_info *speedinfo;
	int ret, steps, speedlevel;
	u32 val;

	if (!motor || !config)
		return -EINVAL;

	ret = steppermotor_check_config(motor, config);
	if (IS_ERR_VALUE(ret))
	{
		dev_err(motor->dev, "error in configuration of steppermotor");
		return -ret;
	}

	if (steppermotor_is_running(motor->status))
		return -EBUSY;

	mutex_lock(&stepmotor_mutex);

	// clear ramp table and speed profile table
	fpga_ram_clear(motordev->ram_base + FPGA_RAM_MOTOR_TABLE_RAMP, FPGA_RAM_MOTOR_TABLE_RAMP_SIZE); 
	fpga_ram_clear(motordev->ram_base + FPGA_RAM_MOTOR_TABLE_COUNT, FPGA_RAM_MOTOR_TABLE_COUNT_SIZE);

	if (config->steps_to_run == 1)	// single-step configuration
	{
		/* write speed table */
		val = SPEED_TO_COUNT(motor->feature.pullin_speed, motordev->stepping, motordev->clock_period);
		val |= FPGA_RAM_MOTOR_TABLE_RAMP_CONST1;
		ret = fpga_writel(val, motordev->ram_base + FPGA_RAM_MOTOR_TABLE_RAMP);

		/* write profile table */
		val = 1 * motordev->stepping;
		ret = fpga_writel(val, motordev->ram_base + FPGA_RAM_MOTOR_TABLE_COUNT); 
	}
	else	// multiple-steps configuration
	{
		struct motor_speedtable *speedtable, *stoptable_low, *stoptable_high;
		void __iomem *addr, *ptr_ramp, *ptr_count;
		int index, rsteps;
		int speed1, speed2;

		speedinfo = config->speedinfo;
		steps = speedinfo->steps;
		speed1 = speedinfo->speed;

		// compose single-speed or multiple-speeds and load int RAM
		index = lookup_speedtable(motor, speed1);
		speedtable = motordev->rampinfo.speeds[index].accel_table;

		rsteps = steps * speedtable->stepping;

		ptr_ramp = motordev->ram_base + FPGA_RAM_MOTOR_TABLE_RAMP;
		ptr_count = motordev->ram_base + FPGA_RAM_MOTOR_TABLE_COUNT;

		// load acceleration table of speed 1
		addr = fpga_ram_load_ramptable(ptr_ramp, MOTOR_TABLE_RAMP_LIMIT, FPGA_RAM_MOTOR_TABLE_RAMP_ACCEL, speedtable); 
		dev_dbg(motor->dev, "load acceleration ramp table (speed1) @ %08x, ret=%08x", (u32)ptr_ramp, (u32)addr);
		if (IS_ERR(addr))
			goto err_handling;
		else
			ptr_ramp = addr;


		// setup acceleration steps
		ptr_count = fpga_ram_load_value(ptr_count, speedtable->ramp_size);
		rsteps -= speedtable->ramp_size;

		speedtable = stoptable_low = stoptable_high = motordev->rampinfo.speeds[index].decel_table;

		// load constant-speed table entry of speed 1
		val = speedtable->ramp_table[0] | FPGA_RAM_MOTOR_TABLE_RAMP_CONST1; 
		ptr_ramp = fpga_ram_load_value(ptr_ramp, val);
		speedlevel = motordev->rampinfo.speeds[index].step_ticks; 

		if (config->num_speed == 1)	// single speed configuration
		{
			// setup constant-speed steps
			rsteps = rsteps - speedtable->ramp_size;
			ptr_count = fpga_ram_load_value(ptr_count, rsteps);
			// setup deceleration steps
			ptr_count = fpga_ram_load_value(ptr_count, speedtable->ramp_size);
		}
		else	// speed shift configuration
		{
			struct motor_speedtable *shifttable;
			int flag, rsteps2;

			speedinfo = speedinfo->nextspeed;
			speed2 = speedinfo->speed;
			// get speed-shift table
			index = lookup_shifttable(motor, speed1, speed2);
			shifttable = motordev->rampinfo.speedshifts[index].shift_table;

			// get deceleration ramp table of speed 2
			index = lookup_speedtable(motor, speed2);
			speedtable = motordev->rampinfo.speeds[index].decel_table;

			rsteps2 = speedinfo->steps * speedtable->stepping;

			// setup stop tables
			if (speed1 < speed2)
				stoptable_high = speedtable;
			else
			{
				stoptable_low = speedtable;
				speedlevel = speedtable->ramp_table[0];		// set level to lower speed
			}

			// setup constant-speed steps of speed 1
			ptr_count = fpga_ram_load_value(ptr_count, rsteps);
			// setup speed-shift steps
			ptr_count = fpga_ram_load_value(ptr_count, shifttable->ramp_size);
			val = rsteps2 - shifttable->ramp_size - speedtable->ramp_size;
			// setup constant-speed steps of speed 2
			ptr_count = fpga_ram_load_value(ptr_count, val);
			// setup deceleration steps of speed 2
			ptr_count = fpga_ram_load_value(ptr_count, speedtable->ramp_size);

			// load speed-shift ramp table
			flag = (speed1 < speed2) ? FPGA_RAM_MOTOR_TABLE_RAMP_ACCEL : FPGA_RAM_MOTOR_TABLE_RAMP_DECEL;
			addr = fpga_ram_load_ramptable(ptr_ramp, MOTOR_TABLE_RAMP_LIMIT, flag, shifttable);
			dev_dbg(motor->dev, "load speed-shift ramp table @ %08x, ret=%08x", (u32)ptr_ramp, (u32)addr);
			if (IS_ERR(addr))
				goto err_handling;
			else
				ptr_ramp = addr;

			// load constant-speed table entry of speed 2
			val = speedtable->ramp_table[0] | FPGA_RAM_MOTOR_TABLE_RAMP_CONST2; 
			ptr_ramp = fpga_ram_load_value(ptr_ramp, val);

		}
		// load deceleration ramp table
		addr = fpga_ram_load_ramptable(ptr_ramp, MOTOR_TABLE_RAMP_LIMIT, FPGA_RAM_MOTOR_TABLE_RAMP_DECEL, speedtable);
		dev_dbg(motor->dev, "load deceleration ramp table @ %08x, ret=%08x", (u32)ptr_ramp, (u32)addr);
err_handling:

		// load stop tables
		ptr_ramp = motordev->ram_base + FPGA_RAM_MOTOR_TABLE_STOPHIGH;
		addr = fpga_ram_load_ramptable(ptr_ramp, MOTOR_TABLE_STOPHIGH_LIMIT, 0, stoptable_high);
		dev_dbg(motor->dev, "load stop table HIGH @ %08x, ret=%08x", (u32)ptr_ramp, (u32)addr);

		ptr_ramp = motordev->ram_base + FPGA_RAM_MOTOR_TABLE_STOPLOW;
		addr = fpga_ram_load_ramptable(ptr_ramp, MOTOR_TABLE_STOPLOW_LIMIT, 0, stoptable_low);
		dev_dbg(motor->dev, "load stop tables LOW @ %08x, ret=%08x", (u32)ptr_ramp, (u32)addr);
	}

	/* setup motion direction */
	val = (config->dir == MOTION_CLOCKWISE) ? FPGA_REG_MOTOR_DIRECTION : 0;
	fpga_update_lbits(motordev->mmio_base + FPGA_REG_MOTOR_CONTROL, FPGA_REG_MOTOR_DIRECTION, val);

	/* setup speed level register */
	fpga_writel(speedlevel, motordev->mmio_base + FPGA_REG_MOTOR_SPEEDLEVEL);

	/* setup preset running steps register */
	val = config->steps_to_run * motordev->stepping;
	fpga_writel(val, motordev->mmio_base + FPGA_REG_MOTOR_PRESET_STEPS);

	mutex_unlock(&stepmotor_mutex);

	return 0;
}


static int fpga_stepmotor_set_sensor_sel_mask(struct steppermotor *motor, int sensel)
{
	struct motor_dev *motordev = to_motor_dev(motor);

	if (!motor)
		return -EINVAL;

	return fpga_writel(sensel, motordev->mmio_base + FPGA_REG_MOTOR_SENSOR_SELECT_MASK);
}


static int fpga_stepmotor_get_running_steps(struct steppermotor *motor)
{
	struct motor_dev *motordev = to_motor_dev(motor);
	u32 val;
	int rs;

	if (!motor)
		return -EINVAL;

	rs = fpga_readl(&val, motordev->mmio_base + FPGA_REG_MOTOR_RUNNING_STEPS);
	if (rs)
		return rs;

	val = val / motordev->stepping;
	return val;
}


static int fpga_stepmotor_get_medialength_in_steps(struct steppermotor *motor)
{
	struct motor_dev *motordev = to_motor_dev(motor);
	u32 val;
	int rs;

	if (!motor)
		return -EINVAL;

	rs = fpga_readl(&val, motordev->mmio_base + FPGA_REG_MOTOR_MEDIALENGTH_STEPS);
	if (rs)
		return rs;

	val = val / motordev->stepping;
	return val;
}


static int fpga_stepmotor_set_trigger_next(struct steppermotor *motor, const struct steppermotor_trigger *trigger)
{
	struct motor_dev *motordev = to_motor_dev(motor);
	u32 val;
	int rs;

	if (!motor)
		return -EINVAL;

	val = 0;
	if (trigger->control_set_trigger_stop) 
		val |= FPGA_REG_MOTOR_STOP_AT_TRGSTEP_END;
	if (trigger->control_set_trigger_sensor) 
		val |= FPGA_REG_MOTOR_SENSOR_CHECK_MODE;
	if (trigger->control_set_sensor_stop) 
		val |= FPGA_REG_MOTOR_SENSER_STOP_ENABLE;
	if (trigger->control_set_sensor_continue_mode) 	
		val |= FPGA_REG_MOTOR_SENSER_CONTINUE_MODE;
	if (trigger->control_set_sensor_stop_mode) 		
		val |= FPGA_REG_MOTOR_SENSER_STOP_MODE;
	if (trigger->control_set_en_skew_steps) 
		val |= FPGA_REG_MOTOR_EN_SKEW_STEPS;
	rs = fpga_update_lbits(motordev->mmio_base + FPGA_REG_MOTOR_CONTROL, TRIGGER_NEXT_CTRL_MASK, val);

	val = (u32)(trigger->steps * motordev->stepping);
	return fpga_writel(val, motordev->mmio_base + FPGA_REG_MOTOR_TRIGGER_STEP_NEXT);
}


static int fpga_stepmotor_set_skew_steps(struct steppermotor *motor, int steps)
{
	struct motor_dev *motordev = to_motor_dev(motor);
	u32 val;

	if (!motor)
		return -EINVAL;

	val = (u32)(steps * motordev->stepping);
	return fpga_writel(val, motordev->mmio_base + FPGA_REG_MOTOR_SKEW_STEPS);
}


static irqreturn_t fpga_stepmotor_isr(int irq, void *dev_id)
{
	struct motor_dev *motordev;
	struct steppermotor *motor;
	int status;

	if (!dev_id)
		return -EINVAL;

	motordev = (struct motor_dev *)dev_id;
	motor = &motordev->motor;

	status = _fpga_stepmotor_status(motor);
	if (status & STEPPERMOTOR_STOPPED_MASK)
		fpga_update_lbits(motordev->mmio_base + FPGA_REG_MOTOR_CONTROL, FPGA_REG_MOTOR_RUN, 0);

	/* update steppermotor status */
	motor->status = status;

	if(motor->callback)
		motor->callback(motor, &motor->callbackdata);
	return IRQ_HANDLED;
}


static struct steppermotor_ops fpga_stepmotor_ops = {
	.config = fpga_stepmotor_config,
	.status = fpga_stepmotor_status,
	.start = fpga_stepmotor_start,
	.stop = fpga_stepmotor_stop,
	.emergencybrake = fpga_stepmotor_emergencybrake,
	.get_running_steps = fpga_stepmotor_get_running_steps,
	.get_medialength_in_steps = fpga_stepmotor_get_medialength_in_steps,
	.set_sensor_sel_mask = fpga_stepmotor_set_sensor_sel_mask,
	.set_trigger_next = fpga_stepmotor_set_trigger_next,
	.set_skew_steps = fpga_stepmotor_set_skew_steps,
	.owner = THIS_MODULE,
};


static int fpga_stepmotor_probe(struct platform_device *pdev)
{
	struct motor_dev *motordev;
	struct device_node *np = pdev->dev.of_node;
	int ret, val;
	u32 reg[3];
	u32 fpga_cs;

	motordev = devm_kzalloc(&pdev->dev, sizeof(*motordev), GFP_KERNEL);
	if (motordev == NULL)
		return -ENOMEM;

	ret = of_property_read_u32_array(np, "reg", reg, 3);
	if (IS_ERR_VALUE(ret)) {
		dev_err(&pdev->dev, "invalid FPGA steppermotor registers memory definition\n");
		return ret;
	}
	fpga_cs = reg[0];
	motordev->mmio_base = fpga_io_get(fpga_cs);
	if (IS_ERR(motordev->mmio_base))
		return PTR_ERR(motordev->mmio_base);

	motordev->mmio_base += reg[1];
	dev_info(&pdev->dev, "mmio_base = %08X.\n", (u32)motordev->mmio_base);

	ret = of_property_read_u32_array(np, "ram-reg", reg, 3);
	if (IS_ERR_VALUE(ret)) {
		dev_err(&pdev->dev, "invalid FPGA steppermotor RAM definition\n");
		return ret;
	}
	fpga_cs = reg[0];
	motordev->ram_base = fpga_io_get(fpga_cs);
	if (IS_ERR(motordev->ram_base))
		return PTR_ERR(motordev->ram_base);

	motordev->ram_base += reg[1];
	motordev->ram_size = reg[2];
	dev_info(&pdev->dev, "ram_base = %08X, ram_size = %08X.\n", (u32)motordev->ram_base, (u32)motordev->ram_size);

	ret = of_property_read_u32(np, "mask", &motordev->mask);
	if (ret) {
		dev_err(&pdev->dev, "Failed to parse interrupt mask of FPGA steppermotor\n");
		return ret;
	}

	ret = motorint_handler_register(motordev->mask, fpga_stepmotor_isr, (void *)motordev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register FPGA steppermotor interrupt handler\n");
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
		dev_err(&pdev->dev, "Failed to parse FPGA steppermotor stepping multiple\n");
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

	motordev->motor.dev = &pdev->dev;
	motordev->motor.ops = &fpga_stepmotor_ops;
	motordev->motor.feature.max_steps = MAXIMUM_STEPS / motordev->stepping; 
	ret = steppermotor_add(&motordev->motor);
	if (IS_ERR_VALUE(ret)) {
		dev_err(&pdev->dev, "Failed to register a steppermotor\n");
		return ret;
	}

#ifdef DEBUG
	dev_dbg(&pdev->dev, "steppermotor features:\n");
	{
		int i;

		printk(KERN_DEBUG "maximum steps = %d\n", motordev->motor.feature.max_steps); 
		printk(KERN_DEBUG "pull-in speed = %d SPS\n", motordev->motor.feature.pullin_speed);
		printk(KERN_DEBUG "supports %d speeds:\n", motordev->motor.feature.num_speed);
		for (i = 0; i < motordev->motor.feature.num_speed; i++)
			printk(KERN_DEBUG "%d SPS, accel_steps = %d, decel_steps = %d\n", motordev->motor.feature.speeds[i].speed,
				motordev->motor.feature.speeds[i].accel_steps, motordev->motor.feature.speeds[i].decel_steps);
		printk(KERN_DEBUG "supports %d speed-shifts:\n", motordev->motor.feature.num_speedshift);
		for (i = 0; i < motordev->motor.feature.num_speedshift; i++)
			printk(KERN_DEBUG "%d -> %d SPS, shift-steps = %d\n", motordev->motor.feature.speedshifts[i].speed1,
				motordev->motor.feature.speedshifts[i].speed2, motordev->motor.feature.speedshifts[i].steps);
		printk(KERN_DEBUG "\n");
	}
#endif
	ret = fpga_stepmotor_hw_init(motordev);

	platform_set_drvdata(pdev, motordev);
	return 0;
}


static int fpga_stepmotor_remove(struct platform_device *pdev)
{
	struct motor_dev *motordev = platform_get_drvdata(pdev);
	steppermotor_remove(&motordev->motor);

	motorint_handler_unregister(fpga_stepmotor_isr);
	return 0;
}


static const struct of_device_id fpga_stepmotor_match[] = {
	{ .compatible = "gwi,fpga-steppermotor", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, fpga_stepmotor_match);

static struct platform_driver fpga_stepmotor_driver = {
	.probe          = fpga_stepmotor_probe,
	.remove         = fpga_stepmotor_remove,
	.driver         = {
		.name   = "steppermotor",
		.of_match_table = fpga_stepmotor_match,
	},
};

module_platform_driver(fpga_stepmotor_driver);

MODULE_AUTHOR("Zhang Xudong <zhangxudong@gwi.com.cn>");
MODULE_DESCRIPTION("FPGA-controlled steppermotor driver");
MODULE_LICENSE("GPL v2");
