/*
 * generic steppermotor driver functions implementation
 *
 * Copyright 2017 Hunan GreatWall Information Financial Equipment Co., Ltd.
 *
 * Licensed under the GPL-2.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>


#include "steppermotor.h"
#include "steptable.h"


static DEFINE_MUTEX(steppermotor_mutex);
static LIST_HEAD(steppermotor_list);
static DEFINE_MUTEX(speedtable_mutex);


/**
 * steppermotor_add() - register a new steppermotor
 * @motor: the steppermotor to add
 *
 * Register a new steppermotor.
 */
int steppermotor_add(struct steppermotor *motor)
{
	if (!motor || !motor->dev || !motor->ops)
		return -EINVAL;

	/* check basic operation implemention */
	if (!motor->ops->owner || !motor->ops->config || !motor->ops->start || !motor->ops->stop)
		return -EINVAL;

	mutex_lock(&steppermotor_mutex);

	INIT_LIST_HEAD(&motor->list);
	list_add(&motor->list, &steppermotor_list);

	mutex_unlock(&steppermotor_mutex);
	return 0;
}
EXPORT_SYMBOL_GPL(steppermotor_add);


/**
 * steppermotor_remove() - remove a steppermotor
 * @motor: the steppermotor to remove
 *
 * Removes a steppermotor. This function may return busy if the
 * steppermotor is still requested.
 */
int steppermotor_remove(struct steppermotor *motor)
{
	if (motor->using)
		return -EBUSY;

	mutex_lock(&steppermotor_mutex);

	list_del_init(&motor->list);

	mutex_unlock(&steppermotor_mutex);
	return 0;
}
EXPORT_SYMBOL_GPL(steppermotor_remove);


/*
 * steppermotor_speedtable_parse() - parse speedtable information of a steppermotor from its devicetree
 * @dev: device structure pointer of the steppermotor
 *
 * parse speedtable information of a steppermotor from devicetree
 */
int steppermotor_speedtable_parse(struct device *dev, struct list_head *speedtable_list, int stepping)
{
	struct device_node *child, *np;
	struct motor_speedtable *speedtable;
	int ret, speedcnt;

	if (!dev || !dev->of_node || !speedtable_list)
		return -EINVAL;

	INIT_LIST_HEAD(speedtable_list);

	speedcnt = 0;
	np = dev->of_node;
	for_each_child_of_node(np, child)
	{
		size_t memsize;
		int val, num, start_speed, object_speed;

		ret = of_property_read_u32(child, "start-speed", &start_speed);
		if (IS_ERR_VALUE(ret)) {
			dev_err(dev, "Failed to read start-speed");
			return -EINVAL;
		}
		ret = of_property_read_u32(child, "object-speed", &object_speed);
		if (IS_ERR_VALUE(ret)) {
			dev_err(dev, "Failed to read object-speed");
			return -EINVAL;
		}
		list_for_each_entry(speedtable, speedtable_list, list)
			if ((speedtable->start_speed == start_speed) && (speedtable->object_speed == object_speed))
			{
				dev_err(dev, "duplicated ramp table (start-speed = %d, object-speed = %d)", start_speed, object_speed);
				return -EINVAL;
			}

		/* count ramp-table elements */
		num = of_property_count_u32_elems(child, "ramp-table");
		if (IS_ERR_VALUE(num)) {
			dev_err(dev, "Failed to find ramp-table");
			return num;
		}
		if (num > MAX_RAMP_LEN) {
			dev_err(dev, "Too many items in the ramp-table");
			return -EINVAL;
		}
		memsize = sizeof(struct motor_speedtable) + num * sizeof(u32);
		speedtable = devm_kzalloc(dev, memsize , GFP_KERNEL);
		if (IS_ERR(speedtable)) {
			dev_err(dev, "Failed to allocate memory for ramp-table");
			return -ENOMEM;
		}
		speedtable->ramp_size = num;
		ret = of_property_read_u32_array(child, "ramp-table", speedtable->ramp_table, num);
		if (IS_ERR_VALUE(ret)) {
			dev_err(dev, "Failed to read ramp-table");
			return -EINVAL;
		}

		/* stepping may be overridden within a ramp-table */
		ret = of_property_read_u32(child, "stepping", &val);
		if (!IS_ERR_VALUE(ret))
		{
			if (val!=1 && val!=2 && val!=4 && val!=8 && val!=16 && val!=32) {
				dev_err(dev, "invalid steppermotor stepping multiple: %d\n", val);
				return -EINVAL;
			}
			stepping = val;		// override stepping value
		}
		if (stepping == 0) {
			dev_err(dev, "steppermotor stepping shoule be defined either globally or locally\n");
			return -EINVAL;
		}

		speedtable->stepping = stepping;
		speedtable->start_speed = start_speed;
		speedtable->object_speed = object_speed;

		mutex_lock(&speedtable_mutex); 
		INIT_LIST_HEAD(&speedtable->list);
		list_add_tail(&speedtable->list, speedtable_list);
		mutex_unlock(&speedtable_mutex);

		++speedcnt;
	}
	return speedcnt;
}
EXPORT_SYMBOL_GPL(steppermotor_speedtable_parse);


/*
 * steppermotor_speedtable_analysis() - analysis speedtable information of a steppermotor
 * @dev: device structure pointer of the steppermotor
 *
 * analysis speedtable information, store results in steppermotor feature table and ramp information table
 */
int steppermotor_speedtable_analysis(struct device *dev, struct list_head *speedtable_list, struct steppermotor_feature *feature, struct ramp_info *rampinfo)
{
	struct motor_speedtable *speedtable;
	int speeds, shifts;

	if (!speedtable_list || !feature || !rampinfo)
		return -EINVAL;

	speeds = shifts = 0;
	list_for_each_entry(speedtable, speedtable_list, list)
	{
		if (speedtable->start_speed == 0 && speedtable->object_speed > 0)	// acceleration speedtable
		{
			if (speeds >= MAX_SPEED_NUMS)
			{
				dev_err(dev, "too many speeds defined");
				return -EINVAL;
			}
			feature->speeds[speeds].speed = speedtable->object_speed;
			rampinfo->speeds[speeds].accel_table = speedtable;
			feature->speeds[speeds].accel_steps = speedtable->ramp_size / speedtable->stepping;
			++speeds;
		}
		else if (speedtable->start_speed > 0 && speedtable->object_speed > 0)	// speed-shift speedtable
		{
			if (shifts >= MAX_SPEED_NUMS)
			{
				dev_err(dev, "too many speed-shifts defined");
				return -EINVAL;
			}
			feature->speedshifts[shifts].speed1 = speedtable->start_speed;
			feature->speedshifts[shifts].speed2 = speedtable->object_speed;
			rampinfo->speedshifts[shifts].shift_table = speedtable;
			feature->speedshifts[shifts].steps = speedtable->ramp_size / speedtable->stepping;
			++shifts;
		}
	}
	feature->num_speed = rampinfo->num_speed = speeds;
	feature->num_speedshift = rampinfo->num_speedshift = shifts;

	// search for deceleration speedtable
	list_for_each_entry(speedtable, speedtable_list, list)
		if (speedtable->start_speed > 0 && speedtable->object_speed == 0)
		{
			int i;
			for (i = 0; i < feature->num_speed; i++) {
				if (speedtable->start_speed == feature->speeds[i].speed)
				{
					rampinfo->speeds[i].decel_table = speedtable;
					feature->speeds[i].decel_steps = speedtable->ramp_size / speedtable->stepping;
					break;
				}
			}
			if (i >= feature->num_speed)
			{
				dev_err(dev, "no matching Deceleration table of speed %d", speedtable->start_speed);
			}
		}

	return 0;
}
EXPORT_SYMBOL_GPL(steppermotor_speedtable_analysis);


static inline void time_to_count(int count, u32 *ramp_table, int clock)
{
	int i;
	for (i = 0; i < count; i++)
		ramp_table[i] /= clock;
}


/*
 * motor_ramptable_convert() - convert all time values from nanoseconds to ticks in a steppermotor's ramptable
 * @dev: device structure pointer of the steppermotor
 *
 * convert all time value to ticks in ramptable of a steppermotor
 */
int steppermotor_ramptable_convert(struct ramp_info *rampinfo, int clock)
{
	int i;

	if (!rampinfo || clock<=0)
		return -EINVAL;

	for (i = 0; i < rampinfo->num_speed; i++)
	{
		struct motor_speedtable *speedtable;

		speedtable = rampinfo->speeds[i].accel_table;
		time_to_count(speedtable->ramp_size, speedtable->ramp_table, clock);
		speedtable = rampinfo->speeds[i].decel_table;
		time_to_count(speedtable->ramp_size, speedtable->ramp_table, clock);
		rampinfo->speeds[i].step_ticks = speedtable->ramp_table[0];
	}
	for (i = 0; i < rampinfo->num_speedshift; i++)
	{
		struct motor_speedtable *speedtable;

		speedtable = rampinfo->speedshifts[i].shift_table;
		time_to_count(speedtable->ramp_size, speedtable->ramp_table, clock);
		rampinfo->speedshifts[i].step_ticks_1 = speedtable->ramp_table[0];
		rampinfo->speedshifts[i].step_ticks_2 = speedtable->ramp_table[speedtable->ramp_size];
	}

	return 0;
}
EXPORT_SYMBOL_GPL(steppermotor_ramptable_convert);


int steppermotor_check_config(struct steppermotor *motor, const struct steppermotor_config *config)
{
	struct speed_info *speedinfo;
	int i, ret, steps, index1, speed1;

	/* check total steps and number of speed configuration */
	if (config->steps_to_run <= 0 || config->steps_to_run > motor->feature.max_steps || config->num_speed <= 0) 
		return -EINVAL;

	/* check speed configuration */
	steps = 0;
	speedinfo = config->speedinfo;
	for (i=0; i<config->num_speed; i++)
	{
		if (!speedinfo)
			return -EINVAL;

		ret = lookup_speedtable(motor, speedinfo->speed);
		if (IS_ERR_VALUE(ret))	// requested speed is not supported
			return -EINVAL;

		steps += speedinfo->steps;
		speedinfo = speedinfo->nextspeed;
	}
	if (steps != config->steps_to_run)
		return -EINVAL;

	if (config->steps_to_run == 1)
		return 0;

	if (config->num_speed > 2)	// currently support only two-speed shift
		return -EINVAL;

	/* check with acceleration/deceleration steps */
	steps = config->steps_to_run;
	speedinfo = config->speedinfo;
	speed1 = speedinfo->speed;
	index1 = lookup_speedtable(motor, speed1);
	steps -= motor->feature.speeds[index1].accel_steps;
	if (steps <= 0)
		return -EINVAL;

	if (config->num_speed == 1)	// single-step configuration
	{
		steps -= motor->feature.speeds[index1].decel_steps;
		if (steps <= 0)
			return -EINVAL;
	}
	else
	{
		int index2, speed2;

		speedinfo = speedinfo->nextspeed;
		speed2 = speedinfo->speed;
		index2 = lookup_shifttable(motor, speed1, speed2);
		steps -= motor->feature.speedshifts[index2].steps;
		if (steps <= 0)
			return -EINVAL;

		index2 = lookup_speedtable(motor, speed2);
		steps -= motor->feature.speeds[index2].decel_steps;
		if (steps <= 0)
			return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(steppermotor_check_config);


struct steppermotor *steppermotor_get(struct platform_device *pdev)
{
	return ERR_PTR(-ENODEV);
}
EXPORT_SYMBOL_GPL(steppermotor_get);


struct steppermotor *of_steppermotor_get(struct device_node *np)
{
	return ERR_PTR(-ENODEV);
}
EXPORT_SYMBOL_GPL(of_steppermotor_get);


struct steppermotor *of_node_to_steppermotor(struct device_node *np)
{
	struct steppermotor *motor;

	mutex_lock(&steppermotor_mutex);

	list_for_each_entry(motor, &steppermotor_list, list)
		if (motor->dev && motor->dev->of_node == np) {
			mutex_unlock(&steppermotor_mutex);
			return motor;
		}

	mutex_unlock(&steppermotor_mutex);

	return ERR_PTR(-EPROBE_DEFER);
}
EXPORT_SYMBOL_GPL(of_node_to_steppermotor);


void steppermotor_put(struct steppermotor *motor)
{
	if (!motor) return;
	motor->using = 0;
}
EXPORT_SYMBOL_GPL(steppermotor_put);


int steppermotor_set_callback(struct steppermotor *motor, void (*callback)(struct steppermotor *, struct callback_data *),
			      struct callback_data *data)
{
	if (!motor || !callback)
		return -EINVAL;
	motor->callback = callback;
	motor->callbackdata = *data;
	return 0;
}
EXPORT_SYMBOL_GPL(steppermotor_set_callback);


int steppermotor_start(struct steppermotor *motor)
{
	if (motor)
		return motor->ops->start(motor);

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(steppermotor_start);


void steppermotor_stop(struct steppermotor *motor)
{
	if (motor)
		motor->ops->stop(motor);
}
EXPORT_SYMBOL_GPL(steppermotor_stop);


void steppermotor_emergencybrake(struct steppermotor *motor)
{
	if (motor)
		motor->ops->emergencybrake(motor);
}
EXPORT_SYMBOL_GPL(steppermotor_emergencybrake);


void steppermotor_lock(struct steppermotor *motor)
{
	if (motor && motor->ops->lock)
		motor->ops->lock(motor);
}
EXPORT_SYMBOL_GPL(steppermotor_lock);


void steppermotor_unlock(struct steppermotor *motor)
{
	if (motor && motor->ops->unlock)
		motor->ops->unlock(motor);
}
EXPORT_SYMBOL_GPL(steppermotor_unlock);


int steppermotor_status(struct steppermotor *motor)
{
	if (!motor)
		return -EINVAL;
	return motor->status;
}
EXPORT_SYMBOL_GPL(steppermotor_status);


int steppermotor_get_running_steps(struct steppermotor *motor)
{
	if (motor && motor->ops->get_running_steps)
		return motor->ops->get_running_steps(motor);

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(steppermotor_get_running_steps);


int steppermotor_get_medialength_in_steps(struct steppermotor *motor)
{
	if (motor && motor->ops->get_medialength_in_steps)
		return motor->ops->get_medialength_in_steps(motor);

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(steppermotor_get_medialength_in_steps);


int steppermotor_get_config(struct steppermotor *motor, struct steppermotor_config *config)
{
	if (!motor || !config)
		return -EINVAL;
	*config = motor->config;
	return 0;
}
EXPORT_SYMBOL_GPL(steppermotor_get_config);


int steppermotor_set_config(struct steppermotor *motor, const struct steppermotor_config *config)
{
	if (!motor || !config)
		return -EINVAL;
	motor->config = *config;
	return motor->ops->config(motor, config);
}
EXPORT_SYMBOL_GPL(steppermotor_set_config);


int steppermotor_get_sensor_sel_mask(struct steppermotor *motor, int *sensel)
{
	if (!motor || !sensel)
		return -EINVAL;

//	if (motor->ops->get_sensor_sel_mask)
//		return motor->ops->get_sensor_sel_mask(motor, sensel);

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(steppermotor_get_sensor_sel_mask);


int steppermotor_set_sensor_sel_mask(struct steppermotor *motor, const int sensel)
{
	if (motor && motor->ops->set_sensor_sel_mask)
		return motor->ops->set_sensor_sel_mask(motor, sensel);

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(steppermotor_set_sensor_sel_mask);


int steppermotor_set_trigger_next(struct steppermotor *motor, const struct steppermotor_trigger *trigger)
{
	if (motor && motor->ops->set_trigger_next)
		return motor->ops->set_trigger_next(motor, trigger);

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(steppermotor_set_trigger_next);


int steppermotor_set_skew_steps(struct steppermotor *motor, const int steps)
{
	if (motor && motor->ops->set_skew_steps)
		return motor->ops->set_skew_steps(motor, steps);

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(steppermotor_set_skew_steps);


MODULE_AUTHOR("Zhang Xudong <zhangxudong@gwi.com.cn>");
MODULE_DESCRIPTION("generic steppermotor driver functions implementation"); 
MODULE_LICENSE("GPL v2");
