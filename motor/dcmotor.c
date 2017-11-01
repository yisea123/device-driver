/*
 * generic DC-motor driver functions implementation
 *
 * Copyright 2017 Hunan GreatWall Information Financial Equipment Co., Ltd.
 *
 * Licensed under the GPL-2.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_device.h>


#include "dcmotor.h"


static DEFINE_MUTEX(dcmotor_mutex);
static LIST_HEAD(dcmotor_list);


/**
 * dcmotor_add() - register a new dcmotor
 * @motor: the dcmotor to add
 *
 * Register a new dcmotor.
 */
int dcmotor_add(struct dcmotor *motor)
{
	if (!motor || !motor->dev || !motor->ops)
		return -EINVAL;

	/* check basic operation implemention */
	if (!motor->ops->owner || !motor->ops->config || !motor->ops->start || !motor->ops->stop) 
		return -EINVAL;

	mutex_lock(&dcmotor_mutex);

	INIT_LIST_HEAD(&motor->list);
	list_add(&motor->list, &dcmotor_list);

	mutex_unlock(&dcmotor_mutex);
	return 0;
}
EXPORT_SYMBOL_GPL(dcmotor_add);


/**
 * dcmotor_remove() - remove a dcmotor
 * @motor: the dcmotor to remove
 *
 * Removes a dcmotor. This function may return busy if the dcmotor is still
 * requested.
 */
int dcmotor_remove(struct dcmotor *motor)
{
	if (motor->using)
		return -EBUSY;

	mutex_lock(&dcmotor_mutex);

	list_del_init(&motor->list);

	mutex_unlock(&dcmotor_mutex);
	return 0;
}
EXPORT_SYMBOL_GPL(dcmotor_remove);


struct dcmotor *dcmotor_get(struct device *dev)
{
	return ERR_PTR(-ENODEV);
}
EXPORT_SYMBOL_GPL(dcmotor_get);


struct dcmotor *of_dcmotor_get(struct device_node *np)
{
	return ERR_PTR(-ENODEV);
}
EXPORT_SYMBOL_GPL(of_dcmotor_get);


struct dcmotor *of_node_to_dcmotor(struct device_node *np)
{
	struct dcmotor *motor;

	mutex_lock(&dcmotor_mutex);

	list_for_each_entry(motor, &dcmotor_list, list)
		if (motor->dev && motor->dev->of_node == np) {
			mutex_unlock(&dcmotor_mutex);
			return motor;
		}

	mutex_unlock(&dcmotor_mutex);

	return ERR_PTR(-EPROBE_DEFER);
}
EXPORT_SYMBOL_GPL(of_node_to_dcmotor);


void dcmotor_put(struct dcmotor *motor)
{
	if (!motor) return;
	motor->using = 0;
}
EXPORT_SYMBOL_GPL(dcmotor_put);


int dcmotor_set_callback(struct dcmotor *motor, void (*callback)(struct dcmotor *, struct callback_data *),
			struct callback_data *data)
{
	if (!motor)
		return -EINVAL;
	motor->callback = callback;
	motor->callbackdata = *data;
	return 0;
}
EXPORT_SYMBOL_GPL(dcmotor_set_callback);


int dcmotor_start(struct dcmotor *motor)
{
	if (motor)
		return motor->ops->start(motor);

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(dcmotor_start);


void dcmotor_stop(struct dcmotor *motor)
{
	if (motor)
		return motor->ops->stop(motor);
}
EXPORT_SYMBOL_GPL(dcmotor_stop);


int dcmotor_status(struct dcmotor *motor)
{
	if (!motor)
		return -EINVAL;
	return motor->status;
}
EXPORT_SYMBOL_GPL(dcmotor_status);


int dcmotor_get_config(struct dcmotor *motor, struct dcmotor_config *config)
{
	if (!motor || !config)
		return -EINVAL;
	*config = motor->config;
	return 0;
}
EXPORT_SYMBOL_GPL(dcmotor_get_config);


int dcmotor_set_config(struct dcmotor *motor, const struct dcmotor_config *config)
{
	if (!motor || !config)
		return -EINVAL;
	motor->config = *config;
	return motor->ops->config(motor, config);
}
EXPORT_SYMBOL_GPL(dcmotor_set_config);


MODULE_AUTHOR("Zhang Xudong <zhangxudong@gwi.com.cn>");
MODULE_DESCRIPTION("generic DC-motor driver functions implementation"); 
MODULE_LICENSE("GPL v2");
