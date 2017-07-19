/*
 * general image sensor functions implementation
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
#include <linux/list.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_device.h>


#include "imagesensor.h"


static DEFINE_MUTEX(imagesensor_lock);
static LIST_HEAD(imagesensor_list);


struct imagesensor *of_imagesensor_get(struct device_node *np)
{
	return ERR_PTR(-ENODEV);
}
EXPORT_SYMBOL_GPL(of_imagesensor_get);


struct imagesensor *imagesensor_get(struct device *dev)
{
	return ERR_PTR(-ENODEV);
}
EXPORT_SYMBOL_GPL(imagesensor_get);


struct imagesensor *of_node_to_imagesensor(struct device_node *np)
{
	struct imagesensor *sensor;

	mutex_lock(&imagesensor_lock);

	list_for_each_entry(sensor, &imagesensor_list, list)
		if (sensor->dev && sensor->dev->of_node == np) {
			mutex_unlock(&imagesensor_lock);
			sensor->using = 1;
			return sensor;
		}

	mutex_unlock(&imagesensor_lock);

	return ERR_PTR(-EPROBE_DEFER);
}
EXPORT_SYMBOL_GPL(of_node_to_imagesensor);


void imagesensor_put(struct imagesensor *sensor)
{
	if (!sensor) return;
	sensor->using = 0;
}
EXPORT_SYMBOL_GPL(imagesensor_put);


int imagesensor_enable(struct imagesensor *sensor)
{
	if (sensor)
		return sensor->enable(sensor);

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(imagesensor_enable);


void imagesensor_disable(struct imagesensor *sensor)
{
	if (sensor)
		sensor->disable(sensor);
}
EXPORT_SYMBOL_GPL(imagesensor_disable);


int imagesensor_get_config(struct imagesensor *sensor, struct scanunit_config *config)
{
	if (sensor)
		return sensor->get_config(sensor, config);

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(imagesensor_get_config);


int imagesensor_set_config(struct imagesensor *sensor, const struct scanunit_config *config)
{
	if (sensor)
		return sensor->set_config(sensor, config);

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(imagesensor_set_config);


/**
 * imagesensor_add() - register a new image sensor
 * @afe: the image sensor to add
 *
 * Register a new image sensor.
 */
int imagesensor_add(struct imagesensor *sensor)
{
	if (!sensor || !sensor->dev || !sensor->get_config || !sensor->set_config ||
	    !sensor->enable || !sensor->disable)
		return -EINVAL;

	mutex_lock(&imagesensor_lock);

	INIT_LIST_HEAD(&sensor->list);
	list_add(&sensor->list, &imagesensor_list);

	mutex_unlock(&imagesensor_lock);
	return 0;
}
EXPORT_SYMBOL_GPL(imagesensor_add);


/**
 * imagesensor_remove() - remove a image sensor
 * @afe: the image sensor to remove
 *
 * Removes a image sensor. This function may return busy if the image sensor is
 * still requested.
 */
int imagesensor_remove(struct imagesensor *sensor)
{
	if (sensor->using)
		return -EBUSY;

	mutex_lock(&imagesensor_lock);

	list_del_init(&sensor->list);

	mutex_unlock(&imagesensor_lock);
	return 0;
}
EXPORT_SYMBOL_GPL(imagesensor_remove);

MODULE_AUTHOR("Zhang Xudong <zhangxudong@gwi.com.cn>");
MODULE_DESCRIPTION("Image Sensor functions implementation"); 
MODULE_LICENSE("GPL v2");
