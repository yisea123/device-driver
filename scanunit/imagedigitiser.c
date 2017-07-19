/*
 * general image digitiser/Analog Front End (AFE) functions implementation
 *
 * Copyright 2016 Hunan GreatWall Information Financial Equipment Co., Ltd.
 *
 * Licensed under the GPL-2.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/of.h>
#include <linux/of_device.h>


#include "imagedigitiser.h"


static DEFINE_MUTEX(imagedigitiser_lock);
static LIST_HEAD(imagedigitiser_list);


struct imagedigitiser *of_imagedigitiser_get(struct device_node *np)
{
	return ERR_PTR(-ENODEV);
}
EXPORT_SYMBOL_GPL(of_imagedigitiser_get);


struct imagedigitiser *imagedigitiser_get(struct device *dev)
{
	return ERR_PTR(-ENODEV);
}
EXPORT_SYMBOL_GPL(imagedigitiser_get);


struct imagedigitiser *of_node_to_imagedigitiser(struct device_node *np)
{
	struct imagedigitiser *afe;

	mutex_lock(&imagedigitiser_lock);

	list_for_each_entry(afe, &imagedigitiser_list, list)
		if (afe->dev && afe->dev->of_node == np) {
			mutex_unlock(&imagedigitiser_lock);
			afe->using = 1;
			return afe;
		}

	mutex_unlock(&imagedigitiser_lock);

	return ERR_PTR(-EPROBE_DEFER);
}
EXPORT_SYMBOL_GPL(of_node_to_imagedigitiser);


void imagedigitiser_put(struct imagedigitiser *afe)
{
	if (!afe) return;
	afe->using = 0;
}
EXPORT_SYMBOL_GPL(imagedigitiser_put);


int imagedigitiser_enable(struct imagedigitiser *afe)
{
	if (afe)
		return afe->enable(afe);

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(imagedigitiser_enable);


void imagedigitiser_disable(struct imagedigitiser *afe)
{
	if (afe)
		afe->disable(afe);
}
EXPORT_SYMBOL_GPL(imagedigitiser_disable);


int imagedigitiser_get_config(struct imagedigitiser *afe, struct scanunit_config *config)
{
	if (afe)
		return afe->get_config(afe, config);

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(imagedigitiser_get_config);


int imagedigitiser_set_config(struct imagedigitiser *afe, const struct scanunit_config *config)
{
	if (afe)
		return afe->set_config(afe, config);

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(imagedigitiser_set_config);


/**
 * imagedigitiser_add() - register a new image digitiser
 * @afe: the image digitiser (AFE) to add
 *
 * Register a new image digitiser.
 */
int imagedigitiser_add(struct imagedigitiser *afe)
{
	if (!afe || !afe->dev || !afe->get_config || !afe->set_config ||
	    !afe->enable || !afe->disable)
		return -EINVAL;

	mutex_lock(&imagedigitiser_lock);

	INIT_LIST_HEAD(&afe->list);
	list_add(&afe->list, &imagedigitiser_list);

	mutex_unlock(&imagedigitiser_lock);
	return 0;
}
EXPORT_SYMBOL_GPL(imagedigitiser_add);


/**
 * imagedigitiser_remove() - remove a image digitiser
 * @afe: the image digitiser (AFE) to remove
 *
 * Removes a image digitiser. This function may return busy
 * if the image digitiser is still requested.
 */
int imagedigitiser_remove(struct imagedigitiser *afe)
{
	if (afe->using)
		return -EBUSY;

	mutex_lock(&imagedigitiser_lock);

	list_del_init(&afe->list);

	mutex_unlock(&imagedigitiser_lock);
	return 0;
}
EXPORT_SYMBOL_GPL(imagedigitiser_remove);

MODULE_AUTHOR("Zhang Xudong <zhangxudong@gwi.com.cn>");
MODULE_DESCRIPTION("Image Digitiser functions implementation"); 
MODULE_LICENSE("GPL v2");
