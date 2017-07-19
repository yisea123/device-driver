/*
 * A simple sysfs interface for the generic steppermotor and DC-motor framework
 *
 * Copyright 2017 Hunan GreatWall Information Financial Equipment Co., Ltd.
 *
 * Licensed under the GPL-2.
 */
#include <linux/module.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/kdev_t.h>
#include <linux/of.h>
#include <linux/of_device.h>


#include "steppermotor.h"
#include "dcmotor.h"


static DEFINE_MUTEX(export_mutex);
static LIST_HEAD(motor_export_list);


struct motor_export {
	struct list_head list;
	struct device *dev;
	motor_type type;
	void *motor;
	int direction;
	int speed1, speed2;
	int steps1, steps2;
};


static ssize_t motor_steps1_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	struct motor_export *export = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", export->steps1); 
}

static ssize_t motor_steps1_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct motor_export *export = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		return ret;
	if (val < 0)
		return -EINVAL;

	export->steps1 = val;
	return ret ? : size;
}


static ssize_t motor_steps2_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	struct motor_export *export = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", export->steps2);
}

static ssize_t motor_steps2_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct motor_export *export = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		return ret;
	if (val < 0)
		return -EINVAL;

	export->steps2 = val;
	return ret ? : size;
}

static ssize_t motor_speed1_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct motor_export *export = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", export->speed1);
}

static ssize_t motor_speed1_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t size)
{
	struct motor_export *export = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		return ret;
	if (val < 0)
		return -EINVAL;

	export->speed1 = val;
	return ret ? : size;
}


static ssize_t motor_speed2_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct motor_export *export = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", export->speed2);
}

static ssize_t motor_speed2_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t size)
{
	struct motor_export *export = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		return ret;
	if (val < 0)
		return -EINVAL;

	export->speed2 = val;
	return ret ? : size;
}

static ssize_t motor_enable_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	struct motor_export *export = dev_get_drvdata(dev);
	int enabled;

	if (export->type == STEPPERMOTOR)
		enabled = steppermotor_is_running(steppermotor_status((struct steppermotor *)export->motor));
	else
		enabled = dcmotor_is_running(dcmotor_status((struct dcmotor *)export->motor));

	return sprintf(buf, "%d\n", enabled);
}

static ssize_t motor_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct motor_export *export = dev_get_drvdata(dev);
	int val, ret;
	int enabled;
	int (*motor_start)(void *motor);
	int (*motor_stop)(void *motor);

	ret = kstrtoint(buf, 0, &val);
	if (ret)
		return ret;

	if (export->type == STEPPERMOTOR)
	{
		enabled = steppermotor_is_running(steppermotor_status((struct steppermotor *)export->motor));
		motor_start = (int (*)(void *))steppermotor_start;
		motor_stop = (int (*)(void *))steppermotor_stop;
	}
	else
	{
		enabled = dcmotor_is_running(dcmotor_status((struct dcmotor *)export->motor));
		motor_start = (int (*)(void *))dcmotor_start;
		motor_stop = (int (*)(void *))dcmotor_stop;
	}

	switch (val) {
	case 0:
		ret = (*motor_stop)(export->motor);	// stop motor forcely
		break;
	case 1:
		if (!enabled)
			ret = (*motor_start)(export->motor);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret ? : size;
}

static ssize_t motor_direction_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct motor_export *export = dev_get_drvdata(dev);

	return sprintf(buf, "%s\n", export->direction == MOTION_CLOCKWISE ? "clockwise" : "counter-clockwise");
}

static ssize_t motor_direction_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t size)
{
	struct motor_export *export = dev_get_drvdata(dev);

	if (sysfs_streq(buf, "clockwise"))
		export->direction = MOTION_CLOCKWISE; 
	else if (sysfs_streq(buf, "counter-clockwise"))
		export->direction = MOTION_COUNTERCLOCKWISE; 
	else
		return -EINVAL;

	return size;
}

static ssize_t motor_status_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	struct motor_export *export = dev_get_drvdata(dev);
	int status;

	if (export->type == STEPPERMOTOR)
		status = steppermotor_status((struct steppermotor *)export->motor);
	else
		status = dcmotor_status((struct dcmotor *)export->motor);

	return sprintf(buf, "%08x\n", status);
}


static ssize_t motor_feature_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	struct motor_export *export = dev_get_drvdata(dev);
	char *ptr = buf;
	int ret;

	if (export->type == STEPPERMOTOR)
	{
		int i;
		struct steppermotor *motor = (struct steppermotor *)export->motor;

		ret = sprintf(ptr, "maximum steps = %d\n", motor->feature.max_steps);
		ptr += ret;
		ret = sprintf(ptr, "pull-in speed = %d SPS\n", motor->feature.pullin_speed);
		ptr += ret;
		ret = sprintf(ptr, "supports %d speeds:\n", motor->feature.num_speed);
		ptr += ret;
		for (i = 0; i < motor->feature.num_speed; i++)
		{
			ret = sprintf(ptr, "%d SPS, accel_steps = %d, decel_steps = %d\n", motor->feature.speeds[i].speed,
				motor->feature.speeds[i].accel_steps, motor->feature.speeds[i].decel_steps);
			ptr += ret;
		}
		ret = sprintf(ptr, "supports %d speed-shifts:\n", motor->feature.num_speedshift);
		ptr += ret;
		for (i = 0; i < motor->feature.num_speedshift; i++)
		{
			ret = sprintf(ptr, "%d -> %d SPS, shift-steps = %d\n", motor->feature.speedshifts[i].speed1,
				motor->feature.speedshifts[i].speed2, motor->feature.speedshifts[i].steps);
			ptr += ret;
		}
		ptr += ret;
	}

	return (int)(ptr-buf);
}


static ssize_t motor_config_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t size)
{
	struct motor_export *export = dev_get_drvdata(dev);
	int ret;

	if (export->type == STEPPERMOTOR)
	{
		struct steppermotor_config config;
		struct speed_info speedinfos[2];

		if (export->steps1 <= 0 || export->speed1 < 0)
			return -EINVAL;

		memset((void *)&config, 0, sizeof(config));
		memset((void *)&speedinfos, 0, sizeof(speedinfos));

		config.dir = export->direction;
		config.num_speed = 1;
		config.steps_to_run = export->steps1;
		config.speedinfo = &speedinfos[0];
		speedinfos[0].speed = export->speed1;
		speedinfos[0].steps = export->steps1;
		if (export->steps2 > 0 && export->speed2 > 0) {
			config.num_speed += 1;
			config.steps_to_run += export->steps2;
			speedinfos[0].nextspeed = &speedinfos[1]; 
			speedinfos[1].speed = export->speed2;
			speedinfos[1].steps = export->steps2;
		}
		ret = steppermotor_set_config((struct steppermotor *)export->motor, &config);
		if (IS_ERR_VALUE(ret))
			return ret;
	}
	else
	{
		struct dcmotor_config config;

		memset((void *)&config, 0, sizeof(config));

		config.dir = export->direction;
		ret = dcmotor_set_config((struct dcmotor *)export->motor, &config);
		if (IS_ERR_VALUE(ret))
			return ret;
	}
	return size;
}

static DEVICE_ATTR(enable, 0644, motor_enable_show, motor_enable_store);
static DEVICE_ATTR(config, 0644, NULL, motor_config_store);
static DEVICE_ATTR(status, 0644, motor_status_show, NULL);
static DEVICE_ATTR(feature, 0644, motor_feature_show, NULL);
static DEVICE_ATTR(direction, 0644, motor_direction_show, motor_direction_store);
static DEVICE_ATTR(steps1, 0644, motor_steps1_show, motor_steps1_store);
static DEVICE_ATTR(steps2, 0644, motor_steps2_show, motor_steps2_store);
static DEVICE_ATTR(speed1, 0644, motor_speed1_show, motor_speed1_store);
static DEVICE_ATTR(speed2, 0644, motor_speed2_show, motor_speed2_store);

static struct attribute *steppermotor_attrs[] = {
	&dev_attr_enable.attr,
	&dev_attr_config.attr,
	&dev_attr_status.attr,
	&dev_attr_feature.attr,
	&dev_attr_direction.attr,
	&dev_attr_steps1.attr,
	&dev_attr_steps2.attr,
	&dev_attr_speed1.attr,
	&dev_attr_speed2.attr,
	NULL
};
ATTRIBUTE_GROUPS(steppermotor);


static struct attribute *dcmotor_attrs[] = {
	&dev_attr_enable.attr,
	&dev_attr_config.attr,
	&dev_attr_status.attr,
	&dev_attr_direction.attr,
	NULL
};
ATTRIBUTE_GROUPS(dcmotor);


static struct class motor_class = {
	.name		= "motor",
	.owner		= THIS_MODULE,
	.dev_groups	= steppermotor_groups,
};

static int motor_sysfs_match(struct device *dev, const void *data)
{
	struct motor_export *export = dev_get_drvdata(dev);
	return export->motor == data;
}

static int motor_sysfs_export(motor_type type, void *motor)
{
	struct motor_export *export;
	struct device *dev, *motordev;

	export = kzalloc(sizeof(*export), GFP_KERNEL);
	if (IS_ERR(export))
		return -ENOMEM;

	export->motor = motor;
	export->type = type;
	dev = export->dev;

	if (type == STEPPERMOTOR)
	{
		motordev = ((struct steppermotor *)motor)->dev;
		motor_class.dev_groups = steppermotor_groups;
	}
	else
	{
		motordev = ((struct dcmotor *)motor)->dev;
		motor_class.dev_groups = dcmotor_groups;
	}
	dev = device_create(&motor_class, NULL, MKDEV(0, 0), export, "motor%d", motordev->of_node->phandle); 

	if (IS_ERR(dev)) {
		dev_warn(motordev, "device_create failed for motor sysfs export\n");
		return -ENODEV;
	}
	export->dev = dev;

	mutex_lock(&export_mutex); 
	INIT_LIST_HEAD(&export->list);
	list_add_tail(&export->list, &motor_export_list);
	mutex_unlock(&export_mutex);

	return 0;
}


static void motor_sysfs_unexport(void *motor)
{
	struct motor_export *export;
	struct device *dev;

	dev = class_find_device(&motor_class, NULL, motor, motor_sysfs_match);
	if (IS_ERR(dev))
		return;

	/* for class_find_device() */
	export = dev_get_drvdata(dev);

	mutex_lock(&export_mutex); 
	list_del(&export->list);
	mutex_unlock(&export_mutex);

	kfree(export);

	put_device(dev);
	device_unregister(dev);
}


static int __init motor_sysfs_init(void)
{
	struct device_node *np, *child;
	int status;

	status = class_register(&motor_class);
	if (status < 0)
		return status;

	np = of_find_node_by_path("/motors");
	if (!np) {
		pr_err("no motors found in devicetree");
		return -ENODEV; 
	}
	for_each_child_of_node(np, child)
	{
		struct steppermotor *motor;
		motor = of_node_to_steppermotor(child);
		if (!IS_ERR(motor))
		{
			struct device *dev = motor->dev;
			printk("export steppermotor: %s.\n", dev->of_node->full_name); 
			motor_sysfs_export(STEPPERMOTOR, motor); 
		}
	}
	for_each_child_of_node(np, child)
	{
		struct dcmotor *motor;
		motor = of_node_to_dcmotor(child);
		if (!IS_ERR(motor))
		{
			struct device *dev = motor->dev;
			printk("export dcmotor: %s.\n", dev->of_node->full_name); 
			motor_sysfs_export(BRUSH_DCMOTOR, motor);
		}
	}

	return 0;
}
module_init(motor_sysfs_init);

static void __exit motor_sysfs_exit(void)
{
	struct motor_export *export, *temp;

	// search for any motor exported
	list_for_each_entry_safe(export, temp, &motor_export_list, list)
		motor_sysfs_unexport(export->motor);

	class_unregister(&motor_class);
}
module_exit(motor_sysfs_exit);


MODULE_AUTHOR("Zhang Xudong <zhangxudong@gwi.com.cn>");
MODULE_DESCRIPTION("sysfs interface for the generic steppermotor and DC-motor framework"); 
MODULE_LICENSE("GPL v2");
