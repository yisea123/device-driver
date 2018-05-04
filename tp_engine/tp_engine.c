/*
 * GWI Thermal Printer Printer Engine Driver.
 *
 * Copyright 2018 Hunan GreatWall Information Financial Equipment Co., Ltd.
 *
 * Licensed under the GPL-2.
*/
#include <linux/device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/sysfs.h>
#include <asm/uaccess.h>
#include <asm/signal.h>
#include <asm/siginfo.h>
#include <linux/platform_device.h>
#include <linux/completion.h>
#include <linux/slab.h>

#include "tp_engine_ph.h"
#include "tp_engine_motor.h"
#include "tp_engine_sensor.h"
#include "tp_engine.h"

#define TP_ENGINE_DEV_MAJOR		0
#define TP_ENGINE_DEV_INDEX		0

static int tp_engine_dev_major = TP_ENGINE_DEV_MAJOR;

struct tp_engine_dev_t
{
	struct cdev cdev;
	struct device *dev;
	struct tp_engine_t tp_engine;
};

/********************************************************
 * LOCAL FUNCTIONS
 ********************************************************/
/* 从设备树获取设备并关联到数据结构 */
static int tp_engine_devicetree_ph_parse(struct device_node *pnode, struct ph_data_t *pph_data, unsigned int ph_num)
{
	struct device_node * pp;
	const char * name;
	unsigned int data;
	int ret = 0;
	
	if(ph_num == 0)
		return 0;
	for_each_child_of_node(pnode, pp)
	{
		ret = of_property_read_u32(pp, "ph_dev", &data);
		if(ret)
		{
			printk(KERN_ERR "ERROR %s %d\n", __FUNCTION__, __LINE__);
			return ret;
		}
		//pph_data->ptp_ph = data;
		ret = of_property_read_string(pp, "ph_name", &name);
		if(ret)
		{
			printk(KERN_ERR "ERROR %s %d\n", __FUNCTION__, __LINE__);
			return ret;
		}
		strcpy(pph_data->ph_name, name);
		ret = of_property_read_u32(pp, "ph_mask", &data);
		if(ret)
		{
			printk(KERN_ERR "ERROR %s %d\n", __FUNCTION__, __LINE__);
			return ret;
		}
		pph_data->ph_mask = data;
	}
	return ret;
}

/* 从设备树获取设备并关联到数据结构 */
static int tp_engine_devicetree_ph_resistor_parse(struct device_node *pnode, struct ph_resistor_data_t *ph_resistor_data, unsigned int ph_resistor_num)
{
	int ret = 0;
	return ret;
}

/* 从设备树获取设备并关联到数据结构 */
static int tp_engine_devicetree_motor_parse(struct device_node *pnode, struct motor_data_t *pmotor_data, unsigned int motor_num)
{
	int ret = 0;
	struct device_node * pp, * pmotor_dev;
	const char * pname, *pmotor_type;
	unsigned int data, i;
	
	printk(KERN_DEBUG "NOTICE %s ==> %d\n", __FUNCTION__, __LINE__);
	if(motor_num == 0)
		return 0;
	for_each_child_of_node(pnode, pp)
	{
		ret = of_property_read_string(pp, "motor_name", &pname);
		if(ret)
		{
			printk(KERN_ERR "ERROR %s ==> %d\n", __FUNCTION__, __LINE__);
			return ret;
		}
		strcpy(pmotor_data->motor_name, pname);
		ret = of_property_read_string(pp, "motor_type", &pmotor_type);
		if (ret)
		{
			printk(KERN_ERR "mechunit_probe_get_motor:Failed to read type of motor!\n");
			return ret;
		}
		ret = of_property_read_u32(pp,"motor_mask",&data);
		if (ret)
		{
			printk(KERN_ERR "ERROR %s ==> %d\n", __FUNCTION__, __LINE__);
			return ret;
		}
		pmotor_data->motor_mask = data;
		ret = of_property_read_u32(pp,"motor_dev",&data);
		if (ret)
		{
			printk(KERN_ERR "ERROR %s ==> %d\n", __FUNCTION__, __LINE__);
			return ret;
		}
		pmotor_dev = of_find_node_by_phandle(data);
		if (pmotor_dev == NULL)
		{
			printk(KERN_ERR "ERROR %s ==> %d\n", __FUNCTION__, __LINE__);
			return -1;
		}
		if (!strcmp(pmotor_type, "stepper_motor"))
		{
			pmotor_data->motor_type = MOTOR_TYPE_STEP;
			pmotor_data->motor_dev.pstepmotor = of_node_to_steppermotor(pmotor_dev); 
			if (pmotor_data->motor_dev.pstepmotor == ERR_PTR(-EPROBE_DEFER))
			{
				printk(KERN_ERR "ERROR %s ==> %d\n", __FUNCTION__, __LINE__);
				return -1;
			}
			else 
				printk(KERN_DEBUG "DEBUG %s ==> %d\n", __FUNCTION__, __LINE__);

		}
		else if (!strcmp(pmotor_type, "brdc_motor"))
		{
			pmotor_data->motor_type = MOTOR_TYPE_BRUSHDC;
			pmotor_data->motor_dev.pdcmotor = of_node_to_dcmotor(pmotor_dev); 
			if (pmotor_data->motor_dev.pdcmotor == ERR_PTR(-EPROBE_DEFER))
			{
				printk(KERN_DEBUG "DEBUG %s ==> %d\n", __FUNCTION__, __LINE__);
				return -1;
			}
			else
			{
				printk(KERN_DEBUG "DEBUG %s ==> %d\n", __FUNCTION__, __LINE__);
			}
		}
		printk(KERN_DEBUG "motor: name = %s, mask = %d\n", pmotor_data->motor_name, pmotor_data->motor_mask);
		i++;
		if(i == motor_num)
			break;
		else
			pmotor_data += 1;
	}
	return ret;
}

/* 从设备树获取设备并关联到数据结构 */
static int tp_engine_devicetree_sensor_parse(struct device_node *pnode, 
					     struct sensor_data_t *psensor_data, unsigned int sensor_num)
{
	struct device_node *pp, *psen_dev;
	const char *p_name;
	unsigned int data;
	int ret = 0;
	unsigned char i = 0;

	printk(KERN_DEBUG "%s start ==> %d", __FUNCTION__, __LINE__);
	if (!sensor_num)
	{
		return 0;
	}
	for_each_child_of_node(pnode, pp)
	{
		ret = of_property_read_string(pp, "sen_name", &p_name);
		if (ret)
		{
			printk(KERN_ERR "ERROR %s ==> %d\n", __FUNCTION__, __LINE__);
			return ret;
		}
		strcpy(psensor_data->sen_name, p_name);
		ret = of_property_read_u32(pp, "sen_mask", &data);
		if (ret)
		{
			printk(KERN_ERR "ERROR %s ==> %d\n", __FUNCTION__, __LINE__);
			return ret;
		}
		psensor_data->sen_mask = data;
		ret = of_property_read_u32(pp, "sen_dev", &data);
		if (ret)
		{
			printk(KERN_ERR "ERROR %s ==> %d\n", __FUNCTION__, __LINE__);
			return ret;
		}
		psen_dev = of_find_node_by_phandle(data);
		if(psen_dev==NULL)
		{
			printk(KERN_ERR "ERROR in Func %s of_find_node_by_phandle\n", __FUNCTION__);
			return ret;
		}
		psensor_data->sen_dev.pphotosensor = of_node_to_photosensor(psen_dev);
		if (psensor_data->sen_dev.pphotosensor == ERR_PTR(-EPROBE_DEFER))
		{
			printk(KERN_ERR "ERROR in Func %s of_node_to_photosensor\n", __FUNCTION__);
			return ret;
		}
		psensor_data->sen_type = psensor_data->sen_dev.pphotosensor->type;
		printk(KERN_DEBUG "DEBUG in Func %s name = %s, mask = %x pphotosensor=%x type=%x\n", __FUNCTION__,
		       psensor_data->sen_name, psensor_data->sen_mask, (unsigned int)psensor_data->sen_dev.pphotosensor,
		       psensor_data->sen_type);
		i++;
		if (i == sensor_num)
		{
			break;
		}
		else
		{
			psensor_data += 1;
		}
	}
	return ret;
}
 
static int tp_engine_devicetree_parse(struct device * pdev, struct tp_engine_t * ptp_engine)
{
	struct device_node *node, *psub_node;
	int ret = 0;
	unsigned int data[8];
	const char *pname;
	
	printk(KERN_DEBUG "tp_engine device tree parse.\n");
	node = pdev->of_node;
	if(!node)
	{
		return (-ENODEV);
	}
	if(!of_find_property(node, "tp_engine_name", NULL))
	{
		dev_warn(pdev, "Found engine without tp_engine_name");
		return (-EINVAL);
	}
	memset(ptp_engine->tp_engine_name, 0, TP_ENGINE_NAME_MAX_SIZE);
	ret = of_property_read_string(node, "tp_engine_name", &pname);
	strcpy(ptp_engine->tp_engine_name, pname);
	printk(KERN_DEBUG "tp_engine_name = %s, ret = %d\n", ptp_engine->tp_engine_name, ret);
	//读取各个组件数量与设备，申请空间，本项目中有打印头，热敏电阻，电机，传感器
	if(!of_find_property(node, "tp_engine_component", NULL));
	{
		dev_warn(pdev, "Found tp_engine_component without tp_engine_component\n");
		return (-EINVAL);
	}
	of_property_read_u32_array(node, "tp_engine_component", data, ARRAY_SIZE(data));
	ptp_engine->ph_num = data[1];
	ptp_engine->pph_data = devm_kzalloc(pdev, sizeof(struct ph_data_t) * ptp_engine->ph_num, GFP_KERNEL);
	if(ptp_engine->pph_data == NULL)
	{
		printk(KERN_ERR "ph_data devm_kzalloc error!\n");
		return -ENOMEM;
	}
	ptp_engine->ph_resistor_num = data[3];
	ptp_engine->pph_resistor_data = devm_kzalloc(pdev, sizeof(struct ph_resistor_data_t) * ptp_engine->ph_resistor_num, GFP_KERNEL);
	if(ptp_engine->pph_resistor_data == NULL)
	{
		printk(KERN_ERR "pph_resistor_data devm_kzalloc error!\n");
		return -ENOMEM;
	}
	ptp_engine->motor_num = data[5];
	ptp_engine->pmotor_data = devm_kzalloc(pdev, sizeof(struct motor_data_t) * ptp_engine->motor_num, GFP_KERNEL);
	if(ptp_engine->pmotor_data == NULL)
	{
		printk(KERN_ERR "pmotor_data devm_kzalloc error!\n");
		return -ENOMEM;
	}
	ptp_engine->sensor_num = data[7];
	ptp_engine->psensor_data = devm_kzalloc(pdev, sizeof(struct sensor_data_t) * ptp_engine->sensor_num, GFP_KERNEL);
	if(ptp_engine->psensor_data == NULL)
	{
		printk(KERN_ERR "psensor_data devm_kzalloc error!\n");
		return -ENOMEM;
	}
	//关联组件设备
	psub_node = of_find_node_by_phandle(data[0]);
	ret = tp_engine_devicetree_ph_parse(psub_node, ptp_engine->pph_data, ptp_engine->ph_num);
	if (ret)
	{
		return ret;
	}
	psub_node = of_find_node_by_phandle(data[2]);
	ret = tp_engine_devicetree_ph_resistor_parse(psub_node, ptp_engine->pph_resistor_data, ptp_engine->ph_resistor_num);
	if (ret)
	{
		return ret;
	}
	psub_node = of_find_node_by_phandle(data[4]);
	ret = tp_engine_devicetree_motor_parse(psub_node, ptp_engine->pmotor_data, ptp_engine->motor_num);
	if (ret)
	{
		return ret;
	}
	psub_node = of_find_node_by_phandle(data[6]);
	ret = tp_engine_devicetree_sensor_parse(psub_node, ptp_engine->psensor_data, ptp_engine->sensor_num);
	if (ret)
	{
		return ret;
	}
	
	return ret;
}

/********************************************************
 * GLOBAL FUNCTIONS
 ********************************************************/
int tp_engine_open(struct inode *inode, struct file *filep)
{
	int ret = 0;
	struct tp_engine_dev_t * pdev;
	
	printk(KERN_DEBUG "tp_engine %s.\n", __FUNCTION__);
	pdev = container_of(inode->i_cdev, struct tp_engine_dev_t, cdev);
	filep->private_data = pdev;
	
	return ret;
}
EXPORT_SYMBOL_GPL(tp_engine_open);

int tp_engine_close(struct inode *inode, struct file *filep)
{
	int ret = 0;
	
	return ret;
}
EXPORT_SYMBOL_GPL(tp_engine_close);

static long tp_engine_ioctl(struct file *filep, unsigned int ioctrl_cmd, unsigned long arg)
{
	//void __user *argp = (void __user *)arg;
	int ret = 0;
	unsigned int cmd;
	//struct tp_engine_dev_t *ptp_eng = (struct tp_engine_dev_t *)(filep->private_data);
	
	cmd = ioctrl_cmd & 0xFF;
	switch(cmd)
	{
		case TP_ENG_IOCTL_PH_UP:
			break;
		case TP_ENG_IOCTL_PH_DOWN:
			break;
		case TP_ENG_IOCTL_PH_RST:
			break;
		case TP_ENG_IOCTL_PRINT:
			break;
		case TP_ENG_IOCTL_PAP_IN:
			break;
		case TP_ENG_IOCTL_PAP_OUT:
			break;
		case TP_ENG_IOCTL_PAP_MOVE:
			break;
		case TP_ENG_IOCTL_RIBBON_RUN:
			break;
		case TP_ENG_IOCTL_SENSOR_ST:
			break;
		default:
			goto __exit__;
	}
__exit__:
	return ret;
}
EXPORT_SYMBOL_GPL(tp_engine_ioctl);

static struct file_operations tp_engine_fops =
{
	.owner =	THIS_MODULE,
	.open =		tp_engine_open,
	.release =	tp_engine_close,
	.unlocked_ioctl = tp_engine_ioctl,
};

int tp_engine_probe(struct platform_device * pdev)
{
	int ret = 0;
	dev_t dev_no;
	struct tp_engine_dev_t * ptp_engine_dev;
	
	ptp_engine_dev = devm_kzalloc(&pdev->dev, sizeof(struct tp_engine_dev_t), GFP_KERNEL);
	if(ptp_engine_dev == NULL)
		return -ENOMEM;
	ptp_engine_dev->dev = &pdev->dev;

	tp_engine_devicetree_parse(&pdev->dev, &ptp_engine_dev->tp_engine);
	dev_no = MKDEV(tp_engine_dev_major, TP_ENGINE_DEV_INDEX);
	if(tp_engine_dev_major)
	{
		ret = register_chrdev_region(dev_no, 1, ptp_engine_dev->tp_engine.tp_engine_name);
	}
	else
	{
		ret = alloc_chrdev_region(&dev_no, 0, 1, ptp_engine_dev->tp_engine.tp_engine_name);
		tp_engine_dev_major = MAJOR(dev_no);
	}
	printk(KERN_NOTICE "tp engine dev: major %d, dev_no %d, ret %x", tp_engine_dev_major, dev_no, ret);
	if(ret < 0)
	{
		printk(KERN_ERR "ERROR tp_engine register chrdev err = %x", ret);
		goto __exit__;
	}
	cdev_init(&ptp_engine_dev->cdev, &tp_engine_fops);
	ptp_engine_dev->cdev.owner = THIS_MODULE;
	
	ret = cdev_add(&ptp_engine_dev->cdev, dev_no, 1);
	if(ret)
	{
		printk(KERN_NOTICE "Error %d add tp_engine cdev", ret);
		cdev_del(&ptp_engine_dev->cdev);
		unregister_chrdev_region(dev_no, 1);
		goto __exit__;
	}
	
__exit__:
	return ret;
}
EXPORT_SYMBOL_GPL(tp_engine_probe);

int tp_engine_remove(struct platform_device *pdev)
{
	int ret = 0;
	return ret;
}
EXPORT_SYMBOL_GPL(tp_engine_remove);

static const struct of_device_id tp_engine_match[] = {
	{ .compatible = "gwi,tp_printer_engine", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, tp_engine_match);

static struct platform_driver tp_engine_driver = {
	.probe = tp_engine_probe,
	.remove = tp_engine_remove,
	.driver = {
		.name = "tp_engine",
		.owner = THIS_MODULE,
		.of_match_table = tp_engine_match,
	},
};

module_platform_driver(tp_engine_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Dawei Shi");
MODULE_DESCRIPTION("Thermal Printer Engine driver");
