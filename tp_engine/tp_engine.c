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
#include "tp_engine_pap_motor.h"
#include "tp_engine_ph_motor.h"
#include "tp_engine_ribbon_motor.h"
#include "tp_engine_sensor.h"
#include "tp_engine.h"
#include "command.h"
#include "tp_engine_fun.h"
#include "../tp_printer_header/printer_header_common.h"

#define TP_ENGINE_DEV_MAJOR		0
#define TP_ENGINE_DEV_INDEX		0

struct tp_engine_dev_t
{
	struct cdev cdev;
	struct device *dev;
	struct device *peng_dev;
	struct class *tp_eng_class;
	dev_t dev_no;
	struct tp_engine_t tp_engine;
};

/********************************************** 
 * LOCAL VARIABLE
 **********************************************/

static int tp_engine_dev_major = TP_ENGINE_DEV_MAJOR;

/********************************************************
 * LOCAL FUNCTIONS
 ********************************************************/
/* 从设备树获取设备并关联到数据结构 */
static int tp_engine_devicetree_ph_parse(struct device_node *pnode, struct ph_data_t *pph_data)
{
	const char * name;
	unsigned int data;
	struct device_node * pph_node;
	int ret = 0;
	
	ret = of_property_read_u32(pnode, "ph_dev", &data);
	if(ret)
	{
		printk(KERN_ERR "ERROR %s %d\n", __FUNCTION__, __LINE__);
		return ret;
	}
	pph_node = of_find_node_by_phandle(data);
	if (pph_node == NULL)
	{
		printk(KERN_ERR "tp_engine_devicetree_ph_parse of_find_node_by_phandle failed.\n");
		return -1;
	}
	pph_data->ptp_ph = of_node_to_ph(pph_node);
	
	ret = of_property_read_string(pnode, "ph_name", &name);
	if(ret)
	{
		printk(KERN_ERR "ERROR %s %d\n", __FUNCTION__, __LINE__);
		return ret;
	}
	strcpy(pph_data->ph_name, name);
	ret = of_property_read_u32(pnode, "ph_mask", &data);
	if(ret)
	{
		printk(KERN_ERR "ERROR %s %d\n", __FUNCTION__, __LINE__);
		return ret;
	}
	pph_data->ph_mask = data;
	return 0;
}

/* 从设备树获取设备并关联到数据结构 */
static int tp_engine_devicetree_ph_resistor_parse(struct device_node *pnode, struct ph_resistor_data_t *pph_resistor_data)
{
	int ret = 0;
	struct device_node *pph_r_dev;
	const char *p_name;
	unsigned int data;

	ret = of_property_read_string(pnode, "th_resistor_name", &p_name);
	if (ret)
	{
		printk(KERN_ERR "ERROR %s ==> %d\n", __FUNCTION__, __LINE__);
		return ret;
	}
	strcpy(pph_resistor_data->ph_resistor_name, p_name);
	ret = of_property_read_u32(pnode, "th_resistor_mask", &data);
	if (ret)
	{
		printk(KERN_ERR "ERROR %s ==> %d\n", __FUNCTION__, __LINE__);
		return ret;
	}
	pph_resistor_data->ph_resistor_mask = data;
	ret = of_property_read_u32(pnode, "th_resistor_dev", &data);
	if (ret)
	{
		printk(KERN_ERR "ERROR %s ==> %d\n", __FUNCTION__, __LINE__);
		return ret;
	}
	pph_r_dev = of_find_node_by_phandle(data);
	if(pph_r_dev==NULL)
	{
		printk(KERN_ERR "ERROR in Func %s of_find_node_by_phandle\n", __FUNCTION__);
		return ret;
	}
	pph_resistor_data->ph_r_dev.pphotosensor = of_node_to_photosensor(pph_r_dev);
	if (pph_resistor_data->ph_r_dev.pphotosensor == ERR_PTR(-EPROBE_DEFER))
	{
		printk(KERN_ERR "ERROR in Func %s of_node_to_photosensor\n", __FUNCTION__);
		return (-EPROBE_DEFER);
	}
	return ret;
}

static int tp_engine_devicetree_pap_motor_parse(struct device_node *pnode, struct pap_motor_data_t *pmotor_data)
{
	int ret = 0;
	struct device_node * pmotor_dev;
	const char * pname;
	unsigned int data;
	ret = of_property_read_string(pnode, "motor_name", &pname);
	if(ret)
	{
		printk(KERN_ERR "ERROR %s ==> %d\n", __FUNCTION__, __LINE__);
		return ret;
	}
	strcpy(pmotor_data->motor_name, pname);
	ret = of_property_read_u32(pnode,"motor_mask",&data);
	if (ret)
	{
		printk(KERN_ERR "ERROR %s ==> %d\n", __FUNCTION__, __LINE__);
		return ret;
	}
	pmotor_data->motor_mask = data;
	ret = of_property_read_u32(pnode,"motor_dev",&data);
	if (ret)
	{
		printk(KERN_ERR "ERROR %s ==> %d\n", __FUNCTION__, __LINE__);
		return ret;
	}
	pmotor_dev = of_find_node_by_phandle(data);
	pmotor_data->pstepmotor = of_node_to_steppermotor(pmotor_dev); 
	if (pmotor_data->pstepmotor == ERR_PTR(-EPROBE_DEFER))
	{
		printk(KERN_ERR "ERROR %s ==> %d\n", __FUNCTION__, __LINE__);
		return -EPROBE_DEFER;
	}
	return 0;
}

static int tp_engine_devicetree_ph_motor_parse(struct device_node *pnode, struct ph_motor_data_t *pmotor_data)
{
	int ret = 0;
	struct device_node * pmotor_dev;
	const char * pname;
	unsigned int data;
	ret = of_property_read_string(pnode, "motor_name", &pname);
	if(ret)
	{
		printk(KERN_ERR "ERROR %s ==> %d\n", __FUNCTION__, __LINE__);
		return ret;
	}
	strcpy(pmotor_data->motor_name, pname);
	ret = of_property_read_u32(pnode,"motor_mask",&data);
	if (ret)
	{
		printk(KERN_ERR "ERROR %s ==> %d\n", __FUNCTION__, __LINE__);
		return ret;
	}
	pmotor_data->motor_mask = data;
	ret = of_property_read_u32(pnode,"motor_dev",&data);
	if (ret)
	{
		printk(KERN_ERR "ERROR %s ==> %d\n", __FUNCTION__, __LINE__);
		return ret;
	}
	pmotor_dev = of_find_node_by_phandle(data);
	pmotor_data->pdcmotor = of_node_to_dcmotor(pmotor_dev);
	if (pmotor_data->pdcmotor == ERR_PTR(-EPROBE_DEFER))
	{
		printk(KERN_ERR "ERROR %s ==> %d\n", __FUNCTION__, __LINE__);
		return -EPROBE_DEFER;
	}
	return 0;
}

static int tp_engine_devicetree_ribbon_motor_parse(struct device_node *pnode, struct ribbon_motor_data_t *pmotor_data)
{
	int ret = 0;
	struct device_node * pmotor_dev;
	const char * pname;
	unsigned int data;
	ret = of_property_read_string(pnode, "motor_name", &pname);
	if(ret)
	{
		printk(KERN_ERR "ERROR %s ==> %d\n", __FUNCTION__, __LINE__);
		return ret;
	}
	strcpy(pmotor_data->motor_name, pname);
	ret = of_property_read_u32(pnode,"motor_mask",&data);
	if (ret)
	{
		printk(KERN_ERR "ERROR %s ==> %d\n", __FUNCTION__, __LINE__);
		return ret;
	}
	pmotor_data->motor_mask = data;
	ret = of_property_read_u32(pnode,"motor_dev",&data);
	if (ret)
	{
		printk(KERN_ERR "ERROR %s ==> %d\n", __FUNCTION__, __LINE__);
		return ret;
	}
	pmotor_dev = of_find_node_by_phandle(data);
	pmotor_data->pdcmotor = of_node_to_dcmotor(pmotor_dev);
	if (pmotor_data->pdcmotor == ERR_PTR(-EPROBE_DEFER))
	{
		printk(KERN_ERR "ERROR %s ==> %d\n", __FUNCTION__, __LINE__);
		return -EPROBE_DEFER;
	}
	return 0;
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
	unsigned int data[12];
	const char *pname;
	
	printk(KERN_DEBUG "tp_engine device tree parse.\n");
	node = pdev->of_node;
	if(!node)
	{
		return (-ENODEV);
	}
	if(!of_find_property(node, "tp_engine_name", NULL))
	{
		dev_err(pdev, "Found engine without tp_engine_name");
		return (-EINVAL);
	}
	memset(ptp_engine->tp_engine_name, 0, TP_ENGINE_NAME_MAX_SIZE);
	ret = of_property_read_string(node, "tp_engine_name", &pname);
	strcpy(ptp_engine->tp_engine_name, pname);
	printk("tp_engine_name = %s, ret = %d\n", ptp_engine->tp_engine_name, ret);
	//读取各个组件数量与设备，申请空间，本项目中有打印头，热敏电阻，走纸电机，打印头电机，走碳带电机，传感器
	if(!of_find_property(node, "tp_engine_component", NULL))
	{
		dev_err(pdev, "ERROR!!! Found tp_engine_component without tp_engine_component\n");
		return (-EINVAL);
	}
	of_property_read_u32_array(node, "tp_engine_component", data, ARRAY_SIZE(data));
	ptp_engine->pph_data = devm_kzalloc(pdev, sizeof(struct ph_data_t), GFP_KERNEL);
	if(ptp_engine->pph_data == NULL)
	{
		printk(KERN_ERR "ERROR!!! ph_data devm_kzalloc error!\n");
		return -ENOMEM;
	}
	ptp_engine->pph_resistor_data = devm_kzalloc(pdev, sizeof(struct ph_resistor_data_t), GFP_KERNEL);
	if(ptp_engine->pph_resistor_data == NULL)
	{
		printk(KERN_ERR "ERROR!!! pph_resistor_data devm_kzalloc error!\n");
		return -ENOMEM;
	}
	ptp_engine->ppap_motor_data = devm_kzalloc(pdev, sizeof(struct pap_motor_data_t), GFP_KERNEL);
	if(ptp_engine->ppap_motor_data == NULL)
	{
		printk(KERN_ERR "ERROR!!! pmotor_data devm_kzalloc error!\n");
		return -ENOMEM;
	}
	ptp_engine->pph_motor_data = devm_kzalloc(pdev, sizeof(struct ph_motor_data_t), GFP_KERNEL);
	if(ptp_engine->pph_motor_data == NULL)
	{
		printk(KERN_ERR "ERROR!!! pmotor_data devm_kzalloc error!\n");
		return -ENOMEM;
	}
	ptp_engine->pribbon_motor_data = devm_kzalloc(pdev, sizeof(struct ribbon_motor_data_t), GFP_KERNEL);
	if(ptp_engine->pribbon_motor_data == NULL)
	{
		printk(KERN_ERR "ERROR!!! pmotor_data devm_kzalloc error!\n");
		return -ENOMEM;
	}
	ptp_engine->sensor_num = data[11];
	ptp_engine->psensor_data = devm_kzalloc(pdev, sizeof(struct sensor_data_t) * ptp_engine->sensor_num, GFP_KERNEL);
	if(ptp_engine->psensor_data == NULL)
	{
		printk(KERN_ERR "ERROR!!! psensor_data devm_kzalloc error!\n");
		return -ENOMEM;
	}
	//关联组件设备
	psub_node = of_find_node_by_phandle(data[0]);
	ret = tp_engine_devicetree_ph_parse(psub_node, ptp_engine->pph_data);
	if (ret)
	{
		return ret;
	}
	psub_node = of_find_node_by_phandle(data[2]);
	ret = tp_engine_devicetree_ph_resistor_parse(psub_node, ptp_engine->pph_resistor_data);
	if (ret)
	{
		return ret;
	}
	psub_node = of_find_node_by_phandle(data[4]);
	ret = tp_engine_devicetree_pap_motor_parse(psub_node, ptp_engine->ppap_motor_data);
	if (ret)
	{
		return ret;
        }
	psub_node = of_find_node_by_phandle(data[6]);
	ret = tp_engine_devicetree_ph_motor_parse(psub_node, ptp_engine->pph_motor_data);
	if (ret)
	{
		return ret;
        }
	psub_node = of_find_node_by_phandle(data[8]);
	ret = tp_engine_devicetree_ribbon_motor_parse(psub_node, ptp_engine->pribbon_motor_data);
	if (ret)
	{
		return ret;
        }
	psub_node = of_find_node_by_phandle(data[10]);
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
	void __user *argp = (void __user *)arg;
	int ret = 0;
	unsigned int cmd;
	struct tp_engine_dev_t *ptp_eng_dev = (struct tp_engine_dev_t *)(filep->private_data);
	
	cmd = ioctrl_cmd & 0xFF;

	switch(cmd)
	{
		case TP_ENG_IOCTL_RESET:
			break;
		case TP_ENG_IOCTL_SET_ENG_CONFIG:
			{
				struct tp_engine_config_t config;

				if (copy_from_user(&config,
						(void __user *)argp, sizeof(struct tp_engine_config_t)))
				{
					printk(KERN_ERR "TP_ENG_IOCTL_SENSOR_CONFIG: copy_from_user fail\n");
					ret = -EFAULT;
					goto __exit__;
				}
				ret = tp_eng_fun_config(&(ptp_eng_dev->tp_engine), &config);
				if (ret < 0)
				{
					ret = -EFAULT;
					goto __exit__;
				}
			}
			break;
		case TP_ENG_IOCTL_SET_PH_CONFIG:
			{
				struct tp_ph_config_t config;

				if (copy_from_user(&config,
						(void __user *)argp, sizeof(struct tp_ph_config_t)))
				{
					printk(KERN_ERR "TP_ENG_IOCTL_SENSOR_CONFIG: copy_from_user fail\n");
					ret = -EFAULT;
					goto __exit__;
				}
				ret = tp_eng_ph_config(ptp_eng_dev->tp_engine.pph_data, &config);
				if (ret < 0)
				{
					ret = -EFAULT;
					goto __exit__;
				}
			}
			break;
	        case TP_ENG_IOCTL_PH_UP_DOWN:
			{
				unsigned int mode = 1;
				if(copy_from_user((void *)(&mode), (void __user *)argp, sizeof(int)))
				{
				    printk(KERN_ERR "tp_engine_ioctl TP_ENG_IOCTL_PH_UP_DOWN: copy_from_user fail\n");
				    ret = -EFAULT;
				    goto __exit__;
				}
				tp_eng_fun_ph_move(&ptp_eng_dev->tp_engine, mode);
			}
			break;
		case TP_ENG_IOCTL_PRINT:
			{
				ret = tp_eng_fun_print(&ptp_eng_dev->tp_engine, argp);
				if (ret)
				{
					goto __exit__;
				}
			}
			break;
		case TP_ENG_IOCTL_PAP_IN:
			tp_eng_fun_pap_in(&ptp_eng_dev->tp_engine);
			break;
		case TP_ENG_IOCTL_PAP_OUT:
			tp_eng_fun_pap_out(&ptp_eng_dev->tp_engine);
			break;
		case TP_ENG_IOCTL_GET_PAP_LENGHT:
			{
				unsigned int len;
				ret = tp_engine_get_pap_lenght(&ptp_eng_dev->tp_engine, &len);
				if (copy_to_user((void __user *)argp, &len,sizeof(int)))
				{
					printk(KERN_NOTICE "tp_engine_ioctl TP_ENG_IOCTL_SENSOR_ST: copy_from_user fail\n");
					ret = -EFAULT;
					goto __exit__;
				}
			}
			break;
		case TP_ENG_IOCTL_GET_ENG_CONFIG:
			{
				struct tp_engine_config_t eng_config;
				ret = tp_engine_get_eng_config(&ptp_eng_dev->tp_engine, &eng_config);
				if (copy_to_user((void __user *)argp, &eng_config,sizeof(eng_config)))
				{
					printk(KERN_NOTICE "tp_engine_ioctl TP_ENG_IOCTL_SENSOR_ST: copy_from_user fail\n");
					ret = -EFAULT;
					goto __exit__;
				}
			}
			break;
		case TP_ENG_IOCTL_GET_PH_CONFIG:
			{

				struct tp_ph_config_t config_data;
				ret = tp_engine_get_ph_config(&ptp_eng_dev->tp_engine, &config_data);
				if (copy_to_user((void __user *)argp, &config_data,sizeof(config_data)))
				{
					printk(KERN_NOTICE "tp_engine_ioctl TP_ENG_IOCTL_SENSOR_ST: copy_from_user fail\n");
					ret = -EFAULT;
					goto __exit__;
				}

			}
			break;
		case TP_ENG_IOCTL_PAP_MOVE:
			{
				int argv;
				if(copy_from_user((void *)(&argv), (void __user *)argp, sizeof(int)))
				{
				    printk(KERN_ERR "tp_engine_ioctl TP_ENG_IOCTL_PH_UP_DOWN: copy_from_user fail\n");
				    ret = -EFAULT;
				    goto __exit__;
				}
				tp_eng_fun_pap_move(&ptp_eng_dev->tp_engine, argv);
			}
			break;
		case TP_ENG_IOCTL_RIBBON_RUN:
			{
			    unsigned int mode = 1;
			    if(copy_from_user((void *)(&mode), (void __user *)argp, sizeof(int)))
			    {
				    printk(KERN_ERR "tp_engine_ioctl TP_ENG_IOCTL_PH_UP_DOWN: copy_from_user fail\n");
				    ret = -EFAULT;
				    goto __exit__;
			    }
			    printk(KERN_DEBUG "TP_ENG_IOCTL_PH_UP_DOWN mode = 0x%x.\n", (unsigned int)mode);
			    tp_eng_fun_ribbon_run(&ptp_eng_dev->tp_engine, mode);
		        }
		        break;
		case TP_ENG_IOCTL_SENSOR_ST:
			{
				unsigned int status;
				ret = tp_engine_get_sensor_status(&ptp_eng_dev->tp_engine, &status);
				if (copy_to_user((void __user *)argp, &status,sizeof(int)))
				{
					printk(KERN_NOTICE "tp_engine_ioctl TP_ENG_IOCTL_SENSOR_ST: copy_from_user fail\n");
					ret = -EFAULT;
					goto __exit__;
				}
			}
			break;
		case TP_ENG_IOCTL_SENSOR_LOGIC_ST:
			{
				tp_eng_fun_sensor_update(&ptp_eng_dev->tp_engine);
				if (copy_to_user((void __user *)argp, &ptp_eng_dev->tp_engine.tp_eng_sen_st, sizeof(struct tp_engine_sen_st_t)))
				{
					printk(KERN_NOTICE "tp_engine_ioctl TP_ENG_IOCTL_SENSOR_ST: copy_from_user fail\n");
					ret = -EFAULT;
					goto __exit__;
				}
			}
			break;
		case TP_ENG_IOCTL_GET_SEN_VAL:
			{
				unsigned int val[20],i;
				unsigned int size = 0;
				unsigned int *tmp;
				unsigned long resistor_val;
				ret = tp_engine_sensor_get_refval(&ptp_eng_dev->tp_engine, val);
				size = ptp_eng_dev->tp_engine.sensor_num;
				tmp = val;
				for (i = 0; i < size; i++)
				{
					printk("TP_ENG_IOCTL_GET_SEN_VAL is %d\n", tmp[i]);
				}
				if (copy_to_user((void __user *)argp, tmp, sizeof(int)*size))
				{
					printk(KERN_ERR "tp_engine_ioctl TP_ENG_IOCTL_PH_UP_DOWN: copy_from_user fail\n");
					ret = -EFAULT;
					goto __exit__;
				}
				ret = tp_eng_ph_resistor_get_val(ptp_eng_dev->tp_engine.pph_resistor_data, &resistor_val);
				if (ret < 0)
				{
					ret = -EFAULT;
					goto __exit__;
				}
			}
			break;
		case TP_ENG_IOCTL_SENSOR_ENABLE:
			{
				unsigned int enable;
				if(copy_from_user((void *)(&enable), (void __user *)argp, sizeof(int)))
				{
					printk(KERN_ERR "tp_engine_ioctl TP_ENG_IOCTL_PH_UP_DOWN: copy_from_user fail\n");
					ret = -EFAULT;
					goto __exit__;
				 }
				if (enable)
				{
					ret = tp_engine_sensor_enable_all(&ptp_eng_dev->tp_engine, enable);
				}
				else
				{
					ret = tp_engine_sensor_enable_all(&ptp_eng_dev->tp_engine, enable);
				}
			}
			break;

		case TP_ENG_IOCTL_SENSOR_CONFIG:
			{
				tp_engine_sen_config_t sen_conf;

				if (copy_from_user(&sen_conf,
						(void __user *)argp, sizeof(tp_engine_sen_config_t)))
				{
					printk(KERN_ERR "TP_ENG_IOCTL_SENSOR_CONFIG: copy_from_user fail\n");
					ret = -EFAULT;
					goto __exit__;
				}
				ret = tp_engine_sensor_set_config(&(ptp_eng_dev->tp_engine), &sen_conf);
				if (ret < 0)
				{
					ret = -EFAULT;
					goto __exit__;
				}
			}
			break;
		default:
			goto __exit__;
	}
__exit__:
	return ret;
}
EXPORT_SYMBOL_GPL(tp_engine_ioctl);

/* tp_engine中断服务程序(打印头抬起传感器) */
int tp_engine_isr_ph_sensor(struct tp_engine_t * ptp_engine)
{
	if (ptp_engine->eng_state.ph_motor_state == PH_MOTOR_STATE_UP)
	{
		tp_eng_ph_motor_stop(ptp_engine->pph_motor_data);
	}
	else if(ptp_engine->eng_state.ph_motor_state == PH_MOTOR_STATE_DOWN)
	{
		tp_eng_ph_motor_stop(ptp_engine->pph_motor_data);
	}
	else
	{

	}
	return 0;
}
EXPORT_SYMBOL_GPL(tp_engine_isr_ph_sensor);

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
	struct tp_engine_dev_t * ptp_engine_dev;
	
	ptp_engine_dev = devm_kzalloc(&pdev->dev, sizeof(struct tp_engine_dev_t), GFP_KERNEL);
	printk(KERN_DEBUG "tp_engine_dev addr = %x.\n", (unsigned int)ptp_engine_dev);
	if(ptp_engine_dev == NULL)
		return -ENOMEM;
	ptp_engine_dev->dev = &pdev->dev;

	ret = tp_engine_devicetree_parse(&pdev->dev, &ptp_engine_dev->tp_engine);
	if (ret)
	{
		printk(KERN_ERR "ERROR!!!  tp_engine_devicetree_parse ret = %x.\n", ret);
		return ret;
	}
	ptp_engine_dev->dev_no = MKDEV(tp_engine_dev_major, TP_ENGINE_DEV_INDEX);
	if(tp_engine_dev_major)
	{
		ret = register_chrdev_region(ptp_engine_dev->dev_no, 1, ptp_engine_dev->tp_engine.tp_engine_name);
	}
	else
	{
		ret = alloc_chrdev_region(&ptp_engine_dev->dev_no, 0, 1, ptp_engine_dev->tp_engine.tp_engine_name);
		tp_engine_dev_major = MAJOR(ptp_engine_dev->dev_no);
	}
	printk(KERN_NOTICE "tp engine dev: major %d, dev_no %x, ret %x\n", tp_engine_dev_major, ptp_engine_dev->dev_no, ret);
	if(ret < 0)
	{
		printk(KERN_ERR "ERROR!!!  tp_engine register chrdev ret = %x\n", ret);
		goto __exit__;
	}
	cdev_init(&ptp_engine_dev->cdev, &tp_engine_fops);
	ptp_engine_dev->cdev.owner = THIS_MODULE;

	ptp_engine_dev->tp_eng_class = class_create(THIS_MODULE, ptp_engine_dev->tp_engine.tp_engine_name);
	if(IS_ERR(ptp_engine_dev->tp_eng_class))
	{
		printk(KERN_ERR "Error %x class_create\n", (int)(ptp_engine_dev->tp_eng_class));
		return PTR_ERR(ptp_engine_dev->tp_eng_class);
	}

	ptp_engine_dev->peng_dev = device_create(ptp_engine_dev->tp_eng_class, NULL, ptp_engine_dev->dev_no, NULL, ptp_engine_dev->tp_engine.tp_engine_name);
	if(IS_ERR(ptp_engine_dev->peng_dev))
	{
		printk(KERN_ERR "Error %x device_create\n", (int)(ptp_engine_dev->peng_dev));
		return PTR_ERR(ptp_engine_dev->peng_dev);
	}
	
	ret = cdev_add(&ptp_engine_dev->cdev, ptp_engine_dev->dev_no, 1);
	if(ret)
	{
		printk(KERN_NOTICE "Error %d add tp_engine cdev\n", ret);
		cdev_del(&ptp_engine_dev->cdev);
		unregister_chrdev_region(ptp_engine_dev->dev_no, 1);
		goto __exit__;
	}
	
__exit__:
	return ret;
}
EXPORT_SYMBOL_GPL(tp_engine_probe);

int tp_engine_remove(struct platform_device *pdev)
{
	int ret = 0;
	struct tp_engine_dev_t *ptp_engine_dev = platform_get_drvdata(pdev);

	printk(KERN_DEBUG "tp_engine Driver - exit\n");
	device_destroy(ptp_engine_dev->tp_eng_class, ptp_engine_dev->dev_no);
	class_destroy(ptp_engine_dev->tp_eng_class);
	cdev_del(&ptp_engine_dev->cdev);
	unregister_chrdev_region(ptp_engine_dev->dev_no, 1);

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

