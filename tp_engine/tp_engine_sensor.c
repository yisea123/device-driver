/*
 * GWI Thermal Printer Printer Engine Sensor.
 *
 * Copyright 2018 Hunan GreatWall Information Financial Equipment Co., Ltd.
 *
 * Licensed under the GPL-2.
*/
#include "tp_engine_sensor.h"

//获取所有传感的值，存放在val中
int tp_engine_sensor_get_refval(struct tp_engine_t *ptp_engine, unsigned int *val)
{
	struct sensor_data_t *psensor_data;
	unsigned int i;
	int ret=0;
	unsigned long tmp;
	unsigned int * p_val;
	
	p_val = val;
	psensor_data = ptp_engine->psensor_data;
	for (i = 0; i < ptp_engine->sensor_num; i++)
	{
		ret = photosensor_read_input(psensor_data->sen_dev.pphotosensor, &tmp);
//		printk("psensor_data[%d] val[%d] is %ld\n",i,i,*tmp);
		psensor_data++;
		*p_val = (unsigned int)tmp;
		p_val++;
	}
	return ret;
}
EXPORT_SYMBOL_GPL(tp_engine_sensor_get_refval);

int tp_engine_sensor_get_logicval(struct tp_engine_t *ptp_engine, unsigned int *appval)
{
	struct sensor_data_t *psensor_data;
	unsigned int i;
	int ret=0;
	unsigned int val = 0;
	
	*appval = 0;
	psensor_data = ptp_engine->psensor_data;
	for (i = 0; i < ptp_engine->sensor_num; i++)
	{
		ret =  photosensor_status(psensor_data->sen_dev.pphotosensor, &val);
		if (ret < 0)
		{
			return ret;
		}
		if (val > 0)
		{
			psensor_data->sen_status = psensor_data->sen_mask;
		}
		else
		{
			psensor_data->sen_status = 0;
		}
		*appval |= psensor_data->sen_status;
		psensor_data++;
	}
	return ret;
}
EXPORT_SYMBOL_GPL(tp_engine_sensor_get_logicval);

//set sensor config
int tp_engine_sensor_set_config(struct tp_engine_t *ptp_engine, tp_engine_sen_config_t *p_sen_config )
{
	struct photosensor_config config;
	unsigned int i;
	int ret=0;
	struct sensor_data_t *psensor_data;

	ptp_engine->sensor_masks = p_sen_config->sen_masks;
	psensor_data = ptp_engine->psensor_data;
	for (i = 0; i < ptp_engine->sensor_num; i++)
	{
		ret = photosensor_get_config(psensor_data->sen_dev.pphotosensor, &config);
		if (ret)
		{
			printk(KERN_ERR "photosensor_get_config ret =  %d\n",ret);
			return ret;
		}
		config.compare_threshold = p_sen_config->pps_ref_value;
		config.led_brightness = p_sen_config->pps_drv_value;
		ret = photosensor_set_config(psensor_data->sen_dev.pphotosensor, &config);
		if (ret)
		{
			printk(KERN_ERR "photosensor_set_config ret =  %d\n",ret);
			return ret;
		}
		psensor_data++;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(tp_engine_sensor_set_config);

//----------------------sensor enable/disable---------------------- 
int tp_engine_sensor_enable_all(struct tp_engine_t *ptp_engine, unsigned char enable)
{
	unsigned int i;
	int ret=0;

	for (i =0; i < ptp_engine->sensor_num; i++ ) {
		if (!enable){
			ret = photosensor_disable(ptp_engine->psensor_data[i].sen_dev.pphotosensor);
		}
		else 
			ret = photosensor_enable(ptp_engine->psensor_data[i].sen_dev.pphotosensor);

		if (ret) {
			return ret;
		}
	}
	return 0;
}
EXPORT_SYMBOL_GPL(tp_engine_sensor_enable_all);

//获取所有传感器的状态
int tp_engine_get_sensor_status(struct tp_engine_t *ptp_engine, unsigned int *pstatus)
{
	int ret=0;
	unsigned int status = 0;
	struct sensor_data_t *psensor_data;

	psensor_data = ptp_engine->psensor_data;
	ret = tp_engine_sensor_get_logicval(ptp_engine, &status);
	if (ret < 0)
	{
		return ret;
	}
	*pstatus = ptp_engine->sensor_masks & (status);

	return ret;
}
EXPORT_SYMBOL_GPL(tp_engine_get_sensor_status);


//获取所有传感器的状态
int tp_engine_get_sensor_statu(struct tp_engine_t *ptp_engine, unsigned int *pstatus, unsigned int mask)
{
	int ret=0, i=0 ;
	struct sensor_data_t *psensor_data;
	unsigned int val = 0;

	psensor_data = ptp_engine->psensor_data;
	for (i = 0; i < ptp_engine->sensor_num; i++)
	{
		if (psensor_data[i].sen_mask == mask)
		{
			if (&psensor_data[i] == NULL)
			{
				printk("&psensor_data[i] == NULL.\r\n");
			}
			ret =  photosensor_status(psensor_data[i].sen_dev.pphotosensor, &val);
			if (ret < 0)
			{
				return ret;
			}
			if (val > 0)
			{
				psensor_data[i].sen_status = psensor_data[i].sen_mask;
			}
			else
			{
				psensor_data[i].sen_status = 0;
			}
			*pstatus = psensor_data[i].sen_status;
		}
	}
	return ret;
}
EXPORT_SYMBOL_GPL(tp_engine_get_sensor_statu);


int tp_engine_sensor_set_callback(struct photosensor *sensor, 
				  void (*callback)(struct photosensor *, 
					struct sen_callback_data *),struct sen_callback_data *data)
{
	if (!sensor || !callback)
		return -EINVAL;
	sensor->callback = callback;
	sensor->sensor_callback_data = *data;
	return 0;
}
EXPORT_SYMBOL_GPL(tp_engine_sensor_set_callback);
