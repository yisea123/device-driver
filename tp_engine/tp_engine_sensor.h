/*
 * GWI Thermal Printer Printer Engine Sensor.
 *
 * Copyright 2018 Hunan GreatWall Information Financial Equipment Co., Ltd.
 *
 * Licensed under the GPL-2.
*/
#ifndef _TP_ENGINE_SENSOR_H_
#define _TP_ENGINE_SENSOR_H_

#include "../photosensor/dispenser_photosensor.h"
#include "tp_engine.h"

#define NAME_MAX_LEN 32
#define SEN_ANALOG_TYPE	    	PHOTOSENSOR_ANALOG
#define SEN_DIGITAL_TYPE	PHOTOSENSOR_DIGITAL

#define SEN_MEDIA_IN	0
#define SEN_MEDIA_OUT	1

typedef struct{
	unsigned int sen_mask;
	unsigned long pps_ref_value;
	unsigned long pps_drv_value;
	unsigned int sen_masks;
}tp_engine_sen_config_t;


struct sensor_data_t
{
	unsigned int sen_mask;
	int sensor_num;					//engine中传感器数量
	unsigned char sen_name[NAME_MAX_LEN];
	union
	{
		struct photosensor *pphotosensor;
	}sen_dev;
	unsigned char sen_type;
	struct photosensor_config config;
	unsigned int sen_status;			//传感器状态
	int err;					//错误码
	tp_engine_sen_config_t	*sen_config;
};


extern int tp_engine_sensor_enable_all(struct tp_engine_t *ptp_engine, unsigned char enable);

extern int tp_engine_sensor_get_refval(struct tp_engine_t *ptp_engine, unsigned int *val);

extern int tp_engine_sensor_get_logicval(struct tp_engine_t *ptp_engine, unsigned int *appval);

extern int tp_engine_sensor_set_config(struct tp_engine_t *ptp_engine, tp_engine_sen_config_t *p_sen_config );

extern int tp_engine_get_sensor_status(struct tp_engine_t *ptp_engine, unsigned int *pstatus);

extern int tp_engine_sensor_set_callback(struct photosensor *sensor, 
				  void (*callback)(struct photosensor *, 
					struct sen_callback_data *),struct sen_callback_data *data);

#endif
