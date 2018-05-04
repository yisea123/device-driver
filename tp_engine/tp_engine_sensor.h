/*
 * GWI Thermal Printer Printer Engine Sensor.
 *
 * Copyright 2018 Hunan GreatWall Information Financial Equipment Co., Ltd.
 *
 * Licensed under the GPL-2.
*/
#ifndef _TP_ENGINE_SENSOR_H_
#define _TP_ENGINE_SENSOR_H_

#include "../photosensor/photosensor.h"

#define NAME_MAX_LEN 32

struct sensor_data_t
{
	unsigned int sen_mask;
	unsigned char sen_name[NAME_MAX_LEN];
	union
	{
		struct photosensor *pphotosensor;
	}sen_dev;
	unsigned char sen_type;
	struct photosensor_config config;
};


#endif