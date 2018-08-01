/*
 * GWI Thermal Printer Printer Engine Printer Header.
 *
 * Copyright 2018 Hunan GreatWall Information Financial Equipment Co., Ltd.
 *
 * Licensed under the GPL-2.
*/
#ifndef _TP_ENGINE_PH_H_
#define _TP_ENGINE_PH_H_
#include "../tp_printer_header/tp_printer_header.h"
#include "tp_engine_sensor.h"
#include "tp_engine.h"

#define NAME_MAX_LEN 32

struct ph_data_t
{
	unsigned short ph_mask;
	unsigned char ph_name[NAME_MAX_LEN];
	struct tp_ph_t * ptp_ph;
};

struct ph_resistor_data_t
{
	unsigned short ph_resistor_mask;
	unsigned char ph_resistor_name[NAME_MAX_LEN];
	unsigned int resistor_status;		//传感器状态
	union
	{
		struct photosensor *pphotosensor;
		//struct iio_channel *ph_r_chan;
	}ph_r_dev;
};

int tp_eng_ph_config(struct ph_data_t * pph_data, struct tp_ph_config_t *pconfig_data);
int tp_eng_ph_write_line(struct ph_data_t * pph_data, unsigned char * buff, unsigned int size);

extern int tp_engine_resister_get_refval(struct tp_engine_t *ptp_engine, unsigned int *val);

extern int tp_engine_resister_enable(struct tp_engine_t *ptp_engine, unsigned char enable);

extern int tp_engine_resister_set_config(struct tp_engine_t *ptp_engine, tp_engine_sen_config_t *p_sen_config );
#endif