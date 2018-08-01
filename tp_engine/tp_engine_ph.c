/*
 * GWI Thermal Printer Printer Engine Printer Header.
 *
 * Copyright 2018 Hunan GreatWall Information Financial Equipment Co., Ltd.
 *
 * Licensed under the GPL-2.
*/

#include "tp_engine_ph.h"
#include "tp_engine_sensor.h"
#include "../tp_printer_header/tp_printer_header.h"
#include "../tp_printer_header/printer_header_common.h"
#include "../photosensor/dispenser_photosensor.h"

/********************************************************
 * LOCAL FUNCTIONS
 ********************************************************/

/********************************************************
 * GLOBAL FUNCTIONS
 ********************************************************/
int tp_eng_ph_config(struct ph_data_t * pph_data, struct tp_ph_config_t *pconfig_data)
{
	return ph_config(pph_data->ptp_ph, pconfig_data);
}
EXPORT_SYMBOL_GPL(tp_eng_ph_config);

int tp_eng_ph_write_line(struct ph_data_t * pph_data, unsigned char * buff, unsigned int size)
{
	return ph_write_line(pph_data->ptp_ph, buff, size);
}
EXPORT_SYMBOL_GPL(tp_eng_ph_write_line);

//set sensor config
int tp_engine_resister_set_config(struct tp_engine_t *ptp_engine, tp_engine_sen_config_t *p_sen_config )
{
	struct photosensor_config config;
	unsigned int i;
	int ret=0;
	struct ph_resistor_data_t * pph_resistor_data;

	ptp_engine->pph_resistor_data->ph_resistor_mask = p_sen_config->sen_masks;
	pph_resistor_data = ptp_engine->pph_resistor_data;

	ret = photosensor_get_config(pph_resistor_data->ph_r_dev.pphotosensor, &config);
	if (ret)
	{
		printk(KERN_ERR "photosensor_get_config ret =  %d\n",ret);
		return ret;
	}
	config.compare_threshold = p_sen_config->pps_ref_value;
	config.led_brightness = p_sen_config->pps_drv_value;
	ret = photosensor_set_config(pph_resistor_data->ph_r_dev.pphotosensor, &config);
	if (ret)
	{
		printk(KERN_ERR "photosensor_set_config ret =  %d\n",ret);
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(tp_engine_resister_set_config);

int tp_engine_resister_get_refval(struct tp_engine_t *ptp_engine, unsigned int *val)
{
	struct ph_resistor_data_t * pph_resistor_data;
	int ret=0;
	unsigned long *tmp;
	
	tmp = (unsigned long *)val;
	pph_resistor_data = ptp_engine->pph_resistor_data;
	ret = photosensor_read_input(pph_resistor_data->ph_r_dev.pphotosensor, tmp);

	return ret;
}
EXPORT_SYMBOL_GPL(tp_engine_resister_get_refval);

//----------------------sensor enable/disable----------------------
int tp_engine_resister_enable(struct tp_engine_t *ptp_engine, unsigned char enable)
{
	unsigned int i;
	int ret=0;
	struct ph_resistor_data_t * pph_resistor_data;

	pph_resistor_data = ptp_engine->pph_resistor_data;
	if (!enable){
		ret = photosensor_disable(ptp_engine->pph_resistor_data->ph_r_dev.pphotosensor);
	}
	else
		ret = photosensor_enable(ptp_engine->pph_resistor_data->ph_r_dev.pphotosensor);

	if (ret) {
		return ret;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(tp_engine_resister_enable);

//int tp_eng_ph_resistor_get_val(struct ph_resistor_data_t * pph_resistor_data, unsigned long * pval)
//{
//	struct photosensor *psen;
//	int ret = 0;
//
//	psen = pph_resistor_data->ph_r_dev.pphotosensor;
//	ret = photosensor_read_input(psen, pval);
//	return ret;
//}
//EXPORT_SYMBOL_GPL(tp_eng_ph_resistor_get_val);
