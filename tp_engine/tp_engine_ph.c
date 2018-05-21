/*
 * GWI Thermal Printer Printer Engine Printer Header.
 *
 * Copyright 2018 Hunan GreatWall Information Financial Equipment Co., Ltd.
 *
 * Licensed under the GPL-2.
*/

#include "tp_engine_ph.h"
#include "../tp_printer_header/tp_printer_header.h"
#include "../tp_printer_header/printer_header_common.h"


/********************************************************
 * LOCAL FUNCTIONS
 ********************************************************/


/********************************************************
 * GLOBAL FUNCTIONS
 ********************************************************/
int tp_engine_ph_config(struct ph_data_t * pph_data, struct tp_ph_config_t *pconfig_data)
{
	return ph_config(pph_data->ptp_ph, pconfig_data);
}
EXPORT_SYMBOL_GPL(tp_engine_ph_config);

int tp_engine_ph_write_data(struct ph_data_t * pph_data, unsigned char * buff, unsigned int size)
{
	return ph_write_data(pph_data->ptp_ph, buff, size);
}
EXPORT_SYMBOL_GPL(tp_engine_ph_write_data);
