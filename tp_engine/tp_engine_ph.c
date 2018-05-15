/*
 * GWI Thermal Printer Printer Engine Printer Header.
 *
 * Copyright 2018 Hunan GreatWall Information Financial Equipment Co., Ltd.
 *
 * Licensed under the GPL-2.
*/

#include "tp_engine_ph.h"


/********************************************************
 * LOCAL FUNCTIONS
 ********************************************************/


/********************************************************
 * GLOBAL FUNCTIONS
 ********************************************************/
int tp_engine_ph_config(struct ph_data_t * pph_data, struct tp_ph_config_t *pconfig_data)
{
	return 0;
}

int tp_engine_ph_write_data(struct ph_data_t * pph_data, unsigned char buff, unsigned int size)
{
	return 0;
}

int tp_engine_ph_latch(struct ph_data_t * pph_data)
{
	return 0;
}

int tp_engine_ph_stb(struct ph_data_t * pph_data)
{
	return 0;
}
