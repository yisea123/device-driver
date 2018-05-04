/*
 * GWI Thermal Printer Printer Engine Printer Header.
 *
 * Copyright 2018 Hunan GreatWall Information Financial Equipment Co., Ltd.
 *
 * Licensed under the GPL-2.
*/
#ifndef _TP_ENGINE_PH_H_
#define _TP_ENGINE_PH_H_

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
};

#endif