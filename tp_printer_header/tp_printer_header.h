/*
 * GWI Thermal Printer Printer Header Driver.
 *
 * Copyright 2018 Hunan GreatWall Information Financial Equipment Co., Ltd.
 *
 * Licensed under the GPL-2.
*/

#ifndef __TP_PRINTER_HEADER_H__
#define __TP_PRINTER_HEADER_H__

/* 打印头的各种时间要求 */
struct tp_ph_period_config_t
{
	unsigned int clock_freq_hz;
	unsigned int clock_width;
	unsigned int data_setup_time;
	unsigned int data_hold_time;
	unsigned int data_out_delay_time;
	unsigned int lat_width;
	unsigned int lat_hold_time;
	unsigned int lat_setup_time;
	unsigned int stb_setup_time;
	unsigned int drv_out_delay_time;
	unsigned int total_time;
};

struct tp_ph_config_t
{
	struct tp_ph_period_config_t period_config;
	unsigned short dots_in_a_line;		//how many dots in a line in printer header
	unsigned char * buffer;
	unsigned int data_size;
};

struct tp_ph_ops_t
{
	int (*config)(struct device * dev, struct tp_ph_period_config_t * config);
	int (*write_data)(struct device * dev, struct tp_ph_period_config_t * config);
	int (*driver_out)(struct device * dev, struct tp_ph_period_config_t * config);
};

struct tp_ph_t
{
	struct device * dev;
	struct tp_ph_config_t config_data;
	const struct tp_ph_ops_t * ops;
};

#endif