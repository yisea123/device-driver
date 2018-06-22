/*
 * GWI Thermal Printer Printer Header Driver.
 *
 * Copyright 2018 Hunan GreatWall Information Financial Equipment Co., Ltd.
 *
 * Licensed under the GPL-2.
*/

#ifndef __TP_PRINTER_HEADER_H__
#define __TP_PRINTER_HEADER_H__

#include <linux/device.h>

/* 打印头的各种时间要求 */
struct tp_ph_period_config_t
{
	unsigned int clock_freq_hz;
	unsigned int delay_after_latch_low;
	unsigned int delay_after_data_in;
	unsigned int delay_after_latch_high;
};

struct tp_ph_config_t
{
	unsigned int time_of_heating_us;	//打印头加热时间
	unsigned short dots_in_a_line;		//打印头宽度，点数
	struct tp_ph_period_config_t period_config;	//打印时间周期控制，打印头要求各步骤时间
};

struct tp_ph_t
{
	struct device * dev;
	struct list_head list;
	struct tp_ph_config_t config_data;
	const struct tp_ph_ops_t * ops;

	unsigned char * buffer;			//打印数据存放缓冲区
	unsigned int data_size;			//打印数据长度
};

struct tp_ph_ops_t
{
	int (*config)(struct tp_ph_t * ptp_ph, struct tp_ph_config_t * pconfig);
	int (*write_line)(struct tp_ph_t * ptp_ph, unsigned char * pbuffer, unsigned int data_size);
};

#endif