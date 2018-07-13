/*
 * GWI Thermal Printer Printer Engine Ribbon Motor.
 *
 * Copyright 2018 Hunan GreatWall Information Financial Equipment Co., Ltd.
 *
 * Licensed under the GPL-2.
*/

#include "tp_engine_ribbon_motor.h"
#include "tp_engine.h"
#include "command.h"

/********************************************************
 * LOCAL FUNCTIONS
 ********************************************************/


/********************************************************
 * GLOBAL FUNCTIONS
 ********************************************************/
void tp_eng_ribbon_motor_stop(struct ribbon_motor_data_t * pribbon_motor_data)
{
	if(pribbon_motor_data->moving_status == 0)
	{
		return;
	}
	complete_all(&(pribbon_motor_data->motor_completion));
	dcmotor_stop(pribbon_motor_data->pdcmotor);
	pribbon_motor_data->moving_status = 0;
}
EXPORT_SYMBOL_GPL(tp_eng_ribbon_motor_stop);

int tp_eng_ribbon_motor_config(struct ribbon_motor_data_t * pribbon_motor_data, motion_dir dir, unsigned int timeout_ms)
{
	unsigned int time;
	time = msecs_to_jiffies(timeout_ms);
	pribbon_motor_data->dc_conf.dir = dir;
	pribbon_motor_data->dc_conf.sensor_compare_mode = 0;
	pribbon_motor_data->dc_conf.sensor_mask = 0;
	pribbon_motor_data->stop_in_ms = time;

	init_completion(&(pribbon_motor_data->motor_completion));
	pribbon_motor_data->moving_status = 0;

	return 0;
}
EXPORT_SYMBOL_GPL(tp_eng_ribbon_motor_config);

int tp_eng_ribbon_motor_start(struct ribbon_motor_data_t * pribbon_motor_data)
{
	int ret = 0;
	ret = dcmotor_set_config(pribbon_motor_data->pdcmotor, &(pribbon_motor_data->dc_conf));
	if(ret)
	{
		printk(KERN_ERR "tp_engine_dcmotor_start: steppermotor_set_config error!\n");
		return -RES_PRINTING_UNKOWN_ERROR;
	}
	ret = dcmotor_start(pribbon_motor_data->pdcmotor);
	if(ret)
	{
		printk(KERN_ERR "tp_eng_stepmotor_start: steppermotor_start error!\n");
		dcmotor_stop(pribbon_motor_data->pdcmotor);
		return -RES_PRINTING_RINBBON_WHELL;
	}
	pribbon_motor_data->moving_status = 1;
	return 0;
}
EXPORT_SYMBOL_GPL(tp_eng_ribbon_motor_start);

int tp_eng_ribbon_motor_wait_stoped(struct ribbon_motor_data_t * pribbon_motor_data)
{
	int ret = 0;
	if(pribbon_motor_data->moving_status == 0)
	{
		return 0;
	}
	ret = wait_for_completion_timeout(&(pribbon_motor_data->motor_completion), pribbon_motor_data->stop_in_ms);
	if(!ret)
	{
		printk(KERN_ERR "tp_eng_stepmotor_wait_stoped timeout!\n");
		complete_all(&(pribbon_motor_data->motor_completion));
		dcmotor_stop(pribbon_motor_data->pdcmotor);
		pribbon_motor_data->moving_status = 0;
		return -RES_PRINTING_UNKOWN_ERROR;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(tp_eng_ribbon_motor_wait_stoped);

