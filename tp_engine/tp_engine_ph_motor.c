/*
 * GWI Thermal Printer Printer Engine Printer Header Motor.
 *
 * Copyright 2018 Hunan GreatWall Information Financial Equipment Co., Ltd.
 *
 * Licensed under the GPL-2.
*/

#include "tp_engine_ph_motor.h"
#include "tp_engine.h"
#include "command.h"

/********************************************************
 * LOCAL FUNCTIONS
 ********************************************************/


/********************************************************
 * GLOBAL FUNCTIONS
 ********************************************************/
void tp_eng_ph_motor_stop(struct ph_motor_data_t * pph_motor_data)
{
	if(pph_motor_data->moving_status == 0)
	{
		return;
	}
	complete_all(&(pph_motor_data->motor_completion));
	dcmotor_stop(pph_motor_data->pdcmotor);
	pph_motor_data->moving_status = 0;
}
EXPORT_SYMBOL_GPL(tp_eng_ph_motor_stop);

int tp_eng_ph_motor_config(struct ph_motor_data_t * pph_motor_data, motion_dir dir, unsigned int timeout_ms)
{
	unsigned int time;
	time = msecs_to_jiffies(timeout_ms);
	pph_motor_data->dc_conf.dir = dir;
	pph_motor_data->dc_conf.sensor_compare_mode = 0;
	pph_motor_data->dc_conf.sensor_mask = 0;
	pph_motor_data->stop_in_ms = time;

	init_completion(&(pph_motor_data->motor_completion));
	pph_motor_data->moving_status = 0;

	return 0;
}
EXPORT_SYMBOL_GPL(tp_eng_ph_motor_config);

int tp_eng_ph_motor_start(struct ph_motor_data_t * pph_motor_data)
{
	int ret = 0;
	ret = dcmotor_set_config(pph_motor_data->pdcmotor, &(pph_motor_data->dc_conf));
	if(ret)
	{
		printk(KERN_ERR "tp_engine_dcmotor_start: steppermotor_set_config error!\n");
		return -RES_PRINTING_UNKOWN_ERROR;
	}
	ret = dcmotor_start(pph_motor_data->pdcmotor);
	if(ret)
	{
		printk(KERN_ERR "tp_eng_stepmotor_start: steppermotor_start error!\n");
		return -RES_PRINTING_PUSH_MOTOR_NOT_WORK;
	}
	pph_motor_data->moving_status = 1;
	return 0;
}
EXPORT_SYMBOL_GPL(tp_eng_ph_motor_start);

int tp_eng_ph_motor_wait_stoped(struct ph_motor_data_t * pph_motor_data)
{
	int ret = 0;
	if(pph_motor_data->moving_status == 0)
	{
		return 0;
	}
	ret = wait_for_completion_timeout(&(pph_motor_data->motor_completion), pph_motor_data->stop_in_ms);
	if(!ret)
	{
		printk(KERN_ERR "tp_eng_stepmotor_wait_stoped timeout!\n");
		pph_motor_data->moving_status = 0;
		complete_all(&(pph_motor_data->motor_completion));
		dcmotor_stop(pph_motor_data->pdcmotor);
		return -RES_PRINTING_PUSH_MOTOR_ERROR;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(tp_eng_ph_motor_wait_stoped);

