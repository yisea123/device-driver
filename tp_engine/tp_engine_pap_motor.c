/*
 * GWI Thermal Printer Printer Engine Paper Motor.
 *
 * Copyright 2018 Hunan GreatWall Information Financial Equipment Co., Ltd.
 *
 * Licensed under the GPL-2.
*/

#include "tp_engine_pap_motor.h"
#include "tp_engine.h"
#include "../motor/steppermotor.h"

#define COMPLETION_TIMEOUT(steps)	(steps/2)

/********************************************************
 * LOCAL FUNCTIONS
 ********************************************************/


/********************************************************
 * GLOBAL FUNCTIONS
 ********************************************************/
int tp_eng_pap_motor_config(struct pap_motor_data_t * ppap_motor_data,
			    int step, motion_dir dir, int num_speed, struct speed_info *speedinfo, 
			    stepmotor_callback_fun callback_complete, struct callback_data * pcallbackdata_comp, 
			    stepmotor_callback_fun callback_per_step, struct callback_data * pcallbackdata_step,
			    struct callback_data * pdata_callback)
{
	printk(KERN_DEBUG "tp_eng_pap_motor_config. \n");
	ppap_motor_data->step_conf.dir = dir;
	ppap_motor_data->step_conf.num_speed = num_speed;
	ppap_motor_data->step_conf.steps_to_run = step;
	ppap_motor_data->step_conf.speedinfo = speedinfo;
	ppap_motor_data->callback_complete = callback_complete;
	ppap_motor_data->callback_per_step = callback_per_step;
	memcpy(&ppap_motor_data->complete_callback_data, pdata_callback, sizeof(struct callback_data));
	return 0;
}
EXPORT_SYMBOL_GPL(tp_eng_pap_motor_config);

int tp_eng_pap_motor_start(struct pap_motor_data_t * ppap_motor_data)
{
	int ret = 0;
	
	ret = steppermotor_set_config(ppap_motor_data->pstepmotor, &ppap_motor_data->step_conf);
	if(ret)
	{
		printk(KERN_ERR "tp_eng_pap_motor_start: steppermotor_set_config error!\n");
		return -1;
	}
	init_completion(&(ppap_motor_data->motor_completion));
	ret = steppermotor_start(ppap_motor_data->pstepmotor);
	if(ret)
	{
		printk(KERN_ERR "tp_eng_pap_motor_start: steppermotor_start error!\n");
		complete_all(&(ppap_motor_data->motor_completion));
		return -1;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(tp_eng_pap_motor_start);

void tp_eng_pap_motor_stop(struct pap_motor_data_t * ppap_motor_data)
{
	steppermotor_stop(ppap_motor_data->pstepmotor);
	complete_all(&(ppap_motor_data->motor_completion));
}
EXPORT_SYMBOL_GPL(tp_eng_pap_motor_stop);

int tp_eng_pap_motor_wait_stop(struct pap_motor_data_t * ppap_motor_data)
{
	int ret = 0;

	ret = wait_for_completion_timeout(&(ppap_motor_data->motor_completion), COMPLETION_TIMEOUT(ppap_motor_data->step_conf.steps_to_run));
	if(!ret)
	{
		printk(KERN_ERR "tp_eng_stepmotor_wait_stoped timeout!\n");
		complete_all(&(ppap_motor_data->motor_completion));
		steppermotor_stop(ppap_motor_data->pstepmotor);
		return -1;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(tp_eng_pap_motor_wait_stop);

int tp_eng_pap_motor_complete_callback(struct pap_motor_data_t * ppap_motor_data)
{
	complete_all(&(ppap_motor_data->motor_completion));
	return 0;
}
EXPORT_SYMBOL_GPL(tp_eng_pap_motor_complete_callback);

int tp_eng_pap_motor_per_step_callback(struct pap_motor_data_t * ppap_motor_data)
{
	return 0;
}
EXPORT_SYMBOL_GPL(tp_eng_pap_motor_per_step_callback);

