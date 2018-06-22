/*
 * GWI Thermal Printer Printer Engine Paper Motor.
 *
 * Copyright 2018 Hunan GreatWall Information Financial Equipment Co., Ltd.
 *
 * Licensed under the GPL-2.
*/
#include "tp_engine_pap_motor.h"
#include "tp_engine.h"
#include "command.h"
#include "../motor/steppermotor.h"

#define COMPLETION_TIMEOUT(steps)	msecs_to_jiffies(steps*5)

extern void tp_eng_pap_motor_complete_callback(struct steppermotor *motor, struct callback_data *data);
extern void tp_eng_pap_motor_per_step_callback(struct steppermotor *motor, struct callback_data *data);

/********************************************************
 * LOCAL FUNCTIONS
 ********************************************************/


/********************************************************
 * GLOBAL FUNCTIONS
 ********************************************************/
int tp_eng_pap_motor_config(struct pap_motor_data_t * ppap_motor_data,
			    int step, motion_dir dir, int num_speed, struct speed_info *speedinfo
			    )
{
	printk(KERN_DEBUG "tp_eng_pap_motor_config.\n");
	ppap_motor_data->step_conf.dir = dir;
	ppap_motor_data->step_conf.num_speed = num_speed;
	if(step < 0)
	{
		step = -step;
	}
	ppap_motor_data->step_conf.steps_to_run = step;
	ppap_motor_data->step_conf.speedinfo = speedinfo;
	return 0;
}
EXPORT_SYMBOL_GPL(tp_eng_pap_motor_config);

int tp_eng_pap_motor_set_callback(struct pap_motor_data_t * ppap_motor_data,
				  pap_motor_callback_fun callback_complete, struct callback_data * pcallbackdata_comp, 
				  pap_motor_callback_fun callback_per_step, struct callback_data * pcallbackdata_step
				  )
{
	ppap_motor_data->callback_complete = callback_complete;
	ppap_motor_data->callback_per_step = callback_per_step;
	memcpy(&ppap_motor_data->callbackdata_complete, pcallbackdata_comp, sizeof(struct callback_data));
	memcpy(&ppap_motor_data->callbackdata_per_step, pcallbackdata_step, sizeof(struct callback_data));
	/* 将pap_motor_data_t作为stepmotor参数带如下级的结构,用于回调函数使用 */
	ppap_motor_data->pstepmotor->callbackdata.data1 = (int)ppap_motor_data;
	ppap_motor_data->pstepmotor->callbackdata_per_step.data1 = (int)ppap_motor_data;
	steppermotor_set_callback(ppap_motor_data->pstepmotor, tp_eng_pap_motor_complete_callback, &ppap_motor_data->pstepmotor->callbackdata);
	steppermotor_set_callback_per_step(ppap_motor_data->pstepmotor, tp_eng_pap_motor_per_step_callback, &ppap_motor_data->pstepmotor->callbackdata_per_step);
	return 0;
}
EXPORT_SYMBOL_GPL(tp_eng_pap_motor_set_callback);

int tp_eng_pap_motor_start(struct pap_motor_data_t * ppap_motor_data)
{
	int ret = 0;
	
	printk("tp_eng_pap_motor_start.\n");
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
		return -RES_PRINTING_UNKOWN_ERROR;
	}
	ppap_motor_data->moving_status = 1;
	return 0;
}
EXPORT_SYMBOL_GPL(tp_eng_pap_motor_start);

void tp_eng_pap_motor_stop(struct pap_motor_data_t * ppap_motor_data)
{
	if(ppap_motor_data->moving_status == 0)
	{
		return;
	}
	steppermotor_stop(ppap_motor_data->pstepmotor);
	complete_all(&(ppap_motor_data->motor_completion));
	ppap_motor_data->moving_status = 0;
}
EXPORT_SYMBOL_GPL(tp_eng_pap_motor_stop);

void tp_eng_pap_motor_stop_after_steps(struct pap_motor_data_t * ppap_motor_data, unsigned int steps)
{
	steppermotor_stop_after_steps(ppap_motor_data->pstepmotor, steps);
	complete_all(&(ppap_motor_data->motor_completion));
}
EXPORT_SYMBOL_GPL(tp_eng_pap_motor_stop_after_steps);

int tp_eng_pap_motor_wait_stop(struct pap_motor_data_t * ppap_motor_data)
{
	int ret = 0;

	if(ppap_motor_data->moving_status == 0)
	{
		return 0;
	}
	ret = wait_for_completion_timeout(&(ppap_motor_data->motor_completion), COMPLETION_TIMEOUT(ppap_motor_data->step_conf.steps_to_run));
	if(!ret)
	{
		printk(KERN_ERR "tp_eng_stepmotor_wait_stoped timeout!\n");
		complete_all(&(ppap_motor_data->motor_completion));
		steppermotor_stop(ppap_motor_data->pstepmotor);
		ppap_motor_data->moving_status = 0;
		return -RES_PRINTING_UNKOWN_ERROR;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(tp_eng_pap_motor_wait_stop);

void tp_eng_pap_motor_complete_callback(struct steppermotor *motor, struct callback_data *data)
{
	struct pap_motor_data_t * ppap_motor_data;

	ppap_motor_data = (struct pap_motor_data_t *)motor->callbackdata_per_step.data1;
	if(ppap_motor_data->moving_status == 0)
	{
		return;
	}
	complete_all(&(ppap_motor_data->motor_completion));
	ppap_motor_data->moving_status = 0;
	if(ppap_motor_data->callback_complete == NULL)
	{
		return;
	}
	ppap_motor_data->callback_complete(ppap_motor_data, 
					   &ppap_motor_data->callbackdata_per_step);
}
EXPORT_SYMBOL_GPL(tp_eng_pap_motor_complete_callback);

void tp_eng_pap_motor_per_step_callback(struct steppermotor *motor, struct callback_data *data)
{
	struct pap_motor_data_t * ppap_motor_data;

	ppap_motor_data = (struct pap_motor_data_t *)motor->callbackdata_per_step.data1;
	if(ppap_motor_data->callback_per_step == NULL)
	{
		return;
	}
	ppap_motor_data->callback_per_step(ppap_motor_data, 
					   &ppap_motor_data->callbackdata_per_step);
}
EXPORT_SYMBOL_GPL(tp_eng_pap_motor_per_step_callback);

