/*
 * GWI Thermal Printer Printer Engine Paper Motor.
 *
 * Copyright 2018 Hunan GreatWall Information Financial Equipment Co., Ltd.
 *
 * Licensed under the GPL-2.
*/

#include "tp_engine_pap_motor.h"
#include "tp_engine.h"


/********************************************************
 * LOCAL FUNCTIONS
 ********************************************************/


/********************************************************
 * GLOBAL FUNCTIONS
 ********************************************************/
void tp_eng_pap_motor_stop(struct pap_motor_data_t * ppap_motor_data)
{
	steppermotor_stop(ppap_motor_data->pstepmotor);
}

int tp_eng_pap_motor_config(struct pap_motor_data_t * ppap_motor_data,
			    int step, motion_dir dir, int num_speed, struct speed_info *speedinfo, 
			    motor_callback_t callback_complete, struct callback_data * pcallbackdata_comp, 
			    motor_callback_t callback_per_step, struct callback_data * pcallbackdata_step)
{
	ppap_motor_data->step_conf.dir = dir;
	ppap_motor_data->step_conf.num_speed = num_speed;
	ppap_motor_data->step_conf.steps_to_run = step;
	ppap_motor_data->step_conf.speedinfo = speedinfo;
	return 0;
}
