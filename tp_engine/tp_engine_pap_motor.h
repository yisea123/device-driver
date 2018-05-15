/*
 * GWI Thermal Printer Printer Engine Motor.
 *
 * Copyright 2018 Hunan GreatWall Information Financial Equipment Co., Ltd.
 *
 * Licensed under the GPL-2.
*/
#ifndef _TP_ENGINE_PAP_MOTOR_H_
#define _TP_ENGINE_PAP_MOTOR_H_

#include "motor.h"
#include "../motor/steppermotor.h"
#include "../motor/dcmotor.h"

#define NAME_MAX_LEN 32


typedef void (*stepmotor_callback_fun)(struct steppermotor *, struct callback_data *);

struct pap_motor_data_t
{
	unsigned short motor_mask;
	unsigned char motor_name[NAME_MAX_LEN];
	struct steppermotor *pstepmotor;		//电机实例指针
	struct steppermotor_config step_conf;		//步进电机配置

	unsigned int moving_status;			//当前运动状态
	unsigned int stoping_status;			//停止状态

	struct completion motor_completion;		//电机运动完成，用于等待电机停止
	
	int err;					//错误码
	stepmotor_callback_fun callback_complete;	//动作结束时回调函数
	struct callback_data complete_callback_data;
	stepmotor_callback_fun callback_per_step;	//每步的回调函数
};

int tp_eng_pap_motor_config(struct pap_motor_data_t * ppap_motor_data,
			    int step, motion_dir dir, int num_speed, struct speed_info *speedinfo, 
			    stepmotor_callback_fun callback_complete, struct callback_data * pcallbackdata_comp, 
			    stepmotor_callback_fun callback_per_step, struct callback_data * pcallbackdata_step
			    );
int tp_eng_pap_motor_start(struct pap_motor_data_t * ppap_motor_data);
void tp_eng_pap_motor_stop(struct pap_motor_data_t * ppap_motor_data);
int tp_eng_pap_motor_wait_stop(struct pap_motor_data_t * ppap_motor_data);

#endif