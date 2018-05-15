/*
 * GWI Thermal Printer Printer Engine Motor.
 *
 * Copyright 2018 Hunan GreatWall Information Financial Equipment Co., Ltd.
 *
 * Licensed under the GPL-2.
*/
#ifndef _TP_ENGINE_RIBBON_MOTOR_H_
#define _TP_ENGINE_RIBBON_MOTOR_H_

#include "motor.h"
#include "../motor/steppermotor.h"
#include "../motor/dcmotor.h"

#define NAME_MAX_LEN 32

typedef void (*dcmotor_callback_fun)(struct dcmotor *, struct callback_data *);

struct ribbon_motor_data_t
{
	unsigned short motor_mask;
	unsigned char motor_name[NAME_MAX_LEN];
	struct dcmotor *pdcmotor;			//电机实例指针
	struct dcmotor_config dc_conf;			//直流电机配置
	unsigned int stop_in_ms;			//直流电机运动停止时间

	unsigned int moving_status;			//当前运动状态
	unsigned int stoping_status;			//停止状态

	struct completion motor_completion;		//电机运动完成，用于等待电机停止
	
	int err;					//错误码
	dcmotor_callback_fun callback_complete;		//动作结束时回调函数
};


void tp_eng_ribbon_motor_stop(struct ribbon_motor_data_t * pribbon_motor_data);
int tp_eng_ribbon_motor_config(struct ribbon_motor_data_t * pribbon_motor_data, motion_dir dir, unsigned int stop_in_ms);
int tp_eng_ribbon_motor_start(struct ribbon_motor_data_t * pribbon_motor_data);
int tp_eng_ribbon_motor_wait_stoped(struct ribbon_motor_data_t * pribbon_motor_data);


#endif