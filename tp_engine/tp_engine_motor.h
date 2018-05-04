/*
 * GWI Thermal Printer Printer Engine Motor.
 *
 * Copyright 2018 Hunan GreatWall Information Financial Equipment Co., Ltd.
 *
 * Licensed under the GPL-2.
*/
#ifndef _TP_ENGINE_MOTOR_H_
#define _TP_ENGINE_MOTOR_H_

#include "motor.h"
#include "../motor/steppermotor.h"
#include "../motor/dcmotor.h"

#define NAME_MAX_LEN 32

typedef enum
{
	MOTOR_TYPE_STEP = 0,
	MOTOR_TYPE_BRUSHDC = 1,
	MOTOR_TYPE_NOBRUSHDC = 2,
}motor_type_t;

typedef union
{
	void (*dcmotor_callback)(struct dcmotor *, struct callback_data *);
	void (*steppermotor_callback)(struct steppermotor *, struct callback_data *);
}motor_callback_t;

struct motor_data_t
{
	unsigned short motor_mask;
	unsigned char motor_name[NAME_MAX_LEN];
	motor_type_t motor_type;			//电机类型step or dc
	union
	{
		struct steppermotor *pstepmotor;
		struct dcmotor *pdcmotor;
	}motor_dev;					//电机实例
	struct steppermotor_config step_conf;		//步进电机配置
	struct dcmotor_config dc_conf;			//直流电机配置
	
	int err;					//错误码
	motor_callback_t complete_callback;		//动作结束时回调函数
};

#endif