/*
 * GWI Thermal Printer Printer Engine Driver.
 *
 * Copyright 2018 Hunan GreatWall Information Financial Equipment Co., Ltd.
 *
 * Licensed under the GPL-2.
*/

#ifndef __TP_ENGINE_H__
#define __TP_ENGINE_H__

#define TP_ENGINE_NAME_MAX_SIZE		32

#define TP_ENG_IOC_MAGIC        0xef

#ifdef	__KERNEL__
#define TP_ENG_CTRL_CMD	(0)
#else
#define	TP_ENG_CTRL_CMD		_IO(TP_ENG_IOC_MAGIC , 0 )
#endif

/* ioctl command define */
#define TP_ENG_IOCTL_INIT		(TP_ENG_CTRL_CMD+0)	//初始化
#define TP_ENG_IOCTL_RESET		(TP_ENG_CTRL_CMD+1)	//复位
#define TP_ENG_IOCTL_PH_UP_DOWN		(TP_ENG_CTRL_CMD+2)	//打印头压下
#define TP_ENG_IOCTL_PH_RST		(TP_ENG_CTRL_CMD+3)	//打印头复位
#define TP_ENG_IOCTL_PRINT		(TP_ENG_CTRL_CMD+4)	//打印数据
#define TP_ENG_IOCTL_PAP_IN		(TP_ENG_CTRL_CMD+5)	//进纸
#define TP_ENG_IOCTL_PAP_OUT		(TP_ENG_CTRL_CMD+6)	//出纸
#define TP_ENG_IOCTL_PAP_MOVE		(TP_ENG_CTRL_CMD+7)	//走纸
#define TP_ENG_IOCTL_RIBBON_RUN		(TP_ENG_CTRL_CMD+8)	//走碳带
#define TP_ENG_IOCTL_SENSOR_ST		(TP_ENG_CTRL_CMD+9)	//查询传感器状态
#define TP_ENG_IOCTL_SENSOR_ENABLE	(TP_ENG_CTRL_CMD+10)	//使能传感器
#define TP_ENG_IOCTL_GET_SEN_VAL	(TP_ENG_CTRL_CMD+11)	//获得传感器真实值
#define TP_ENG_IOCTL_SENSOR_CONFIG	(TP_ENG_CTRL_CMD+12)	//配置传感器
#define TP_ENG_IOCTL_GET_SENLOGIC_VAL	(TP_ENG_CTRL_CMD+13)	//查询传感器逻辑状态

typedef enum
{
	MOTOR_MOVE_STATUS_INIT = 0x01,
	MOTOR_MOVE_STATUS_INUSE = 0x02,
	MOTOR_MOVE_STATUS_RUNNING = 0x04,
	MOTOR_MOVE_STATUS_STOP = 0x08,
	MOTOR_STOP_BY_SOFT_START = 0x10,
}motor_stop_status_t;

//走纸电机的各种状态
typedef enum
{
	PAP_MOTOR_STATE_STOP = 0,
	PAP_MOTOR_STATE_IN,
	PAP_MOTOR_STATE_OUT,
	PAP_MOTOR_STATE_PRINT,
	PAP_MOTOR_STATE_MOVE,
}pap_motor_state_t;

typedef enum
{
	PH_MOTOR_STATE_STOP = 0,
	PH_MOTOR_STATE_UP,
	PH_MOTOR_STATE_DOWN,
}ph_motor_state_t;

typedef enum
{
	RIBBON_MOTOR_STATE_STOP = 0,
	RIBBON_MOTOR_STATE_RUN,
}ribbon_motor_state_st;

/* tp_engine state */
struct tp_engine_state_t
{
	pap_motor_state_t pap_motor_state;	//用于表示走纸电机目前状态，中断函数中根据它来判断下一步
	ph_motor_state_t ph_motor_state;	//用于表示打印头电机目前状态，中断函数中根据它来判断下一步
	ribbon_motor_state_st ribbon_motor_state;
};

/* 用于记录传感器状态变化 */
struct tp_engine_sen_st_t
{
	unsigned char pap_in;
	unsigned char pap_out;
	unsigned char ribbon_exsit;
	unsigned char ribbon_broken;
	unsigned char ph_up;
	unsigned char mac_close;
};

struct tp_engine_t
{
	struct device * dev;
	char tp_engine_name[TP_ENGINE_NAME_MAX_SIZE];
	int eng_st;
	struct tp_engine_state_t eng_state;		//用于控制各种状态变化的结构体
	struct tp_engine_sen_st_t tp_eng_sen_st;	//传感器状态结构体
	
	struct ph_data_t * pph_data;			//engine中打印头数据指针
	struct ph_resistor_data_t * pph_resistor_data;	//engine中热敏点数数据指针
	struct pap_motor_data_t * ppap_motor_data;	//engine中走纸电机
	struct ph_motor_data_t * pph_motor_data;	//engine中打印头电机
	struct ribbon_motor_data_t * pribbon_motor_data;	//engine中碳带电机

	int sensor_num;					//engine中传感器数量
	unsigned int sensor_masks;			//engine中多个传感器masks
	struct sensor_data_t * psensor_data;		//engine中传感器数据指针

};


#endif
