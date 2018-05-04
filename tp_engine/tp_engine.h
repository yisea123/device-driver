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

/* ioctl command define */
#define TP_ENG_IOCTL_PH_UP		1	//打印头抬起
#define TP_ENG_IOCTL_PH_DOWN		2	//打印头压下
#define TP_ENG_IOCTL_PH_RST		3	//打印头复位
#define TP_ENG_IOCTL_PRINT		4	//打印数据
#define TP_ENG_IOCTL_PAP_IN		5	//进纸
#define TP_ENG_IOCTL_PAP_OUT		6	//出纸
#define TP_ENG_IOCTL_PAP_MOVE		7	//走纸
#define TP_ENG_IOCTL_RIBBON_RUN		8	//走碳带
#define TP_ENG_IOCTL_SENSOR_ST		9	//查询传感器状态

struct tp_engine_t
{
	struct device * dev;
	char tp_engine_name[TP_ENGINE_NAME_MAX_SIZE];
	int eng_st;
	
	int ph_num;				//engine中打印头数量
	struct ph_data_t * pph_data;		//engine中打印头数据指针
	int ph_resistor_num;			//engine中热敏电阻数量
	struct ph_resistor_data_t * pph_resistor_data;	//engine中热敏点数数据指针
	int motor_num;				//engine中电机数量
	struct motor_data_t * pmotor_data;	//engine中电机数据指针
	int sensor_num;				//engine中传感器数量
	struct sensor_data_t * psensor_data;	//engine中传感器数据指针
};


#endif