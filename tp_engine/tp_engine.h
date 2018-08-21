/*
 * GWI Thermal Printer Printer Engine Driver.
 *
 * Copyright 2018 Hunan GreatWall Information Financial Equipment Co., Ltd.
 *
 * Licensed under the GPL-2.
*/

#ifndef __TP_ENGINE_H__
#define __TP_ENGINE_H__

#include <linux/timer.h>

#define TP_ENGINE_NAME_MAX_SIZE		32

#define TP_ENG_IOC_MAGIC        0xef

#ifdef	__KERNEL__
#define TP_ENG_CTRL_CMD	(0)
#else
#define	TP_ENG_CTRL_CMD		_IO(TP_ENG_IOC_MAGIC , 0 )
#endif

#define RESISTOR			0

/* ioctl command define */
#define TP_ENG_IOCTL_INIT		(TP_ENG_CTRL_CMD+0)	//初始化
#define TP_ENG_IOCTL_RESET		(TP_ENG_CTRL_CMD+1)	//复位
#define TP_ENG_IOCTL_SET_ENG_CONFIG	(TP_ENG_CTRL_CMD+2)	//配置eng
#define TP_ENG_IOCTL_PH_UP_DOWN		(TP_ENG_CTRL_CMD+3)	//打印头压下
#define TP_ENG_IOCTL_PH_RST		(TP_ENG_CTRL_CMD+4)	//打印头复位
#define TP_ENG_IOCTL_PRINT		(TP_ENG_CTRL_CMD+5)	//打印数据
#define TP_ENG_IOCTL_PAP_IN		(TP_ENG_CTRL_CMD+6)	//进纸
#define TP_ENG_IOCTL_PAP_OUT		(TP_ENG_CTRL_CMD+7)	//出纸
#define TP_ENG_IOCTL_PAP_MOVE		(TP_ENG_CTRL_CMD+8)	//走纸
#define TP_ENG_IOCTL_RIBBON_RUN		(TP_ENG_CTRL_CMD+9)	//走碳带
#define TP_ENG_IOCTL_SENSOR_ST		(TP_ENG_CTRL_CMD+10)	//查询传感器状态
#define TP_ENG_IOCTL_SENSOR_ENABLE	(TP_ENG_CTRL_CMD+11)	//使能传感器
#define TP_ENG_IOCTL_GET_SEN_VAL	(TP_ENG_CTRL_CMD+12)	//获得传感器真实值
#define TP_ENG_IOCTL_SENSOR_CONFIG	(TP_ENG_CTRL_CMD+13)	//配置传感器
#define TP_ENG_IOCTL_GET_SENLOGIC_VAL	(TP_ENG_CTRL_CMD+14)	//查询传感器逻辑状态
#define TP_ENG_IOCTL_SET_PH_CONFIG	(TP_ENG_CTRL_CMD+15)	//配置打印头参数
#define TP_ENG_IOCTL_GET_PAP_LENGHT	(TP_ENG_CTRL_CMD+16)	//获得纸张长度
#define TP_ENG_IOCTL_GET_ENG_CONFIG	(TP_ENG_CTRL_CMD+18)	//获得ENG配置
#define TP_ENG_IOCTL_GET_PH_CONFIG	(TP_ENG_CTRL_CMD+19)	//获得打印头配置
#define TP_ENG_IOCTL_SENSOR_LOGIC_ST	(TP_ENG_CTRL_CMD+20)	//获得打印机状态信息
#define TP_ENG_IOCTL_RESISTOR_VAL	(TP_ENG_CTRL_CMD+21)	//获得打印头温度

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
	unsigned char ph_down;
	unsigned char mac_close;
	unsigned long ph_resistor_val;		//打印头热敏电阻阻值adc
};

#pragma pack(1)

/* bmp文件头 */
typedef struct tagBITMAPFILEHEADER {
	uint16_t   bfType;		//"BM"表示windows位图标志
	uint32_t   bfSize;		//整个bmp文件大小
	uint16_t   bfReserved1;		//0
	uint16_t   bfReserved2;		//0
	uint32_t   bfOffBits;		//文件起始位置到图像像素数据的字节偏移量
} BITMAPFILEHEADER;

/* bmp信息头 */
typedef struct tagBITMAPINFOHEADER{
	uint32_t   biSize;		//INFOHEADER结构体大小
	int32_t    biWidth;		//图像宽度
	int32_t    biHeight;		//图像高度
	uint16_t   biPlanes;		//图像数据平面,BMP存储RGB数据,因此总为1
	uint16_t   biBitCount;		//图像像素位数
	uint32_t   biCompression;	//0:不压缩, 1:RLE8, 2:RLE4
	uint32_t   biSizeImage;		//4字节对齐的图像数据大小
	int32_t    biXPelsPerMeter;	//说明水平分辨率，用像素/米表示，有符号整数
	int32_t    biYPelsPerMeter;	//说明垂直分辨率，用像素/米表示，有符号整数
	uint32_t   biClrUsed;		//实际使用的调色板索引数，0：使用所有的调色板索引
	uint32_t   biClrImportant;	//重要的调色板索引数，0：所有的调色板索引都重要
} BITMAPINFOHEADER;

typedef struct tagBITMAPCOREHEADER {
	uint32_t   bcSize;
	uint16_t   bcWidth;
	uint16_t   bcHeight;
	uint16_t   bcPlanes;
	uint16_t   bcBitCount;
} BITMAPCOREHEADER;

typedef struct tagRGBTRIPLE {
	uint8_t    rgbtBlue;
	uint8_t    rgbtGreen;
	uint8_t    rgbtRed;
} RGBTRIPLE;

#pragma pack()

//将要打印的bmp数据结构体
struct tp_engine_bmp_data
{
	unsigned char * buff;
	unsigned int data_size;
	struct tagBITMAPFILEHEADER bmp_header;
	struct tagBITMAPINFOHEADER bmp_info;

	unsigned char * p_cur_c;	//当前字符指针
	unsigned int cur_char_index;	//当前字符所在位置
	unsigned int left_char_size;	//剩余字符个数
};

struct tp_engine_config_t
{
	int paper_in_speed;
	int printing_speed;
	int paper_out_speed;
};

struct tp_engine_pap_info_t
{
	int pap_length;		//纸张长度
	int pap_pos;		//纸张位置，相对与打印位置，处于打印位置为0，负数表示还未到打印位置
	int pap_total_flag;	//是否获得整个纸张长度标志，仅当为1时，允许返回纸张长度
};

struct tp_engine_ribbon_info_t
{
	int ribbon_exsit;		//碳带有无
	int ribbon_broken_black;	//碳带断裂---黑块
	int ribbon_broken_white;	//碳带断裂---白块
};

struct tp_engine_t
{
	struct device * dev;
	char tp_engine_name[TP_ENGINE_NAME_MAX_SIZE];
	struct tp_engine_state_t eng_state;		//用于控制各种状态变化的结构体
	struct tp_engine_sen_st_t tp_eng_sen_st;	//传感器状态结构体
	struct tp_engine_bmp_data bmp_data;		//打印bmp数据结构体
	struct tp_engine_config_t config;               //engine　配置
	struct tp_engine_pap_info_t pap_info_st;	//纸张位置信息
	struct tp_engine_ribbon_info_t ribbon_info_st;	//碳带信息
	
	struct ph_data_t * pph_data;			//engine中打印头数据指针
	struct ph_resistor_data_t * pph_resistor_data;	//engine中热敏电阻数据指针
	struct pap_motor_data_t * ppap_motor_data;	//engine中走纸电机
	struct ph_motor_data_t * pph_motor_data;	//engine中打印头电机
	struct ribbon_motor_data_t * pribbon_motor_data;	//engine中碳带电机

	int sensor_num;					//engine中传感器数量
	unsigned int sensor_masks;			//engine中多个传感器masks
	struct sensor_data_t * psensor_data;		//engine中传感器数据指针
	
	struct workqueue_struct * print_go_workqueue;
	struct work_struct print_go_work;
};


#endif
