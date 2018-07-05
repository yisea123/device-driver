#ifndef __COMMAND_H__
#define __COMMAND_H__


#define TP_PH_TIME_OF_HEATING_DEF	300		//us,默认加热时间
#define TP_PH_DOT_IN_LINE		1280		//一行点数
#define TP_PH_BUFFER_SIZE		(1280/8)	//点数据缓冲区
#define TP_PH_FREQ_HZ			16000000	//Hz
#define TP_PH_DELAY_AFTER_LATCH_LOW	1		//us
#define TP_PH_DELAY_AFTER_DATA_IN	1		//us
#define TP_PH_DELAY_AFTER_LATCH_HIGH	1		//us

#define SPEED_0		800
#define SPEED_1		1000
#define SPEED_2		1200
#define SPEED_3		1350
#define SPEED_4		1500
#define SPEED_5		1650
#define SPEED_6		1800
#define SPEED_7		2254
#define SPEED_8		2400
#define SPEED_9		2800

#define SPEED_PRINTING	SPEED_2
#define SPEED_PAP_IN	SPEED_8
#define SPEED_PAP_OUT	SPEED_8


#define DEVICE_ID	(0x02020202)	// (always be 0x02020202)

#define MAX_PACKET_LEN	(1024 * 1024 * 4)
#define MAX_DATA_LEN	(MAX_PACKET_LEN - sizeof(comm_packet_head_t))


#pragma pack(1)		// set one-byte alignment


#pragma pack()		// restore default alignment


/* macro for command word definition */
#define CMD_TO_HEX(b1, b2)		(unsigned short)(((b2)<<8)+(b1))
#define CMD_BYTE1(cmd)			(unsigned char)(cmd)
#define CMD_BYTE2(cmd)			(unsigned char)(cmd>>8)

int cmd_get_end_str(void);
int cmd_code(unsigned char ch);
int cmd_M(unsigned char crt);
int cmd_H(unsigned char crt);
int cmd_E(unsigned char crt);
int cmd_G(unsigned char crt);
int cmd_Q(unsigned char crt);
int cmd_N(unsigned char crt);
int cmd_W(unsigned char crt);
int cmd_S(unsigned char crt);
int cmd_exclusive_or(unsigned char crt);


#define RESN_OFFSET	0x00
//-----RESN_CLASS_APP_CUSTOM-----
#define RESN_OFFSET_APP_CUSTOM		(RESN_OFFSET+0x100)

#define RESN_APP_CUSTOM_CODE(x)		(RESN_OFFSET_APP_CUSTOM+x)

/* positive response status definitions */
#define RES_NO_MEDIA			0x0000		// no media in printer 
#define RES_MEDIA_AT_IN			0x0001		// media detected at in side
#define RES_MEDIA_AT_OUT		0x0002		// media detected at out side

/* printer respond */
#define RES_MEDIA_JAM			0x0015		// media jam
#define RES_PRINTING_PAP_INSERT_ERROR	RESN_APP_CUSTOM_CODE(0x0081)		//进纸或无纸错误
#define RES_PRINTING_PAP_OUT_ERROR	RESN_APP_CUSTOM_CODE(0x0082)		//出纸错误
#define RES_PRINTING_PH_DOWN_ERROR	RESN_APP_CUSTOM_CODE(0x0083)		//打印头上台错误
#define RES_PRINTING_PH_UP_ERROR	RESN_APP_CUSTOM_CODE(0x0084)		//打印头下压错误
#define RES_PRINTING_HAVE_PAPER		RESN_APP_CUSTOM_CODE(0x0085)		//已经有纸错误
#define RES_PRINTING_INIT_ERROR		RESN_APP_CUSTOM_CODE(0x0086)		//初始化错误
#define RES_PRINTING_RINBBON_BROKEN	RESN_APP_CUSTOM_CODE(0x0087)		//色带断裂
#define RES_PRINTING_RD_ERROR		RESN_APP_CUSTOM_CODE(0x0090)		//读数据错误
#define RES_PRINTING_WR_ERROR		RESN_APP_CUSTOM_CODE(0x0091)		//写数据错误
#define RES_PRINTING_RINBBON_WHELL	RESN_APP_CUSTOM_CODE(0x0092)		//色带张紧轮错误或传感器异常
#define RES_PRINTING_PUSH_MOTOR_NOT_WORK	RESN_APP_CUSTOM_CODE(0x0093)	//下压电机不工作
#define RES_PRINTING_PUSH_MOTOR_ERROR	RESN_APP_CUSTOM_CODE(0x0094)		//下压电机故障
#define RES_PRINTING_NO_PAP_ERROR	RESN_APP_CUSTOM_CODE(0x0095)		//打印时无纸错误
#define RES_PRINTING_UNKOWN_ERROR	RESN_APP_CUSTOM_CODE(0x0096)		//打印未知错误

#define RES_PRINTING_MODEL_NOT_CLOSE	RESN_APP_CUSTOM_CODE(0x0099)		//打印机模块未闭合
#define RES_PRINTING_RINBON_END		RESN_APP_CUSTOM_CODE(0x009a)		//碳带将尽



#endif
