/*
 * image scanning unit device driver data structure definitions
 *
 * Copyright 2016 Hunan GreatWall Information Financial Equipment Co., Ltd.
 *
 */
#ifndef __SCANCTRL_H__
#define __SCANCTRL_H__

// data structure of scanning control block
typedef struct scandrv_control_s
{
	int mode;			// scanning mode
	int state;			// scanning state
	int scanflag;			// scanning control flags
	int scanlines;			// expected scanning lines
	volatile int linecount;		// scanning lines count
	unsigned char *buffer;		// scanning buffer
	unsigned char *buffptr;		// scanning buffer pointer
	unsigned int linedlen;		// scanning line data length
	struct completion scan_completion;
	int cnt;
	int start_flag;

} scandrv_control_t;

extern scandrv_control_t scandrv_ctrl;
extern void imgcpy_do_tasklet(unsigned long para);
#endif /* __SCANCTRL_H__ */
