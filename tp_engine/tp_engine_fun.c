/*
 * GWI Thermal Printer Printer Engine Driver.
 *
 * Copyright 2018 Hunan GreatWall Information Financial Equipment Co., Ltd.
 *
 * Licensed under the GPL-2.
*/
#include <linux/device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/sysfs.h>
#include <asm/uaccess.h>
#include <asm/signal.h>
#include <asm/siginfo.h>
#include <linux/platform_device.h>
#include <linux/completion.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#include "tp_engine_fun.h"
#include "tp_engine_ph.h"
#include "tp_engine_pap_motor.h"
#include "tp_engine_ph_motor.h"
#include "tp_engine_ribbon_motor.h"
#include "tp_engine_sensor.h"
#include "tp_engine.h"
#include "command.h"
#include "../tp_printer_header/printer_header_common.h"
#include "../tp_printer_header/tp_printer_header.h"

#define IMAGE_ROTATE	1

#define STEP_PAP_IN	4000
#define STEP_PAP_OUT	4000

#define PH_MOTOR_TIME_OUT	1000

/* 1 step = 25.4mm/300 = 0.084666667 */
#define PAP_IN_STOP_AFTER_STEPS		326
#define PAP_OUT_STOP_AFTER_STEPS	(278*3/2)

#define to_eng_dev(ptp_eng)	container_of(ptp_eng, struct tp_engine_dev_t, tp_engine)

static struct tp_engine_t * ptp_eng_backup = NULL;
static struct work_struct pap_in_wq;
static void tp_eng_fun_pap_in_do_work(struct work_struct * work)
{
	if (ptp_eng_backup)
	{
		tp_eng_fun_sensor_update(ptp_eng_backup);
	}
}

static void tp_eng_fun_pap_in_callback(struct pap_motor_data_t *ppap_motor_data, struct callback_data *pcallback_data)
{
	struct tp_engine_t * ptp_eng;
	int step_lost = 0;

	ptp_eng = (struct tp_engine_t *)pcallback_data->data1;
	if (ptp_eng_backup != ptp_eng)
	{
		ptp_eng_backup = ptp_eng;
	}
	step_lost = steppermotor_get_running_steps(ppap_motor_data->pstepmotor);
	if ((step_lost % 2 == 0) && ((step_lost & STEP_MOTOR_STEPS_STOP_FLAG) == 0))
	{
		schedule_work(&pap_in_wq);
	}
	if (ptp_eng->tp_eng_sen_st.pap_in)
	{
		if (ptp_eng->eng_state.pap_motor_state == PAP_MOTOR_STATE_IN)
		{
			printk(KERN_DEBUG "tp_eng_fun_pap_in_callback stop after %d.\n", PAP_IN_STOP_AFTER_STEPS);
			ptp_eng->eng_state.pap_motor_state = PAP_MOTOR_STATE_STOP;
			tp_eng_pap_motor_stop_after_steps(ptp_eng->ppap_motor_data, PAP_IN_STOP_AFTER_STEPS);
		}
		ptp_eng->pap_info_st.pap_length++;
	}
}


int tp_eng_fun_config(struct tp_engine_t * ptp_eng, struct tp_engine_config_t *pconfig)
{
	int ret = 0;

	memcpy(&ptp_eng->config, pconfig, sizeof(struct tp_engine_config_t));
	return ret;
}
EXPORT_SYMBOL_GPL(tp_eng_fun_config);

int tp_eng_fun_pap_in(struct tp_engine_t * ptp_eng)
{
	struct pap_motor_data_t *ppap_motor_data;
	motion_dir dir;
	int ret, step = STEP_PAP_IN;
	struct speed_info spd_info;
	struct callback_data clbk_data;
	
	ppap_motor_data = ptp_eng->ppap_motor_data;
	memset(&clbk_data, 0, sizeof(clbk_data));
	clbk_data.data1 = (int)ptp_eng;
	tp_eng_fun_sensor_update(ptp_eng);
	if (ptp_eng->tp_eng_sen_st.pap_in)
	{
		printk(KERN_DEBUG "paper already in.\n");
		return 0;
	}
	else
	{
		ptp_eng->pap_info_st.pap_length = 0;
		ptp_eng->pap_info_st.pap_pos = 0;
		ptp_eng->pap_info_st.pap_total_flag = 0;
	}
	//print header up
	if(tp_eng_fun_ph_move(ptp_eng, 2))
	{
		printk(KERN_ERR "tp_eng_fun_ph_up up error.\n");
		return -RES_PRINTING_PUSH_MOTOR_ERROR;
	}
	if (step >= 0)
	{
		dir = MOTION_CLOCKWISE;
	}
	else
	{
		dir = MOTION_COUNTERCLOCKWISE;
		step = -step;
	}
	if (ptp_eng->config.paper_in_speed)
	{
		spd_info.speed = ptp_eng->config.paper_in_speed;
	}
	else
	{
		spd_info.speed = SPEED_PAP_IN;
	}
	spd_info.steps = step;
	spd_info.nextspeed = NULL;
	ret = tp_eng_pap_motor_config(ppap_motor_data, step, dir, 1, &spd_info);
	if (ret < 0)
	{
		return ret;
	}
	INIT_WORK(&pap_in_wq, tp_eng_fun_pap_in_do_work);
	ret = tp_eng_pap_motor_set_callback(ppap_motor_data, NULL, &clbk_data, tp_eng_fun_pap_in_callback, &clbk_data);
	if (ret < 0)
	{
		return ret;
	}
	ret = tp_eng_pap_motor_start(ppap_motor_data);
	if (ret < 0)
	{
		return ret;
	}
	ptp_eng->eng_state.pap_motor_state = PAP_MOTOR_STATE_IN;
	ret = tp_eng_pap_motor_wait_stop(ppap_motor_data);
	if (ret < 0)
	{
		return ret;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(tp_eng_fun_pap_in);

static struct work_struct pap_out_wq;
static void tp_eng_fun_pap_out_do_work(struct work_struct * work)
{
	if (ptp_eng_backup)
	{
		tp_eng_fun_sensor_update(ptp_eng_backup);
	}
}

static void tp_eng_fun_pap_out_callback(struct pap_motor_data_t *ppap_motor_data, struct callback_data *pcallback_data)
{
	struct tp_engine_t * ptp_eng;
	int step_lost = 0;

	ptp_eng = (struct tp_engine_t *)pcallback_data->data1;
	if (ptp_eng_backup != ptp_eng)
	{
		ptp_eng_backup = ptp_eng;
	}
	step_lost = steppermotor_get_running_steps(ppap_motor_data->pstepmotor);
	if ((step_lost % 2 == 0) && ((step_lost & STEP_MOTOR_STEPS_STOP_FLAG) == 0))
	{
		schedule_work(&pap_out_wq);
	}
	if ((!ptp_eng->tp_eng_sen_st.pap_out) && (!ptp_eng->tp_eng_sen_st.pap_in))
	{
		if (ptp_eng->eng_state.pap_motor_state == PAP_MOTOR_STATE_OUT)
		{
			printk(KERN_DEBUG "tp_eng_fun_pap_out_callback stop after %d.\n", PAP_OUT_STOP_AFTER_STEPS);
			ptp_eng->eng_state.pap_motor_state = PAP_MOTOR_STATE_STOP;
			tp_eng_pap_motor_stop_after_steps(ptp_eng->ppap_motor_data, PAP_OUT_STOP_AFTER_STEPS);
		}
		ptp_eng->pap_info_st.pap_length = 0;
		ptp_eng->pap_info_st.pap_pos = 0;
		ptp_eng->pap_info_st.pap_total_flag = 0;
	}
}

int tp_eng_fun_pap_out(struct tp_engine_t * ptp_eng)
{
	struct pap_motor_data_t *ppap_motor_data;
	motion_dir dir;
	int step = STEP_PAP_OUT;
	struct speed_info spd_info;
	struct callback_data clbk_data;
	int ret = 0;
	
	ppap_motor_data = ptp_eng->ppap_motor_data;
	memset(&clbk_data, 0, sizeof(clbk_data));
	clbk_data.data1 = (int)ptp_eng;
	tp_eng_fun_sensor_update(ptp_eng);
	if ((!ptp_eng->tp_eng_sen_st.pap_out) && (!ptp_eng->tp_eng_sen_st.pap_in))
	{
		printk(KERN_DEBUG "no paper.\n");
		return 0;
	}
	if(tp_eng_fun_ph_move(ptp_eng, 2))
	{
		return -RES_PRINTING_PUSH_MOTOR_ERROR;
	}
	if (step >= 0)
	{
		dir = MOTION_CLOCKWISE;
	}
	else
	{
		dir = MOTION_COUNTERCLOCKWISE;
		step = -step;
	}
	if (ptp_eng->config.paper_out_speed)
	{
		spd_info.speed = ptp_eng->config.paper_out_speed;
	}
	else
	{
		spd_info.speed = SPEED_PAP_OUT;
	}
	spd_info.steps = step;
	spd_info.nextspeed = NULL;
	tp_eng_pap_motor_config(ppap_motor_data, step, dir, 1, &spd_info);
	INIT_WORK(&pap_out_wq, tp_eng_fun_pap_out_do_work);
	tp_eng_pap_motor_set_callback(ppap_motor_data, NULL, &clbk_data, tp_eng_fun_pap_out_callback, &clbk_data);
	ret = tp_eng_pap_motor_start(ppap_motor_data);
	if (ret)
	{
		return ret;
	}
	ptp_eng->eng_state.pap_motor_state = PAP_MOTOR_STATE_OUT;
	ret = tp_eng_pap_motor_wait_stop(ppap_motor_data);
	if (ret)
	{
		return ret;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(tp_eng_fun_pap_out);

static struct work_struct pap_move_wq;
static void tp_eng_fun_pap_move_do_work(struct work_struct * work)
{
	if (ptp_eng_backup)
	{
		tp_eng_fun_sensor_update(ptp_eng_backup);
	}
}

static void tp_eng_fun_pap_move_callback(struct pap_motor_data_t *ppap_motor_data, struct callback_data *pcallback_data)
{
	struct tp_engine_t * ptp_eng;
	int step_lost = 0;

	ptp_eng = (struct tp_engine_t *)pcallback_data->data1;
	if (ptp_eng_backup != ptp_eng)
	{
		ptp_eng_backup = ptp_eng;
	}
	step_lost = steppermotor_get_running_steps(ppap_motor_data->pstepmotor);
	if ((step_lost % 10 == 0) && ((step_lost & STEP_MOTOR_STEPS_STOP_FLAG) == 0))
	{
		schedule_work(&pap_out_wq);
	}
	if (ptp_eng->tp_eng_sen_st.pap_in)
	{
		ptp_eng->pap_info_st.pap_length++;
		ptp_eng->pap_info_st.pap_pos++;
	}
	else
	{
		if (ptp_eng->pap_info_st.pap_total_flag == 0)
		{
			ptp_eng->pap_info_st.pap_total_flag = 1;
		}
		ptp_eng->pap_info_st.pap_pos++;
	}
	if ((!ptp_eng->tp_eng_sen_st.pap_out) && (!ptp_eng->tp_eng_sen_st.pap_in))
	{
		if (ptp_eng->eng_state.pap_motor_state == PAP_MOTOR_STATE_OUT)
		{
			printk(KERN_DEBUG "tp_eng_fun_pap_out_callback stop after %d.\n", PAP_OUT_STOP_AFTER_STEPS);
			ptp_eng->eng_state.pap_motor_state = PAP_MOTOR_STATE_STOP;
			tp_eng_pap_motor_stop_after_steps(ptp_eng->ppap_motor_data, PAP_OUT_STOP_AFTER_STEPS);
		}
		ptp_eng->pap_info_st.pap_length = 0;
		ptp_eng->pap_info_st.pap_pos = 0;
		ptp_eng->pap_info_st.pap_total_flag = 0;
	}
}

int tp_eng_fun_pap_move(struct tp_engine_t * ptp_eng, int step)
{
	struct pap_motor_data_t *ppap_motor_data;
	motion_dir dir;
	struct speed_info spd_info;
	struct callback_data clbk_data;
	int ret = 0;
	
	ppap_motor_data = ptp_eng->ppap_motor_data;
	memset(&clbk_data, 0, sizeof(clbk_data));
	clbk_data.data1 = (int)ptp_eng;
	tp_eng_fun_sensor_update(ptp_eng);
	if ((!ptp_eng->tp_eng_sen_st.pap_out) && (!ptp_eng->tp_eng_sen_st.pap_in))
	{
		printk(KERN_DEBUG "no paper.\n");
		return 0;
	}
	if(tp_eng_fun_ph_move(ptp_eng, 2))
	{
		return -RES_PRINTING_PUSH_MOTOR_ERROR;
	}
	if (step >= 0)
	{
		dir = MOTION_CLOCKWISE;
	}
	else
	{
		dir = MOTION_COUNTERCLOCKWISE;
		step = -step;
	}
	if (ptp_eng->config.paper_in_speed)
	{
		spd_info.speed = ptp_eng->config.paper_in_speed;
	}
	else
	{
		spd_info.speed = SPEED_PAP_IN;
	}
	spd_info.steps = step;
	spd_info.nextspeed = NULL;
	ret = tp_eng_pap_motor_config(ppap_motor_data, step, dir, 1, &spd_info);
	if (ret)
	{
		return ret;
	}
	INIT_WORK(&pap_move_wq, tp_eng_fun_pap_move_do_work);
	tp_eng_pap_motor_set_callback(ppap_motor_data, NULL, &clbk_data, tp_eng_fun_pap_move_callback, &clbk_data);
	ret = tp_eng_pap_motor_start(ppap_motor_data);
	if (ret)
	{
		return ret;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(tp_eng_fun_pap_move);


void tp_eng_fun_ph_up_callback(struct photosensor * psen, struct sen_callback_data * pcallback_data)
{
	struct ph_motor_data_t * pph_motor_data;

	pph_motor_data = (struct ph_motor_data_t *)pcallback_data->data1;
	tp_eng_ph_motor_stop(pph_motor_data);
}
EXPORT_SYMBOL_GPL(tp_eng_fun_ph_up_callback);

/* mode: 0 stop, 1 down, 2 up */
int tp_eng_fun_ph_move(struct tp_engine_t * ptp_eng, unsigned char mode)
{
	struct ph_motor_data_t * pph_motor_data;
	int ret = 0;
	struct photosensor * psen;
	struct sen_callback_data sen_clbk_data;
	
	pph_motor_data = ptp_eng->pph_motor_data;
	psen = ptp_eng->psensor_data[SEN_INDEX_PH_UP].sen_dev.pphotosensor;
	sen_clbk_data.data1 = (int)pph_motor_data;
	sen_clbk_data.data2 = (int)ptp_eng->ppap_motor_data;

	switch (mode&0xF)
	{
		case 1:
			printk(KERN_DEBUG "ph down.\n");
			tp_eng_fun_sensor_update(ptp_eng);
			if (ptp_eng->tp_eng_sen_st.ph_down)
			{
				ret = 0;
				goto __exit__;
			}
			ret = tp_eng_ph_motor_config(pph_motor_data, MOTION_CLOCKWISE, PH_MOTOR_TIME_OUT);
			if (ret)
			{
				printk(KERN_ERR "ERROR!!! tp_engine_ioctl tp_eng_ph_motor_config.\n");
				ret = RES_PRINTING_PUSH_MOTOR_ERROR;
				goto __exit__;
			}
			tp_engine_sensor_set_callback(psen, tp_eng_fun_ph_up_callback, &sen_clbk_data);
			ret = tp_eng_ph_motor_start(pph_motor_data);
			if (ret)
			{
				printk(KERN_ERR "ERROR!!! tp_engine_ioctl tp_eng_ph_motor_start.\n");
				ret = -RES_PRINTING_PUSH_MOTOR_ERROR;
				goto __exit__;
			}
			ret = tp_eng_ph_motor_wait_stoped(pph_motor_data);
			if (ret)
			{
				printk(KERN_ERR "ERROR!!! tp_engine_ioctl tp_eng_ph_motor_wait_stoped.\n");
				ret = -RES_PRINTING_PUSH_MOTOR_ERROR;
				goto __exit__;
			}
			ret = tp_eng_ph_motor_config(pph_motor_data, MOTION_CLOCKWISE, PH_MOTOR_TIME_OUT);
			if (ret)
			{
				printk(KERN_ERR "ERROR!!! tp_engine_ioctl tp_eng_ph_motor_config.\n");
				ret = -RES_PRINTING_PUSH_MOTOR_ERROR;
				goto __exit__;
			}
			ret = tp_eng_ph_motor_start(pph_motor_data);
			if (ret)
			{
				printk(KERN_ERR "ERROR!!! tp_engine_ioctl tp_eng_ph_motor_start.\n");
				ret = -RES_PRINTING_PUSH_MOTOR_ERROR;
				goto __exit__;
			}
			mdelay(15);
			tp_eng_ph_motor_stop(pph_motor_data);
			tp_eng_fun_sensor_update(ptp_eng);
			if (ptp_eng->tp_eng_sen_st.ph_down == 0)	//压下失败
			{
				ret = -RES_PRINTING_PUSH_MOTOR_ERROR;
				goto __exit__;
			}
			break;
		case 2:
			printk(KERN_DEBUG "ph up.\n");
			tp_eng_fun_sensor_update(ptp_eng);
			if (ptp_eng->tp_eng_sen_st.ph_down == 0)
			{
				ret = 0;
				goto __exit__;
			}
			ret = tp_eng_ph_motor_config(pph_motor_data, MOTION_CLOCKWISE, PH_MOTOR_TIME_OUT);
			if (ret)
			{//	if (ptp_eng->ribbon_info_st.ribbon_broken_black == 0 || ptp_eng->ribbon_info_st.ribbon_broken_white == 0)
//	{
//		printk("ribbon broken. \n");
//		return -1;
//	}
				printk(KERN_ERR "ERROR!!! tp_engine_ioctl tp_eng_ph_motor_config.\n");
				ret = -RES_PRINTING_PUSH_MOTOR_NOT_WORK;
				goto __exit__;
			}
			tp_engine_sensor_set_callback(psen, tp_eng_fun_ph_up_callback, &sen_clbk_data);
			ret = tp_eng_ph_motor_start(pph_motor_data);
			if (ret)
			{
				printk(KERN_ERR "ERROR!!! tp_engine_ioctl tp_eng_ph_motor_start.\n");
				ret = -RES_PRINTING_PUSH_MOTOR_NOT_WORK;
				goto __exit__;
			}
			ret = tp_eng_ph_motor_wait_stoped(pph_motor_data);
			if (ret)
			{
				printk(KERN_ERR "ERROR!!! tp_engine_ioctl tp_eng_ph_motor_wait_stoped.\n");
				ret = -RES_PRINTING_PUSH_MOTOR_NOT_WORK;
				goto __exit__;
			}
			ret = tp_eng_ph_motor_config(pph_motor_data, MOTION_CLOCKWISE, PH_MOTOR_TIME_OUT);
			if (ret)
			{
				printk(KERN_ERR "ERROR!!! tp_engine_ioctl tp_eng_ph_motor_config.\n");
				ret = -RES_PRINTING_PUSH_MOTOR_NOT_WORK;
				goto __exit__;
			}
			ret = tp_eng_ph_motor_start(pph_motor_data);
			if (ret)
			{
				printk(KERN_ERR "ERROR!!! tp_engine_ioctl tp_eng_ph_motor_start.\n");
				ret = -RES_PRINTING_PUSH_MOTOR_NOT_WORK;
				goto __exit__;
			}
			mdelay(45);
			tp_eng_ph_motor_stop(pph_motor_data);
			break;
		case 0:
		default:
			tp_eng_ph_motor_stop(pph_motor_data);
			ret = 0;
			break;
	}
__exit__:
	return ret;
}
EXPORT_SYMBOL_GPL(tp_eng_fun_ph_move);

/* mode 1 run, 0 stop */
int tp_eng_fun_ribbon_run(struct tp_engine_t * ptp_eng, unsigned char mode)
{
	struct ribbon_motor_data_t * pribbon_motor_data;
	int ret = 0;
	
	pribbon_motor_data = ptp_eng->pribbon_motor_data;
	if (mode)
	{
		ret = tp_eng_ribbon_motor_config(pribbon_motor_data, MOTION_COUNTERCLOCKWISE, 3000);
		if (ret)
		{
		    printk(KERN_ERR "ERROR!!! tp_engine_ioctl tp_eng_ph_motor_config.\n");
		    ret = RES_PRINTING_UNKOWN_ERROR;
		}
		ret = tp_eng_ribbon_motor_start(pribbon_motor_data);
		if (ret)
		{
		    printk(KERN_ERR "ERROR!!! tp_engine_ioctl tp_eng_ph_motor_start.\n");
		    ret = RES_PRINTING_UNKOWN_ERROR;
		}
	}
	else
	{
		tp_eng_ribbon_motor_stop(pribbon_motor_data);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(tp_eng_fun_ribbon_run);

int tp_eng_fun_sensor_update(struct tp_engine_t * ptp_eng)
{
	unsigned int sen_st = 0;
	if(tp_engine_sensor_get_logicval(ptp_eng, &sen_st))
	{
		return -1;
	}
	if (sen_st & SEN_ST_PAPAR_IN)
	{
		ptp_eng->tp_eng_sen_st.pap_in = 1;
	}
	else
	{
		ptp_eng->tp_eng_sen_st.pap_in = 0;
	}
	if (sen_st & SEN_ST_PAPAR_OUT)
	{
		ptp_eng->tp_eng_sen_st.pap_out = 1;
	}
	else
	{
		ptp_eng->tp_eng_sen_st.pap_out = 0;
	}
	if (sen_st & SEN_ST_RIBBON_EXSIT)
	{
		ptp_eng->tp_eng_sen_st.ribbon_exsit = 1;
	}
	else
	{
		ptp_eng->tp_eng_sen_st.ribbon_exsit = 0;
	}
/*	if (sen_st & SEN_ST_RIBBON_BROKEN)
	{
		ptp_eng->tp_eng_sen_st.ribbon_broken = 1;
	}
	else
	{
		ptp_eng->tp_eng_sen_st.ribbon_broken = 0;
	}
*/
	if (sen_st & SEN_ST_PH_DOWN)
	{
		ptp_eng->tp_eng_sen_st.ph_down = 1;
	}
	else
	{
		ptp_eng->tp_eng_sen_st.ph_down = 0;
	}
	if (sen_st & SEN_ST_MAC_CLOSE)
	{
		ptp_eng->tp_eng_sen_st.mac_close = 1;
	}
	else
	{
		ptp_eng->tp_eng_sen_st.mac_close = 0;
	}
	//if(tp_eng_ph_resistor_get_val(ptp_eng->pph_resistor_data, &ptp_eng->tp_eng_sen_st.ph_resistor_val))
	//{
	//	return -1;
	//}
	return 0;
}
EXPORT_SYMBOL_GPL(tp_eng_fun_sensor_update);

static struct work_struct update_sensor_wq;
static void tp_eng_fun_update_sensor_wq(struct work_struct * work)
{
	struct tp_engine_t * ptp_eng;
	int rs = 0;
	unsigned int st;

	ptp_eng = ptp_eng_backup;
	//tp_eng_fun_sensor_update(ptp_eng);
	rs = tp_engine_get_sensor_statu(ptp_eng, &st, SEN_ST_PAPAR_IN);
	if (rs)
	{
		printk(KERN_ERR "tp_engine_get_sensor_statu error.\n");
	}
	else
	{
		if (st > 0)
		{
			ptp_eng->tp_eng_sen_st.pap_in = 1;
		}
		else
		{
			ptp_eng->tp_eng_sen_st.pap_in = 0;
		}
	}
	rs = tp_engine_get_sensor_statu(ptp_eng, &st, SEN_ST_RIBBON_EXSIT);
	if (rs)
	{
		printk(KERN_ERR "tp_engine_get_sensor_statu errtp_eng_pap_motor_wait_stopor.\n");
	}
	else
	{
		if (st > 0)
		{
			ptp_eng->tp_eng_sen_st.ribbon_exsit = 1;
		}
		else
		{
			ptp_eng->tp_eng_sen_st.ribbon_exsit = 0;
		}
	}

/*	rs = tp_engine_get_sensor_statu(ptp_eng, &st, SEN_ST_RIBBON_BROKEN);
	if (rs)
	{
		printk("tp_engine_get_sensor_statu error.\n");
	}
	else
	{
		if (st > 0)
		{
			ptp_eng->tp_eng_sen_st.ribbon_broken = 1;
		}
		else
		{
			ptp_eng->tp_eng_sen_st.ribbon_broken = 0;
		}
	}
*/

}

static struct work_struct print_go_wq;
static struct workqueue_struct * print_go_workqueue = NULL;
static struct workqueue_struct * update_sensor_workqueue = NULL;
static void tp_eng_fun_print_go_do_work(struct work_struct * work)
{
	struct tp_engine_t * ptp_eng;
	unsigned int size;
	unsigned int st;

	ptp_eng = ptp_eng_backup;
	if (ptp_eng)
	{
		if(ptp_eng->eng_state.pap_motor_state == PAP_MOTOR_STATE_PRINT)
		{
			//还没有获得介质长度，或者走纸位置小于介质长度时允许打印
			if ((ptp_eng->pap_info_st.pap_total_flag == 0) || (ptp_eng->pap_info_st.pap_pos < ptp_eng->pap_info_st.pap_length))
			{
#if IMAGE_ROTATE
				if (ptp_eng->bmp_data.p_cur_c < ptp_eng->bmp_data.buff)
#else
				if (ptp_eng->bmp_data.p_cur_c >= (ptp_eng->bmp_data.buff + ptp_eng->bmp_data.bmp_info.biSizeImage))
#endif
				{
					ptp_eng->eng_state.pap_motor_state = PAP_MOTOR_STATE_STOP;
					tp_eng_pap_motor_stop(ptp_eng->ppap_motor_data);
					tp_eng_fun_ribbon_run(ptp_eng, 0);
					printk(KERN_DEBUG "tp_eng_fun_print_go_callback print done.\n");
					return;
				}
				size = ptp_eng->bmp_data.bmp_info.biWidth / 8;
				tp_eng_ph_write_line(ptp_eng->pph_data, ptp_eng->bmp_data.p_cur_c, size);
				ptp_eng->bmp_data.p_cur_c = ptp_eng->bmp_data.p_cur_c  - ((ptp_eng->bmp_data.bmp_info.biWidth + 31) / 32 * 4);
#if 1
				if (tp_engine_get_sensor_statu(ptp_eng, &st, SEN_ST_RIBBON_BROKEN))
				{
					printk(KERN_DEBUG "tp_engine_get_sensor_statu error.\n");
				}
				if (st)
				{
					ptp_eng->ribbon_info_st.ribbon_broken_white++;
				}
				else
				{
					ptp_eng->ribbon_info_st.ribbon_broken_black++;
				}
//				printk("ptp_eng->ribbon_info_st.ribbon_broken_white is %d ribbon_broken_black %d.\n", ptp_eng->ribbon_info_st.ribbon_broken_white, ptp_eng->ribbon_info_st.ribbon_broken_black);
				if ((ptp_eng->ribbon_info_st.ribbon_broken_white) > (ptp_eng->ribbon_info_st.ribbon_broken_black))
				{
					if(((ptp_eng->ribbon_info_st.ribbon_broken_white) - (ptp_eng->ribbon_info_st.ribbon_broken_black)) > 1000)
					{
						printk(KERN_DEBUG "tp_eng_fun_print_go_callback ribbon_broken_black.\n");
						printk(KERN_DEBUG "ptp_eng->ribbon_info_st.ribbon_broken_white is %d.\n", ptp_eng->ribbon_info_st.ribbon_broken_white);
						printk(KERN_DEBUG "ptp_eng->ribbon_info_st.ribbon_broken_black is %d.\n", ptp_eng->ribbon_info_st.ribbon_broken_black);
						ptp_eng->tp_eng_sen_st.ribbon_broken = 1;
						ptp_eng->eng_state.pap_motor_state = PAP_MOTOR_STATE_STOP;
						tp_eng_pap_motor_stop(ptp_eng->ppap_motor_data);
						tp_eng_fun_ribbon_run(ptp_eng, 0);
					}
				}
				else
				{
					if(((ptp_eng->ribbon_info_st.ribbon_broken_black) - (ptp_eng->ribbon_info_st.ribbon_broken_white)) > 1000)
					{
						printk(KERN_DEBUG "tp_eng_fun_print_go_callback ribbon_broken_black.\n");
						printk(KERN_DEBUG "ptp_eng->ribbon_info_st.ribbon_broken_white is %d.\n", ptp_eng->ribbon_info_st.ribbon_broken_white);
						printk(KERN_DEBUG "ptp_eng->ribbon_info_st.ribbon_broken_black is %d.\n", ptp_eng->ribbon_info_st.ribbon_broken_black);
						ptp_eng->tp_eng_sen_st.ribbon_broken = 1;
						ptp_eng->eng_state.pap_motor_state = PAP_MOTOR_STATE_STOP;
						tp_eng_pap_motor_stop(ptp_eng->ppap_motor_data);
						tp_eng_fun_ribbon_run(ptp_eng, 0);
					}
				}
#endif
			}
			else
			{
				printk(KERN_DEBUG "tp_eng_fun_print_go_callback print finish with paper end.\n");
				ptp_eng->eng_state.pap_motor_state = PAP_MOTOR_STATE_STOP;
				tp_eng_pap_motor_stop(ptp_eng->ppap_motor_data);
				tp_eng_fun_ribbon_run(ptp_eng, 0);
				return;
			}
		}
	}
}

static void tp_eng_fun_print_go_callback(struct pap_motor_data_t *ppap_motor_data, struct callback_data *pcallback_data)
{
	struct tp_engine_t * ptp_eng;
	int step_lost = 0;

	ptp_eng = (struct tp_engine_t *)pcallback_data->data1;
	if (ptp_eng_backup != ptp_eng)
	{
		ptp_eng_backup = ptp_eng;
	}
	if (print_go_workqueue)
	{
		queue_work(print_go_workqueue, &print_go_wq);
	}
	else
	{
		queue_work(system_highpri_wq, &print_go_wq);
	}
	step_lost = steppermotor_get_running_steps(ppap_motor_data->pstepmotor);
	if ((step_lost % 10 == 0) && ((step_lost & STEP_MOTOR_STEPS_STOP_FLAG) == 0))
	{
		if (update_sensor_workqueue)
		{
			queue_work(update_sensor_workqueue, &update_sensor_wq);
		}
		else
		{
			queue_work(system_highpri_wq, &update_sensor_wq);
		}
	}
	if (ptp_eng->tp_eng_sen_st.pap_in)
	{
		ptp_eng->pap_info_st.pap_length++;
		ptp_eng->pap_info_st.pap_pos++;
	}
	else
	{
		if (ptp_eng->pap_info_st.pap_total_flag == 0)
		{
			ptp_eng->pap_info_st.pap_total_flag = 1;
		}
		ptp_eng->pap_info_st.pap_pos++;
	}
}

static void tp_eng_fun_print_go_end_callback(struct pap_motor_data_t *ppap_motor_data, struct callback_data *pcallback_data)
{
	struct tp_engine_t * ptp_eng;

	ptp_eng = (struct tp_engine_t *)pcallback_data->data1;
	if (ptp_eng_backup != ptp_eng)
	{
		ptp_eng_backup = ptp_eng;
	}
	if(ptp_eng->eng_state.pap_motor_state == PAP_MOTOR_STATE_PRINT)
	{

		ptp_eng->eng_state.pap_motor_state = PAP_MOTOR_STATE_STOP;
		tp_eng_pap_motor_stop(ptp_eng->ppap_motor_data);
		tp_eng_fun_ph_move(ptp_eng, 0);
		tp_eng_fun_ribbon_run(ptp_eng, 0);
	}
}

static long tp_eng_fun_print_go(struct tp_engine_t * ptp_eng, unsigned char * buff)
{
	long ret = 0;
	BITMAPFILEHEADER * pbmp_header;
	BITMAPINFOHEADER * pbmp_info;
	struct pap_motor_data_t *ppap_motor_data;
	//struct tp_ph_config_t ph_conf;
	motion_dir dir;
	int step;
	struct speed_info spd_info;
	struct callback_data clbk_data;

	tp_eng_fun_sensor_update(ptp_eng);
	if (ptp_eng->tp_eng_sen_st.pap_in == 0)
	{
		printk(KERN_ERR "no paper.\n");
		return -RES_PRINTING_NO_PAP_ERROR;
	}
	if (ptp_eng->tp_eng_sen_st.ribbon_exsit == 0)
	{
		printk(KERN_ERR "no ribbon.\n");
		return -RES_PRINTING_RINBON_END;
	}
	if (ptp_eng->tp_eng_sen_st.ribbon_broken == 1)
	{
		printk(KERN_ERR "ribbon broken.\n");
		return -RES_PRINTING_RINBBON_BROKEN;
	}
	ppap_motor_data = ptp_eng->ppap_motor_data;
	pbmp_header = &ptp_eng->bmp_data.bmp_header;
	pbmp_info = &ptp_eng->bmp_data.bmp_info;
	memcpy(pbmp_header, buff, sizeof(BITMAPFILEHEADER));
	if(pbmp_header->bfType != 0x4D42)
	{
		printk(KERN_DEBUG "bmp_header != BM.\n");
		return -RES_PRINTING_RD_ERROR;
	}
	memcpy(pbmp_info, buff + sizeof(BITMAPFILEHEADER), sizeof(BITMAPINFOHEADER));
	if(pbmp_info->biBitCount != 1)
	{
		printk(KERN_DEBUG "bmp_info is not match. %d, %d\n", pbmp_info->biSize, pbmp_info->biBitCount);
		return -RES_PRINTING_RD_ERROR;
	}
	ptp_eng->bmp_data.buff = buff + pbmp_header->bfOffBits;
#if IMAGE_ROTATE
	ptp_eng->bmp_data.p_cur_c = ptp_eng->bmp_data.buff + ptp_eng->bmp_data.bmp_info.biSizeImage;
#else
	ptp_eng->bmp_data.p_cur_c = ptp_eng->bmp_data.buff;
#endif
	printk(KERN_DEBUG "pbmp_header->bfType = %x, pbmp_header->bfOffBits = %d..\n", pbmp_header->bfType, pbmp_header->bfOffBits);
	printk(KERN_DEBUG "pbmp_info->biSize = %d, pbmp_info->biSizeImage = %d.\n", pbmp_info->biSize, pbmp_info->biSizeImage);
	printk(KERN_DEBUG "pbmp_info->biWidth = %d, pbmp_info->biHeight = %d.\n", pbmp_info->biWidth, pbmp_info->biHeight);
	printk(KERN_DEBUG "buff = %x, cur = %x\n", (int)ptp_eng->bmp_data.buff, (int)ptp_eng->bmp_data.p_cur_c);
	if(ptp_eng->tp_eng_sen_st.ribbon_broken)
	{
		ptp_eng->ribbon_info_st.ribbon_broken_white = 0;
		ptp_eng->ribbon_info_st.ribbon_broken_black = 0;
	}
	ret = tp_eng_fun_ph_move(ptp_eng, 1);		//打印头下压
	if (ret)
	{
		return ret;
	}
	ret = tp_eng_fun_ribbon_run(ptp_eng, 1);      //碳带启动
	if (ret)
	{
		return ret;
	}
	/* 打印与走纸 */
	step = ptp_eng->bmp_data.bmp_info.biHeight;
	dir = MOTION_CLOCKWISE;
	if (ptp_eng->config.printing_speed)
	{
		spd_info.speed = ptp_eng->config.printing_speed;
	}
	else
	{
		spd_info.speed = SPEED_PRINTING;
	}
	spd_info.steps = step;
	spd_info.nextspeed = NULL;
	tp_eng_pap_motor_config(ppap_motor_data, step, dir, 1, &spd_info);
	//tp_eng_ph_config(ptp_eng->pph_data, &ph_conf);
	print_go_workqueue = alloc_workqueue("print_go_wq", (WQ_HIGHPRI|WQ_CPU_INTENSIVE), 1);//create_workqueue("print_go_wq");
	if (!print_go_workqueue)
	{
		panic("Failed to create print_go_workqueue\n");
	}
	update_sensor_workqueue = alloc_workqueue("update_sensor_workqueue", 0, 0);
	if (!update_sensor_workqueue)
	{
		panic("Failed to create update_sensor_workqueue\n");
	}
	INIT_WORK(&print_go_wq, tp_eng_fun_print_go_do_work);
	INIT_WORK(&update_sensor_wq, tp_eng_fun_update_sensor_wq);
	memset(&clbk_data, 0, sizeof(clbk_data));
	clbk_data.data1 = (int)ptp_eng;
	ret = tp_eng_pap_motor_set_callback(ppap_motor_data, tp_eng_fun_print_go_end_callback, &clbk_data, tp_eng_fun_print_go_callback, &clbk_data);
	if (ret)
	{
		return ret;
	}
	ret = tp_eng_pap_motor_start(ppap_motor_data);
	if (ret)
	{
		return ret;
	}
	ptp_eng->eng_state.pap_motor_state = PAP_MOTOR_STATE_PRINT;
	ret = tp_eng_pap_motor_wait_stop(ppap_motor_data);
	if (ret)
	{
		return ret;
	}
	ret = tp_eng_ribbon_motor_wait_stoped(ptp_eng->pribbon_motor_data);
	if (ret)
	{
		return ret;
	}
	if (print_go_workqueue)
	{
		destroy_workqueue(print_go_workqueue);
	}
	if(update_sensor_workqueue)
	{
		destroy_workqueue(update_sensor_workqueue);
	}
	tp_eng_fun_sensor_update(ptp_eng);
	
	if (ptp_eng->tp_eng_sen_st.ribbon_broken == 1)
	{
		printk(KERN_ERR "ribbon broken.\n");
		return -RES_PRINTING_RINBBON_BROKEN;
	}
	if (ptp_eng->tp_eng_sen_st.pap_out == 0)
	{
		printk(KERN_ERR "paper jam.\n");
		return -RES_MEDIA_JAM;
	}
	if (ptp_eng->tp_eng_sen_st.ribbon_exsit == 0)
	{
		printk(KERN_ERR "ribbon end.\n");
		return -RES_PRINTING_RINBON_END;
	}

	return ret;
}

#if 0
static int debug_image(struct tp_engine_t * ptp_eng, unsigned char * buff)
{
	BITMAPFILEHEADER * pbmp_header;
	BITMAPINFOHEADER * pbmp_info;
	unsigned char * pbuf_cur, *pbuf_start;
	unsigned char byte;
	int i = 0, j = 0, k = 0;

	pbmp_header = &ptp_eng->bmp_data.bmp_header;
	pbmp_info = &ptp_eng->bmp_data.bmp_info;
	memcpy(pbmp_header, buff, sizeof(BITMAPFILEHEADER));
	if(pbmp_header->bfType != 0x4D42)
	{
		printk("bmp_header != BM.\n");
		return -1;
	}
	printk("pbmp_header->bfType = %x, pbmp_header->bfOffBits = %d..\n", pbmp_header->bfType, pbmp_header->bfOffBits);
	memcpy(pbmp_info, buff + sizeof(BITMAPFILEHEADER), sizeof(BITMAPINFOHEADER));
	if(pbmp_info->biBitCount != 1)
	{
		printk("bmp_info is not match. %d, %d\n", pbmp_info->biSize, pbmp_info->biBitCount);
		return -1;
	}
	printk("pbmp_info->biSize = %d, pbmp_info->biSizeImage = %d.\n", pbmp_info->biSize, pbmp_info->biSizeImage);
	printk("pbmp_info->biWidth = %d, pbmp_info->biHeight = %d.\n", pbmp_info->biWidth, pbmp_info->biHeight);
	ptp_eng->bmp_data.buff = buff + pbmp_header->bfOffBits;
	pbuf_start = ptp_eng->bmp_data.buff + ptp_eng->bmp_data.bmp_info.biSizeImage;
	for(i = 0; i < pbmp_info->biHeight; i++)
	{
		for(j = 0; j < (pbmp_info->biWidth)/8; j++)
		{
			/* 定位必须按4字节对齐 */
			pbuf_cur = pbuf_start - (i + 1) * ((pbmp_info->biWidth + 31)/32 *4) + j;
			for(k = 0; k < 8; k++)
			{
				byte = *pbuf_cur;
				if(byte & (0x80>>k))
				{
					printk("*");
				}
				else
				{
					printk(" ");
				}
			}
		}
		printk("\n");
	}
	return 0;
}
#endif

struct print_data_t
{
	unsigned int size;
	unsigned char * buff;
};

long tp_eng_fun_print(struct tp_engine_t * ptp_eng, void __user * argp)
{
	struct print_data_t pr_data;
	unsigned char * buff;
	long ret = 0;
	
	if(copy_from_user((void *)(&pr_data), (void __user *)argp, sizeof(struct print_data_t)))
	{
		printk(KERN_ERR "tp_eng_fun_print copy_from_user error.\n");
		return -RES_PRINTING_UNKOWN_ERROR;
	}
	buff = kzalloc(pr_data.size, GFP_KERNEL);
	if (buff == NULL)
	{
		printk(KERN_ERR "tp_eng_fun_print kzalloc failed.\n");
		return -RES_PRINTING_UNKOWN_ERROR;
	}
	if (copy_from_user((void *)buff, (void __user *)pr_data.buff, pr_data.size))
	{
		printk(KERN_ERR "tp_eng_fun_print copy_from_user error.\n");
		ret = -RES_PRINTING_UNKOWN_ERROR;
		goto __exit__;
	}
/*
        if (debug_image(ptp_eng, buff))
        {                              
                ret = -1;              
                goto __exit__;         
        }                              
*/
	ret = tp_eng_fun_print_go(ptp_eng, buff);
	if (ret)
	{
		goto __exit__;
	}
__exit__:
	if (buff)
	{
		kfree(buff);
	}
	return ret;
}
EXPORT_SYMBOL_GPL(tp_eng_fun_print);


//获取纸张长度
int tp_engine_get_pap_lenght(struct tp_engine_t *ptp_engine, unsigned int *lenght)
{
	unsigned int val;
	
	if (ptp_engine->pap_info_st.pap_total_flag)
	{
		val = ptp_engine->pap_info_st.pap_length;
	}
	else
	{
		val = 0;
	}
	*lenght = val;
	return 0;

}
EXPORT_SYMBOL_GPL(tp_engine_get_pap_lenght);

//获取当前打印速度走纸速度
int tp_engine_get_eng_config(struct tp_engine_t *ptp_engine, struct tp_engine_config_t *peng_config)
{

	peng_config->paper_in_speed = ptp_engine->config.paper_in_speed;
	peng_config->paper_out_speed = ptp_engine->config.paper_out_speed;
	peng_config->printing_speed = ptp_engine->config.printing_speed;

	return 0;

}
EXPORT_SYMBOL_GPL(tp_engine_get_eng_config);


//获取当前打印头配置
int tp_engine_get_ph_config(struct tp_engine_t * ptp_engine, struct tp_ph_config_t * pconfig_data)
{
	memcpy(pconfig_data, &ptp_engine->pph_data->ptp_ph->config_data, sizeof(struct tp_ph_config_t));
	
	return 0;

}
EXPORT_SYMBOL_GPL(tp_engine_get_ph_config);


