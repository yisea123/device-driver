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
#include "../tp_printer_header/printer_header_common.h"

#define SPEED_PAP_IN	800
#define SPEED_PAP_OUT	SPEED_PAP_IN
#define SPEED_PRINTING	800

#define STEP_PAP_IN	2000
#define STEP_PAP_OUT	2000

/* one step = 25.4mm/300 */
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

	ptp_eng = (struct tp_engine_t *)pcallback_data->data1;
	ptp_eng_backup = ptp_eng;
	schedule_work(&pap_in_wq);
	if (ptp_eng->tp_eng_sen_st.pap_in)
	{
		if (ptp_eng->eng_state.pap_motor_state == PAP_MOTOR_STATE_IN)
		{
			printk("tp_eng_fun_pap_in_callback stop after %d.\n", PAP_IN_STOP_AFTER_STEPS);
			ptp_eng->eng_state.pap_motor_state = PAP_MOTOR_STATE_STOP;
			tp_eng_pap_motor_stop_after_steps(ptp_eng->ppap_motor_data, PAP_IN_STOP_AFTER_STEPS);
		}
	}
}

int tp_eng_fun_pap_in(struct tp_engine_t * ptp_eng)
{
	struct pap_motor_data_t *ppap_motor_data;
	motion_dir dir;
	int step = STEP_PAP_IN;
	struct speed_info spd_info;
	struct callback_data clbk_data;
	struct photosensor * psen;
	
	ppap_motor_data = ptp_eng->ppap_motor_data;
	psen = ptp_eng->psensor_data[SEN_INDEX_PAPAR_IN].sen_dev.pphotosensor;
	memset(&clbk_data, 0, sizeof(clbk_data));
	clbk_data.data1 = (int)ptp_eng;
	tp_eng_fun_sensor_update(ptp_eng);
	if (ptp_eng->tp_eng_sen_st.pap_in)
	{
		return 0;
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
	spd_info.speed = SPEED_PAP_IN;
	spd_info.steps = step;
	spd_info.nextspeed = NULL;
	tp_eng_pap_motor_config(ppap_motor_data, step, dir, 1, &spd_info);
	INIT_WORK(&pap_in_wq, tp_eng_fun_pap_in_do_work);
	tp_eng_pap_motor_set_callback(ppap_motor_data, NULL, &clbk_data, tp_eng_fun_pap_in_callback, &clbk_data);
	tp_eng_pap_motor_start(ppap_motor_data);
	ptp_eng->eng_state.pap_motor_state = PAP_MOTOR_STATE_IN;
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

	ptp_eng = (struct tp_engine_t *)pcallback_data->data1;
	ptp_eng_backup = ptp_eng;
	schedule_work(&pap_out_wq);
	if (!ptp_eng->tp_eng_sen_st.pap_out)
	{
		if (ptp_eng->eng_state.pap_motor_state == PAP_MOTOR_STATE_OUT)
		{
			printk("tp_eng_fun_pap_out_callback stop after %d.\n", PAP_OUT_STOP_AFTER_STEPS);
			ptp_eng->eng_state.pap_motor_state = PAP_MOTOR_STATE_STOP;
			tp_eng_pap_motor_stop_after_steps(ptp_eng->ppap_motor_data, PAP_OUT_STOP_AFTER_STEPS);
		}
	}
}

int tp_eng_fun_pap_out(struct tp_engine_t * ptp_eng)
{
	struct pap_motor_data_t *ppap_motor_data;
	motion_dir dir;
	int step = STEP_PAP_OUT;
	struct speed_info spd_info;
	struct callback_data clbk_data;
	struct photosensor * psen;
	
	ppap_motor_data = ptp_eng->ppap_motor_data;
	psen = ptp_eng->psensor_data[SEN_INDEX_PAPAR_OUT].sen_dev.pphotosensor;
	memset(&clbk_data, 0, sizeof(clbk_data));
	clbk_data.data1 = (int)ptp_eng;
	tp_eng_fun_sensor_update(ptp_eng);
	if (!ptp_eng->tp_eng_sen_st.pap_out)
	{
		printk("no paper.\n");
		return 0;
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
	spd_info.speed = SPEED_PAP_OUT;
	spd_info.steps = step;
	spd_info.nextspeed = NULL;
	tp_eng_pap_motor_config(ppap_motor_data, step, dir, 1, &spd_info);
	INIT_WORK(&pap_out_wq, tp_eng_fun_pap_out_do_work);
	tp_eng_pap_motor_set_callback(ppap_motor_data, NULL, &clbk_data, tp_eng_fun_pap_out_callback, &clbk_data);
	tp_eng_pap_motor_start(ppap_motor_data);
	ptp_eng->eng_state.pap_motor_state = PAP_MOTOR_STATE_OUT;
	return 0;
}
EXPORT_SYMBOL_GPL(tp_eng_fun_pap_out);

int tp_eng_fun_pap_move(struct tp_engine_t * ptp_eng, int step)
{
	struct pap_motor_data_t *ppap_motor_data;
	motion_dir dir;
	struct speed_info spd_info;
	struct callback_data clbk_data;
	
	ppap_motor_data = ptp_eng->ppap_motor_data;
	memset(&clbk_data, 0, sizeof(clbk_data));
	if (step >= 0)
	{
		dir = MOTION_CLOCKWISE;
	}
	else
	{
		dir = MOTION_COUNTERCLOCKWISE;
		step = -step;
	}
	spd_info.speed = 800;
	spd_info.steps = step;
	spd_info.nextspeed = NULL;
	tp_eng_pap_motor_config(ppap_motor_data, step, dir, 1, &spd_info);
	tp_eng_pap_motor_set_callback(ppap_motor_data, NULL, &clbk_data, NULL, &clbk_data);
	tp_eng_pap_motor_start(ppap_motor_data);
	return 0;
}
EXPORT_SYMBOL_GPL(tp_eng_fun_pap_move);


void tp_eng_fun_ph_up_callback(struct photosensor * psen, struct sen_callback_data * pcallback_data)
{
	struct ph_motor_data_t * pph_motor_data;
	printk("tp_eng_fun_ph_up_callback.\n");

	pph_motor_data = (struct ph_motor_data_t *)pcallback_data->data1;
	tp_eng_pap_motor_stop((struct pap_motor_data_t *)pcallback_data->data2);
	tp_eng_ph_motor_stop(pph_motor_data);
	psen->callback = NULL;
}
EXPORT_SYMBOL_GPL(tp_eng_fun_ph_up_callback);

int tp_eng_fun_ph_up(struct tp_engine_t * ptp_eng, unsigned char mode)
{
	struct ph_motor_data_t * pph_motor_data;
	int ret = 0;
	struct photosensor * psen;
	struct sen_callback_data sen_clbk_data;
	
	pph_motor_data = ptp_eng->pph_motor_data;
	psen = ptp_eng->psensor_data[SEN_INDEX_PH_UP].sen_dev.pphotosensor;
	sen_clbk_data.data1 = (int)pph_motor_data;
	sen_clbk_data.data2 = (int)ptp_eng->ppap_motor_data;
	
	if (mode)
	{
		ret = tp_eng_ph_motor_config(pph_motor_data, MOTION_CLOCKWISE, 500);
		if (ret)
		{
			printk(KERN_ERR "ERROR!!! tp_engine_ioctl tp_eng_ph_motor_config.\n");
		}
		tp_engine_sensor_set_callback(psen, tp_eng_fun_ph_up_callback, &sen_clbk_data);
		ret = tp_eng_ph_motor_start(pph_motor_data);
		if (ret)
		{
			printk(KERN_ERR "ERROR!!! tp_engine_ioctl tp_eng_ph_motor_start.\n");
		}
	}
	else
	{
		tp_eng_ph_motor_stop(pph_motor_data);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(tp_eng_fun_ph_up);

int tp_eng_fun_sensor_update(struct tp_engine_t * ptp_eng)
{
	unsigned int sen_st = 0;
	tp_engine_sensor_get_logicval(ptp_eng, &sen_st);
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
	if (sen_st & SEN_ST_RIBBON_BROKEN)
	{
		ptp_eng->tp_eng_sen_st.ribbon_broken = 1;
	}
	else
	{
		ptp_eng->tp_eng_sen_st.ribbon_broken = 0;
	}
	if (sen_st & SEN_ST_PH_UP)
	{
		ptp_eng->tp_eng_sen_st.ph_up = 1;
	}
	else
	{
		ptp_eng->tp_eng_sen_st.ph_up = 0;
	}
	if (sen_st & SEN_ST_MAC_CLOSE)
	{
		ptp_eng->tp_eng_sen_st.mac_close = 1;
	}
	else
	{
		ptp_eng->tp_eng_sen_st.mac_close = 0;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(tp_eng_fun_sensor_update);

