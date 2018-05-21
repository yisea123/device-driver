/*
 * GWI Thermal Printer Printer Engine Driver.
 *
 * Copyright 2018 Hunan GreatWall Information Financial Equipment Co., Ltd.
 *
 * Licensed under the GPL-2.
*/

#ifndef __TP_ENGINE_FUN_H__
#define __TP_ENGINE_FUN_H__

#include "tp_engine.h"

int tp_eng_fun_ph_up(struct tp_engine_t * ptp_eng, unsigned char mode);
int tp_eng_fun_pap_in(struct tp_engine_t * ptp_eng);
int tp_eng_fun_pap_out(struct tp_engine_t * ptp_eng);
int tp_eng_fun_pap_move(struct tp_engine_t * ptp_eng, int step);
int tp_eng_fun_sensor_update(struct tp_engine_t * ptp_eng);


#endif

