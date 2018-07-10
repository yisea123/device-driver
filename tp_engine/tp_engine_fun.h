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
#include "../tp_printer_header/tp_printer_header.h"

int tp_eng_fun_config(struct tp_engine_t * ptp_eng, struct tp_engine_config_t *pconfig);
int tp_eng_fun_ph_move(struct tp_engine_t * ptp_eng, unsigned char mode);
int tp_eng_fun_ribbon_run(struct tp_engine_t * ptp_eng, unsigned char mode);
int tp_eng_fun_pap_in(struct tp_engine_t * ptp_eng);
int tp_eng_fun_pap_out(struct tp_engine_t * ptp_eng);
int tp_eng_fun_pap_move(struct tp_engine_t * ptp_eng, int step);
int tp_eng_fun_sensor_update(struct tp_engine_t * ptp_eng);
long tp_eng_fun_print(struct tp_engine_t * ptp_eng, void __user * argp);
int tp_engine_get_pap_lenght(struct tp_engine_t *ptp_engine, unsigned int *lenght);
int tp_engine_get_eng_config(struct tp_engine_t *ptp_engine, struct tp_engine_config_t *peng_config);

int tp_engine_get_ph_config(struct tp_engine_t *ptp_engine, struct tp_ph_config_t *pconfig_data);

#endif

