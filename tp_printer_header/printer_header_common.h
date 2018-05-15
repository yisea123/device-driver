/*
 * GWI Thermal Printer Header Common Interface.
 *
 * Copyright 2018 Hunan GreatWall Information Financial Equipment Co., Ltd.
 *
 * Licensed under the GPL-2.
*/

#ifndef __PRINTER_HEADER_COMMON_H__
#define __PRINTER_HEADER_COMMON_H__

int ph_add(struct tp_ph_t * pph);
int ph_remove(struct tp_ph_t * pph);
struct tp_ph_t * of_node_to_ph(struct device_node * np);


#endif
