/*
 * Image Sensor device driver definitions
 *
 * Copyright 2016 Hunan GreatWall Information Financial Equipment Co., Ltd.
 *
 */
#ifndef __IMAGESENSOR_H__
#define __IMAGESENSOR_H__

#include <linux/device.h>
#include <linux/err.h>
#include <linux/of.h>

#include "scanunit.h"


/*
 * struct imagesensor
 *
 * One for each imagesensor device.
 */
struct imagesensor {
	struct device *dev;
	struct list_head list;
	int using;
	int (*enable)(struct imagesensor *sensor);
	void (*disable)(struct imagesensor *sensor);
	int (*get_config)(struct imagesensor *sensor, struct scanunit_config *config);
	int (*set_config)(struct imagesensor *sensor, const struct scanunit_config *config);
};


/* function prototypes of image sensor driver (chip level) */
extern int imagesensor_add(struct imagesensor *sensor);
extern int imagesensor_remove(struct imagesensor *sensor);


/* function prototypes of image sensor driver (user level) */
extern struct imagesensor *imagesensor_get(struct device *dev);
extern struct imagesensor *of_imagesensor_get(struct device_node *np);
extern struct imagesensor *of_node_to_imagesensor(struct device_node *np);
extern void imagesensor_put(struct imagesensor *sensor);

extern int imagesensor_enable(struct imagesensor *sensor);
extern void imagesensor_disable(struct imagesensor *sensor);

extern int imagesensor_get_config(struct imagesensor *sensor, struct scanunit_config *config);
extern int imagesensor_set_config(struct imagesensor *sensor, const struct scanunit_config *config);


#endif /* __IMAGESENSOR_H__ */
