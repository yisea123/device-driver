/*
 * image digitiser (analog front end [AFE]) device driver definitions
 *
 * Copyright 2016 Hunan GreatWall Information Financial Equipment Co., Ltd.
 *
 */
#ifndef __IMAGEDIGITISER_H__
#define __IMAGEDIGITISER_H__

#include <linux/device.h>
#include <linux/err.h>
#include <linux/of.h>

#include "scanunit.h"


/*
 * struct imagedigitiser
 *
 * One for each image digitiser device.
 */
struct imagedigitiser {
	struct device *dev;
	struct list_head list;
	int using;
	int (*enable)(struct imagedigitiser *afe);
	void (*disable)(struct imagedigitiser *afe);
	int (*get_config)(struct imagedigitiser *afe, struct scanunit_config *config);
	int (*set_config)(struct imagedigitiser *afe, const struct scanunit_config *config);
};


/* function prototypes of image digitiser driver (chip level) */
extern int imagedigitiser_add(struct imagedigitiser *afe);
extern int imagedigitiser_remove(struct imagedigitiser *afe);


/* function prototypes of image digitiser driver (user level) */
extern struct imagedigitiser *imagedigitiser_get(struct device *dev);
extern struct imagedigitiser *of_imagedigitiser_get(struct device_node *np);
extern struct imagedigitiser *of_node_to_imagedigitiser(struct device_node *np);
extern void imagedigitiser_put(struct imagedigitiser *afe);

extern int imagedigitiser_enable(struct imagedigitiser *afe);
extern void imagedigitiser_disable(struct imagedigitiser *afe);

extern int imagedigitiser_get_config(struct imagedigitiser *afe, struct scanunit_config *config);
extern int imagedigitiser_set_config(struct imagedigitiser *afe, const struct scanunit_config *config);


#endif /* __IMAGEDIGITISER_H__ */
