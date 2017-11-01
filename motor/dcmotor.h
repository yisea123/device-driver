#ifndef __DCMOTOR_H__
#define __DCMOTOR_H__

#include <linux/device.h>
#include <linux/err.h>
#include <linux/of.h>
#include <motor.h>


/* definition of dcmotor status */
#define DCMOTOR_RUNNING				(1 << 0)
#define DCMOTOR_STOPPED_BY_SENSOR		(1 << 1)   
#define DCMOTOR_ERROR				(1 << 16)

/* macros to test dcmotor status */
#define dcmotor_is_running(s)			((s) & DCMOTOR_RUNNING)
#define dcmotor_is_stopped_by_sensor(s)		((s) & DCMOTOR_STOPPED_BY_SENSOR)
#define dcmotor_is_error(s)			((s) & DCMOTOR_ERROR)


struct dcmotor_config {
	motion_dir dir;				// motor motion direction
	u32 sensor_compare_mode;
	u32 sensor_mask;
};

struct dcmotor_feature{
	int feature_flag;			// feature flags
};

/*
 * struct dcmotor
 *
 * One for each dcmotor device.
 */
struct dcmotor {
	struct device *dev;
	struct list_head list;
	struct dcmotor_feature feature;
	int using;
	int status;
	struct dcmotor_config config;			// copy of current motor motion configuration
	const struct dcmotor_ops *ops;
	void (*callback)(struct dcmotor *motor, struct callback_data *data);	// callback handling dcmotor events after a motor interrupt
	struct callback_data callbackdata;
};


/*
 * struct dcmotor_ops - dcmotor operations
 * @config: set configure of this dcmotor
 * @status: get status of this dcmotor
 * @start: start running this dcmotor
 * @stop: stop running this dcmotor 
 */ 
struct dcmotor_ops {
	int	(*config)(struct dcmotor *motor, const struct dcmotor_config *config);
	int	(*status)(struct dcmotor *motor);

	int	(*start)(struct dcmotor *motor);
	void	(*stop)(struct dcmotor *motor);

	struct module	*owner;
};



/* macro definition of dcmotor driver */
static inline const char *dcmotor_get_label(struct dcmotor *motor)
{
	return (!motor) ? ERR_PTR(-ENODEV) : motor->dev->of_node->name;
}


/* function prototypes of dcmotor driver (low-level) */
int dcmotor_add(struct dcmotor *motor);
int dcmotor_remove(struct dcmotor *motor);


/* function prototypes of photo motor driver */
struct dcmotor *dcmotor_get(struct device *dev);
struct dcmotor *of_dcmotor_get(struct device_node *np);
struct dcmotor *of_node_to_dcmotor(struct device_node *np);
void dcmotor_put(struct dcmotor *motor);

int dcmotor_set_callback(struct dcmotor *motor, void (*callback)(struct dcmotor *, struct callback_data *), struct callback_data *data);

int dcmotor_start(struct dcmotor *motor);
void dcmotor_stop(struct dcmotor *motor);
int dcmotor_status(struct dcmotor *motor);

int dcmotor_get_config(struct dcmotor *sensor, struct dcmotor_config *config);
int dcmotor_set_config(struct dcmotor *sensor, const struct dcmotor_config *config);


#endif /* __DCMOTOR_H__ */
