#ifndef __STEPPERMOTOR_H__
#define __STEPPERMOTOR_H__

#include <linux/device.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include "motor.h"


/* definition of steppermotor status */
#define STEPPERMOTOR_RUNNING			(1 << 0)
#define STEPPERMOTOR_STOPPED_BY_SENSOR		(1 << 1)
#define STEPPERMOTOR_STOPPED_BY_TOTAL_STEPS	(1 << 2)
#define STEPPERMOTOR_STOPPED_BY_TRIGGER		(1 << 3)
#define STEPPERMOTOR_STOPPED_BY_SKEW		(1 << 4)
#define STEPPERMOTOR_TRIGGER_STEPS_DONE		(1 << 5)
#define STEPPERMOTOR_MEDIALENGTH_OK		(1 << 6)
#define STEPPERMOTOR_FAULT			(1 << 15)
#define STEPPERMOTOR_STOPPED_MASK		(STEPPERMOTOR_STOPPED_BY_TOTAL_STEPS | STEPPERMOTOR_STOPPED_BY_SENSOR | STEPPERMOTOR_STOPPED_BY_TRIGGER | STEPPERMOTOR_STOPPED_BY_SKEW)

/* macros to test steppermotor status */
#define steppermotor_is_running(s)			((s) & STEPPERMOTOR_RUNNING)
#define steppermotor_is_medialength_ok(s)		((s) & STEPPERMOTOR_MEDIALENGTH_OK)
#define steppermotor_is_stopped_by_total_steps(s)	((s) & STEPPERMOTOR_STOPPED_BY_TOTAL_STEPS)
#define steppermotor_is_stopped_by_trigger(s)		((s) & STEPPERMOTOR_STOPPED_BY_TRIGGER)
#define steppermotor_is_stopped_by_sensor(s)		((s) & STEPPERMOTOR_STOPPED_BY_SENSOR)
#define steppermotor_is_stopped_by_skew(s)		((s) & STEPPERMOTOR_STOPPED_BY_SKEW)
#define steppermotor_is_triggersteps_done(s)		((s) & STEPPERMOTOR_TRIGGER_STEPS_DONE)
#define steppermotor_is_fault(s)			((s) & STEPPERMOTOR_FAULT)

/* definition of steppermotor features */
#define STEPPERMOTOR_FIXED_SPEEDS			(1 << 0)
#define STEPPERMOTOR_SUPPORT_LOCK			(1 << 1)
#define STEPPERMOTOR_SUPPORT_MEDIALENGTH		(1 << 2)
#define STEPPERMOTOR_SUPPORT_SENSORTRIGGER		(1 << 3)
#define STEPPERMOTOR_SUPPORT_SKEWSTOP			(1 << 4)


/* macros to test steppermotor features */
#define steppermotor_is_fixed_speeds(f)			((f) & STEPPERMOTOR_FIXED_SPEEDS)
#define steppermotor_is_support_lock(f)			((f) & STEPPERMOTOR_SUPPORT_LOCK)
#define steppermotor_is_support_medialength(f)		((f) & STEPPERMOTOR_SUPPORT_MEDIALENGTH)
#define steppermotor_is_support_sensortrigger(f)	((f) & STEPPERMOTOR_SUPPORT_SENSORTRIGGER)
#define steppermotor_is_support_skewstop(f)		((f) & STEPPERMOTOR_SUPPORT_SKEWSTOP)


#define MAX_SPEED_NUMS		32
#define STOP_TABLE_DEEP		(256)
#define MOTO_RUN_STEP_MASK	0xffff


/* detailed information of a speed */
struct speed_detail {
	int speed;				// speed (in SPS [full Steps per Second])
	int accel_steps;			// full steps to accelerate
	int decel_steps;			// full steps to decelerate
};


/* information block to define a speed-shift */
struct speed_shift {
	int speed1;				// speed 1 (in SPS)
	int speed2;				// speed 2 (in SPS)
	int steps;				// full steps to shift
};


/* steppermotor feature information block */
struct steppermotor_feature {
	int feature_flag;				// feature flags
	int max_steps;					// maximum steps value (hardware or software limitation)
	int pullin_speed;				// pull-in speed (in SPS [full Steps per Second])
	int num_speed;					// number of speed supported
	struct speed_detail speeds[MAX_SPEED_NUMS];	// list of supported speeds
	int num_speedshift;				// number of speed-shift supported
	struct speed_shift speedshifts[MAX_SPEED_NUMS];	// list of supported speed-shifts 
};


/* speed information block to define a speed section */
struct speed_info {
	int speed;				// speed in SPS (full Steps per Second)
	int steps;				// number of steps to run at current speed
	struct speed_info *nextspeed;		// pointer to next speed information block
};


/* steppermotor configuration block to define motion direction, total steps and speed profile information */
struct steppermotor_config {
	motion_dir dir;				// motor motion direction
	int steps_to_run;			// total number of steps to run
	int num_speed;				// number of speed info blocks
	struct speed_info *speedinfo;		// pointer to the first speed information block
};


/*
 * struct steppermotor
 *
 * One for each steppermotor device.
 */
struct steppermotor {
	struct device *dev;
	struct list_head list;
	struct steppermotor_feature feature;
	int using;
	int status;
	struct steppermotor_config config;		// copy of current motor motion configuration
	const struct steppermotor_ops *ops;
	void (*callback)(struct steppermotor *motor, struct callback_data *data);	// callback handling steppermotor events after a motor interrupt
	struct callback_data callbackdata;
	void (*callback_per_step)(struct steppermotor *motor, struct callback_data *data);
	struct callback_data callbackdata_per_step;
};


/*
 * struct steppermotor_trigger
 */
struct steppermotor_trigger{
	int control_set_trigger_stop : 1;
	int control_set_trigger_sensor : 1;
	int control_set_sensor_stop : 1;
	int control_set_sensor_continue_mode : 1;
	int control_set_sensor_stop_mode : 1;
	int control_set_en_skew_steps : 1;
	int control_set_motor_en_hold:1;
	int steps;
};


/*
 * struct steppermotor_ops - steppermotor operations
 * @config: set configure of this steppermotor
 * @status: get status of this steppermotor
 * @start: start running this steppermotor
 * @stop: stop running this steppermotor gracefully 
 * @emergencybrake: stop running this steppermotor in emergency 
 * @lock: (optional) lock this steppermotor
 * @unlock: (optional) unlock this steppermotor
 * @get_running_steps: get running steps of this steppermotor
 * @get_medialength_in_steps: (optional) get media length in steps binding to this steppermotor
 * @set_sensor_sel_mask: (optional) set sensor selecting mask of this steppermotor
 * @set_skew_steps: (optional) set skew steps of this steppermotor
 * @set_trigger_next: (optional) set next trigger configuration of this steppermotor
 */ 
struct steppermotor_ops {
	int	(*config)(struct steppermotor *motor, const struct steppermotor_config *config);
	int	(*status)(struct steppermotor *motor);

	int	(*start)(struct steppermotor *motor);
	void	(*stop)(struct steppermotor *motor);
	void	(*stop_after_steps)(struct steppermotor *motor, unsigned int stpe);
	int	(*lock)(struct steppermotor *motor);
	int	(*unlock)(struct steppermotor *motor);
	void	(*emergencybrake)(struct steppermotor *motor);

	int	(*get_running_steps)(struct steppermotor *motor);
	int	(*get_medialength_in_steps)(struct steppermotor *motor);

	int	(*set_sensor_sel_mask)(struct steppermotor *motor, int sensel);

	int	(*set_skew_steps)(struct steppermotor *motor, int steps);
	int	(*set_trigger_next)(struct steppermotor *motor, const struct steppermotor_trigger *trigger);

	struct module	*owner;
};


/* lower-level function prototypes of steppermotor driver */
int steppermotor_add(struct steppermotor *motor);
int steppermotor_remove(struct steppermotor *motor);


/* macro definition of steppermotor driver */
static inline const char *steppermotor_get_label(struct steppermotor *motor)
{
	return (!motor) ? ERR_PTR(-EINVAL) : motor->dev->of_node->name;
}


static inline const struct steppermotor_feature *steppermotor_get_feature(struct steppermotor *motor)
{
	return (!motor) ? ERR_PTR(-EINVAL) : &motor->feature;
}


/* upper-level function prototypes of steppermotor driver */
struct steppermotor *steppermotor_get(struct platform_device *pdev);
void steppermotor_put(struct steppermotor *motor);
struct steppermotor *of_steppermotor_get(struct device_node *np);
struct steppermotor *of_node_to_steppermotor(struct device_node *np);

int steppermotor_get_config(struct steppermotor *motor, struct steppermotor_config *config);
int steppermotor_set_config(struct steppermotor *motor, const struct steppermotor_config *config);

int steppermotor_set_callback(struct steppermotor *motor, void (*callback)(struct steppermotor *, struct callback_data *),
			      struct callback_data *data);
int steppermotor_set_callback_per_step(struct steppermotor *motor, void (*callback)(struct steppermotor *, struct callback_data *),
			      struct callback_data *data);
int steppermotor_status(struct steppermotor *motor);

int steppermotor_start(struct steppermotor *motor);
void steppermotor_stop(struct steppermotor *motor);
void steppermotor_stop_after_steps(struct steppermotor *motor, unsigned int steps);
void steppermotor_emergencybrake(struct steppermotor *motor);
void steppermotor_lock(struct steppermotor *motor);
void steppermotor_unlock(struct steppermotor *motor);

int steppermotor_get_running_steps(struct steppermotor *motor);
int steppermotor_get_medialength_in_steps(struct steppermotor *motor);

int steppermotor_get_sensor_sel_mask(struct steppermotor *motor, int *sensel);
int steppermotor_set_sensor_sel_mask(struct steppermotor *motor, int sensel);

int steppermotor_set_skew_steps(struct steppermotor *motor, int steps);
int steppermotor_set_trigger_next(struct steppermotor *motor, const struct steppermotor_trigger *trigger);

#endif /* __STEPPERMOTOR_H__ */
