#ifndef __STEPTABLE_H__
#define __STEPTABLE_H__

#include <linux/device.h>
#include <linux/err.h>
#include <linux/of.h>

#include "steppermotor.h"


/*******************************************************************************
 * data structures of lower-level steppermotor driver implemention	       *
 *******************************************************************************/

#define MAX_RAMP_LEN	1024

/* struct speed_ramp (ramptable of a speed) */
struct speed_ramp {
	struct motor_speedtable *accel_table;	// pointer to accelation speedtable
	struct motor_speedtable *decel_table;	// pointer to decelation speedtable
	u32 step_ticks;				// ticks per step at current speed  
};


/* struct speedshift_ramp (ramptable of a speed-shift) */
struct speedshift_ramp {
	struct motor_speedtable *shift_table;	// pointer to shift speedtable
	u32 step_ticks_1;			// ticks per step at speed 1
	u32 step_ticks_2;			// ticks per step at speed 2
};


/* struct ramp_info (speeds + speed-shifts)*/
struct ramp_info {
	int num_speed;						// number of speed supported
	int num_speedshift;					// number of speed-shift supported
	struct speed_ramp speeds[MAX_SPEED_NUMS];		// speed ramp tables
	struct speedshift_ramp speedshifts[MAX_SPEED_NUMS];	// speedshift ramp tables
};


/*
 * struct steppermotor speed table
 */
struct motor_speedtable {
	int stepping;
	u32 ramp_size;
	u32 start_speed;
	u32 object_speed;
	struct list_head list;
	u32 ramp_table[];
};


/*******************************************************************************
 * definitions of macro to lookup steppermotor speedtable		       *
 *******************************************************************************/
static inline int lookup_speedtable(struct steppermotor *motor, int speed)
{
	int i;
	for (i = 0; i < motor->feature.num_speed; i++)
		if (speed == motor->feature.speeds[i].speed)
			return i;
	return -EINVAL;
}


static inline int lookup_shifttable(struct steppermotor *motor, int speed1, int speed2)
{
	int i;
	for (i = 0; i < motor->feature.num_speedshift; i++)
		if ((speed1 == motor->feature.speedshifts[i].speed1) &&
		    (speed2 == motor->feature.speedshifts[i].speed2))
			return i;
	return -EINVAL;
}


/*******************************************************************************
 * macro definitions of lower-level steppermotor driver implemention	       *
 *******************************************************************************/
int steppermotor_speedtable_parse(struct device *dev, struct list_head *speedtable_list, int stepping);
int steppermotor_speedtable_analysis(struct device *dev, struct list_head *speedtable_list, struct steppermotor_feature *feature, struct ramp_info *rampinfo);
int steppermotor_ramptable_convert(struct ramp_info *rampinfo, int clock);
int steppermotor_check_config(struct steppermotor *motor, const struct steppermotor_config *config);


#endif /* __STEPTABLES_H__ */
