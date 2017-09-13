/*
 * Photo Sensor device driver definitions
 *
 * Copyright 2016 Hunan GreatWall Information Financial Equipment Co., Ltd.
 *
 */
#ifndef __PHOTOSENSOR_H__
#define __PHOTOSENSOR_H__

#include <linux/device.h>
#include <linux/err.h>
#include <linux/of.h>


typedef enum {
	PHOTOSENSOR_DIGITAL,
	PHOTOSENSOR_ANALOG
} sensor_type;


typedef enum{
	PHOTOSENSOR_THROUGHBEAM,
	PHOTOSENSOR_REFLECTIVE
}sensor_mode_t;

typedef enum{
	PHOTOSENSOR_0MEANS_DETECTED,
	PHOTOSENSOR_1MEANS_DETECTED
}sensor_polarity_t;

typedef enum{
        PHOTOSENSOR_LEDCONTRL_PWM,
        PHOTOSENSOR_LEDCONTRL_GPIO
}sensor_ledcontrl_t;

/* definition of photosensor status */
#define PHOTOSENSOR_COVERED			(1 << 0)

/* macros to test photosensor status */
#define photosensor_is_covered(s)		((s) & PHOTOSENSOR_COVERED)


#define MINIMUM_BRIGHTNESS	0
#define MAXIMUM_BRIGHTNESS	255


/* photosensor trigger configuration block */
struct sensor_trigger {
	int mode;				// trigger mode: 0 - from COVERED to UNCOVERED; 1 from UNCOVERED to COVERED;
	int enable;				// trigger enable flag: 0 - disabled; 1 - enabled
};


/* photosensor configuration block */
struct photosensor_config {
	int led_brightness;			// LED brightness level of photosensor: should be at [0,255]
	struct sensor_trigger trigger;		// photosensor trigger information
	unsigned long compare_threshold;	// ADC compare threshold value (for analog sensor ONLY)
        int sample_rate;                        // ADC sample rate
};


/* photosensor feature block */
struct photosensor_feature {
	int led_brightness_max;			// maximum LED brightness level of photosensor
	unsigned long raw_input_max;		// maximum raw input value
	int input_scale_mv;			// scale to convert raw input value to voltage (mV)
	#if 1   //add by hl 2016.11.29
	int covered_mode;
	int calibrate_mode;			//0——only without paper， 1——only with paper
	#endif
};


/*
 * struct photosensor
 *
 * One for each photosensor device.
 */
struct photosensor {
	struct device *dev;
	struct list_head list;
	sensor_type type;
	#if 1   //add by hl 2016.11.29
	sensor_mode_t sensor_mode;
	sensor_polarity_t sensor_polarity;
	#endif
	#if 1
	u32 dig_bit_index;
	#endif
        int led_contrl;
        u32 threshold;
};


static inline sensor_type photosensor_type(struct photosensor *sensor)
{
	return sensor->type;
}


static inline const char *photosensor_get_label(struct photosensor *sensor)
{
	return (!sensor) ? ERR_PTR(-ENODEV) : sensor->dev->of_node->name;
}


/* function prototypes of photo sensor driver */
extern struct photosensor *photosensor_get(struct device *dev);
extern struct photosensor *of_photosensor_get(struct device_node *np);
extern struct photosensor *of_node_to_photosensor(struct device_node *np);
extern void photosensor_put(struct photosensor *sensor);

extern int photosensor_enable(struct photosensor *sensor);
extern int photosensor_disable(struct photosensor *sensor);
extern int photosensor_status(struct photosensor *sensor, int *status);
extern int photosensor_read_input(struct photosensor *sensor, unsigned long *val);
extern int photosensor_get_feature(struct photosensor *sensor, struct photosensor_feature *feature);

extern int photosensor_get_config(struct photosensor *sensor, struct photosensor_config *config);
extern int photosensor_set_config(struct photosensor *sensor, const struct photosensor_config *config);

//del by hl 2016.11.2
//extern int photosensor_get_trigger(struct photosensor *sensor, struct sensor_trigger *trigger);
//extern int photosensor_set_trigger(struct photosensor *sensor, const struct sensor_trigger *trigger);

extern int photosensor_get_trigger_next(struct photosensor *sensor, struct sensor_trigger *trigger);
extern int photosensor_set_trigger_next(struct photosensor *sensor, const struct sensor_trigger *trigger);
extern int photosensor_clear_trigger_next(struct photosensor *sensor);

#endif /* __PHOTOSENSOR_H__ */
