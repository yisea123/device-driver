#ifndef __MECH_SENSOR_H__
#define __MECH_SENSOR_H__

#include <mechlib.h>
#include "../photosensor/photosensor.h"
#include <mech_base.h>
//#include "mech_unit.h"

//---------------------------------------
#define SEN_ANALOG_TYPE	    	PHOTOSENSOR_ANALOG
#define SEN_DIGITAL_TYPE	PHOTOSENSOR_DIGITAL

#define SEN_MEDIA_IN	0
#define SEN_MEDIA_OUT	1

//-------------------------------------------------------
// database struct of sensors in a mechanism unit.
struct sensor_data{
//defined in code
    unsigned short sen_mask;
    unsigned char sen_name[MECHUINT_NAME_LEN];
// get from dts(device tree source)
    union{
        struct photosensor *pphotosensor;
    }sen_dev;
    unsigned char sen_type;
// for config the photosensor
    struct photosensor_config config;

};

typedef struct {
	unsigned char  sensor_num;
	struct sensor_data *sensor;
	unsigned short sensor_masks;
}mechanism_uint_sensor_data_t;

#define sensor_get_data(punit_sensor_data, senmask, psen_data, i) \
     for(i=0; i<(punit_sensor_data)->sensor_num; i++) \
        if((punit_sensor_data)->sensor[i].sen_mask== (senmask)) \
        {\
            psen_data = &((punit_sensor_data)->sensor[i]); \
            break;\
        } \


//----------------------------------------------------
//----------------------sensor init----------------------
extern int sensor_init(mechanism_uint_sensor_data_t *punit_sensor_data,  unsigned short sen_masks);
//----------------------sensor enable/disable---------------------- 
extern int sensor_enable(mechanism_uint_sensor_data_t *punit_sensor_data, unsigned short sen_masks, unsigned char enable);
//----------------------get the output value of sensor----------------------
extern int sensor_get_val(mechanism_uint_sensor_data_t *punit_sensor_data, unsigned short sen_mask, unsigned long *val);
//----------------------get the status of sensor(detected or undetected)----------------------
extern int sensor_get_appval(mechanism_uint_sensor_data_t *punit_sensor_data, unsigned short sen_mask, unsigned int *appval);
//----------------------set sensor trigger----------------------
extern int sensor_set_trigger(mechanism_uint_sensor_data_t *punit_sensor_data, unsigned short sen_masks, unsigned char trigger_type);
//----------------------set sensor next trigger----------------------
extern int sensor_set_trigger_next(mechanism_uint_sensor_data_t *punit_sensor_data, unsigned short sen_masks, unsigned char trigger_type);
//----------------------clear sensor next trigger----------------------
extern int sensor_clear_trigger_next(mechanism_uint_sensor_data_t *punit_sensor_data, unsigned short sen_masks);
//----------------------set sensor config----------------------
extern int sensor_set_config(mechanism_uint_sensor_data_t *punit_sensor_data, mech_unit_sen_config_t *pmech_unit_sen_config, sen_config_t *p_sen_config );
//----------------------get sensor feature----------------------
extern int sensor_get_feature(mechanism_uint_sensor_data_t *punit_sensor_data, sen_feature_t *p_ppsfeature);

#endif

