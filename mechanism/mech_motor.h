#ifndef __MECH_MOTOR_H__
#define __MECH_MOTOR_H__

#include "../motor/steppermotor.h"
#include "../motor/dcmotor.h"
#include "../photosensor/photosensor.h"
#include <mech_base.h>
#include "mech_sensor.h"
#include "dcmotor.h"
#include "mechlib.h"

//-----type of motor-----
//HL£ºshould be defined in sonsor driver files at the end.
#define MOTOR_STEP_TYPE		0
#define MOTOR_BRUSHDC_TYPE	1
#define MOTOR_NOBRUSHDC_TYPE	2

//-------------------------------------------------------
#define STEPSPEED_PHASE_NUM 10

typedef  union{
	    void (*dcmotor_callback)(struct dcmotor *, struct callback_data *);
	    void (*steppermotor_callback)(struct steppermotor *, struct callback_data *);
}motor_callback_t;


struct motor_data{
// get from dts(device tree source)
	unsigned short motor_mask;
	unsigned char motor_name[MECHUINT_NAME_LEN];
	union{
		struct steppermotor *psteppermotor;
		struct dcmotor *pdcmotor;
	}motor_dev;
	unsigned char motor_type;
// about move config
    //for sensor triger
    //unsigned char triger_num;
    struct sensor_trigger trigerinfo[STEPSPEED_PHASE_NUM];
    unsigned char phase_current_num;
    unsigned short last_sen_mask;		//sensors roll in this phase
    //unsigned char  last_motor_sen_flag;  //MOTOR_SEN_ARRIVE / MOTOR_SEN_LEAVE
    //for stepmotor move
    struct steppermotor_config step_config;
    struct speed_info speedinfo[STEPSPEED_PHASE_NUM];
    //for dcmotor move
    struct dcmotor_config dc_config;

    motor_mov_t *pmotor_mov;
// completion
    struct completion motor_completion;
    unsigned int motor_comp_accout;

    struct completion motor_phase_completion;
    unsigned int motor_phase_accout;
//about moving status
    /*#define MOTOR_MOVE_STATUS_NORMAL    0x000
    #define MOTOR_WAIT_TRIGER_TIMEOUT   0x100
    #define MOTOR_WAIT_STOP_TIMEOUT     0x200*/
    #if MECH_OPTIMIZE_20170515
    unsigned char moving_status;
    unsigned long stoping_status;
    #else
    unsigned long moving_status;
    #endif
    #ifdef MECH_OPTIMIZE_20170606
    int err_status;
    #endif
    motor_callback_t callback;
};

typedef struct {
	unsigned char  motor_num;
	struct motor_data *motor;
}mechanism_uint_motor_data_t;

#define motor_get_data(punit_motor_data, motormask, pmotor_data, i) \
     for(i=0; i<(punit_motor_data)->motor_num; i++) \
        if((punit_motor_data)->motor[i].motor_mask== (motormask)) \
        {\
            pmotor_data = &((punit_motor_data)->motor[i]); \
            break;\
        } \


static inline void motormove_err_callback(struct motor_data *pmotor_data, int ret)
{
	switch (ret) 
	{
	case -RESN_MECH_ERR_MOTOR_WAIT_TRIGER_TIMEOUT:
	case -RESN_MECH_ERR_MOTOR_WAIT_STOP_TIMEOUT:
	case -RESN_MECH_ERR_MOTOR_MOVE_SET_CONFIG_ERR:
	case -RESN_MECH_ERR_MOTOR_MOVE_SET_SENMASK_ERR:
	case -RESN_MECH_ERR_MOTOR_MOVE_SET_TRIGGER_NEXT:
	case -RESN_MECH_ERR_MOTOR_MOVE_START_ERR:
	case -RESN_MECH_ERR_MOTOR_SENSOR_CONFIG_ERR:
		printk("motormove_err_callback motor_phase_accout=%d motor_comp_accout=%d moving_status=%x\n", 
			pmotor_data->motor_phase_accout, pmotor_data->motor_comp_accout, pmotor_data->moving_status);

		if (pmotor_data->motor_phase_accout != 0)
    			complete(&pmotor_data->motor_phase_completion);
		if (pmotor_data->motor_comp_accout != 0)
    			complete_all(&(pmotor_data->motor_completion));
	
		#ifdef MECH_OPTIMIZE_20170515
		pmotor_data->moving_status = MOTOR_MOVE_STATUS_STOP;
		pmotor_data->stoping_status = MOTOR_STOP_BY_ABNORMAL;
		#else
		pmotor_data->moving_status |= MOTOR_STOP_BY_ABNORMAL;
		//printk("moving_status=%x\n", pmotor_data->moving_status);
		pmotor_data->moving_status &= ~(MOTOR_MOVE_STATUS_RUNNING|MOTOR_MOVE_STATUS_INUSE);
		//printk("moving_status=%x\n", pmotor_data->moving_status);
		#endif
		return;
	default:
		break;
	}
}

#ifdef MECH_OPTIMIZE_20170606
static inline  int step_motor_triger_deal(struct motor_data *pmotor_data, char triger_index, mechanism_uint_motor_data_t *punit_motor_data, mechanism_uint_sensor_data_t *punit_sensor_data)
{
	int ret = 0;
	motor_trigger_phase_t *motor_trigger_phase;
	struct steppermotor_trigger motor_trigger;

	motor_trigger_phase = &(pmotor_data->pmotor_mov->motor_trigger_phase[(int)triger_index]);
	motor_trigger.steps = motor_trigger_phase->to_trigger_steps;
	motor_trigger.control_set_trigger_stop = motor_trigger_phase->motor_triger_flag.motor_trigger_stop_flag;
	motor_trigger.control_set_trigger_sensor = motor_trigger_phase->motor_triger_flag.motor_trigger_sensor_flag;
	motor_trigger.control_set_sensor_stop = motor_trigger_phase->motor_triger_flag.motor_sensor_stop_flag;
	motor_trigger.control_set_sensor_continue_mode = motor_trigger_phase->motor_triger_flag.motor_sensor_continue_mode;
	motor_trigger.control_set_sensor_stop_mode = motor_trigger_phase->motor_triger_flag.motor_sensor_stop_mode;
	motor_trigger.control_set_en_skew_steps = motor_trigger_phase->motor_triger_flag.motor_en_skew_steps;

	if(motor_trigger_phase->sen_mask)
	{
		ret=sensor_set_trigger_next(punit_sensor_data, motor_trigger_phase->sen_mask, 
			       motor_trigger_phase->motor_sen_flag);
		if(ret)
		{
			 printk("step_motor_start:sensor_set_trigger_next error!\n");
			 ret = -RESN_MECH_ERR_MOTOR_SENSOR_CONFIG_ERR;
			 motormove_err_callback(pmotor_data, ret);
			 return ret;
		}
		pmotor_data->last_sen_mask = motor_trigger_phase->sen_mask;
	}

			
			
	pr_debug("%dto_trigger_steps=%ld \n",triger_index,
		motor_trigger_phase->to_trigger_steps);
	#if 0
	pr_debug("%d %d %d %d %d\n",
		motor_trigger_phase->motor_triger_flag.motor_trigger_stop_flag,
		motor_trigger_phase->motor_triger_flag.motor_trigger_sensor_flag,
		motor_trigger_phase->motor_triger_flag.motor_sensor_stop_flag,
		motor_trigger_phase->motor_triger_flag.motor_sensor_continue_mode,
		motor_trigger_phase->motor_triger_flag.motor_sensor_stop_mode); 
	#endif

	
	ret=steppermotor_set_trigger_next(pmotor_data->motor_dev.psteppermotor, &motor_trigger);
	if(ret)
	{
		printk("step_motor_start:steppermotor_set_triggersteps_next error!\n");
		ret = -RESN_MECH_ERR_MOTOR_MOVE_SET_TRIGGER_NEXT;
		motormove_err_callback(pmotor_data, ret);
		return ret;
	}
	return ret;

}
#endif

#if 0//ndef MECH_OPTIMIZE_20170428
extern int motor_init(mechanism_uint_motor_data_t *punit_motor_data, unsigned short motor_mask, void (*callback)(void *motor));
#endif
extern int motor_move_init(mechanism_uint_motor_data_t *punit_motor_data, unsigned short motor_mask);
extern int motor_start(mechanism_uint_motor_data_t *punit_motor_data, mechanism_uint_sensor_data_t *punit_sensor_data, 
	unsigned short motor_mask, unsigned char dir, motor_mov_t  *pmotor_mov);
extern int motor_stop(mechanism_uint_motor_data_t *punit_motor_data, unsigned short motor_mask);
extern int bmotor_stoped(mechanism_uint_motor_data_t *punit_motor_data, unsigned short motor_mask);
extern int motor_wait_stop(mechanism_uint_motor_data_t *punit_motor_data, mechanism_uint_sensor_data_t *punit_sensor_data, unsigned short motor_mask);
extern int motor_get_feature(mechanism_uint_motor_data_t *punit_motor_data, motor_feature_t *p_motorfeature);
extern int motor_lock(mechanism_uint_motor_data_t *punit_motor_data, unsigned short motor_mask);
extern int motor_unlock(mechanism_uint_motor_data_t *punit_motor_data, unsigned short motor_mask);
extern int motor_skewsteps_set(mechanism_uint_motor_data_t *punit_motor_data, unsigned short motor_mask, int skew_steps);
extern int motor_get_running_steps(mechanism_uint_motor_data_t *punit_motor_data, unsigned short motor_mask, int *psteps);
#endif

