#include <linux/module.h>

#include "mech_sensor.h"
#include "mech_motor.h"


//#define COMPLETION_TIMEOUT  0x3ff
#define COMPLETION_TIMEOUT(steps)	(steps/2)
//#define	PHASE_COMPLETION_TIMEOUT(phase_steps)	(phase_steps/2)
#define	PHASE_COMPLETION_TIMEOUT(phase_steps)	(0x3ff)

static void step_motor_stop(struct motor_data *pmotor_data )
{
	steppermotor_stop(pmotor_data->motor_dev.psteppermotor); 
	//steppermotor_emergencybrake(pmotor_data->motor_info.psteppermotor);
}

#ifdef MECH_OPTIMIZE_20170606
static int step_motor_start(mechanism_uint_motor_data_t *punit_motor_data, mechanism_uint_sensor_data_t *punit_sensor_data,
	struct motor_data *pmotor_data,unsigned char dir,  motor_mov_t  * pmotor_mov)		
{
	unsigned char i;
	int ret=0;
	int timeout_ret;
	
	unsigned short sensor_mask;

	motor_trigger_phase_t *motor_trigger_phase;

	pr_debug("step_motor_start.................speed_phase_num=%d\n", pmotor_mov->speed_phase_num);
	if ((pmotor_mov->speed_phase_num > STEPSPEED_PHASE_NUM) || (pmotor_mov->trigger_phase_num > STEPSPEED_PHASE_NUM)) {
		return -1;
	}

	pmotor_data->pmotor_mov = pmotor_mov;

	// get speed config and set
	pmotor_data->step_config.steps_to_run = 0;
	pmotor_data->step_config.dir = dir;
	pmotor_data->step_config.num_speed = pmotor_data->pmotor_mov->speed_phase_num;
	pmotor_data->step_config.speedinfo = pmotor_data->speedinfo;
	pmotor_data->moving_status = MOTOR_MOVE_STATUS_INUSE; 
	pmotor_data->phase_current_num = 0;
	pmotor_data->last_sen_mask = 0;
	#ifdef MECH_OPTIMIZE_20170606
	pmotor_data->stoping_status = 0;
	pmotor_data->err_status = 0;
	#endif

	for (i=0; i<pmotor_mov->speed_phase_num; i++) {
		pmotor_data->speedinfo[i].speed = pmotor_data->pmotor_mov->motor_speed_phase[i].speed;
		pmotor_data->speedinfo[i].steps = pmotor_data->pmotor_mov->motor_speed_phase[i].speed_steps;
		pmotor_data->step_config.steps_to_run += pmotor_data->pmotor_mov->motor_speed_phase[i].speed_steps;
		pr_debug("speed_phase_%d speed_steps:%d %d\n", i, pmotor_data->pmotor_mov->motor_speed_phase[i].speed_steps, pmotor_mov->motor_speed_phase[i].speed_steps);
		pr_debug("speed_phase_%d speed:%d %d\n", i, pmotor_data->pmotor_mov->motor_speed_phase[i].speed, pmotor_mov->motor_speed_phase[i].speed);
		if (i<pmotor_mov->speed_phase_num-1){
			pmotor_data->speedinfo[i].nextspeed = &pmotor_data->speedinfo[i+1];
		}
		else
			pmotor_data->speedinfo[i].nextspeed = NULL;
	}

	pr_debug("step_motor_start:dir=%d num_speed=%d steps_to_run=%d\n", pmotor_data->step_config.dir, pmotor_data->step_config.num_speed, pmotor_data->step_config.steps_to_run);
	pr_debug("step_motor_start:speedinfo： speed=%d steps=%d \n", pmotor_data->step_config.speedinfo[0].speed, pmotor_data->step_config.speedinfo[0].steps); 
	pr_debug("psteppermotor=%x\n", (int)pmotor_data->motor_dev.psteppermotor);

	ret=steppermotor_set_config(pmotor_data->motor_dev.psteppermotor, &(pmotor_data->step_config));
	if(ret)
	{
		pr_debug("step_motor_start:steppermotor_set_config error!\n");
		ret = -MECH_ERR_MOTOR_MOVE_SET_CONFIG_ERR;
		motormove_err_callback(pmotor_data, ret);
		return ret;
	}

	//get triger config and set
	if (pmotor_data->pmotor_mov->medialegth_detect) {
		sensor_mask = pmotor_data->pmotor_mov->medialength_source->sensor_mask;
	}
	else
		sensor_mask = 0;
	ret=steppermotor_set_sensor_sel_mask(pmotor_data->motor_dev.psteppermotor, sensor_mask);
	if(ret)
	{
		pr_debug("step_motor_start:steppermotor_set_sensor_sel_mask error!\n");
		ret = -MECH_ERR_MOTOR_MOVE_SET_SENMASK_ERR;
		motormove_err_callback(pmotor_data, ret);
		return ret;
	}


	if (pmotor_data->pmotor_mov->trigger_phase_num){
		pr_debug("step_motor_start:triger_num=%d triger_mask=%x  sen_step=%ld\n", 
			pmotor_data->pmotor_mov->trigger_phase_num, 
			pmotor_mov->motor_trigger_phase[0].sen_mask, pmotor_mov->motor_trigger_phase[0].to_trigger_steps);

		#if 0
		for (i = 0; i < pmotor_data->pmotor_mov->trigger_phase_num; i++) {
			motor_trigger_phase = &(pmotor_data->pmotor_mov->motor_trigger_phase[i]);
			motor_trigger[i].steps = motor_trigger_phase->to_trigger_steps;
			motor_trigger[i].control_set_trigger_stop = motor_trigger_phase->motor_triger_flag.motor_trigger_stop_flag;
			motor_trigger[i].control_set_trigger_sensor = motor_trigger_phase->motor_triger_flag.motor_trigger_sensor_flag;
			motor_trigger[i].control_set_sensor_stop = motor_trigger_phase->motor_triger_flag.motor_sensor_stop_flag;
			motor_trigger[i].control_set_sensor_continue_mode = motor_trigger_phase->motor_triger_flag.motor_sensor_continue_mode;
			motor_trigger[i].control_set_sensor_stop_mode = motor_trigger_phase->motor_triger_flag.motor_sensor_stop_mode;
			motor_trigger[i].control_set_en_skew_steps = motor_trigger_phase->motor_triger_flag.motor_en_skew_steps;
		}
		#endif
		pmotor_data->motor_phase_accout++;
		ret = step_motor_triger_deal(pmotor_data, 0, punit_motor_data, punit_sensor_data);
		if (ret) {
			return ret;
		}
		ret=steppermotor_start(pmotor_data->motor_dev.psteppermotor);
		if(ret)
		{
			pr_debug("step_motor_start:steppermotor_start error!\n");
			ret = -MECH_ERR_MOTOR_MOVE_START_ERR;
			motormove_err_callback(pmotor_data, ret);
			return ret;
		}
		pmotor_data->moving_status = MOTOR_MOVE_STATUS_RUNNING; 

		for (i = 0; i < pmotor_data->pmotor_mov->trigger_phase_num; i++) {
			motor_trigger_phase = &(pmotor_data->pmotor_mov->motor_trigger_phase[i]);
			if (!pmotor_data->motor_phase_accout) {
				pmotor_data->motor_phase_accout++;
			}
			
			pr_debug("%dphase start to wait \n", i);
			//timeout_ret=wait_for_completion_timeout(&(pmotor_data->motor_phase_completion), COMPLETION_TIMEOUT);
			timeout_ret = wait_for_completion_timeout(&(pmotor_data->motor_phase_completion), PHASE_COMPLETION_TIMEOUT(pmotor_data->pmotor_mov->motor_trigger_phase[i].to_trigger_steps)); 
			pr_debug("timeout_ret=%d \n", timeout_ret);
			if(!timeout_ret)
			{
				pr_debug("step_motor_start: %dphase timeout!\n", i);
				ret = -MECH_ERR_MOTOR_WAIT_TRIGER_TIMEOUT;
				motormove_err_callback(pmotor_data, ret);

				pmotor_data->motor_phase_accout--;
				sensor_clear_trigger_next(punit_sensor_data, motor_trigger_phase->sen_mask);

				step_motor_stop(pmotor_data);
				return ret;
			}
			else{
				pr_debug("%dphase end\n", i);
				pmotor_data->motor_phase_accout--;
				

				ret=sensor_clear_trigger_next(punit_sensor_data, motor_trigger_phase->sen_mask);
				if(ret)
				{
					 printk("step_motor_start:sensor_clear_trigger_next error!\n");
					 ret = -MECH_ERR_MOTOR_SENSOR_CONFIG_ERR;
					 motormove_err_callback(pmotor_data, ret);
					 return ret;
				}
			}

			if(pmotor_data->moving_status == MOTOR_MOVE_STATUS_STOP){
				printk("step_motor_start:next phase stoped!moving_status=%x stoping_status=%lx\n", pmotor_data->moving_status, pmotor_data->stoping_status);
			
				ret=sensor_clear_trigger_next(punit_sensor_data, motor_trigger_phase->sen_mask);
				if(ret)
				{
					 printk("step_motor_start:sensor_clear_trigger_next error!\n");
					 
					 ret = -MECH_ERR_MOTOR_SENSOR_CONFIG_ERR;
					 motormove_err_callback(pmotor_data, ret);
					 return ret;
				}

				if ((pmotor_data->stoping_status & MOTOR_STOP_MASK) ==MOTOR_STOP_BY_ABNORMAL) {
					#ifdef MECH_OPTIMIZE_20170606
					ret = pmotor_data->err_status;
					#else
					if ((pmotor_data->stoping_status & MOTOR_ABNORMAL_MASK)==MOTOR_INT_INVALID)
						ret = -MECH_ERR_MOTOR_INT_INVALID;
					else if ((pmotor_data->stoping_status & MOTOR_ABNORMAL_MASK)== MOTOR_HW_ERR) {
						ret = -MECH_ERR_MOTOR_HW_ERR;
					}
					else 
						ret = 0;
					#endif
					
				}
				else
					ret = 0;
				break;
			}
		}
	}
	pr_debug("step_motor_start.................over!\n");
	return ret;
}
#else
static struct steppermotor_trigger motor_trigger[STEPSPEED_PHASE_NUM];
static int step_motor_start(mechanism_uint_motor_data_t *punit_motor_data, mechanism_uint_sensor_data_t *punit_sensor_data,
	struct motor_data *pmotor_data,unsigned char dir,  motor_mov_t  * pmotor_mov)		
{
	unsigned char i;
	int ret=0;
	int timeout_ret;
	
	unsigned short sensor_mask;

	motor_trigger_phase_t *motor_trigger_phase;

	pr_debug("step_motor_start.................speed_phase_num=%d\n", pmotor_mov->speed_phase_num);
	if ((pmotor_mov->speed_phase_num > STEPSPEED_PHASE_NUM) || (pmotor_mov->trigger_phase_num > STEPSPEED_PHASE_NUM)) {
		return -1;
	}

	pmotor_data->pmotor_mov = pmotor_mov;

	// get speed config and set
	pmotor_data->step_config.steps_to_run = 0;
	pmotor_data->step_config.dir = dir;
	pmotor_data->step_config.num_speed = pmotor_data->pmotor_mov->speed_phase_num;
	pmotor_data->step_config.speedinfo = pmotor_data->speedinfo;
	pmotor_data->moving_status = MOTOR_MOVE_STATUS_INUSE; 
	pmotor_data->phase_current_num = 0;
	pmotor_data->last_sen_mask = 0;

	for (i=0; i<pmotor_mov->speed_phase_num; i++) {
		pmotor_data->speedinfo[i].speed = pmotor_data->pmotor_mov->motor_speed_phase[i].speed;
		pmotor_data->speedinfo[i].steps = pmotor_data->pmotor_mov->motor_speed_phase[i].speed_steps;
		pmotor_data->step_config.steps_to_run += pmotor_data->pmotor_mov->motor_speed_phase[i].speed_steps;
		pr_debug("speed_phase_%d speed_steps:%d %d\n", i, pmotor_data->pmotor_mov->motor_speed_phase[i].speed_steps, pmotor_mov->motor_speed_phase[i].speed_steps);
		pr_debug("speed_phase_%d speed:%d %d\n", i, pmotor_data->pmotor_mov->motor_speed_phase[i].speed, pmotor_mov->motor_speed_phase[i].speed);
		if (i<pmotor_mov->speed_phase_num-1){
			pmotor_data->speedinfo[i].nextspeed = &pmotor_data->speedinfo[i+1];
		}
		else
			pmotor_data->speedinfo[i].nextspeed = NULL;
	}

	pr_debug("step_motor_start:dir=%d num_speed=%d steps_to_run=%d\n", pmotor_data->step_config.dir, pmotor_data->step_config.num_speed, pmotor_data->step_config.steps_to_run);
	pr_debug("step_motor_start:speedinfo： speed=%d steps=%d \n", pmotor_data->step_config.speedinfo[0].speed, pmotor_data->step_config.speedinfo[0].steps); 
	pr_debug("psteppermotor=%x\n", (int)pmotor_data->motor_dev.psteppermotor);

	ret=steppermotor_set_config(pmotor_data->motor_dev.psteppermotor, &(pmotor_data->step_config));
	if(ret)
	{
		pr_debug("step_motor_start:steppermotor_set_config error!\n");
		ret = -MECH_ERR_MOTOR_MOVE_SET_CONFIG_ERR;
		motormove_err_callback(pmotor_data, ret);
		return ret;
	}

	//get triger config and set
	if (pmotor_data->pmotor_mov->medialegth_detect) {
		sensor_mask = pmotor_data->pmotor_mov->medialength_source->sensor_mask;
	}
	else
		sensor_mask = 0;
	ret=steppermotor_set_sensor_sel_mask(pmotor_data->motor_dev.psteppermotor, sensor_mask);
	if(ret)
	{
		pr_debug("step_motor_start:steppermotor_set_sensor_sel_mask error!\n");
		ret = -MECH_ERR_MOTOR_MOVE_SET_SENMASK_ERR;
		motormove_err_callback(pmotor_data, ret);
		return ret;
	}


	if (pmotor_data->pmotor_mov->trigger_phase_num){
		pr_debug("step_motor_start:triger_num=%d triger_mask=%x  sen_step=%ld\n", 
			pmotor_data->pmotor_mov->trigger_phase_num, 
			pmotor_mov->motor_trigger_phase[0].sen_mask, pmotor_mov->motor_trigger_phase[0].to_trigger_steps);
	
		for (i = 0; i < pmotor_data->pmotor_mov->trigger_phase_num; i++) {
			motor_trigger_phase = &(pmotor_data->pmotor_mov->motor_trigger_phase[i]);
			motor_trigger[i].steps = motor_trigger_phase->to_trigger_steps;
			motor_trigger[i].control_set_trigger_stop = motor_trigger_phase->motor_triger_flag.motor_trigger_stop_flag;
			motor_trigger[i].control_set_trigger_sensor = motor_trigger_phase->motor_triger_flag.motor_trigger_sensor_flag;
			motor_trigger[i].control_set_sensor_stop = motor_trigger_phase->motor_triger_flag.motor_sensor_stop_flag;
			motor_trigger[i].control_set_sensor_continue_mode = motor_trigger_phase->motor_triger_flag.motor_sensor_continue_mode;
			motor_trigger[i].control_set_sensor_stop_mode = motor_trigger_phase->motor_triger_flag.motor_sensor_stop_mode;
			motor_trigger[i].control_set_en_skew_steps = motor_trigger_phase->motor_triger_flag.motor_en_skew_steps;
		}

		for (i = 0; i < pmotor_data->pmotor_mov->trigger_phase_num; i++) {
			motor_trigger_phase = &(pmotor_data->pmotor_mov->motor_trigger_phase[i]);

			pmotor_data->motor_phase_accout++;
			if(motor_trigger_phase->sen_mask)
			{
				ret=sensor_set_trigger_next(punit_sensor_data, motor_trigger_phase->sen_mask, 
					       motor_trigger_phase->motor_sen_flag);
				if(ret)
				{
					 printk("step_motor_start:sensor_set_trigger_next error!\n");
					 ret = -MECH_ERR_MOTOR_SENSOR_CONFIG_ERR;
					 motormove_err_callback(pmotor_data, ret);
					 return ret;
				}
				pmotor_data->last_sen_mask = motor_trigger_phase->sen_mask;
			}

			
			
			pr_debug("%dto_trigger_steps=%ld \n",i,
				motor_trigger_phase->to_trigger_steps);
			#if 0
			pr_debug("%d %d %d %d %d\n",
				motor_trigger_phase->motor_triger_flag.motor_trigger_stop_flag,
				motor_trigger_phase->motor_triger_flag.motor_trigger_sensor_flag,
				motor_trigger_phase->motor_triger_flag.motor_sensor_stop_flag,
				motor_trigger_phase->motor_triger_flag.motor_sensor_continue_mode,
				motor_trigger_phase->motor_triger_flag.motor_sensor_stop_mode); 
			#endif

			
			ret=steppermotor_set_trigger_next(pmotor_data->motor_dev.psteppermotor, &motor_trigger[i]);
			if(ret)
			{
				printk("step_motor_start:steppermotor_set_triggersteps_next error!\n");
				ret = -MECH_ERR_MOTOR_MOVE_SET_TRIGGER_NEXT;
				motormove_err_callback(pmotor_data, ret);
				return ret;
			}

			if (i==0) {
				ret=steppermotor_start(pmotor_data->motor_dev.psteppermotor);
				if(ret)
				{
					pr_debug("step_motor_start:steppermotor_start error!\n");
					ret = -MECH_ERR_MOTOR_MOVE_START_ERR;
					motormove_err_callback(pmotor_data, ret);
					return ret;
				}
				pmotor_data->moving_status = MOTOR_MOVE_STATUS_RUNNING; 
			}
		
		
			pr_debug("%dphase start to wait \n", i);
			timeout_ret=wait_for_completion_timeout(&(pmotor_data->motor_phase_completion), COMPLETION_TIMEOUT);
			pr_debug("timeout_ret=%d \n", timeout_ret);
			if(!timeout_ret)
			{
				pr_debug("step_motor_start: %dphase timeout!\n", i);
				ret = -MECH_ERR_MOTOR_WAIT_TRIGER_TIMEOUT;
				motormove_err_callback(pmotor_data, ret);

				pmotor_data->motor_phase_accout--;
				sensor_clear_trigger_next(punit_sensor_data, motor_trigger_phase->sen_mask);
				return ret;
			}
			else{
				pr_debug("%dphase end\n", i);
				pmotor_data->motor_phase_accout--;
				pmotor_data->phase_current_num++;

				ret=sensor_clear_trigger_next(punit_sensor_data, motor_trigger_phase->sen_mask);
				if(ret)
				{
					 printk("step_motor_start:sensor_clear_trigger_next error!\n");
					 ret = -MECH_ERR_MOTOR_SENSOR_CONFIG_ERR;
					 motormove_err_callback(pmotor_data, ret);
					 return ret;
				}
			}
			#ifdef MECH_OPTIMIZE_20170515
			if(pmotor_data->moving_status == MOTOR_MOVE_STATUS_STOP){
				printk("step_motor_start:next phase stoped!moving_status=%x stoping_status=%lx\n", pmotor_data->moving_status, pmotor_data->stoping_status);
			#else
			if(pmotor_data->moving_status & MOTOR_STOP_STATUS){
				printk("step_motor_start:next phase stoped!moving_status=%lx\n", pmotor_data->moving_status);
			#endif
				ret=sensor_clear_trigger_next(punit_sensor_data, motor_trigger_phase->sen_mask);
				if(ret)
				{
					 printk("step_motor_start:sensor_clear_trigger_next error!\n");
					 
					 ret = -MECH_ERR_MOTOR_SENSOR_CONFIG_ERR;
					 motormove_err_callback(pmotor_data, ret);
					 return ret;
				}
				#ifdef MECH_OPTIMIZE_20170515
				if ((pmotor_data->stoping_status & MOTOR_STOP_MASK) ==MOTOR_STOP_BY_ABNORMAL) {
				
					if ((pmotor_data->stoping_status & MOTOR_ABNORMAL_MASK)==MOTOR_INT_INVALID)
						ret = -MECH_ERR_MOTOR_INT_INVALID;
					else if ((pmotor_data->stoping_status & MOTOR_ABNORMAL_MASK)== MOTOR_HW_ERR) {
						ret = -MECH_ERR_MOTOR_HW_ERR;
					}
					else 
						ret = 0;
					
				}
				#else
				if ((pmotor_data->moving_status & MOTOR_STOP_MASK)==MOTOR_STOP_BY_ABNORMAL) {
				
					if ((pmotor_data->moving_status & MOTOR_ABNORMAL_MASK)==MOTOR_INT_INVALID)
						ret = -MECH_ERR_MOTOR_INT_INVALID;
					else if ((pmotor_data->moving_status & MOTOR_ABNORMAL_MASK)== MOTOR_HW_ERR) {
						ret = -MECH_ERR_MOTOR_HW_ERR;
					}
					else 
						ret = 0;
					
				}
				#endif
				else
					ret = 0;
				break;
			}
		}
	}
	pr_debug("step_motor_start.................over!\n");
	return ret;
}
#endif



static unsigned char bstep_motor_stoped(struct motor_data *pmotor_data)
{
	int status = steppermotor_status(pmotor_data->motor_dev.psteppermotor);
	if (status) {
		return 0;
	}
	pr_debug("bstep_motor_stoped:status=%x\n", status);
	if (steppermotor_is_running(status))
		return 0;
	return 1;
}

static int step_motor_wait_stop(struct motor_data *pmotor_data)
{
	unsigned int timeout_ret;
	int ret=0;

	#ifdef MECH_OPTIMIZE_20170515
	pr_debug("step_motor_wait_stop pmotor_data=%x moving_status=%x stoping_status=%lx\n", (int)pmotor_data, pmotor_data->moving_status, pmotor_data->stoping_status); 
    
	if (pmotor_data->moving_status == MOTOR_MOVE_STATUS_RUNNING) 
	#else
	pr_debug("step_motor_wait_stop pmotor_data=%x moving_status=%lx\n", (int)pmotor_data, pmotor_data->moving_status); 
    
	if(!(pmotor_data->moving_status & MOTOR_STOP_STATUS))
	#endif
	{
		pmotor_data->motor_comp_accout++;
		pr_debug("step_motor_wait_stop 1 &motor_comp_accout=%d \n",pmotor_data->motor_comp_accout); 
		//if(!(timeout_ret=wait_for_completion_timeout(&(pmotor_data->motor_completion), COMPLETION_TIMEOUT)))
		if(!(timeout_ret=wait_for_completion_timeout(&(pmotor_data->motor_completion), COMPLETION_TIMEOUT(pmotor_data->step_config.steps_to_run))))
		{
			printk("step_motor_wait_stop timeout!\n");
			
			ret = -MECH_ERR_MOTOR_WAIT_STOP_TIMEOUT;
			motormove_err_callback(pmotor_data, ret);
			step_motor_stop(pmotor_data);
			
		}
		pmotor_data->motor_comp_accout--;

		pr_debug("step_motor_wait_stop1\n");
		if(pmotor_data->motor_comp_accout == 0)
			init_completion(&(pmotor_data->motor_completion));
		pr_debug("step_motor_wait_stop2\n");
	}
	pmotor_data->moving_status &= ~MOTOR_MOVE_STATUS_RUNNING;
#ifdef MECH_OPTIMIZE_20170515
	pmotor_data->moving_status = MOTOR_MOVE_STATUS_STOP;
#endif
	return ret;
}
//-------------------------------------------
static int brushdc_motor_start(mechanism_uint_motor_data_t *punit_motor_data, mechanism_uint_sensor_data_t *punit_sensor_data,
		struct motor_data *pmotor_data, unsigned char dir,  motor_mov_t  * pmotor_mov)
{
	unsigned char i;
	int ret=0;

	pr_debug("brushdc_motor_start.................speed_phase_num=%d\n", pmotor_mov->speed_phase_num);

	pmotor_data->pmotor_mov = pmotor_mov;

	pmotor_data->dc_config.dir = dir;

	pr_debug("brushdc_motor_start:dir=%d \n", pmotor_data->dc_config.dir);
	pr_debug("pdcmotor=%x\n", (int)pmotor_data->motor_dev.pdcmotor);
	ret=dcmotor_set_config(pmotor_data->motor_dev.pdcmotor, &(pmotor_data->dc_config));
	if(ret)
	{
		printk(KERN_INFO "brushdc_motor_start:dcmotor_set_config error!\n");
		
		ret = -MECH_ERR_MOTOR_MOVE_SET_CONFIG_ERR;
		motormove_err_callback(pmotor_data, ret);
		return ret;
	}

	pmotor_data->phase_current_num = 0;
	pmotor_data->moving_status = MOTOR_MOVE_STATUS_INUSE; 
	pmotor_data->last_sen_mask = 0;
#ifdef MECH_OPTIMIZE_20170606
	pmotor_data->stoping_status = 0;
	pmotor_data->err_status = 0;
	#endif
    
	pr_debug("brushdc_motor_start:triger_num=%d", pmotor_data->pmotor_mov->trigger_phase_num);
   
	for (i = 0; i < pmotor_data->pmotor_mov->trigger_phase_num; i++) {
		if(pmotor_data->pmotor_mov->motor_trigger_phase[i].sen_mask){
			ret=sensor_set_trigger_next(punit_sensor_data, pmotor_data->pmotor_mov->motor_trigger_phase[i].sen_mask, 
                                               pmotor_data->pmotor_mov->motor_trigger_phase[i].motor_sen_flag);
			if(ret)
                        {
                                 printk(KERN_INFO "brushdc_motor_start:sensor_set_trigger_next error!\n");
				 ret = -MECH_ERR_MOTOR_MOVE_SET_TRIGGER_NEXT;
				 motormove_err_callback(pmotor_data, ret);
				 return ret;
                        }
			pmotor_data->last_sen_mask = pmotor_data->pmotor_mov->motor_trigger_phase[i].sen_mask;
	      }
	}

	ret=dcmotor_start(pmotor_data->motor_dev.pdcmotor);
	if(ret)
	{
		printk(KERN_INFO "brushdc_motor_start:dcmotor_start error!\n");
		ret = -MECH_ERR_MOTOR_MOVE_START_ERR;
		motormove_err_callback(pmotor_data, ret);
		return ret;
	}
	pmotor_data->phase_current_num++;
	pmotor_data->moving_status = MOTOR_MOVE_STATUS_RUNNING; 
	printk("brushdc_motor_start.................over!\n");
	return ret;
}

void brushdc_motor_stop(struct motor_data *pmotor_data )
{
	pr_debug("brushdc_motor_stop\n");
	dcmotor_stop(pmotor_data->motor_dev.pdcmotor); 
}

static unsigned char bbrushdc_motor_stoped(struct motor_data *pmotor_data)
{
	int status = dcmotor_status(pmotor_data->motor_dev.pdcmotor);

	pr_debug("bbrushdc_motor_stoped\n");
	if(dcmotor_is_running(status))
		return 0;
	return 1;
}

static int brushdc_motor_wait_stop(struct motor_data *pmotor_data)
{
	unsigned int timeout_ret;
	unsigned long time;
	int ret=0;

	#ifdef MECH_OPTIMIZE_20170515
	if (pmotor_data->moving_status& MOTOR_STOP_STATUS)
	#else
	if (pmotor_data->moving_status!= MOTOR_MOVE_STATUS_RUNNING)
	#endif
		return ret;

	pr_debug("brushdc_motor_wait_stop:motor_comp_accout1=%d", pmotor_data->motor_comp_accout);
	pmotor_data->motor_comp_accout++;
	pr_debug("brushdc_motor_wait_stop:motor_comp_accout2=%d", pmotor_data->motor_comp_accout);

	pr_debug("brushdc_motor_wait_stop %x %x %s\n", (int)pmotor_data, pmotor_data->motor_comp_accout, pmotor_data->motor_name); 
	pr_debug("motor_completion =%x motor_comp_accout=%d\n",(int)&(pmotor_data->motor_completion), pmotor_data->motor_comp_accout);

	pr_debug("brushdc_motor_wait_stop & moving_status=%x\n", pmotor_data->moving_status);

	if (pmotor_data->pmotor_mov->motor_trigger_phase[0].sen_mask)
		time = pmotor_data->pmotor_mov->motor_trigger_phase[0].to_trigger_steps*3/2;
	else
		time = pmotor_data->pmotor_mov->motor_trigger_phase[0].to_trigger_steps;


	#ifdef MECH_OPTIMIZE_20170515
	if ((pmotor_data->moving_status==MOTOR_MOVE_STATUS_RUNNING)&&(!(timeout_ret = wait_for_completion_timeout(&(pmotor_data->motor_completion), 
		time))))
	#else
	if ((!(pmotor_data->moving_status& MOTOR_STOP_STATUS))&&(!(timeout_ret = wait_for_completion_timeout(&(pmotor_data->motor_completion), 
		time))))
	#endif
	{
		printk(KERN_INFO "brushdc_motor_wait_stop timeout!\n");
		if (pmotor_data->pmotor_mov->motor_trigger_phase[0].sen_mask) {	//for (sen_mask==0)，need to stop motor by software
			ret = -MECH_ERR_MOTOR_WAIT_STOP_TIMEOUT;
			motormove_err_callback(pmotor_data, ret);
			
		}
		brushdc_motor_stop(pmotor_data); 	//because stop do not cause interrupt, so brushdc_motor_stop must before callback.
		//printk("brushdc_motor_wait_stop1& moving_status=%x\n", pmotor_data->moving_status);
		
		if (!pmotor_data->pmotor_mov->motor_trigger_phase[0].sen_mask) 	//for (sen_mask==0)，need to stop motor by software
			pmotor_data->callback(pmotor_data->motor_dev.pdcmotor, &(pmotor_data->motor_dev.pdcmotor->callbackdata)); 
		//else
		//	complete_all(&(pmotor_data->motor_completion)); 
		
		//printk("brushdc_motor_wait_stop2& moving_status=%x\n", pmotor_data->moving_status);
	}
	else
	{
		if (pmotor_data->stoping_status==MOTOR_STOP_BY_ABNORMAL) {
			ret = pmotor_data->err_status;
		}
	}
	pmotor_data->motor_comp_accout--;

	pr_debug("brushdc_motor_wait_stop3& moving_status=%x\n", pmotor_data->moving_status);


	if(pmotor_data->motor_comp_accout == 0)
		init_completion(&(pmotor_data->motor_completion));

	#ifdef MECH_OPTIMIZE_20170606
	pmotor_data->moving_status = MOTOR_MOVE_STATUS_STOP;
	#else
	pmotor_data->moving_status &= ~MOTOR_MOVE_STATUS_RUNNING;
	#ifdef MECH_OPTIMIZE_20170515
	pmotor_data->moving_status |= MOTOR_MOVE_STATUS_STOP;
	#endif
	#endif
	pr_debug("brushdc_motor_wait_stop4\n");
	return ret;
}

/*----------------------------------------------------------------------- 
 
-----------------------------------------------------------------------*/
int motor_move_init(mechanism_uint_motor_data_t *punit_motor_data, unsigned short motor_mask)
{
	struct motor_data *pmotor_data;
	unsigned char i;

	//pr_debug("motor_init.............motor_mask=%x\n", motor_mask);

	motor_get_data(punit_motor_data, motor_mask, pmotor_data, i);
	if (i==punit_motor_data->motor_num) {
		return -MECH_ERR_MOTOR_GETDATA;
	}
	//pr_debug("motor_init:pmotor_data=%x %x \n", pmotor_data, &((punit_data)->motor[i]));	
	//pr_debug("motor_completion=%x motor_mask=%x motor_name=%s\n", &pmotor_data->motor_completion, pmotor_data->motor_mask, pmotor_data->motor_name); 
	//pr_debug("motor_num=%d i=%d motor_completion=%x motor_mask=%x motor_name=%s\n", (punit_data)->motor_num, i, &(punit_data)->motor[i].motor_completion, (punit_data)->motor[i].motor_mask, (punit_data)->motor[i].motor_name); 

	if (pmotor_data->moving_status&(MOTOR_MOVE_STATUS_INUSE|MOTOR_MOVE_STATUS_RUNNING)) {
		return -MECH_ERR_MOTOR_BUSY;
	}

	init_completion(&(pmotor_data->motor_completion));
	pmotor_data->motor_comp_accout = 0;

	init_completion(&(pmotor_data->motor_phase_completion));
	pmotor_data->motor_phase_accout = 0; 

	pmotor_data->moving_status = MOTOR_MOVE_STATUS_INIT;
#ifdef MECH_OPTIMIZE_20170515
	pmotor_data->stoping_status = 0;
#endif
	return 0;
}
EXPORT_SYMBOL_GPL(motor_move_init);
/*
   

	
*/
int motor_start(mechanism_uint_motor_data_t *punit_motor_data, mechanism_uint_sensor_data_t *punit_sensor_data,
	unsigned short motor_mask, unsigned char dir, motor_mov_t  *pmotor_mov)
{
	struct motor_data *pmotor_data;
	unsigned char i;

	pr_debug("motor_start.................\n");
    
	motor_get_data(punit_motor_data, motor_mask, pmotor_data, i);
	if (i==punit_motor_data->motor_num) {
		return -MECH_ERR_MOTOR_GETDATA;
	}
    
    	
	switch(pmotor_data->motor_type)
	{
	case MOTOR_STEP_TYPE:		
		return step_motor_start(punit_motor_data, punit_sensor_data, pmotor_data, dir, pmotor_mov);
		break;
	case MOTOR_BRUSHDC_TYPE:
		return brushdc_motor_start(punit_motor_data, punit_sensor_data, pmotor_data, dir, pmotor_mov);
		break;
	default:
		break;
	}
	pr_debug(KERN_INFO "motor_start.................over!\n");
	return 0;
}
EXPORT_SYMBOL_GPL(motor_start);
//----------------------stop motor moving and wait until stopped----------------------
int motor_stop(mechanism_uint_motor_data_t *punit_motor_data, unsigned short motor_mask)
{
	struct motor_data *pmotor_data;
	unsigned char i;
	int ret=0;

	pr_debug("1.motor_stop\n");
	motor_get_data(punit_motor_data, motor_mask, pmotor_data, i);
	if (i==punit_motor_data->motor_num) {
		return -MECH_ERR_MOTOR_GETDATA;
	}
    	
	pr_debug("2.motor_stop\n");
	switch(pmotor_data->motor_type)
	{
	case MOTOR_STEP_TYPE:
		if(bstep_motor_stoped(pmotor_data))
			break;
		pmotor_data->moving_status |= MOTOR_STOP_BY_SOFT_START; 
		pr_debug("motor_stop pmotor_data=%x moving_status=%x\n", (unsigned int)pmotor_data, pmotor_data->moving_status);
		step_motor_stop(pmotor_data);
		//step_motor_wait_stop(pmotor_data);
		break;
	case MOTOR_BRUSHDC_TYPE:
		if(bbrushdc_motor_stoped(pmotor_data))
			break;
		pmotor_data->moving_status |= MOTOR_STOP_BY_SOFT_START;
		brushdc_motor_stop(pmotor_data);
		//brushdc_motor_wait_stop(pmotor_data);
		break;
	default:
		break;
	}
	return ret;
}
EXPORT_SYMBOL_GPL(motor_stop);

int bmotor_stoped(mechanism_uint_motor_data_t *punit_motor_data, unsigned short motor_mask)
{
	struct motor_data *pmotor_data;
	unsigned char i;

	motor_get_data(punit_motor_data, motor_mask, pmotor_data, i);
	if (i==punit_motor_data->motor_num) {
		return -MECH_ERR_MOTOR_GETDATA;
	}

	switch(pmotor_data->motor_type)
	{
	case MOTOR_STEP_TYPE:
		return bstep_motor_stoped(pmotor_data);
	case MOTOR_BRUSHDC_TYPE:
		return bbrushdc_motor_stoped(pmotor_data);
	default:
		break;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(bmotor_stoped);

int motor_wait_stop(mechanism_uint_motor_data_t *punit_motor_data, mechanism_uint_sensor_data_t *punit_sensor_data, unsigned short motor_mask)
{
	struct motor_data *pmotor_data;
	unsigned char i;
	int ret=0;

	pr_debug("motor_wait_stop\n");

	motor_get_data(punit_motor_data, motor_mask, pmotor_data, i);
	if (i==punit_motor_data->motor_num) {
		return -MECH_ERR_MOTOR_GETDATA;
	}
    
	pr_debug("motor_type=%x \n", pmotor_data->motor_type);

	
	switch(pmotor_data->motor_type)
	{
	case MOTOR_STEP_TYPE:
		ret = step_motor_wait_stop(pmotor_data);
		if(pmotor_data->last_sen_mask)
			sensor_clear_trigger_next(punit_sensor_data, pmotor_data->last_sen_mask);
		break;
	case MOTOR_BRUSHDC_TYPE:
		ret = brushdc_motor_wait_stop(pmotor_data);
		if(pmotor_data->last_sen_mask)
			sensor_clear_trigger_next(punit_sensor_data, pmotor_data->last_sen_mask);
		break;
	default:
		break;
	}
	
	pr_debug("motor_wait_stop over!\n");
	return ret;
}
EXPORT_SYMBOL_GPL(motor_wait_stop);

//----------------------get feature of motor----------------------
int motor_get_feature(mechanism_uint_motor_data_t *punit_motor_data, motor_feature_t *p_motorfeature)
{
	struct motor_data *pmotor_data;
	unsigned char i;
	int ret=0;

	pr_debug("motor_get_feature\n");

	motor_get_data(punit_motor_data, p_motorfeature->motor_mask, pmotor_data, i); 
	if (i==punit_motor_data->motor_num) {
		return -MECH_ERR_MOTOR_GETDATA;
	}
    
	pr_debug("motor_type=%x \n", pmotor_data->motor_type);
	strcpy(p_motorfeature->motor_name, pmotor_data->motor_name); 

	//for temp. should get from steppermotor_get_feature endly
	if (pmotor_data->motor_type==MOTOR_STEP_TYPE) {
		struct steppermotor_feature *feature;
		#if 1
		feature = steppermotor_get_feature(pmotor_data->motor_dev.psteppermotor);
		if (!(IS_ERR(feature)))
		{
			p_motorfeature->step_max = feature->max_steps;
			p_motorfeature->pullin_speed = feature->pullin_speed;
			p_motorfeature->num_speed = feature->num_speed;
			for (i=0; i<p_motorfeature->num_speed; i++) {
				p_motorfeature->speeds[i].accel_steps = feature->speeds[i].accel_steps;
				p_motorfeature->speeds[i].decel_steps = feature->speeds[i].decel_steps;
				p_motorfeature->speeds[i].speed = feature->speeds[i].speed; 
			}
			p_motorfeature->num_speedshift = feature->num_speedshift; 
			for (i=0; i<p_motorfeature->num_speedshift; i++) {
				p_motorfeature->speedshifts[i].speed1 = feature->speedshifts[i].speed1;
				p_motorfeature->speedshifts[i].speed2 = feature->speedshifts[i].speed2;
				p_motorfeature->speedshifts[i].steps = feature->speedshifts[i].steps; 
			}
		}
		#else
		p_motorfeature->step_max = 0x0FFF;
		#endif
	}
	else
		p_motorfeature->step_max = -1;

	return ret;
}
EXPORT_SYMBOL_GPL(motor_get_feature);

//----------------------motor_lock----------------------
int motor_lock(mechanism_uint_motor_data_t *punit_motor_data, unsigned short motor_mask)
{
	struct motor_data *pmotor_data;
	unsigned char i;
	int ret=0;

	pr_debug("1.motor_lock\n");
	motor_get_data(punit_motor_data, motor_mask, pmotor_data, i);
	if (i==punit_motor_data->motor_num) {
		return -MECH_ERR_MOTOR_GETDATA;
	}
    	
	pr_debug("2.motor_lock\n");
	switch(pmotor_data->motor_type)
	{
	case MOTOR_STEP_TYPE:
		steppermotor_lock(pmotor_data->motor_dev.psteppermotor); 
		break;
	case MOTOR_BRUSHDC_TYPE:
		break;
	default:
		break;
	}
	return ret;
}
EXPORT_SYMBOL_GPL(motor_lock);

//----------------------motor_unlock----------------------
int motor_unlock(mechanism_uint_motor_data_t *punit_motor_data, unsigned short motor_mask)
{
	struct motor_data *pmotor_data;
	unsigned char i;
	int ret=0;

	pr_debug("1.motor_unlock\n");
	motor_get_data(punit_motor_data, motor_mask, pmotor_data, i);
	if (i==punit_motor_data->motor_num) {
		return -MECH_ERR_MOTOR_GETDATA;
	}
    	
	pr_debug("2.motor_unlock\n");
	switch(pmotor_data->motor_type)
	{
	case MOTOR_STEP_TYPE:
		steppermotor_unlock(pmotor_data->motor_dev.psteppermotor); 
		break;
	case MOTOR_BRUSHDC_TYPE:
		break;
	default:
		break;
	}
	return ret;
}
EXPORT_SYMBOL_GPL(motor_unlock);

//----------------------motor_skewsteps_set----------------------
int motor_skewsteps_set(mechanism_uint_motor_data_t *punit_motor_data, unsigned short motor_mask, int skew_steps)
{
	struct motor_data *pmotor_data;
	unsigned char i;
	int ret=0;

	pr_debug("1.motor_skewsteps_set\n");
	motor_get_data(punit_motor_data, motor_mask, pmotor_data, i);
	if (i==punit_motor_data->motor_num) {
		return -MECH_ERR_MOTOR_GETDATA;
	}
    	
	pr_debug("2.motor_skewsteps_set\n");
	switch(pmotor_data->motor_type)
	{
	case MOTOR_STEP_TYPE:
		steppermotor_set_skew_steps(pmotor_data->motor_dev.psteppermotor, skew_steps); 
		break;
	case MOTOR_BRUSHDC_TYPE:
		break;
	default:
		break;
	}
	return ret;
}
EXPORT_SYMBOL_GPL(motor_skewsteps_set);

