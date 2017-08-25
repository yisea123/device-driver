#ifndef __MECH_UNIT_H__
#define __MECH_UNIT_H__

#include <linux/device.h>
#include <linux/completion.h>
#include <linux/ioctl.h>
#include <linux/cdev.h>
#include <linux/string.h>

#include "../motor/steppermotor.h"
#include "../motor/dcmotor.h"
#include "../photosensor/photosensor.h"
//#include "../fpga_io.h"

#include "mech_motor.h"
#include "mech_sensor.h"
#include "mechlib.h"

//--------------------------------------------------------
struct mechanism_uint_data{
	char mech_unit_name[MECHUINT_NAME_LEN];
	char mech_unit_type;
	mechanism_uint_motor_data_t	unit_motor_data;
	char bmotor_filled;
	mechanism_uint_sensor_data_t 	unit_sensor_data;
	char bsensor_filled;
};

extern int mechunit_get_sensor_status(struct mechanism_uint_data *punit_data,unsigned short sensor_masks, unsigned short *pstatus);
extern int mechunit_get_sensors_rawinput(struct mechanism_uint_data *punit_data, mech_unit_sen_raw_input_t * psen_raw_input);
extern int mechunit_set_sensor_config(struct mechanism_uint_data *punit_data, mech_unit_sen_config_t *pmech_unit_sen_config);
extern int mechunit_get_sensor_feature(struct mechanism_uint_data *punit_data, mech_unit_sen_feature_t *pmech_unit_sen_feature);
extern int mechunit_get_motor_feature(struct mechanism_uint_data *punit_data, mech_unit_motor_feature_t *pmech_unit_motor_feature);
/*-----------------------------------------------------------------------
	struct mechanism_dev_t
 -----------------------------------------------------------------------*/


struct mechanism_dev_t {
//	spinlock_t		lock;		/* lock this structure */
	struct cdev		cdev;
//	struct device		*pdev;
	struct fasync_struct *async_queue;
	unsigned long 	sigio_event;
	int driver_status;
	int mech_status;
	struct mechanism_uint_data mech_unit_data;
	mech_unit_control_t mech_unit_control;
	struct device *dev;
	int dev_major;
	dev_t dev_no;
	struct device *mech_dev;
	struct class *mech_class;
	#ifdef MECH_OPTIMIZE_20170502
	mechunit_drv_status_t mech_unit_drv_status;
	#endif
};

extern int mechunit_probe_get_devtree_pdata(struct device*dev, struct mechanism_uint_data *pmech_unit_data);
extern void mechunit_sigio(struct mechanism_dev_t *pmechanism_dev);
#ifdef MECH_OPTIMIZE_20170502
extern int mechunit_probe(struct platform_device *pdev);
extern int mechunit_remove(struct platform_device *pdev);
#endif

extern int mechunit_open(struct inode *inode, struct file *filep);
extern int mechunit_close(struct inode *inode, struct file *filep);
extern int mechunit_async(int fd, struct file *filep, int mode);
#ifdef MECH_OPTIMIZE_20170428 
extern int mechunit_motor_init(mechanism_uint_motor_data_t *punit_motor_data, unsigned short motor_mask, void (*callback)(void *motor, struct callback_data *),struct mechanism_dev_t *mech_dev);
#endif

#define	ARGPTR(cptr)			((cptr)->argptr)

#define	MECHCTRL_DIR(cptr)		(((mech_control_t *)ARGPTR(cptr))->dir)
#define	MECHCTRL_MODE(cptr)	    	(((mech_control_t *)ARGPTR(cptr))->mode)
#define MECHCTRL_SPEED(cptr)       	(((mech_control_t *)ARGPTR(cptr))->time)
#define	MECHCTRL_STEPS(cptr)	    	(((mech_control_t *)ARGPTR(cptr))->steps)
#define	MECHCTRL_BUFFER(cptr)	    	(((mech_control_t *)ARGPTR(cptr))->buffer)

//--------------------------------------------------------
typedef unsigned char byte;
typedef unsigned short word;
typedef long address;

typedef	struct {		/* command class item		*/
    address		argptr;		/* argument pointer*/	
	//byte		*statusptr;	/* status byte pointer*/
} class_cmd;
 	
typedef	int	(*seqptr_t)(struct mechanism_dev_t *, class_cmd *);

//-----mech_status--------------------------------------
#define	IDLE			0	/* devices states	*/ 
#define	BUSY			1
#define	RECOVERABLE		2
#define	UNRECOVERABLE		4
#define DEVICE_IN_ERROR	(RECOVERABLE | UNRECOVERABLE)

//------mechanism driver states-------------------------
#define	MECH_DRIVER_IDLE			(word)0		/*HL:驱动空闲*/
#define	MECH_DRIVER_OPENED			(word)1		/*HL:驱动打开*/
#define	MECH_DRIVER_WORKING			(word)2		/*HL:驱动工作*/
#define	MECH_DRIVER_DEFERRED 			(word)3		/*HL:驱动延迟*/
#define	MECH_DRIVER_CLOSING			(word)4		/*HL:驱动正在关闭*/
#define	MECH_DRIVER_TERMINATED			(word)5		/*HL:驱动结束*/

#define	MECH_DRIVER_WORKING_ASYNC		(word)6		/*HL:驱动异步方式工作*/

//-------------------------------
static inline void stepmotor_callback(struct motor_data *pmotor_data, struct mechanism_dev_t *pmechanism_dev, int status, unsigned long stop_flag)
{
	int ret;

	if (steppermotor_is_triggersteps_done(status))  {
		pr_debug("steppermotor_is_triggersteps_done锛宮otor_phase_accout=%d\n", pmotor_data->motor_phase_accout);

		if (pmotor_data->motor_phase_accout != 0){
			#ifdef MECH_OPTIMIZE_20170606
			pmotor_data->phase_current_num++;
			if(pmotor_data->phase_current_num < pmotor_data->pmotor_mov->trigger_phase_num)
			{
				ret = step_motor_triger_deal(pmotor_data, pmotor_data->phase_current_num, &(pmechanism_dev->mech_unit_data.unit_motor_data), &(pmechanism_dev->mech_unit_data.unit_sensor_data)); 
				if (ret) {
					pmotor_data->stoping_status = MOTOR_STOP_BY_ABNORMAL;
					pmotor_data->moving_status = MOTOR_MOVE_STATUS_STOP;
					pmotor_data->err_status = ret;
				}
			}
			#endif
			complete(&pmotor_data->motor_phase_completion);
			return;
		}
		else	/*if triger int appeared and motor_phase_account==0, maybe next triger params(sensor & motor) not filled 
			or wait_for_completion_timeout not executed yet.Wait these operations completed and next triger int appeared. 2017.3.29*/
			return;
	}
     
   	if(steppermotor_is_stopped_by_total_steps(status) || steppermotor_is_stopped_by_trigger(status) ||
	   steppermotor_is_stopped_by_sensor(status)|| steppermotor_is_fault(status) || steppermotor_is_stopped_by_skew(status)) {
		if(steppermotor_is_stopped_by_total_steps(status)){
			pr_debug("steppermotor_is_totalsteps_done\n");
			if (pmotor_data->moving_status & MOTOR_STOP_BY_SOFT_START){
				pmotor_data->stoping_status = MOTOR_STOP_BY_SOFT;
			} else
				pmotor_data->stoping_status = MOTOR_STOP_BY_TOTAL;

			pmotor_data->moving_status = MOTOR_MOVE_STATUS_STOP;
			pmechanism_dev->sigio_event = stop_flag|pmotor_data->stoping_status;
			
		}
		else if (steppermotor_is_stopped_by_sensor(status)||steppermotor_is_stopped_by_skew(status)) {
			if (steppermotor_is_stopped_by_sensor(status))
			{
				pr_debug("steppermotor_is_stopped_by_sensor ");
				pmotor_data->stoping_status = MOTOR_STOP_BY_SENSOR;
			}
			else{
				pr_debug("steppermotor_is_stopped_by_skew ");
				pmotor_data->stoping_status = MOTOR_STOP_BY_SKEW;
			}
			pmotor_data->moving_status = MOTOR_MOVE_STATUS_STOP;
			
			#if 0
			pr_debug("phase_current_num=%d sen_mask=%x motor_sen_flag=%x\n",pmotor_data->phase_current_num,
				pmotor_data->pmotor_mov->motor_trigger_phase[pmotor_data->phase_current_num-1].sen_mask,
				pmotor_data->pmotor_mov->motor_trigger_phase[pmotor_data->phase_current_num-1].motor_sen_flag);

			pmotor_data->moving_status = MOTOR_MOVE_STATUS_STOP; 
			pmechanism_dev->sigio_event = stop_flag|pmotor_data->stoping_status | 
				(pmotor_data->pmotor_mov->motor_trigger_phase[pmotor_data->phase_current_num-1].sen_mask<<MOTOR_STOP_SEN_POS_SHIFT) |
				(pmotor_data->pmotor_mov->motor_trigger_phase[pmotor_data->phase_current_num-1].motor_sen_flag<<MOTOR_STOP_SEN_FLAG_SHIFT);
			#else
			pr_debug("phase_current_num=%d stop_flag=%x sen_mask=%x motor_sen_flag=%x\n",pmotor_data->phase_current_num,stop_flag,
				pmotor_data->pmotor_mov->motor_trigger_phase[pmotor_data->phase_current_num-1].sen_mask,
				pmotor_data->pmotor_mov->motor_trigger_phase[pmotor_data->phase_current_num-1].motor_sen_flag);

			pmotor_data->moving_status = MOTOR_MOVE_STATUS_STOP; 
			pmechanism_dev->sigio_event = stop_flag|pmotor_data->stoping_status | 
				#ifdef MECH_SENSOR_INDEX_20170815
				MOTOR_STOP_SEN_POS_TO_RES(pmotor_data->pmotor_mov->motor_trigger_phase[pmotor_data->phase_current_num-1].sen_pos_index) |
				#else
				(pmotor_data->pmotor_mov->motor_trigger_phase[pmotor_data->phase_current_num-1].sen_mask<<MOTOR_STOP_SEN_POS_SHIFT) |
				#endif
				MOTOR_STOP_SEN_FLAG_TO_RES(pmotor_data->pmotor_mov->motor_trigger_phase[pmotor_data->phase_current_num-1].motor_sen_flag);
			#endif
			
		}
		else if (steppermotor_is_stopped_by_trigger(status)) {
			pr_debug("steppermotor_is_stopped_by_trigger\n");
			pmotor_data->stoping_status = MOTOR_STOP_BY_TRIGER;
			pmotor_data->moving_status = MOTOR_MOVE_STATUS_STOP;
			pmechanism_dev->sigio_event = stop_flag|pmotor_data->stoping_status;
		}

		if (steppermotor_is_fault(status)) {
		    pr_debug("steppermotor_is_error");
		    #ifdef MECH_OPTIMIZE_20170606
		    pmotor_data->stoping_status = MOTOR_STOP_BY_ABNORMAL;
		    pmotor_data->err_status = -RESN_MECH_ERR_MOTOR_HW_ERR;
		    #else
		    pmotor_data->stoping_status |= MOTOR_STOP_BY_ABNORMAL |MOTOR_HW_ERR;
		    #endif
		    pmotor_data->moving_status = MOTOR_MOVE_STATUS_STOP;
		}

		if((pmotor_data->stoping_status & MOTOR_STOP_MASK)!=MOTOR_STOP_BY_ABNORMAL)
			mechunit_sigio(pmechanism_dev);
		
		if (pmotor_data->motor_phase_accout != 0){
			//pr_debug("complete motor_phase_completion=%x\n", &pmotor_data->motor_phase_completion);
			complete(&pmotor_data->motor_phase_completion);
		}
		if (pmotor_data->motor_comp_accout != 0){
			//pr_debug("complete motor_completion=%x\n", &pmotor_data->motor_completion);
			complete_all(&(pmotor_data->motor_completion)); 
		}
	}
   
	else{
		printk("MOTOR_INT_INVALID & status=%x motor_phase_accout=%d\n", status, pmotor_data->motor_phase_accout);
		{
			#if 0
			void __iomem *fpga_regs = NULL;
			void __iomem *fpga_reg_tmp;
			u32 value;

			
			fpga_regs = fpga_io_get(3); 

			fpga_reg_tmp = fpga_regs+ 0x0000;
			fpga_readl(&value,fpga_reg_tmp);
			printk("Motor_Control_1(30000) = %d\n",value); 

			fpga_reg_tmp = fpga_regs+ 0x0008;
			fpga_readl(&value,fpga_reg_tmp);
			printk("motor_running_step_1(30008) = %d\n",value); 

			fpga_reg_tmp = fpga_regs+ 0x001c;
			fpga_readl(&value,fpga_reg_tmp);
			printk("trigger_steps_next(3001c) = %d\n",value); 
			#endif
		}
		#ifdef MECH_OPTIMIZE_20170606
		pmotor_data->stoping_status = MOTOR_STOP_BY_ABNORMAL;
		pmotor_data->err_status = -RESN_MECH_ERR_MOTOR_INT_INVALID;
		pmotor_data->moving_status = MOTOR_MOVE_STATUS_STOP;
		#else
		pmotor_data->stoping_status |= MOTOR_STOP_BY_ABNORMAL|MOTOR_INT_INVALID;
		pmotor_data->moving_status = MOTOR_MOVE_STATUS_STOP;
		pmechanism_dev->sigio_event = MOTOR_INT_INVALID|stop_flag | pmotor_data->stoping_status | ((status&0xff) << MOTOR_STOP_SEN_POS_SHIFT);
		#endif

		if (pmotor_data->motor_phase_accout != 0)
			complete(&pmotor_data->motor_phase_completion);
		if (pmotor_data->motor_comp_accout != 0)
			complete_all(&(pmotor_data->motor_completion));
	}
}


#endif
