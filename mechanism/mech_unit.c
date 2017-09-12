#include <linux/device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/sysfs.h>

#include <asm/uaccess.h>
#include <asm/signal.h>
#include <asm/siginfo.h>

#include <asm/uaccess.h>

#include <linux/platform_device.h>
#include <linux/completion.h>
#include <linux/slab.h>

#include "mech_unit.h"


int mechunit_get_sensor_status(struct mechanism_uint_data *punit_data,unsigned short sensor_masks, unsigned short *pstatus)
{
	unsigned char j;
	unsigned short mask, status=0;
	unsigned int val;
	int ret=0;
	
	//*pstatus = 0;
	
	//pr_debug("mechunit_get_sensor_status:sensor_masks=%x sensor_num=%d\n", sensor_masks, punit_data->unit_sensor_data.sensor_num);
	#if 0
	for (i = 0; i < punit_data->unit_sensor_data.sensor_num; i++) {
		mask = sensor_masks & (1 << i);
		if (!mask) {
			continue;
		}
		pr_debug("mechunit_get_sensor_status：mask=%x\n", mask);
		for (j = 0; j < punit_data->unit_sensor_data.sensor_num; j++) {
			if(mask == punit_data->unit_sensor_data.sensor[j].sen_mask){
				ret = sensor_get_appval(&(punit_data->unit_sensor_data), mask, &val);
				if (ret < 0) {
					return ret;
				}
				//*pstatus |= (val<<i);
				status |= (val<<i);
				break;
			}
		}
	}
	#else
	//for (i = 0; i < punit_data->unit_sensor_data.sensor_num; i++) 
	{
		//mask = sensor_masks & (1 << i);
		//if (!mask) {
			//continue;
		//}
		//pr_debug("mechunit_get_sensor_status：mask=%x\n", mask);
		for (j = 0; j < punit_data->unit_sensor_data.sensor_num; j++) {
			//pr_debug("%d:sen_mask=%x %x\n", j, punit_data->unit_sensor_data.sensor[j].sen_mask, sensor_masks & punit_data->unit_sensor_data.sensor[j].sen_mask);
			if(sensor_masks & punit_data->unit_sensor_data.sensor[j].sen_mask){
				mask = punit_data->unit_sensor_data.sensor[j].sen_mask;
				ret = sensor_get_appval(&(punit_data->unit_sensor_data), mask, &val);
				if (ret < 0) {
					return ret;
				}
				//*pstatus |= (val<<i);
				//status |= (val<<j);
				if (val) {
					status |= punit_data->unit_sensor_data.sensor[j].sen_mask;
				}
				//break;
			}
		}
	}
	#endif
	*pstatus = status;
	//pr_debug("mechunit_get_sensor_status：=%x\n", *pstatus); 
	return ret;
}
EXPORT_SYMBOL_GPL(mechunit_get_sensor_status);
	
int mechunit_get_sensors_rawinput(struct mechanism_uint_data *punit_data, mech_unit_sen_raw_input_t * psen_raw_input)
{
	int ret=0, i, j;

	psen_raw_input->sen_num = punit_data->unit_sensor_data.sensor_num;
	for (i=0; i < psen_raw_input->sen_num; i++) {
		//psen_raw_input->sen_raw_input[i].sen_mask = 0x01<<i;
		psen_raw_input->sen_raw_input[i].sen_mask = punit_data->unit_sensor_data.sensor[i].sen_mask;
		//pr_debug("mechunit_get_sensors_rawinput %d mask=%x\n", i, psen_raw_input->sen_raw_input[i].sen_mask); 
		psen_raw_input->sen_raw_input[i].raw_input_value[0] = 0;
	}

#ifdef MECH_SENSOR_TEST_20170509
	for (i = 0; i < psen_raw_input->sen_num; i++) {
			ret = sensor_get_val(&(punit_data->unit_sensor_data), 
						psen_raw_input->sen_raw_input[i].sen_mask, 
					     &(psen_raw_input->sen_raw_input[i].raw_input_value[0])); 
			if (ret) 
			{
				pr_debug("mechunit_sen_rawinput :err exit & mask=%x\n", psen_raw_input->sen_raw_input[i].sen_mask);
				goto exit;
			}

			psen_raw_input->sen_raw_input[i].raw_input_value[1] = psen_raw_input->sen_raw_input[i].raw_input_value[0];
			for (j=2; j<=SENSOR_RAW_GET_NUM; j++) {
				 psen_raw_input->sen_raw_input[i].raw_input_value[j] = psen_raw_input->sen_raw_input[i].raw_input_value[j-1];
			}
			psen_raw_input->sen_raw_input[i].raw_input_value[0] *= SENSOR_RAW_GET_NUM;
			usleep()
			
		}
#else
	for (j=1; j<=SENSOR_RAW_GET_NUM; j++) {
		for (i = 0; i < psen_raw_input->sen_num; i++) {
			ret = sensor_get_val(&(punit_data->unit_sensor_data), 
						psen_raw_input->sen_raw_input[i].sen_mask, 
					     &(psen_raw_input->sen_raw_input[i].raw_input_value[j])); 
			psen_raw_input->sen_raw_input[i].raw_input_value[0] += psen_raw_input->sen_raw_input[i].raw_input_value[j];
			if (ret) 
			{
				pr_debug("mechunit_sen_rawinput :err exit & mask=%x\n", psen_raw_input->sen_raw_input[i].sen_mask);
				goto exit;
			}
		}
	}
#endif
	for (i=0; i < psen_raw_input->sen_num; i++) {
		psen_raw_input->sen_raw_input[i].raw_input_value[0] /= 10;
		//pr_debug("mechunit_get_sensors_rawinput sen_mask=%x raw_input_value=%d\n", psen_raw_input->sen_raw_input[i].sen_mask, psen_raw_input->sen_raw_input[i].raw_input_value[0]); 
	}
	
exit:         
	return ret;
}
EXPORT_SYMBOL_GPL(mechunit_get_sensors_rawinput);
	
int mechunit_set_sensor_config(struct mechanism_uint_data *punit_data, mech_unit_sen_config_t *pmech_unit_sen_config)
{
	unsigned char i;
	int ret=0;

	//pr_debug("mechunit_set_sensor_config\n");
	for (i=0; i < pmech_unit_sen_config->sen_num; i++) {
		ret = sensor_set_config(&(punit_data->unit_sensor_data), pmech_unit_sen_config, &pmech_unit_sen_config->sen_config[i]);
		if (ret) {
			return ret;
		}
	}
	return 0;
}
EXPORT_SYMBOL_GPL(mechunit_set_sensor_config);
	
int mechunit_get_sensor_feature(struct mechanism_uint_data *punit_data, mech_unit_sen_feature_t *pmech_unit_sen_feature)
{
	unsigned char i;
	int ret=0;

	//pr_debug("mechunit_get_sensor_feature\n");
	pmech_unit_sen_feature->sen_num = punit_data->unit_sensor_data.sensor_num;
	for (i=0; i < pmech_unit_sen_feature->sen_num; i++) {
		pmech_unit_sen_feature->sen_feature[i].sen_mask = punit_data->unit_sensor_data.sensor[i].sen_mask;
		ret = sensor_get_feature(&(punit_data->unit_sensor_data), &pmech_unit_sen_feature->sen_feature[i]); 
		if (ret) {
			return ret;
		}
	}
	return ret;
}
EXPORT_SYMBOL_GPL(mechunit_get_sensor_feature);

int mechunit_get_motor_feature(struct mechanism_uint_data *punit_data, mech_unit_motor_feature_t *pmech_unit_motor_feature)
{
	unsigned char i;
	int ret=0;

	//pr_debug("mechunit_get_motor_feature\n");
	pmech_unit_motor_feature->motor_num = punit_data->unit_motor_data.motor_num; 
	for (i=0; i < pmech_unit_motor_feature->motor_num; i++) {
		pmech_unit_motor_feature->motor_feature[i].motor_mask = punit_data->unit_motor_data.motor[i].motor_mask; 
		ret = motor_get_feature(&(punit_data->unit_motor_data), &pmech_unit_motor_feature->motor_feature[i]); 
		if (ret) {
			return ret;
		}
	}
	return ret;
}
EXPORT_SYMBOL_GPL(mechunit_get_motor_feature);
//----------------------mechunit_probe_get_sensor----------------------
int mechunit_probe_get_sensor(struct device_node *ppsensor, struct sensor_data *p, unsigned char sensor_num)
{
	struct device_node *pp, *psen_dev;
	//char str[50];
	//char str_array[50];
	//char *pstr=str_array;
	const char *psen_name;
	unsigned int data;
	int ret=0;
	unsigned char i=0;

	pr_debug("mechunit_probe_get_sensor\n");
	if (!sensor_num) {
		return 0;
	}
	for_each_child_of_node(ppsensor, pp) {
		ret = of_property_read_string(pp, "sen_name",&psen_name);
		if (ret){
			pr_debug("mechunit_probe_get_sensor:Failed to read name of sensor!\n");
			return ret;
		}
		strcpy(p->sen_name,psen_name);

		ret = of_property_read_u32(pp,"sen_mask",&data);
		if (ret){
			pr_debug("mechunit_probe_get_sensor:Failed to read mask of sensor!\n");
			return ret;
		}
		p->sen_mask = data;

		ret = of_property_read_u32(pp,"sen_dev",&data);
		if (ret){
			pr_debug("mechunit_probe_get_sensor:Failed to read dev of sensor!\n");
			return ret;
		}

		psen_dev = of_find_node_by_phandle(data);
		if (psen_dev==NULL) {
			pr_debug("mechunit_probe_get_sensor: of_find_node_by_phandle err!\n");
			return -1;
		}

		p->sen_dev.pphotosensor = of_node_to_photosensor(psen_dev);
		if (p->sen_dev.pphotosensor ==ERR_PTR(-EPROBE_DEFER)) {
				pr_debug("mechunit_probe_get_sensor:of_node_to_photosensor err!\n");
				return -1;
		}
		
		p->sen_type = p->sen_dev.pphotosensor->type;
		pr_debug("sen: name=%s mask=%x pphotosensor=%x type=%x\n", p->sen_name, p->sen_mask, (unsigned int)p->sen_dev.pphotosensor, p->sen_type);
		i++;
		if (i==sensor_num) {
			break;
		}
		else
			p += 1;
	}
	return 0;
}

int mechunit_probe_get_motor(struct device_node *ppmotor, struct motor_data *p, unsigned char motor_num)
{
	struct device_node *pp, *pmotor_dev;
	//char str_array[50];
	//char *pstr=str_array;
	//const char *pstr;
	const char *pmotor_name, *pmotor_type;
	unsigned int data, i=0;
	int ret=0;
	//struct steppermotor *psteppermotor;//add by hl for debug 2016.10.31

	if (!motor_num) {
		return 0;
	}
	for_each_child_of_node(ppmotor, pp) {
		ret = of_property_read_string(pp, "motor_name", &pmotor_name); 
		if (ret){
			pr_debug("mechunit_probe_get_motor:Failed to read name of motor!\n");
			return ret;
		}
		strcpy(p->motor_name, pmotor_name);

		ret = of_property_read_string(pp, "motor_type", &pmotor_type);
		if (ret){
			pr_debug("mechunit_probe_get_motor:Failed to read type of motor!\n");
			return ret;
		}

		ret = of_property_read_u32(pp,"motor_mask",&data);
		if (ret){
			pr_debug("mechunit_probe_get_motor:Failed to read mask of motor!\n");
			return ret;
		}
		p->motor_mask = data;

		ret = of_property_read_u32(pp,"motor_dev",&data);
		if (ret){
			pr_debug("mechunit_probe_get_motor:Failed to read dev of motor!\n");
			return ret;
		}

		pmotor_dev = of_find_node_by_phandle(data);
		if (pmotor_dev==NULL) {
			pr_debug("mechunit_probe_get_motor:of_find_node_by_phandle err!\n");
			return -1;
		}
		pr_debug("mechanism_get_motor :motor_type=%s\n", pmotor_type);
		if (!strcmp(pmotor_type, "stepper_motor")) {
			p->motor_type = MOTOR_STEP_TYPE;
			p->motor_dev.psteppermotor = of_node_to_steppermotor(pmotor_dev); 
			if (p->motor_dev.psteppermotor==ERR_PTR(-EPROBE_DEFER)) {
				printk("mechunit_probe_get_motor:of_node_to_steppermotor err!\n");
				return -1;
			}
			else 
				pr_debug("mechanism_get_motor=%x motor_type=%x\n", (unsigned int)p->motor_dev.psteppermotor, p->motor_type);
			#if 0
			psteppermotor = of_node_to_steppermotor(pmotor_dev); 
			if (psteppermotor==ERR_PTR(-EPROBE_DEFER)) {
				pr_debug("mechanism_get_motor-------------of_node_to_steppermotor err!\n");
			}
			else 
				pr_debug("mechanism_get_motor=%x\n", psteppermotor);
			#endif
		}
		else if (!strcmp(pmotor_type, "brdc_motor")) {
			p->motor_type = MOTOR_BRUSHDC_TYPE;
			p->motor_dev.pdcmotor = of_node_to_dcmotor(pmotor_dev); 
			if (p->motor_dev.pdcmotor==ERR_PTR(-EPROBE_DEFER)) {
				printk("mechunit_probe_get_motor:of_node_to_dcmotor err!\n");
				return -1;
			}
			else 
				pr_debug("mechanism_get_motor=%x motor_type=%x\n", (int)p->motor_dev.pdcmotor, p->motor_type);
		}
		
		pr_debug("motor: name=%s mask=%x\n", p->motor_name, p->motor_mask);
		i++;
		if (i==motor_num) 
			break;
		else
			p += 1;
	}
	return 0;
}

int mechunit_probe_get_devtree_pdata(struct device*dev, struct mechanism_uint_data *pmech_unit_data)
{
	struct device_node *node, *ppmotor, *ppsensor;
	int ret, i;
	unsigned int data[4];
	//const char *str;
	//char str_array[50];
	//char *pstr=str_array;
	//const char *pstr;
	const char *pmech_unit_name;

	pr_debug("mechunit driver - mechunit_probe_get_devtree_pdata\n");        
	node = dev->of_node;
	if (!node) {
		return (-ENODEV);
	}

	pr_debug("1\n");
        if (!of_find_property(node, "mech_unit_name",NULL)) {
                 dev_warn(dev, "Found mechanism without  mech_unit_name");
		 return (-EINVAL);
        }

	pr_debug("2\n");
        memset(pmech_unit_data->mech_unit_name, 0, MECHUINT_NAME_LEN);
        ret = of_property_read_string(node, "mech_unit_name", &pmech_unit_name);
        strcpy(pmech_unit_data->mech_unit_name , pmech_unit_name);
        pr_debug("mech_unit_name=%s %s ret=%d\n", pmech_unit_name, pmech_unit_data->mech_unit_name, ret);

	if (!of_find_property(node, "mech_unit_type",NULL)) {
		pmech_unit_data->mech_unit_type = 0;
	}
	else
	{
		ret = of_property_read_u8(node, "mech_unit_type", &pmech_unit_data->mech_unit_type);
		if (ret){
			pr_debug("mechunit_probe_get_devtree_pdata:Failed to read mech_unit_type!\n");
			return ret;
		}
	}
        pr_debug("mech_unit_type=%d \n", pmech_unit_data->mech_unit_type);

	if (!of_find_property(node, "mechanisam_unit",NULL)) {
                 dev_warn(dev, "Found mechanism without  mechanisam_unit");
		 return (-EINVAL);
        }
        of_property_read_u32_array(node, "mechanisam_unit" ,data, ARRAY_SIZE(data));
        pmech_unit_data->unit_motor_data.motor_num = (unsigned char)data[0];
        pmech_unit_data->unit_motor_data.motor = devm_kzalloc(dev, sizeof(struct motor_data) *pmech_unit_data->unit_motor_data.motor_num,GFP_KERNEL); 
	if (pmech_unit_data->unit_motor_data.motor == NULL){
		pr_debug("motor devm_kzalloc error!\n");
		return -ENOMEM;
	}
        pmech_unit_data->unit_sensor_data.sensor_num = (unsigned char)data[2];
        pmech_unit_data->unit_sensor_data.sensor = devm_kzalloc(dev, sizeof(struct sensor_data) *pmech_unit_data->unit_sensor_data.sensor_num,GFP_KERNEL); 
	if (pmech_unit_data->unit_sensor_data.sensor == NULL){
		pr_debug("sensor devm_kzalloc error!\n");
		return -ENOMEM;
	}

	pr_debug("motor_num=%d, sensor_num=%d\n",pmech_unit_data->unit_motor_data.motor_num, pmech_unit_data->unit_sensor_data.sensor_num);
        ppmotor = of_find_node_by_phandle(data[1]);
        ret = mechunit_probe_get_motor(ppmotor, pmech_unit_data->unit_motor_data.motor, pmech_unit_data->unit_motor_data.motor_num); 
	if (ret==-1) {
		return (-EINVAL);
	}

	ppsensor = of_find_node_by_phandle(data[3]);
        ret = mechunit_probe_get_sensor(ppsensor, pmech_unit_data->unit_sensor_data.sensor, pmech_unit_data->unit_sensor_data.sensor_num); 
	if (ret==-1) {
		return (-EINVAL);
	}

	pmech_unit_data->unit_sensor_data.sensor_masks = 0;
	for (i=0; i < pmech_unit_data->unit_sensor_data.sensor_num; i++) {
		pmech_unit_data->unit_sensor_data.sensor_masks |= pmech_unit_data->unit_sensor_data.sensor[i].sen_mask;
	}

	pr_debug("mechanism_dev_unit=%x\n", (int)pmech_unit_data);
       
	return 0;
}
EXPORT_SYMBOL_GPL(mechunit_probe_get_devtree_pdata);

#ifdef MECH_OPTIMIZE_20170503
#include "mechlib.h"
void mechunit_probe_get_control(struct device*dev, struct mechanism_dev_t *pmechanism_dev)
{	
	unsigned char motor_num, sensor_num;

	motor_num = pmechanism_dev->mech_unit_data.unit_motor_data.motor_num;
	sensor_num = pmechanism_dev->mech_unit_data.unit_sensor_data.sensor_num;

	pmechanism_dev->mech_unit_control.mech_unit_motor_feature.motor_num = motor_num;
	pmechanism_dev->mech_unit_control.mech_unit_motor_feature.motor_feature = devm_kzalloc(dev, sizeof(motor_feature_t) * motor_num, GFP_KERNEL); 
	
	pmechanism_dev->mech_unit_control.mech_unit_sen_feature.sen_num = sensor_num;
	pmechanism_dev->mech_unit_control.mech_unit_sen_feature.sen_feature = devm_kzalloc(dev, sizeof(sen_feature_t) * sensor_num, GFP_KERNEL); 

	pmechanism_dev->mech_unit_control.mech_unit_sen_config.sen_num = sensor_num;
	pmechanism_dev->mech_unit_control.mech_unit_sen_config.sen_config = devm_kzalloc(dev, sizeof(sen_config_t) * sensor_num, GFP_KERNEL); 

	pmechanism_dev->mech_unit_control.mech_unit_sen_raw_input.sen_num = sensor_num; 
	pmechanism_dev->mech_unit_control.mech_unit_sen_raw_input.sen_raw_input = devm_kzalloc(dev, sizeof(sen_raw_input_t) * sensor_num, GFP_KERNEL); 
}
#endif

EXPORT_SYMBOL_GPL(mechunit_probe_get_control);

void mechunit_sigio(struct mechanism_dev_t *pmechanism_dev)
{
	#ifdef MECH_SENSOR_INDEX_20170815
	pmechanism_dev->sigio_event += RESN_OFFSET_MECH_DRIVER_MOV;
	#endif
	pr_debug("mechaunit_sigio! mech_sigio=%lx\n", pmechanism_dev->sigio_event); 
	kill_fasync(&pmechanism_dev->async_queue, SIGIO, POLL_IN);
}
EXPORT_SYMBOL_GPL(mechunit_sigio);


//----------------------------------------------------------
int mechunit_open(struct inode *inode, struct file *filep)
{
	struct mechanism_dev_t *dev ;
	dev = container_of(inode->i_cdev, struct mechanism_dev_t, cdev);
	filep->private_data = dev;

	dev->driver_status = MECH_DRIVER_OPENED; 
	dev->mech_status = IDLE;

	return 0;
}
EXPORT_SYMBOL_GPL(mechunit_open);

int mechunit_close(struct inode *inode, struct file *filep)
{
	return 0;
}
EXPORT_SYMBOL_GPL(mechunit_close);

int mechunit_async(int fd, struct file *filep, int mode)
{
	struct mechanism_dev_t *dev = (struct mechanism_dev_t *)(filep->private_data);

	pr_debug("mechanism_async fd=%x, filep=%x mode=%x dev.async_queue=%x\n", fd, (int)filep, mode, (int)dev->async_queue);
	fasync_helper(fd, filep, mode, &dev->async_queue);	
	return 0;
}
EXPORT_SYMBOL_GPL(mechunit_async);


int mechunit_motor_init(mechanism_uint_motor_data_t *punit_motor_data, unsigned short motor_mask, motor_callback_t *pcallback, struct mechanism_dev_t *mech_dev)
{
	struct motor_data *pmotor_data;
	unsigned char i;
	struct callback_data call_back_data;
	
	pr_debug("mechunit_motor_init.............mech_dev=%x motor_mask=%x\n", (int)mech_dev, motor_mask);

	motor_get_data(punit_motor_data, motor_mask, pmotor_data, i);
	if (i==punit_motor_data->motor_num) {
		return -RESN_MECH_ERR_MOTOR_GETDATA;
	}
	//pr_debug("motor_init:pmotor_data=%x %x \n", pmotor_data, &((punit_data)->motor[i]));	
	//pr_debug("motor_completion=%x motor_mask=%x motor_name=%s\n", &pmotor_data->motor_completion, pmotor_data->motor_mask, pmotor_data->motor_name); 
	//pr_debug("motor_num=%d i=%d motor_completion=%x motor_mask=%x motor_name=%s\n", (punit_data)->motor_num, i, &(punit_data)->motor[i].motor_completion, (punit_data)->motor[i].motor_mask, (punit_data)->motor[i].motor_name); 

	call_back_data.data1 = motor_mask;
	call_back_data.data2 = (int)mech_dev;

	switch(pmotor_data->motor_type)
	{
	case MOTOR_STEP_TYPE:	
		//pr_debug("2.steppermotor_set_callback %x %x %x\n", pmotor_data, pmotor_data->motor_dev.psteppermotor, callback);	
		pmotor_data->callback.steppermotor_callback = pcallback->steppermotor_callback;
		steppermotor_set_callback(pmotor_data->motor_dev.psteppermotor, pcallback->steppermotor_callback, &call_back_data); 
		//pr_debug("3.step_motor_init\n");
		break;
	case MOTOR_BRUSHDC_TYPE:
		//pr_debug("dcmotor_set_callback\n");
		pmotor_data->callback.dcmotor_callback = pcallback->dcmotor_callback;
		dcmotor_set_callback(pmotor_data->motor_dev.pdcmotor, pcallback->dcmotor_callback, &call_back_data); 
		break;
	default:
		return -1;
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
EXPORT_SYMBOL_GPL(mechunit_motor_init);

#ifdef MECH_OPTIMIZE_20170428
void mechunit_stepmotor_callback(struct steppermotor *motor, struct callback_data *pcall_back_data)
{
	struct motor_data *pmotor_data;
	int status;
	static unsigned char i=0;
	int motor_mask = pcall_back_data->data1; 
	struct mechanism_dev_t *pmech_dev = (struct mechanism_dev_t *)(pcall_back_data->data2); 

	pr_debug("mechunit_stepmotor_callback...........pmech_dev=%x motor_mask=%x\n", (int)pmech_dev, motor_mask);

	motor_get_data(&pmech_dev->mech_unit_data.unit_motor_data, motor_mask, pmotor_data, i); 
	if (i == pmech_dev->mech_unit_data.unit_motor_data.motor_num) {
		#ifdef MECH_SENSOR_INDEX_20170815
		pmech_dev->sigio_event = MOTOR_STOP_BY_ABNORMAL|MOTOR_STOP_UINT_TO_RES(motor_mask);
		#else
		#ifdef MECH_OPTIMIZE_20170515
		pmech_dev->sigio_event = MOTOR_STOP_BY_ABNORMAL|motor_mask;//(1<<motor_mask);
		#else
		pmech_dev->sigio_event = MOTOR_STOP_BY_ABNORMAL|(1<<(motor_mask-1));
		#endif
		#endif
		mechunit_sigio(pmech_dev);
		pr_debug("err~!\n");
		return;
	}


	//pr_debug("phase_current_num=%d moving_status=%x\n",pmotor_data->phase_current_num, pmotor_data->moving_status);

	//to avoid invalid interrupts after motor stoped
	#ifdef MECH_OPTIMIZE_20170515
	//if(pmotor_data->moving_status != MOTOR_MOVE_STATUS_RUNNING)
	if(!(pmotor_data->moving_status & MOTOR_MOVE_STATUS_RUNNING))
	#else
	if(pmotor_data->moving_status & MOTOR_STOP_STATUS)
	#endif
		return;

	pr_debug("phase_current_num=%x moving_status=%x motor_phase_accout=%d motor_comp_accout=%d\n", 
		pmotor_data->phase_current_num,
		pmotor_data->moving_status, 
		pmotor_data->motor_phase_accout, 
		pmotor_data->motor_comp_accout);

	status = steppermotor_status(pmotor_data->motor_dev.psteppermotor);
    
	pr_debug("status=%x\n", status);
	//pr_debug("pmotor_data=%x, motor_comp_accout=%x\n",  pmotor_data, pmotor_data->motor_comp_accout);

	if (steppermotor_is_medialength_ok(status)) {
		pmech_dev->mech_unit_drv_status.form_length = steppermotor_get_medialength_in_steps(motor); 
		status &= ~steppermotor_is_medialength_ok(status);
		if (steppermotor_is_running(status)) {
		    status &= ~steppermotor_is_running(status);
		}
		if (!status) {
			return;
		}
	}

	if (steppermotor_is_triggersteps_done(status))  {
		//pr_debug("steppermotor_is_triggersteps_done，  motor_phase_accout=%d\n", pmotor_data->motor_phase_accout);

		if (pmotor_data->phase_current_num &&(pmotor_data->pmotor_mov->scantriger_source != NULL)){
			//printk("scantriger_source! phase_current_num=%d \n", pmotor_data->phase_current_num);
			//printk("sen_mask=%x\n", pmotor_data->pmotor_mov->motor_trigger_phase[pmotor_data->phase_current_num-1].sen_mask);
			//printk("motor_sen_flag=%x\n", pmotor_data->pmotor_mov->motor_trigger_phase[pmotor_data->phase_current_num-1].motor_sen_flag);
			if((pmotor_data->pmotor_mov->motor_trigger_phase[pmotor_data->phase_current_num-1].sen_mask==pmotor_data->pmotor_mov->scantriger_source->sensor_mask)&&
			 (pmotor_data->pmotor_mov->motor_trigger_phase[pmotor_data->phase_current_num-1].motor_sen_flag == pmotor_data->pmotor_mov->scantriger_source->motor_sen_flag)) 
			{
				pmech_dev->mech_unit_drv_status.scantriger_status = 1;
			}
		}
	}
	#ifdef MECH_SENSOR_INDEX_20170815
	stepmotor_callback(pmotor_data, pmech_dev, status, MOTOR_STOP_UINT_TO_RES(motor_mask));
	#else
	#ifdef MECH_OPTIMIZE_20170515
	stepmotor_callback(pmotor_data, pmech_dev, status, motor_mask);
	#else
	stepmotor_callback(pmotor_data, pmech_dev, status, (1<<(motor_mask-1)));
	#endif
	#endif
	pr_debug("mechunit_stepmotor_callback...........over! mech_sigio=%lx\n", pmech_dev->sigio_event);
    
}

void mechunit_dcmotor_callback(struct dcmotor *motor,struct callback_data *pcall_back_data)
{
	struct motor_data *pmotor_data;
	int status;
	static unsigned char i=0;
	int motor_mask = pcall_back_data->data1; 
	struct mechanism_dev_t *pmech_dev = (struct mechanism_dev_t *)(pcall_back_data->data2); 

	pr_debug("mechunit_dcmotor_callback...........pmech_dev=%x motor_mask=%x\n", (int)pmech_dev, motor_mask);
 
	motor_get_data(&pmech_dev->mech_unit_data.unit_motor_data, motor_mask, pmotor_data, i); 
	if (i == pmech_dev->mech_unit_data.unit_motor_data.motor_num) {
		#ifdef MECH_OPTIMIZE_20170515
		pmech_dev->sigio_event = MOTOR_STOP_BY_ABNORMAL|motor_mask;//(1<<motor_mask);
		#else
		pmech_dev->sigio_event = MOTOR_STOP_BY_ABNORMAL|(1<<(motor_mask-1));
		#endif
		mechunit_sigio(pmech_dev);
		return;
	}
	  
	pr_debug("mechunit_dcmotor_callback:phase_current_num=%d moving_status=%x\n",pmotor_data->phase_current_num, pmotor_data->moving_status);
	#ifdef MECH_OPTIMIZE_20170515
	//if(pmotor_data->moving_status != MOTOR_MOVE_STATUS_RUNNING)
	if(!(pmotor_data->moving_status & MOTOR_MOVE_STATUS_RUNNING))
	#else
	if(pmotor_data->moving_status & MOTOR_STOP_STATUS)
	#endif
	{
		return;
	}

	status = dcmotor_status(pmotor_data->motor_dev.pdcmotor);
    
	pr_debug("mechunit_dcmotor_callback:status=%x, pmotor_data=%x, motor_comp_accout=%x \n", status, (int)pmotor_data, pmotor_data->motor_comp_accout);
  
	if ((!dcmotor_is_running(status))||(dcmotor_is_stopped_by_sensor(status))||dcmotor_is_error(status)){
		if (!dcmotor_is_running(status))
		{
			#ifdef MECH_OPTIMIZE_20170515
			if (pmotor_data->moving_status & MOTOR_STOP_BY_SOFT_START){
				pmotor_data->stoping_status = MOTOR_STOP_BY_SOFT;
			} else
				pmotor_data->stoping_status = MOTOR_STOP_BY_TOTAL;
			pmotor_data->moving_status = MOTOR_MOVE_STATUS_STOP;
			pmech_dev->sigio_event = pmotor_data->stoping_status|MOTOR_STOP_UINT_TO_RES(motor_mask);
			#else
			if (pmotor_data->moving_status & MOTOR_STOP_BY_SOFT_START){
				pmotor_data->moving_status |= MOTOR_STOP_BY_SOFT;
				pmotor_data->moving_status &= ~MOTOR_STOP_BY_SOFT_START;
			} else
				pmotor_data->moving_status |= MOTOR_STOP_BY_TOTAL;
			pmotor_data->moving_status &= ~MOTOR_MOVE_STATUS_RUNNING;
			pmech_dev->sigio_event = pmotor_data->moving_status|(1<<(motor_mask-1));
			#endif
			pr_debug("mechunit_dcmotor_callback1:sigio_event=%lx ", pmech_dev->sigio_event);
		}
		if ((dcmotor_is_stopped_by_sensor(status))) {
			pmotor_data->stoping_status = MOTOR_STOP_BY_SENSOR;
			pmotor_data->moving_status = MOTOR_MOVE_STATUS_STOP;
			//pmech_dev->sigio_event = pmotor_data->stoping_status|motor_mask;
			
			#ifdef MECH_SENSOR_INDEX_20170815
			pmech_dev->sigio_event = pmotor_data->stoping_status | MOTOR_STOP_UINT_TO_RES(motor_mask) |
				MOTOR_STOP_SEN_POS_TO_RES(pmotor_data->pmotor_mov->motor_trigger_phase[pmotor_data->phase_current_num-1].sen_pos_index) |
			#else
			pmech_dev->sigio_event = pmotor_data->stoping_status | motor_mask |
				(pmotor_data->pmotor_mov->motor_trigger_phase[pmotor_data->phase_current_num-1].sen_mask<<MOTOR_STOP_SEN_POS_SHIFT) |
			#endif
				MOTOR_STOP_SEN_FLAG_TO_RES(pmotor_data->pmotor_mov->motor_trigger_phase[pmotor_data->phase_current_num-1].motor_sen_flag);
			pr_debug("mechunit_dcmotor_callback2:sigio_event=%lx sen_mask=%x sen_pos_index=%d\n", pmech_dev->sigio_event, 
				pmotor_data->pmotor_mov->motor_trigger_phase[pmotor_data->phase_current_num - 1].sen_mask, 
				pmotor_data->pmotor_mov->motor_trigger_phase[pmotor_data->phase_current_num - 1].sen_pos_index); 
		}
		if (dcmotor_is_error(status))
		{
			#ifdef MECH_OPTIMIZE_20170606
			pmotor_data->moving_status = MOTOR_MOVE_STATUS_STOP;
			pmotor_data->stoping_status = MOTOR_STOP_BY_ABNORMAL; 
			pmotor_data->err_status = -RESN_MECH_ERR_MOTOR_HW_ERR;
			#else
			pmotor_data->moving_status = MOTOR_MOVE_STATUS_STOP;
			pmotor_data->stoping_status |= MOTOR_STOP_BY_ABNORMAL; 
			pmech_dev->sigio_event = MOTOR_HW_ERR|pmotor_data->stoping_status |motor_mask;
			#endif
			
		}
		if (pmotor_data->motor_comp_accout != 0)
			complete_all(&(pmotor_data->motor_completion)); 
		mechunit_sigio(pmech_dev);
		
	}
	else if (pmotor_data->moving_status) {
        //pr_debug("motor_phase_accout=%d motor_comp_accout=%d\n", pmotor_data->motor_phase_accout, pmotor_data->motor_comp_accout);
		if (pmotor_data->motor_phase_accout != 0)
			complete(&pmotor_data->motor_phase_completion);
		if (pmotor_data->motor_comp_accout != 0)
			complete_all(&(pmotor_data->motor_completion));
		#ifdef MECH_OPTIMIZE_20170606
		pmotor_data->moving_status = MOTOR_MOVE_STATUS_STOP;
		pmotor_data->stoping_status = MOTOR_STOP_BY_ABNORMAL;
		pmotor_data->err_status = -RESN_MECH_ERR_MOTOR_INT_INVALID;
		#else
		pmotor_data->moving_status = MOTOR_MOVE_STATUS_STOP;
		pmotor_data->stoping_status |= MOTOR_STOP_BY_ABNORMAL; 
		pmech_dev->sigio_event = pmotor_data->stoping_status | motor_mask;
		mechunit_sigio(pmech_dev);
		#endif
	}
	else{
		if (pmotor_data->motor_phase_accout != 0)
			complete(&pmotor_data->motor_phase_completion);
		if (pmotor_data->motor_comp_accout != 0)
			complete_all(&(pmotor_data->motor_completion));

		#ifdef MECH_OPTIMIZE_20170606
		pmotor_data->stoping_status = MOTOR_STOP_BY_ABNORMAL; 
		pmotor_data->moving_status = MOTOR_MOVE_STATUS_STOP;
		pmotor_data->err_status = -RESN_MECH_ERR_MOTOR_INT_INVALID;
		#else
		pmotor_data->stoping_status |= MOTOR_STOP_BY_ABNORMAL; 
		pmotor_data->moving_status = MOTOR_MOVE_STATUS_STOP;
		pmech_dev->sigio_event = MOTOR_INT_INVALID | pmotor_data->stoping_status | motor_mask | ((status&0xff) << MOTOR_STOP_SEN_POS_SHIFT); 
		mechunit_sigio(pmech_dev);
		#endif
	}

	pr_debug("mechunit_dcmotor_callback...........over! mech_sigio=%lx\n", pmech_dev->sigio_event);
    
}
#endif

#ifdef MECH_OPTIMIZE_20170502
static	int 	mech_init(struct mechanism_dev_t * mech_dev, class_cmd *cptr)
{
	int ret=0, i;
	motor_callback_t callback;

	for (i=0; i < mech_dev->mech_unit_data.unit_motor_data.motor_num; i++) {
		if (mech_dev->mech_unit_data.unit_motor_data.motor[i].motor_type==MOTOR_STEP_TYPE) {
			callback.steppermotor_callback = mechunit_stepmotor_callback;
		}
		else
			callback.dcmotor_callback = mechunit_dcmotor_callback;

		ret = mechunit_motor_init(&mech_dev->mech_unit_data.unit_motor_data,
			mech_dev->mech_unit_data.unit_motor_data.motor[i].motor_mask, 
			&callback, mech_dev); 
	
		if (ret)
		{
			return ret;
		}
	}

	return ret;
}


static	int	mech_motor_move(struct mechanism_dev_t * mech_dev, class_cmd *cptr)
{
	mech_control_t *mech_control = (mech_control_t *)(cptr->argptr);
	int ret;

	motor_mov_t motor_mov;
	scantriger_source_t *pscantriger_source = NULL;
	motor_speed_phase_t  *pmotor_speed_phase=NULL;
	motor_trigger_phase_t *pmotor_trigger_phase=NULL;
	mechunit_motor_mov_t  mech_motor_move;

	pr_debug("mech_motor_move...............\n");
	if(copy_from_user((void *)&mech_motor_move,(void __user *)mech_control->buffer,sizeof(mechunit_motor_mov_t)))
		return -EFAULT;

	if (copy_from_user((void *)&motor_mov, (void __user *)mech_motor_move.pmotor_mov, sizeof(motor_mov_t))) 
		return -EFAULT;
	mech_motor_move.pmotor_mov = &motor_mov;

	pr_debug("speed_phase_num=%d trigger_phase_num=%d\n", motor_mov.speed_phase_num, motor_mov.trigger_phase_num);

	if (motor_mov.scantriger_source) {
		pscantriger_source = kzalloc(sizeof(scantriger_source_t), GFP_KERNEL);

		if (!pscantriger_source) {
			pr_debug("alloc media source err\n");
			return (-ENOMEM);
		}

		ret = copy_from_user((void *)pscantriger_source, (void __user *)motor_mov.scantriger_source, sizeof(scantriger_source_t));
		if (ret) 
		{
			pr_debug("sensor_mask=%d\n", motor_mov.scantriger_source->sensor_mask); 
			pr_debug("copy scantriger source err:%x \n", ret);
			kfree(pscantriger_source);
			return -EFAULT;
		}
		motor_mov.scantriger_source = pscantriger_source;
	}

	if (motor_mov.speed_phase_num) {
		pmotor_speed_phase = kzalloc(sizeof(motor_speed_phase_t)*motor_mov.speed_phase_num, GFP_KERNEL);
		if (!pmotor_speed_phase) {
			return (-ENOMEM);
		}
		if (copy_from_user((void *)pmotor_speed_phase, (void __user *)motor_mov.motor_speed_phase, sizeof(motor_speed_phase_t)*motor_mov.speed_phase_num)) 
		{
			kfree(pmotor_speed_phase);
			kfree(pscantriger_source);
			return -EFAULT;
		}
		motor_mov.motor_speed_phase = pmotor_speed_phase;  
	}
	if (motor_mov.trigger_phase_num) {
		pmotor_trigger_phase = kzalloc(sizeof(motor_trigger_phase_t)*motor_mov.trigger_phase_num, GFP_KERNEL);
		if (!pmotor_trigger_phase) {
			kfree(pmotor_speed_phase);
			kfree(pscantriger_source);
			return (-ENOMEM);
		}
		if (copy_from_user((void *)pmotor_trigger_phase, (void __user *)motor_mov.motor_trigger_phase, sizeof(motor_trigger_phase_t) * motor_mov.trigger_phase_num)) 
		{
			kfree(pmotor_speed_phase);
			kfree(pmotor_trigger_phase);
			kfree(pscantriger_source);
			return -EFAULT;
		}
		motor_mov.motor_trigger_phase = pmotor_trigger_phase;  
	}
	else 
		motor_mov.motor_trigger_phase = NULL;

	//ret = motor_init(&acceptor_dev.mech_unit_data.unit_motor_data, ACCEPT_MOTOR, accept_motor_callback);
	//ret = motor_move_init(&mech_dev->mech_unit_data.unit_motor_data, ACCEPT_MOTOR);
	ret = motor_move_init(&mech_dev->mech_unit_data.unit_motor_data, mech_motor_move.motor_mask);
	if (ret) 
		goto end_mech_motor_move;
	mech_dev->sigio_event = 0;
	ret = motor_start(&mech_dev->mech_unit_data.unit_motor_data, &mech_dev->mech_unit_data.unit_sensor_data,
		mech_motor_move.motor_mask, mech_motor_move.dir, &motor_mov);
	if (ret) 
	{
		goto end_mech_motor_move;
	}
	ret = motor_wait_stop(&mech_dev->mech_unit_data.unit_motor_data, 
		&mech_dev->mech_unit_data.unit_sensor_data,
		mech_motor_move.motor_mask); 

end_mech_motor_move:
	if (pscantriger_source){
		kfree(pscantriger_source);
	}
	if (pmotor_speed_phase) {
		kfree(pmotor_speed_phase);
	}
	if (pmotor_trigger_phase) {
		kfree(pmotor_trigger_phase);
	}
	pr_debug("mech_motor_move...............over!ret=%x\n", ret);
	return ret;
}

static	int	mech_stop(struct mechanism_dev_t *mech_dev, class_cmd *cptr)
{
	mech_control_t *pmech_control= (mech_control_t *)(cptr->argptr);
	unsigned short motor_mask = (unsigned short)pmech_control->buffer;

	motor_stop(&mech_dev->mech_unit_data.unit_motor_data, motor_mask); 
		
	return 0;
}

static	int	mech_pps(struct mechanism_dev_t * mech_dev, class_cmd *cptr)
{
	int ret=0;

	switch(MECHCTRL_MODE(cptr))
	{
	case PPS_ON:
		//pr_debug("accept_pps ON: %x\n", ACCEPT_BUFFER(cptr));
		ret = sensor_enable(&mech_dev->mech_unit_data.unit_sensor_data, MECHCTRL_BUFFER(cptr), 1); 
		break;
	case PPS_OFF:
		//pr_debug("accept_pps OFF: %x\n", ACCEPT_BUFFER(cptr));
		ret = sensor_enable(&mech_dev->mech_unit_data.unit_sensor_data, MECHCTRL_BUFFER(cptr), 0); 
		break;
	case PPS_SETCONFIG:
		pr_debug("accept_pps PPS_CONFIG: %lx\n", MECHCTRL_BUFFER(cptr));
		ret = sensor_set_config(&mech_dev->mech_unit_data.unit_sensor_data, &(mech_dev->mech_unit_control.mech_unit_sen_config), (sen_config_t *)MECHCTRL_BUFFER(cptr)); 
		break;
	default:
		break;
	}
	return ret;
}

static	int	mech_lock(struct mechanism_dev_t * mech_dev, class_cmd *cptr)
{
	int ret=0;
	mech_control_t *pmech_control= (mech_control_t *)(cptr->argptr);
	unsigned short motor_mask = (unsigned short)pmech_control->buffer;

	ret = motor_lock(&mech_dev->mech_unit_data.unit_motor_data, motor_mask); 
	return ret;
}

static	int	mech_unlock(struct mechanism_dev_t * mech_dev, class_cmd *cptr)
{	
	int ret=0;
	mech_control_t *pmech_control= (mech_control_t *)(cptr->argptr);
	unsigned short motor_mask = (unsigned short)pmech_control->buffer;

	ret = motor_unlock(&mech_dev->mech_unit_data.unit_motor_data, motor_mask); 
	return ret;
}


static	int	mech_motor_move_lock(struct mechanism_dev_t * mech_dev, class_cmd *cptr)
{
	int ret=0;

	ret = mech_motor_move(mech_dev, cptr);
	if (!ret) {
		ret = mech_lock(mech_dev, cptr);
	}
	return ret;
}

static	int	mech_skew_set(struct mechanism_dev_t * mech_dev, class_cmd *cptr)
{	
	int ret=0;
	mech_control_t *pmech_control= (mech_control_t *)(cptr->argptr);
	unsigned short motor_mask = (unsigned short)pmech_control->buffer;
	int	steps =  (int)pmech_control->steps;

	ret = motor_skewsteps_set(&mech_dev->mech_unit_data.unit_motor_data, motor_mask, steps); 
	return ret;
}

seqptr_t	const mechunit_library[MECH_CTRL_FUNCTIONS] = {
	mech_init,
        mech_motor_move,
        mech_stop,
        mech_pps,
	mech_motor_move_lock,
	mech_lock,
	mech_unlock,
	mech_skew_set
};

static long mechunit_ioctl( struct file *filep, unsigned int ioctrl_cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	class_cmd classcmd;
	int ret=RESN_MECH_NO_ERROR;
	mech_control_t mech_ctrl;
	struct mechanism_dev_t *mech_dev = (struct mechanism_dev_t *)(filep->private_data);
	unsigned int cmd;

	cmd = ioctrl_cmd&0xff;	/* convert standard IOCTL number to engine IOCTL command id */
	

	//pr_debug("mechunit_ioctl: cmd=%d \n", cmd);
	switch (cmd)
	{
	case GET_MECH_STATUS:
		ret = mechunit_get_sensor_status(&mech_dev->mech_unit_data, mech_dev->mech_unit_data.unit_sensor_data.sensor_masks, &mech_dev->mech_unit_drv_status.sen_status); 
		if (ret)
		{
			printk(KERN_INFO "mechunit_ioctl GET_MECH_STATUS: acceptpath_getstatus fail\n");
			goto mechunit_ioctl_ret;
		}
		if (copy_to_user((void __user *)argp, (void *)&(mech_dev->mech_unit_drv_status), sizeof(mechunit_drv_status_t))) 
		{
			printk(KERN_INFO "mechunit_ioctl GET_MECH_STATUS: copy_to_user fail\n");
			ret = -EFAULT;
			goto mechunit_ioctl_ret;
		}
		goto mechunit_ioctl_ret;
	case GET_MECH_SENSORS_RAW:
		ret = mechunit_get_sensors_rawinput(&mech_dev->mech_unit_data, &(mech_dev->mech_unit_control.mech_unit_sen_raw_input));
		if (ret) 
		{
			printk(KERN_INFO "mechunit_ioctl GET_MECH_SENSORS_RAW: acceptpath_get_sen_rawinput fail\n");
			goto mechunit_ioctl_ret;
		}
		#ifdef MECH_OPTIMIZE_20170503
		put_user(mech_dev->mech_unit_control.mech_unit_sen_raw_input.sen_num, (int __user *)(&(((mech_unit_sen_raw_input_t *)argp)->sen_num)));
		if (copy_to_user((void __user *)(((mech_unit_sen_raw_input_t *)argp)->sen_raw_input), 
				 (void *)(mech_dev->mech_unit_control.mech_unit_sen_raw_input.sen_raw_input), 
			sizeof(sen_raw_input_t)*mech_dev->mech_unit_control.mech_unit_sen_raw_input.sen_num))
		#else
		if (copy_to_user((void *)arg, (void *)&(mech_dev->mech_unit_control.mech_unit_sen_raw_input), sizeof(mech_unit_sen_raw_input_t))) 
		#endif
		{
			printk(KERN_INFO "mechunit_ioctl GET_MECH_SENSORS_RAW: copy_to_user fail\n");
			ret = -EFAULT;
			goto mechunit_ioctl_ret;
		}
		goto mechunit_ioctl_ret;
	case GET_MECH_SENSOR_VAL:
		{
			int i;
			sen_raw_input_t	sen_raw_input;

			sen_raw_input.sen_mask = ((sen_raw_input_t *)arg)->sen_mask; 

			ret = sensor_get_val(&(mech_dev->mech_unit_data.unit_sensor_data), sen_raw_input.sen_mask, &sen_raw_input.raw_input_value[0]); 
			if (ret) 
			{
				printk(KERN_INFO "mechunit_ioctl GET_MECH_SENSOR_VAL: sensor_get_val fail\n");
				goto mechunit_ioctl_ret;
			}

			pr_debug("GET_MECH_SENSOR_VAL:  sen_raw_input.sen_mask=%x val=%ld\n", sen_raw_input.sen_mask, sen_raw_input.raw_input_value[0]);

			for (i=1; i<=SENSOR_RAW_GET_NUM; i++) {
				sen_raw_input.raw_input_value[i] = sen_raw_input.raw_input_value[0];
			}

			if (copy_to_user((void __user *)(((sen_raw_input_t *)argp)->raw_input_value), 
					 (void *)(sen_raw_input.raw_input_value), 
					sizeof(sen_raw_input.raw_input_value))) 
			{
				printk(KERN_INFO "mechunit_ioctl GET_MECH_SENSOR_VAL: copy_to_user fail\n");
				ret = -EFAULT;
				goto mechunit_ioctl_ret;
			}

		}
		goto mechunit_ioctl_ret;
	case GET_MECH_CONFIG:
		pr_debug("GET_MECH_CONFIG!");
		#ifdef MECH_OPTIMIZE_20170503
		put_user(mech_dev->mech_unit_control.mech_unit_sen_config.sen_num, (int __user *)(&(((mech_unit_sen_config_t *)argp)->sen_num)));
		if (copy_to_user((void __user *)(((mech_unit_sen_config_t *)argp)->sen_config), 
			(void *)(mech_dev->mech_unit_control.mech_unit_sen_config.sen_config), 
			sizeof(sen_config_t)*mech_dev->mech_unit_control.mech_unit_sen_config.sen_num)) 
		#else
		if (copy_to_user((void *)arg, (void *)&(mech_dev->mech_unit_control.mech_unit_sen_config), sizeof(mech_unit_sen_config_t))) 
		#endif
		{
			printk(KERN_INFO "mechunit_ioctl GET_MECH_CONFIG: copy_to_user fail\n");
			ret = -EFAULT;
			goto mechunit_ioctl_ret;
		}
		goto mechunit_ioctl_ret;
	case SET_MECH_CONFIG:
		#ifdef MECH_OPTIMIZE_20170503
		if (copy_from_user((void *)(mech_dev->mech_unit_control.mech_unit_sen_config.sen_config),
			(void __user *)(((mech_unit_sen_config_t *)argp)->sen_config), 
			sizeof(sen_config_t)*mech_dev->mech_unit_control.mech_unit_sen_config.sen_num)) 

		#else
		if(copy_from_user((void *)&(mech_dev->mech_unit_control.mech_unit_sen_config),(void *)arg,sizeof(mech_unit_sen_config_t)))
		#endif
		{
			printk(KERN_INFO "mechunit_ioctl SET_MECH_CONFIG: copy_from_user fail\n");
			ret = -EFAULT;
			goto mechunit_ioctl_ret;
		}
		ret = mechunit_set_sensor_config(&mech_dev->mech_unit_data, &(mech_dev->mech_unit_control.mech_unit_sen_config)); 
		if (ret) 
		{
			printk(KERN_INFO "mechunit_ioctl SET_MECH_CONFIG: mechunit_set_sensor_config fail\n");
			goto mechunit_ioctl_ret;
		}
		goto mechunit_ioctl_ret;
	case GET_MECH_SIGIO:
		pr_debug("GET_MECH_SIGIO!");
		if (copy_to_user((void __user *)argp, (void *)&mech_dev->sigio_event, sizeof(mech_dev->sigio_event))) 
		{
			printk(KERN_INFO "mechunit_ioctl GET_MECH_SIGIO: copy_to_user fail\n");
			ret = -EFAULT;
			goto mechunit_ioctl_ret;
		}
		goto mechunit_ioctl_ret;
	case GET_MECH_PPSFEATURE:
		pr_debug("GET_MECH_PPSFEATURE!");
		#ifdef MECH_OPTIMIZE_20170503
		#else
		if (copy_from_user((void *)&(mech_dev->mech_unit_control.mech_unit_sen_feature), (void *)arg, sizeof(mech_unit_sen_feature_t)))
		{
			printk(KERN_INFO "mechunit_ioctl GET_MECH_PPSFEATURE: copy_from_user fail\n");
			ret = -EFAULT;
			goto mechunit_ioctl_ret;
		}
		#endif
		pr_debug("sen_num=%d \n", mech_dev->mech_unit_control.mech_unit_sen_feature.sen_num); 
		ret = mechunit_get_sensor_feature(&mech_dev->mech_unit_data, &(mech_dev->mech_unit_control.mech_unit_sen_feature)); 
		if (ret)
		{
			printk(KERN_INFO "mechunit_ioctl GET_MECH_PPSFEATURE: mechunit_get_sensor_feature fail & ret=%x\n", ret);
			goto mechunit_ioctl_ret;
		}
		#ifdef MECH_OPTIMIZE_20170503
		put_user(mech_dev->mech_unit_control.mech_unit_sen_feature.sen_num, (int __user *)(&(((mech_unit_sen_feature_t *)argp)->sen_num)));
		if (copy_to_user((void __user *)(((mech_unit_sen_feature_t *)argp)->sen_feature), 
			(void *)(mech_dev->mech_unit_control.mech_unit_sen_feature.sen_feature), 
			sizeof(sen_feature_t)*mech_dev->mech_unit_control.mech_unit_sen_feature.sen_num)) 
		#else
		if(copy_to_user((void *)arg,(void *)&(mech_dev->mech_unit_control.mech_unit_sen_feature),sizeof(mech_unit_sen_feature_t)))
		#endif
		{
			printk(KERN_INFO "mechunit_ioctl GET_MECH_PPSFEATURE: copy_to_user fail\n");
			ret = -EFAULT;
			goto mechunit_ioctl_ret;
		}
		goto mechunit_ioctl_ret;
	case GET_MECH_MOTORFEATURE:
		#ifdef MECH_OPTIMIZE_20170503
		#else
		if(copy_from_user((void *)&(mech_dev->mech_unit_control.mech_unit_motor_feature),(void *)arg,sizeof(mech_unit_motor_feature_t)))
		{
			printk(KERN_INFO "mechunit_ioctl GET_MECH_MOTORFEATURE: copy_from_user fail\n");
			ret = -EFAULT;
			goto mechunit_ioctl_ret;
		}
		#endif

		ret = mechunit_get_motor_feature(&mech_dev->mech_unit_data, &(mech_dev->mech_unit_control.mech_unit_motor_feature)); 
		if (ret)
		{
			printk(KERN_INFO "mechunit_ioctl GET_MECH_MOTORFEATURE: mechunit_get_sensor_feature fail\n");
			goto mechunit_ioctl_ret;
		}
		#ifdef MECH_OPTIMIZE_20170503
		put_user(mech_dev->mech_unit_control.mech_unit_motor_feature.motor_num, (int __user *)(&(((mech_unit_motor_feature_t *)argp)->motor_num)));
		if (copy_to_user((void __user *)(((mech_unit_motor_feature_t *)arg)->motor_feature), 
				(void *)(mech_dev->mech_unit_control.mech_unit_motor_feature.motor_feature), 
				sizeof(motor_feature_t) * mech_dev->mech_unit_control.mech_unit_motor_feature.motor_num)) 
		#else
		if(copy_to_user((void *)arg,(void *)&(mech_dev->mech_unit_control.mech_unit_motor_feature),sizeof(mech_unit_motor_feature_t)))
		#endif
		{
			printk(KERN_INFO "mechunit_ioctl GET_MECH_MOTORFEATURE: copy_to_user fail\n");
			ret = -EFAULT;
			goto mechunit_ioctl_ret;
		}
		goto mechunit_ioctl_ret;
	case GET_MECH_MECHFEATURE:
		{
			mechunit_feature_t mechunit_feature;

			mechunit_feature.mech_unit_type = mech_dev->mech_unit_data.mech_unit_type; 
			mechunit_feature.motor_num = mech_dev->mech_unit_control.mech_unit_motor_feature.motor_num;
			mechunit_feature.sensor_num = mech_dev->mech_unit_control.mech_unit_sen_feature.sen_num;
			if(copy_to_user((void __user *)argp,(void *)&(mechunit_feature),sizeof(mechunit_feature_t)))
			{
				printk(KERN_INFO "mechunit_ioctl GET_MECH_MECHFEATURE: copy_to_user fail\n");
				ret = -EFAULT;
				goto mechunit_ioctl_ret;
			}
			goto mechunit_ioctl_ret;
		}

		goto mechunit_ioctl_ret;
	case GET_MECH_MOTOR_RUNNING_STEPS:
		{
			unsigned short motor_mask = ((mechunit_motor_steps_t *)arg)->motor_mask; 
			int steps=0;

			ret = motor_get_running_steps(&(mech_dev->mech_unit_data.unit_motor_data), motor_mask, &steps);
			if (ret) 
			{
				printk(KERN_INFO "mechunit_ioctl GET_MECH_MOTOR_RUNNING_STEPS: motor_get_running_steps fail\n");
				goto mechunit_ioctl_ret;
			}

			if (copy_to_user((void __user *)(&(((mechunit_motor_steps_t *)argp)->motor_steps)), 
					 (void *)&steps, sizeof(int)))
			{
				printk(KERN_INFO "mechunit_ioctl GET_MECH_MOTOR_RUNNING_STEPS: copy_to_user fail\n");
				ret = -EFAULT;
				goto mechunit_ioctl_ret;
			}
		}
		goto mechunit_ioctl_ret;
	case PAP_GET_SCANTRIGER_STATUS:
		if (copy_to_user((void __user *)argp, (void *)&(mech_dev->mech_unit_drv_status.scantriger_status), 
			sizeof(mech_dev->mech_unit_drv_status.scantriger_status))) 
		{
			printk(KERN_INFO "mechunit_ioctl PAP_GET_SCANTRIGER_STATUS: copy_to_user fail\n");
			ret = -EFAULT;
			goto mechunit_ioctl_ret;
		}
		break;
	case PAP_RESET_SCANTRIGER_STATUS:
		mech_dev->mech_unit_drv_status.scantriger_status = 0;
		break;
	case MECH_CONTROL_CMD:
		if (copy_from_user(&mech_ctrl, (void __user *)argp, sizeof(mech_control_t)))
		{
			printk("mechunit_ioctl: MECH_CONTROL_CMD copy_from_user error\n");
			ret = -EFAULT;
			goto mechunit_ioctl_ret;
		}
		classcmd.argptr = (address)&mech_ctrl;
		pr_debug("mechunit_ioctl: mech_ctrl.cmd=%d \n", mech_ctrl.cmd);
		if (mech_ctrl.cmd>=MECH_CTRL_FUNCTIONS) {
			printk("mechunit_ioctl: mech_ctrl.cmd out of range\n");
			mech_dev->mech_unit_drv_status.mechdev_status = RECOVERABLE; 
			ret = -RESN_MECH_ERR_IVALID_CMD;
			goto mechunit_ioctl_ret;
		}
		ret = mechunit_library[mech_ctrl.cmd](mech_dev, &classcmd);
		//pr_debug("mechunit_ioctl: ret=%x \n", ret);
		if (ret)
		{
			printk(KERN_INFO "mechunit_ioctl MECH_CONTROL_CMD: libfun fail\n");
			goto mechunit_ioctl_ret;
		}
		break;
	default:
	        printk("mechunit_ioctl: cmd=%x out of range", cmd);
		mech_dev->mech_unit_drv_status.mechdev_status = RECOVERABLE; 
	        ret = -RESN_MECH_ERR_IVALID_CMD;
		goto mechunit_ioctl_ret;
	}
mechunit_ioctl_ret:
	//printk("paperpath_ioctl over!ret=%x\n", ret);
	mech_dev->mech_status = IDLE;
	return ret;
}
#endif

#ifdef MECH_OPTIMIZE_20170502
static ssize_t flag_show(struct device *dev, struct device_attribute *attr, char *buf );
static ssize_t flag_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t mech_show(struct device *dev, struct device_attribute *attr, char *buf );
static ssize_t mech_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);


static DEVICE_ATTR(mech_flag, S_IRUGO | S_IWUSR, flag_show, flag_set);
static DEVICE_ATTR(mech_mechio, S_IRUGO | S_IWUSR, mech_show, mech_set);

static struct attribute *mech_attr_list[] = {
        &dev_attr_mech_flag.attr,    
        &dev_attr_mech_mechio.attr,
        NULL,
};

static const struct attribute_group mech_attr_group = {
	.attrs = (struct attribute **) mech_attr_list,
};

#include "mechlib.h"
static ssize_t mech_show(struct device *dev, struct device_attribute *attr, char *buf )
{

	size_t count = 0;

    //sprintf(&buf[count], "mech_status:%lu\n", mech_status.mech_status); 

	return count;
}

static ssize_t mech_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long value;
    unsigned char cmd;
    unsigned short arg;
  //  class_cmd classcmd;
    mech_control_t mech_ctrl;
;
    sscanf(buf, "%lx", &value);
    cmd = (unsigned char)((value>>16)&0xff);
    arg = (unsigned short)(value & 0xffff);
    //pr_debug("mech_set:value=%lx cmd=%x arg=%x\n", value, cmd, arg);

    mech_ctrl.cmd = arg;
         pr_debug("acceptor_set: mech_ctrl.cmd=%d \n", mech_ctrl.cmd);
        if (mech_ctrl.cmd>=MECH_CTRL_FUNCTIONS) {
            pr_debug("acceptor_set: accept.cmd out of range");
	   // accept_uint_status.acceptdev_status = RECOVERABLE; 
            return 0;
        }
      //  accept_library[mech_ctrl.cmd](&acceptor_dev, &classcmd);
	return 0;
}
//-----------------------------------------------------------------
static unsigned long flag=0;
static ssize_t flag_show(struct device *dev, struct device_attribute *attr, char *buf )
{

	size_t count = 0;

	count += sprintf(&buf[count], "%lu\n", flag);

	return count;
}



static ssize_t flag_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	
	//struct mechnism_dev_t *mech_dev=container_of(dev->object, struct mechnism_dev_t, cdev) ;
	flag = buf[0]-'0';
	#if 0
	kill_fasync(&paperpath_dev.async_queue, SIGIO, POLL_IN);
	#else
	//mechunit_sigio(&acceptor_dev);
	#endif
	return count;
}


static struct file_operations mech_fops = {
	.owner      = THIS_MODULE,
	.open       = mechunit_open,
	.release    = mechunit_close,
	.unlocked_ioctl      = mechunit_ioctl,
	.fasync	= mechunit_async,
};

int mechunit_probe(struct platform_device *pdev)
{
	int err;
	struct mechanism_dev_t *pmechanism_dev;
	printk("mechunit_probe\n");

	pmechanism_dev = devm_kzalloc(&pdev->dev, sizeof(struct mechanism_dev_t), GFP_KERNEL);  
	dev_dbg(&pdev->dev,"size of sizeof(struct mechanism_dev_t) is %d\n",sizeof(struct mechanism_dev_t));
	if (pmechanism_dev == NULL)
		return -ENOMEM;
	pmechanism_dev->dev = &pdev->dev;

	mechunit_probe_get_devtree_pdata(&pdev->dev, &pmechanism_dev->mech_unit_data);
	pr_debug("mech_unit_name=%s\n", pmechanism_dev->mech_unit_data.mech_unit_name); 
    

#ifdef MECH_OPTIMIZE_20170503
	mechunit_probe_get_control(&pdev->dev, pmechanism_dev);
#endif
	pmechanism_dev->dev_major = 0;

	pmechanism_dev->dev_no = MKDEV(pmechanism_dev->dev_major, 0);

	if(pmechanism_dev->dev_major)
		err = register_chrdev_region(pmechanism_dev->dev_no, 1, pmechanism_dev->mech_unit_data.mech_unit_name);
	else
	{
		err = alloc_chrdev_region(&pmechanism_dev->dev_no, 0, 1, pmechanism_dev->mech_unit_data.mech_unit_name);
		pmechanism_dev->dev_major = MAJOR(pmechanism_dev->dev_no);
	}
	
	pr_debug( KERN_INFO ": major= %x, dev_no=%x err=%d\n", pmechanism_dev->dev_major, pmechanism_dev->dev_no,err);

    	if (err < 0)
	{
		pr_debug(KERN_NOTICE "Error %x register chrdev", err);
		goto error_reg;
	}

	cdev_init(&pmechanism_dev->cdev, &mech_fops);
	pmechanism_dev->cdev.owner = THIS_MODULE;

	
	
	pmechanism_dev->mech_class = class_create(THIS_MODULE, pmechanism_dev->mech_unit_data.mech_unit_name);
	if(IS_ERR(pmechanism_dev->mech_class))
	{
		pr_debug(KERN_NOTICE "Error %x class_create", (int)(pmechanism_dev->mech_class));
		return PTR_ERR(pmechanism_dev->mech_class);
	}

	pmechanism_dev->mech_dev = device_create(pmechanism_dev->mech_class, NULL, pmechanism_dev->dev_no, NULL, pmechanism_dev->mech_unit_data.mech_unit_name);
	if(IS_ERR(pmechanism_dev->mech_dev))
	{
		pr_debug(KERN_NOTICE "Error %x device_create", (int)(pmechanism_dev->mech_dev));
		return PTR_ERR(pmechanism_dev->mech_dev);
	}

	err = sysfs_create_group(&pmechanism_dev->dev->kobj, &mech_attr_group);	
	if(err)
	{
		pr_debug(KERN_NOTICE "Error %x sysfs_create_group", err);
		goto error_reg;
	}

	err = cdev_add(&pmechanism_dev->cdev, pmechanism_dev->dev_no, 1);
	if(err)
	{
		pr_debug(KERN_NOTICE "Error %x adding mechnism", err);
		//pr_err("Failed to add %s as char device\n", pmechanism_dev->mech_unit_data.mech_unit_name);
		//printk("Failed to add %s as char device\n", pmechanism_dev->mech_unit_data.mech_unit_name);
		sysfs_remove_group(&pmechanism_dev->dev->kobj, &mech_attr_group);
		device_destroy(pmechanism_dev->mech_class, pmechanism_dev->dev_no);
		class_destroy(pmechanism_dev->mech_class);
		cdev_del(&pmechanism_dev->cdev);
		unregister_chrdev_region(pmechanism_dev->dev_no, 1);
		goto error_reg;
	}
	else
		printk("cdev_add %s OK!\n", pmechanism_dev->mech_unit_data.mech_unit_name);
	platform_set_drvdata(pdev, pmechanism_dev);
error_reg:
	return err;
}
EXPORT_SYMBOL_GPL(mechunit_probe);

int mechunit_remove(struct platform_device *pdev)
{
	struct device *dev;
	struct mechanism_dev_t *pmechanism_dev= platform_get_drvdata(pdev);
	pr_debug("Mechnism Driver - exit\n");
	
	dev = &pdev->dev;

	sysfs_remove_group(&dev->kobj, &mech_attr_group);
	device_destroy(pmechanism_dev->mech_class, pmechanism_dev->dev_no);

	class_destroy(pmechanism_dev->mech_class);

	cdev_del(&pmechanism_dev->cdev);
	unregister_chrdev_region(pmechanism_dev->dev_no, 1);
	
	return 0;
}
EXPORT_SYMBOL_GPL(mechunit_remove);
#endif
//----------------------------------------------------------
//#ifdef MECH_OPTIMIZE_20170508
#if 1
static const struct of_device_id mechanism_of_match[]={
    {.compatible = "gwi,mechanism",},
    {},
};

MODULE_DEVICE_TABLE(of, mechanism_of_match);

static struct platform_driver mechanism_driver={
	.driver = {
		.name = "mechanism",
		.owner = THIS_MODULE,
		.of_match_table = mechanism_of_match,
	},
	.probe = mechunit_probe,
	.remove = mechunit_remove,
};

module_platform_driver(mechanism_driver);
#else

static int __init mechunit_init(void)
{
	return 0;
}


static void __exit mechunit_init_exit(void)
{
	
}
module_init(mechunit_init);
module_exit(mechunit_init_exit);
#endif
MODULE_AUTHOR("HL");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("mech_unit driver module");
MODULE_ALIAS("mech_unit driver module");
