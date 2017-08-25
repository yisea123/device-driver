#include <linux/module.h>

#include "mech_sensor.h"

#define for_each_mask_of_unit(masks, mask, i) \
	for (i=0, mask = masks&0x01; i<16; i++, mask = (masks & (0x01<<i)))
	//for (i=0, mask = masks&0x01; (i<16)&&masks; i++, mask = (masks & (0x01<<i)), masks &= ~(0x01<<i))


//----------------------sensor init----------------------
int sensor_init(mechanism_uint_sensor_data_t *punit_sensor_data,  unsigned short sen_masks)
{
	struct sensor_data *psen_data;
	unsigned short sen_mask, i,j;
	int ret=0;

	for_each_mask_of_unit(sen_masks, sen_mask, i) {
		if (!sen_mask) continue;
		
		sensor_get_data(punit_sensor_data, sen_mask, psen_data, j);
		if (j==punit_sensor_data->sensor_num) {
		    return -RESN_MECH_ERR_SENSOR_GETDATA;
		}
		switch (psen_data->sen_type) 
		{
		case SEN_ANALOG_TYPE:
		case SEN_DIGITAL_TYPE:
			break;
		default:
		    return -1;
		}

		ret = photosensor_get_config(psen_data->sen_dev.pphotosensor, &(psen_data->config));
		if (ret) {
			return ret;
		}

		psen_data->config.trigger.mode = 0;
		psen_data->config.trigger.enable = 0;
		ret = photosensor_set_config(psen_data->sen_dev.pphotosensor, &(psen_data->config));
		if (ret) {
		    return ret;
		}
	}
	return 0;
}
EXPORT_SYMBOL_GPL(sensor_init);
//----------------------sensor enable/disable---------------------- 
int sensor_enable(mechanism_uint_sensor_data_t *punit_sensor_data, unsigned short sen_masks, unsigned char enable)
{
	struct sensor_data *psen_data;
	unsigned short sen_mask, i, j;
	int ret=0;

	for_each_mask_of_unit(sen_masks, sen_mask, i) {
		if (!sen_mask) continue;

		sensor_get_data(punit_sensor_data, sen_mask, psen_data, j);
		if (j==punit_sensor_data->sensor_num) {
			return -RESN_MECH_ERR_SENSOR_GETDATA;
		}

		pr_debug("sensor_enable: mask=%x enable=%x\n", sen_mask, enable);

		if (!enable){
			ret = photosensor_disable(psen_data->sen_dev.pphotosensor); 
		}
		else
			ret = photosensor_enable(psen_data->sen_dev.pphotosensor); 

		if (ret) {
			return ret;
		}
	}
	return 0;
}
EXPORT_SYMBOL_GPL(sensor_enable);
//----------------------get the output value of sensor----------------------
int sensor_get_val(mechanism_uint_sensor_data_t *punit_sensor_data, unsigned short sen_mask, unsigned long *val)
{
	struct sensor_data *psen_data;
	unsigned short j;
        //pr_debug("sensor_get_val mask=%x \n", sen_mask);
	//pr_debug("sensor_get_val sensor_num=%d \n", punit_sensor_data->sensor_num); 
        sensor_get_data(punit_sensor_data, sen_mask, psen_data, j);
        if (j==punit_sensor_data->sensor_num) {
            return -RESN_MECH_ERR_SENSOR_GETDATA;
        }
       
	return photosensor_read_input(psen_data->sen_dev.pphotosensor, val);
}
EXPORT_SYMBOL_GPL(sensor_get_val);
//----------------------get the status of sensor(detected or undetected)----------------------
int sensor_get_appval(mechanism_uint_sensor_data_t *punit_sensor_data, unsigned short sen_mask, unsigned int *appval)
{
	struct sensor_data *psen_data;
	unsigned short j;
	int ret=0;

	sensor_get_data(punit_sensor_data, sen_mask, psen_data, j);
	if (j==punit_sensor_data->sensor_num) {
		return -RESN_MECH_ERR_SENSOR_GETDATA;
	}
	//pr_debug("sensor_get_appval sen_mask=%x\n", sen_mask);
	ret =  photosensor_status(psen_data->sen_dev.pphotosensor, appval); 
	//pr_debug("sensor_get_appval appval=%x\n", *appval);
	return ret;
}
EXPORT_SYMBOL_GPL(sensor_get_appval);
//----------------------set sensor trigger----------------------
int sensor_set_trigger(mechanism_uint_sensor_data_t *punit_sensor_data, unsigned short sen_masks, unsigned char trigger_type)
{
	struct sensor_data *psen_data;
	unsigned short sen_mask, i, j;
	int ret=0;

	for_each_mask_of_unit(sen_masks, sen_mask, i) {
		if (!sen_mask) continue;

		sensor_get_data(punit_sensor_data, sen_mask, psen_data, j);
		if (j==punit_sensor_data->sensor_num) {
		    return -RESN_MECH_ERR_SENSOR_GETDATA;
		}
        
	
		ret = photosensor_get_config(psen_data->sen_dev.pphotosensor, &(psen_data->config));
		if (ret) {
			return ret;
		}

		if (trigger_type == SEN_MEDIA_IN) {
			psen_data->config.trigger.mode = 1;
		}
		else
			psen_data->config.trigger.mode = 0;

		psen_data->config.trigger.enable = 1;
		ret = photosensor_set_config(psen_data->sen_dev.pphotosensor, &(psen_data->config));
	}
	return ret;
}
EXPORT_SYMBOL_GPL(sensor_set_trigger);
//----------------------set sensor next trigger----------------------
int sensor_set_trigger_next(mechanism_uint_sensor_data_t *punit_sensor_data, unsigned short sen_masks, unsigned char trigger_type)
{
	struct sensor_data *psen_data;
	unsigned short sen_mask, i, j;
	int ret=0;
	struct sensor_trigger trigger;

	pr_debug("sensor_set_trigger_next : masks=%x \n",sen_masks);
	for_each_mask_of_unit(sen_masks, sen_mask, i) {
		if (!sen_mask) continue;

		sensor_get_data(punit_sensor_data, sen_mask, psen_data, j);
		if (j==punit_sensor_data->sensor_num) {
			return -RESN_MECH_ERR_SENSOR_GETDATA;
		}
       
		pr_debug("sensor_set_trigger_next : masks=%x trigger_type=%d\n",sen_mask, trigger_type);
		trigger.mode = trigger_type;
		trigger.enable = 1;
		ret = photosensor_set_trigger_next(psen_data->sen_dev.pphotosensor, &trigger);
		if (!ret)
			continue;
	}
	pr_debug("sensor_set_trigger_next : over! & ret=%x \n",ret);
	return ret;
}
EXPORT_SYMBOL_GPL(sensor_set_trigger_next);

int sensor_clear_trigger_next(mechanism_uint_sensor_data_t *punit_sensor_data, unsigned short sen_masks)
{
	struct sensor_data *psen_data;
	unsigned short sen_mask, i, j;
	int ret=0;

	//pr_debug("sensor_clear_trigger_next\n");
	for_each_mask_of_unit(sen_masks, sen_mask, i) {
	//    pr_debug("sensor_clear_trigger_next : i=%d sen_mask=%x \n",i, sen_mask);
		if (!sen_mask) continue;

		sensor_get_data(punit_sensor_data, sen_mask, psen_data, j);
		//pr_debug("sensor_clear_trigger_next :j=%d sen_mask=%x \n",j,sen_mask);
		if (j==punit_sensor_data->sensor_num) {
			return -RESN_MECH_ERR_SENSOR_GETDATA;
		}

	//pr_debug("sensor_clear_trigger_next : !!!sen_mask=%x \n",sen_mask);
		ret = photosensor_clear_trigger_next(psen_data->sen_dev.pphotosensor);
		if (ret) {
			return ret;
		    }

	}
	return 0;
}
EXPORT_SYMBOL_GPL(sensor_clear_trigger_next);
//----------------------set sensor config----------------------
int sensor_set_config(mechanism_uint_sensor_data_t *punit_sensor_data, mech_unit_sen_config_t *pmech_unit_sen_config, sen_config_t *p_sen_config )
{
	struct sensor_data *psen_data;
	struct photosensor_config config;
	unsigned short sen_mask, i, j, k;
	int ret=0;

	pr_debug("\nsensor_set_config :sen_masks=%x pps_drv_value=%ld\n", p_sen_config->sen_mask, p_sen_config->pps_drv_value);

	for_each_mask_of_unit(p_sen_config->sen_mask, sen_mask, i) {
		if (!sen_mask) continue;

		sensor_get_data(punit_sensor_data, sen_mask, psen_data, j);
		if (j==punit_sensor_data->sensor_num) {
			return -RESN_MECH_ERR_SENSOR_GETDATA;
		}

		//pr_debug(" sensor_set_config:i=%d mask=%x led_brightness=%d compare_threshold=%d\n", i, sen_mask, p_sen_config->pps_drv_value, p_sen_config->pps_ref_value);
		//pr_debug("pphotosensor=%x type=%x\n", psen_data->sen_info.pphotosensor, psen_data->sen_info.pphotosensor->type); 
		ret = photosensor_get_config(psen_data->sen_dev.pphotosensor, &config);
		if (ret) {
			return ret;
		}
             

	       //pr_debug("sensor=%x type=%x\n", psen_data->sen_info.pphotosensor, psen_data->sen_info.pphotosensor->type); 
	       pr_debug("sensor_set_config:sen_mask=%x trigger_enable=%d trigger_mode=%d\n", sen_mask, p_sen_config->trigger_enable, p_sen_config->trigger_mode);
	       //pr_debug("pps_drv_value=%d\n", p_sen_config->pps_drv_value);
	      // pr_debug("sen_mask=%x sensor=%x \n", sen_mask, psen_data->sen_info.pphotosensor);

		config.compare_threshold = p_sen_config->pps_ref_value; 
		config.led_brightness = p_sen_config->pps_drv_value; 
		config.trigger.enable = p_sen_config->trigger_enable; 
		config.trigger.mode = p_sen_config->trigger_mode; 
		for (k=0; k < pmech_unit_sen_config->sen_num; k++) {
			if (pmech_unit_sen_config->sen_config[k].sen_mask==sen_mask) {
			    pmech_unit_sen_config->sen_config[k].pps_drv_value = config.led_brightness;
			    pmech_unit_sen_config->sen_config[k].pps_ref_value = config.compare_threshold;
			    pmech_unit_sen_config->sen_config[k].trigger_enable = config.trigger.enable;
			    pmech_unit_sen_config->sen_config[k].trigger_mode = config.trigger.mode;
			    break;
			}
		}
		if (k == pmech_unit_sen_config->sen_num) {
			return -1;
		}
		ret = photosensor_set_config(psen_data->sen_dev.pphotosensor, &config);

		if (ret) 
			return ret;
	    
	}
	return ret;
}
EXPORT_SYMBOL_GPL(sensor_set_config);
//----------------------get feature of sensor----------------------
int sensor_get_feature(mechanism_uint_sensor_data_t *punit_sensor_data, sen_feature_t *p_ppsfeature)
{
	struct sensor_data *psen_data;
	int ret;
	struct photosensor_feature feature;
	unsigned short j;

	// pr_debug("sensor_get_feature1\n");
	sensor_get_data(punit_sensor_data, p_ppsfeature->sen_mask, psen_data, j);
	if (j==punit_sensor_data->sensor_num) {
		return -RESN_MECH_ERR_SENSOR_GETDATA;
	}

	//pr_debug("sensor_get_feature2:sen_mask=%x sensor=%x\n", p_ppsfeature->sen_mask, psen_data->sen_dev.pphotosensor);
	if(!(ret = photosensor_get_feature(psen_data->sen_dev.pphotosensor, &feature)))
	{
		//pr_debug("sensor_get_feature3\n");
		pr_debug("sensor_get_feature3:sen_mask=%x sensor=%x\n", p_ppsfeature->sen_mask, (int)psen_data->sen_dev.pphotosensor);
		strcpy(p_ppsfeature->sen_name, psen_data->sen_name); 
		p_ppsfeature->led_brightness_max = feature.led_brightness_max;
		p_ppsfeature->raw_input_max = feature.raw_input_max;
		p_ppsfeature->input_scale_mv = feature.input_scale_mv; 
		p_ppsfeature->coverd_mode = feature.covered_mode;
		p_ppsfeature->calibrate_mode = feature.calibrate_mode;
	}
	return ret;
}
EXPORT_SYMBOL_GPL(sensor_get_feature);


