/*
 * Analog and Digital Photo Sensor driver
 *
 * Copyright 2016 Hunan GreatWall Information Financial Equipment Co., Ltd.
 *
 * Licensed under the GPL-2.
 */
#include <linux/err.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/pwm.h>
#include <linux/iio/iio.h>
#include <linux/iio/types.h>
#include <linux/iio/consumer.h>
#include <linux/kern_levels.h>
#include <linux/interrupt.h>
#include <linux/delay.h>


static const struct of_device_id photosensor_match[] = {
	{ .compatible = "gwi,photosensor", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, photosensor_match);

#if 1
#include "iio_extra.h"
#include "dispenser_photosensor.h"


static DEFINE_MUTEX(photosensor_lock);
static LIST_HEAD(photosensor_list);


struct sensor_dev {
	struct photosensor sensor;
	struct pwm_device *led_pwm;
	struct iio_channel *adc_chan;
	int input_gpio;
        int led_gpio;
};

#define to_sensor_dev(sensor)	container_of(sensor, struct sensor_dev, sensor)


#define PWM_TO_BRIGHTNESS(duty,period)		(((duty)*(int)MAXIMUM_BRIGHTNESS)/(period))
#define BRIGHTNESS_TO_DUTY(brightness,period)	(((brightness)*(period)/(int)MAXIMUM_BRIGHTNESS))

#define PHOTOSENSOR_CS		(2)

struct photosensor *of_photosensor_get(struct device_node *np)
{
	return ERR_PTR(-ENODEV);
}
EXPORT_SYMBOL_GPL(of_photosensor_get);


struct photosensor *photosensor_get(struct device *dev)
{
	return ERR_PTR(-ENODEV);
}
EXPORT_SYMBOL_GPL(photosensor_get);

struct photosensor *of_node_to_photosensor(struct device_node *np)
{
	struct photosensor *sensor;

	mutex_lock(&photosensor_lock);

	list_for_each_entry(sensor, &photosensor_list, list)
		if (sensor->dev && sensor->dev->of_node == np) {
			mutex_unlock(&photosensor_lock);
			return sensor;
		}

	mutex_unlock(&photosensor_lock);

	return ERR_PTR(-EPROBE_DEFER);
}
EXPORT_SYMBOL_GPL(of_node_to_photosensor);


void photosensor_put(struct photosensor *sensor)
{
}
EXPORT_SYMBOL_GPL(photosensor_put);


int photosensor_enable(struct photosensor *sensor)
{
	struct sensor_dev *sensordev = to_sensor_dev(sensor);
        int period_ns=0, duty_ns=0;
        int err;
	//pr_debug("photosensor_enable\n");
	if (!sensor)
		return -EINVAL;
        if (sensordev->led_gpio) {
            err = gpio_direction_output(sensordev->led_gpio, GPIOF_OUT_INIT_HIGH);
        }else if(sensordev->led_pwm)
        {
            pwm_enable(sensordev->led_pwm);
        }
 	return 0;
}
EXPORT_SYMBOL_GPL(photosensor_enable);


int photosensor_disable(struct photosensor *sensor)
{
	struct sensor_dev *sensordev = to_sensor_dev(sensor);
        int period_ns=0, duty_ns=0;
	//pr_debug("photosensor_disable\n");
	if (!sensor) 
		return -EINVAL;
	// stop LED PWM output of a sensor
	//pr_debug("photosensor_disable1\n");

        pwm_disable(sensordev->led_pwm);

	return 0;
}
EXPORT_SYMBOL_GPL(photosensor_disable);

/*获取传感器的状态值*/
int photosensor_status(struct photosensor *sensor, int *status)
{
	struct sensor_dev *sensordev = to_sensor_dev(sensor);
	int rs = 0;


	if (!sensor)
		return -EINVAL;
	if (sensordev->sensor.type == PHOTOSENSOR_DIGITAL)
	{
		*status = gpio_get_value(sensordev->input_gpio);
	}
	else {
		int ret, tmp, val;
                u32 threshold = sensor->threshold;

                ret = iio_read_channel_raw(sensordev->adc_chan, &val);
		if (IS_ERR_VALUE(ret))
			rs = ret;
                if (val >= threshold) {
                    *status = 1;
                }
                else{
                    *status = 0;
                }
                //ret = iio_read_event_value(sensordev->adc_chan, IIO_EV_TYPE_CHANGE, IIO_EV_DIR_EITHER, IIO_EV_INFO_VALUE, status, &tmp);
		//if (IS_ERR_VALUE(ret))
		//	rs = ret;
	}
		
	if (sensor->sensor_polarity == PHOTOSENSOR_0MEANS_DETECTED) {
		*status = (!(*status))?1:0;
	}
	return rs;
}
EXPORT_SYMBOL_GPL(photosensor_status);


/** 获取传感器原始数据值
 * photosensor_read_input() - read raw input value from a photosensor
 * @sensor: photosensor
 */
int photosensor_read_input(struct photosensor *sensor, unsigned long *val)
{
	#if 0

	#else	//modify by hl 2017.1.17
	struct sensor_dev *sensordev = to_sensor_dev(sensor);
	int rs = 0;
	if (!sensor)
		return -EINVAL;
	if (sensor->type == PHOTOSENSOR_DIGITAL) {
		*val = gpio_get_value(sensordev->input_gpio);
	}
	else
	{
		int ret;
		ret = iio_read_channel_raw(sensordev->adc_chan, (int *)val);
		if (IS_ERR_VALUE(ret))
			rs = ret;
	}

        return rs;
	#endif
}
EXPORT_SYMBOL_GPL(photosensor_read_input);


/**
 * photosensor_get_feature() - get feature of a photosensor
 * @sensor: photosensor
 */
int photosensor_get_feature(struct photosensor *sensor, struct photosensor_feature *feature)
{
	struct sensor_dev *sensordev = to_sensor_dev(sensor);

        printk(KERN_DEBUG "photosensor_get_feature start\n");
	if (!sensor || !feature)
	{
		pr_debug("photosensor_get_feature err=EINVAL\n");
		return -EINVAL;
	}

	feature->led_brightness_max = MAXIMUM_BRIGHTNESS;

	if (sensor->type == PHOTOSENSOR_DIGITAL) {
		feature->raw_input_max = 1;
		feature->input_scale_mv = 3300;		/* scale = 3.3V */
	}
	else
	{
		int rs, val;
                printk(KERN_DEBUG "iio_convert_raw_to_processed start \n");
		rs = iio_convert_raw_to_processed(sensordev->adc_chan, 1, &val, 1000);	// convert raw value 1 to voltage (mV)
                printk(KERN_DEBUG "iio_convert_raw_to_processed end -----and val = %d\n", val);
		if (IS_ERR_VALUE(rs))
		{
			pr_debug("photosensor_get_feature iio_convert_raw_to_processed=%x\n", rs);
			return rs;
		}
        
		feature->raw_input_max = 0xfff;	// TODO: should get maximum value from IIO functions
		feature->input_scale_mv = val;
	}
	#if 0   //add by hl 2016.11.29

	#else
	if (sensor->sensor_polarity == PHOTOSENSOR_0MEANS_DETECTED)
	{
		feature->covered_mode = 1;
	}
	else{
		feature->covered_mode = 0;
	}

	if (sensor->sensor_mode == PHOTOSENSOR_THROUGHBEAM) {
		feature->calibrate_mode = 0; 
	}
	else{
		feature->calibrate_mode = 1;
	}
	#endif
return 0;
}
EXPORT_SYMBOL_GPL(photosensor_get_feature);


int photosensor_get_config(struct photosensor *sensor, struct photosensor_config *config)
{
	struct sensor_dev *sensordev = to_sensor_dev(sensor);

	if (!sensor || !config)
		return -EINVAL;
	#if 0   
	memset((void *)sensor, 0, sizeof(*config));
	#else   //modify by hl 2016.11.4
	memset((void *)config, 0, sizeof(*config));
	#endif

	if (sensor->type == PHOTOSENSOR_DIGITAL) {
	}
	else
	{
                config->led_brightness = PWM_TO_BRIGHTNESS(pwm_get_duty_cycle(sensordev->led_pwm), pwm_get_period(sensordev->led_pwm)); 
                config->compare_threshold = sensor->threshold;
                #if 0
		int ret, val, val2;
		/* get ADC comparision threshold value */
		ret = iio_read_event_value(sensordev->adc_chan, IIO_EV_TYPE_THRESH, IIO_EV_DIR_EITHER, IIO_EV_INFO_VALUE, &val, &val2);
		if (IS_ERR_VALUE(ret))
			return ret;
		config->compare_threshold = val;
		/* get trigger enable/disabe state */
		ret = iio_read_event_config(sensordev->adc_chan, IIO_EV_TYPE_THRESH, IIO_EV_DIR_EITHER);
		if (IS_ERR_VALUE(ret))
			return ret;
		config->trigger.enable = ret;
		/* get compare mode */
		ret = iio_read_event_value(sensordev->adc_chan, IIO_EV_TYPE_CHANGE, IIO_EV_DIR_NONE, IIO_EV_INFO_VALUE, &val, &val2);
		if (IS_ERR_VALUE(ret))
			return ret;
		config->trigger.mode = val;
                #endif

	}
        
	return 0;
}
EXPORT_SYMBOL_GPL(photosensor_get_config);


int photosensor_set_config(struct photosensor *sensor, const struct photosensor_config *config)
{
	int duty, period;
	struct sensor_dev *sensordev = to_sensor_dev(sensor);

	if (!sensor || !config)
		return -EINVAL;


	//pr_debug("photosensor_set_config:duty=%d, period=%d type=%d sensor=%x\n", duty, period, sensor->type, sensor);
	/* set input configuration */
	if (sensor->type == PHOTOSENSOR_DIGITAL) {
	}
	else
	{
		int ret, val, dir;

        	/* set LED PWM configuration */
        	period = pwm_get_period(sensordev->led_pwm);

                if (config->led_brightness > MAXIMUM_BRIGHTNESS) {
                    return 0;
                }
                duty = BRIGHTNESS_TO_DUTY(config->led_brightness, period); 
        	pwm_config(sensordev->led_pwm, duty, period);

		//val = config->compare_threshold;
                sensor->threshold = config->compare_threshold;
                /*
		ret = iio_write_event_value(sensordev->adc_chan, IIO_EV_TYPE_THRESH, IIO_EV_DIR_EITHER, IIO_EV_INFO_VALUE, val, 0);
		if (IS_ERR_VALUE(ret))
			return ret;
		dir = config->trigger.mode;

		//ret = iio_write_event_config(sensordev->adc_chan, IIO_EV_TYPE_THRESH, dir, config->trigger.enable);
		ret = iio_write_event_config(sensordev->adc_chan, IIO_EV_TYPE_CHANGE, dir, config->trigger.enable); //modify by hl 2016.11.2
		if (IS_ERR_VALUE(ret))
			return ret;
                */

	}

	return 0;
}
EXPORT_SYMBOL_GPL(photosensor_set_config);
#endif

#if 1   //del by lxh 2018.05.15
static irqreturn_t sen_isr(int irq, void *dev_id)
{
	struct sensor_dev *sensordev;
	struct photosensor *sensor;
	unsigned long val;
	int i;
	
	sensordev = (struct sensor_dev *)dev_id;
	sensor = &sensordev->sensor;
	val = gpio_get_value(sensordev->input_gpio);
	for (i = 0; i < 5; i++) {
		if (val^gpio_get_value(sensordev->input_gpio))
		{
			return IRQ_HANDLED;
		}
		ndelay(10);
	}
	printk("#####neter sen_isr \n");
	if (sensor->callback)
		sensor->callback(sensor, &sensor->sensor_callback_data);

	return IRQ_HANDLED;
}

#endif

int photosensor_get_trigger_next(struct photosensor *sensor, struct sensor_trigger *trigger)
{
	struct sensor_dev *sensordev = to_sensor_dev(sensor);
	if (!sensor)
		return -EINVAL;
	if (sensor->type == PHOTOSENSOR_DIGITAL) {
		/*void __iomem *fpga_regs = NULL;
		u32 val;

		fpga_regs = fpga_io_get(PHOTOSENSOR_CS) + FPGA_REG_SENSOR_DC_SENSOR_MASK;
		fpga_readl(&val, fpga_regs);
            
		if (val &(1<<sensor->dig_bit_index)){
			trigger->enable = 0;
		}
		else
			trigger->enable = 1;

		fpga_regs = fpga_io_get(PHOTOSENSOR_CS) + FPGA_REG_SENSOR_DC_COMPARE_MODE;
		fpga_readl(&val, fpga_regs);
		if (val & (1<<sensor->dig_bit_index)) {
			trigger->mode = 1;
		}
		else
			trigger->mode = 0;*/
	}
	else
	{
		int ret, val, val2;
		/* get next trigger enable/disabe state */
		ret = iio_read_event_config(sensordev->adc_chan, IIO_EV_TYPE_CHANGE, IIO_EV_DIR_EITHER);
		if (IS_ERR_VALUE(ret))
			return ret;
		trigger->enable = ret;
		/* get next compare mode */
		ret = iio_read_event_value(sensordev->adc_chan, IIO_EV_TYPE_CHANGE, IIO_EV_DIR_NONE, IIO_EV_INFO_VALUE, &val, &val2);
		if (IS_ERR_VALUE(ret))
			return ret;
		trigger->mode = val2;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(photosensor_get_trigger_next);


int photosensor_set_trigger_next(struct photosensor *sensor, const struct sensor_trigger *trigger)
{
	struct sensor_dev *sensordev = to_sensor_dev(sensor);
	int ret, dir;

	if (!sensor)
		return -EINVAL;
	
	if (sensor->type == PHOTOSENSOR_DIGITAL) {
		/*void __iomem *fpga_regs = NULL;
		u32 val;

		fpga_regs = fpga_io_get(PHOTOSENSOR_CS) + FPGA_REG_SENSOR_DC_SENSOR_MASK;
		//fpga_readl(&val, fpga_regs);
            
		val = 0;
		if (trigger->enable) {
			val |= (1<<sensor->dig_bit_index);
		}
		else
			val &= ~(1<<sensor->dig_bit_index);
		fpga_writel(val, fpga_regs);
		//printk("photosensor_set_trigger_next:mask---------addr=%x val=%x\n",fpga_regs, val);

		fpga_regs = fpga_io_get(PHOTOSENSOR_CS) + FPGA_REG_SENSOR_DC_COMPARE_MODE;
		//fpga_readl(&val, fpga_regs);
		val = 0;
		if (trigger->mode) {
			val |= (1<<sensor->dig_bit_index);
		}
		else
			val &= ~(1<<sensor->dig_bit_index);
		fpga_writel(val, fpga_regs);*/
		//printk("photosensor_set_trigger_next:mode------addr=%x val=%x\n",fpga_regs, val);
	}
	else
	{
		dir = trigger->mode;
		ret = iio_write_event_config(sensordev->adc_chan, IIO_EV_TYPE_CHANGE, dir, trigger->enable);
		if (IS_ERR_VALUE(ret))
			return ret;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(photosensor_set_trigger_next);

int photosensor_clear_trigger_next(struct photosensor *sensor)
{
	struct sensor_dev *sensordev = to_sensor_dev(sensor);
	int ret, dir;

	if (!sensor)
		return -EINVAL;

	if (sensor->type == PHOTOSENSOR_DIGITAL) {
		/*void __iomem *fpga_regs = NULL;
		u32 val;

		fpga_regs = fpga_io_get(PHOTOSENSOR_CS) + FPGA_REG_SENSOR_DC_SENSOR_MASK;
		fpga_readl(&val, fpga_regs);
            
		//val &= ~(1<<sensor->dig_bit_index);
		val = 0;
		fpga_writel(val, fpga_regs);
		pr_debug("photosensor_clear_trigger_next:mask---------addr=%x val=%x\n",fpga_regs, val);

		fpga_regs = fpga_io_get(PHOTOSENSOR_CS) + FPGA_REG_SENSOR_DC_COMPARE_MODE;
		fpga_readl(&val, fpga_regs);

		//val &= ~(1<<sensor->dig_bit_index);
		val = 0;
		fpga_writel(val, fpga_regs);
		pr_debug("photosensor_clear_trigger_next:mode------addr=%x val=%x\n",fpga_regs, val);*/
	}
	else
	{
		ret = iio_write_event_config(sensordev->adc_chan, IIO_EV_TYPE_CHANGE, IIO_EV_DIR_FALLING, 0);
		if (IS_ERR_VALUE(ret))
			return ret;

	}
	return 0;
}
EXPORT_SYMBOL_GPL(photosensor_clear_trigger_next);

struct pwm_device *pled_pwm_ext; 
int gpio_ext;
static int photosensor_probe(struct platform_device *pdev)
{
	struct sensor_dev *sensordev;
	struct device_node *node = pdev->dev.of_node;
	int ret;
	int sen_irq = 0;


#if 1
	sensordev = devm_kzalloc(&pdev->dev, sizeof(*sensordev), GFP_KERNEL);
	if (sensordev == NULL)
		return -ENOMEM;

	sensordev->sensor.dev = &pdev->dev;

	/* parse the device node */
	/* determine photosensor type from DT property */
	ret = of_property_match_string(node, "type", "digital");
	if (ret >= 0){
		printk(KERN_DEBUG "digital:%x\n", sensordev->sensor);
		sensordev->sensor.type = PHOTOSENSOR_DIGITAL;
	} else 
	{
		ret = of_property_match_string(node, "type", "analog");
		if (ret >= 0){
			printk(KERN_DEBUG "analog:%x\n", sensordev->sensor);
			sensordev->sensor.type = PHOTOSENSOR_ANALOG;
                } else{
                    printk(KERN_ERR "error = of_property_match_string-----type\n");
                    return -EINVAL;
                }
		
	}

#if 1   //add by hl 2016.11.29
	ret = of_property_match_string(node, "mode", "throughbeam");
	if (ret >= 0){
		printk(KERN_DEBUG "throughbeam:%x\n", sensordev->sensor);
		sensordev->sensor.sensor_mode = PHOTOSENSOR_THROUGHBEAM; 
	} else 
	{
		ret = of_property_match_string(node, "mode", "reflective");
		if (ret >= 0){
			printk(KERN_DEBUG, "reflective:%x\n", sensordev->sensor);
		sensordev->sensor.sensor_mode = PHOTOSENSOR_REFLECTIVE; 
                } else{
                    printk(KERN_ERR "error = of_property_match_string----mode\n");
                    return -EINVAL;
                }
		
	}

	ret = of_property_match_string(node, "app-polarity", "1");
	if (ret >= 0){
		printk(KERN_DEBUG "app-polarity:%x\n", sensordev->sensor);
		sensordev->sensor.sensor_polarity = PHOTOSENSOR_1MEANS_DETECTED; 
	} else 
	{
		ret = of_property_match_string(node, "app-polarity", "0");
		if (ret >= 0){
			printk(KERN_DEBUG "reflective:%x\n", sensordev->sensor);
                        sensordev->sensor.sensor_polarity = PHOTOSENSOR_0MEANS_DETECTED; 
                } else{
                        printk(KERN_ERR "error = of_property_match_string------ app-polarity");
                        return -EINVAL;
                }
	}

	ret = of_property_match_string(node, "led-contrl", "pwm");
        if (ret >= 0) {

                sensordev->sensor.led_contrl = PHOTOSENSOR_LEDCONTRL_PWM;
        } else
        {
                ret = of_property_match_string(node, "led-contrl", "gpio");
                if (ret >= 0) {
                    sensordev->sensor.led_contrl = PHOTOSENSOR_LEDCONTRL_GPIO;

                } else{
                    printk(KERN_ERR "error = of_property_match_string------ led-contrl");
                    return -EINVAL;
                }
                
        }
#endif


        if (sensordev->sensor.led_contrl == PHOTOSENSOR_LEDCONTRL_PWM) {
            int period_ns=0, duty_ns=0;
            /* get sensor LED PWM information from DT property led-contrl = "pwmcontrl";*/
            sensordev->led_pwm = devm_pwm_get(&pdev->dev, NULL);
            if (IS_ERR(sensordev->led_pwm)){
                ret = PTR_ERR(sensordev->led_pwm);

                sensordev->led_pwm = pled_pwm_ext;
                //printk("replace sensordev->led_pwm- with other pwms\n");
                //return -ENODEV;
            }
            else{
                 pled_pwm_ext = sensordev->led_pwm;
                 //printk("save sensordev->led_pwm\n");    
            }

            period_ns = pwm_get_period(sensordev->led_pwm);
            duty_ns = (period_ns*9/10);
            pwm_config(sensordev->led_pwm, duty_ns, period_ns);

        }
        else if (sensordev->sensor.led_contrl == PHOTOSENSOR_LEDCONTRL_GPIO) {

            int led_gpio;

            //led_gpio = of_get_gpio(node, 0);
            led_gpio = of_get_named_gpio(node, "led-gpio", 0); 
            if (!gpio_is_valid(led_gpio)) {
                    printk(KERN_ERR "\n of_get_gpio (0) is not valid %d.\n", led_gpio);
                    return ret;
            }
            ret = gpio_request(led_gpio, "led-gpio");
            if (IS_ERR_VALUE(ret)) {
                    //printk("\n gpio_request (0) failure with code %d.\n", ret);
                    //return ret;
                    led_gpio = gpio_ext;
            }else{
                gpio_ext = led_gpio;
            }

            sensordev->led_gpio = led_gpio;

        }
        else{
            return -EINVAL;
        }

	/* get sensor input information from DT property */
	if (sensordev->sensor.type == PHOTOSENSOR_DIGITAL)
	{	// request GPIO for digital sensor

                int gpio;
                /****************************************************************************************************/

		gpio = of_get_gpio(node, 0); 
		if (!gpio_is_valid(gpio)) {
			printk(KERN_ERR "\n of_get_gpio (1) is not valid %d.\n", gpio);
			return ret;
		}
		sensordev->input_gpio = gpio;

		ret = gpio_request(gpio, "gpio");
		if (IS_ERR_VALUE(ret)) {
			printk(KERN_ERR "\n gpio_request (1) failure with code %d.\n", ret);
			return ret;
		}
		sen_irq = gpio_to_irq(gpio);
		if (sen_irq < 0) {
			printk("gpio_to_irq failed!\n");
		}
		
		ret = request_irq(sen_irq, sen_isr, IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING, "sen_irq", sensordev);

		of_property_read_u32(node, "bit-index", &sensordev->sensor.dig_bit_index); 

	}
	else
	{	
                // request ADC IIO channel for analog sensor
		struct iio_channel *iiochan;
		iiochan = iio_channel_get(&pdev->dev, NULL);
		if (IS_ERR(iiochan)) {
			ret = (int)iiochan;
			printk(KERN_ERR "iio_channel_get failure with code %d.\n", ret);
			return ret;
		}
		sensordev->adc_chan = iiochan;
	}



	mutex_lock(&photosensor_lock);

	INIT_LIST_HEAD(&sensordev->sensor.list);
	list_add(&sensordev->sensor.list, &photosensor_list);

	mutex_unlock(&photosensor_lock);

	platform_set_drvdata(pdev, sensordev);
#endif
 
        return 0; 
}


static int photosensor_remove(struct platform_device *pdev)
{
	struct sensor_dev *sensordev;

	sensordev = platform_get_drvdata(pdev);

	if (sensordev->adc_chan)
		iio_channel_release(sensordev->adc_chan); 
 
	return 0; 
}



static struct platform_driver photosensor_driver = {
	.driver         = {
		.name   = "photosensor",
		.of_match_table = photosensor_match,
	},
	.probe          = photosensor_probe,
	.remove         = photosensor_remove,
};

module_platform_driver(photosensor_driver);

MODULE_AUTHOR("Zhang Xudong <zhangxudong@gwi.com.cn>");
MODULE_DESCRIPTION("Analog and Digital Photo Sensor driver");
MODULE_LICENSE("GPL v2");
