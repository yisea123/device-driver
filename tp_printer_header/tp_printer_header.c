/*
 * GWI Thermal Printer Printer Header Driver.
 *
 * Copyright 2018 Hunan GreatWall Information Financial Equipment Co., Ltd.
 *
 * Licensed under the GPL-2.
*/

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/string.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <linux/regmap.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>
#include <../arch/arm/mach-imx/hardware.h>
#include <linux/delay.h>

#include "tp_printer_header.h"
#include "printer_header_common.h"

#define STROBE_GPIO_NUM_MAX		4
#define GPIO_VALUE_HIGH 		1
#define GPIO_VALUE_LOW  		0

#define TP_PH_TIME_OF_HEATING_DEF	300		//us,默认加热时间
#define TP_PH_DOT_IN_LINE		1280		//一行点数
#define TP_PH_BUFFER_SIZE		(1280/8)	//点数据缓冲区
#define TP_PH_FREQ_HZ			16000000	//Hz
#define TP_PH_DELAY_AFTER_LATCH_LOW	1		//us
#define TP_PH_DELAY_AFTER_DATA_IN	1		//us
#define TP_PH_DELAY_AFTER_LATCH_HIGH	1		//us

#define to_tp_ph_dev(ptp_ph)	container_of(ptp_ph, struct tp_ph_dev_t, tp_ph)

const char str_stb[STROBE_GPIO_NUM_MAX][16] =
{
	"stb-1-gpio",
	"stb-2-gpio",
	"stb-3-gpio",
	"stb-4-gpio",
};

struct tp_ph_dev_t
{
	struct tp_ph_t tp_ph;
	struct spi_device *spi;		//printer header spi for write dot data
	int lat_gpio;
	int stb_gpio[STROBE_GPIO_NUM_MAX];
	int en_24v_gpio;
	int en_5v_gpio;
};

static int tp_ph_write_line(struct tp_ph_t * ptp_ph, unsigned char * pbuffer, unsigned int data_size)
{
	int ret = 0, i = 0;
	struct tp_ph_dev_t * ptp_ph_dev = to_tp_ph_dev(ptp_ph);
	struct spi_transfer xfer;
	struct spi_message msg;
	struct spi_device *spi;
	struct tp_ph_period_config_t * pperiod_config;
	int offset = 0;
	int line_size = 0;
	unsigned int buf_sum = 0;
	
	spi = ptp_ph_dev->spi;
	pperiod_config = &ptp_ph->config_data.period_config;
	//判断数据是否为空，为空则不打印直接退出
	for(i = 0; i < data_size; i++)
	{
		buf_sum |= pbuffer[i];
	}
	if(buf_sum == 0)
	{
		return 0;
	}
	//data through, latch low
	gpio_direction_output(ptp_ph_dev->lat_gpio, GPIO_VALUE_LOW);
	udelay(pperiod_config->delay_after_latch_low);
	//data in, spi write
	if(data_size > (ptp_ph->config_data.dots_in_a_line/8))	//超出打印范围,截断
	{
		ptp_ph->data_size = (ptp_ph->config_data.dots_in_a_line/8);
	}
	else
	{
		ptp_ph->data_size = data_size;
	}
	line_size = ptp_ph->config_data.dots_in_a_line/8;
	offset = line_size - ptp_ph->data_size;
	memset(ptp_ph->buffer, 0, line_size);
	memcpy(ptp_ph->buffer + offset, pbuffer, ptp_ph->data_size);
	memset(&xfer, 0, sizeof(xfer));
	xfer.len = line_size;//ptp_ph->data_size;//ptp_ph->config_data.dots_in_a_line/8;
	xfer.speed_hz = ptp_ph->config_data.period_config.clock_freq_hz;
	xfer.tx_buf = ptp_ph->buffer;
	xfer.rx_buf = NULL;
	xfer.bits_per_word = 8;
	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);
	ret = spi_sync(spi, &msg);
	if(ret)
	{
		printk("spi_sync error.\n");
		return ret;
	}
	udelay(pperiod_config->delay_after_data_in);
	//data hold, latch high
	gpio_direction_output(ptp_ph_dev->lat_gpio, GPIO_VALUE_HIGH);
	udelay(pperiod_config->delay_after_latch_high);
	for(i = 0; i < STROBE_GPIO_NUM_MAX; i++)
	{
		if(ptp_ph_dev->stb_gpio != NULL)
		{
			gpio_direction_output(ptp_ph_dev->stb_gpio[i], GPIO_VALUE_HIGH);
		}
	}
	udelay(ptp_ph->config_data.time_of_heating_us);
	//cooling, stb low
	for(i = 0; i < STROBE_GPIO_NUM_MAX; i++)
	{
		if(ptp_ph_dev->stb_gpio != NULL)
		{
			gpio_direction_output(ptp_ph_dev->stb_gpio[i], GPIO_VALUE_LOW);
		}
	}

	return ret;
}

static int tp_ph_config(struct tp_ph_t * ptp_ph, struct tp_ph_config_t * pconfig)
{
	int ret = 0;

	memcpy(&ptp_ph->config_data, pconfig, sizeof(struct tp_ph_config_t));
	return ret;
}

static int tp_ph_gpio_init_from_dts(struct device * dev, struct tp_ph_dev_t * p_tp_ph_dev)
{
	int ret;
	struct device_node * np = dev->of_node;
	int i = 0;
	
	p_tp_ph_dev->lat_gpio = of_get_named_gpio(np, "lat-gpio", 0);
	if (!gpio_is_valid(p_tp_ph_dev->lat_gpio)) 
	{
		dev_err(dev, "no lat-gpio pin available in dts.\n");
		goto __exit__;
	}
	ret = gpio_request(p_tp_ph_dev->lat_gpio, NULL);
	if(ret)
		return ret;
	
	p_tp_ph_dev->en_24v_gpio = of_get_named_gpio(np, "en-24v-gpio", 0);
	if (!gpio_is_valid(p_tp_ph_dev->en_24v_gpio)) 
	{
		dev_err(dev, "no en-24v-gpio pin available in dts.\n");
		goto __exit__;
	}
	ret = gpio_request(p_tp_ph_dev->en_24v_gpio, NULL);
	if(ret)
		return ret;

	
	p_tp_ph_dev->en_5v_gpio = of_get_named_gpio(np, "en-5v-gpio", 0);
	if (!gpio_is_valid(p_tp_ph_dev->en_5v_gpio)) 
	{
		dev_err(dev, "no en-5v-gpio pin available in dts.\n");
		goto __exit__;
	}
	ret = gpio_request(p_tp_ph_dev->en_5v_gpio, NULL);
	if(ret)
		return ret;

	for(i = 0; i < STROBE_GPIO_NUM_MAX; i++)
	{
		p_tp_ph_dev->stb_gpio[i] = of_get_named_gpio(np, str_stb[i], 0);
		if(!gpio_is_valid(p_tp_ph_dev->stb_gpio[i]))
		{
			dev_err(dev, "no stb-%d-gpio pin available in dts.\n", i);
			goto __exit__;
		}
		ret = gpio_request(p_tp_ph_dev->stb_gpio[i], NULL);
		if(ret)
		{
			dev_err(dev, "stb-%d-gpio gpio_request failed.\n", i);
			return ret;
		}
	}



__exit__:
	return 0;
}


static int tp_ph_init(struct tp_ph_dev_t * ptp_ph_dev)
{
	int ret = 0;
	int buf_size, i;
	struct tp_ph_period_config_t * pperiod_config;
	

	//申请存放数据缓冲区
	buf_size = ptp_ph_dev->tp_ph.config_data.dots_in_a_line;
	ptp_ph_dev->tp_ph.buffer = devm_kzalloc(ptp_ph_dev->tp_ph.dev, buf_size, GFP_KERNEL);
	ptp_ph_dev->tp_ph.data_size = 0;
	//init value
	ptp_ph_dev->tp_ph.config_data.dots_in_a_line = TP_PH_DOT_IN_LINE;
	ptp_ph_dev->tp_ph.config_data.time_of_heating_us = TP_PH_TIME_OF_HEATING_DEF;
	pperiod_config = &ptp_ph_dev->tp_ph.config_data.period_config;
	pperiod_config->clock_freq_hz = TP_PH_FREQ_HZ;
	pperiod_config->delay_after_latch_low = TP_PH_DELAY_AFTER_LATCH_LOW;
	pperiod_config->delay_after_data_in = TP_PH_DELAY_AFTER_DATA_IN;
	pperiod_config->delay_after_latch_high = TP_PH_DELAY_AFTER_LATCH_HIGH;

	gpio_direction_output(ptp_ph_dev->lat_gpio, GPIO_VALUE_HIGH);
	gpio_direction_output(ptp_ph_dev->en_24v_gpio, GPIO_VALUE_HIGH);
	gpio_direction_output(ptp_ph_dev->en_5v_gpio, GPIO_VALUE_HIGH);
	
	for(i = 0; i < STROBE_GPIO_NUM_MAX; i++)
	{
		if(ptp_ph_dev->stb_gpio != NULL)
		{
			gpio_direction_output(ptp_ph_dev->stb_gpio[i], GPIO_VALUE_LOW);
		}
	}
	
	return ret;
}

const struct tp_ph_ops_t tp_ph_ops =
{
	.config		= tp_ph_config,
	.write_line	= tp_ph_write_line,
};

static int tp_ph_probe(struct spi_device * spi)
{
	struct tp_ph_dev_t * tp_ph_dev;

	struct device_node * np = spi->dev.of_node;
	int ret = 0;
	unsigned short spi_mode = 0;
	unsigned int spi_max_speed_hz = TP_PH_FREQ_HZ;
	printk("tp printer header probe.\n");
	
	ret = of_property_read_u16(np, "spi-mode", &spi_mode);
	if(ret)
	{
		dev_err(&spi->dev, "Failed to read spi-mode from dts\n");
		return ret;
	}
	ret = of_property_read_u32(np, "spi-max-frequency", &spi_max_speed_hz);
	if(ret)
	{
		dev_err(&spi->dev, "Failed to read spi-max-frequency from dts\n");
		return ret;
	}
	spi->mode = SPI_MODE_3;
	spi->bits_per_word = 8;
	spi->max_speed_hz = spi_max_speed_hz;
	ret = spi_setup(spi);
	if (ret < 0)
	{
		printk(KERN_ERR "tp_ph_probe spi_setup error.\n");
		return ret;
	}
	
	tp_ph_dev = devm_kzalloc(&spi->dev, sizeof(*tp_ph_dev), GFP_KERNEL);
	if(tp_ph_dev == NULL)
	{
		dev_err(&spi->dev, "Failed to devm_kzalloc tp_ph_dev\n");
		return -ENOMEM;
	}
	spi_set_drvdata(spi, tp_ph_dev);
	
	tp_ph_dev->spi = spi;
	tp_ph_dev->tp_ph.dev = &spi->dev;
	tp_ph_dev->tp_ph.ops = &tp_ph_ops;
	
	ret = of_property_read_u16(np, "dots-in-line", &tp_ph_dev->tp_ph.config_data.dots_in_a_line);
	if(ret)
	{
		dev_err(&spi->dev, "Failed to read dots-in-line from dts\n");
		return ret;
	}
	
	ret = tp_ph_gpio_init_from_dts(&spi->dev, tp_ph_dev);
	if (ret)
	{
		dev_err(&spi->dev, "Failed to init gpio from dts: %d\n",ret);
		return ret;
	}
	
	ret = tp_ph_init(tp_ph_dev);
	if (ret)
	{
		dev_err(&spi->dev, "Init printer header config data fail: %d\n",ret);
		return ret;
	}
	ret = ph_add(&tp_ph_dev->tp_ph);
	if (ret < 0)
		return ret;
	spi_set_drvdata(spi, tp_ph_dev);
	return 0;
}

static int tp_ph_remove(struct spi_device * spi)
{
	struct tp_ph_dev_t * ptp_ph_dev;

	ptp_ph_dev = spi_get_drvdata(spi);
	printk("tp printer header remove.\n");
	return ph_remove(&ptp_ph_dev->tp_ph);
}

static const struct of_device_id tp_ph_match[] = {
	{ .compatible = "gwi,tp-printer-header", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, tp_ph_match);

static struct spi_driver tp_printer_header_driver = {
	.driver = {
		.name = "gwi,tp-printer-header",
		.of_match_table = tp_ph_match,
	},
	.probe = tp_ph_probe,
	.remove = tp_ph_remove,
};

module_spi_driver(tp_printer_header_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Dawei Shi");
MODULE_DESCRIPTION("Thermal Printer Header driver");