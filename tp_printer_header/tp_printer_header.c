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

#include "tp_printer_header.h"

#define STROBE_GPIO_NO_MAX	4
#define TP_PH_BUFFER_SIZE	(1280/8)
#define TP_PH_FREQ_HZ		16000000	//Hz
#define TP_PH_CLOCK_WIDTH	30		//ns
#define TP_PH_DATA_SETUP_TIME	10		//ns
#define TP_PH_DATA_HOLD_TIME	10		//ns
#define TP_PH_DATA_OUT_DELAY_TIME	90	//ns
#define TP_PH_LAT_WIDTH			100	//ns
#define TP_PH_LAT_HOLD_TIME		50	//ns
#define TP_PH_LAT_SETUP_TIME		200	//ns
#define TP_PH_STB_SETUP_TIME		200	//ns
#define TP_PH_DRV_OUT_DELAY_TIME	10	//us

const char str_stb[4][16] =
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
	int stb_gpio[STROBE_GPIO_NO_MAX];
};

static int tp_ph_write_data(struct device * dev, struct tp_ph_period_config_t * config)
{
	int ret = 0;
	return ret;
}

static int tp_ph_driver_out(struct device * dev, struct tp_ph_period_config_t * config)
{
	int ret = 0;
	return ret;
}

static int tp_ph_config(struct device * dev, struct tp_ph_period_config_t * config)
{
	int ret = 0;
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
	
	for(i = 0; i < STROBE_GPIO_NO_MAX; i++)
	{
		p_tp_ph_dev->stb_gpio[i] = of_get_named_gpio(np, str_stb[i], 0);
		if(!gpio_is_valid(p_tp_ph_dev->stb_gpio[i]))
		{
			dev_err(dev, "no stb-%d-gpio pin available in dts.\n", i);
			goto __exit__;
		}
		ret = gpio_request(p_tp_ph_dev->stb_gpio[i], NULL);
		if(ret)
			return ret;
	}

__exit__:
	return 0;
}

static int tp_ph_config_init(struct tp_ph_t * p_tp_ph)
{
	int ret = 0;
	int buf_size;
	
	buf_size = p_tp_ph->config_data.dots_in_a_line/8;
	p_tp_ph->config_data.buffer = devm_kzalloc(p_tp_ph->dev, buf_size, GFP_KERNEL);
	p_tp_ph->config_data.period_config.clock_freq_hz = TP_PH_FREQ_HZ;
	p_tp_ph->config_data.period_config.clock_width = TP_PH_CLOCK_WIDTH;
	p_tp_ph->config_data.period_config.data_hold_time = TP_PH_DATA_HOLD_TIME;
	p_tp_ph->config_data.period_config.data_setup_time = TP_PH_DATA_SETUP_TIME;
	p_tp_ph->config_data.period_config.data_out_delay_time = TP_PH_DATA_OUT_DELAY_TIME;
	p_tp_ph->config_data.period_config.lat_hold_time = TP_PH_LAT_HOLD_TIME;
	p_tp_ph->config_data.period_config.lat_width = TP_PH_LAT_WIDTH;
	p_tp_ph->config_data.period_config.lat_setup_time = TP_PH_LAT_SETUP_TIME;
	p_tp_ph->config_data.period_config.stb_setup_time = TP_PH_STB_SETUP_TIME;
	p_tp_ph->config_data.period_config.drv_out_delay_time = TP_PH_DRV_OUT_DELAY_TIME;
	return ret;
}

const struct tp_ph_ops_t tp_ph_ops =
{
	.config		= tp_ph_config,
	.write_data	= tp_ph_write_data,
	.driver_out	= tp_ph_driver_out,
};

static int tp_ph_probe(struct spi_device * spi)
{
	struct tp_ph_dev_t * tp_ph_dev;
	struct device_node * np = spi->dev.of_node;
	int ret = 0;
	unsigned short spi_mode = 0;
	unsigned int spi_max_speed_hz = 16000000;
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
	spi->mode = spi_mode;
	spi->bits_per_word = 16;
	spi->max_speed_hz = spi_max_speed_hz;
	ret = spi_setup(spi);
	if (ret < 0)
		return ret;
	
	tp_ph_dev = devm_kzalloc(&spi->dev, sizeof(*tp_ph_dev), GFP_KERNEL);
	if(tp_ph_dev == NULL)
		return -ENOMEM;
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
	
	ret = tp_ph_config_init(&tp_ph_dev->tp_ph);
	if (ret)
	{
		dev_err(&spi->dev, "Init printer header config data fail: %d\n",ret);
		return ret;
	}
	
	return 0;
}

static int tp_ph_remove(struct spi_device * spi)
{
	return 0;
}

static const struct of_device_id tp_ph_match[] = {
	{ .compatible = "gwi,tp-printer-header", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, tp_ph_match);

static struct spi_driver tp_printer_header_driver = {
	.probe = tp_ph_probe,
	.remove = tp_ph_remove,
	.driver = {
		.name = "tp_printer_header",
		.owner = THIS_MODULE,
		.of_match_table = tp_ph_match,
	},
};

module_spi_driver(tp_printer_header_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Dawei Shi");
MODULE_DESCRIPTION("Thermal Printer Header driver");