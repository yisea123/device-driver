/*
 * GWI FPGA-controlled Image Analog Front End (AFE) driver
 *
 * Copyright 2016 Hunan GreatWall Information Financial Equipment Co., Ltd.
 *
 * Licensed under the GPL-2.
 */
#include <linux/module.h>
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

#include "imagedigitiser.h"
#define WM8215_REG_IMGADC_CONTROL 0x000001
#define WM8215_REG_IMGADC_CONTROL_MASK BIT(0)
#define SEN1_GPIO_ID  106
#define SEN2_GPIO_ID  97
#define GPIO_VALUE_HIGH   1
#define GPIO_VALUE_LOW    0

struct afe_dev {
	struct imagedigitiser afe; 
	struct gpio wm8215_config_sen;
	struct spi_device *spi;
};


static const struct regmap_config wm8215_regmap_config = {
	.reg_bits = 6,
	.val_bits = 8,
};

#define to_afe_dev(afe)	container_of(afe, struct afe_dev, afe)

static int spi_wm8215_enable(struct imagedigitiser *afe)
{
	int rs;
//	struct afe_dev *afedev = to_afe_dev(afe);
	if (!afe)
		return -EINVAL;

	rs = 0;//regmap_update_bits(afedev->spi_reg, WM8215_REG_IMGADC_CONTROL, WM8215_REG_IMGADC_CONTROL_MASK, WM8215_REG_IMGADC_CONTROL_MASK);
	return rs;
}


static void spi_wm8215_disable(struct imagedigitiser *afe)
{
//	struct afe_dev *afedev = to_afe_dev(afe);
	if (!afe)
		return;

//	regmap_update_bits(afedev->spi_reg, WM8215_REG_IMGADC_CONTROL, WM8215_REG_IMGADC_CONTROL_MASK, 0);
}

int gpio_init_from_dts(struct device *dev, struct gpio *wm8215_config_sen)
{
	int ret;
	struct device_node *np = dev->of_node; 

	printk("gpio_init_from_dts enter!\n");
	wm8215_config_sen->gpio = of_get_named_gpio(np, "config_sen_gpio", 0);
	if (!wm8215_config_sen->gpio) {
		dev_err(dev, "no config_sen_gpio pin available\n");
		goto err;
	}
	wm8215_config_sen->flags = GPIOF_OUT_INIT_LOW;
	wm8215_config_sen->label = "config_sen_gpio";
	printk("gpio_init_from_dts: gpio_request_one=%d!\n", wm8215_config_sen->gpio);
	ret = gpio_request_one(wm8215_config_sen->gpio, wm8215_config_sen->flags, wm8215_config_sen->label);
	printk("gpio_request_one: ret=%d!\n", ret);
	if(ret)
	   return ret; 

	gpio_direction_output(wm8215_config_sen->gpio, GPIOF_OUT_INIT_LOW); 

err:
	return 0;
}

static int wm8215_spi_write(struct spi_device *spi, struct gpio wm8215_config_sen, u8 addr,
                            int bytes, u8 data)
{
	int i, ret;
	/* bit 14 is the read/write bit */
	unsigned short spi_data = 0 << 12 | addr << 8 | data;
	struct spi_transfer xfer;
	struct spi_message msg;
	u32 tx_buf;

	gpio_set_value(wm8215_config_sen.gpio, GPIO_VALUE_LOW);

//	printk("wm8215_spi_write enter, spi_data=%x\n", spi_data);
	tx_buf = spi_data;

	memset(&xfer, 0, sizeof(xfer));
	xfer.tx_buf = &tx_buf;
	xfer.rx_buf = NULL;
	xfer.len    = sizeof(unsigned short);
	xfer.bits_per_word = 14;
//	xfer.speed_hz = 10000000;
//	printk("spi=%x, spi->maxspeedhz=%d, xfer.speedhz=%d\n ", spi, spi->max_speed_hz, xfer.speed_hz);	

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

	ret = spi_sync(spi, &msg);

	gpio_set_value(wm8215_config_sen.gpio, GPIO_VALUE_HIGH);
	for(i = 0; i < 5; i++);
	gpio_set_value(wm8215_config_sen.gpio, GPIO_VALUE_LOW);

	return ret;
}

static int wm8215_spi_read(struct spi_device *spi, struct gpio wm8215_config_sen, u8 addr,
                            int bytes, u32 *data)
{
	/* bit 14 is the read/write bit */
	unsigned short spi_data = 1 << 12 | addr << 8;
	struct spi_transfer xfer;
	struct spi_message msg;
	int ret, i;
	u32 tx_buf, rx_buf;

//	printk("wm8215_spi_read enter, spi_data=%x\n", spi_data);

	tx_buf = spi_data;
	rx_buf = 0;

	gpio_set_value(wm8215_config_sen.gpio, GPIO_VALUE_LOW);
	memset(&xfer, 0, sizeof(xfer));
	xfer.tx_buf = &tx_buf;
	xfer.rx_buf = &rx_buf;
	xfer.len    = sizeof(unsigned short);
	xfer.bits_per_word = 14;

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

	if (spi == NULL)
		return 0;

	ret = spi_sync(spi, &msg);

	gpio_set_value(wm8215_config_sen.gpio, GPIO_VALUE_HIGH);
	for(i = 0; i < 5; i++);
	gpio_set_value(wm8215_config_sen.gpio, GPIO_VALUE_LOW);

	ret = spi_sync(spi, &msg);
	if (ret == 0)
		*data = (u8) (rx_buf & 0xFF);
	return ret;
}

static int spi_wm8215_get_config(struct imagedigitiser *afe, struct scanunit_config *config)
{
	struct afe_dev *afedev = to_afe_dev(afe);
	struct scan_reg_config *regconfig;
	int i;

	if (!afe || !config)
		return -EINVAL;
	if (config->regcount <= 0 || !config->regconfig) 
		return -EINVAL;

/*	for (i=0; i<config->regcount; i++) {
		printk("wm8215config->regconfig[%d].value=%x, address=%x\n", i, config->regconfig[i].value, config->regconfig[i].address);
	}*/

	regconfig = config->regconfig;
	for (i=0; i<config->regcount; i++) {
		u32 value, mask = regconfig[i].mask;

		wm8215_spi_read(afedev->spi, afedev->wm8215_config_sen, regconfig[i].address, 1, &value);

		if (mask != 0)
			value &= mask;
		regconfig[i].value = value;
	}
	return 0;
}


static int spi_wm8215_set_config(struct imagedigitiser *afe, const struct scanunit_config *config)
{
	struct afe_dev *afedev = to_afe_dev(afe);
	struct scan_reg_config *regconfig;
	int i;

	if (!afe || !config)
		return -EINVAL;
	if (config->regcount <= 0 || !config->regconfig) 
		return -EINVAL;

	regconfig = config->regconfig;
	for (i=0; i<config->regcount; i++) {
//		regmap_update_bits(afedev->spi_reg, regconfig[i].address, regconfig[i].mask, regconfig[i].value);
		wm8215_spi_write(afedev->spi, afedev->wm8215_config_sen, regconfig[i].address, 1, regconfig[i].value);
	}
	return 0;
}


static int spi_wm8215_probe(struct spi_device *spi)
{
	struct afe_dev *afedev;
	int ret = 0;

	printk("spi_wm8215_probe enter!\n");

	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 14;
	spi->max_speed_hz = 10000000;
	ret = spi_setup(spi);
	if (ret < 0) 
		return ret;

	afedev = devm_kzalloc(&spi->dev, sizeof(*afedev), GFP_KERNEL);
	if (afedev == NULL)
		return -ENOMEM;
	spi_set_drvdata(spi, afedev);

	afedev->spi = spi;
	afedev->afe.dev = &spi->dev;
	afedev->afe.enable = spi_wm8215_enable;
	afedev->afe.disable = spi_wm8215_disable;
	afedev->afe.get_config = spi_wm8215_get_config;
	afedev->afe.set_config = spi_wm8215_set_config;

	ret = imagedigitiser_add(&afedev->afe);
	if (ret < 0)
		return ret;
	ret =  gpio_init_from_dts(&spi->dev, &afedev->wm8215_config_sen); 
	if (ret)
	{
		dev_err(&spi->dev, "Failed to init gpio from dts: %d\n",ret);
		return ret;
	}
	return 0;
}

static int spi_wm8215_remove(struct spi_device *spi)
{
	struct afe_dev *afedev = spi_get_drvdata(spi);

	imagedigitiser_remove(&afedev->afe);
	return 0;
}


static const struct of_device_id spi_wm8215_match[] = {
	{ .compatible = "gwi,spi-wm8215", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, spi_wm8215_match);

static struct spi_driver spi_wm8215_driver = {
	.driver		= {
		.name	= "gwi,spi-wm8215",
		.of_match_table = spi_wm8215_match,
	},
	.probe		= spi_wm8215_probe,
	.remove		= spi_wm8215_remove,
};

module_spi_driver(spi_wm8215_driver);

MODULE_AUTHOR("Zhang Xudong <zhangxudong@gwi.com.cn>");
MODULE_DESCRIPTION("GWI SPI-controlled Image Analog Front End (AFE) driver");
MODULE_LICENSE("GPL v2");
