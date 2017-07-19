/*
 * GWI FPGA-controlled ADC driver
 *
 * Copyright 2016 Hunan GreatWall Information Financial Equipment Co., Ltd.
 *
 * Licensed under the GPL-2.
 */

#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/bitops.h>
#include <linux/platform_device.h>
#include <linux/iio/iio.h>
#include <linux/iio/driver.h>
#include <linux/iio/sysfs.h>

#include "../fpga.h"
#include "../fpga_io.h"


struct fpga_adc {
	struct device *dev;
	void __iomem *regs;
	u32 sample_rate;
	u32 vref_mv;
	u32 channel;
};


struct fpga_adc_channel_info {
	u32 reg_average;
	u32 reg_threshold;
	u32 chan_mask;
};

static const struct fpga_adc_channel_info fpga_adc_chans[] =
{
	{ FPGA_REG_SENSOR_ADC_AVERAGE_A1,	FPGA_REG_SENSOR_ADC_THRESHOLD_A1,	BIT(0) },
	{ FPGA_REG_SENSOR_ADC_AVERAGE_A2,	FPGA_REG_SENSOR_ADC_THRESHOLD_A2,	BIT(1) },
	{ FPGA_REG_SENSOR_ADC_AVERAGE_A3,	FPGA_REG_SENSOR_ADC_THRESHOLD_A3,	BIT(2) },
	{ FPGA_REG_SENSOR_ADC_AVERAGE_A4,	FPGA_REG_SENSOR_ADC_THRESHOLD_A4,	BIT(3) },
	{ FPGA_REG_SENSOR_ADC_AVERAGE_A5,	FPGA_REG_SENSOR_ADC_THRESHOLD_A5,	BIT(4) },
	{ FPGA_REG_SENSOR_ADC_AVERAGE_A6,	FPGA_REG_SENSOR_ADC_THRESHOLD_A6,	BIT(5) },
	{ FPGA_REG_SENSOR_ADC_AVERAGE_B1,	FPGA_REG_SENSOR_ADC_THRESHOLD_B1,	BIT(6) },
	{ FPGA_REG_SENSOR_ADC_AVERAGE_B2,	FPGA_REG_SENSOR_ADC_THRESHOLD_B2,	BIT(7) },
	{ FPGA_REG_SENSOR_ADC_AVERAGE_B3,	FPGA_REG_SENSOR_ADC_THRESHOLD_B3,	BIT(8) },
	{ FPGA_REG_SENSOR_ADC_AVERAGE_B4,	FPGA_REG_SENSOR_ADC_THRESHOLD_B4,	BIT(9) },
	{ FPGA_REG_SENSOR_ADC_AVERAGE_B5,	FPGA_REG_SENSOR_ADC_THRESHOLD_B5,	BIT(10)},
	{ FPGA_REG_SENSOR_ADC_AVERAGE_B6,	FPGA_REG_SENSOR_ADC_THRESHOLD_B6,	BIT(11)},
};

#define FPGA_ADC_MAX_TOTAL_CHANS	(ARRAY_SIZE(fpga_adc_chans))

#define FPGA_REG_SENSOR_ADC_OFFSET_MAX	FPGA_REG_SENSOR_ADC_AVERAGE_B6

static const struct iio_event_spec fpga_adc_events[] = {
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_RISING,
		.mask_separate = BIT(IIO_EV_INFO_ENABLE),
	}, {
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_FALLING,
		.mask_separate = BIT(IIO_EV_INFO_ENABLE),
	}, {
		.type = IIO_EV_TYPE_CHANGE, 
		.dir = IIO_EV_DIR_RISING,
		.mask_separate = BIT(IIO_EV_INFO_ENABLE),
	}, {
		.type = IIO_EV_TYPE_CHANGE,
		.dir = IIO_EV_DIR_FALLING,
		.mask_separate = BIT(IIO_EV_INFO_ENABLE),
	}, {
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_EITHER,
		.mask_separate = BIT(IIO_EV_INFO_VALUE),
	}, {
		.type = IIO_EV_TYPE_CHANGE,
		.dir = IIO_EV_DIR_NONE,
		.mask_separate = BIT(IIO_EV_INFO_VALUE),
	}, {
		.type = IIO_EV_TYPE_CHANGE,
		.dir = IIO_EV_DIR_EITHER,
		.mask_separate = BIT(IIO_EV_INFO_VALUE),
	},
};


#define FPGA_ADC_CHAN(_idx) {					\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.channel = (_idx),					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
				BIT(IIO_CHAN_INFO_AVERAGE_RAW),	\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |	\
				BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.event_spec = fpga_adc_events,				\
	.num_event_specs = ARRAY_SIZE(fpga_adc_events),		\
}

static const struct iio_chan_spec fpga_adc_iio_channels[] = {
	FPGA_ADC_CHAN(0),
	FPGA_ADC_CHAN(1),
	FPGA_ADC_CHAN(2),
	FPGA_ADC_CHAN(3),
	FPGA_ADC_CHAN(4),
	FPGA_ADC_CHAN(5),
	FPGA_ADC_CHAN(6),
	FPGA_ADC_CHAN(7),
	FPGA_ADC_CHAN(8),
	FPGA_ADC_CHAN(9),
	FPGA_ADC_CHAN(10),
	FPGA_ADC_CHAN(11),
};



static int fpga_adc_read_raw(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan,
			int *val,
			int *val2,
			long mask)
{
	struct fpga_adc *fpga = iio_priv(indio_dev);

	u32 channel;
	long ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
	case IIO_CHAN_INFO_AVERAGE_RAW:
		channel = chan->channel & 0x0f;
		ret = fpga_readl(val, fpga->regs + fpga_adc_chans[channel].reg_average);
		if (ret)
			return ret;
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		*val = fpga->vref_mv;
		*val2 = 12;
		return IIO_VAL_FRACTIONAL_LOG2;

	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = fpga->sample_rate; 
		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}
}


static int fpga_adc_read_event_config(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, enum iio_event_type type,
	enum iio_event_direction dir)
{
	struct fpga_adc *fpga = iio_priv(indio_dev);
	int rs;
	u32 val, reg;
	u32 channel;

	//pr_debug("read_event_config: type = %d, dir = %d\n", type, dir); 

	channel = chan->channel & 0x0f;
	switch (type) {
	case IIO_EV_TYPE_THRESH:
		reg = FPGA_REG_SENSOR_ADC_TRIGGER_MASK;
		break;
	case IIO_EV_TYPE_CHANGE:
		reg = FPGA_REG_SENSOR_ADC_TRIGGER_MASK_NEXT;
		break;
	default:
		return -EINVAL;
	}

	rs = fpga_readl(&val, fpga->regs + reg);
	return (bool)(val & fpga_adc_chans[channel].chan_mask);
}


static int fpga_adc_write_event_config(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, enum iio_event_type type,
	enum iio_event_direction dir, int state)
{
	struct fpga_adc *fpga = iio_priv(indio_dev);
	int rs;
	u32 mask, val, reg1, reg2;
	u32 channel;

	channel = chan->channel & 0x0f;
	mask = fpga_adc_chans[channel].chan_mask;

	//pr_debug("write_event_config: type = %d, dir = %d, state = %d\n", type, dir, state);

	switch (type) {
	case IIO_EV_TYPE_THRESH:
		reg1 = FPGA_REG_SENSOR_ADC_TRIGGER_MASK;
		reg2 = FPGA_REG_SENSOR_ADC_COMPARE_MODE;
		break;
	case IIO_EV_TYPE_CHANGE:
		reg1 = FPGA_REG_SENSOR_ADC_TRIGGER_MASK_NEXT;
		reg2 = FPGA_REG_SENSOR_ADC_COMPARE_MODE_NEXT;
		break;
	default:
		return -EINVAL;
	}

	mutex_lock(&indio_dev->mlock);

	val = ((dir == IIO_EV_DIR_FALLING)||(dir == IIO_EV_DIR_EITHER)) ? 0 : mask;	//modify by hl 2016.12.13
	rs = fpga_update_lbits(fpga->regs + reg2, mask, val); 
	if (IS_ERR_VALUE(rs))
		goto err_ret;

	val = (state == 0) ? 0 : mask;
	rs = fpga_update_lbits(fpga->regs + reg1, mask, val);
err_ret:
	mutex_unlock(&indio_dev->mlock);
	return rs;
}


static int fpga_adc_read_event_value(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, enum iio_event_type type,
	enum iio_event_direction dir, enum iio_event_info info,
	int *val, int *val2)
{
	struct fpga_adc *fpga = iio_priv(indio_dev);
	int ret, rs = IIO_VAL_INT;
	u32 mask, tmp, reg;
	u32 channel;

	channel = chan->channel & 0x0f;
	mask = fpga_adc_chans[channel].chan_mask;

	//pr_debug("read_event_value: type = %d, dir = %d, info = %d\n", type, dir, info);

	if (info != IIO_EV_INFO_VALUE)
		return -EINVAL;

	switch (type) {
	case IIO_EV_TYPE_THRESH:
		ret = fpga_readl(val, fpga->regs + fpga_adc_chans[channel].reg_threshold);
		if (IS_ERR_VALUE(ret))
			rs = ret;
		break;
	case IIO_EV_TYPE_CHANGE:
		switch (dir) {
		case IIO_EV_DIR_EITHER:
			reg = FPGA_REG_SENSOR_ADC_COMPARE_STATUS; 
			break;
		case IIO_EV_DIR_NONE:
			ret = fpga_readl(&tmp, fpga->regs + FPGA_REG_SENSOR_ADC_COMPARE_MODE_NEXT); 
			if (IS_ERR_VALUE(ret))
				return ret;
			*val2 = tmp & mask;
			reg = FPGA_REG_SENSOR_ADC_COMPARE_MODE;
			rs = IIO_VAL_INT_PLUS_MICRO;
			break;
		default:
			rs = -EINVAL;
		}
		ret = fpga_readl(&tmp, fpga->regs + reg);
		if (IS_ERR_VALUE(ret))
			rs = ret;
		else
			*val = tmp & mask;
		break;
	default:
		rs = -EINVAL;
	}

	return rs;
}


static int fpga_adc_write_event_value(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, enum iio_event_type type,
	enum iio_event_direction dir, enum iio_event_info info,
	int val, int val2)
{
	struct fpga_adc *fpga = iio_priv(indio_dev);
	int rs;
	u32 mask;
	u32 channel;

	channel = chan->channel & 0x0f;
	mask = fpga_adc_chans[channel].chan_mask;

	//pr_debug("write_event_value: type = %d, dir = %d, info = %d, val = %d, val2 = %d\n", type, dir, info, val, val2);

	if (val < 0 || val > 0xffff)
		return -EINVAL;

	mutex_lock(&indio_dev->mlock);

	switch (info) {
	case IIO_EV_INFO_VALUE:
		if (type == IIO_EV_TYPE_THRESH) {
			rs = fpga_writel(val, fpga->regs + fpga_adc_chans[channel].reg_threshold);
			break;
		}
	default:
		rs = -EINVAL;
	}

	mutex_unlock(&indio_dev->mlock);
	return rs;
}


static int fpga_adc_reg_access(struct iio_dev *indio_dev,
			unsigned reg, unsigned writeval,
			unsigned *readval)
{
	struct fpga_adc *fpga = iio_priv(indio_dev);

	if (!readval || reg > FPGA_REG_SENSOR_ADC_OFFSET_MAX)
		return -EINVAL;

	return fpga_readl(readval, fpga->regs + reg);
}


static void fpga_adc_hw_init(struct fpga_adc *fpga)
{
	int rs;
	rs = fpga_writel(0, fpga->regs + FPGA_REG_SENSOR_ADC_COMPARE_MODE);
	rs = fpga_writel(0, fpga->regs + FPGA_REG_SENSOR_ADC_TRIGGER_MASK);
}


static const struct iio_info fpga_adc_iio_info = {
	.driver_module = THIS_MODULE,
	.read_raw = &fpga_adc_read_raw,
	.read_event_config = &fpga_adc_read_event_config,
	.write_event_config = &fpga_adc_write_event_config,
	.read_event_value = &fpga_adc_read_event_value,
	.write_event_value = &fpga_adc_write_event_value, 
	.debugfs_reg_access = &fpga_adc_reg_access,
};


static int fpga_adc_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct fpga_adc *fpga;
	struct iio_dev *indio_dev;
	u32 fpga_cs, value;
	u32 reg[3];
	int ret;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*fpga));
	if (!indio_dev) {
		dev_err(&pdev->dev, "Failed allocating iio device\n");
		return -ENOMEM;
	}

	fpga = iio_priv(indio_dev);
	fpga->dev = &pdev->dev;

	ret = of_property_read_u32_array(np, "reg", reg, 3);
	if (IS_ERR_VALUE(ret)) {
		dev_err(&pdev->dev, "invalid FPGA ADC registers  definition\n");
		return ret;
	}
	fpga_cs = reg[0];
	fpga->regs = fpga_io_get(fpga_cs);
	if (IS_ERR(fpga->regs))
		return PTR_ERR(fpga->regs);


	ret = of_property_read_u32(np, "sample-rate", &value);
	dev_dbg(&pdev->dev, "FPGA ADC sample rate is %d\n", value);
	if (ret){
		dev_err(&pdev->dev, "Failed to parse sample rate of FPGA ADC\n");
		return ret;
	}
	fpga->sample_rate = value;

	ret = of_property_read_u32(np, "vref-mv", &value);
	dev_dbg(&pdev->dev, "FPGA ADC reference voltage is %d mV\n", value);
	if (ret){
		dev_err(&pdev->dev, "Failed to parse reference voltage of FPGA ADC\n");
		return ret;
	}
	fpga->vref_mv = value; 

	platform_set_drvdata(pdev, indio_dev);


	indio_dev->name = dev_name(&pdev->dev);
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->info = &fpga_adc_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = fpga_adc_iio_channels;
	indio_dev->num_channels = ARRAY_SIZE(fpga_adc_iio_channels);

	fpga_adc_hw_init(fpga);

	ret = iio_device_register(indio_dev);
	if (ret) {
		dev_err(&pdev->dev, "Couldn't register the device.\n");
		goto error_iio_return;
	}

	return 0;

error_iio_return:

	return ret;
}

static int fpga_adc_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);

	iio_device_unregister(indio_dev);

	return 0;
}


static const struct of_device_id fpga_adc_match[] = {
	{ .compatible = "gwi,fpga-adc", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, fpga_adc_match);

static struct platform_driver fpga_adc_driver = {
	.probe          = fpga_adc_probe,
	.remove         = fpga_adc_remove,
	.driver         = {
		.name   = "fpga_adc",
		.of_match_table = fpga_adc_match,
	},
};

module_platform_driver(fpga_adc_driver);

MODULE_AUTHOR("Zhang Xudong <zhangxudong@gwi.com.cn>");
MODULE_DESCRIPTION("GWI FPGA-controlled ADC driver");
MODULE_LICENSE("GPL v2");
