/*
 * Image Scan Unit IOCTL functions
 *
 * Copyright 2016 Hunan GreatWall Information Financial Equipment Co., Ltd.
 *
 * Licensed under the GPL-2.
 */
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/delay.h>

#include "../fpga.h"
#include "../fpga_io.h"
#include "scanunit.h"
#include "scanctrl.h"
#include "imagedigitiser.h"
#include "imagesensor.h"


extern void __iomem *ints_reg_base, *cis_reg_base, *cis_dpi_reg_base;
extern struct scanunit_hwinfo scanner_info;
extern struct imagedigitiser *image_digitisers[];
extern struct imagesensor *image_sensors[];
extern int scanmode;

static inline long scanunit_io_reset(unsigned long arg)
{
	fpga_writel(0, cis_reg_base + FPGA_REG_CIS_CONTROL);
	return 0;
}


static inline long scanunit_io_set_scanmode(unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct scanunit_scanmode_config_arg config_arg;
	int rs;

	if (copy_from_user(&config_arg, argp, sizeof(struct scanunit_scanmode_config_arg)))
		return -EFAULT;

	rs = fpga_update_lbits(cis_reg_base + FPGA_REG_CIS_CONTROL, FPGA_REG_CIS_SCANMODE_MASK, (config_arg.mode.ledmode<<2)); 
	rs = fpga_writel(config_arg.mode.dpimode, cis_dpi_reg_base + FPGA_REG_CIS_DPI_CONTROL);
	rs = fpga_writel(config_arg.fpga_reg_dpi_high, cis_dpi_reg_base + FPGA_REG_DPI_SI_H);
	rs = fpga_writel(config_arg.fpga_reg_dpi_low, cis_dpi_reg_base + FPGA_REG_DPI_SI_L);

	return rs;
}

static inline long scanunit_io_start_scanning(unsigned long arg)
{
	int rs;
	void __iomem *imgdata_int_status = NULL;
	void __iomem *imgdata_int_clear = NULL;
	u32 imgdata_status;

	scandrv_ctrl.linecount = 0;
	scandrv_ctrl.scanlines = 0;
	reinit_completion(&scandrv_ctrl.scan_completion);

	imgdata_int_status = ints_reg_base + FPGA_REG_IMG_DATA_INT_STATUS;
	imgdata_int_clear = ints_reg_base + FPGA_REG_IMG_DATA_INT_CLEAR;

	fpga_readl(&imgdata_status, imgdata_int_status);
	fpga_writel(imgdata_status, imgdata_int_clear); //clear int status bits
	//reset fpga_scanunit
	rs = fpga_update_lbits(cis_reg_base + FPGA_REG_CIS_CONTROL, 0x200, 0x200);
	udelay(10);
	rs = fpga_update_lbits(cis_reg_base + FPGA_REG_CIS_CONTROL, 0x200, 0);
	udelay(10);
	if (arg == 0)
		rs = fpga_update_lbits(cis_reg_base + FPGA_REG_CIS_CONTROL, FPGA_REG_CIS_SCAN_ENABLE, FPGA_REG_CIS_SCAN_ENABLE);
	else if (arg == 1)
		rs = fpga_update_lbits(cis_reg_base + FPGA_REG_CIS_CONTROL, FPGA_REG_CIS_SCAN_TRIGGER_ENABLE, FPGA_REG_CIS_SCAN_TRIGGER_ENABLE);
	else
		return -EINVAL;
	scandrv_ctrl.start_flag = 1;
	return rs;
}


static inline long scanunit_io_stop_scanning(unsigned long arg)
{
	int rs;
	rs = fpga_update_lbits(cis_reg_base + FPGA_REG_CIS_CONTROL, FPGA_REG_CIS_SCAN_ENABLE|FPGA_REG_CIS_SCAN_TRIGGER_ENABLE, 0);
	scandrv_ctrl.start_flag = 0; 
	complete(&scandrv_ctrl.scan_completion);
	return rs;
}


static inline long scanunit_io_turnon_lights(unsigned long arg)
{
	int rs;
	rs = fpga_update_lbits(cis_reg_base + FPGA_REG_CIS_CONTROL, FPGA_REG_CIS_LEDS_ENABLE, FPGA_REG_CIS_LEDS_ENABLE);
	return rs;
}


static inline long scanunit_io_turnoff_lights(unsigned long arg)
{
	int rs;
	rs = fpga_update_lbits(cis_reg_base + FPGA_REG_CIS_CONTROL, FPGA_REG_CIS_LEDS_ENABLE, 0);
	return rs;
}


static long scanunit_io_get_hwinfo(unsigned long arg)
{
	void __user *argp = (void __user *)arg;

	if (argp == NULL)
		return -EINVAL;

	if (copy_to_user(argp, &scanner_info, sizeof(scanner_info)))
		return -EFAULT;
	return 0;
}


static long scanunit_io_get_digitiser_config(unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	void __user *dataptr;
	struct scanunit_config_arg config_arg;
	struct scanunit_config tmpconfig;
	int datasize;
	int rs;

	if (copy_from_user(&config_arg, argp, sizeof(config_arg)))
		return -EFAULT;
	if (config_arg.device < 0 || config_arg.device >= scanner_info.digitisers) 
		return -EINVAL;
	if (config_arg.config.regcount <= 0 || config_arg.config.regconfig == NULL)
		return -EINVAL;

	tmpconfig.regcount = config_arg.config.regcount;
	datasize = tmpconfig.regcount * sizeof(*tmpconfig.regconfig);
	tmpconfig.regconfig = kzalloc(datasize, GFP_KERNEL); 
	if (IS_ERR(tmpconfig.regconfig))
		return -ENOMEM;
	memcpy(tmpconfig.regconfig, config_arg.config.regconfig, datasize);

	rs = imagedigitiser_get_config(image_digitisers[config_arg.device], &tmpconfig);
	if (IS_ERR_VALUE(rs))
		goto free_n_return;

	dataptr = config_arg.config.regconfig;
	rs = copy_to_user(dataptr, tmpconfig.regconfig, datasize) ? -EFAULT : 0;

free_n_return:
	kfree(tmpconfig.regconfig);
	return rs;
}


static long scanunit_io_set_digitiser_config(unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	void __user *dataptr;
	struct scanunit_config_arg config_arg;
	struct scanunit_config tmpconfig;
	struct imagedigitiser *digitiser;
	int datasize;
	int rs;

	if (copy_from_user(&config_arg, argp, sizeof(config_arg)))
		return -EFAULT;
	if (config_arg.device < -1 || config_arg.device >= scanner_info.digitisers) 
		return -EINVAL;
	if (config_arg.config.regcount <= 0 || config_arg.config.regconfig == NULL)
		return -EINVAL;

	tmpconfig.regcount = config_arg.config.regcount;
	datasize = tmpconfig.regcount * sizeof(*tmpconfig.regconfig);
	tmpconfig.regconfig = kzalloc(datasize, GFP_KERNEL); 
	if (IS_ERR(tmpconfig.regconfig))
		return -ENOMEM;

	dataptr = config_arg.config.regconfig;
	rs = copy_from_user(tmpconfig.regconfig, dataptr, datasize) ? -EFAULT : 0;
	if (IS_ERR_VALUE(rs))
		goto free_n_return;

	if (config_arg.device == -1)	// device == -1: set all available digitisers with the same configuration
	{
		int i;

		for (i=0; i<scanner_info.digitisers; i++) {
			digitiser = image_digitisers[i];
			rs = imagedigitiser_set_config(digitiser, &tmpconfig);
			if (IS_ERR_VALUE(rs))
				goto free_n_return;
			imagedigitiser_disable(digitiser);
			imagedigitiser_enable(digitiser);	// enable digitiser to update its configuration data by FPGA
		}
	}
	else				// device <> -1: configure a digitiser specified by "device"
	{
		digitiser = image_digitisers[config_arg.device];
		rs = imagedigitiser_set_config(digitiser, &tmpconfig);
		if (IS_ERR_VALUE(rs))
			goto free_n_return;
		imagedigitiser_disable(digitiser);
		imagedigitiser_enable(digitiser);	// enable digitiser to update its configuration data by FPGA
	}

free_n_return:
	kfree(tmpconfig.regconfig);
	return rs;
}


static long scanunit_io_get_sensor_config(unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	void __user *dataptr;
	struct scanunit_config_arg config_arg;
	struct scanunit_config tmpconfig;
	int datasize;
	int rs;

	if (copy_from_user(&config_arg, argp, sizeof(config_arg)))
		return -EFAULT;
	if (config_arg.device < 0 || config_arg.device >= scanner_info.sensors) 
		return -EINVAL;
	if (config_arg.config.regcount <= 0 || config_arg.config.regconfig == NULL)
		return -EINVAL;

	tmpconfig.regcount = config_arg.config.regcount;
	datasize = tmpconfig.regcount * sizeof(*tmpconfig.regconfig);
	tmpconfig.regconfig = kzalloc(datasize, GFP_KERNEL); 
	if (IS_ERR(tmpconfig.regconfig))
		return -ENOMEM;

	rs = imagesensor_get_config(image_sensors[config_arg.device], &tmpconfig);
	if (IS_ERR_VALUE(rs))
		goto free_n_return;

	dataptr = config_arg.config.regconfig;
	rs = copy_to_user(dataptr, tmpconfig.regconfig, datasize) ? -EFAULT : 0;

free_n_return:
	kfree(tmpconfig.regconfig);
	return rs;
}

static long scanunit_io_set_sensor_config(unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	void __user *dataptr;
	struct scanunit_config_arg config_arg;
	struct scanunit_config tmpconfig;
	int datasize;
	int rs;

	if (copy_from_user(&config_arg, argp, sizeof(config_arg)))
		return -EFAULT;
	if (config_arg.device < -1 || config_arg.device >= scanner_info.sensors) 
		return -EINVAL;
	if (config_arg.config.regcount <= 0 || config_arg.config.regconfig == NULL)
		return -EINVAL;

	tmpconfig.regcount = config_arg.config.regcount;
	datasize = tmpconfig.regcount * sizeof(*tmpconfig.regconfig);
	tmpconfig.regconfig = kzalloc(datasize, GFP_KERNEL); 
	if (IS_ERR(tmpconfig.regconfig))
		return -ENOMEM;

	dataptr = config_arg.config.regconfig;
	rs = copy_from_user(tmpconfig.regconfig, dataptr, datasize) ? -EFAULT : 0;
	if (IS_ERR_VALUE(rs))
		goto free_n_return;

	if (config_arg.device == -1)	// device == -1: set all available sensors with the same configuration
	{
		int i;

		for (i=0; i<scanner_info.sensors; i++) {
			rs = imagesensor_set_config(image_sensors[i], &tmpconfig);
			if (IS_ERR_VALUE(rs))
				goto free_n_return;
		}
	}
	else				// device <> -1: configure a sensor specified by "device"
	{
		rs = imagesensor_set_config(image_sensors[config_arg.device], &tmpconfig);
		if (IS_ERR_VALUE(rs))
			goto free_n_return;
	}

free_n_return:
	kfree(tmpconfig.regconfig);
	return rs;
}


static long scanunit_io_set_sensor_common_config(unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct scan_reg_config regconfig;
	u32 value;

	if (copy_from_user(&regconfig, argp, sizeof(struct scan_reg_config)))
		return -EFAULT;

	if (regconfig.mask != 0) {	// bit operation
		fpga_readl(&value, cis_reg_base + regconfig.address);
		value &= ~regconfig.mask;
		value |= regconfig.value & regconfig.mask;
	}
	else
		value = regconfig.value;

	fpga_writel(value, cis_reg_base + regconfig.address);

	return 0;
}

static long scanunit_io_get_sensor_common_config(unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct scan_reg_config regconfig;
	int rs;

	if (copy_from_user(&regconfig, argp, sizeof(struct scan_reg_config)))
		return -EFAULT;

	fpga_readl(&regconfig.value, cis_reg_base + regconfig.address);

	rs = copy_to_user(argp, &regconfig, sizeof(struct scan_reg_config)) ? -EFAULT : 0;

	return rs;
}

static inline long scanunit_io_get_scanline(unsigned long arg)
{
	return scandrv_ctrl.linecount;
}

static inline long scanunit_io_set_scanline(unsigned long arg)
{
//	scandrv_ctrl.scanlines = (int)arg;
//	scandrv_ctrl.linecount = 0;
//	memset(scandrv_ctrl.buffer, 0, SCANBUF_SIZE);
	return 0;
}

static inline long scanunit_io_wait_scancomplete(unsigned long arg)
{
	int timeout = msecs_to_jiffies(arg);
	int rs;

	if (scandrv_ctrl.linecount == SCANLINE_MAX_CNT) {
		return 0;
	}
	rs = wait_for_completion_timeout(&scandrv_ctrl.scan_completion, timeout);
	
	return (rs==0)? -EAGAIN : 0;
}

long scanunit_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	switch (cmd) {
	case SCANUNIT_RESET:
		return scanunit_io_reset(arg);
	case SCANUNIT_SET_SCANMODE:
		return scanunit_io_set_scanmode(arg);
	case SCANUNIT_START_SCANNING:
		return scanunit_io_start_scanning(arg);
	case SCANUNIT_STOP_SCANNING:
		return scanunit_io_stop_scanning(arg);
	case SCANUNIT_TURNON_LIGHTS:
		return scanunit_io_turnon_lights(arg);
	case SCANUNIT_TURNOFF_LIGHTS:
		return scanunit_io_turnoff_lights(arg);
	case SCANUNIT_GET_HWINFO:
		return scanunit_io_get_hwinfo(arg);
	case SCANUNIT_GET_DIGITISER_CONFIG:
		return scanunit_io_get_digitiser_config(arg);
	case SCANUNIT_SET_DIGITISER_CONFIG:
		return scanunit_io_set_digitiser_config(arg);
	case SCANUNIT_GET_SENSOR_CONFIG:
		return scanunit_io_get_sensor_config(arg);
	case SCANUNIT_SET_SENSOR_CONFIG:
		return scanunit_io_set_sensor_config(arg);
	case SCANUNIT_SET_SENSOR_COM_CONFIG:
		return scanunit_io_set_sensor_common_config(arg);
	case SCANUNIT_GET_SENSOR_COM_CONFIG:
		return scanunit_io_get_sensor_common_config(arg);
	case SCANUNIT_SET_SCANLINES:
		return scanunit_io_set_scanline(arg);
	case SCANUNIT_GET_SCANLINES:
		return scanunit_io_get_scanline(arg);
	case SCANUNIT_WAIT_SCAN_END:
		return scanunit_io_wait_scancomplete(arg);
	}
	return -EINVAL;
}

