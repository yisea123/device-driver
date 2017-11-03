/*
 * image scanning unit device driver user interface definitions
 *
 * Copyright 2016 Hunan GreatWall Information Financial Equipment Co., Ltd.
 *
 */
#ifndef __SCANUNIT_H__
#define __SCANUNIT_H__

#include <linux/ioctl.h>
#include <linux/types.h>

#ifdef CONFIG_MICROSEMI_010T
#define SCANUNIT_NAME		"imagescan"
#define SCAN_RAM_SIZE		31104
#define LIGHT_SOURCE_NUM	8
#define SCANLINE_MAX_PIXELS	10368
	
#define SCANLINE_PIXELS		2592
#define SCANLINE_DATA_CNT	SCANLINE_PIXELS
#define SCANLINE_SIZE		LIGHT_SOURCE_NUM*SCANLINE_DATA_CNT
#define SCANLINE_MAX_CNT	100
#define SCANBUF_SIZE		SCANLINE_SIZE*SCANLINE_MAX_CNT*2
#define RESERVE_SCANBUF_SIZE	0x16800000
#endif

#define MAX_IMAGE_SENSORS	2
#define MAX_IMAGE_DIGITISERS	8


/* IOCTL commands definition */
#define SCANUNIT_IOC_MAGIC		'S'

#define SCANUNIT_RESET			_IO(SCANUNIT_IOC_MAGIC, 0)
#ifdef CONFIG_MICROSEMI_010T
#define SCANUNIT_SET_SCANMODE		_IOW(SCANUNIT_IOC_MAGIC, 17, struct scanunit_scanmode_config_arg)
#else
#define SCANUNIT_SET_SCANMODE		_IO(SCANUNIT_IOC_MAGIC, 1)
#endif

#define SCANUNIT_START_SCANNING		_IO(SCANUNIT_IOC_MAGIC, 2)
#define SCANUNIT_STOP_SCANNING		_IO(SCANUNIT_IOC_MAGIC, 3)
#define SCANUNIT_TURNON_LIGHTS		_IO(SCANUNIT_IOC_MAGIC, 4)
#define SCANUNIT_TURNOFF_LIGHTS		_IO(SCANUNIT_IOC_MAGIC, 5)
#ifdef CONFIG_MICROSEMI_010T
#define SCANUNIT_SET_SCANLINES		_IO(SCANUNIT_IOC_MAGIC, 6)
#define SCANUNIT_GET_SCANLINES		_IO(SCANUNIT_IOC_MAGIC, 7)
#define SCANUNIT_WAIT_SCAN_END		_IO(SCANUNIT_IOC_MAGIC, 8)
#endif

#define SCANUNIT_GET_HWINFO		_IOR(SCANUNIT_IOC_MAGIC, 10, struct scanunit_hwinfo)

#define SCANUNIT_GET_DIGITISER_CONFIG	_IOR(SCANUNIT_IOC_MAGIC, 11, struct scanunit_config_arg)
#define SCANUNIT_SET_DIGITISER_CONFIG	_IOW(SCANUNIT_IOC_MAGIC, 12, struct scanunit_config_arg)
#define SCANUNIT_GET_SENSOR_CONFIG	_IOR(SCANUNIT_IOC_MAGIC, 13, struct scanunit_config_arg)
#define SCANUNIT_SET_SENSOR_CONFIG	_IOW(SCANUNIT_IOC_MAGIC, 14, struct scanunit_config_arg)
#define SCANUNIT_GET_SENSOR_COM_CONFIG	_IOR(SCANUNIT_IOC_MAGIC, 15, struct scan_reg_config)
#define SCANUNIT_SET_SENSOR_COM_CONFIG	_IOW(SCANUNIT_IOC_MAGIC, 16, struct scan_reg_config)

/* scanning mode definition */
#define SCANNING_SIX_LIGHTSOURCE_MODE	0
#define SCANNING_TEN_LIGHTSOURCE_MODE	1

/* scanning section description block */
struct scan_section {
	int lstart;				// start logical pixel nubmer of a scanning section
	int lend;				// end logical pixel nubmer of a scanning section
	int pstart;				// start physical pixel nubmer of a scanning section
	int pend;				// end physical pixel nubmer of a scanning section
	int digitiser_id;			// ID of the digitiser which converts pixel data of a section
	int channel;				// channel of the digitiser which converts pixel data of a section
};


/* scanunit hardware information block */
struct scanunit_hwinfo {
	int sides;				// number of scanning side
	int colors;				// image sensor colors (1 - greyscale; 3 - R/G/B color)
	int lightsources;			// number of light sources
	int sensors;				// number of image sensor
	int digitisers;				// number of image digitiser
	int sensor_a;				// image sensor ID of side A
	int sections_a;				// number of scanning sections of side A
	struct scan_section sectinfo_a[10];	// scanning section information block of side A	
	int sensor_b;				// image sensor ID of side B
	int sections_b;				// number of scanning sections of side B
	struct scan_section sectinfo_b[10];	// scanning section information block of side B
#ifdef CONFIG_MICROSEMI_010T
	int dpi_pixels[10];
#endif
};

#define MAX_SCANNING_SECTIONS	(sizeof(((struct scanunit_hwinfo *)0)->sectinfo_a)/sizeof(struct scan_section))


/* scanunit hardware register configuration structure */
struct scan_reg_config {
	unsigned int address;			// address of scanunit register to configure
	unsigned int value;			// value of scanunit register to configure
	unsigned int mask;			// bit mask when mask is not ZERO
};


/* scanunit hardware configuration block */
struct scanunit_config {
	int regcount;				// value of scanunit register to configure
	struct scan_reg_config *regconfig;	// address of scanunit register configure block
};

/* argument block of scanunit device's/chip's hardware configuration IOCTL functions */
struct scanunit_config_arg {
	int device;				// device/chip ID of scanunit to configure
	struct scanunit_config config;		// hardware configuration data of the device/chip
};

#ifdef CONFIG_MICROSEMI_010T
struct scanunit_scanmode {
	unsigned char dpimode;			
	unsigned char ledmode;
};

struct scanunit_scanmode_config_arg {
	unsigned int fpga_reg_dpi_high;	// fpga_reg_dpi_si_high config value
	unsigned int fpga_reg_dpi_low;		// fpga_reg_dpi_si_low config value
	struct scanunit_scanmode mode;		// hardware scanmode configuration
};
#endif

#endif /* __SCANUNIT_H__ */
