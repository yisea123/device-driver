/*
 * GWI FPGA hardware definitions
 *
 * Copyright 2016 Hunan GreatWall Information Financial Equipment Co., Ltd.
 *
 */
#ifndef __FPGA_H__
#define __FPGA_H__

#include <linux/bitops.h>

/* Section 1: FPGA control & information registers */
/* memory map (offset): */
#define FPGA_REG_CONTROL			0x0000
#define FPGA_REG_VERSION			0x0004
#define FPGA_REG_BCLK_FREQUENCY			0x0008

/* bits definition of FPGA_REG_CONTROL */
#define FPGA_REG_CONTROL_RESET			BIT(0)


/* Section 2: FPGA interrupt registers */
/* memory map (offset to base address): */
#define FPGA_REG_INT_ENABLE			0x0000
#define FPGA_REG_INT_CLEAR			0x0004
#define FPGA_REG_INT_STATUS			0x0018
#define FPGA_REG_SENSOR_INT_STATUS		0x0008
#define FPGA_REG_SENSOR_INT_CLEAR		0x0008
#define FPGA_REG_IMG_DATA_INT_STATUS		0x000c
#define FPGA_REG_IMG_DATA_INT_CLEAR		0x0020
#define FPGA_REG_MOTOR_INT_STATUS		0x0010
#define FPGA_REG_MOTOR_INT_CLEAR		0x001c
#define FPGA_REG_IMG_ADC_INT_STATUS		0x0014
#define FPGA_REG_IMG_ADDR_INT_STATUS		0x0024
#define FPGA_REG_IMG_ADDR_INT_CLEAR		0x0028

/* bits definition of FPGA_REG_INT_xxx */
#define FPGA_REG_INT_SEL_TYPE1			BIT(0)
#define FPGA_REG_INT_SEL_TYPE2			BIT(1)
#define FPGA_REG_INT_SEL_TYPE3			BIT(2)
#define FPGA_REG_INT_SEL_TYPE4			BIT(3)
#define FPGA_REG_INT_SEL_TYPE5			BIT(4)
#define FPGA_REG_INT_SEL_ALL			BIT(15)

/* bits definition of FPGA_REG_SENSOR_INT_STATUS */
#define FPGA_REG_SENSOR_COVERED			BIT(0)

/* bits definition of FPGA_REG_IMG_DATA_INT_STATUS & FPGA_REG_IMG_DATA_INT_CLEAR */
/* RGB-interlaced mode */
#define FPGA_REG_IMG_DATA_LIGHT_VI_A		BIT(0)
#define FPGA_REG_IMG_DATA_LIGHT_IR_A		BIT(1)
#define FPGA_REG_IMG_DATA_LIGHT_IRT_A		BIT(2)
#define FPGA_REG_IMG_DATA_LIGHT_UV_A		BIT(3)
#define FPGA_REG_IMG_DATA_LIGHT_UVT_A		BIT(4)
#define FPGA_REG_IMG_DATA_LIGHT_VI_B		BIT(5)
#define FPGA_REG_IMG_DATA_LIGHT_IR_B		BIT(6)
#define FPGA_REG_IMG_DATA_LIGHT_IRT_B		BIT(7)
#define FPGA_REG_IMG_DATA_LIGHT_UV_B		BIT(8)
#define FPGA_REG_IMG_DATA_LIGHT_UVT_B		BIT(9)

/* bits definition of FPGA_REG_IMG_DATA_INT_STATUS & FPGA_REG_IMG_DATA_INT_CLEAR */
/* greyscale mode */
#define FPGA_REG_IMG_DATA_LIGHT_R		BIT(0)
#define FPGA_REG_IMG_DATA_LIGHT_G		BIT(1)
#define FPGA_REG_IMG_DATA_LIGHT_B		BIT(2)
#define FPGA_REG_IMG_DATA_LIGHT_IR		BIT(3)
#define FPGA_REG_IMG_DATA_LIGHT_RGB		BIT(4)
#define FPGA_REG_IMG_DATA_LIGHT_UV		BIT(5)


/* bits definition of FPGA_REG_IMG_ADDR_INT_STATUS & FPGA_REG_IMG_ADDR_INT_CLEAR */
#define FPGA_REG_IMG_DATA_ADDR_01		BIT(0)
#define FPGA_REG_IMG_DATA_ADDR_02		BIT(1)
#define FPGA_REG_IMG_DATA_ADDR_03		BIT(2)
#define FPGA_REG_IMG_DATA_ADDR_04		BIT(3)
#define FPGA_REG_IMG_DATA_ADDR_05		BIT(4)
#define FPGA_REG_IMG_DATA_ADDR_06		BIT(5)
#define FPGA_REG_IMG_DATA_ADDR_07		BIT(6)
#define FPGA_REG_IMG_DATA_ADDR_08		BIT(7)
#define FPGA_REG_IMG_DATA_ADDR_09		BIT(8)
#define FPGA_REG_IMG_DATA_ADDR_10		BIT(9)
#define FPGA_REG_IMG_DATA_ADDR_11		BIT(10)
#define FPGA_REG_IMG_DATA_ADDR_12		BIT(11)

/* bits definition of FPGA_REG_IMG_ADC_INT_STATUS */
#define FPGA_REG_IMG_ADC1_CONFIG_DONE		BIT(0)
#define FPGA_REG_IMG_ADC2_CONFIG_DONE		BIT(1)
#define FPGA_REG_IMG_ADC3_CONFIG_DONE		BIT(2)
#define FPGA_REG_IMG_ADC4_CONFIG_DONE		BIT(3)
#define FPGA_REG_IMG_ADC5_CONFIG_DONE		BIT(4)

#define FPGA_REG_IMG_ADC1_SELFTEST_OK		(3 << 16)
#define FPGA_REG_IMG_ADC1_SELFTEST_ERR		(1 << 16)
#define FPGA_REG_IMG_ADC2_SELFTEST_OK		(3 << 18)
#define FPGA_REG_IMG_ADC2_SELFTEST_ERR		(1 << 18)
#define FPGA_REG_IMG_ADC3_SELFTEST_OK		(3 << 20)
#define FPGA_REG_IMG_ADC3_SELFTEST_ERR		(1 << 20)
#define FPGA_REG_IMG_ADC4_SELFTEST_OK		(3 << 22)
#define FPGA_REG_IMG_ADC4_SELFTEST_ERR		(1 << 22)
#define FPGA_REG_IMG_ADC5_SELFTEST_OK		(3 << 24)
#define FPGA_REG_IMG_ADC5_SELFTEST_ERR		(1 << 24)

/* bits definition of FPGA_REG_MOTOR_INT_STATUS & FPGA_REG_MOTOR_INT_CLEAR */
#define FPGA_REG_MOTOR_INT_XXX			BIT(0)


/* Section 3: position sensor (PWM + ADC) registers */
/* memory map (offset): */
#define FPGA_REG_SENSOR_ADC_COMPARE_MODE	0x0000
#define FPGA_REG_SENSOR_ADC_COMPARE_STATUS	0x0004
#define FPGA_REG_SENSOR_ADC_TRIGGER_MASK	0x0008
#define FPGA_REG_SENSOR_ADC_MOTOR_CONTROL	0x000c
#define FPGA_REG_SENSOR_ADC_COMPARE_MODE_NEXT	0x0010
#define FPGA_REG_SENSOR_ADC_TRIGGER_MASK_NEXT	0x0014
#define FPGA_REG_SENSOR_ADC_COMPARE_RESULT	0x001c
#define FPGA_REG_SENSOR_DC_COMPARE_MODE		0x0020
#define FPGA_REG_SENSOR_DC_SENSOR_MASK		0x0024

#define FPGA_REG_SENSOR_ADC_THRESHOLD_A1	0x0030
#define FPGA_REG_SENSOR_ADC_THRESHOLD_A2	0x0034
#define FPGA_REG_SENSOR_ADC_THRESHOLD_A3	0x0038
#define FPGA_REG_SENSOR_ADC_THRESHOLD_A4	0x003c
#define FPGA_REG_SENSOR_ADC_THRESHOLD_A5	0x0040
#define FPGA_REG_SENSOR_ADC_THRESHOLD_A6	0x0044
#define FPGA_REG_SENSOR_ADC_THRESHOLD_B1	0x0048
#define FPGA_REG_SENSOR_ADC_THRESHOLD_B2	0x004c
#define FPGA_REG_SENSOR_ADC_THRESHOLD_B3	0x0050
#define FPGA_REG_SENSOR_ADC_THRESHOLD_B4	0x0054
#define FPGA_REG_SENSOR_ADC_THRESHOLD_B5	0x0058
#define FPGA_REG_SENSOR_ADC_THRESHOLD_B6	0x005c

#define FPGA_REG_SENSOR_ADC_AVERAGE_A1		0x0060
#define FPGA_REG_SENSOR_ADC_AVERAGE_A2		0x0064
#define FPGA_REG_SENSOR_ADC_AVERAGE_A3		0x0068
#define FPGA_REG_SENSOR_ADC_AVERAGE_A4		0x006c
#define FPGA_REG_SENSOR_ADC_AVERAGE_A5		0x0070
#define FPGA_REG_SENSOR_ADC_AVERAGE_A6		0x0074
#define FPGA_REG_SENSOR_ADC_AVERAGE_B1		0x0078
#define FPGA_REG_SENSOR_ADC_AVERAGE_B2		0x007c
#define FPGA_REG_SENSOR_ADC_AVERAGE_B3		0x0080
#define FPGA_REG_SENSOR_ADC_AVERAGE_B4		0x0084
#define FPGA_REG_SENSOR_ADC_AVERAGE_B5		0x0088
#define FPGA_REG_SENSOR_ADC_AVERAGE_B6		0x008c

#define FPGA_REG_SENSOR_PWM_CONTROL		0x0228

#define FPGA_REG_SENSOR_PWMx_PERIOD		0x0104
#define FPGA_REG_SENSOR_PWMx_PULSEWIDTH		0x0108

#define FPGA_REG_SENSOR_SADC_IN			0x0220

/* bits definition of FPGA_REG_SENSOR_PWM_CONTROL */
#define FPGA_REG_SENSOR_PWM1_ENABLE		BIT(0)
#define FPGA_REG_SENSOR_PWM2_ENABLE		BIT(1)
#define FPGA_REG_SENSOR_PWM3_ENABLE		BIT(2)
#define FPGA_REG_SENSOR_PWM4_ENABLE		BIT(3)
#define FPGA_REG_SENSOR_PWM5_ENABLE		BIT(4)
#define FPGA_REG_SENSOR_PWM6_ENABLE		BIT(5)
#define FPGA_REG_SENSOR_PWM7_ENABLE		BIT(6)
#define FPGA_REG_SENSOR_PWM8_ENABLE		BIT(7)
#define FPGA_REG_SENSOR_PWM9_ENABLE		BIT(8)
#define FPGA_REG_SENSOR_PWM10_ENABLE		BIT(9)
#define FPGA_REG_SENSOR_PWM11_ENABLE		BIT(10)
#define FPGA_REG_SENSOR_PWM12_ENABLE		BIT(11)

#define FPGA_REG_SENSOR_SPWM1_ENABLE		BIT(0)
#define FPGA_REG_SENSOR_SPWM2_ENABLE		BIT(1)
#define FPGA_REG_SENSOR_SPWM3_ENABLE		BIT(2)
#define FPGA_REG_SENSOR_SPWM4_ENABLE		BIT(3)
#define FPGA_REG_SENSOR_SPWM5_ENABLE		BIT(4)
#define FPGA_REG_SENSOR_SPWM6_ENABLE		BIT(5)

/* bits definition of FPGA_REG_SENSOR_SADC_IN */
#define FPGA_REG_SENSOR_SADC_IN1		BIT(0)
#define FPGA_REG_SENSOR_SADC_IN2		BIT(1)
#define FPGA_REG_SENSOR_SADC_IN3		BIT(2)
#define FPGA_REG_SENSOR_SADC_IN4		BIT(3)
#define FPGA_REG_SENSOR_SADC_IN5		BIT(4)
#define FPGA_REG_SENSOR_SADC_IN6		BIT(5)


/* Section 4: steppermotor control registers and speed profile table */
/* memory map (offset to unit base): */
#define FPGA_REG_MOTOR_CONTROL			0x0000
#define FPGA_REG_MOTOR_MEDIALENGTH_STEPS	0x0004
#define FPGA_REG_MOTOR_RUNNING_STEPS		0x0008
#define FPGA_REG_MOTOR_PRESET_STEPS		0x000c
#define FPGA_REG_MOTOR_STATUS			0x0010
#define FPGA_REG_MOTOR_TRIGGER_STEP_NEXT	0x001c
#define FPGA_REG_MOTOR_SPEEDLEVEL		0x0014
#define FPGA_REG_MOTOR_SENSOR_SELECT_MASK	0x0018
#define FPGA_REG_MOTOR_SKEW_STEPS		0x0038


/* bits definition of FPGA_REG_MOTORx_CONTROL */
#define FPGA_REG_MOTOR_RUN			BIT(0)
#define FPGA_REG_MOTOR_DIRECTION		BIT(1)
#define FPGA_REG_MOTOR_STOP			(2 << 2)
#define FPGA_REG_MOTOR_EMERGENCY_BRAKE		(3 << 2)
#define FPGA_REG_MOTOR_STOP_AT_TRGSTEP_END	BIT(4)
#define FPGA_REG_MOTOR_SENSOR_CHECK_MODE	BIT(5)
#define FPGA_REG_MOTOR_SENSER_STOP_ENABLE	BIT(6)
#define FPGA_REG_MOTOR_SENSER_CONTINUE_MODE	BIT(7)
#define FPGA_REG_MOTOR_SENSER_STOP_MODE		BIT(8)
#define FPGA_REG_MOTOR_EN_SKEW_STEPS		BIT(9)
#define FPGA_REG_MOTOR_CONTROL_MASK 		0x3ff

/* bits definition of FPGA_REG_MOTORx_STATUS */
#define FPGA_REG_MOTOR_STOPPED_BY_SENSOR	BIT(0)
#define FPGA_REG_MOTOR_STOPPED_BY_TRIGGERSTEP	BIT(1)
#define FPGA_REG_MOTOR_STOPPED			BIT(2)
#define FPGA_REG_MOTOR_MEDIALENGTH_OK		BIT(3)
#define FPGA_REG_MOTOR_TRIGGERSTEP_DONE		BIT(4)
#define FPGA_REG_MOTOR_STOPPED_BY_SKEW		BIT(5)
#define FPGA_REG_MOTOR_FAULT			BIT(6)

/* size of steppermotor speed profile tables: */
#define FPGA_RAM_MOTOR_TABLE_RAMP_DEPTH		1936
#define FPGA_RAM_MOTOR_TABLE_COUNT_DEPTH	16
#define FPGA_RAM_MOTOR_TABLE_STOPHIGH_DEPTH	64
#define FPGA_RAM_MOTOR_TABLE_STOPLOW_DEPTH	32
//
#define FPGA_RAM_MOTOR_TABLE_RAMP_SIZE		(4*FPGA_RAM_MOTOR_TABLE_RAMP_DEPTH)
#define FPGA_RAM_MOTOR_TABLE_COUNT_SIZE		(4*FPGA_RAM_MOTOR_TABLE_COUNT_DEPTH)
#define FPGA_RAM_MOTOR_TABLE_STOPHIGH_SIZE	(4*FPGA_RAM_MOTOR_TABLE_STOPHIGH_DEPTH)
#define FPGA_RAM_MOTOR_TABLE_STOPLOW_SIZE	(4*FPGA_RAM_MOTOR_TABLE_STOPLOW_DEPTH)
//
/* memory map (offset to unit base): */
#define FPGA_RAM_MOTOR_TABLE_RAMP		0x0000
#define FPGA_RAM_MOTOR_TABLE_COUNT		0x1e40
#define FPGA_RAM_MOTOR_TABLE_STOPHIGH		0x1e80
#define FPGA_RAM_MOTOR_TABLE_STOPLOW		0x1f80

/* bits definition of FPGA_RAM_MOTOR_TABLE_RAMP */
#define FPGA_RAM_MOTOR_TABLE_RAMP_ACCEL		0
#define FPGA_RAM_MOTOR_TABLE_RAMP_CONST1	(1<<14)
#define FPGA_RAM_MOTOR_TABLE_RAMP_CONST2	(2<<14)
#define FPGA_RAM_MOTOR_TABLE_RAMP_DECEL		(3<<14)


/* Section 5: DC-motor control registers */
/* memory map (offset to unit base): */
#define FPGA_REG_DCMOTOR_CONTROL		0x0000
#define FPGA_REG_DCMOTOR_STATUS			0x0004

/* bits definition of FPGA_REG_DCMOTOR_STATUS */
#define FPGA_REG_DCMOTOR_RUNNING		BIT(0)
#define FPGA_REG_DCMOTOR_STOPPED_BY_SENSOR	BIT(1)


/* Section 6: CIS control & information registers */
/* memory map (offset): */
#define FPGA_REG_CIS_CONTROL			0x0000
#define FPGA_REG_CIS_T1				0x0004

/* bits definition of FPGA_REG_CIS_CONTROL */
#define FPGA_REG_CIS_SCAN_ENABLE		BIT(0)
#define FPGA_REG_CIS_LEDS_ENABLE		BIT(1)
#define FPGA_REG_CIS_SCAN_TRIGGER_ENABLE	BIT(8)
#define FPGA_REG_CIS_SCANMODE_SIX_LIGHTS	(0x0 << 2)
#define FPGA_REG_CIS_SCANMODE_TEN_LIGHTS	(0x1 << 2)
#define FPGA_REG_CIS_SCANMODE_MASK		(0x3f << 2)

/* Section 7: CIS DPI CONTROL & information registers */
#define FPGA_REG_CIS_DPI_CONTROL	0x0000
#define FPGA_REG_DPI_SI_H		0x0004
#define FPGA_REG_DPI_SI_L		0x0008

/* Section 8: Image ADC control & information registers */
/* memory map (offset): */
#define FPGA_REG_IMGADC_CONTROL		0x0000

/* bits definition of FPGA_REG_IMGADC_CONTROL */
#define FPGA_REG_IMGADC1_ENABLE			BIT(0)
#define FPGA_REG_IMGADC2_ENABLE			BIT(1)
#define FPGA_REG_IMGADC3_ENABLE			BIT(2)
#define FPGA_REG_IMGADC4_ENABLE			BIT(3)
#define FPGA_REG_IMGADC5_ENABLE			BIT(4)
#define FPGA_REG_IMGADC1_CONFIG_ENABLE		BIT(5)
#define FPGA_REG_IMGADC2_CONFIG_ENABLE		BIT(5)
#define FPGA_REG_IMGADC3_CONFIG_ENABLE		BIT(5)
#define FPGA_REG_IMGADC4_CONFIG_ENABLE		BIT(5)
#define FPGA_REG_IMGADC5_CONFIG_ENABLE		BIT(5)


#endif /* __FPGA_H__ */
