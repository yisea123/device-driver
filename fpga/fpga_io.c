/*
 * GWI FPGA low-level IO driver
 *
 * Copyright 2016 Hunan GreatWall Information Financial Equipment Co., Ltd.
 *
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>
#include <linux/busfreq-imx.h>
#include <linux/gpio.h>
#include <linux/firmware.h>
#include <linux/workqueue.h>
#include <../arch/arm/mach-imx/clk.h>
#include <../arch/arm/mach-imx/common.h>
#include <../arch/arm/mach-imx/hardware.h>
#include <asm/uaccess.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/jiffies.h>

#include "../fpga.h"
#include "../fpga_io.h"
//#include "fpgaunit.h"

#define  FPGA_CONFIG_DONE 0
#define  FPGA_CONFIG_INIT 1
#define  FPGA_CONFIG_DATA 2
#define  FPGA_CONFIG_CLK 3
#define  FPGA_CONFIG_PRG 4

#define  FPGA_INT0 0
#define  FPGA_INT1 1
#define  FPGA_INT2 2
#define  FPGA_INT3 3

#define  GPIO_VALUE_HIGH 1
#define  GPIO_VALUE_LOW  0

#if defined(CONFIG_REG_DOWNLOAD)
#define CONFIG_BIT_CLK		BIT(26)
#define CONFIG_BIT_DATA		BIT(29)

#undef  CONFIG_REG_DOWNLOAD_FAST
#define CONFIG_REG_DOWNLOAD_FAST

#if defined(CONFIG_REG_DOWNLOAD_FAST)
#define write_cpu_reg(reg, value)	(*gpio1reg = value)
#else
#define write_cpu_reg(reg, value)	writel(value, reg)
#endif

#endif

struct fpga_interrupt_defs
{
	irq_handler_t fpga_irq_handler0;
	void * sub_dev_id0;             
	irq_handler_t fpga_irq_handler1;
	void * sub_dev_id1;             
	unsigned int irq_number;
	bool benabled; 
};

struct fpga_interrupt_defs fpga_irq_info[4];

static void __iomem *ccm_analog_misc1_addr =  MX6Q_IO_ADDRESS(0x020c8160);
static void __iomem *imx6_reg_iomux_gpr1 =  MX6Q_IO_ADDRESS(0x020e0004);

struct gpio *fpga_config_gpios;
struct gpio *fpga_int_gpios;
static void __iomem *ints_reg_base, *ctrl_reg_base;
u8 bit_mask[8] = {0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01};

static void __iomem *cs0_base, *cs1_base, *cs2_base, *cs3_base;
const struct firmware *fw;
static struct device ghost_device;

struct class    *fpga_class;
struct device	*fpga_dev;
EXPORT_SYMBOL_GPL(fpga_dev);
struct kobject  *fpga_kobj;
dev_t           fpga_dev_no;
unsigned int version;

int download_proc(const struct firmware *fw, const char *fw_name);
static ssize_t version_show(struct device *dev, struct device_attribute *attr, char *buf );
static ssize_t reset_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t config_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

static DEVICE_ATTR(version, S_IRUGO, version_show, NULL);
static DEVICE_ATTR(reset, S_IWUSR, NULL, reset_set);
static DEVICE_ATTR(config, S_IWUSR, NULL, config_set);

static struct attribute *fpga_attr_list[] = {
	&dev_attr_version.attr,  
	&dev_attr_reset.attr,  
	&dev_attr_config.attr,
	NULL,
};

static const struct attribute_group fpga_attr_group = {
	.attrs = (struct attribute **) fpga_attr_list,
};

static ssize_t version_show(struct device *dev, struct device_attribute *attr, char *buf )
{
	size_t count = 0;

	fpga_readl(&version, ctrl_reg_base + FPGA_REG_VERSION);

	count = sprintf(buf, "%04x\n", version);

	return count;
}

static ssize_t reset_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{

	count = fpga_update_lbits(ctrl_reg_base + FPGA_REG_CONTROL, FPGA_REG_CONTROL_RESET, 1);

	return count;
}

static ssize_t config_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{	
	int ret;	

	/*start download fpga*/
	ret = download_proc(fw, "top_check_scanner.bin");
	if(!ret)	
	    printk("fpga download success!\n");
	else
	    printk("fpga download fail!\n");
	
	return count;
}

int fpga_reconfigure(struct device *dev)
{
	return 0;
}
EXPORT_SYMBOL_GPL(fpga_reconfigure);

void __iomem *fpga_io_get(u32 chipsel)
{
	void __iomem* addr = NULL;
	switch (chipsel)
	{
		case 0:
			addr = cs0_base;
			break;
		case 1:
			addr = cs1_base;
			break;
		case 2:
			addr = cs2_base;
			break;
		case 3:
			addr = cs3_base;
			break;
		default:
			return ERR_PTR(-ENODEV);
	}

	return addr;
}
EXPORT_SYMBOL_GPL(fpga_io_get);


void fpga_io_put(void __iomem *addr)
{
}
EXPORT_SYMBOL_GPL(fpga_io_put);


int gpio_init_from_dts(struct device *dev)
{
	int ret;
	struct device_node *np = dev->of_node; 

	fpga_config_gpios = devm_kzalloc(dev, 5*sizeof(struct gpio), GFP_KERNEL);
	if (fpga_config_gpios == NULL)
		return -ENOMEM;

	fpga_int_gpios = devm_kzalloc(dev, 4*sizeof(struct gpio), GFP_KERNEL);
	if (fpga_int_gpios == NULL)
		return -ENOMEM;	

	for_each_node_by_name(np, "fpga_config"){
		fpga_config_gpios[FPGA_CONFIG_DONE].gpio = of_get_named_gpio(np, "config_done_gpio", 0);
		if (!gpio_is_valid(fpga_config_gpios[FPGA_CONFIG_DONE].gpio)) {
			dev_err(dev, "no config_done_gpio pin available\n");
			goto err;
		}
		fpga_config_gpios[FPGA_CONFIG_DONE].flags = GPIOF_IN;
		fpga_config_gpios[FPGA_CONFIG_DONE].label = "config_done_gpio";

		fpga_config_gpios[FPGA_CONFIG_INIT].gpio = of_get_named_gpio(np, "config_init_gpio", 0);
		if (!gpio_is_valid(fpga_config_gpios[FPGA_CONFIG_INIT].gpio)) {
			dev_err(dev, "no config_init_gpio pin available\n");
			goto err;
		}
		fpga_config_gpios[FPGA_CONFIG_INIT].flags = GPIOF_IN;
		fpga_config_gpios[FPGA_CONFIG_INIT].label = "config_init_gpio";

		fpga_config_gpios[FPGA_CONFIG_DATA].gpio = of_get_named_gpio(np, "config_data_gpio", 0);
		if (!gpio_is_valid(fpga_config_gpios[FPGA_CONFIG_DATA].gpio)) {
			dev_err(dev, "no config_data_gpio pin available\n");
			goto err;
		}
		fpga_config_gpios[FPGA_CONFIG_DATA].flags = GPIOF_OUT_INIT_LOW;
		fpga_config_gpios[FPGA_CONFIG_DATA].label = "config_data_gpio";

		fpga_config_gpios[FPGA_CONFIG_CLK].gpio = of_get_named_gpio(np, "config_clock_gpio", 0);
		if (!gpio_is_valid(fpga_config_gpios[FPGA_CONFIG_CLK].gpio)) {
			dev_err(dev, "no config_clock_gpio pin available\n");
			goto err;
		}
		fpga_config_gpios[FPGA_CONFIG_CLK].flags = GPIOF_OUT_INIT_LOW;
		fpga_config_gpios[FPGA_CONFIG_CLK].label = "config_clock_gpio";

		fpga_config_gpios[FPGA_CONFIG_PRG].gpio = of_get_named_gpio(np, "config_program_gpio", 0);
		if (!gpio_is_valid(fpga_config_gpios[FPGA_CONFIG_PRG].gpio)) {
			dev_err(dev, "no config_program_gpio pin available\n");
			goto err;
		}
		fpga_config_gpios[FPGA_CONFIG_PRG].flags = GPIOF_OUT_INIT_HIGH;
		fpga_config_gpios[FPGA_CONFIG_PRG].label = "config_program_gpio";

		ret = gpio_request_array(fpga_config_gpios, sizeof(fpga_config_gpios));
		if(ret)
		    return ret; 

		fpga_int_gpios[FPGA_INT0].gpio = of_get_named_gpio(np, "fpga_int0_gpio", 0);
		if (!gpio_is_valid(fpga_int_gpios[FPGA_INT0].gpio)) {
			dev_err(dev, "no fpga_int0_gpio pin available\n");
			goto err;
		}
		fpga_int_gpios[FPGA_INT0].flags = GPIOF_IN;
		fpga_int_gpios[FPGA_INT0].label = "fpga_int0_gpio";

		fpga_int_gpios[FPGA_INT1].gpio = of_get_named_gpio(np, "fpga_int1_gpio", 0);
		if (!gpio_is_valid(fpga_int_gpios[FPGA_INT1].gpio)) {
			dev_err(dev, "no fpga_int1_gpio pin available\n");
			goto err;
		}
		fpga_int_gpios[FPGA_INT1].flags = GPIOF_IN;
		fpga_int_gpios[FPGA_INT1].label = "fpga_int1_gpio";

		fpga_int_gpios[FPGA_INT2].gpio = of_get_named_gpio(np, "fpga_int2_gpio", 0);
		if (!gpio_is_valid(fpga_int_gpios[FPGA_INT2].gpio)) {
			dev_err(dev, "no fpga_int2_gpio pin available\n");
			goto err;
		}
		fpga_int_gpios[FPGA_INT2].flags = GPIOF_IN;
		fpga_int_gpios[FPGA_INT2].label = "fpga_int2_gpio";

		fpga_int_gpios[FPGA_INT3].gpio = of_get_named_gpio(np, "fpga_int3_gpio", 0);
		if (!gpio_is_valid(fpga_int_gpios[FPGA_INT3].gpio)) {
			dev_err(dev, "no fpga_int3_gpio pin available\n");
			goto err;
		}
		fpga_int_gpios[FPGA_INT3].flags = GPIOF_IN;
		fpga_int_gpios[FPGA_INT3].label = "fpga_int3_gpio";
	}

	ret = gpio_request_array(fpga_int_gpios, sizeof(fpga_int_gpios));
	if(ret)
	    return ret; 
err:
	return 0;

}


static inline void fpga_reset_interrupts(void)
{
	// disable all FPGA interrupts(enale global only)
	fpga_writel(FPGA_REG_INT_SEL_ALL, ints_reg_base + FPGA_REG_INT_ENABLE);
	// clear all FPGA interrupts
	fpga_writel((FPGA_REG_INT_SEL_TYPE1|FPGA_REG_INT_SEL_TYPE2|FPGA_REG_INT_SEL_TYPE3|FPGA_REG_INT_SEL_TYPE4|FPGA_REG_INT_SEL_TYPE5),
		ints_reg_base + FPGA_REG_INT_CLEAR);
}

/* download fpga bin data */
int download_proc(const struct firmware *fw, const char *fw_name)
{
	unsigned int i; 
	int gpio_clock, gpio_data;
	unsigned long before, after;

	printk(KERN_DEBUG "fw_name: %s\n", fw_name);
	if(request_firmware(&fw, fw_name, &ghost_device))
	{
	    printk(KERN_ERR "request_firmware err!\n");
	    return -ENOENT;
	}
	printk(KERN_INFO "FPGA binary size: %d\n", fw->size);

	gpio_direction_input(fpga_config_gpios[FPGA_CONFIG_DONE].gpio);
	gpio_direction_input(fpga_config_gpios[FPGA_CONFIG_INIT].gpio);
	gpio_direction_output(fpga_config_gpios[FPGA_CONFIG_DATA].gpio, GPIOF_OUT_INIT_LOW);
	gpio_direction_output(fpga_config_gpios[FPGA_CONFIG_CLK].gpio,  GPIOF_OUT_INIT_LOW);
	gpio_direction_output(fpga_config_gpios[FPGA_CONFIG_PRG].gpio,  GPIOF_OUT_INIT_HIGH);

	gpio_set_value(fpga_config_gpios[FPGA_CONFIG_PRG].gpio,  GPIO_VALUE_HIGH);
	gpio_set_value(fpga_config_gpios[FPGA_CONFIG_CLK].gpio,  GPIO_VALUE_LOW);
	gpio_set_value(fpga_config_gpios[FPGA_CONFIG_DATA].gpio, GPIO_VALUE_LOW);

	gpio_set_value(fpga_config_gpios[FPGA_CONFIG_PRG].gpio,  GPIO_VALUE_LOW);

	i = 0xFFF;
	while(gpio_get_value(fpga_config_gpios[FPGA_CONFIG_INIT].gpio) && --i);/*quest: FPGA_CONFIG_INIT is high? */ 
	printk(KERN_DEBUG "gpio_get_value(FPGA_CONFIG_INIT)=%d\n", gpio_get_value(fpga_config_gpios[FPGA_CONFIG_INIT].gpio));	   
	if(i == 0)
	{
		return 0;
	}

	gpio_set_value(fpga_config_gpios[FPGA_CONFIG_PRG].gpio, GPIO_VALUE_HIGH);/*prg set  high*/

	i= 0xFFF;
	while(!gpio_get_value(fpga_config_gpios[FPGA_CONFIG_INIT].gpio) && --i);/*again quest: FPGA_CONFIG_INIT is high? yes, go on*/
	if(i == 0)
	{
		return 0;
	}

	gpio_data = fpga_config_gpios[FPGA_CONFIG_DATA].gpio;
	gpio_clock = fpga_config_gpios[FPGA_CONFIG_CLK].gpio;

	before = jiffies;

#if defined(CONFIG_REG_DOWNLOAD)
	{
		volatile u32 __iomem *gpio1reg = IOMEM(0xc0810000);
		register u8 *ptr;
		register u32 val;
		val = readl(gpio1reg);
		ptr = (u8 *)fw->data;
		for (i = 0; i < fw->size; i++)
		{
			register u8 data = *ptr++;
			register int j;
			local_irq_disable();
			for (j = 0; j < 8; j++)
			{
				val &= ~CONFIG_BIT_DATA;
				val |= (data & 0x80) ? CONFIG_BIT_DATA : 0;
				val |= CONFIG_BIT_CLK;
				write_cpu_reg(gpio1reg, val);
				val &= ~CONFIG_BIT_CLK;
				data <<= 1;
				write_cpu_reg(gpio1reg, val);
			}
			local_irq_enable();
		}
	}
#else
	for(i = 0; i < fw->size; i++)
	{
		u8 data = fw->data[i];
		int j;
		local_irq_disable();
		for (j = 0; j < 8; j++)
		{
			int bit = (data & 0x80) ? GPIO_VALUE_HIGH : GPIO_VALUE_LOW;

			data <<= 1;
			gpio_set_value(gpio_clock, GPIO_VALUE_LOW);
			gpio_set_value(gpio_data, bit);
			gpio_set_value(gpio_clock, GPIO_VALUE_HIGH);
			gpio_set_value(gpio_data, bit);
		}
		local_irq_enable();
	}
#endif
	after = jiffies;
	printk(KERN_DEBUG "download time = %d (ms): %u -> %u\n", jiffies_to_msecs(after-before), jiffies_to_msecs(before), jiffies_to_msecs(after));
	printk(KERN_INFO "fpga configuration end\n");

	i = 0xFFF;
	while(!gpio_get_value(fpga_config_gpios[FPGA_CONFIG_DONE].gpio) && --i);

	release_firmware(fw);
	if(i == 0)
	{
		return 1;
	}
	else
	{
		return 0; 
	}        
}
EXPORT_SYMBOL_GPL(download_proc);

/*
irq--0----image ADC interrupt	--- INT0
irq--1--- image data interrupt  --- INT1
irq--2--- reserverd		--- INT2
irq--3--- motor interrupts	--- INT3
*/
//int fpga_request_irq(unsigned int irq, irq_handler_t handler)
int fpga_request_irq(unsigned int irq, irq_handler_t handler, void * sub_dev_id)
{
	if (irq < 0 || irq > 4)
		return -1;

	fpga_irq_info[irq].fpga_irq_handler0 = handler;
        fpga_irq_info[irq].sub_dev_id0 = sub_dev_id;

	if (!fpga_irq_info[irq].benabled) 
	{
		u32 int_mask;
		void __iomem *fpga_reg_int_enable = ints_reg_base + FPGA_REG_INT_ENABLE;

		enable_irq(fpga_irq_info[irq].irq_number);
		fpga_irq_info[irq].benabled = true;
		fpga_readl(&int_mask, fpga_reg_int_enable);
		int_mask |= BIT(irq);
		fpga_writel(int_mask, fpga_reg_int_enable);
	}

//	printk(KERN_INFO "fpga irq enabled,  irq: %d, int_number: %d handler:%x\n", irq, fpga_irq_info[irq].irq_number, handler); 
	return 0;
}
EXPORT_SYMBOL_GPL(fpga_request_irq);

int fpga_free_irq(unsigned int irq)
{
	int int_number;
	if (irq < 0 || irq > 4)
		return -1;

	if (irq == 0 || irq == 1)
		int_number = 0;

	if (irq == 2 || irq == 4)
		int_number = 1;

	if (irq == 3)
		int_number = 3;

	disable_irq(fpga_irq_info[int_number].irq_number);
	fpga_irq_info[int_number].benabled = false;

	return 0;
}
EXPORT_SYMBOL_GPL(fpga_free_irq);

static const struct of_device_id fpga_io_match[] = {
	{ .compatible = "gwi,fpga-io", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, fpga_io_match);

#ifdef CONFIG_FPGA_SPI

struct fpga_spi_work_def
{
	struct device *dev;
	struct work_struct fpga_spi_wq;
	int irq;
};

struct fpga_spi_work_def fpga_spi_work = {
	.dev = NULL,
	.irq = -1,
};

void fpga_spi_do_work(struct work_struct *work)
{
	int fpga_int_status = 0,ret,i;
	int fpga_cur_int = -1;
	void __iomem *fpga_regs_status = fpga_io_get(2) + FPGA_REG_INT_STATUS; 
	int irq_0_mux = 0;

	struct fpga_spi_work_def *spi_work = container_of(work, struct fpga_spi_work_def, fpga_spi_wq);

	for (i=0;i<4;i++)
	{
		if (fpga_irq_info[i].irq_number == spi_work->irq) 
		{
			fpga_cur_int = i;
			break;
		}
	}

//	printk("fpga_isr spi fpga_cur_int: %d\n", fpga_cur_int);

	if (fpga_cur_int == -1) 
	{
		printk("fpga isr irq not match\n");
		return;
	}

	ret = fpga_readl(&fpga_int_status, fpga_regs_status);
	if (ret)
	{
		printk("fpga_read fpga int status error, return: %d\n",ret);
		return;
	}

	if (fpga_cur_int == 0)
	{
		if (fpga_int_status & 0x01)
			irq_0_mux = 0;
		else if (fpga_int_status & 0x02)
			irq_0_mux = 1;
		else
		{
			printk("fpga int status error, status: %x\n", fpga_int_status);
			return;
		}
	}

	if (fpga_cur_int == 1)
	{
		if (fpga_int_status & 0x04)
			irq_0_mux = 0;
		else if (fpga_int_status & 0x10)
			irq_0_mux = 1;
		else
		{
			printk("fpga int status error, status: %x\n", fpga_int_status);
			return;
		}

	}

	if (fpga_cur_int == 3)
	{
		if (fpga_int_status & 0x08)
			irq_0_mux = 0;
		else
		{
			printk("fpga int status error, status: %x\n", fpga_int_status);
			return;
		}
	}

	printk("fpga_isr clear_int ok !!\n");

	//call intrrupt handler
	if (irq_0_mux == 0)
		fpga_irq_info[fpga_cur_int].fpga_irq_handler0(spi_work->irq, spi_work->dev);
	else
		fpga_irq_info[fpga_cur_int].fpga_irq_handler1(spi_work->irq,spi_work->dev);
	return;
}
#endif

static irqreturn_t fpga_isr(int irq, void *dev_id)
{

#ifdef CONFIG_FPGA_SPI
	struct fpga_spi_work_def* spi_work = NULL;
	printk("fpga isr spi enter, irq: %d\n", irq);

	spi_work = (struct fpga_spi_work_def*)dev_id;
	spi_work->irq = irq;

	schedule_work(&spi_work->fpga_spi_wq); 
	return IRQ_HANDLED;
#else
	u32 mask;
	int fpga_int_status = 0,ret,i;
	int fpga_cur_int = -1;
	void __iomem *fpga_regs_int_status = ints_reg_base + FPGA_REG_INT_STATUS;
	void __iomem *fpga_regs_int_clear = ints_reg_base + FPGA_REG_INT_CLEAR;
	irqreturn_t irq_ret = IRQ_HANDLED;


	//printk("fpga_isr ----------------\n");


	ret = fpga_readl(&fpga_int_status, fpga_regs_int_status);
	if (ret)
	{
//		printk("KERN_INFO fpga_read fpga int status error, return: %d\n",ret);
		return IRQ_HANDLED;
	}
	//printk("fpga_int_status:%x\n", fpga_int_status);
	if (fpga_int_status == 0)
	{
//		printk("fpga int status error\n");
		return IRQ_HANDLED;
	}
	fpga_writel(fpga_int_status, fpga_regs_int_clear);
	fpga_cur_int = 0;
	mask = FPGA_REG_INT_SEL_TYPE1;
	for (i=0; i<4; i++) {
		if ((fpga_int_status & mask) && (fpga_irq_info[fpga_cur_int].fpga_irq_handler0 != NULL)) {
			if (fpga_irq_info[fpga_cur_int].sub_dev_id0 == NULL) {
				//printk("fpga_isr fpga_cur_int1: %d\n", fpga_cur_int);
			    irq_ret = fpga_irq_info[fpga_cur_int].fpga_irq_handler0(irq, dev_id);
			}
			else{
				//printk("fpga_isr fpga_cur_int2: %d\n", fpga_cur_int);
			    irq_ret = fpga_irq_info[fpga_cur_int].fpga_irq_handler0(irq, fpga_irq_info[fpga_cur_int].sub_dev_id0);
			}
		}
		mask <<= 1;
		++fpga_cur_int;
	}
	return irq_ret;
}
#endif

int fpga_request_irq_fun(struct device *dev)
{
	int fpga_int0,fpga_int1,fpga_int2,fpga_int3;
	int ret;

	void* dev_id = NULL;

#ifdef CONFIG_FPGA_SPI
	dev_id = &fpga_spi_work;
#else
	dev_id = dev;
#endif
	fpga_int0 = gpio_to_irq(fpga_int_gpios[FPGA_INT0].gpio);
	ret = devm_request_irq(dev, fpga_int0, fpga_isr, IRQF_TRIGGER_RISING,dev_name(dev), dev_id);
	if (ret)
	{
		dev_err(dev, "failsed requesting fpga irq0, irq = %d\n", fpga_int0);
		return ret;
	}
	disable_irq(fpga_int0);
	fpga_irq_info[0].irq_number = fpga_int0;
	fpga_irq_info[0].fpga_irq_handler0= NULL;
	fpga_irq_info[0].fpga_irq_handler1= NULL;
	fpga_irq_info[0].benabled = false;

	fpga_int1 = gpio_to_irq(fpga_int_gpios[FPGA_INT1].gpio);
	ret = devm_request_irq(dev, fpga_int1, fpga_isr, IRQF_TRIGGER_RISING,dev_name(dev), dev_id);
	if (ret)
	{
		dev_err(dev, "failsed requesting fpga irq1, irq = %d\n", fpga_int1);
		return ret;
	}
	disable_irq(fpga_int1);
	fpga_irq_info[1].irq_number = fpga_int1;
	fpga_irq_info[1].fpga_irq_handler0= NULL;
	fpga_irq_info[1].fpga_irq_handler1= NULL;
	fpga_irq_info[1].benabled = false;

	fpga_int2 = gpio_to_irq(fpga_int_gpios[FPGA_INT2].gpio);
	ret = devm_request_irq(dev, fpga_int2, fpga_isr, IRQF_TRIGGER_RISING,dev_name(dev), dev_id);
	if (ret)
	{
		dev_err(dev, "failsed requesting fpga irq2, irq = %d\n", fpga_int2);
		return ret;
	}
	disable_irq(fpga_int2);
	fpga_irq_info[2].irq_number = fpga_int2;
	fpga_irq_info[2].fpga_irq_handler0= NULL;
	fpga_irq_info[2].fpga_irq_handler1= NULL;
	fpga_irq_info[2].benabled = false;

	fpga_int3 = gpio_to_irq(fpga_int_gpios[FPGA_INT3].gpio);
	ret = devm_request_irq(dev, fpga_int3, fpga_isr, IRQF_TRIGGER_RISING,dev_name(dev), dev_id);
	if (ret)
	{
		dev_err(dev, "failsed requesting fpga irq3, irq = %d\n", fpga_int3);
		return ret;
	}
	disable_irq(fpga_int3);
	fpga_irq_info[3].irq_number = fpga_int3;
	fpga_irq_info[3].fpga_irq_handler0= NULL;
	fpga_irq_info[3].fpga_irq_handler1= NULL;
	fpga_irq_info[3].benabled = false;
	dev_info(dev, "FPGA irq number = %d,%d,%d,%d\n", fpga_int0, fpga_int1, fpga_int2, fpga_int3);
	gpio_direction_input(fpga_int_gpios[FPGA_INT0].gpio);
	gpio_direction_input(fpga_int_gpios[FPGA_INT1].gpio);
	gpio_direction_input(fpga_int_gpios[FPGA_INT2].gpio);
	gpio_direction_input(fpga_int_gpios[FPGA_INT3].gpio);

	return 0;
}


static int fpga_clock_core_init(struct device *dev, struct clk *fpga_bus, struct clk *fpga_clk)
{
	int tmp, ret;
	
	tmp = readl(ccm_analog_misc1_addr);
	tmp |= 0x00000252;

	writel(tmp, ccm_analog_misc1_addr);		

	request_bus_freq(BUS_FREQ_HIGH);		
	
	ret = clk_prepare_enable(fpga_bus);
	if (ret) {
		dev_err(dev, "unable to enable fpga_bus clock\n");
		goto err_fpga_bus;
	}

	ret = clk_prepare_enable(fpga_clk);
	if (ret) {
		dev_err(dev, "unable to enable fpga_clk clock\n");
		goto err_fpga_clk;
	}

	tmp = readl(ccm_analog_misc1_addr);
	return 0;

err_fpga_bus:
 	clk_disable_unprepare(fpga_bus);

err_fpga_clk:
 	clk_disable_unprepare(fpga_clk);
	return 0;
}

int fpga_clk_fetch(struct device *dev)
{
	struct clk *fpga_bus = NULL;
	struct clk *fpga_clk = NULL;
	struct device_node *np = dev->of_node;
	int ret;

	/* Fetch clocks */
	for_each_node_by_name(np, "fpga_config"){
		
		fpga_bus = of_clk_get_by_name(np, "fpga_bus");
		if (IS_ERR(fpga_bus)) {
			dev_err(dev,"fpga_bus clock source missing or invalid\n");
			return PTR_ERR(fpga_bus);
		}

		fpga_clk = of_clk_get_by_name(np, "fpga_clk");
		if (IS_ERR(fpga_clk)) {
			dev_err(dev,"fpga_clk clock source missing or invalid\n");
			return PTR_ERR(fpga_clk);
		}
	}

	ret = fpga_clock_core_init(dev, fpga_bus, fpga_clk);
	if(ret)
	{
		printk(KERN_INFO "fpga_clock_core_init err!\n");
		return ret;
	}
	printk(KERN_INFO "fpga_clock_core_init!\n");
	
	return 0;

}
#ifdef CONFIG_FPGA_SPI

struct regmap *fpga_reg = NULL;
#define FPGA_MAX_REG 0x094B

static const struct regmap_config fpga_spi_regmap = {
	.reg_bits = 24,
	.val_bits = 32,
};

int fpga_readl(u32 *value, const volatile void __iomem *addr)
{
	u32 val;
	int ret;

	ret = regmap_read(fpga_reg, (u32)addr, &val);
	*value = val;

	return ret;
}
EXPORT_SYMBOL_GPL(fpga_readl);

int fpga_writel(u32 value, volatile void __iomem *addr)
{
	int ret;
	ret = regmap_write(fpga_reg, (u32)addr, value);
	return ret;
}	
EXPORT_SYMBOL_GPL(fpga_writel);

int fpga_readnl(const void __iomem *addr, void *buffer, int count)
{
	int ret;

	printk("fpga_readnl addr: %x\n",(u32)addr);

	ret = regmap_bulk_read(fpga_reg, (u32)addr, buffer, count);

	printk("fpga_readnl return: %d\n",ret);

	return ret;
}
EXPORT_SYMBOL_GPL(fpga_readnl);

int fpga_writenl(void __iomem *addr, const void *buffer, int count)
{
	int ret;

	ret = regmap_bulk_write(fpga_reg, (u32)addr, buffer, count);

	return ret;
}
EXPORT_SYMBOL_GPL(fpga_writenl);

int fpga_update_lbits(volatile void __iomem *addr, u32 mask, u32 value)
{
	int ret;
	ret = regmap_update_bits(fpga_reg, (u32)addr, mask, value);
	return ret;
}
EXPORT_SYMBOL_GPL(fpga_update_lbits)

static int fpga_io_spi_probe(struct spi_device *spi)
{
	int ret;

	cs0_base = IOMEM(0x010000);
	cs1_base = IOMEM(0x020000);
	cs2_base = IOMEM(0x040000);
	cs3_base = IOMEM(0x080000);

	printk("fpga_io_spi_probe enter!\n");

	if (fpga_reg != NULL)
	{
		printk("fpga_io_spi_probe fpga_reg is non null!\n");

		ret = PTR_ERR(fpga_reg);
		dev_err(&spi->dev, "register map is already exist: %d\n", ret);
		return ret;
	}

	ret =  gpio_init_from_dts(&spi->dev);
	if (ret)
	{
		dev_err(&spi->dev, "Failed to init gpio from dts: %d\n", ret);
		return ret;
	}

	/*start download fpga*/
	ret = download_proc(fw, "top_check_scanner.bin");
	if(ret)	
	    printk("fpga download success!\n");
	else
	    printk("fpga download fail!\n");

	//初始化工作队列
	fpga_spi_work.dev = &spi->dev;
	INIT_WORK(&fpga_spi_work.fpga_spi_wq, fpga_spi_do_work); 

	ret = fpga_request_irq_fun(&spi->dev);
	if (ret)
	{
		return ret;
	}

	ret = fpga_clk_fetch(&spi->dev);
	if (ret)
	{
		dev_err(&spi->dev, "Failed to fetch fpga clk: %d\n",ret);
		return ret;
	}

	spi->mode = SPI_MODE_1;

	fpga_reg = devm_regmap_init_spi(spi, &fpga_spi_regmap);
	if (IS_ERR(fpga_reg)) {
		ret = PTR_ERR(fpga_reg);
		dev_err(&spi->dev, "Failed to allocate register map: %d\n",ret);
		return ret;
	}

	regcache_cache_bypass(fpga_reg, true);

	return 0;
}

static int fpga_io_spi_remove(struct spi_device *spi)
{
	gpio_free_array(fpga_config_gpios, sizeof(fpga_config_gpios));
	gpio_free_array(fpga_int_gpios, sizeof(fpga_int_gpios));
	printk(KERN_INFO "fpga_remove\n");	
	return 0;
}

static struct spi_driver fpga_io_spi_driver = {
	.driver		= {
		.name	= "fpga-io",
		.of_match_table = fpga_io_match,
	},
	.probe		= fpga_io_spi_probe,
	.remove		= fpga_io_spi_remove,
};

module_spi_driver(fpga_io_spi_driver);

#else

int fpga_chipbase_get(struct platform_device *pdev)
{
	struct resource *mem0_res, *mem1_res, *mem2_res, *mem3_res;	
	/* get the resource */
	mem0_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "fpga-cs0");
	if (mem0_res == NULL) {
		dev_err(&pdev->dev, "missing platform resources mem0_res data\n");
		return -ENODEV;
	}

	cs0_base = devm_ioremap(&pdev->dev, mem0_res->start, resource_size(mem0_res));
	printk(KERN_INFO "base = %x\n", (u32)cs0_base);
	if (IS_ERR(cs0_base))
		return PTR_ERR(cs0_base);


	mem1_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "fpga-cs1");
	if (mem1_res == NULL) {
		dev_err(&pdev->dev, "missing platform  resources mem1_res data\n");
		return -ENODEV;
	}
	cs1_base = devm_ioremap(&pdev->dev, mem1_res->start, resource_size(mem1_res));
	printk(KERN_INFO "cs1_base = %x\n", (u32)cs1_base);
	if (IS_ERR(cs1_base))
		return PTR_ERR(cs1_base);

	mem2_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "fpga-cs2");
	cs2_base = devm_ioremap_resource(&pdev->dev, mem2_res);
	printk(KERN_INFO "cs2_base = %x\n", (u32)cs2_base);
	if (IS_ERR(cs2_base))
		return PTR_ERR(cs2_base);


	mem3_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "fpga-cs3");
	cs3_base = devm_ioremap_resource(&pdev->dev, mem3_res);
	printk(KERN_INFO "cs3_base = %x\n", (u32)cs3_base);
	if (IS_ERR(cs3_base))
		return PTR_ERR(cs3_base);

	return 0;
}


int fpga_io_creat_dev(void)
{
	int status;	

	fpga_class = class_create(THIS_MODULE, "fpga");
	if(IS_ERR(fpga_class))
		return PTR_ERR(fpga_class);
	
	fpga_dev = device_create(fpga_class, NULL, fpga_dev_no, NULL, "fpga");
	if(IS_ERR(fpga_dev))
		return PTR_ERR(fpga_dev);	

	status = sysfs_create_group(&fpga_dev->kobj, &fpga_attr_group);		

	return status;
}

static int fpga_io_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device_node *np = pdev->dev.of_node;
	u32 reg[3];
	u32 fpga_cs;
	u32 tmp1, tmp2;

	printk(KERN_INFO "Fpga EIM Driver - Copyright GWI\n");

	tmp1 = readl(imx6_reg_iomux_gpr1);
	tmp2 = tmp1 & ~0xfff;
	tmp2 |= 0x249;

	writel(tmp2, imx6_reg_iomux_gpr1);
	tmp2 = readl(imx6_reg_iomux_gpr1);
	printk(KERN_INFO "IOMUX GPR1 = %08x -> %08x\n", tmp1, tmp2);

	ret = fpga_chipbase_get(pdev);
	if (ret)
	{
		dev_err(&pdev->dev, "Failed to get fpga chipsel: %d\n",ret);
		return ret;
	}

	ret = of_property_read_u32_array(np, "reg-ints", reg, 3);
	if (IS_ERR_VALUE(ret)) {
		dev_err(&pdev->dev, "invalid FPGA INT registers  definition\n");
		return ret;
	}
	fpga_cs = reg[0];
	ints_reg_base = fpga_io_get(fpga_cs);
	if (IS_ERR(ints_reg_base))
		return PTR_ERR(ints_reg_base);
	ints_reg_base += reg[1];
	printk("ints_reg_base = %x\n", (u32)ints_reg_base);

	ret = of_property_read_u32_array(np, "reg-control", reg, 3);
	if (IS_ERR_VALUE(ret)) {
		dev_err(&pdev->dev, "invalid FPGA INT registers  definition\n");
		return ret;
	}
	fpga_cs = reg[0];
	ctrl_reg_base = fpga_io_get(fpga_cs);
	if (IS_ERR(ctrl_reg_base))
		return PTR_ERR(ctrl_reg_base);
	ctrl_reg_base += reg[1];
	printk("ctrl_reg_base = %x\n", (u32)ctrl_reg_base);

	/* Fetch clocks */
	ret = fpga_clk_fetch(&pdev->dev);
	if (ret)
	{
		dev_err(&pdev->dev, "Failed to fetch fpga clk: %d\n",ret);
		return ret;
	}

	ret =  gpio_init_from_dts(&pdev->dev);
	if (ret)
	{
		dev_err(&pdev->dev, "Failed to init gpio from dts: %d\n",ret);
		return ret;
	}

	device_initialize(&ghost_device);
	/*start download fpga*/
	ret = download_proc(fw, "top_check_scanner.bin");
	if(ret == 0)	
	    printk(KERN_INFO "fpga download success!\n");
	else
	    printk(KERN_INFO "fpga download fail!\n");

	mdelay(200);
	fpga_reset_interrupts();

	ret = fpga_request_irq_fun(&pdev->dev);
	if (ret)
		return ret;

	ret = fpga_io_creat_dev();
	if (ret)
		return ret;
	{
		u32 int_mask;
		fpga_readl(&int_mask, ints_reg_base + FPGA_REG_INT_ENABLE);
		printk(KERN_INFO "fpga int_mask=%08x\n", int_mask);
	}
	return 0;
}

static int fpga_io_remove(struct platform_device *pdev)
{
	fpga_reset_interrupts();
	gpio_free_array(fpga_int_gpios, sizeof(fpga_int_gpios));
	gpio_free_array(fpga_config_gpios, sizeof(fpga_config_gpios));

	sysfs_remove_group(&fpga_dev->kobj, &fpga_attr_group);
	device_destroy(fpga_class, fpga_dev_no);
	class_destroy(fpga_class);

	printk(KERN_INFO "fpga_remove\n");	
	return 0;
}

static struct platform_driver fpga_io_driver = {
	.driver		= {
		.name	= "fpga-io",
		.of_match_table = fpga_io_match,
	},
	.probe		= fpga_io_probe,
	.remove		= fpga_io_remove,
};

module_platform_driver(fpga_io_driver);
#endif

MODULE_AUTHOR("xxx <xxx@gwi.com.cn>");
MODULE_DESCRIPTION("GWI FPGA low-level IO driver");
MODULE_LICENSE("GPL v2");
