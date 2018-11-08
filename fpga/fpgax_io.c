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

#include "../fpga.h"
#include "../fpga_io.h"
#include "../fpgax_io.h"


//#define  FPGAX_INT0 0


#define  GPIO_VALUE_HIGH 1
#define  GPIO_VALUE_LOW  0

struct fpga_interrupt_defs
{
	irq_handler_t fpga_irq_handler0;
	void * sub_dev_id0;             
	irq_handler_t fpga_irq_handler1;
	void * sub_dev_id1;             
	unsigned int irq_number;
	bool benabled; 
};

struct fpgax_spi {
     struct spi_device   *spi;
     struct spi_transfer xfer[2];
     struct spi_message  msg;
};

struct fpgax_spi *data;

struct fpga_interrupt_defs fpgax_irq_info[4];


//struct gpio *fpgax_int_gpios;
int fpgax_int_gpios;
static void __iomem *ints_reg_base, *ctrl_reg_base;
u8 bit_mask[8] = {0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01};

static void __iomem *cs0_base, *cs1_base, *cs2_base, *cs3_base, *cs4_base, *cs5_base;
const struct firmware *fw;
static struct device ghost_device;

struct class    *fpgax_class;
struct device	*fpgax_dev;
EXPORT_SYMBOL_GPL(fpgax_dev);
struct kobject  *fpgax_kobj;
dev_t           fpgax_dev_no;
unsigned int version;
unsigned int regval;
int writetesttimes;
u32 offset, value;
static ssize_t versionx_show(struct device *dev, struct device_attribute *attr, char *buf );
static ssize_t resetx_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t reg_read(struct device *dev, struct device_attribute *attr, char *buf );
static ssize_t reg_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t address_show(struct device *dev, struct device_attribute *attr, char *buf );
static ssize_t address_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t value_show(struct device *dev, struct device_attribute *attr, char *buf );
static ssize_t value_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);


static ssize_t writetest_show(struct device *dev, struct device_attribute *attr, char *buf );
static ssize_t writetest_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);


static DEVICE_ATTR(versionx, S_IRUGO, versionx_show, NULL);
static DEVICE_ATTR(resetx, S_IWUSR, NULL, resetx_set);
static DEVICE_ATTR(regwrx, S_IRUGO|S_IWUSR, reg_read, reg_write);
static DEVICE_ATTR(addressx, S_IRUGO | S_IWUSR, address_show, address_set);
static DEVICE_ATTR(valuex, S_IRUGO | S_IWUSR, value_show, value_set);
//static DEVICE_ATTR(RAMtestx, S_IRUGO | S_IWUSR, value_show, value_set);
static DEVICE_ATTR(writetestx, S_IRUGO | S_IWUSR, writetest_show, writetest_set);

static struct attribute *fpga_attr_list[] = {
	&dev_attr_versionx.attr,  
	&dev_attr_resetx.attr,
        &dev_attr_regwrx.attr, 
	&dev_attr_addressx.attr,  
	&dev_attr_valuex.attr, 
	&dev_attr_writetestx.attr,           
	NULL,
};

static const struct attribute_group fpgax_attr_group = {
	.attrs = (struct attribute **) fpga_attr_list,
};



static ssize_t writetest_show(struct device *dev, struct device_attribute *attr, char *buf )
{
	size_t count = 0;
	count = sprintf(buf, "0x%08x\n", writetesttimes);
	return count;
}

static ssize_t writetest_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{	
	int rs;	
	u32 i = 0, j=0, cs = 0; 
	u32 addr = 0;
	void __iomem *fpga_regs = NULL;
	u32 ww,rw;

	//cs = offset >> 16;
	//addr = 0xffff & offset;
	fpga_regs = fpgax_io_get(2);
	printk("fpga_regs = %x\n", (u32)fpga_regs);

	rs = sscanf(buf, "%d\n", &writetesttimes);	
	printk("writetesttimes set is %d\n",  writetesttimes);

	while (i < writetesttimes) {
		/*ww = (i+5) & 0xffff;

                for (j=0; j<0x80; j++) {
                    ww = (i+j+5) & 0xffff;

                    fpgax_writel(ww, fpga_regs+4*j);
                    //mdelay(1); 
                    fpgax_readl(&rw, fpga_regs+4*j);

                    if (rw != ww) {
    			printk("ERROR: %d time test: address %08x read: %08x , expected: %08x\n", i, (u32)(fpga_regs)+4*j, rw, ww);
    			return count;
                    }
                }*/
            fpgax_writel(0, fpga_regs);
            for (j=0; j<16; j++) {
                fpgax_readl(&rw, fpga_regs);
                printk("before update val is %x\n", rw);
                fpgax_update_lbits(fpga_regs, 1<<j, 1<<j);
                fpgax_readl(&rw, fpga_regs);
                printk("**************************before update val is %x\n", rw);
            }
            
            i++;
	}
	if (i == writetesttimes) {
		printk("total %d test times is ok \n",  writetesttimes);
	}
	return count;
}



static ssize_t versionx_show(struct device *dev, struct device_attribute *attr, char *buf )
{
	size_t count = 0;

	fpgax_readl(&version, fpgax_io_get(0) + FPGA_REG_VERSION);

	count = sprintf(buf, "%04x\n", version);

	return count;
}

static ssize_t resetx_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{

	fpgax_update_lbits(fpgax_io_get(0) + FPGA_REG_CONTROL, FPGA_REG_CONTROL_RESET, 1);

	return count;
}

static ssize_t reg_read(struct device *dev, struct device_attribute *attr, char *buf )
{
        int i;
	size_t count = 0;
        for (i=0; i<30; i++) {

            fpgax_readl(&regval, (fpgax_io_get(5) + i*4));
            printk("%04x\n", regval);
            count = sprintf(buf, "%04x\n", regval);
        }
        //fpgax_readl(&regval, fpgax_io_get(2) + FPGA_REG_SENSOR_ADC_COMPARE_MODE); 



	return count*30;
}

static ssize_t reg_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int rs;	
	
	rs = sscanf(buf, "%x\n", &regval);
		
	printk("\n regval set = 0x%08x\n",  regval);

	fpgax_writel(regval, fpgax_io_get(2) + FPGA_REG_SENSOR_ADC_COMPARE_MODE);


	//fpgax_writel(&version, 0x2000);//fpgax_io_get(2) + FPGA_REG_SENSOR_ADC_COMPARE_MODE);

	return count;
}


static ssize_t address_show(struct device *dev, struct device_attribute *attr, char *buf )
{
	size_t count = 0;
	count = sprintf(buf, "0x%08x\n", offset);
	return count;
}

static ssize_t address_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{	
	int rs;	
        //printk("\n address set = 0x%08x\n",  offset);
	rs = sscanf(buf, "%x\n", &offset);	
	return count;
}

static ssize_t value_show(struct device *dev, struct device_attribute *attr, char *buf )
{
	size_t count = 0;
	u32 addr = 0;
	void __iomem *fpga_regs = NULL;

	addr = 0xffff & offset;
	fpga_regs = fpgax_io_get(0) + addr;
	fpgax_readl(&value,fpga_regs);	
	count = sprintf(buf, "%08x\n", value);
//        printk("value = %x, and fpga_regs=%x\n", value,fpga_regs);
	return count;
}

static ssize_t value_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int rs;	
        u32 addr = 0;
	void __iomem *fpga_regs = NULL;

	addr = 0xffff & offset;
	fpga_regs = fpgax_io_get(0) + addr;

	rs = sscanf(buf, "%x\n", &value);	
	fpgax_writel(value, fpga_regs);
//        printk("\n value set = 0x%08x and fpga_regs=%x\n",  value, fpga_regs);
	return count;
}

static inline void fpgax_reset_interrupts(void)
{
	// disable all FPGA interrupts(enale global only)
	fpgax_writel(FPGA_REG_INT_SEL_ALL, ints_reg_base + FPGA_REG_INT_ENABLE);
	// clear all FPGA interrupts
	fpgax_writel((FPGA_REG_INT_SEL_TYPE1|FPGA_REG_INT_SEL_TYPE2|FPGA_REG_INT_SEL_TYPE3|FPGA_REG_INT_SEL_TYPE4|FPGA_REG_INT_SEL_TYPE5),
		ints_reg_base + FPGA_REG_INT_CLEAR);
}


/*
irq--0----image ADC interrupt	--- INT0
irq--1--- image data interrupt  --- INT1
irq--2--- reserverd		--- INT2
irq--3--- motor interrupts	--- INT3
*/
//int fpga_request_irq(unsigned int irq, irq_handler_t handler)
int fpgax_request_irq(unsigned int irq, irq_handler_t handler, void * sub_dev_id)
{
	if (irq < 0 || irq > 4)
		return -1;

	fpgax_irq_info[irq].fpga_irq_handler0 = handler;
        printk("\nfpgax_request_irq:irq=%d, fpgax_irq_info[irq].fpga_irq_handler0=%x\n", irq, fpgax_irq_info[irq].fpga_irq_handler0);
        fpgax_irq_info[irq].sub_dev_id0 = sub_dev_id;
        printk("fpgax_irq_info[irq].sub_dev_id0=%x\n", fpgax_irq_info[irq].sub_dev_id0);

	if (!fpgax_irq_info[irq].benabled) 
	{
		u32 int_mask;
		void __iomem *fpga_reg_int_enable = ints_reg_base + FPGA_REG_INT_ENABLE;

		enable_irq(fpgax_irq_info[irq].irq_number);
		fpgax_irq_info[irq].benabled = true;
		fpgax_readl(&int_mask, fpga_reg_int_enable);
		int_mask |= BIT(irq);
		fpgax_writel(int_mask, fpga_reg_int_enable);
	}

	printk(KERN_INFO "fpgax irq enabled,  irq: %d, int_number: %d handler:%x\n", irq, fpgax_irq_info[irq].irq_number, handler); 
	return 0;
}
EXPORT_SYMBOL_GPL(fpgax_request_irq);

int fpgax_free_irq(unsigned int irq)
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

	disable_irq(fpgax_irq_info[int_number].irq_number);
	fpgax_irq_info[int_number].benabled = false;

	return 0;
}
EXPORT_SYMBOL_GPL(fpgax_free_irq);



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
        u32 mask;
        irqreturn_t irq_ret = IRQ_HANDLED;
	int fpga_int_status = 0,ret,i;
	int fpga_cur_int = -1;
	void __iomem *fpga_regs_int_status = ints_reg_base + FPGA_REG_INT_STATUS;
	void __iomem *fpga_regs_int_clear = ints_reg_base + FPGA_REG_INT_CLEAR; 
	int irq_0_mux = 0;

	struct fpga_spi_work_def *spi_work = container_of(work, struct fpga_spi_work_def, fpga_spi_wq);

        printk("\nfpga_isr:fpga_isr\n");

	for (i=0;i<4;i++)
	{
		if (fpgax_irq_info[i].irq_number == spi_work->irq) 
		{
			fpga_cur_int = i;
			break;
		}
	}

	//printk("fpga_isr spi fpga_cur_int: %d\n", fpga_cur_int);

	if (fpga_cur_int == -1) 
	{
		//printk("fpga isr irq not match\n");
		return;
	}
        
	ret = fpgax_readl(&fpga_int_status, fpga_regs_int_status);
	if (ret)
	{
		//printk("fpga_read fpga int status error, return: %d\n",ret);
		return;
	}

        printk("fpga_isr:fpga_int_status:%x\n", fpga_int_status);

        fpgax_writel(fpga_int_status, fpga_regs_int_clear);
        

	fpga_cur_int = 0;
	mask = FPGA_REG_INT_SEL_TYPE1;
	for (i=0; i<4; i++) {
		if ((fpga_int_status & mask) && (fpgax_irq_info[fpga_cur_int].fpga_irq_handler0 != NULL)) {
                        //printk("loop1\n");
			if (fpgax_irq_info[fpga_cur_int].sub_dev_id0 == NULL) {
                            printk("fpga_isr fpga_cur_int1: %d\n", fpga_cur_int);
			    irq_ret = fpgax_irq_info[fpga_cur_int].fpga_irq_handler0(spi_work->irq, spi_work->dev);
                            printk("fpga_cur_int=%d,fpga_irq_handler0=%x, dev_id=%x\n", fpga_cur_int,fpgax_irq_info[fpga_cur_int].fpga_irq_handler0,spi_work->dev);
			}
			else{
				printk("fpgax_isr fpgax_cur_int2: %d\n", fpga_cur_int);
			    irq_ret = fpgax_irq_info[fpga_cur_int].fpga_irq_handler0(spi_work->irq, fpgax_irq_info[fpga_cur_int].sub_dev_id0);
			}
		}
		mask <<= 1;
		++fpga_cur_int;
	}


	//fpgax_irq_info[fpga_cur_int].fpga_irq_handler0(spi_work->irq, spi_work->dev);
	//printk("over\n");
	return;
}


static irqreturn_t fpgax_isr(int irq, void *dev_id)
{
#if 1
    	struct fpga_spi_work_def* spi_work = NULL;
	printk("fpga isr spi enter, irq: %d\n", irq);

	spi_work = (struct fpga_spi_work_def*)dev_id;
	spi_work->irq = irq;

	schedule_work(&spi_work->fpga_spi_wq); 
	return IRQ_HANDLED;
#endif

}


int fpgax_request_irq_fun(struct device *dev)
{
	int fpga_int0;
	int ret;

	void* dev_id = NULL;


	dev_id = &fpga_spi_work;

	//dev_id = dev;

	fpga_int0 = gpio_to_irq(fpgax_int_gpios);
	ret = devm_request_irq(dev, fpga_int0, fpgax_isr, IRQF_TRIGGER_RISING,dev_name(dev), dev_id);
	if (ret)
	{
		dev_err(dev, "failsed requesting fpga irq0, irq = %d\n", fpga_int0);
		return ret;
	}
	disable_irq(fpga_int0);
	fpgax_irq_info[1].irq_number = fpga_int0;
	fpgax_irq_info[1].fpga_irq_handler0= NULL;
	fpgax_irq_info[1].fpga_irq_handler1= NULL;
	fpgax_irq_info[1].benabled = false;


	dev_info(dev, "FPGA irq number = %d\n", fpga_int0);
	gpio_direction_input(fpgax_int_gpios);

	return 0;
}

void __iomem *fpgax_io_get(u32 chipsel)
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
		case 4:
			addr = cs4_base;
			break;
		case 5:
			addr = cs5_base;
			break;
		default:
			return ERR_PTR(-ENODEV);
	}

	return addr;
}
EXPORT_SYMBOL_GPL(fpgax_io_get);
/*
struct regmap *fpga_reg = NULL;
#define FPGA_MAX_REG 0x094B

static const struct regmap_config fpga_spi_regmap = {
	.reg_bits = 24,
	.val_bits = 32,
};
*/
int fpgax_readl(u32 *value, const volatile void __iomem *addr)
{

        ssize_t status;
        u16 command[8];
        u16 rxbuf[8];
        struct spi_transfer t[2];
        struct spi_message msg;

        command[0] = ((u32)addr & 0x7FFF)|0x8000;      

        spi_message_init(&msg);
        memset(t, 0, sizeof(t));

        t[0].tx_buf = command;
        t[0].len = sizeof(command[0]);
        spi_message_add_tail(&t[0], &msg);

        t[1].rx_buf = rxbuf;
        t[1].len = sizeof(rxbuf[0]);
        spi_message_add_tail(&t[1], &msg);

        status = spi_sync(data->spi, &msg);

	*value = rxbuf[0];

	return status;
}
EXPORT_SYMBOL_GPL(fpgax_readl);

int fpgax_writel(u32 value, volatile void __iomem *addr)
{

        ssize_t status;
        u16 command[8];
        u16 rxbuf[8];
        struct spi_transfer t[2];
        struct spi_message msg;

        command[0] = (u32)addr & 0x7FFF;
        command[1] = value & 0xFFFF;;


        spi_message_init(&msg);
        memset(t, 0, sizeof(t));

        t[0].tx_buf = command;
        t[0].len = sizeof(command[0])*2;
        spi_message_add_tail(&t[0], &msg);

        status = spi_sync(data->spi, &msg);

	return status;
}	
EXPORT_SYMBOL_GPL(fpgax_writel);

int fpgax_readnl(const void __iomem *addr, void *buffer, int count)
{

        ssize_t status;
        u16 command[8];
        struct spi_transfer t[2];
        struct spi_message msg;
  
        command[0] = ((u32)addr & 0x7FFF)|0x8000;

        spi_message_init(&msg);
        memset(t, 0, sizeof(t));

        t[0].tx_buf = command;
        t[0].len = sizeof(command[0]);
        spi_message_add_tail(&t[0], &msg);

        t[1].rx_buf = buffer;
        t[1].len = 2*count;
        spi_message_add_tail(&t[1], &msg);

        status = spi_sync(data->spi, &msg);

	//printk("fpga_readnl addr: %x\n",(u32)addr);

	return status;
}
EXPORT_SYMBOL_GPL(fpgax_readnl);

int fpgax_writenl(void __iomem *addr, const void *buffer, int count)
{
        ssize_t status;
        u16 command[8];
        struct spi_transfer t[2];
        struct spi_message msg;
  
        command[0] = (u32)addr & 0x7FFF;

        spi_message_init(&msg);
        memset(t, 0, sizeof(t));

        t[0].tx_buf = command;
        t[0].len = sizeof(command[0]);
        spi_message_add_tail(&t[0], &msg);

        t[1].tx_buf = buffer;
        t[1].len = 2*count;
        spi_message_add_tail(&t[1], &msg);

        status = spi_sync(data->spi, &msg);

	return status;
}
EXPORT_SYMBOL_GPL(fpgax_writenl);

int fpgax_update_lbits(volatile void __iomem *addr, u32 mask, u32 value)
{
    u32 val=0;
    ssize_t status;

    fpgax_readl(&val, addr);

    val = (val & ~mask) | (value & mask);

    status = fpgax_writel(val, addr);

    return status;

#if 0
        ssize_t status;
        u16 val=0;
        u16 command[8];
        u16 rxbuf[8];
        struct spi_transfer t[2];
        struct spi_message msg;
	// read first
        command[0] = ((u32)addr & 0x7FFF)|0x8000;

        spi_message_init(&msg);
        memset(t, 0, 2*sizeof(t));

        t[0].tx_buf = command;
        t[0].len = sizeof(command[0]);
        spi_message_add_tail(&t[0], &msg);

        t[1].rx_buf = rxbuf;
        t[1].len = sizeof(rxbuf[0]);
        spi_message_add_tail(&t[1], &msg);

        status = spi_sync(data->spi, &msg);

        val = rxbuf[0];
        val = (val & ~mask) | (value & mask);

        /*********************write update**************************************/

	command[0] = (u32)addr & 0x7FFF;
        command[1] = val;
        //printk("addr=%x,val=%x\n", addr, val);
        //rxbuf[0] = value;
        spi_message_init(&msg);
        memset(t, 0, sizeof(t));
	
        t[0].tx_buf = command;
        t[0].len = 2*sizeof(command[0]);
        spi_message_add_tail(&t[0], &msg);
        /*
        t[1].tx_buf = rxbuf;
        t[1].len = sizeof(rxbuf[0]);
        spi_message_add_tail(&t[1], &msg);	
	*/
        status = spi_sync(data->spi, &msg);
#endif
	return status;
}
EXPORT_SYMBOL_GPL(fpgax_update_lbits);


int gpiox_init_from_dts(struct device *dev)
{
	int ret;
        
	struct device_node *np = dev->of_node; 
	u32 reg[3];
	u32 fpga_cs;
        /*
	fpgax_int_gpios = devm_kzalloc(dev, sizeof(struct gpio), GFP_KERNEL);
	if (fpgax_int_gpios == NULL)
		return -ENOMEM;	*/

        fpgax_int_gpios = of_get_named_gpio(np, "fpgax_int_gpio", 0);
        printk("fpgax_int_gpio=%d\n", fpgax_int_gpios);
        if (!gpio_is_valid(fpgax_int_gpios)) {
                dev_err(dev, "no fpgax_int_gpio pin available\n");
                goto err;
        }
        //fpgax_int_gpios[FPGAX_INT0].flags = GPIOF_IN;
        //fpgax_int_gpios[FPGAX_INT0].label = "fpgax_int_gpio";

	ret = gpio_request_one(fpgax_int_gpios, GPIOF_IN, "fpgax_int_gpio");//sizeof(fpgax_int_gpios));
         if (ret < 0) {
             printk("gpio_request_one(): %d\n", ret);
             return ret;
         }

	ret = of_property_read_u32_array(np, "regx-ints", reg, 3);
	if (IS_ERR_VALUE(ret)) {
		dev_err(dev, "invalid FPGA INT registers  definition\n");
		return ret;
	}
	fpga_cs = reg[0];
	ints_reg_base = fpgax_io_get(fpga_cs);
	if (IS_ERR(ints_reg_base))
		return PTR_ERR(ints_reg_base);
	ints_reg_base += reg[1];
	printk("\nints_reg_base = %x\n", (u32)ints_reg_base);

err:
	return 0;

}


int fpga_io_creat_dev(void)
{
	int status;	

	fpgax_class = class_create(THIS_MODULE, "fpgax");
	if(IS_ERR(fpgax_class))
		return PTR_ERR(fpgax_class);
	
	fpgax_dev = device_create(fpgax_class, NULL, fpgax_dev_no, NULL, "fpgax");
	if(IS_ERR(fpgax_dev))
		return PTR_ERR(fpgax_dev);	

	status = sysfs_create_group(&fpgax_dev->kobj, &fpgax_attr_group);		

	return status;
}


static int fpgax_io_probe(struct spi_device *spi)
{
        int ret;

	cs0_base = IOMEM(0x00000);
	cs1_base = IOMEM(0x01000);
	cs2_base = IOMEM(0x02000);
	cs3_base = IOMEM(0x03000);
	cs4_base = IOMEM(0x04000);
	cs5_base = IOMEM(0x05000);

	printk(KERN_INFO "Fpga SPI Driver - Copyright GWI\n");

        
        ret = gpiox_init_from_dts(&spi->dev);
	if (ret)
	{
		printk("Failed to init gpio from dts: %d\n", ret);
		return ret;
	}
                     
	
	fpga_spi_work.dev = &spi->dev;
	INIT_WORK(&fpga_spi_work.fpga_spi_wq, fpga_spi_do_work); 
        
        
        
        
                             
        data = devm_kzalloc(&spi->dev, sizeof(struct fpgax_spi), GFP_KERNEL);
        if (data == NULL)
            return -ENOMEM;

        spi->bits_per_word = 16;
        spi->mode = SPI_MODE_1;
        spi->max_speed_hz = 100000;

        spi_setup(spi);

        data->spi = spi;

        spi_set_drvdata(spi, data);

        fpgax_reset_interrupts();

	ret = fpgax_request_irq_fun(&spi->dev);
	if (ret)
		return ret;

	ret = fpga_io_creat_dev();

	{
		u32 int_mask;
		fpgax_readl(&int_mask, ints_reg_base + FPGA_REG_INT_ENABLE);
		printk(KERN_INFO "fpga int_mask=%08x and ints_reg_base=%x\n", int_mask, (u32)ints_reg_base);
	}
	return 0;
}

static const struct of_device_id fpgax_io_match[] = {
	{ .compatible = "gwi,fpgax-io", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, fpgax_io_match);

static int fpgax_io_remove(struct spi_device *spi)
{
        //struct fpgax_spi *data = spi_get_drvdata(spi);
	sysfs_remove_group(&fpgax_dev->kobj, &fpgax_attr_group);
	device_destroy(fpgax_class, fpgax_dev_no);
	class_destroy(fpgax_class);
	printk(KERN_INFO "fpga_remove\n");	
	return 0;
}

static struct spi_driver fpgax_io_driver = {
	.driver		= {
                .owner = THIS_MODULE,
		.name	= "gwi,fpgax-io",
		.of_match_table = fpgax_io_match,
	},
	.probe		= fpgax_io_probe,
	.remove		= fpgax_io_remove,
};

module_spi_driver(fpgax_io_driver);


MODULE_AUTHOR("xxx <xxx@gwi.com.cn>");
MODULE_DESCRIPTION("GWI FPGA low-level IO driver");
MODULE_LICENSE("GPL v2");
