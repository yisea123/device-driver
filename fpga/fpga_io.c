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
#include <linux/ioport.h>
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

#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>



#include "../fpga.h"
#include "../fpga_io.h"

#define  FPGA_INT0 0
#define  FPGA_INT1 1
#define  FPGA_INT2 2
#define  FPGA_INT3 3
#define  A4 1

#define  GPIO_VALUE_HIGH 1
#define  GPIO_VALUE_LOW  0

#define IMX6UL_GPR1_CS_DIR 0x249
#define IMX6UL_GPR1_CS_OUTPUT 0x249

#define AXI_CLK_ROOT		0X000A0085  /*265.3MHz*/
#define ENFC_CLK_ROOT		0X000A0086  /*--------*/

#define SIM_CLK_ROOT		0X000A0088  /*--------*/

#define LCDIF_PIX_CLK_ROOT	0X000A008A  /*--------*/
#define AHB_CLK_ROOT		0X000A008B  /*132.5MHz*/
#define IPG_CLK_ROOT		0X000A009C  /*66MHz/2 */
#define PERCLK_ROOT		0X000A008D  /*24MHz   */
#define CKIL_SYNC_CLK_ROOT	0X000A008E  /*36,2MHz */
#define PLL4_MAIN_CLK		0X000A00FF  /*--------*/

//extern char *reserve_memory;
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

static void __iomem *ccm_ccosr_addr =  MX6Q_IO_ADDRESS(0x020c4060);
static void __iomem *clko1_mux_ctl_rgr =  MX6Q_IO_ADDRESS(0x020e0048);
static void __iomem *ccm_analog_pll_audion_addr =  MX6Q_IO_ADDRESS(0x020c8070);
static void __iomem *ccm_analog_pll_audio_num_addr =  MX6Q_IO_ADDRESS(0x020c8080);
static void __iomem *ccm_analog_pll_audio_denom_addr =  MX6Q_IO_ADDRESS(0x020c8090);
static void __iomem *ccm_ccgr6_addr =  MX6Q_IO_ADDRESS(0x20c4080);
//static void __iomem *ccm_ccgr6_addr_test =  IMX_IO_P2V(0x020e4004);

struct gpio *fpga_int_gpios;

static void __iomem *cs0_base, *cs1_base, *cs2_base, *cs3_base;
static void __iomem *ints_reg_base, *ctrl_reg_base;

struct class    *fpga_class;
struct device	*fpga_dev;
EXPORT_SYMBOL_GPL(fpga_dev);
struct kobject  *fpga_kobj;
dev_t           fpga_dev_no;
unsigned int version;

int download_proc(const struct firmware *fw, const char *fw_name);
static ssize_t version_show(struct device *dev, struct device_attribute *attr, char *buf );
static ssize_t reset_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

static DEVICE_ATTR(version, S_IRUGO, version_show, NULL);
static DEVICE_ATTR(reset, S_IWUSR, NULL, reset_set);

static struct attribute *fpga_attr_list[] = {
	&dev_attr_version.attr,
	&dev_attr_reset.attr,
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

int fpga_bclk_freq_set(void)
{
	int ret;
	u32 weim_timing[10];
	struct device_node *np;
	unsigned char freq_val;

	np = of_find_node_by_path("/soc/aips-bus@02100000/weim@021b8000/fpga@0,0");
	if (!np) {
		pr_err("no fpga@0,0 found in devicetree");
		return -ENODEV; 
	}

	ret = of_property_read_u32_array(np, "fsl,weim-cs-timing", weim_timing, 6);
	if (IS_ERR_VALUE(ret)) {
		pr_err("invalid fsl,weim-cs-timing registers  value.\n");
		return ret;
	}

	switch ((weim_timing[0] & 0x3000) >> 12) {
		case 0x3: //33M
			freq_val = 0;
			break;
		case 0x1://66M
			freq_val = 1;
			break;
		case 0x0://132M
			freq_val = 2;
			break;
		default:
			freq_val = -1;
			break;
	}
	if (freq_val != -1) {
		fpga_writel(freq_val, ctrl_reg_base + FPGA_REG_BCLK_FREQUENCY);
	}

	return 0;
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

	fpga_int_gpios = devm_kzalloc(dev, 4*sizeof(struct gpio), GFP_KERNEL);
	if (fpga_int_gpios == NULL)
		return -ENOMEM;

	for_each_node_by_name(np, "fpga_config"){

		fpga_int_gpios[FPGA_INT0].gpio = of_get_named_gpio(np, "fpga_int0_gpio", 0);
		if (!gpio_is_valid(fpga_int_gpios[FPGA_INT0].gpio)) {
			dev_err(dev, "no fpga_int0_gpio pin available\n");
			goto err;
		}
		fpga_int_gpios[FPGA_INT0].flags = GPIOF_IN;
		fpga_int_gpios[FPGA_INT0].label = "fpga_int0_gpio";
		printk("fpga_init0 gpio ok!\n");

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
	fpga_writel((FPGA_REG_INT_SEL_TYPE1|FPGA_REG_INT_SEL_TYPE2|FPGA_REG_INT_SEL_TYPE3|FPGA_REG_INT_SEL_TYPE4),
		ints_reg_base + FPGA_REG_INT_CLEAR);
}
/*
irq--0----reserved	--- INT0
irq--1--- reserved 	--- INT1
irq--2--- image data interrupt	--- INT2
irq--3--- motor interrupt	--- INT3
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

	printk(KERN_INFO "fpga irq enabled,  irq: %d, int_number: %d handler:%x\n", irq, fpga_irq_info[irq].irq_number, handler);
	return 0;
}
EXPORT_SYMBOL_GPL(fpga_request_irq);

int fpga_free_irq(unsigned int irq)
{
	disable_irq(fpga_irq_info[irq].irq_number);
	fpga_irq_info[irq].benabled = false;

	return 0;
}
EXPORT_SYMBOL_GPL(fpga_free_irq);

static const struct of_device_id fpga_io_match[] = {
	{ .compatible = "gwi,fpga-io", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, fpga_io_match);

static irqreturn_t fpga_isr(int irq, void *dev_id)
{

	u32 mask;
	int fpga_int_status = 0, ret, i;
	int fpga_cur_int = -1;
	void __iomem *fpga_regs_int_status = ints_reg_base + FPGA_REG_INT_STATUS;
	void __iomem *fpga_regs_int_clear = ints_reg_base + FPGA_REG_INT_CLEAR;
	irqreturn_t irq_ret = IRQ_HANDLED;


//	printk("fpga_isr fpga_cur_int: %d\n", fpga_cur_int);

	ret = fpga_readl(&fpga_int_status, fpga_regs_int_status);
	if (ret)
	{
//		printk("KERN_INFO fpga_read fpga int status error, return: %d\n",ret);
		return IRQ_HANDLED;
	}
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
			    irq_ret = fpga_irq_info[fpga_cur_int].fpga_irq_handler0(irq, dev_id);
			}
			else{
			    irq_ret = fpga_irq_info[fpga_cur_int].fpga_irq_handler0(irq, fpga_irq_info[fpga_cur_int].sub_dev_id0);
			}
		}
		mask <<= 1;
		++fpga_cur_int;
	}
	return irq_ret;
}

int fpga_request_irq_fun(struct device *dev)
{
	int fpga_int0,fpga_int1,fpga_int2,fpga_int3;
	int ret;

	void* dev_id = NULL;

	dev_id = dev;

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

	gpio_direction_input(fpga_int_gpios[FPGA_INT0].gpio);
	gpio_direction_input(fpga_int_gpios[FPGA_INT1].gpio);
	gpio_direction_input(fpga_int_gpios[FPGA_INT2].gpio);
	gpio_direction_input(fpga_int_gpios[FPGA_INT3].gpio);

	return 0;
}

/*
 *  PLL4_MAIN_CLK set
 *
 * PLL output frequency = Fref * (DIV_SELECT + NUM/DENOM)
 * (CCM_ANALOG_PLL_AUDIOn)” describes the DIV_SELECT bit field, mentioned
 * Numerator of Audio PLL Fractional Loop Divider Register
 * (CCM_ANALOG_PLL_AUDIO_NUM)”
 * Denominator of Audio PLL Fractional Loop Divider
 * Register (CCM_ANALOG_PLL_AUDIO_DENOM)”
 *
 *
 */
static int fpga_clock_core_init(void)
{
	int tmp;
#if 0
	/*set jtag_tms to ccm_clko1*/
	tmp = readl(clko1_mux_ctl_rgr);
	printk(KERN_INFO "clko1_mux_ctl_rgr = %08x\n", clko1_mux_ctl_rgr);
	printk(KERN_INFO "tmp2 = %08x\n", tmp);
	tmp |= 0x3;  //clko1
	printk(KERN_INFO "tmp1 = %08x\n", tmp);
	writel(tmp, clko1_mux_ctl_rgr);
	tmp = readl(clko1_mux_ctl_rgr);
	printk(KERN_INFO "tmp3 = %08x\n", tmp);

	tmp = readl(ccm_analog_pll_audion_addr);
	printk(KERN_INFO "ccm_analog_pll_audion_addr = %08x\n", ccm_analog_pll_audion_addr);
	printk(KERN_INFO "tmp2 = %08x\n", tmp);
	tmp = 0x2029;  //clko1
	printk(KERN_INFO "tmp1 = %08x\n", tmp);
	writel(tmp, ccm_analog_pll_audion_addr);
	tmp = readl(ccm_analog_pll_audion_addr);
	printk(KERN_INFO "tmp3 = %08x\n", tmp);

	tmp = readl(ccm_analog_pll_audio_num_addr);
	printk(KERN_INFO "ccm_analog_pll_audio_num = %08x\n", ccm_analog_pll_audio_num_addr);
	printk(KERN_INFO "tmp2 = %08x\n", tmp);
	tmp = 0x2710;  //10000
	printk(KERN_INFO "tmp1 = %08x\n", tmp);
	writel(tmp, ccm_analog_pll_audio_num_addr);
	tmp = readl(ccm_analog_pll_audio_num_addr);
	printk(KERN_INFO "tmp3 = %08x\n", tmp);

	tmp = readl(ccm_analog_pll_audio_denom_addr);
	printk(KERN_INFO "ccm_analog_pll_audio_denom_addr = %08x\n", ccm_analog_pll_audio_denom_addr);
	printk(KERN_INFO "tmp2 = %08x\n", tmp);
	tmp = 0x07a79ad1;  //50000
	printk(KERN_INFO "tmp1 = %08x\n", tmp);
	writel(tmp, ccm_analog_pll_audio_denom_addr);
	tmp = readl(ccm_analog_pll_audio_denom_addr);
	printk(KERN_INFO "tmp3 = %08x\n", tmp);

#endif

#if A4
	/*set jtag_tms to ccm_clko1 mode*/
	tmp = readl(clko1_mux_ctl_rgr);
	printk(KERN_INFO "clko1_mux_ctl_rgr = %08x\n", (u32)clko1_mux_ctl_rgr);
	printk(KERN_INFO "tmp2 = %08x\n", tmp);
	tmp |= 0x3;  //clko1
	printk(KERN_INFO "tmp1 = %08x\n", tmp);
	writel(tmp, clko1_mux_ctl_rgr);
	tmp = readl(clko1_mux_ctl_rgr);
	printk(KERN_INFO "tmp3 = %08x\n", tmp);

	/* set pll enable 13bit to 1, set Fref to 24MHz
	 * set  DIV_SELECT to 33--> 0x21
	 */
	tmp = readl(ccm_analog_pll_audion_addr);
	printk(KERN_INFO "ccm_analog_pll_audion_addr = %08x\n", (u32)ccm_analog_pll_audion_addr);
	printk(KERN_INFO "tmp2 = %08x\n", tmp);
	tmp = 0x2021;  //clko1
	printk(KERN_INFO "tmp1 = %08x\n", tmp);
	writel(tmp, ccm_analog_pll_audion_addr);
	tmp = readl(ccm_analog_pll_audion_addr);
	printk(KERN_INFO "tmp3 = %08x\n", tmp);

	tmp = readl(ccm_analog_pll_audio_num_addr);
	printk(KERN_INFO "ccm_analog_pll_audio_num = %08x\n", (u32)ccm_analog_pll_audio_num_addr);
	printk(KERN_INFO "tmp2 = %08x\n", tmp);
	tmp = 0x2710;  //10000
	printk(KERN_INFO "tmp1 = %08x\n", tmp);
	writel(tmp, ccm_analog_pll_audio_num_addr);
	tmp = readl(ccm_analog_pll_audio_num_addr);
	printk(KERN_INFO "tmp3 = %08x\n", tmp);

	tmp = readl(ccm_analog_pll_audio_denom_addr);
	printk(KERN_INFO "ccm_analog_pll_audio_denom_addr = %08x\n", (u32)ccm_analog_pll_audio_denom_addr);
	printk(KERN_INFO "tmp2 = %08x\n", tmp);
	tmp = 0x7530;  //30000
	printk(KERN_INFO "tmp1 = %08x\n", tmp);
	writel(tmp, ccm_analog_pll_audio_denom_addr);
	tmp = readl(ccm_analog_pll_audio_denom_addr);
	printk(KERN_INFO "tmp3 = %08x\n", tmp);

#endif

#if 0
	/*eim_cs0gcr1 set*/
	tmp = readl(eim_cs0gcr1_addr);
	printk(KERN_INFO "eim_cs0gcr1_addr = %08x\n", eim_cs0gcr1_addr);
	printk(KERN_INFO "tmp2 = %08x\n", tmp);
//	tmp |= 0x3;  //clko1
	printk(KERN_INFO "tmp1 = %08x\n", tmp);
	writel(tmp, eim_cs0gcr1_addr);
	tmp = readl(eim_cs0gcr1_addr);
	printk(KERN_INFO "tmp3 = %08x\n", tmp);
#endif
	/*set clk*/
	tmp = readl(ccm_ccosr_addr);
	printk(KERN_INFO "ccm_ccosr_addr = %08x\n", (u32)ccm_ccosr_addr);
	printk(KERN_INFO "tmp2 = %08x\n", tmp);
	//tmp |= 0x0000008d;  //clko1
	tmp = PLL4_MAIN_CLK;
/* 24M 1001010010
 *         tmp |= 0x00000252;
 */
	printk(KERN_INFO "tmp1 = %08x\n", tmp);
	writel(tmp, ccm_ccosr_addr);
	tmp = readl(ccm_ccosr_addr);
	printk(KERN_INFO "tmp3 = %08x\n", tmp);

	tmp = readl(ccm_ccosr_addr);
	printk(KERN_INFO "tmp4 = %08x\n", tmp);
	return 0;
}

int fpga_chipbase_get(struct platform_device *pdev)
{
	/* 设置eim片选 配置寄存器IOMUXC_GPR_GPR1
	 * 其地址物理地址为 20E4004
	 * 对应的虚拟地址为 0xa0858000+4
	 * IOMUXC_GPR_GPR1配置为4个片选每个片选32M
	 * 相应的配置IOMUXC_GPR_GPR1的低12位
	 * 001001001001
	 *
	 */
	u32 tmp;
	u32 addr;
	struct resource *mem0_res, *mem1_res, *mem2_res, *mem3_res;

	addr = 0xa0858000 + 0x4 ;
/*	tmp1 = readl((void __iomem *)addr);
	printk(KERN_INFO "tmp1 = %08x\n", tmp1);
	tmp2 = tmp1 & ~0xfff;
	tmp2 |= 0x249;
	printk(KERN_INFO "tmp2 = %08x\n", tmp2);
	writel(tmp2, (void __iomem *)addr);*/
	tmp = readl((void __iomem *)addr);
	printk(KERN_INFO "IOMUX GPR1 = %08x\n", tmp);


	/* 配置eim的时钟，通过寄存器ccgr6,设置eim_slow_clock
	 * 物理地址为：20c4080对应的虚拟地址为：0xf42c4080
	 * eim时钟为第10-11位11
	 *
	 */
	tmp = readl(ccm_ccgr6_addr);
	printk(KERN_INFO "ccm_ccgr6_addr = %08x\n", (u32)ccm_ccgr6_addr);
	printk(KERN_INFO "tmp2 = %08x\n", tmp);
	tmp |=0xc00 ;  //50000
	printk(KERN_INFO "tmp1 = %08x\n", tmp);
	writel(tmp, ccm_ccgr6_addr);
	tmp = readl(ccm_ccgr6_addr);
	printk(KERN_INFO "tmp3 = %08x, %d\n", tmp,__LINE__);


#if 1
	/* get the resource */
	mem0_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "fpga-cs0");
	if (mem0_res == NULL) {
		dev_err(&pdev->dev, "missing platform resources mem0_res data\n");
		return -ENODEV;
	}
	printk("mem0_res = %x\n",mem0_res->start);
//	mem0_res = request_mem_region(mem0_res->start, resource_size(mem0_res),"fpga-cs0");_
	cs0_base = devm_ioremap(&pdev->dev, mem0_res->start, resource_size(mem0_res));
	printk("base = %x\n", (u32)cs0_base);
	if (IS_ERR(cs0_base))
		return PTR_ERR(cs0_base);


	mem1_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "fpga-cs1");
	if (mem1_res == NULL) {
		dev_err(&pdev->dev, "missing platform  resources mem1_res data\n");
		return -ENODEV;
	}
	cs1_base = devm_ioremap(&pdev->dev, mem1_res->start, resource_size(mem1_res));
	if (IS_ERR(cs1_base))
		return PTR_ERR(cs1_base);
        printk("cs1_base = %x\n", (u32)cs1_base);

	mem2_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "fpga-cs2");
	cs2_base = devm_ioremap_resource(&pdev->dev, mem2_res);
	if (IS_ERR(cs2_base))
		return PTR_ERR(cs2_base);
	printk("cs2_base = %x\n", (u32)cs2_base);

	mem3_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "fpga-cs3");
	cs3_base = devm_ioremap_resource(&pdev->dev, mem3_res);
	if (IS_ERR(cs3_base))
		return PTR_ERR(cs3_base);

	printk("cs3_base = %x\n", (u32)cs3_base);
#endif
	return 0;
}

int fpga_io_creat_dev(void)
{
	int status;
	fpga_class = class_create(THIS_MODULE, "fpga");
	if(IS_ERR(fpga_class))
		return PTR_ERR(fpga_class);
	fpga_dev_no = MKDEV(500,0);
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
	u32 reset_buf[8];
	u32 reg[3];
	u32 fpga_cs;
	u32 tmp1,tmp2,tmp3;

	printk(KERN_INFO "Fpga EIM Driver - Copyright GWI\n");
        //printk("\n\nreserve_memory=0x%lx******add by jackLee\n\n",reserve_memory);
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
	fpga_clock_core_init();

	ret =  gpio_init_from_dts(&pdev->dev);
	if (ret)
	{
		dev_err(&pdev->dev, "Failed to init gpio from dts: %d\n",ret);
		return ret;
	}

	tmp1 = readl(0xf42e03a0);
	printk(KERN_INFO "lcd_hsync = %08x, %d\n", tmp1,__LINE__);
	writel(0x10B0,0xf42e03a0);
	tmp1 = readl(0xf42e03a0);
	printk(KERN_INFO "tmp3 = %08x, %d\n", tmp1,__LINE__);
	tmp2 = readl(0xf42e0114);
	printk(KERN_INFO "lcd_hsync = %08x, %d\n", tmp2,__LINE__);
	writel(0x5,0xf42e0114);
	tmp2 = readl(0xf42e0114);
	printk(KERN_INFO "tmp3 = %08x, %d\n", tmp2,__LINE__);
	tmp1 = readl(0xf42e03b4);
	printk(KERN_INFO "lcd_hsync = %08x, %d\n", tmp1,__LINE__);
	writel(0x10B0,0xf42e03b4);
	tmp1 = readl(0xf42e03b4);
	printk(KERN_INFO "tmp3 = %08x, %d\n", tmp1,__LINE__);
	tmp3 = readl(0xf42e0128);
	printk(KERN_INFO "lcd_hsync = %08x, %d\n", tmp3,__LINE__);
	writel(0x5,0xf42e0128);
	tmp2 = readl(0xf42e0128);
	printk(KERN_INFO "tmp3 = %08x, %d\n", tmp3,__LINE__);

	/*fpga soft reset*/
	fpga_writel(1, ctrl_reg_base);
	memcpy(reset_buf, fpga_io_get(0), 32);
	fpga_writel(0, ctrl_reg_base);

	fpga_reset_interrupts();

	ret = fpga_request_irq_fun(&pdev->dev);
	if (ret)
		return ret;

	ret = fpga_io_creat_dev();
	if (ret)
		return ret;

	ret = fpga_bclk_freq_set();
	if (ret)
		return ret;

	return 0;
}

static int fpga_io_remove(struct platform_device *pdev)
{
	fpga_reset_interrupts();
	gpio_free_array(fpga_int_gpios, sizeof(fpga_int_gpios));

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

MODULE_AUTHOR("xxx <xxx@gwi.com.cn>");
MODULE_DESCRIPTION("GWI FPGA low-level IO driver");
MODULE_LICENSE("GPL v2");
