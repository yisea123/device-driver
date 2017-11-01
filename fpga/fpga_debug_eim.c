
#include <linux/err.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <stdbool.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/firmware.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/resource.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/crc32.h>
#include <linux/time.h>

#include <asm-generic/dma.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/platform_data/dma-imx.h>

#include "../fpga.h"
#include "../fpga_io.h"

static ssize_t address_show(struct device *dev, struct device_attribute *attr, char *buf );
static ssize_t address_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

static ssize_t value_show(struct device *dev, struct device_attribute *attr, char *buf );
static ssize_t value_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

static ssize_t synctest_show(struct device *dev, struct device_attribute *attr, char *buf );
static ssize_t synctest_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

static ssize_t async_ram_test_show(struct device *dev, struct device_attribute *attr, char *buf );
static ssize_t async_ram_test_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

static ssize_t transmode_show(struct device *dev, struct device_attribute *attr, char *buf );
static ssize_t transmode_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

static ssize_t scanmode_show(struct device *dev, struct device_attribute *attr, char *buf );
static ssize_t scanmode_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

static ssize_t imgscan_show(struct device *dev, struct device_attribute *attr, char *buf );
static ssize_t imgscan_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

static ssize_t imgdump_show(struct device *dev, struct device_attribute *attr, char *buf );
static ssize_t imgdump_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

static ssize_t dmadump_show(struct device *dev, struct device_attribute *attr, char *buf );

static ssize_t test_show(struct device *dev, struct device_attribute *attr, char *buf );
static ssize_t test_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

static ssize_t eimbaseaddr_show(struct device *dev, struct device_attribute *attr, char *buf );
static ssize_t eimbaseaddr_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

static ssize_t offseteim_show(struct device *dev, struct device_attribute *attr, char *buf );
static ssize_t offseteim_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

static ssize_t eimregvalue_show(struct device *dev, struct device_attribute *attr, char *buf );
static ssize_t eimregvalue_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

static ssize_t writetest_show(struct device *dev, struct device_attribute *attr, char *buf );
static ssize_t writetest_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

static ssize_t readtest_show(struct device *dev, struct device_attribute *attr, char *buf );
static ssize_t readtest_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

static DEVICE_ATTR(address, S_IRUGO | S_IWUSR, address_show, address_set);
static DEVICE_ATTR(value, S_IRUGO | S_IWUSR, value_show, value_set);
static DEVICE_ATTR(test, S_IRUGO | S_IWUSR, test_show, test_set);
static DEVICE_ATTR(offseteim, S_IRUGO | S_IWUSR, offseteim_show, offseteim_set);
static DEVICE_ATTR(eimbaseaddr, S_IRUGO | S_IWUSR, eimbaseaddr_show, eimbaseaddr_set);
static DEVICE_ATTR(eimregvalue, S_IRUGO | S_IWUSR, eimregvalue_show, eimregvalue_set);
static DEVICE_ATTR(writetest, S_IRUGO | S_IWUSR, writetest_show, writetest_set);
static DEVICE_ATTR(readtest, S_IRUGO | S_IWUSR, readtest_show, readtest_set);
static DEVICE_ATTR(imgdump, S_IRUGO | S_IWUSR, imgdump_show, imgdump_set);
static DEVICE_ATTR(imgscan, S_IRUGO | S_IWUSR, imgscan_show, imgscan_set);
static DEVICE_ATTR(dmadump, S_IRUGO, dmadump_show, NULL);
static DEVICE_ATTR(scanmode, S_IRUGO | S_IWUSR, scanmode_show, scanmode_set);
static DEVICE_ATTR(transmode, S_IRUGO | S_IWUSR, transmode_show, transmode_set);
static DEVICE_ATTR(synctest, S_IRUGO | S_IWUSR, synctest_show, synctest_set);
static DEVICE_ATTR(async_ram_test, S_IRUGO | S_IWUSR, async_ram_test_show, async_ram_test_set);

static struct attribute *fpga_attr_list[] = {
	&dev_attr_address.attr,  
	&dev_attr_value.attr,  
	&dev_attr_test.attr,
	&dev_attr_offseteim.attr,
	&dev_attr_eimbaseaddr.attr,	
	&dev_attr_eimregvalue.attr,
	&dev_attr_writetest.attr,	
	&dev_attr_readtest.attr,
	&dev_attr_imgdump.attr,	 
	&dev_attr_imgscan.attr,
	&dev_attr_scanmode.attr,  
	&dev_attr_transmode.attr,  
	&dev_attr_dmadump.attr,
	&dev_attr_synctest.attr,
	&dev_attr_async_ram_test.attr,
	NULL,
};

static const struct attribute_group fpga_attr_group = {
	.attrs = (struct attribute **) fpga_attr_list,
};

struct fpga_instance {
	struct platform_driver *pdev;
	struct kobject  *fpga_kobj;
	dev_t           fpga_dev_no;
};
extern  struct device	*fpga_dev;

u32 offset, value, syncvalue, eimregvalue = 0;
int testresult, testtimes, readtesttimes, writetesttimes = 0;
u32 eimbaseoffset = 0xc08e8000;
const struct firmware *fw;
u32 eimoffset = 0;
u32 scanmode = 0;
u32 transmode = 0;
int lightsource = 0;
volatile u32 synctest_loop = 10000;
static char *dmabuf = NULL;
static char *syncbigbuf = NULL;

#define MAX_PIXEL_NUM 1008
#define SCANLINE_DATA_CNT (MAX_PIXEL_NUM*4)
#define DMA_KZALLOC_SIZE (4*1024)
#define SYNC_BUF_SIZE (64*SCANLINE_DATA_CNT) 
#define IMAGE_RAM_SIZE 31104

u32 syncbuf[MAX_PIXEL_NUM];
u32 imagebuf[MAX_PIXEL_NUM*10];
u8 sample_rambuf[IMAGE_RAM_SIZE];
u8 rambuf[IMAGE_RAM_SIZE];

struct completion img_completion;
EXPORT_SYMBOL_GPL(img_completion);
EXPORT_SYMBOL_GPL(scanmode);

const u32 img_offsets[10] = {
	0x0000,		// VI_A
	0x0fc0,		// IR_A
	0x1f80,		// IRT_A
	0x2f40,		// UV_A
	0x3f00,		// UVT_A
	0x4ec0,		// VI_B
	0x5e80,		// IR_B
	0x6e40,		// IRT_B
	0x7e00,		// UV_B
	0x8dc0,		// UVT_B
};


const u32 six_lights_masks[] = {
	FPGA_REG_IMG_DATA_LIGHT_VI_A,
	FPGA_REG_IMG_DATA_LIGHT_VI_B,
	FPGA_REG_IMG_DATA_LIGHT_IR_A,
	FPGA_REG_IMG_DATA_LIGHT_IRT_A,
	FPGA_REG_IMG_DATA_LIGHT_UV_A,
	FPGA_REG_IMG_DATA_LIGHT_UVT_A,
};

const u32 ten_lights_masks[] = {
	FPGA_REG_IMG_DATA_LIGHT_VI_A,
	FPGA_REG_IMG_DATA_LIGHT_VI_B,
	FPGA_REG_IMG_DATA_LIGHT_IR_A,
	FPGA_REG_IMG_DATA_LIGHT_IR_B,
	FPGA_REG_IMG_DATA_LIGHT_IRT_A,
	FPGA_REG_IMG_DATA_LIGHT_IRT_B,
	FPGA_REG_IMG_DATA_LIGHT_UV_A,
	FPGA_REG_IMG_DATA_LIGHT_UV_B,
	FPGA_REG_IMG_DATA_LIGHT_UVT_A,
	FPGA_REG_IMG_DATA_LIGHT_UVT_B,
};

const char *lights_strs[] = {
	"VI_A", "IR_A", "IRT_A", "UV_A", "UVT_A", "VI_B", "IR_B", "IRT_B", "UV_B", "UVT_B"
};

const char *transmode_strs[] = {
	"memcpy", "C loop", "VLD/VST(NEON)", "LDM/STM"
};

struct dma_chan *dma_m2m_chan;

struct completion dma_m2m_ok;

dma_cap_mask_t dma_m2m_mask;
struct imx_dma_data m2m_dma_data = {0};
struct scatterlist sg[3], sg2[3];

static bool dma_m2m_filter(struct dma_chan *chan, void *param)
{
	if (!imx_dma_is_general_purpose(chan))
		return false;
	chan->private = param;
	return true;
}

static ssize_t scanmode_show(struct device *dev, struct device_attribute *attr, char *buf )
{
	size_t count = 0;

	count = sprintf(buf, "%d (%s_lights_mode)\n", scanmode, (scanmode==0)?"six":"ten");
	return count;
}

static ssize_t scanmode_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{	
	int rs;
	u32 val;	
	rs = sscanf(buf, "%d\n", &scanmode);
	void __iomem *foga_regs_scan_ctrl = fpga_io_get(1) + FPGA_REG_CIS_CONTROL;
	fpga_readl(&val, foga_regs_scan_ctrl);
	val &= ~FPGA_REG_CIS_SCANMODE_MASK;
	val |= (scanmode==0)? FPGA_REG_CIS_SCANMODE_SIX_LIGHTS : FPGA_REG_CIS_SCANMODE_TEN_LIGHTS;
	fpga_writel(val, foga_regs_scan_ctrl);
	return count;
}

static ssize_t transmode_show(struct device *dev, struct device_attribute *attr, char *buf )
{
	size_t count = 0;

	count = sprintf(buf, "%d (%s_mode)\n", transmode, transmode_strs[transmode]);
	return count;
}

static ssize_t transmode_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{	
	int rs;	
	rs = sscanf(buf, "%x\n", &transmode);	
	
	return count;
}

static ssize_t offseteim_show(struct device *dev, struct device_attribute *attr, char *buf )
{
	size_t count = 0;

	count = sprintf(buf, "0x%08x\n", eimoffset);
	return count;
}

static ssize_t offseteim_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{	
	int rs;	
	rs = sscanf(buf, "%x\n", &eimoffset);	
	return count;
}

static ssize_t eimbaseaddr_show(struct device *dev, struct device_attribute *attr, char *buf )
{
	size_t count = 0;
	count = sprintf(buf, "0x%08x\n", eimbaseoffset);
	return count;
}

static ssize_t eimbaseaddr_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{	
	int rs;	
	rs = sscanf(buf, "%x\n", &eimbaseoffset);	
	printk("\n eim regbaseaddr set = 0x%08x\n", eimbaseoffset);	
	return count;
}

static ssize_t eimregvalue_show(struct device *dev, struct device_attribute *attr, char *buf )
{
	size_t count = 0;
	void __iomem *eim_regs = NULL;

	eimregvalue = readl(eim_regs + eimbaseoffset + eimoffset);	
	count = sprintf(buf, "%08x\n", eimregvalue);
	printk("\n eim regaddr = 0x%08x, eim regvalue = 0x%08x\n", (u32)(eim_regs + eimbaseoffset + eimoffset), eimregvalue);
	return count;
}

static ssize_t eimregvalue_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int rs;	
	void __iomem *eim_regs = NULL;

	rs = sscanf(buf, "%x\n", &eimregvalue);	
	printk("\n eim regvalue set = 0x%08x\n",  eimregvalue);
	writel(eimregvalue, eim_regs + eimbaseoffset + eimoffset);

	return count;
}

static ssize_t writetest_show(struct device *dev, struct device_attribute *attr, char *buf )
{
	size_t count = 0;
	count = sprintf(buf, "0x%08x\n", writetesttimes);
	return count;
}

static ssize_t writetest_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{	
	int rs;	
	u32 i = 0, cs = 0; 
	u32 addr = 0;
	void __iomem *fpga_regs = NULL;
	u32 ww,rw;

	cs = offset >> 16;
	addr = 0xffff & offset;
	fpga_regs = fpga_io_get(cs) + addr;
	printk("fpga_regs = %x, cs = %d\n", (u32)fpga_regs, cs);

	rs = sscanf(buf, "%d\n", &writetesttimes);	
	printk("writetesttimes set is %d\n",  writetesttimes);

	while (i < writetesttimes) {
		ww = i & 0xffff;
		fpga_writel(ww, fpga_regs);
		fpga_readl(&rw, fpga_regs);
		if (rw != ww) {
			printk("ERROR: %d time test: address %08x read: %08x , expected: %08x\n", i, (u32)(fpga_regs), rw, ww);
			return count;
		}
		i++;
	}
	if (i == writetesttimes) {
		printk("total %d test times is ok \n",  writetesttimes);
	}
	return count;
}

static ssize_t readtest_show(struct device *dev, struct device_attribute *attr, char *buf )
{
	size_t count = 0;
	count = sprintf(buf, "0x%08x\n", readtesttimes);
	return count;
}

static ssize_t readtest_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{	
	int rs;	
	u32 i = 0, cs = 0; 
	u32 addr = 0;
	void __iomem *fpga_regs = NULL;
	u32 readbuf1, readbuf2 = 0;

	cs = offset >> 16;
	addr = 0xffff & offset;
	fpga_regs = fpga_io_get(cs) + addr;
	printk("fpga_regs = %x, cs = %d\n", (u32)fpga_regs, cs);

	rs = sscanf(buf, "%d\n", &readtesttimes);	
	printk("readtesttimes set is %d\n",  readtesttimes);

	while (i < readtesttimes) {
		fpga_readl(&readbuf1, fpga_regs);
		fpga_readl(&readbuf2, fpga_regs);
		if (readbuf1 != readbuf2) {
			printk("ERROR: %d time test: address %08x read: %08x , expected: %08x\n", readtesttimes, (u32)(fpga_regs), readbuf1, readbuf2);
		}
		i++;
	}
	if (i == readtesttimes) {
		printk("total %d test times is ok \n",  readtesttimes);
	}
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
	rs = sscanf(buf, "%x\n", &offset);	
	return count;
}

static ssize_t value_show(struct device *dev, struct device_attribute *attr, char *buf )
{
	size_t count = 0;
	u32 cs = 0; 
	u32 addr = 0;
	void __iomem *fpga_regs = NULL;

	cs = offset >> 16;
	addr = 0xffff & offset;
	fpga_regs = fpga_io_get(cs) + addr;
	fpga_readl(&value,fpga_regs);	
	count = sprintf(buf, "%08x\n", value);

	return count;
}

static ssize_t value_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int rs;	
	u32 cs = 0; 
	u32 addr = 0;
	void __iomem *fpga_regs = NULL;

	cs = offset >> 16;
	addr = 0xffff & offset;
	fpga_regs = fpga_io_get(cs) + addr;

	rs = sscanf(buf, "%x\n", &value);	
	fpga_writel(value, fpga_regs);

	return count;
}


#define read_block_l8(src) \
	__asm__("ldmia	%0!, {%1, %2, %3, %4, %5, %6, %7, %8}" \
		: "=r" (src), "=r" (a1), "=r" (a2), "=r" (a3), "=r" (a4), "=r" (a5), "=r" (a6), "=r" (a7), "=r" (a8) \
		: "0" (src))


#define write_block_l8(dst) \
	__asm__ __volatile__("stmia	%0!, {%2, %3, %4, %5, %6, %7, %8, %9}" \
		: "=r" (dst) \
		: "0" (dst), "r" (a1), "r" (a2), "r" (a3), "r" (a4), "r" (a5), "r" (a6), "r" (a7), "r" (a8))


static ssize_t imgscan_show(struct device *dev, struct device_attribute *attr, char *buf )
{
	int i, lights;
	size_t count = 0;
	u32 mask, imgdata_status;
	const u32 *light_sel;

	void __iomem *imgdata_int_status;
	void __iomem *imgdata;

	void __iomem *imgdata_int_clear = NULL;
	imgdata_int_clear = fpga_io_get(2) + FPGA_REG_IMG_DATA_INT_CLEAR;

	imgdata = fpga_io_get(0);
	imgdata_int_status = fpga_io_get(2) + FPGA_REG_IMG_DATA_INT_STATUS;

	if (scanmode == 0) {	// six-lights mode
		lights = 6;  
		light_sel = six_lights_masks;
	}
	else {			// ten-lights mode
		lights = 10;  
		light_sel = ten_lights_masks;
	}

	fpga_readl(&imgdata_status, imgdata_int_status);

//	printk("imgdata_status = %04x\n",imgdata_status );

	if (imgdata_status == 0)
	{
//		printk("wait_for_completion\n");
		wait_for_completion(&img_completion);
	}

	for (i = 0; i < lights; i++) {
		register void __iomem *src;
		register void *dst;
		src = imgdata + img_offsets[i];
		dst = imagebuf + i*MAX_PIXEL_NUM;//SCANLINE_DATA_CNT;
		mask = light_sel[i];
		for(;;)
		{
			fpga_readl(&imgdata_status, imgdata_int_status);
			
			if ((imgdata_status & mask) != 0)
				break;
			wait_for_completion(&img_completion);
		}


		fpga_writel(light_sel[i], imgdata_int_clear); //clear int status bit
		memcpy(dst, src, SCANLINE_DATA_CNT);
	}
	count = SCANLINE_DATA_CNT;//lights;
	memcpy(buf, imagebuf + lightsource * MAX_PIXEL_NUM, SCANLINE_DATA_CNT);
	return count;
}


static ssize_t imgscan_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int rs;
	rs = sscanf(buf, "%d\n", &scanmode);
	if (scanmode != 0 && scanmode != 1)
		return -EINVAL;
	printk("set image scanning mode = %d (%s_light_mode)\n", scanmode, (scanmode==0)?"six":"ten");
	return count;
}


static void dma_m2m_callback(void *data)
{
	complete(&dma_m2m_ok);
	return ;
}

static ssize_t dmadump_show(struct device *dev, struct device_attribute *attr, char *buf )
{
	size_t count = 0;
	u32 val, cnt, crc32, errcnt, offset = 0;
	int ret;

	void __iomem *fpga_regs = NULL;
	void __iomem *imgdata = NULL;
	
	imgdata = fpga_io_get(0);
	fpga_regs = fpga_io_get(2) + 0x40c;

	//wait_for_completion(&img_completion);
	//printk("wait for completion return \n");

	fpga_readl(&val, fpga_regs);

	cnt = synctest_loop;
	val = 0x000f & val;	//下标
	offset = 0;//img_offsets[val];
	memset(syncbuf, 0, sizeof(syncbuf));

	if (offset == 0xffff) {
		memset(syncbuf, 0xffff, MAX_PIXEL_NUM * 2);	//填充全黑数据
	}
	else{
		struct dma_slave_config dma_m2m_config;
		struct dma_async_tx_descriptor *dma_m2m_desc;
		int len = DMA_KZALLOC_SIZE;
		
		dma_m2m_config.direction = DMA_MEM_TO_MEM;
		dma_m2m_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		dmaengine_slave_config(dma_m2m_chan, &dma_m2m_config);

		memset(dmabuf, '\0', DMA_KZALLOC_SIZE);
		errcnt = 0;
		while (cnt--) {
#define CRC32_VAL_DMA 0x16EC284A
			sg_init_table(sg, 1);
			sg_set_page(&sg[0], pfn_to_page(PFN_DOWN(0x08000000)), len, 0x08000000 & (PAGE_SIZE - 1));
			sg_dma_address(&sg[0]) = 0x08000000;

			sg_init_table(sg2, 1);
			sg_set_buf(&sg2[0], dmabuf, SCANLINE_DATA_CNT);
			ret = dma_map_sg(NULL, sg2, 1, dma_m2m_config.direction);
			dma_m2m_desc = dmaengine_prep_dma_sg(dma_m2m_chan, sg2, 1, sg, 1, DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
					dma_m2m_desc->callback = dma_m2m_callback;
			dmaengine_submit(dma_m2m_desc);

			//printk("sdma_write submit !!!!!!!!!!!\n");

			dma_async_issue_pending(dma_m2m_chan);

			wait_for_completion(&dma_m2m_ok);

			dma_unmap_sg(NULL, sg2, 1, dma_m2m_config.direction);
			//printk("sdma_write ok !!!!!!!!!!!\n");
			crc32 = crc32_le(0, dmabuf, SCANLINE_DATA_CNT);
			if(crc32 != CRC32_VAL_DMA)
			{
				errcnt++;
			}
		}
	}

	memcpy(buf, dmabuf, SCANLINE_DATA_CNT);
	count = sprintf(buf, "total test loop = %d, error count =%d\n", synctest_loop, errcnt);
	return count;
}


static ssize_t imgdump_show(struct device *dev, struct device_attribute *attr, char *buf )
{
	size_t count = 0;
	u32 light_sel, cnt, offset = 0;// timeout =  msecs_to_jiffies(2000);
	int lights;

	void __iomem *imgdata = NULL;
	void __iomem *imgdata_int_status = NULL;
	void __iomem *imgdata_int_clear = NULL;



	imgdata_int_status = fpga_io_get(2) + FPGA_REG_IMG_DATA_INT_STATUS;
	imgdata_int_clear = fpga_io_get(2) + FPGA_REG_IMG_DATA_INT_CLEAR;

	light_sel = BIT(lightsource);
	lights = (scanmode == 0) ? 6 : 10;

	offset = img_offsets[lightsource];
	imgdata = fpga_io_get(0) + offset;
#if 0
	for (i=0; i<lights; i++) {
		fpga_readl(&imgdata_status, imgdata_int_status);
		if ((imgdata_status & light_sel) != 0)
		{
			fpga_writel(light_sel, imgdata_int_clear); //clear int status bit
			break;	// exit loop if data is ready
		}
	}
#endif
	if (offset == 0xffff) {
		memset(buf, 0xff, SCANLINE_DATA_CNT);	//填充全黑数据
	}
	else{
		cnt = SCANLINE_DATA_CNT;
		
		if(transmode == 1){
			printk("mode is c\n");
			fpga_readnw(imgdata, &buf, MAX_PIXEL_NUM * 2);
		}

		else if(transmode == 2){
			register void __iomem *src;
			register void *dst;
			src = imgdata;
			dst = buf;
			printk("mode is vld\n");
			__asm__ __volatile__(
				"vldloop:\n"
				"pld	[%0, #0x40]\n"
				"vldm	%0, {d0-d15}\n"
				"vstm	%1, {d0-d15}\n"
				"subs	%2, #0x40\n"
				"bgt	vldloop\n"
				:
				: "r" (src), "r"(dst), "r"(cnt)
			);
		}

		else if(transmode == 3){
			register void __iomem *src  __asm__("r0");
			register void *dst __asm__("r1");
			src = imgdata;
			dst = buf;
			printk("mode is ldm\n");
			__asm__ __volatile__(
				"push   {r3-r10}\n" 
				"ldmloop:\n"
				"pld	[%0, #32]\n"
				"ldmia	%0, {r3-r10}\n"     
				"stmia	%1, {r3-r10}\n"
				"subs	%2, #32\n"
				"bgt	ldmloop\n"
				"pop    {r3-r10}\n" 
				:
				: "r" (src), "r"(dst), "r"(cnt)
			);
		}

		else if(transmode == 0){
			register void __iomem *src;
			register void *dst;
			src = imgdata;
			dst = buf;
//			printk("test1\n");
//			printk("src = %08x\n", src);
//			printk("dst = %08x\n", dst);
//			printk("SCANLINE_DATA_CNT = %08x\n", SCANLINE_DATA_CNT);
			memcpy(dst, src, SCANLINE_DATA_CNT); 
//			printk("test2\n");
		}
	}
	count = SCANLINE_DATA_CNT;
	return count;
}


static ssize_t imgdump_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int rs;
	rs = sscanf(buf, "%d\n", &lightsource);
	if (lightsource < 0 || lightsource > 9)
		return -EINVAL;
	printk("set image dumpping light-source = %d (%s)\n", lightsource, lights_strs[lightsource]);
	return count;
}


static ssize_t synctest_show(struct device *dev, struct device_attribute *attr, char *buf )
{
	size_t count = 0;
	u32 cnt, sample_crc32, crc32, errcnt = 0;
	int i;
	void __iomem *imgdata = NULL;
	
	imgdata = fpga_io_get(0);
	memset(sample_rambuf, 0, sizeof(sample_rambuf));
	for(i = 0; i < IMAGE_RAM_SIZE; i++)
	{
		sample_rambuf[i] = (u8)i;
	} 		
	sample_crc32 = crc32_le(0, (char *)sample_rambuf, IMAGE_RAM_SIZE);
	cnt = 0;
	
	memset(rambuf, 0, sizeof(rambuf));
	while (cnt++ < synctest_loop) 
	{
		memcpy(rambuf, imgdata, IMAGE_RAM_SIZE);
		crc32 = crc32_le(0, (char *)rambuf, IMAGE_RAM_SIZE);
//		printk("crc32 = %08X\n", crc32);
		if(crc32 != sample_crc32)
		{
			int i;
			int dcnt = 0;
			errcnt++;
			printk("err time is %d, crc is %08x->%08x,current err bytes is :\n", cnt, sample_crc32, crc32);
			for (i=0; i < IMAGE_RAM_SIZE; i++) {
				if (rambuf[i] != sample_rambuf[i]) {
					printk("%d, corect=%08x<-err=%08x\n", i, sample_rambuf[i], rambuf[i]);
					dcnt++;
					if (dcnt > 257) 
						break;					
				}
			}
		}
		if (errcnt > 10) 
			break;
	}

	count = sprintf(buf, "total test loop = %d, error count =%d\n", synctest_loop, errcnt);
	return count;
}

static ssize_t synctest_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int rs;
	rs = sscanf(buf, "%d\n", &synctest_loop);

	return count;
}

static ssize_t async_ram_test_show(struct device *dev, struct device_attribute *attr, char *buf )
{

	size_t count = 0;
	u32 cnt, sample_crc32, crc32, errcnt = 0;
	int i;
	void __iomem *imgdata = NULL;
	
	imgdata = fpga_io_get(0);
	memset(sample_rambuf, 0, sizeof(sample_rambuf));
	for(i = 0; i < IMAGE_RAM_SIZE; i++)
	{
		sample_rambuf[i] = (u8)i;
	} 		
	sample_crc32 = crc32_le(0, (char *)sample_rambuf, IMAGE_RAM_SIZE);
	cnt = 0;
	
	memset(rambuf, 0, sizeof(rambuf));
	while (cnt++ < synctest_loop) 
	{
		memcpy(rambuf, imgdata, IMAGE_RAM_SIZE);
		crc32 = crc32_le(0, (char *)rambuf, IMAGE_RAM_SIZE);
//		printk("crc32 = %08X\n", crc32);
		if(crc32 != sample_crc32)
		{
			int i;
			int dcnt = 0;
			errcnt++;
			printk("err time is %d, crc is %08x->%08x,current err bytes is :\n", cnt, sample_crc32, crc32);
			for (i=0; i < IMAGE_RAM_SIZE; i++) {
				if (rambuf[i] != sample_rambuf[i]) {
					printk("%d, corect=%08x<-err=%08x\n", i, sample_rambuf[i], rambuf[i]);
					dcnt++;
					if (dcnt > 257) 
						break;					
				}
			}
		}
		if (errcnt > 10) 
			break;
	}

	count = sprintf(buf, "total test loop = %d, error count =%d\n", synctest_loop, errcnt);
	return count;
}

static ssize_t async_ram_test_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int rs;
	rs = sscanf(buf, "%d\n", &synctest_loop);

	return count;
}

static ssize_t test_show(struct device *dev, struct device_attribute *attr, char *buf )
{
	size_t count = 0;
	
	count = sprintf(buf, "%d\n", testresult);

	return count;
}

int eim_register_test(void __iomem *fpga_reg, int times)
{
	int i, j;
	int num = 16;
	u32 wbuf[16] , rbuf[16];

	for (i = 0; i < num; i++) {
		wbuf[i] = (times + i) & 0xffff;
		fpga_writel((wbuf[i]), fpga_reg + 4 * i);
	}

	for (j = 0; j < num; j++) {
		fpga_readl(&rbuf[j], fpga_reg + 4 * j);
		if (rbuf[j] != wbuf[j]) {
			printk("ERROR: %d time test: address %08x read: %08x , expected: %08x\n", times, (u32)(fpga_reg + 4 * j), rbuf[j], wbuf[j]);
			return 0;
		}
	}
	return 1;
}

static ssize_t test_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int rs;	
	u32 i = 0, cs = 0; 
	u32 addr = 0;
	void __iomem *fpga_regs = NULL;

	cs = offset >> 16;
	addr = 0xffff & offset;
	fpga_regs = fpga_io_get(cs) + addr;
	printk("fpga_regs = %x, cs = %d\n", (u32)fpga_regs, cs);

	rs = sscanf(buf, "%d\n", &testtimes);	
	printk("testtimes set is %d\n",  testtimes);

	while (i < testtimes) {
		testresult = eim_register_test(fpga_regs, i);
		if (!testresult) {
			return count;
		}
		i++;
	}
	if (i == testtimes) {
		printk("total %d test times is ok \n",  testtimes);
	}
	return count;
}

static int kobj_debug_creat(void)
{
	int status;    
	printk("test3\n");
	status = sysfs_create_group(&fpga_dev->kobj, &fpga_attr_group);		
	printk("sysfs_create_group: %d\n", status);
	return status;
}

static void kobj_debug_destroy(void)
{
	sysfs_remove_group(&fpga_dev->kobj, &fpga_attr_group);
}

/**
 * fpga_remove - Remove method for the GPIO device.
 * @pdev: pointer to the platform device
 *
 * This function remove gpiochips and frees all the allocated resources.
 */
static int fpga_debug_remove(struct platform_device *pdev)
{
//	struct fpga_instance *fpga_chip = platform_get_drvdata(pdev);

	kobj_debug_destroy();

	if (dmabuf) 
	{
		kfree(dmabuf);
	}

	printk("fpga_remove\n");
	return 0;
}

static irqreturn_t fpga_debug_isr0(int irq, void *dev_id)
{
//	printk("fpga int 0 handled\n");
	return IRQ_HANDLED;
}

static irqreturn_t fpga_debug_isr1(int irq, void *dev_id)
{
//	printk("fpga int 1 handled\n");
	complete(&img_completion);
	return IRQ_HANDLED;
}

static irqreturn_t fpga_debug_isr2(int irq, void *dev_id)
{
//	printk("fpga int 2 handled\n");
	return IRQ_HANDLED;
}

/*
 * fpga_of_probe - Probe method for the GPIO device.
 * @pdev: pointer to the platform device
 *
 * Return:
 * It returns 0, if the driver is bound to the GPIO device, or
 * a negative value if there is an error.
 */
static int fpga_debug_probe(struct platform_device *pdev)
{
	struct fpga_instance *fpga_chip;
	int ret;

	printk("Fpga Debug Driver - Copyright GWI\n");

	fpga_chip = devm_kzalloc(&pdev->dev, sizeof(*fpga_chip), GFP_KERNEL);
	if (!fpga_chip)
	    return -ENOMEM;

	ret = kobj_debug_creat();

	printk("dev_fpga_creat ret= %d\n", ret);
	if(ret)
	    return ret;	

	platform_set_drvdata(pdev, fpga_chip);

	init_completion(&img_completion);
	init_completion(&dma_m2m_ok);

	dmabuf = kzalloc(SCANLINE_DATA_CNT, GFP_DMA);
	if (dmabuf == NULL)
	{
		printk("alloc dmabuf memory failure!\n");

		return -ENOMEM;
	}
	
	dma_cap_zero(dma_m2m_mask);
	dma_cap_set(DMA_SLAVE, dma_m2m_mask);
	m2m_dma_data.peripheral_type = IMX_DMATYPE_MEMORY;
	m2m_dma_data.priority = DMA_PRIO_HIGH;
	printk("before dma_request_channel\n");
	dma_m2m_chan = dma_request_channel(dma_m2m_mask, dma_m2m_filter, &m2m_dma_data);
	if (!dma_m2m_chan) {
		printk("Error opening the SDMA memory to memory channel\n");
		return -EINVAL;
	}

	syncbigbuf = devm_kzalloc(&pdev->dev, SYNC_BUF_SIZE, GFP_KERNEL); 
	if (syncbigbuf == NULL)
		return -ENOMEM;
	return 0;
}

static const struct of_device_id of_fpga_debug_match[] = {
	{ .compatible = "gwi,fpga-debug", },
	{ /* end of list */ },
};

MODULE_DEVICE_TABLE(of, of_fpga_debug_match);

static struct platform_driver fpga_debug_driver = {
	.probe		= fpga_debug_probe,
	.remove		= fpga_debug_remove,
	.driver		= {
		.name = "fpga-debug",
		.owner = THIS_MODULE,
		.of_match_table	= of_fpga_debug_match,
	},
};

static int __init fpga_debug_init(void)
{
//	fpga_request_irq(2, fpga_debug_isr2, NULL);
//	fpga_request_irq(1, fpga_debug_isr1, NULL);
//	fpga_request_irq(0, fpga_debug_isr0, NULL);

	return platform_driver_register(&fpga_debug_driver);
}


static void __exit fpga_debug_exit(void)
{
	platform_driver_unregister(&fpga_debug_driver);
	dma_release_channel(dma_m2m_chan);
	dma_m2m_chan = NULL;
}
module_init(fpga_debug_init);
module_exit(fpga_debug_exit);

MODULE_AUTHOR("gwi, Inc.");
MODULE_DESCRIPTION("FPGA DEBUG driver");
MODULE_LICENSE("GPL");
