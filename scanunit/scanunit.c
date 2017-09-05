/*
 * Image Scan Unit driver
 *
 * Copyright 2016 Hunan GreatWall Information Financial Equipment Co., Ltd.
 *
 * Licensed under the GPL-2.
 */
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <asm/pgtable.h>
#include <asm/io.h>
#include <linux/completion.h>
#include <asm/uaccess.h>

#include "../fpga_io.h"
#include "../fpga.h"
#include "scanunit.h"
#include "imagedigitiser.h"
#include "imagesensor.h"


#define SCANUNIT_NAME	"imagescan"
#define MAX_PIXEL_NUM 1008
#define SCANLINE_DATA_CNT (MAX_PIXEL_NUM*4)

u32 imagebuf[MAX_PIXEL_NUM*10];

static int scanunit_major = 0;
static struct class *scanunit_class;
dev_t scanunit_dev_no;
struct cdev scanunit_cdev;

static struct completion img_completion;
static DEFINE_MUTEX(scanunit_mutex);

int irq;

void __iomem *ints_reg_base, *imgram_base;
void __iomem *cis_reg_base;
struct scanunit_hwinfo scanner_info;
struct imagedigitiser *image_digitisers[MAX_IMAGE_DIGITISERS];
struct imagesensor *image_sensors[MAX_IMAGE_SENSORS];
phandle digitiser_list[MAX_IMAGE_DIGITISERS]; 
phandle sensor_list[MAX_IMAGE_SENSORS];

u32 imagebuf[MAX_PIXEL_NUM*10];
unsigned int scanmode = 0;
unsigned int irq_timeout_value = 1000;

enum light_index_order {
	LIGHT_VI_A,
	LIGHT_IR_A,
	LIGHT_IRT_A,
	LIGHT_UV_A,
	LIGHT_UVT_A,
	LIGHT_VI_B,
	LIGHT_IR_B,
	LIGHT_IRT_B,
	LIGHT_UV_B,
	LIGHT_UVT_B,
};

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

const u32 six_lights_sequence[] = {
	LIGHT_VI_A,
	LIGHT_VI_B,
	LIGHT_IR_A,
	LIGHT_IRT_A,
	LIGHT_UV_A,
	LIGHT_UVT_A,
};

const u32 ten_lights_sequence[] = {
	LIGHT_VI_A,
	LIGHT_VI_B,
	LIGHT_IR_A,
	LIGHT_IR_B,
	LIGHT_IRT_A,
	LIGHT_IRT_B,
	LIGHT_UV_A,
	LIGHT_UV_B,
	LIGHT_UVT_A,
	LIGHT_UVT_B,
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

static int scanunit_open(struct inode *inode, struct file *file)
{
	return 0;
}


static int scanunit_close(struct inode *inode, struct file *file)
{
	return 0;
}


static int scanunit_mmap(struct file *file, struct vm_area_struct * vma)
{
	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
	                    vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
                return -EAGAIN;
	}
	return 0;
}


static ssize_t scanunit_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	int ret = -EINVAL;
	int rs;
	int i, lights = 6, len, lightcnt = 0;
	u32 mask, imgdata_status, fetch_mask;
	u32 timeout = 0;
	const u32 *light_sel;
	const u32 *light_index;
	void __iomem *imgdata_int_status;
	void __iomem *imgdata;
	void __iomem *imgdata_int_clear = NULL;

	len = SCANLINE_DATA_CNT*10;

	if (count <= 0)
	{
		printk("scanunit_read: read error, data is NULL\n");
		return ret;
	}

	if (count > len) {
		count = len;
	}

	memset(imagebuf, 0, len);

	imgdata_int_clear = ints_reg_base + FPGA_REG_IMG_DATA_INT_CLEAR;
	imgdata = imgram_base;
	imgdata_int_status = ints_reg_base + FPGA_REG_IMG_DATA_INT_STATUS;

	if (scanmode == 0) {	// six-lights mode
		lights = 6;  
		light_sel = six_lights_masks;
		light_index = six_lights_sequence;
		fetch_mask = 0x3f;
	}
	else {			// ten-lights mode
		lights = 10;  
		light_sel = ten_lights_masks;
		light_index = ten_lights_sequence;
		fetch_mask = 0x3ff;
	}

	lightcnt = 0;
	imgdata_status = 0;
	timeout = msecs_to_jiffies(irq_timeout_value);

	while (fetch_mask != 0) 
	{
		register void __iomem *src;
		register void *dst;

		for(;;)
		{
			fpga_readl(&imgdata_status, imgdata_int_status);
			if (imgdata_status != 0)
				break;
			rs = wait_for_completion_timeout(&img_completion, timeout);
			if (rs == 0) {
				printk(KERN_ERR "scanunit_read:timeout err!!!\r\n");
				return -EAGAIN;
			}
		}
		fpga_writel(imgdata_status, imgdata_int_clear); //clear int status bits
		fetch_mask &= ~imgdata_status; //clear geted mask status

		for (i = 0; i < lights; i++) {
			
			src = imgdata + img_offsets[light_index[i]];
			dst = imagebuf + i*MAX_PIXEL_NUM;
			mask = light_sel[i];
			if (imgdata_status & mask) {
				memcpy(dst, src, SCANLINE_DATA_CNT);
				lightcnt++;
			}
		}
	}

	if (copy_to_user (buf, (const void *)imagebuf, count))
	{
		ret = -EFAULT;
		printk(KERN_ERR "scanunit_read:copy_to_user fail\n");
		goto fail;
	}
	
	return len;
fail:
	return ret;
}


extern long scanunit_ioctl(struct file *file, unsigned int cmd, unsigned long arg);


static struct file_operations scanunit_fops = {
	.owner      = THIS_MODULE,
	.open       = scanunit_open,
	.release    = scanunit_close,
	.read       = scanunit_read,
	.mmap       = scanunit_mmap,
	.unlocked_ioctl      = scanunit_ioctl,
};


static int parse_sensor_info(struct device *dev, struct device_node *np, int side)
{
	int ret;
	phandle psensor;

	ret = of_property_read_u32(np, "sensor", &psensor);
	if (IS_ERR_VALUE(ret)) {
		dev_err(dev, "Failed to parse image sensor definition of scanning side %c\n", side==0 ?'A':'B');
		return ret;
	}
	if (psensor == sensor_list[0]) {
		return 0;
	}
	else if (psensor == sensor_list[1]) {
		return 1;
	}
	return -EINVAL;
}


static int parse_section_info(struct device *dev, struct device_node *np, struct scan_section *section)
{
	int i, index, ret, sectnum;
	struct scan_section *sect;

	/* parse scanning section definition of a scanning side */
	ret = of_property_count_u32_elems(np, "sections");
	if (IS_ERR_VALUE(ret)) {
		dev_err(dev, "Failed to find scanning section definition");
		return ret;
	}
	sectnum = ret / 4;
	if ((ret % 4) || (sectnum == 0)) {
		dev_err(dev, "Incomplete scanning section definition property");
		return -EINVAL;
	}
	if (sectnum > MAX_SCANNING_SECTIONS) {
		dev_err(dev, "Incomplete scanning section definition property");
		return -EINVAL;
	}
	index = 0;
	sect = section;
	for (i = 0; i < sectnum; i++) {
		phandle tmp;
		int j;

		of_property_read_u32_index(np, "sections", index++, &sect->pstart);
		of_property_read_u32_index(np, "sections", index++, &sect->pend);
		of_property_read_u32_index(np, "sections", index++, &tmp);
		of_property_read_u32_index(np, "sections", index++, &sect->channel);

		sect->lstart = sect->pstart / scanner_info.colors;
		if (scanner_info.colors > 1)
			sect->lstart += 1;
		sect->lend = sect->pend / scanner_info.colors;

		for (j=0; j<scanner_info.digitisers; j++) 
			if (tmp == digitiser_list[j]) break;
		if (j>=scanner_info.digitisers)
			return -EINVAL;
		sect->digitiser_id = j;

		++sect;
	}
	return sectnum;
}


static int scanunit_parse_hwinfo(struct device *dev)
{
	struct device_node *child, *np = dev->of_node;
	int i, ret;
	struct scan_section *sect;

	/* try to find number of scanning colors  */
	ret = of_property_read_u32(np, "colors", &scanner_info.colors); 
	if (IS_ERR_VALUE(ret)) {
		dev_err(dev, "Failed to parse scanunit color definition\n");
		return ret;
	}
	if (scanner_info.colors != 1 && scanner_info.colors != 3) {
		dev_err(dev, "Invailid scanunit color definition %d\n", scanner_info.colors);
		return -EINVAL;
	}

	/* try to find number of scanning lightsources  */
	ret = of_property_read_u32(np, "lightsources", &scanner_info.lightsources);
	if (IS_ERR_VALUE(ret)) {
		dev_err(dev, "Failed to parse scanunit lightsources definition\n");
		return ret;
	}
	if (scanner_info.colors < 0) {
		dev_err(dev, "Invailid scanunit lightsources definition %d\n", scanner_info.lightsources);
		return -EINVAL;
	}

	/* try to find image digitisers list */
	ret = of_property_count_u32_elems(np, "digitisers");
	if (IS_ERR_VALUE(ret)) {
		dev_err(dev, "Failed to find image digitiser list");
		return ret;
	}
	if (ret > MAX_IMAGE_DIGITISERS) {
		dev_err(dev, "Too many items in the image digitiser list");
		return ret;
	}
	scanner_info.digitisers = ret;
	of_property_read_u32_array(np, "digitisers", digitiser_list, scanner_info.digitisers);
	for (i=0; i<scanner_info.digitisers; i++) {
		struct device_node *node;
		struct imagedigitiser *afe;

		node = of_find_node_by_phandle(digitiser_list[i]);
		afe = of_node_to_imagedigitiser(node);
		if (IS_ERR(afe)) {
			dev_err(dev, "Failed to get image digitiser %s\n", node->name);
			return -ENODEV;
		}
		image_digitisers[i] = afe;
	}

	/* try to find image sensors list */
	ret = of_property_count_u32_elems(np, "sensors");
	if (IS_ERR_VALUE(ret)) {
		dev_err(dev, "Failed to find image sensor list");
		return ret;
	}
	if (ret > MAX_IMAGE_DIGITISERS) {
		dev_err(dev, "Too many items in the image sensor list");
		return ret;
	}
	scanner_info.sensors = ret;
	of_property_read_u32_array(np, "sensors", sensor_list, scanner_info.sensors);
	for (i=0; i<scanner_info.sensors; i++) {
		struct device_node *node;
		struct imagesensor *sensor;

		node = of_find_node_by_phandle(sensor_list[i]);
		sensor = of_node_to_imagesensor(node);
		if (IS_ERR(sensor)) {
			dev_err(dev, "Failed to get image sensor %s\n", node->name);
			return -ENODEV;
		}
		image_sensors[i] = sensor;
	}

	/* try to find hardware information of scanning side A */
	child = of_get_next_child(np, NULL);
	if (IS_ERR(child)) {
		dev_err(dev, "No scanning side defined");
		return PTR_ERR(child);
	}
	/* try to find image sensor definition of scanning side A */
	ret = parse_sensor_info(dev, child, 0);
	if (IS_ERR_VALUE(ret))
		return ret;
	scanner_info.sensor_a = ret;
	/* try to find scanning section definition of scanning side A */
	ret = parse_section_info(dev, child, scanner_info.sectinfo_a); 
	if (IS_ERR_VALUE(ret))
		return ret;
	scanner_info.sections_a = ret;
	scanner_info.sides = 1;

	/* try to find hardware information of scanning side B */
	child = of_get_next_child(np, child);
	if (!IS_ERR(child)) {
		/* try to find image sensor definition of scanning side B */
		ret = parse_sensor_info(dev, child, 1);
		if (IS_ERR_VALUE(ret))
			return ret;
		scanner_info.sensor_b = ret;
		/* try to find scanning section definition of scanning side B */
		ret = parse_section_info(dev, child, scanner_info.sectinfo_b); 
		if (IS_ERR_VALUE(ret)) {
			return ret;
		}
		scanner_info.sections_b = ret;
		scanner_info.sides += 1;
	}

#ifdef DEBUG
	printk(KERN_INFO "Parsing scanunit hardware information:\n");
	printk(KERN_INFO "Number of image digitisers = %d\n", scanner_info.digitisers);
	printk(KERN_INFO "Number of image sensors = %d\n", scanner_info.sensors);
	printk(KERN_INFO "Number of light sources = %d\n", scanner_info.lightsources);
	printk(KERN_INFO "scanning colors = %d (%s)\n",scanner_info.colors, (scanner_info.colors==1)?"greyscale":"RGB color");
	printk(KERN_INFO "scanning sides = %d:\n", scanner_info.sides);
	printk(KERN_INFO "  side A -- image sensor ID = %d, number of sections = %d;\n", scanner_info.sensor_a, scanner_info.sections_a);
	if (scanner_info.sides == 2)
		printk(KERN_INFO "  side B -- image sensor ID = %d, number of sections = %d;\n", scanner_info.sensor_b, scanner_info.sections_b);
	printk(KERN_INFO "Section   Physical  Physical  Logical   Logical   Digitiser Digitiser\n");
	printk(KERN_INFO "Number    Start     End       Start     End       ID        Channel\n");
	sect = scanner_info.sectinfo_a;
	for (i = 0; i < scanner_info.sections_a; i++) {
		printk(KERN_INFO " A-%d\t%6d%10d%10d%10d%8d%10d\n", i+1, sect[i].pstart, sect[i].pend, 
			    sect[i].lstart, sect[i].lend, sect[i].digitiser_id, sect[i].channel); 
	}
	if (scanner_info.sections_b) {
		sect = scanner_info.sectinfo_b;
		for (i = 0; i < scanner_info.sections_b; i++) {
			printk(KERN_INFO " B-%d\t%6d%10d%10d%10d%8d%10d\n", i+1, sect[i].pstart, sect[i].pend,
				     sect[i].lstart, sect[i].lend, sect[i].digitiser_id, sect[i].channel); 
		}
	}
#endif
	return 0;
}


static irqreturn_t scanunit_isr(int irq, void *dev_id)
{
	complete(&img_completion);
	return IRQ_HANDLED;
}

static int scanunit_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct device *temp_dev;
	u32 fpga_cs;
	u32 reg[3];
	int ret;

	memset((void *)&scanner_info, 0, sizeof(scanner_info));
	memset((void *)&image_digitisers, 0, sizeof(image_digitisers));
	memset((void *)&image_sensors, 0, sizeof(image_sensors));

	ret = of_property_read_u32_array(np, "reg-ctrl", reg, 3);
	if (IS_ERR_VALUE(ret)) {
		dev_err(&pdev->dev, "invalid SCANUNIT registers  definition\n");
		return ret;
	}
	fpga_cs = reg[0];
	cis_reg_base = fpga_io_get(fpga_cs);
	if (IS_ERR(cis_reg_base))
		return PTR_ERR(cis_reg_base);
	cis_reg_base += reg[1];

	ret = of_property_read_u32_array(np, "reg-ints", reg, 3);
	if (IS_ERR_VALUE(ret)) {
		dev_err(&pdev->dev, "invalid SCANUNIT INT registers  definition\n");
		return ret;
	}
	fpga_cs = reg[0];
	ints_reg_base = fpga_io_get(fpga_cs);
	if (IS_ERR(ints_reg_base))
		return PTR_ERR(ints_reg_base);
	ints_reg_base += reg[1];

	ret = of_property_read_u32(np, "iram-cs", &fpga_cs);
	dev_dbg(&pdev->dev, "fpga_cs of scanunit image RAM is %d\n", fpga_cs);
	if (IS_ERR_VALUE(ret)) {
		dev_err(&pdev->dev, "Failed to parse chip select number of scanunit image RAM\n");
		return ret;
	}
	imgram_base = fpga_io_get(fpga_cs);
	if (IS_ERR(imgram_base)) {
		ret = PTR_ERR(imgram_base);
		dev_err(&pdev->dev, "Failed to remap scanunit image RAM, err = %d\n", ret);
		return ret;
	}

	ret = of_property_read_u32(np, "fpga-irq", &irq);
	dev_dbg(&pdev->dev, "irq of scanunit is %d\n", irq);
	if (IS_ERR_VALUE(ret)) {
		dev_err(&pdev->dev, "Failed to parse FPGA interrupt number of scanunit\n");
		return ret;
	}

	ret = fpga_request_irq(irq, scanunit_isr, NULL);
	if (IS_ERR_VALUE(ret)) {
		dev_err(&pdev->dev, "Failed to request FPGA irq of scanunit\n");
		return ret;
	}

	ret = scanunit_parse_hwinfo(&pdev->dev); 
	if (IS_ERR_VALUE(ret)) {
		dev_err(&pdev->dev, "Failed to parse hardware information of scanunit\n");
		return ret;
	}

	scanunit_dev_no = MKDEV(scanunit_major, 0);
	printk( KERN_INFO "scanunit_major= %x\n", scanunit_major);

	if(scanunit_major)
		ret = register_chrdev_region(scanunit_dev_no, 1, SCANUNIT_NAME);
	else
	{
		ret = alloc_chrdev_region(&scanunit_dev_no, 0, 1, SCANUNIT_NAME);
		scanunit_major = MAJOR(scanunit_dev_no);
	}
	
	printk( KERN_INFO ": scanunit_major= %x, scanunit_dev_no=%x ret=%d\n", scanunit_major, scanunit_dev_no, ret);

    	if (ret < 0)
		goto error_return1;

	cdev_init(&scanunit_cdev, &scanunit_fops);
	scanunit_cdev.owner = THIS_MODULE;

	ret = cdev_add(&scanunit_cdev, scanunit_dev_no, 1);
	if(ret)
	{
		printk(KERN_NOTICE "Error %x adding scanunit", ret);
		goto error_return1;
	}

	scanunit_class = class_create(THIS_MODULE, SCANUNIT_NAME);
	if (IS_ERR(scanunit_class)) {
		dev_err(&pdev->dev, "Error creating image scanner module class.\n");
		ret = PTR_ERR(scanunit_class);
		goto error_return2;
	}

	temp_dev = device_create(scanunit_class, NULL, MKDEV(scanunit_major, 0), NULL, SCANUNIT_NAME);
	if (IS_ERR(temp_dev)) {
		dev_err(&pdev->dev, "Error creating image scanner class device.\n");
		ret = PTR_ERR(temp_dev);
		goto error_return3;
	}

	init_completion(&img_completion);

	return 0;

error_return3:
	class_destroy(scanunit_class);
error_return2:
	cdev_del(&scanunit_cdev);
error_return1:
	fpga_free_irq(irq);

	return ret;
}

static int scanunit_remove(struct platform_device *pdev)
{
	device_destroy(scanunit_class, MKDEV(scanunit_major, 0));
	class_destroy(scanunit_class);
	cdev_del(&scanunit_cdev);
	unregister_chrdev_region(scanunit_dev_no, 1);
	fpga_free_irq(irq);
	return 0;
}


static const struct of_device_id scanunit_match[] = {
	{ .compatible = "gwi,scanunit", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, scanunit_match);

static struct platform_driver scanunit_driver = {
	.probe          = scanunit_probe,
	.remove         = scanunit_remove,
	.driver         = {
		.name   = "scan_unit",
		.of_match_table = scanunit_match,
	},
};

module_platform_driver(scanunit_driver);

MODULE_AUTHOR("Zhang Xudong <zhangxudong@gwi.com.cn>");
MODULE_DESCRIPTION("Image Scan Unit driver");
MODULE_LICENSE("GPL v2");
