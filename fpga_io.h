/*
 * FPGA IO device driver definitions
 *
 * Copyright 2016 Hunan GreatWall Information Financial Equipment Co., Ltd.
 *
 */
#ifndef __FPGA_IO_H__
#define __FPGA_IO_H__

#include <linux/io.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>


extern int fpga_reconfigure(struct device *dev);
extern void __iomem *fpga_io_get(u32 chipsel);
extern void fpga_io_put(void __iomem *addr);
extern int fpga_request_irq(unsigned int irq, irq_handler_t handler, void * sub_dev_id);    
extern int fpga_free_irq(unsigned int irq);


#if defined(CONFIG_FPGA_EIM)

static inline int fpga_readl(u32 *value, const volatile void __iomem *addr)
{
	*value = (readw(addr) & 0x0000ffff);
	return 0;
}

static inline int fpga_writel(u32 value, volatile void __iomem *addr)
{
	writew(value, addr);
	return 0;
}

static inline int fpga_readnl(const void __iomem *addr, void *buffer, int count)
{
	u32 *lptr = buffer;
	if (addr == NULL || buffer == NULL || count <= 0)
		return -EINVAL;
	while (count-- > 0) {
		*lptr++ = readl(addr);
		addr += sizeof(u32);
	}
	return 0;
}

static inline int fpga_writenl(void __iomem *addr, const void *buffer, int count)
{
	u32 *lptr = (u32 *)buffer;

	if (addr == NULL || buffer == NULL || count <= 0)
		return -EINVAL;

	while (count-- > 0) {
		writel(*lptr++, addr);
		addr += sizeof(u32);
	}
	return 0;
}

static inline int fpga_readnw(const void __iomem *addr, void *buffer, int count)
{
	u32 *lptr = buffer;
	if (addr == NULL || buffer == NULL || count <= 0)
		return -EINVAL;

	while (count-- > 0) {

		*lptr++ = readw(addr);
		addr += sizeof(u16);
	}
	return 0;
}

static inline int fpga_writenw(void __iomem *addr, const void *buffer, int count)
{
	u16 *lptr = (u16 *)buffer;
	if (addr == NULL || buffer == NULL || count <= 0)
		return -EINVAL;
	while (count-- > 0) {
		writel(*lptr++, addr);
		addr += sizeof(u16);
	}
	return 0;
}

static inline int fpga_update_lbits(volatile void __iomem *addr, u32 mask, u32 value)
{
	u32 val = readl(addr);
	val = (val & ~mask) | (value & mask);
	writel(val, addr);
	return 0;
}

#elif defined(CONFIG_FPGA_SPI)

extern int fpga_readl(u32 *value, const volatile void __iomem *addr);
extern int fpga_writel(u32 value, volatile void __iomem *addr);
extern int fpga_readnl(const void __iomem *addr, void *buffer, int count);
extern int fpga_writenl(void __iomem *addr, const void *buffer, int count);
extern int fpga_update_lbits(volatile void __iomem *addr, u32 mask, u32 value);

#elif defined(CONFIG_FPGA_DUMMY)

static inline int fpga_readl(u32 *value, const volatile void __iomem *addr)
{
	printk("fpga_readl @ addr: %08x value = %08x\n",(u32)addr, *value);
	return 0;
}

static inline int fpga_writel(u32 value, volatile void __iomem *addr)
{
	printk("fpga_writel @ addr: %08x value = %08x\n",(u32)addr, value);
	return 0;
}

static inline int fpga_readnl(const void __iomem *addr, void *buffer, int count)
{
	printk("fpga_readnl @ addr: %08x count = %08x\n",(u32)addr, count);
	return 0;
}

static inline int  fpga_writenl(void __iomem *addr, const void *buffer, int count)
{
	printk("fpga_writenl @ addr: %08x count = %08x\n",(u32)addr, count);
	return 0;
}

static inline int fpga_update_lbits(volatile void __iomem *addr, u32 mask, u32 value)
{
	printk("fpga_update_lbits @ addr: %08x mask = %08x val = %08x\n",(u32)addr, mask, value);
	return 0;
}

#else

static inline int fpga_readl(u32 *value, const volatile void __iomem *addr)
{
	return -ENODEV;
}

static inline int fpga_writel(u32 value, volatile void __iomem *addr)
{
	return -ENODEV;
}

static inline int fpga_readnl(const void __iomem *addr, void *buffer, int count)
{
	return -ENODEV;
}

static inline int  fpga_writenl(void __iomem *addr, const void *buffer, int count)
{
	return -ENODEV;
}

static inline int fpga_update_lbits(volatile void __iomem *addr, u32 mask, u32 value)
{
	return -ENODEV;
}

#endif

#endif /* __FPGA_IO_H__ */
