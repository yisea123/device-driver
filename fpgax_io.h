/*
 * FPGA IO device driver definitions
 *
 * Copyright 2016 Hunan GreatWall Information Financial Equipment Co., Ltd.
 *
 */
#ifndef __FPGAX_IO_H__
#define __FPGAX_IO_H__

#include <linux/io.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>


extern int fpgax_reconfigure(struct device *dev);
extern void __iomem *fpgax_io_get(u32 chipsel);
extern void fpgax_io_put(void __iomem *addr);
extern int fpgax_request_irq(unsigned int irq, irq_handler_t handler, void * sub_dev_id);    
extern int fpgax_free_irq(unsigned int irq);


extern int fpgax_readl(u32 *value, const volatile void __iomem *addr);
extern int fpgax_writel(u32 value, volatile void __iomem *addr);
extern int fpgax_readnl(const void __iomem *addr, void *buffer, int count);
extern int fpgax_writenl(void __iomem *addr, const void *buffer, int count);
extern int fpgax_update_lbits(volatile void __iomem *addr, u32 mask, u32 value);


#endif /* __FPGA_IO_H__ */
