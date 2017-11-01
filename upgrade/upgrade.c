#include <linux/module.h>
#include <asm/uaccess.h>
#include <linux/version.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <asm/mach/arch.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/regmap.h>
#include <../arch/arm/mach-imx/common.h>
#include <../arch/arm/mach-imx/hardware.h>

#define SET_FLAG_ENABLE				0x88888888	/* 在线下载标志*/
#define DOWN_FLAG_ADDR				0x020d8030	/* 在线下载寄存器地址*/

static void __iomem *DOWNLOAD_FLAG_ADDRESS = MX6Q_IO_ADDRESS(DOWN_FLAG_ADDR);

struct kobject *kobj;

static ssize_t upgrade_show(struct kobject *kobj, struct kobj_attribute *attr,char *buf )
{
	int tmp;
	tmp =readl(DOWNLOAD_FLAG_ADDRESS);
	if (tmp == SET_FLAG_ENABLE){ 
		printk(KERN_INFO "dwnld enable\n");
	}
	else{
		printk(KERN_INFO "dwnld disable\n");
	}
	return 0;
}
static ssize_t upgrade_store(struct kobject *kobj,struct kobj_attribute *attr,const char* buf, size_t n)
{
	int tmp;

	writel(SET_FLAG_ENABLE, DOWNLOAD_FLAG_ADDRESS); 
	tmp =readl(DOWNLOAD_FLAG_ADDRESS);
	if (tmp == SET_FLAG_ENABLE){ 
		printk(KERN_INFO "write ok dwnld enable\n");
	}
	else{
		printk(KERN_INFO "write fail dwnld disable\n");
	}
	return tmp ;
}

static struct kobj_attribute  kobj_attr ={
	.attr = {
		.name =  __stringify(upgrade), 
		.mode = 0644, //表示此属性可读可写
	},
	.show = upgrade_show, //对应读
	.store= upgrade_store, //对应写

};

static struct attribute *attr_g[] ={
	&kobj_attr.attr,
	NULL,
};

static struct attribute_group grp ={
	.attrs = attr_g,
};
    
static int __init upgrade_init(void)
{

    
	kobj = kobject_create_and_add("upgrade",NULL);
	if(!kobj )
	{
		printk(KERN_INFO "kobject don't create \n");
		return -ENOMEM;
	}
	return  sysfs_create_group(kobj, &grp);

}
static void __exit upgrade_exit(void)
{
	printk(KERN_INFO "%s:enter\n",__func__);
	sysfs_remove_group(kobj, &grp);
	kobject_del(kobj);
}

module_init(upgrade_init);
module_exit(upgrade_exit);
MODULE_LICENSE("GPL");
