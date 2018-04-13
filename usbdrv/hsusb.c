
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/utsname.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/usb/composite.h>
#include <linux/of_platform.h>
#include <linux/completion.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/usb/gadget.h>
#include <linux/timer.h>
#include <asm/uaccess.h>

#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include "hsusb.h"

#include "composite.c"
#include "usbstring.c"
#include "config.c"
#include "epautoconf.c"

struct gpio *usb_reset_gpios;
#define  GPIO_VALUE_HIGH 1
#define  GPIO_VALUE_LOW  0

#define SEND_BUFLEN (128*1024)
#define RECV_BUFLEN (1024)

#define DRIVER_VENDOR_NUM       0x206d          
#define DRIVER_PRODUCT_NUM      0x0003 

#define STRING_MANUFACTURER_IDX   0
#define STRING_PRODUCT_IDX        1
#define STRING_SERIAL_IDX        2

#define SIMPLE_IO_TIMEOUT   10000   /* in milliseconds */ 

unsigned char data_buf[RECV_BUFLEN] = {0};


char productname[50] = "Gwi Hsusb";
static char manufacturer[50] = "GWI";
static char serial[] = "12345678.12345678";

struct f_sendrecv {
	struct usb_function function;
	struct usb_ep       *in_ep;
	struct usb_ep       *out_ep;
	struct completion gdt_completion;
	spinlock_t		lock;
	unsigned char data[RECV_BUFLEN];
	unsigned int actual;//actual date len
	wait_queue_head_t rx_wait; 
	wait_queue_head_t tx_wait; 
	bool is_read;
	bool is_write;
};

static inline struct f_sendrecv *func_to_ss(struct usb_function *f)
{
	return container_of(f, struct f_sendrecv, function);
}

/*-------------------------------------*/
/*interface des*/
static struct usb_interface_descriptor send_recv_intf = {
	.bLength =        sizeof send_recv_intf,
	.bDescriptorType =    USB_DT_INTERFACE,
	.bNumEndpoints =      2,
	.bInterfaceClass =    USB_CLASS_VENDOR_SPEC,
	.bInterfaceSubClass = 00,
	.bInterfaceProtocol = 00,
	/* .iInterface = DYNAMIC */
};

/* full speed support: */
static struct usb_endpoint_descriptor fs_send_desc = {
	.bLength =        USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =    USB_DT_ENDPOINT,

	.bEndpointAddress =   1|USB_DIR_IN,
	.bmAttributes =       USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor fs_recv_desc = {
	.bLength =        USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =    USB_DT_ENDPOINT,

	.bEndpointAddress =    2|USB_DIR_OUT,
	.bmAttributes =        USB_ENDPOINT_XFER_BULK,
};

static struct usb_descriptor_header *fs_send_recv_descs[] = {
	(struct usb_descriptor_header *) &send_recv_intf,
	(struct usb_descriptor_header *) &fs_send_desc,
	(struct usb_descriptor_header *) &fs_recv_desc,
	NULL,
};

/* high speed support: */
static struct usb_endpoint_descriptor hs_send_desc = {
	.bLength =        USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =   USB_DT_ENDPOINT,

	.bmAttributes =      USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =    cpu_to_le16(512),
};

static struct usb_endpoint_descriptor hs_recv_desc = {
	.bLength =        USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =   USB_DT_ENDPOINT,

	.bmAttributes =      USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =    cpu_to_le16(512),
};

static struct usb_descriptor_header *hs_send_recv_descs[] = {
	(struct usb_descriptor_header *) &send_recv_intf,
	(struct usb_descriptor_header *) &hs_send_desc,
	(struct usb_descriptor_header *) &hs_recv_desc,
	NULL,
};

/* function-specific strings: */
static struct usb_string strings_sendrecv[] = {
	[0].s = "send and recv data",
	{  }            /* end of list */
};

static struct usb_gadget_strings stringtab_sendrecv = {
	.language    = 0x0409,    /* en-us */
	.strings    = strings_sendrecv,
};

static struct usb_gadget_strings *sendrecv_strings[] = {
	&stringtab_sendrecv,
	NULL,
};

/*usb device des*/
static struct usb_device_descriptor device_desc = {
	.bLength =         sizeof device_desc,
	.bDescriptorType = USB_DT_DEVICE,

	.bcdUSB =         cpu_to_le16(0x0200),
	.bDeviceClass =   USB_CLASS_VENDOR_SPEC,

	.idVendor =       cpu_to_le16(DRIVER_VENDOR_NUM),
	.idProduct =      cpu_to_le16(DRIVER_PRODUCT_NUM),
	.bNumConfigurations =   1,
};

/*-------------------------------------*/
struct usb_request *alloc_ep_req(struct usb_ep *ep , unsigned len)
{
	struct usb_request    *req;

	req = usb_ep_alloc_request(ep, GFP_KERNEL);
	if (req != NULL) {
		req->length = len;
		req->buf = kmalloc(len, GFP_KERNEL);
		if (!req->buf) {
		    usb_ep_free_request(ep, req);
		    printk(KERN_ERR "kmalloc error\n");
		    req = NULL;
		}
	}
	return req;
}

void free_ep_req(struct usb_ep *ep, struct usb_request *req)
{	
	if ((ep != NULL)&&(req != 0)) {
		kfree(req->buf);
		usb_ep_free_request(ep, req);
	}
}

static void disable_ep(struct usb_composite_dev *cdev, struct usb_ep *ep)
{
	int value;

	if (ep->driver_data) {
		value = usb_ep_disable(ep);
		ep->driver_data = NULL;
	}
}

void disable_endpoints(struct usb_composite_dev *cdev, struct usb_ep *in, struct usb_ep *out)
{
	disable_ep(cdev, in);
	disable_ep(cdev, out);
}

static int __init sendrecv_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct f_sendrecv    *ss = func_to_ss(f);
	int    id;

	id = usb_interface_id(c, f);
	if (id < 0)
		return id;

	send_recv_intf.bInterfaceNumber = id;

	usb_ep_autoconfig_reset(cdev->gadget);    
	ss->in_ep = usb_ep_autoconfig(cdev->gadget, &fs_send_desc);
	if (!ss->in_ep) {
autoconf_fail:
		return -ENODEV;
	}
	ss->in_ep->driver_data = cdev;    /* claim */

	ss->out_ep = usb_ep_autoconfig(cdev->gadget, &fs_recv_desc);

	if (!ss->out_ep)
		goto autoconf_fail;
	ss->out_ep->driver_data = cdev;    /* claim */

	/* support high speed hardware */
	if (gadget_is_dualspeed(c->cdev->gadget)) {
		hs_send_desc.bEndpointAddress =
			fs_send_desc.bEndpointAddress;
		hs_recv_desc.bEndpointAddress =
			fs_recv_desc.bEndpointAddress;
		f->hs_descriptors = hs_send_recv_descs;
	}
	return 0;
}

static void sendrecv_unbind(struct usb_configuration *c, struct usb_function *f)
{
	kfree(func_to_ss(f));
}

static void disable_send_recv(struct f_sendrecv *ss)
{
	struct usb_composite_dev    *cdev;
	cdev = ss->function.config->cdev;
	disable_ep(cdev, ss->in_ep);
	disable_ep(cdev, ss->out_ep);
	ss->in_ep->desc = NULL;
	ss->out_ep->desc = NULL;
}

static int enable_send_recv(struct usb_composite_dev *cdev, struct f_sendrecv *ss)
{
	int                    result = 0;
	struct usb_ep                *ep;   

	/* one endpoint writes (sends) zeroes IN (to the host) */
	ep = ss->in_ep;
	result = config_ep_by_speed(cdev->gadget, &(ss->function), ep);
	if (result)
		goto out_fail;

	result = usb_ep_enable(ep);
	if (result < 0)
		goto in_fail;

	ep->driver_data = ss;

	/* one endpoint reads (recvs) anything OUT (from the host) */
	ep = ss->out_ep;
	result = config_ep_by_speed(cdev->gadget, &(ss->function), ep);	
	if (result)
		goto out_fail;

	result = usb_ep_enable(ep);
	if (result < 0)
		goto out_fail;

	ep->driver_data = ss;
		return result;

in_fail:
	ep = ss->in_ep;
	usb_ep_disable(ep);
	ep->driver_data = NULL;
	return result;
out_fail:
	ep = ss->out_ep;
	usb_ep_disable(ep);
	ep->driver_data = NULL;
	return result;
}

static int sendrecv_set_alt(struct usb_function *f, unsigned intf, unsigned alt)
{
	struct f_sendrecv    *ss = func_to_ss(f);
	struct usb_composite_dev *cdev = f->config->cdev;

	if (ss->in_ep->driver_data)
		disable_send_recv(ss);
	return enable_send_recv(cdev, ss);
}

static void sendrecv_disable(struct usb_function *f)
{
	struct f_sendrecv    *ss = func_to_ss(f);

	disable_send_recv(ss);
}

/*-------------------------------------*/

static struct f_sendrecv    *ss;

static int __init sendrecv_bind_config(struct usb_configuration *c)
{
	int            status;

	ss = kzalloc(sizeof *ss, GFP_KERNEL);
	if (!ss)
		return -ENOMEM;

	ss->function.name = "send/recv";
	ss->function.hs_descriptors = fs_send_recv_descs;
	ss->function.bind = sendrecv_bind;
	ss->function.unbind = sendrecv_unbind;
	ss->function.set_alt = sendrecv_set_alt;
	ss->function.disable = sendrecv_disable;

	status = usb_add_function(c, &ss->function);

	if (status)
		kfree(ss);
	return status;
}

static int sendrecv_setup(struct usb_configuration *f, const struct usb_ctrlrequest *ctrl)
{
	return 0;
}

static struct usb_configuration sendrecv_driver = {
	.label        = "send/recv",
	.strings      = sendrecv_strings,
	.setup        = sendrecv_setup,
	.bConfigurationValue = 1,
	.bmAttributes = USB_CONFIG_ATT_SELFPOWER,
	/* .iConfiguration = DYNAMIC */
};

/**
 * sendrecv_add - add a send/recv testing configuration to a device
 * @cdev: the device to support the configuration
 */
static int __init sendrecv_add(struct usb_composite_dev *cdev)
{
	int id;
	/* allocate string ID(s) */
	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_sendrecv[0].id = id;

	send_recv_intf.iInterface = id;
	sendrecv_driver.iConfiguration = id;

	return usb_add_config(cdev, &sendrecv_driver, sendrecv_bind_config);
}

struct gadget_misc
{
	struct miscdevice miscdevp;
};

/*-------------------------------------*/
static void send_recv_complete(struct usb_ep *ep, struct usb_request *req)
{
	int status = req->status;	

	switch (status) {
		case 0:                 /* normal completion? */
			if (req->actual > 0) {
				if (ep == ss->out_ep) {
					memset(ss->data, 0, sizeof(ss->data));
					memcpy(ss->data, req->buf, req->actual);
					wake_up_interruptible(&ss->rx_wait);
					ss->is_read = true;
				}else if (ep == ss->in_ep) {
					wake_up_interruptible(&ss->tx_wait);
					ss->is_write = true;
				}
				ss->actual = req->actual;
			}
			break;
		case -ECONNABORTED:     /* hardware forced ep reset */
		case -ECONNRESET:       /* request dequeued */
		case -ESHUTDOWN:        /* disconnect from host */
		case -EOVERFLOW: /*buffer overrun on read means that*/
		default:
		case -EREMOTEIO: /*short read*/   
			break;
	}
	free_ep_req(ep, req);
	complete(&ss->gdt_completion);
}

static int hsusb_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static int hsusb_hard_reset(struct f_sendrecv *ss)
{

	printk(KERN_ERR "*****husb reset\n");
//	gpio_direction_output(usb_reset_gpios->gpio, GPIOF_OUT_INIT_LOW);
	gpio_set_value(usb_reset_gpios->gpio,  GPIO_VALUE_HIGH);

	mdelay(200);
	gpio_set_value(usb_reset_gpios->gpio,  GPIO_VALUE_LOW);
	
	return 0;

}

static ssize_t hsusb_read(struct file *filp, char __user * buf, size_t count, loff_t * f_pos)
{
	struct usb_request  *req;
	int ret;

	ss->actual = 0;//add by jl 2017.3.10
	if(count == 0)
		return -EINVAL;

	init_completion(&ss->gdt_completion);
	req = alloc_ep_req(ss->out_ep, RECV_BUFLEN);
	if (!req){
		ret = -ENOMEM;
		goto fail;
	}

	req->complete = send_recv_complete;

	ret = usb_ep_queue(ss->out_ep, req, GFP_ATOMIC);
	if (ret) {
//		printk(KERN_ERR "###usb_ep_queue out_ep error\n");
		goto fail;
	}

	wait_for_completion(&ss->gdt_completion);

	ss->actual = (count < ss->actual) ? count : ss->actual;
	if (copy_to_user (buf, ss->data, ss->actual))
	{
		ret = -EFAULT;
		printk(KERN_ERR "copy_to_user fail\n");
		goto fail;
	}

	ss->is_read = false;
	return ss->actual;

fail:
	free_ep_req(ss->out_ep, req);
	ss->actual = 0;
	return ret;
}

static ssize_t hsusb_write(struct file *filp, const char __user * buf, size_t count, loff_t * f_pos)
{
	struct usb_request   *req;
	int ret;
	unsigned long       expire;
//	printk(KERN_ERR "#### write enter\n");
	if(count == 0)
		return -EINVAL;

	init_completion(&ss->gdt_completion);
	req = alloc_ep_req(ss->in_ep, SEND_BUFLEN);

	if (!req){
		return -ENOMEM;
	}

	req->length = (count < SEND_BUFLEN) ? count : SEND_BUFLEN;

	if (copy_from_user (req->buf, buf, req->length)) 
	{
		ret = -EFAULT;
		printk(KERN_ERR "#### write error\n");
		goto fail;
	}
	req->complete = send_recv_complete;

	ret = usb_ep_queue(ss->in_ep, req, GFP_ATOMIC);
	if (ret) 
	{
		printk(KERN_ERR "###usb_ep_queue in_ep error\n");
		goto fail;
	}

	expire = msecs_to_jiffies(SIMPLE_IO_TIMEOUT);
        if(!wait_for_completion_timeout(&ss->gdt_completion,expire))	
        {
                printk(KERN_ERR"****timed out on in_ep=%d!\r\n",ss->actual); 
		ret = -EAGAIN;
                goto fail_2;
        }
	ss->is_write = false;

	return ss->actual;

fail:
	free_ep_req(ss->in_ep, req);
fail_2:
	ss->actual = 0;
//	hsusb_hard_reset(ss);
	return ret;
}

static int hsusb_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static unsigned int hsusb_poll(struct file *filp, struct poll_table_struct *wait){
 
	unsigned int status = 0;       /*将等待队列添加到poll_table */
 
	poll_wait(filp, &ss->rx_wait, wait);
	poll_wait(filp, &ss->tx_wait, wait);
	printk("%d-%d\n", ss->is_read, ss->is_write);
	if (ss->is_read) {
		status |= POLLIN | POLLRDNORM;  /* readable */
	}
	else if (ss->is_write) {
		status |= POLLOUT | POLLWRNORM;  /* writeable */
	}

	return status;
}


static long
hsusb_ioctl(struct file *fd, unsigned int code, unsigned long arg)
{
//	struct printer_dev	*dev = fd->private_data;
	struct f_sendrecv 	*ss = fd->private_data;
	unsigned long		flags;

	printk(KERN_ERR "%s\n", __func__); 
//	DBG(dev, "printer_ioctl: cmd=0x%4.4x, arg=%lu\n", code, arg);
	printk(KERN_ERR "printer_ioctl: cmd=0x%4.4x, arg=%lu\n", code, arg);

	/* handle ioctls */

//	spin_lock_irqsave(&ss->lock, flags);

	switch (code) {
	case SET_HSUSB_SOFTRESET:
		hsusb_hard_reset(ss);
		break;
	default:
		return -EINVAL;
	}

//	spin_unlock_irqrestore(&ss->lock, flags);

	return 0;
}


static struct file_operations hsusb_fops = {
	.owner = 	THIS_MODULE,
	.open = 	hsusb_open,
	.read = 	hsusb_read,
	.write = 	hsusb_write,
	.release = 	hsusb_release,
	.poll =		hsusb_poll,
	.unlocked_ioctl = hsusb_ioctl,
};


static struct miscdevice hsusb_misc = {
	MISC_DYNAMIC_MINOR,
	"hsusb",
	&hsusb_fops,
};

/*-------------------------------------*/

static struct usb_string strings_dev[] = {
	[STRING_MANUFACTURER_IDX].s = manufacturer,
	[STRING_PRODUCT_IDX].s = productname,
	[STRING_SERIAL_IDX].s = serial,
	{  }            /* end of list */
};

static struct usb_gadget_strings stringtab_dev = {
	.language    = 0x0409,    /* en-us */
	.strings    = strings_dev,
};

static struct usb_gadget_strings *dev_strings[] = {
	&stringtab_dev,
	NULL,
};

static int hsusb_bind(struct usb_composite_dev *cdev)
{
	int id, ret;

	ret = misc_register(&hsusb_misc);
	if (ret){
		goto fail_reg;
	}

	//string description for index
	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_MANUFACTURER_IDX].id = id;
	device_desc.iManufacturer = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_PRODUCT_IDX].id = id;
	device_desc.iProduct = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_SERIAL_IDX].id = id;
	device_desc.iSerialNumber = id;

	sendrecv_add(cdev);

	init_completion(&ss->gdt_completion);
	init_waitqueue_head(&ss->rx_wait);
	init_waitqueue_head(&ss->tx_wait);
	ss->is_read = true;
	ss->is_write = false;
	device_desc.bcdDevice = cpu_to_le16(0x0002);

fail_reg:
	return 0;
}

static int hsusb_unbind(struct usb_composite_dev *cdev)
{
	misc_deregister(&hsusb_misc);
	return 0;
}


static struct usb_composite_driver hsusb_driver = {
	.name      = "hsusb",
	.dev       = &device_desc,
	.strings   = dev_strings,
	.max_speed = USB_SPEED_HIGH,
	.unbind    = hsusb_unbind,
	.bind      = hsusb_bind,
};

int gpio_init_from_dts(struct device *dev)
{
	int ret;
	struct device_node *np = dev->of_node; 

	usb_reset_gpios = devm_kzalloc(dev, 2*sizeof(struct gpio), GFP_KERNEL);
	if (usb_reset_gpios == NULL)
		return -ENOMEM;
	
	printk("gpio_init_from_dts enter!\n");
	usb_reset_gpios->gpio = of_get_named_gpio(np, "config_usb_gpio", 0);
	if (!usb_reset_gpios->gpio) {
		dev_err(dev, "no config_usb_gpio pin available\n");
		goto err;
	}
	usb_reset_gpios->flags = GPIOF_OUT_INIT_LOW;
	usb_reset_gpios->label = "config_usb_gpio";
	printk("gpio_init_from_dts: gpio_request_one=%d!\n", usb_reset_gpios->gpio);
	ret = gpio_request_one(usb_reset_gpios->gpio, usb_reset_gpios->flags, usb_reset_gpios->label);
	printk("gpio_request_one: ret=%d!\n", ret);
	if(ret)
	   return ret; 

	gpio_direction_output(usb_reset_gpios->gpio, GPIOF_OUT_INIT_LOW); 
	printk("gpio_init_from_dts over!\n");

err:
	return 0;
}

static int plat_hsusb_probe(struct platform_device *pdev)
{
	int ret;
	struct device_node *np = pdev->dev.of_node; 
	const char *ptr;
//	unsigned char usbclass;
	unsigned int usbclass, productid;

	ret =  gpio_init_from_dts(&pdev->dev);
	if (ret)
	{
		dev_err(&pdev->dev, "Failed to init gpio from dts: %d\n",ret);
		return ret;
	}
	
	ret= of_property_read_string(np, "usb-product-name", &ptr);
	if (ret){
		pr_debug("plat_hsusb_probe:Failed to read name of usb product name!\n");
		return ret;
	}
	strcpy(productname, ptr);
	strings_dev[STRING_PRODUCT_IDX].s = productname;

	ret = of_property_read_u32(np, "usb-class", &usbclass);
	if (ret){
		pr_debug("plat_hsusb_probe:Failed to read name of usb class!\n");
		return ret;
	}
	send_recv_intf.bInterfaceClass = usbclass;

	ret = of_property_read_u32(np, "usb-pid", &productid);
	if (ret){
		pr_debug("plat_hsusb_probe:Failed to read name of usb pid!\n");
		return ret;
	}
	device_desc.idProduct = cpu_to_le16(productid); 

	printk("hsusb probe ok: VID=0x%04x, PID=0x%04x, productname=%s.\n", DRIVER_VENDOR_NUM, productid, productname);
	return usb_composite_probe(&hsusb_driver);
}

static int plat_hsusb_remove(struct platform_device *pdev)
{
	usb_composite_unregister(&hsusb_driver);
	return 0;
}

static const struct of_device_id hsusb_match[] = {
	{ .compatible = "gwi,usb", },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, hsusb_match);

static struct platform_driver plat_hsusb_driver = {
	.driver		= {
		.name	= "hsusb",
		.of_match_table = hsusb_match,
	},
	.probe		= plat_hsusb_probe,
	.remove		= plat_hsusb_remove,
};

module_platform_driver(plat_hsusb_driver);

MODULE_LICENSE("GPL");
