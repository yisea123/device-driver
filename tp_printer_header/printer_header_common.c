

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>

#include "tp_printer_header.h"

static DEFINE_MUTEX(printer_header_mutex);
static LIST_HEAD(printer_header_list);

int ph_add(struct tp_ph_t * pph)
{
	if(!pph || !pph->dev)
	{
		return -EINVAL;
	}
	mutex_lock(&printer_header_mutex);

	INIT_LIST_HEAD(&pph->list);
	list_add(&pph->list, &printer_header_list);

	mutex_unlock(&printer_header_mutex);
	return 0;
}
EXPORT_SYMBOL_GPL(ph_add);

int ph_remove(struct tp_ph_t * pph)
{
	mutex_lock(&printer_header_mutex);

	list_del_init(&pph->list);

	mutex_unlock(&printer_header_mutex);
	return 0;
}
EXPORT_SYMBOL_GPL(ph_remove);

struct tp_ph_t * of_node_to_ph(struct device_node * np)
{
	struct tp_ph_t *pph;

	mutex_lock(&printer_header_mutex);
	list_for_each_entry(pph, &printer_header_list, list)
	{
		if (pph->dev && pph->dev->of_node == np)
		{
			mutex_unlock(&printer_header_mutex);
			return pph;
		}
	}
	mutex_unlock(&printer_header_mutex);

	return ERR_PTR(-EPROBE_DEFER);
}
EXPORT_SYMBOL_GPL(of_node_to_ph);

int ph_config(struct tp_ph_t * ptp_ph, struct tp_ph_config_t * pconfig)
{
	if((!ptp_ph) || (!pconfig))
	{
		return -EINVAL;
	}
	return ptp_ph->ops->config(ptp_ph, pconfig);
}
EXPORT_SYMBOL_GPL(ph_config);

int ph_write_data(struct tp_ph_t * ptp_ph, unsigned char * pbuffer, unsigned int data_size)
{
	if((!ptp_ph) || (!pbuffer) || (!data_size))
	{
		return -EINVAL;
	}
	return ptp_ph->ops->write_data(ptp_ph, pbuffer, data_size);
}
EXPORT_SYMBOL_GPL(ph_write_data);
