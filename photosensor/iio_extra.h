/*
 * Nostandard extra IIO functions definitions
 *
 * Copyright 2016 Hunan GreatWall Information Financial Equipment Co., Ltd.
 *
 */
#ifndef __IIO_EXTRA_H__
#define __IIO_EXTRA_H__

#include <linux/iio/types.h>


static inline int iio_channel_read(struct iio_channel *chan, int *val, int *val2,
	enum iio_chan_info_enum info)
{
	int unused;
	int ret;

	if (val2 == NULL)
		val2 = &unused;

	if(!iio_channel_has_info(chan->channel, info))
		return -EINVAL;

	ret = chan->indio_dev->info->read_raw(chan->indio_dev,
					chan->channel, val, val2, info);
	return ret;
}


static inline int iio_read_event_config(struct iio_channel *chan, enum iio_event_type type,
	enum iio_event_direction dir)
{
	int ret;

	if (chan->indio_dev->info == NULL)
		ret = -ENODEV;
	else
		ret = chan->indio_dev->info->read_event_config(chan->indio_dev, chan->channel, type, dir);

	return ret;
}


static inline int iio_write_event_config(struct iio_channel *chan, enum iio_event_type type, 
	enum iio_event_direction dir, int state)
{
	int ret;

	mutex_lock(&chan->indio_dev->info_exist_lock);
	if (chan->indio_dev->info == NULL) {
		ret = -ENODEV;
		goto err_unlock;
	}
	ret = chan->indio_dev->info->write_event_config(chan->indio_dev, chan->channel, type, dir, state);

err_unlock:
	mutex_unlock(&chan->indio_dev->info_exist_lock);

	return ret;
}


static inline int iio_read_event_value(struct iio_channel *chan, enum iio_event_type type,
	enum iio_event_direction dir, enum iio_event_info info,
	int *val, int *val2)
{
	int ret;

	mutex_lock(&chan->indio_dev->info_exist_lock);
	if (chan->indio_dev->info == NULL) {
		ret = -ENODEV;
		goto err_unlock;
	}
	ret = chan->indio_dev->info->read_event_value(chan->indio_dev, chan->channel, type, dir, info, val, val2);

err_unlock:
	mutex_unlock(&chan->indio_dev->info_exist_lock);

	return ret;
}


static inline int iio_write_event_value(struct iio_channel *chan, enum iio_event_type type,
	enum iio_event_direction dir, enum iio_event_info info,
	int val, int val2)
{
	int ret;

	mutex_lock(&chan->indio_dev->info_exist_lock);
	if (chan->indio_dev->info == NULL) {
		ret = -ENODEV;
		goto err_unlock;
	}
	ret = chan->indio_dev->info->write_event_value(chan->indio_dev, chan->channel, type, dir, info, val, val2);

err_unlock:
	mutex_unlock(&chan->indio_dev->info_exist_lock);

	return ret;
}


#endif /* __IIO_EXTRA_H__ */
