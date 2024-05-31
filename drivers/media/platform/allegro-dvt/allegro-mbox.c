// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 Pengutronix, Michael Tretter <kernel@pengutronix.de>
 *
 * Allegro DVT video encoder driver
 */

#include "allegro-mbox.h"

struct allegro_mbox *allegro_mbox_init(struct allegro_dev *dev,
					      unsigned int base, size_t size)
{
	struct allegro_mbox *mbox;

	mbox = devm_kmalloc(&dev->plat_dev->dev, sizeof(*mbox), GFP_KERNEL);
	if (!mbox)
		return ERR_PTR(-ENOMEM);

	mbox->dev = dev;

	mbox->head = base;
	mbox->tail = base + 0x4;
	mbox->data = base + 0x8;
	mbox->size = size;
	mutex_init(&mbox->lock);

	regmap_write(dev->sram, mbox->head, 0);
	regmap_write(dev->sram, mbox->tail, 0);

	return mbox;
}

int allegro_mbox_write(struct allegro_mbox *mbox,
			      const u32 *src, size_t size)
{
	struct regmap *sram = mbox->dev->sram;
	unsigned int tail;
	size_t size_no_wrap;
	int err = 0;
	int stride = regmap_get_reg_stride(sram);

	if (!src)
		return -EINVAL;

	if (size > mbox->size)
		return -EINVAL;

	mutex_lock(&mbox->lock);
	regmap_read(sram, mbox->tail, &tail);
	if (tail > mbox->size) {
		err = -EIO;
		goto out;
	}
	size_no_wrap = min(size, mbox->size - (size_t)tail);
	regmap_bulk_write(sram, mbox->data + tail,
			  src, size_no_wrap / stride);
	regmap_bulk_write(sram, mbox->data,
			  src + (size_no_wrap / sizeof(*src)),
			  (size - size_no_wrap) / stride);
	regmap_write(sram, mbox->tail, (tail + size) % mbox->size);

out:
	mutex_unlock(&mbox->lock);

	return err;
}

ssize_t allegro_mbox_read(struct allegro_mbox *mbox,
				 u32 *dst, size_t nbyte)
{
	struct {
		u16 length;
		u16 type;
	} __attribute__ ((__packed__)) *header;
	struct regmap *sram = mbox->dev->sram;
	unsigned int head;
	ssize_t size;
	size_t body_no_wrap;
	int stride = regmap_get_reg_stride(sram);

	regmap_read(sram, mbox->head, &head);
	if (head > mbox->size)
		return -EIO;

	/* Assume that the header does not wrap. */
	regmap_bulk_read(sram, mbox->data + head,
			 dst, sizeof(*header) / stride);
	header = (void *)dst;
	size = header->length + sizeof(*header);
	if (size > mbox->size || size & 0x3)
		return -EIO;
	if (size > nbyte)
		return -EINVAL;

	/*
	 * The message might wrap within the mailbox. If the message does not
	 * wrap, the first read will read the entire message, otherwise the
	 * first read will read message until the end of the mailbox and the
	 * second read will read the remaining bytes from the beginning of the
	 * mailbox.
	 *
	 * Skip the header, as was already read to get the size of the body.
	 */
	body_no_wrap = min((size_t)header->length,
			   (size_t)(mbox->size - (head + sizeof(*header))));
	regmap_bulk_read(sram, mbox->data + head + sizeof(*header),
			 dst + (sizeof(*header) / sizeof(*dst)),
			 body_no_wrap / stride);
	regmap_bulk_read(sram, mbox->data,
			 dst + (sizeof(*header) + body_no_wrap) / sizeof(*dst),
			 (header->length - body_no_wrap) / stride);

	regmap_write(sram, mbox->head, (head + size) % mbox->size);

	return size;
}

/**
 * allegro_mbox_send() - Send a message via the mailbox
 * @mbox: the mailbox which is used to send the message
 * @msg: the message to send
 */
int allegro_mbox_send(struct allegro_mbox *mbox, void *msg)
{
	struct allegro_dev *dev = mbox->dev;
	ssize_t size;
	int err;
	u32 *tmp;

	tmp = kzalloc(mbox->size, GFP_KERNEL);
	if (!tmp) {
		err = -ENOMEM;
		goto out;
	}

	size = allegro_encode_mail(tmp, msg);

	err = allegro_mbox_write(mbox, tmp, size);
	kfree(tmp);
	if (err)
		goto out;

	allegro_mcu_interrupt(dev);

out:
	return err;
}

/**
 * allegro_mbox_notify() - Notify the mailbox about a new message
 * @mbox: The allegro_mbox to notify
 */
void allegro_mbox_notify(struct allegro_mbox *mbox)
{
	struct allegro_dev *dev = mbox->dev;
	union mcu_msg_response *msg;
	ssize_t size;
	u32 *tmp;
	int err;

	msg = kmalloc(sizeof(*msg), GFP_KERNEL);
	if (!msg)
		return;

	msg->header.version = dev->fw_info->mailbox_version;

	tmp = kmalloc(mbox->size, GFP_KERNEL);
	if (!tmp)
		goto out;

	size = allegro_mbox_read(mbox, tmp, mbox->size);
	if (size < 0)
		goto out;

	err = allegro_decode_mail(msg, tmp);
	if (err)
		goto out;

	allegro_handle_message(dev, msg);

out:
	kfree(tmp);
	kfree(msg);
}

