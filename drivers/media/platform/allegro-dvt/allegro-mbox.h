// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 Pengutronix, Michael Tretter <kernel@pengutronix.de>
 *
 * Allegro DVT video encoder driver
 */

#ifndef __ALLEGRO_MBOX_H__
#define __ALLEGRO_MBOX_H__

#include "allegro.h"
#include "allegro-mail.h"

/*
 * Mailbox interface to send messages to the MCU.
 */

struct allegro_mbox *allegro_mbox_init(struct allegro_dev *dev,
					      unsigned int base, size_t size);

int allegro_mbox_write(struct allegro_mbox *mbox,
			      const u32 *src, size_t size);

ssize_t allegro_mbox_read(struct allegro_mbox *mbox,
				 u32 *dst, size_t nbyte);


/**
 * allegro_mbox_send() - Send a message via the mailbox
 * @mbox: the mailbox which is used to send the message
 * @msg: the message to send
 */
int allegro_mbox_send(struct allegro_mbox *mbox, void *msg);

/**
 * allegro_mbox_notify() - Notify the mailbox about a new message
 * @mbox: The allegro_mbox to notify
 */
void allegro_mbox_notify(struct allegro_mbox *mbox);

#endif // __ALLEGRO_MBOX_H__

