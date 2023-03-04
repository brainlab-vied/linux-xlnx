// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 Pengutronix, Michael Tretter <kernel@pengutronix.de>
 *
 * Helper functions for handling messages that are send via mailbox to the
 * Allegro VCU firmware.
 */

#include <linux/bitfield.h>
#include <linux/export.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/videodev2.h>

#include "allegro-mail.h"

const char *msg_type_name(enum mcu_msg_type type)
{
	static char buf[9];

	switch (type) {
	case MCU_MSG_TYPE_INIT:
		return "INIT";
	default:
		snprintf(buf, sizeof(buf), "(0x%04x)", type);
		return buf;
	}
}

static ssize_t
allegro_enc_init(u32 *dst, struct mcu_msg_init_request *msg)
{
	unsigned int i = 0;
	enum mcu_msg_version version = msg->header.version;

	dst[i++] = msg->reserved0;
	dst[i++] = msg->suballoc_dma;
	dst[i++] = msg->suballoc_size;
	dst[i++] = msg->l2_cache[0];
	dst[i++] = msg->l2_cache[1];
	dst[i++] = msg->l2_cache[2];
	if (version >= MCU_MSG_VERSION_2019_2) {
		dst[i++] = -1;
		dst[i++] = 0;
	}

	return i * sizeof(*dst);
}

ssize_t
allegro_encode_config_blob(u32 *dst, struct create_channel_param *param)
{
	enum mcu_msg_version version = param->version;
	unsigned int i = 0;
	u32 val;

	dst[i++] = param->width;
	dst[i++] = param->height;
	dst[i++] = param->framerate;
	dst[i++] = param->clk_ratio;
	dst[i++] = param->max_latency;
	
	dst[i++] = param->num_core;
	dst[i++] = param->ddr_width;

	dst[i++] = param->low_lat;
	dst[i++] = param->parallel_wpp;
	dst[i++] = param->disable_cache;
	dst[i++] = param->frame_buffer_compression;
	dst[i++] = param->use_early_callback;
	
	dst[i++] = param->fb_storage_mode;
	dst[i++] = (version < MCU_MSG_VERSION_2019_2) ? 0 : 1;
	dst[i++] = param->max_slices;
	dst[i++] = param->dec_unit;
	dst[i++] = param->buffer_output_mode;

	return i * sizeof(*dst);
}

static ssize_t
allegro_dec_init(struct mcu_msg_init_response *msg, u32 *src)
{
	unsigned int i = 0;

	msg->reserved0 = src[i++];

	return i * sizeof(*src);
}

static ssize_t
allegro_enc_create_channel(u32 *dst, struct mcu_msg_create_channel *msg)
{
	enum mcu_msg_version version = msg->header.version;
	unsigned int i = 0;

	dst[i++] = msg->user_id;

	if (version >= MCU_MSG_VERSION_2019_2) {
		dst[i++] = msg->blob_mcu_addr;
	} else {
		memcpy(&dst[i], msg->blob, msg->blob_size);
		i += msg->blob_size / sizeof(*dst);
	}

	return i * sizeof(*dst);
}

static ssize_t
allegro_enc_destroy_channel(u32 *dst, struct mcu_msg_destroy_channel *msg)
{
	unsigned int i = 0;

	dst[i++] = msg->channel_id;

	return i * sizeof(*dst);
}

/**
 * allegro_encode_mail() - Encode allegro messages to firmware format
 * @dst: Pointer to the memory that will be filled with data
 * @msg: The allegro message that will be encoded
 */
ssize_t allegro_encode_mail(u32 *dst, void *msg)
{
	const struct mcu_msg_header *header = msg;
	ssize_t size;

	if (!msg || !dst)
		return -EINVAL;

	switch (header->type) {
	case MCU_MSG_TYPE_INIT:
		size = allegro_enc_init(&dst[1], msg);
		break;
	case MCU_MSG_TYPE_CREATE_CHANNEL:
		size = allegro_enc_create_channel(&dst[1], msg);
		break;
	case MCU_MSG_TYPE_DESTROY_CHANNEL:
		size = allegro_enc_destroy_channel(&dst[1], msg);
		break;
	default:
		return -EINVAL;
	}

	/*
	 * The encoded messages might have different length depending on
	 * the firmware version or certain fields. Therefore, we have to
	 * set the body length after encoding the message.
	 */
	dst[0] = FIELD_PREP(GENMASK(31, 16), header->type) |
		 FIELD_PREP(GENMASK(15, 0), size);

	return size + sizeof(*dst);
}

/**
 * allegro_decode_mail() - Parse allegro messages from the firmware.
 * @msg: The mcu_msg_response that will be filled with parsed values.
 * @src: Pointer to the memory that will be parsed
 *
 * The message format in the mailbox depends on the firmware. Parse the
 * different formats into a uniform message format that can be used without
 * taking care of the firmware version.
 */
int allegro_decode_mail(void *msg, u32 *src)
{
	struct mcu_msg_header *header;

	if (!src || !msg)
		return -EINVAL;

	header = msg;
	header->type = FIELD_GET(GENMASK(31, 16), src[0]);

	src++;
	switch (header->type) {
	case MCU_MSG_TYPE_INIT:
		allegro_dec_init(msg, src);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}
