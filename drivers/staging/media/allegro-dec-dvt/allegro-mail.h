/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2019 Pengutronix, Michael Tretter <kernel@pengutronix.de>
 *
 * Allegro VCU firmware mailbox mail definitions
 */

#ifndef ALLEGRO_MAIL_H
#define ALLEGRO_MAIL_H

#include <linux/kernel.h>

enum mcu_msg_type {
	MCU_MSG_TYPE_INIT = 0x0000,
	MCU_MSG_TYPE_CREATE_CHANNEL = 0x0005,
	MCU_MSG_TYPE_DESTROY_CHANNEL = 0x0006,
	MCU_MSG_TYPE_DECODE_FRAME = 0x0008,
	MCU_MSG_TYPE_SEARCH_START_CODE = 0x0009,
};

enum mcu_msg_version {
	MCU_MSG_VERSION_2018_2,
	MCU_MSG_VERSION_2019_2,
	MCU_MSG_VERSION_2021_1,
};

const char *msg_type_name(enum mcu_msg_type type);

struct mcu_msg_header {
	enum mcu_msg_type type;
	enum mcu_msg_version version;
};

struct mcu_msg_init_request {
	struct mcu_msg_header header;
	u32 reserved0;		/* maybe a unused channel id */
	u32 suballoc_dma;
	u32 suballoc_size;
	s32 l2_cache[3];
};

struct mcu_msg_init_response {
	struct mcu_msg_header header;
	u32 reserved0;
};

struct create_channel_param {
	enum mcu_msg_version version;

	s32 width;
	s32 height;
	u32 framerate;
	/*!< Frame rate value used if syntax element isn't present */
	u32 clk_ratio;
	/*!< Clock ratio value used if syntax element isn't present */
	u32 max_latency;
	u8 num_core;
	/*!< number of hevc decoder core used for the decoding */
	u8 ddr_width;
	/*!< width of the DDR used by the decoder */
	bool low_lat;
	/*!< use low latency decoding */
	bool parallel_wpp;
	/*!< use wavefront parallelization processing */
	bool disable_cache;
	/*!< disable the decoder cache */
	bool frame_buffer_compression;
	/*!< use internal frame buffer compression */
	bool use_early_callback;
	s32 fb_storage_mode;
	/*!< storage mode for the decoder frame buffers*/
	u32 codec;
	/*!< Specify which codec is used - 0 AVC, 1 HEVC, 2 AV1, 3 VP9, 4 JPEG*/
	s32 max_slices;
	u32 dec_unit;
	/*< decode at the (0) AU level (frame), (1) NALU level (slice) */
	u32 buffer_output_mode;
	/*< (0) Output reconstructed buffers stored as in the encoder */
};

struct mcu_msg_create_channel {
	struct mcu_msg_header header;
	u32 user_id;
	u32 *blob;
	size_t blob_size;
	u32 blob_mcu_addr;
};

struct mcu_msg_create_channel_response {
	struct mcu_msg_header header;
	u32 channel_id;
	u32 user_id;

	u32 reserved;
	u32 error_code;
};

struct mcu_msg_destroy_channel {
	struct mcu_msg_header header;
	u32 channel_id;
};

struct mcu_msg_destroy_channel_response {
	struct mcu_msg_header header;
	u32 channel_id;
};

union mcu_msg_response {
	struct mcu_msg_header header;
	struct mcu_msg_init_response init;
	struct mcu_msg_create_channel_response create_channel;
	struct mcu_msg_destroy_channel_response destroy_channel;
};

ssize_t allegro_encode_config_blob(u32 *dst,
				struct create_channel_param *param);

int allegro_decode_mail(void *msg, u32 *src);
ssize_t allegro_encode_mail(u32 *dst, void *msg);

#endif
