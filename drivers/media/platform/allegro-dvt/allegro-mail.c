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
	case MCU_MSG_TYPE_CREATE_CHANNEL:
		return "CREATE_CHANNEL";
	case MCU_MSG_TYPE_DESTROY_CHANNEL:
		return "DESTROY_CHANNEL";
	case MCU_MSG_TYPE_ENCODE_FRAME:
		return "ENCODE_FRAME";
	case MCU_MSG_TYPE_DECODE_FRAME:
		return "DECODE_FRAME";
	case MCU_MSG_TYPE_PUT_STREAM_BUFFER:
		return "PUT_STREAM_BUFFER";
	case MCU_MSG_TYPE_PUSH_BUFFER_INTERMEDIATE:
		return "PUSH_BUFFER_INTERMEDIATE";
	case MCU_MSG_TYPE_PUSH_BUFFER_REFERENCE:
		return "PUSH_BUFFER_REFERENCE";
	case MCU_MSG_TYPE_DECODE_SLICE:
		return "DECODE_SLICE";
	default:
		snprintf(buf, sizeof(buf), "(0x%04x)", type);
		return buf;
	}
}
EXPORT_SYMBOL(msg_type_name);

static ssize_t
allegro_enc_init(u32 *dst, struct mcu_msg_init_request *msg)
{
	unsigned int i = 0;
	enum mcu_msg_version version = msg->header.version;

	dst[i++] = msg->reserved0;
	dst[i++] = msg->suballoc_dma;
	dst[i++] = msg->suballoc_size;
#if 0 // DW_6-1 - disabled - Encoder change	
	// 6.1 structure update
	// https://github.com/Xilinx/linux-xlnx/commit/98f1cbf65bf275cce2b9985a8d89cd4fc287a9cc
	dst[i++] = msg->encoder_buffer_size;
	dst[i++] = msg->encoder_buffer_color_depth;
	dst[i++] = msg->num_cores;
	if (version >= MCU_MSG_VERSION_2019_2) {
		dst[i++] = msg->clk_rate;
		dst[i++] = 0;
	}
#else
	// old 5.15 behavior
	dst[i++] = msg->l2_cache[0];
	dst[i++] = msg->l2_cache[1];
	dst[i++] = msg->l2_cache[2];
	if (version >= MCU_MSG_VERSION_2019_2) {
		dst[i++] = -1;
		dst[i++] = 0;
	}
#endif // DW_PORT_DEC

	return i * sizeof(*dst);
}

static inline u32 settings_get_mcu_codec(
			enum mcu_msg_version version, u32 pixelformat)
{
#if 0
	// DW - this part of code is not presented in 2022 dec
	enum mcu_msg_version version = param->version;
	u32 pixelformat = param->codec;
#endif

	if (version < MCU_MSG_VERSION_2019_2) {
		switch (pixelformat) {
		case V4L2_PIX_FMT_HEVC:
			return 2;
		case V4L2_PIX_FMT_H264:
		default:
			return 1;
		}
	} else {
		switch (pixelformat) {
		case V4L2_PIX_FMT_HEVC:
			return 1;
		case V4L2_PIX_FMT_H264:
		default:
			return 0;
		}
	}
}

ssize_t allegro_pack_encoder_config_blob(
			u32 *dst, struct create_encode_channel_param *param)
{
	enum mcu_msg_version version = param->version;
	unsigned int i = 0;
	unsigned int j = 0;
	u32 val;
	unsigned int codec = settings_get_mcu_codec(version, param->codec);

	if (version >= MCU_MSG_VERSION_2019_2)
		dst[i++] = param->layer_id;
	dst[i++] = FIELD_PREP(GENMASK(31, 16), param->height) |
		   FIELD_PREP(GENMASK(15, 0), param->width);
	if (version >= MCU_MSG_VERSION_2019_2)
		dst[i++] = param->videomode;
	dst[i++] = param->format;
	if (version < MCU_MSG_VERSION_2019_2)
		dst[i++] = param->colorspace;
	dst[i++] = param->src_mode;
	if (version >= MCU_MSG_VERSION_2019_2)
		dst[i++] = param->src_bit_depth;
	dst[i++] = FIELD_PREP(GENMASK(31, 24), codec) |
		   FIELD_PREP(GENMASK(23, 8), param->constraint_set_flags) |
		   FIELD_PREP(GENMASK(7, 0), param->profile);
	dst[i++] = FIELD_PREP(GENMASK(31, 16), param->tier) |
		   FIELD_PREP(GENMASK(15, 0), param->level);

	val = 0;
	val |= param->temporal_mvp_enable ? BIT(20) : 0;
	val |= FIELD_PREP(GENMASK(7, 4), param->log2_max_frame_num);
	if (version >= MCU_MSG_VERSION_2019_2)
		val |= FIELD_PREP(GENMASK(3, 0), param->log2_max_poc - 1);
	else
		val |= FIELD_PREP(GENMASK(3, 0), param->log2_max_poc);
	dst[i++] = val;

	val = 0;
	val |= param->enable_reordering ? BIT(0) : 0;
	val |= param->dbf_ovr_en ? BIT(2) : 0;
	val |= param->override_lf ? BIT(12) : 0;
	dst[i++] = val;

	if (version >= MCU_MSG_VERSION_2019_2) {
		val = 0;
		val |= param->custom_lda ? BIT(2) : 0;
		val |= param->rdo_cost_mode ? BIT(20) : 0;
		dst[i++] = val;

		val = 0;
		val |= param->lf ? BIT(2) : 0;
		val |= param->lf_x_tile ? BIT(3) : 0;
		val |= param->lf_x_slice ? BIT(4) : 0;
		dst[i++] = val;
	} else {
		val = 0;
		dst[i++] = val;
	}

	dst[i++] = FIELD_PREP(GENMASK(15, 8), param->beta_offset) |
		   FIELD_PREP(GENMASK(7, 0), param->tc_offset);
	dst[i++] = param->unknown11;
	dst[i++] = param->unknown12;
#if 0 // DW_6-1 - disabled - Encoder change
	// 6.1 structure update
	// https://github.com/Xilinx/linux-xlnx/commit/98f1cbf65bf275cce2b9985a8d89cd4fc287a9cc
	dst[i++] = param->num_slices;
	dst[i++] = param->encoder_buffer_offset;
	dst[i++] = param->encoder_buffer_enabled;
#else
	// previous 5.15 behavior
	if (version >= MCU_MSG_VERSION_2019_2)
		dst[i++] = param->num_slices;
	else
		dst[i++] = FIELD_PREP(GENMASK(31, 16), param->prefetch_auto) |
			   FIELD_PREP(GENMASK(15, 0), param->num_slices);
	dst[i++] = param->prefetch_mem_offset;
	dst[i++] = param->prefetch_mem_size;
#endif // DW_PORT_DEC

	dst[i++] = FIELD_PREP(GENMASK(31, 16), param->clip_vrt_range) |
		   FIELD_PREP(GENMASK(15, 0), param->clip_hrz_range);
	dst[i++] = FIELD_PREP(GENMASK(31, 16), param->me_range[1]) |
		   FIELD_PREP(GENMASK(15, 0), param->me_range[0]);
	dst[i++] = FIELD_PREP(GENMASK(31, 16), param->me_range[3]) |
		   FIELD_PREP(GENMASK(15, 0), param->me_range[2]);
	dst[i++] = FIELD_PREP(GENMASK(31, 24), param->min_tu_size) |
		   FIELD_PREP(GENMASK(23, 16), param->max_tu_size) |
		   FIELD_PREP(GENMASK(15, 8), param->min_cu_size) |
		   FIELD_PREP(GENMASK(8, 0), param->max_cu_size);
	dst[i++] = FIELD_PREP(GENMASK(15, 8), param->max_transfo_depth_intra) |
		   FIELD_PREP(GENMASK(7, 0), param->max_transfo_depth_inter);
	dst[i++] = param->entropy_mode;
	dst[i++] = param->wp_mode;

	dst[i++] = param->rate_control_mode;
	dst[i++] = param->initial_rem_delay;
	dst[i++] = param->cpb_size;
	dst[i++] = FIELD_PREP(GENMASK(31, 16), param->clk_ratio) |
		   FIELD_PREP(GENMASK(15, 0), param->framerate);
	dst[i++] = param->target_bitrate;
	dst[i++] = param->max_bitrate;
	dst[i++] = FIELD_PREP(GENMASK(31, 16), param->min_qp) |
		   FIELD_PREP(GENMASK(15, 0), param->initial_qp);
	dst[i++] = FIELD_PREP(GENMASK(31, 16), param->ip_delta) |
		   FIELD_PREP(GENMASK(15, 0), param->max_qp);
	dst[i++] = FIELD_PREP(GENMASK(31, 16), param->golden_ref) |
		   FIELD_PREP(GENMASK(15, 0), param->pb_delta);
	dst[i++] = FIELD_PREP(GENMASK(31, 16), param->golden_ref_frequency) |
		   FIELD_PREP(GENMASK(15, 0), param->golden_delta);
	if (version >= MCU_MSG_VERSION_2019_2)
		dst[i++] = param->rate_control_option;
	else
		dst[i++] = 0;

	if (version >= MCU_MSG_VERSION_2019_2) {
		dst[i++] = param->num_pixel;
		dst[i++] = FIELD_PREP(GENMASK(31, 16), param->max_pixel_value) |
			FIELD_PREP(GENMASK(15, 0), param->max_psnr);
		for (j = 0; j < 3; j++)
			dst[i++] = param->maxpicturesize[j];
	}

	if (version >= MCU_MSG_VERSION_2019_2)
		dst[i++] = param->gop_ctrl_mode;
	else
		dst[i++] = 0;

	if (version >= MCU_MSG_VERSION_2019_2)
		dst[i++] = FIELD_PREP(GENMASK(31, 24), param->freq_golden_ref) |
			   FIELD_PREP(GENMASK(23, 16), param->num_b) |
			   FIELD_PREP(GENMASK(15, 0), param->gop_length);
	dst[i++] = param->freq_idr;
	if (version >= MCU_MSG_VERSION_2019_2)
		dst[i++] = param->enable_lt;
	dst[i++] = param->freq_lt;
	dst[i++] = param->gdr_mode;
	if (version < MCU_MSG_VERSION_2019_2)
		dst[i++] = FIELD_PREP(GENMASK(31, 24), param->freq_golden_ref) |
			   FIELD_PREP(GENMASK(23, 16), param->num_b) |
			   FIELD_PREP(GENMASK(15, 0), param->gop_length);

	if (version >= MCU_MSG_VERSION_2019_2)
		dst[i++] = param->tmpdqp;

	dst[i++] = param->subframe_latency;
	dst[i++] = param->lda_control_mode;
	if (version < MCU_MSG_VERSION_2019_2)
		dst[i++] = param->unknown41;

	if (version >= MCU_MSG_VERSION_2019_2) {
		for (j = 0; j < 6; j++)
			dst[i++] = param->lda_factors[j];
		dst[i++] = param->max_num_merge_cand;
	}

	return i * sizeof(*dst);
}

ssize_t allegro_pack_decoder_config_blob(
	u32 *dst, struct create_decode_channel_param *param)
{
	enum mcu_msg_version version = param->version;
	unsigned int codec = settings_get_mcu_codec(version, param->codec);
	unsigned int i = 0;

	dst[i++] = param->width;
	dst[i++] = param->height;
	if (version >= MCU_MSG_VERSION_2021_1)
		dst[i++] = param->log2_max_cu_size;
	dst[i++] = param->framerate;
	dst[i++] = param->clk_ratio;
	dst[i++] = param->max_latency;
	dst[i++] = FIELD_PREP(GENMASK(31, 24), param->parallel_wpp) |
		   FIELD_PREP(GENMASK(23, 16), param->low_lat) |
		   FIELD_PREP(GENMASK(15, 8), param->ddr_width) |
		   FIELD_PREP(GENMASK(7, 0), param->num_core);
	dst[i++] = FIELD_PREP(GENMASK(23, 16), param->use_early_callback) |
		   FIELD_PREP(GENMASK(15, 8),param->frame_buffer_compression) |
		   FIELD_PREP(GENMASK(7, 0), param->disable_cache);
	dst[i++] = param->fb_storage_mode;
	dst[i++] = codec;
	dst[i++] = param->max_slices;
	dst[i++] = param->dec_unit;
	dst[i++] = param->buffer_output_mode;

	return i * sizeof(*dst);
}

static ssize_t
allegro_enc_create_channel(u32 *dst, struct mcu_msg_create_channel *msg)
{
	enum mcu_msg_version version = msg->header.version;
	enum mcu_msg_devtype devtype = msg->header.devtype;
	unsigned int i = 0;

	dst[i++] = msg->user_id;

	if (version >= MCU_MSG_VERSION_2019_2) {
		dst[i++] = msg->blob_mcu_addr;
	} else {
		memcpy(&dst[i], msg->blob, msg->blob_size);
		i += msg->blob_size / sizeof(*dst);
	}

	if (devtype == MCU_MSG_DEVTYPE_ENCODER)
		if (version >= MCU_MSG_VERSION_2019_2)
			dst[i++] = msg->ep1_addr;

	return i * sizeof(*dst);
}

ssize_t allegro_unpack_encoder_config_blob(
				   struct create_encode_channel_param *param,
				   struct mcu_msg_create_channel_response *msg,
				   u32 *src)
{
	enum mcu_msg_version version = msg->header.version;

	if (version >= MCU_MSG_VERSION_2019_2) {
		param->num_ref_idx_l0 = FIELD_GET(GENMASK(7, 4), src[9]);
		param->num_ref_idx_l1 = FIELD_GET(GENMASK(11, 8), src[9]);
	} else {
		param->num_ref_idx_l0 = msg->num_ref_idx_l0;
		param->num_ref_idx_l1 = msg->num_ref_idx_l1;
	}

	return 0;
}

static ssize_t
allegro_enc_destroy_channel(u32 *dst, struct mcu_msg_destroy_channel *msg)
{
	unsigned int i = 0;

	dst[i++] = msg->channel_id;

	return i * sizeof(*dst);
}

static ssize_t
allegro_enc_push_buffers(u32 *dst, struct mcu_msg_push_buffers_internal *msg)
{
	unsigned int i = 0;
	struct mcu_msg_push_buffers_internal_buffer *buffer;
	unsigned int num_buffers = msg->num_buffers;
	unsigned int j;

	dst[i++] = msg->channel_id;

	for (j = 0; j < num_buffers; j++) {
		buffer = &msg->buffer[j];
		dst[i++] = buffer->dma_addr;
		dst[i++] = buffer->mcu_addr;
		dst[i++] = buffer->size;
	}

	return i * sizeof(*dst);
}

static ssize_t
allegro_enc_put_stream_buffer(u32 *dst,
			      struct mcu_msg_put_stream_buffer *msg)
{
	unsigned int i = 0;

	dst[i++] = msg->channel_id;
	dst[i++] = msg->dma_addr;
	dst[i++] = msg->mcu_addr;
	dst[i++] = msg->size;
	dst[i++] = msg->offset;
	dst[i++] = lower_32_bits(msg->dst_handle);
	dst[i++] = upper_32_bits(msg->dst_handle);

	return i * sizeof(*dst);
}

static ssize_t
allegro_enc_encode_frame(u32 *dst, struct mcu_msg_encode_frame *msg)
{
	enum mcu_msg_version version = msg->header.version;
	unsigned int i = 0;

	dst[i++] = msg->channel_id;

	dst[i++] = msg->reserved;
	dst[i++] = msg->encoding_options;
	dst[i++] = FIELD_PREP(GENMASK(31, 16), msg->padding) |
		   FIELD_PREP(GENMASK(15, 0), msg->pps_qp);

	if (version >= MCU_MSG_VERSION_2019_2) {
		dst[i++] = 0;
		dst[i++] = 0;
		dst[i++] = 0;
		dst[i++] = 0;
	}

	dst[i++] = lower_32_bits(msg->user_param);
	dst[i++] = upper_32_bits(msg->user_param);
	dst[i++] = lower_32_bits(msg->src_handle);
	dst[i++] = upper_32_bits(msg->src_handle);
	dst[i++] = msg->request_options;
	dst[i++] = msg->src_y;
	dst[i++] = msg->src_uv;
	if (version >= MCU_MSG_VERSION_2019_2)
		dst[i++] = msg->is_10_bit;
	dst[i++] = msg->stride;
	if (version >= MCU_MSG_VERSION_2019_2)
		dst[i++] = msg->format;
	dst[i++] = msg->ep2;
	dst[i++] = lower_32_bits(msg->ep2_v);
	dst[i++] = upper_32_bits(msg->ep2_v);

	return i * sizeof(*dst);
}

static ssize_t
allegro_enc_decode_frame(u32 *dst, struct mcu_msg_decode_frame *msg)
{
	enum mcu_msg_version version = msg->header.version;
	u32 codec = settings_get_mcu_codec(version, msg->codec);
	unsigned int i = 0;
	unsigned int j;

	dst[i++] = msg->channel_id;
	dst[i++] = codec;
	dst[i++] = FIELD_PREP(GENMASK(31, 24), msg->max_transfo_depth_inter) |
		   FIELD_PREP(GENMASK(23, 16), msg->max_transfo_depth_intra) |
		   FIELD_PREP(GENMASK(15, 8), msg->mv_buf_id) |
		   FIELD_PREP(GENMASK(7, 0), msg->frm_buf_id);

	dst[i++] = FIELD_PREP(GENMASK(31, 24), msg->log2_min_pcm_size) |
		   FIELD_PREP(GENMASK(23, 16), msg->log2_max_tu_skip_size) |
		   FIELD_PREP(GENMASK(15, 8), msg->log2_max_tu_size) |
		   FIELD_PREP(GENMASK(7, 0), msg->log2_min_tu_size);

	dst[i++] = FIELD_PREP(GENMASK(31, 24), msg->pcm_bit_depth_y) |
		   FIELD_PREP(GENMASK(23, 16), msg->log2_max_cu_size) |
		   FIELD_PREP(GENMASK(15, 8), msg->log2_min_cu_size) |
		   FIELD_PREP(GENMASK(7, 0), msg->log2_max_pcm_size);

	dst[i++] = FIELD_PREP(GENMASK(31, 24), msg->chroma_qp_offs_depth) |
		   FIELD_PREP(GENMASK(23, 16), msg->bit_depth_chroma) |
		   FIELD_PREP(GENMASK(15, 8), msg->bit_depth_luma) |
		   FIELD_PREP(GENMASK(7, 0), msg->pcm_bit_depth_c);

	dst[i++] = FIELD_PREP(GENMASK(31, 24), msg->pic_cb_qp_offs) |
		   FIELD_PREP(GENMASK(23, 16), msg->coloc_pic_id) |
		   FIELD_PREP(GENMASK(15, 8), msg->parallel_merge) |
		   FIELD_PREP(GENMASK(7, 0), msg->qp_off_lst_size);

	dst[i++] = FIELD_PREP(GENMASK(31, 24), msg->cb_qp_off_lst[2]) |
		   FIELD_PREP(GENMASK(23, 16), msg->cb_qp_off_lst[1]) |
		   FIELD_PREP(GENMASK(15, 8), msg->cb_qp_off_lst[0]) |
		   FIELD_PREP(GENMASK(7, 0), msg->pic_cr_qp_offs);

	dst[i++] = FIELD_PREP(GENMASK(31, 24), msg->cr_qp_off_lst[0]) |
		   FIELD_PREP(GENMASK(23, 16), msg->cb_qp_off_lst[5]) |
		   FIELD_PREP(GENMASK(15, 8), msg->cb_qp_off_lst[4]) |
		   FIELD_PREP(GENMASK(7, 0), msg->cb_qp_off_lst[3]);

	dst[i++] = FIELD_PREP(GENMASK(31, 24), msg->cr_qp_off_lst[4]) |
		   FIELD_PREP(GENMASK(23, 16), msg->cr_qp_off_lst[3]) |
		   FIELD_PREP(GENMASK(15, 8), msg->cr_qp_off_lst[2]) |
		   FIELD_PREP(GENMASK(7, 0), msg->cr_qp_off_lst[1]);

	dst[i++] = FIELD_PREP(GENMASK(31, 16), msg->pic_width) |
		   FIELD_PREP(GENMASK(15, 8), msg->delta_qp_cu_depth) |
		   FIELD_PREP(GENMASK(7, 0), msg->cr_qp_off_lst[5]);

	dst[i++] = FIELD_PREP(GENMASK(31, 16), msg->lcu_pic_width) |
		   FIELD_PREP(GENMASK(15, 0), msg->pic_height);

	dst[i++] = FIELD_PREP(GENMASK(31, 16), msg->column_width[0]) |
		   FIELD_PREP(GENMASK(15, 0), msg->lcu_pic_height);

	for (j = 1; j < AL_DEC_MAX_COLUMNS_TILE - 1; j += 2)
		dst[i++] = FIELD_PREP(GENMASK(31, 16), msg->column_width[j+1]) |
			   FIELD_PREP(GENMASK(15, 0), msg->column_width[j]);

	dst[i++] = FIELD_PREP(GENMASK(31, 16), msg->row_height[0]) |
		   FIELD_PREP(GENMASK(15, 0), msg->column_width[j]);

	for (j = 1; j < AL_DEC_MAX_ROWS_TILE - 1; j += 2)
		dst[i++] = FIELD_PREP(GENMASK(31, 16), msg->row_height[j+1]) |
			   FIELD_PREP(GENMASK(15, 0), msg->row_height[j]);

	dst[i++] = FIELD_PREP(GENMASK(31, 16), msg->num_tile_columns) |
		   FIELD_PREP(GENMASK(15, 0), msg->row_height[j]);

	dst[i++] = FIELD_PREP(GENMASK(31, 16), msg->padding1) |
		   FIELD_PREP(GENMASK(15, 0), msg->num_tile_rows);

	dst[i++] = msg->current_poc;
	dst[i++] = msg->pic_struct;

	dst[i++] = msg->option_flags;

	dst[i++] = FIELD_PREP(GENMASK(31, 24), msg->ladf_qp_offs[1]) |
		   FIELD_PREP(GENMASK(23, 16), msg->ladf_qp_offs[0]) |
		   FIELD_PREP(GENMASK(15, 8), msg->ladf_lowest_interval_qp_offs) |
		   FIELD_PREP(GENMASK(7, 0), msg->num_ladf_intervals);

	dst[i++] = FIELD_PREP(GENMASK(31, 16), msg->ladf_delta_threshold[0]) |
		   FIELD_PREP(GENMASK(15, 8), msg->ladf_qp_offs[3]) |
		   FIELD_PREP(GENMASK(7, 0), msg->ladf_qp_offs[2]);

	dst[i++] = FIELD_PREP(GENMASK(31, 16), msg->ladf_delta_threshold[2]) |
		   FIELD_PREP(GENMASK(15, 0), msg->ladf_delta_threshold[1]);

	dst[i++] = FIELD_PREP(GENMASK(23, 16), msg->qp_prime_ts_min) |
		   FIELD_PREP(GENMASK(15, 0), msg->ladf_delta_threshold[3]);

	dst[i++] = msg->chroma_mode;
	dst[i++] = msg->entropy_mode;

	dst[i++] = msg->frame_num;

	dst[i++] = msg->user_param[0];//lower_32_bits(msg->user_param);
	dst[i++] = msg->user_param[1];//upper_32_bits(msg->user_param);

	dst[i++] = FIELD_PREP(GENMASK(31, 16), msg->padding3) |
		   FIELD_PREP(GENMASK(15, 8), msg->log2_sao_offs_scale_chroma) |
		   FIELD_PREP(GENMASK(7, 0), msg->log2_sao_offs_scale_luma);

	dst[i++] = 0;//msg->padding4;

	dst[i++] = msg->addr_comp_data;
	dst[i++] = msg->addr_comp_map;
	dst[i++] = msg->addr_list_ref;
	dst[i++] = msg->addr_stream;
	dst[i++] = msg->stream_size;
	dst[i++] = msg->addr_rec_y;
	dst[i++] = msg->addr_rec_c1;
	dst[i++] = msg->addr_rec_fbc_map_y;
	dst[i++] = msg->addr_rec_fbc_map_c1;
	dst[i++] = msg->pitch;
	dst[i++] = msg->addr_scl;
	dst[i++] = msg->addr_poc;
	dst[i++] = msg->addr_mv;
	dst[i++] = msg->addr_wp;
	dst[i++] = msg->blob_mcu_addr;

	return i * sizeof(*dst);
}

static ssize_t
allegro_enc_search_sc(u32 *dst, struct mcu_msg_search_sc *msg)
{
	enum mcu_msg_version version = msg->header.version;
	u32 codec = settings_get_mcu_codec(version, msg->codec);
	unsigned int i = 0;

	dst[i++] = msg->channel_id;

	dst[i++] = codec;
	dst[i++] = FIELD_PREP(GENMASK(31, 16), msg->max_size) |
		   FIELD_PREP(GENMASK(15, 8), msg->stop_cond) |
		   FIELD_PREP(GENMASK(7, 0), msg->stop_param);

	dst[i++] = msg->addr_stream;
	dst[i++] = msg->stream_size;
	dst[i++] = msg->offset;
	dst[i++] = msg->avail_size;
	dst[i++] = msg->addr_output;

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
allegro_dec_create_channel(struct mcu_msg_create_channel_response *msg,
			   u32 *src)
{
	enum mcu_msg_version version = msg->header.version;
	enum mcu_msg_devtype devtype = msg->header.devtype;
	unsigned int i = 0;

	msg->channel_id = src[i++];
	msg->user_id = src[i++];

	if (devtype == MCU_MSG_DEVTYPE_DECODER) {
		msg->error_code = src[i++];
	} else {
		/*
		 * Version >= MCU_MSG_VERSION_2019_2 is handled in
		 * allegro_decode_config_blob().
		 */
		if (version < MCU_MSG_VERSION_2019_2) {
			msg->options = src[i++];
			msg->num_core = src[i++];
			msg->num_ref_idx_l0 = FIELD_GET(GENMASK(7, 4), src[i]);
			msg->num_ref_idx_l1 = FIELD_GET(GENMASK(11, 8), src[i++]);
		}
		msg->int_buffers_count = src[i++];
		msg->int_buffers_size = src[i++];
		msg->rec_buffers_count = src[i++];
		msg->rec_buffers_size = src[i++];
		msg->reserved = src[i++];
		msg->error_code = src[i++];
	}

	return i * sizeof(*src);
}

static ssize_t
allegro_dec_destroy_channel(struct mcu_msg_destroy_channel_response *msg,
			    u32 *src)
{
	unsigned int i = 0;

	msg->channel_id = src[i++];

	return i * sizeof(*src);
}

static ssize_t
allegro_dec_encode_frame(struct mcu_msg_encode_frame_response *msg, u32 *src)
{
	enum mcu_msg_version version = msg->header.version;
	unsigned int i = 0;
	unsigned int j;

	msg->channel_id = src[i++];

	msg->dst_handle = src[i++];
	msg->dst_handle |= (((u64)src[i++]) << 32);
	msg->user_param = src[i++];
	msg->user_param |= (((u64)src[i++]) << 32);
	msg->src_handle = src[i++];
	msg->src_handle |= (((u64)src[i++]) << 32);
	msg->skip = FIELD_GET(GENMASK(31, 16), src[i]);
	msg->is_ref = FIELD_GET(GENMASK(15, 0), src[i++]);
	msg->initial_removal_delay = src[i++];
	msg->dpb_output_delay = src[i++];
	msg->size = src[i++];
	msg->frame_tag_size = src[i++];
	msg->stuffing = src[i++];
	msg->filler = src[i++];
	// special fixes on 6.1 compare with 5.15
	// https://github.com/Xilinx/linux-xlnx/commit/436ee4b515bb9a63f68f9d1917c7df75010c251d
	msg->num_row = FIELD_GET(GENMASK(31, 16), src[i]);
	msg->num_column = FIELD_GET(GENMASK(15, 0), src[i++]);
	// end of fix
	msg->num_ref_idx_l1 = FIELD_GET(GENMASK(31, 24), src[i]);
	msg->num_ref_idx_l0 = FIELD_GET(GENMASK(23, 16), src[i]);
	msg->qp = FIELD_GET(GENMASK(15, 0), src[i++]);
	msg->partition_table_offset = src[i++];
	msg->partition_table_size = src[i++];
	msg->sum_complex = src[i++];
	for (j = 0; j < 4; j++)
		msg->tile_width[j] = src[i++];
	for (j = 0; j < 22; j++)
		msg->tile_height[j] = src[i++];
	msg->error_code = src[i++];
	msg->slice_type = src[i++];
	msg->pic_struct = src[i++];
	msg->reserved = FIELD_GET(GENMASK(31, 24), src[i]);
	msg->is_last_slice = FIELD_GET(GENMASK(23, 16), src[i]);
	msg->is_first_slice = FIELD_GET(GENMASK(15, 8), src[i]);
	msg->is_idr = FIELD_GET(GENMASK(7, 0), src[i++]);

	msg->reserved1 = FIELD_GET(GENMASK(31, 16), src[i]);
	msg->pps_qp = FIELD_GET(GENMASK(15, 0), src[i++]);

	msg->reserved2 = src[i++];
	if (version >= MCU_MSG_VERSION_2019_2) {
		msg->reserved3 = src[i++];
		msg->reserved4 = src[i++];
		msg->reserved5 = src[i++];
		msg->reserved6 = src[i++];
	}

	return i * sizeof(*src);
}

static ssize_t
allegro_dec_decode_frame(struct mcu_msg_decode_frame_response *msg, u32 *src)
{
	enum mcu_msg_version version = msg->header.version;
	unsigned int i = 0;

	msg->channel_id = src[i++];
	msg->response_type = src[i++];

	if (msg->response_type == AL_DEC_RESPONSE_END_PARSING) {
		msg->p.frame_id = src[i++];
		msg->p.parsing_id = src[i++];
	} else if (msg->response_type == AL_DEC_RESPONSE_END_DECODING) {
		msg->d.frm_buf_id = FIELD_GET(GENMASK(7, 0), src[i]);
		msg->d.mv_buf_id = FIELD_GET(GENMASK(15, 8), src[i]);
		msg->d.padding = FIELD_GET(GENMASK(31, 16), src[i++]);
		msg->d.num_lcu = src[i++];
		msg->d.num_bytes = src[i++];
		msg->d.num_bins = src[i++];
		msg->d.crc = src[i++];
		msg->d.pic_state = FIELD_GET(GENMASK(7, 0), src[i++]);
	}

	return i * sizeof(*src);
}

static ssize_t
allegro_dec_search_sc(struct mcu_msg_search_sc_response *msg, u32 *src)
{
	enum mcu_msg_version version = msg->header.version;
	unsigned int i = 0;

	msg->channel_id = src[i++];
	msg->num_sc = FIELD_GET(GENMASK(15, 0), src[i]);
	msg->reserved = FIELD_GET(GENMASK(31, 16), src[i++]);
	msg->num_bytes = src[i++];

	return i * sizeof(*src);
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
	case MCU_MSG_TYPE_ENCODE_FRAME:
		size = allegro_enc_encode_frame(&dst[1], msg);
		break;
	case MCU_MSG_TYPE_DECODE_FRAME:
	case MCU_MSG_TYPE_DECODE_SLICE:
		size = allegro_enc_decode_frame(&dst[1], msg);
		break;
	case MCU_MSG_TYPE_SEARCH_START_CODE:
		size = allegro_enc_search_sc(&dst[1], msg);
		break;
	case MCU_MSG_TYPE_PUT_STREAM_BUFFER:
		size = allegro_enc_put_stream_buffer(&dst[1], msg);
		break;
	case MCU_MSG_TYPE_PUSH_BUFFER_INTERMEDIATE:
	case MCU_MSG_TYPE_PUSH_BUFFER_REFERENCE:
		size = allegro_enc_push_buffers(&dst[1], msg);
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
int allegro_decode_mail(void *msg, u32 *src, enum mcu_msg_devtype devtype)
{
	struct mcu_msg_header *header;

	if (!src || !msg)
		return -EINVAL;

	header = msg;
	header->type = FIELD_GET(GENMASK(31, 16), src[0]);
	header->devtype = devtype;

	src++;

	switch (header->type) {
	case MCU_MSG_TYPE_INIT:
		allegro_dec_init(msg, src);
		break;
	case MCU_MSG_TYPE_CREATE_CHANNEL:
		allegro_dec_create_channel(msg, src);
		break;
	case MCU_MSG_TYPE_DESTROY_CHANNEL:
		allegro_dec_destroy_channel(msg, src);
		break;
	case MCU_MSG_TYPE_ENCODE_FRAME:
		allegro_dec_encode_frame(msg, src);
		break;
	case MCU_MSG_TYPE_DECODE_FRAME:
	case MCU_MSG_TYPE_DECODE_SLICE:
		allegro_dec_decode_frame(msg, src);
		break;
	case MCU_MSG_TYPE_SEARCH_START_CODE:
		allegro_dec_search_sc(msg, src);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}
