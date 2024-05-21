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
	MCU_MSG_TYPE_ENCODE_FRAME = 0x0007,
	MCU_MSG_TYPE_DECODE_FRAME = 0x0008,

	MCU_MSG_TYPE_SEARCH_START_CODE = 0x0009,
	MCU_MSG_TYPE_PUT_STREAM_BUFFER = 0x0012,
	MCU_MSG_TYPE_PUSH_BUFFER_INTERMEDIATE = 0x000e,
	MCU_MSG_TYPE_PUSH_BUFFER_REFERENCE = 0x000f,

	MCU_MSG_TYPE_DECODE_SLICE = 0x0017,
};

enum mcu_msg_version {
	MCU_MSG_VERSION_2018_2,
	MCU_MSG_VERSION_2019_2,
	MCU_MSG_VERSION_2021_1,
	MCU_MSG_VERSION_2022_2,
};

enum mcu_msg_devtype {
	MCU_MSG_DEVTYPE_ENCODER,
	MCU_MSG_DEVTYPE_DECODER,
};

const char *msg_type_name(enum mcu_msg_type type);

struct mcu_msg_header {
	enum mcu_msg_type type;
	enum mcu_msg_version version;
	enum mcu_msg_devtype devtype;
};

struct mcu_msg_init_request {
	struct mcu_msg_header header;
	u32 reserved0;		/* maybe a unused channel id */
	u32 suballoc_dma;
	u32 suballoc_size;
#if 0 // DW_6-1 - disabled - Encoder special patch
// https://github.com/Xilinx/linux-xlnx/commit/98f1cbf65bf275cce2b9985a8d89cd4fc287a9cc
	s32 encoder_buffer_size;
	s32 encoder_buffer_color_depth;
	s32 num_cores;
	s32 clk_rate;
#else
// old behavior
	// pull from 2022 and 5.15 - DW check
	// added here - https://github.com/Xilinx/linux-xlnx/commit/ecd07f4b9d2173694be9214a3ab07f9efb5ba206
	s32 l2_cache[3]; 
#endif // DW
};

struct mcu_msg_init_response {
	struct mcu_msg_header header;
	u32 reserved0;
};

struct create_encode_channel_param {
	enum mcu_msg_version version;
	u32 layer_id;
	u16 width;
	u16 height;
	u32 videomode;
	u32 format;
	u32 colorspace;
	u32 src_mode;
	u32 src_bit_depth;
	u8 profile;
	u16 constraint_set_flags;
	u32 codec;
	u16 level;
	u16 tier;
	u32 log2_max_poc;
	u32 log2_max_frame_num;
	u32 temporal_mvp_enable;
	u32 enable_reordering;
	u32 dbf_ovr_en;
	u32 override_lf;
	u32 num_ref_idx_l0;
	u32 num_ref_idx_l1;
	u32 custom_lda;
	u32 rdo_cost_mode;
	u32 lf;
	u32 lf_x_tile;
	u32 lf_x_slice;
	s8 beta_offset;
	s8 tc_offset;
	u16 reserved10;
	u32 unknown11;
	u32 unknown12;
	u16 num_slices;
#if 0 // DW_6-1 - disabled - Encoder special patch
// https://github.com/Xilinx/linux-xlnx/commit/98f1cbf65bf275cce2b9985a8d89cd4fc287a9cc
	u32 encoder_buffer_offset;
	u32 encoder_buffer_enabled;
#else
// old behavior
	u16 prefetch_auto;
	u32 prefetch_mem_offset;
	u32 prefetch_mem_size;
#endif
	u16 clip_hrz_range;
	u16 clip_vrt_range;
	u16 me_range[4];
	u8 max_cu_size;
	u8 min_cu_size;
	u8 max_tu_size;
	u8 min_tu_size;
	u8 max_transfo_depth_inter;
	u8 max_transfo_depth_intra;
	u16 reserved20;
	u32 entropy_mode;
	u32 wp_mode;

	/* rate control param */
	u32 rate_control_mode;
	u32 initial_rem_delay;
	u32 cpb_size;
	u16 framerate;
	u16 clk_ratio;
	u32 target_bitrate;
	u32 max_bitrate;
	u16 initial_qp;
	u16 min_qp;
	u16 max_qp;
	s16 ip_delta;
	s16 pb_delta;
	u16 golden_ref;
	u16 golden_delta;
	u16 golden_ref_frequency;
	u32 rate_control_option;
	u32 num_pixel;
	u16 max_psnr;
	u16 max_pixel_value;
	u32 maxpicturesize[3];

	/* gop param */
	u32 gop_ctrl_mode;
	u32 freq_idr;
	u32 freq_lt;
	u32 gdr_mode;
	u16 gop_length;
	u8 num_b;
	u8 freq_golden_ref;
	u32 enable_lt;
	u32 tmpdqp;

	u32 subframe_latency;
	u32 lda_control_mode;
	u32 unknown41;

	u32 lda_factors[6];

	u32 max_num_merge_cand;
};

struct create_decode_channel_param {
	enum mcu_msg_version version;
	s32 width;
	s32 height;
	u8 log2_max_cu_size;
	u32 framerate;
	u32 clk_ratio;
	u32 max_latency;
	u8 num_core;

	u8 ddr_width;
#define AL_DEC_DDR_WIDTH_16             0
#define AL_DEC_DDR_WIDTH_32             1
#define AL_DEC_DDR_WIDTH_64             2

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
#define AL_DEC_FB_RASTER              0
#define AL_DEC_FB_TILE_32x4           2
#define AL_DEC_FB_TILE_64x4           3

	u32 codec;
#define AL_DEC_CODEC_AVC              0
#define AL_DEC_CODEC_HEVC             1
#define AL_DEC_CODEC_AV1              2
#define AL_DEC_CODEC_VP9              3
#define AL_DEC_CODEC_JPEG             4

	s32 max_slices;

	u32 dec_unit;
#define AL_DEC_UNIT_FRAME             0
#define AL_DEC_UNIT_SLICE             1

	u32 buffer_output_mode;
#define AL_DEC_OUTPUT_INTERNAL        0
};

struct mcu_msg_create_channel {
	struct mcu_msg_header header;
	u32 user_id;
	u32 *blob;
	size_t blob_size;
	u32 blob_mcu_addr;
	u32 ep1_addr;
};

struct mcu_msg_create_channel_response {
	struct mcu_msg_header header;
	u32 channel_id;
	u32 user_id;
	u32 options;
	u32 num_core;
	u32 num_ref_idx_l0;
	u32 num_ref_idx_l1;
	u32 int_buffers_count;
	u32 int_buffers_size;
	u32 rec_buffers_count;
	u32 rec_buffers_size;
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

struct mcu_msg_push_buffers_internal_buffer {
	u32 dma_addr;
	u32 mcu_addr;
	u32 size;
};

struct mcu_msg_push_buffers_internal {
	struct mcu_msg_header header;
	u32 channel_id;
	size_t num_buffers;
	struct mcu_msg_push_buffers_internal_buffer buffer[];
};

struct mcu_msg_put_stream_buffer {
	struct mcu_msg_header header;
	u32 channel_id;
	u32 dma_addr;
	u32 mcu_addr;
	u32 size;
	u32 offset;
	u64 dst_handle;
};

struct mcu_msg_encode_frame {
	struct mcu_msg_header header;
	u32 channel_id;
	u32 reserved;

	u32 encoding_options;
#define AL_OPT_USE_QP_TABLE		BIT(0)
#define AL_OPT_FORCE_LOAD		BIT(1)
#define AL_OPT_USE_L2			BIT(2)
#define AL_OPT_DISABLE_INTRA		BIT(3)
#define AL_OPT_DEPENDENT_SLICES		BIT(4)

	s16 pps_qp;
	u16 padding;
	u64 user_param;
	u64 src_handle;

	u32 request_options;
#define AL_OPT_SCENE_CHANGE		BIT(0)
#define AL_OPT_RESTART_GOP		BIT(1)
#define AL_OPT_USE_LONG_TERM		BIT(2)
#define AL_OPT_UPDATE_PARAMS		BIT(3)

	/* u32 scene_change_delay (optional) */
	/* rate control param (optional) */
	/* gop param (optional) */
	/* dynamic resolution params (optional) */
	u32 src_y;
	u32 src_uv;
	u32 is_10_bit;
	u32 stride;
	u32 format;
	u32 ep2;
	u64 ep2_v;
};

struct mcu_msg_encode_frame_response {
	struct mcu_msg_header header;
	u32 channel_id;
	u64 dst_handle;		/* see mcu_msg_put_stream_buffer */
	u64 user_param;		/* see mcu_msg_encode_frame */
	u64 src_handle;		/* see mcu_msg_encode_frame */
	u16 skip;
	u16 is_ref;
	u32 initial_removal_delay;
	u32 dpb_output_delay;
	u32 size;
	u32 frame_tag_size;
	s32 stuffing;
	s32 filler;
	u16 num_column;
	u16 num_row;
	u16 qp;
	u8 num_ref_idx_l0;
	u8 num_ref_idx_l1;
	u32 partition_table_offset;
	s32 partition_table_size;
	u32 sum_complex;
	s32 tile_width[4];
	s32 tile_height[22];
	u32 error_code;

	u32 slice_type;
#define AL_ENC_SLICE_TYPE_B             0
#define AL_ENC_SLICE_TYPE_P             1
#define AL_ENC_SLICE_TYPE_I             2

	u32 pic_struct;
	u8 is_idr;
	u8 is_first_slice;
	u8 is_last_slice;
	u8 reserved;
	u16 pps_qp;
	u16 reserved1;
	u32 reserved2;
	u32 reserved3;
	u32 reserved4;
	u32 reserved5;
	u32 reserved6;
};

struct mcu_msg_search_sc {
	struct mcu_msg_header header;

	u32 channel_id;

	u32 codec;
	u8 stop_param;
	u8 stop_cond;
	u16 max_size;

	u32 addr_stream;
	u32 stream_size;
	u32 offset;
	u32 avail_size;
	u32 addr_output;
};

struct mcu_msg_search_sc_response {
	struct mcu_msg_header header;
	u32 channel_id;
	u16 num_sc;
	u16 reserved;
	u32 num_bytes;
};

struct decode_slice_param {
	u8 max_merged_cand;
	u8 cabac_init_idc;
	u8 coloc_from_l0;
	u8 mvd_l1_zero_flag;

	u16 slice_id;
	u8 num_ref_idx_l1_minus1;
	u8 num_ref_idx_l0_minus1;

	u8 weigthed_pred;
	u8 weigthed_bi_pred;

	bool valid_conceal;

	u8 slice_hdrlen;
	u8 tile_ngb_a;
	u8 tile_ngb_b;
	u8 tile_ngb_c;

	u8 tile_ngb_d;
	u8 tile_ngb_e;
	u16 num_entry_point;

	u8 pic_id_l0[16];

	u8 pic_id_l1[16];

	u8 coloc_pic_id;
	u8 conceal_pic_id;
	u8 cb_qp_offs;
	u8 cr_qp_offs;

	u8 slice_qp;
	u8 tc_offs_div2;
	u8 beta_offs_div2;

	u16 tile_width;
	u16 tile_height;

	u16 first_tile_lcu;
	u16 first_lcu_tile_id;

	u16 lcu_tile_width;
	u16 lcu_tile_height;

	u32 first_lcu;
	u32 num_lcu;
	u32 next_slice_segment;
	u32 first_lcu_slice_segment;
	u32 first_lcu_slice;

	union {
		bool direct_spatial;
		bool temporal_mvp;
	};

	bool last_slice;
	bool dependent_slice;
	bool sao_filter_chroma;
	bool sao_filter_luma;
	bool disable_loop_filter;
	bool x_slice_loop_filter;
	bool cu_chroma_qp_offs;
	bool next_is_dependent;
	bool tile;

	u32 slice_type;
#define AL_DEC_SLICE_SI      4  /*!< AVC SI Slice */
#define AL_DEC_SLICE_SP      3  /*!< AVC SP Slice */
#define AL_DEC_SLICE_GOLDEN  3  /*!< Golden Slice */
#define AL_DEC_SLICE_I       2  /*!< I Slice (can contain I blocks) */
#define AL_DEC_SLICE_P       1  /*!< P Slice (can contain I and P blocks) */
#define AL_DEC_SLICE_B       0  /*!< B Slice (can contain I, P and B blocks) */
#define AL_DEC_SLICE_CONCEAL 6  /*!< Conceal Slice (slice was concealed) */
#define AL_DEC_SLICE_SKIP    7  /*!< Skip Slice */

	u32 str_avail_size;
	u32 comp_offs;
	u32 str_offs;
	u32 entry_point_offs[529];
	u32 parsing_id;
};

struct mcu_msg_decode_frame {
	struct mcu_msg_header header;
	u32 channel_id;

	u32 codec;

	u8 frm_buf_id;
	u8 mv_buf_id;
	u8 max_transfo_depth_intra;
	u8 max_transfo_depth_inter;

	u8 log2_min_tu_size;
	u8 log2_max_tu_size;
	u8 log2_max_tu_skip_size;
	s8 log2_min_pcm_size;

	s8 log2_max_pcm_size;
	s8 log2_min_cu_size;
	u8 log2_max_cu_size;
	u8 pcm_bit_depth_y;

	u8 pcm_bit_depth_c;
	u8 bit_depth_luma;
	u8 bit_depth_chroma;
	u8 chroma_qp_offs_depth;

	u8 qp_off_lst_size;
	u8 parallel_merge;
	u8 coloc_pic_id;
	s8 pic_cb_qp_offs;

	s8 pic_cr_qp_offs;
	s8 cb_qp_off_lst[6];
	s8 cr_qp_off_lst[6];
	s8 delta_qp_cu_depth;
	u16 pic_width;

	u16 pic_height;
	u16 lcu_pic_width;

	u16 lcu_pic_height;
#define AL_DEC_MAX_COLUMNS_TILE				20
#define AL_DEC_MAX_ROWS_TILE				22
	u16 column_width[AL_DEC_MAX_COLUMNS_TILE];
	u16 row_height[AL_DEC_MAX_ROWS_TILE];
	u16 num_tile_columns;

	u16 num_tile_rows;
	u16 padding1;

	s32 current_poc;

	s32 pic_struct;
#define AL_DEC_PS_FRM 						0
#define AL_DEC_PS_TOP_FLD 					1
#define AL_DEC_PS_BOT_FLD 					2
#define AL_DEC_PS_TOP_BOT 					3
#define AL_DEC_PS_BOT_TOP 					4
#define AL_DEC_PS_TOP_BOT_TOP 				5
#define AL_DEC_PS_BOT_TOP_BOT 				6
#define AL_DEC_PS_FRM_x2 					7
#define AL_DEC_PS_FRM_x3 					8
#define AL_DEC_PS_TOP_FLD_WITH_PREV_BOT 	9
#define AL_DEC_PS_BOT_FLD_WITH_PREV_TOP 	10
#define AL_DEC_PS_TOP_FLD_WITH_NEXT_BOT 	11
#define AL_DEC_PS_BOT_FLD_WITH_NEXT_TOP 	12

	u32 option_flags;
#define AL_DEC_OPT_ENABLE_SCL_LIST			BIT(0)
#define AL_DEC_OPT_LOAD_SCL_LIST			BIT(1)
#define AL_DEC_OPT_INTRA_PCM				BIT(5)
#define AL_DEC_OPT_LOSSLESS					BIT(7)
#define AL_DEC_OPT_INTRA_ONLY				BIT(8)
#define AL_DEC_OPT_DIRECT8x8INFER			BIT(9)
#define AL_DEC_OPT_CONSTRAINED_INTRA_PRED	BIT(19)
#define AL_DEC_OPT_TILE						BIT(21)

	u8 num_ladf_intervals;
	s8 ladf_lowest_interval_qp_offs;

	s8 ladf_qp_offs[4];

	u16 ladf_delta_threshold[4];

	u8 qp_prime_ts_min;
	u8 padding2;

	s32 chroma_mode;
#define AL_DEC_CHROMA_MONO 					0
#define AL_DEC_CHROMA_4_0_0 				AL_DEC_CHROMA_MONO
#define AL_DEC_CHROMA_4_2_0 				1
#define AL_DEC_CHROMA_4_2_2 				2
#define AL_DEC_CHROMA_4_4_4 				3

	s32 entropy_mode;
#define AL_DEC_MODE_CAVLC					0
#define AL_DEC_MODE_CABAC					1

	s32 frame_num;
	//u64 user_param __attribute__((aligned(8)));
	u32 user_param[2];

	u8 log2_sao_offs_scale_luma;
	u8 log2_sao_offs_scale_chroma;
	u16 padding3;

	u32 padding4;

	/* compressed MVDs + header + residuals pool buffer */
	u32 addr_comp_data;
	/* Compression map : LCU size + LCU offset pool buffer */
	u32 addr_comp_map;
	/* Reference adresses for the board pool buffer */
	u32 addr_list_ref;
	u32 addr_stream;
	u32 stream_size;
	u32 addr_rec_y;
	u32 addr_rec_c1;
	u32 addr_rec_fbc_map_y;
	u32 addr_rec_fbc_map_c1;
	u32 pitch;
	u32 addr_scl;
	u32 addr_poc;
	u32 addr_mv;
	/* Weighted Pred Tables pool buffer */
	u32 addr_wp;

	/* slice param blob */
	u32 blob_mcu_addr;
	//u32 *blob;
	size_t blob_size;
};

struct mcu_msg_decode_frame_response {
	struct mcu_msg_header header;
	u32 channel_id;

	u32 response_type;
#define AL_DEC_RESPONSE_END_PARSING		1
#define AL_DEC_RESPONSE_END_DECODING	2

	union {
		struct {
			u32 frame_id;
			u32 parsing_id;
		} p;

		struct {
			u8 frm_buf_id;
			u8 mv_buf_id;
			u16 padding;
			u32 num_lcu;
			u32 num_bytes;
			u32 num_bins;
			u32 crc;

			u8 pic_state;
#define AL_DEC_PIC_STATE_CONCEAL		BIT(0)
#define AL_DEC_PIC_STATE_HANGED			BIT(1)
#define AL_DEC_PIC_STATE_NOT_FINISHED	BIT(2)
#define AL_DEC_PIC_STATE_CMD_INVALID	BIT(3)
		} d;
	};
};

union mcu_msg_response {
	struct mcu_msg_header header;
	struct mcu_msg_init_response init;
	struct mcu_msg_create_channel_response create_channel;
	struct mcu_msg_destroy_channel_response destroy_channel;
	struct mcu_msg_encode_frame_response encode_frame;
	struct mcu_msg_decode_frame_response decode_frame;
	struct mcu_msg_search_sc_response search_sc;
};

ssize_t allegro_pack_encoder_config_blob(u32 *dst,
					struct create_encode_channel_param *param);
ssize_t allegro_unpack_encoder_config_blob(
				   struct create_encode_channel_param *param,
				   struct mcu_msg_create_channel_response *msg,
				   u32 *src);

ssize_t allegro_pack_decoder_config_blob(u32 *dst,
					struct create_decode_channel_param *param);

int allegro_decode_mail(void *msg, u32 *src, enum mcu_msg_devtype devtype);
ssize_t allegro_encode_mail(u32 *dst, void *msg);

#endif
