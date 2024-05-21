// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 Pengutronix, Michael Tretter <kernel@pengutronix.de>
 *
 * Allegro DVT video encoder driver
 */

#include <linux/bits.h>
#include <linux/firmware.h>
#include <linux/gcd.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/log2.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-vmalloc.h>

#include <linux/version.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>

#include "allegro.h"
#include "allegro-mail.h"
#include "nal-h264.h"
#include "nal-hevc.h"



/* Encoding options */
#define LOG2_MAX_FRAME_NUM      4
#define LOG2_MAX_PIC_ORDER_CNT      10
#define BETA_OFFSET_DIV_2       -1
#define TC_OFFSET_DIV_2         -1



static inline int
allegro_channel_get_i_frame_qp(
    struct allegro_channel *channel, unsigned int codec)
{
    if (codec == V4L2_PIX_FMT_HEVC)
        return v4l2_ctrl_g_ctrl(channel->mpeg_video_hevc_i_frame_qp);
    else
        return v4l2_ctrl_g_ctrl(channel->mpeg_video_h264_i_frame_qp);
}

static inline int
allegro_channel_get_p_frame_qp(
    struct allegro_channel *channel, unsigned int codec)
{
    if (codec == V4L2_PIX_FMT_HEVC)
        return v4l2_ctrl_g_ctrl(channel->mpeg_video_hevc_p_frame_qp);
    else
        return v4l2_ctrl_g_ctrl(channel->mpeg_video_h264_p_frame_qp);
}

static inline int
allegro_channel_get_b_frame_qp(
    struct allegro_channel *channel, unsigned int codec)
{
    if (codec == V4L2_PIX_FMT_HEVC)
        return v4l2_ctrl_g_ctrl(channel->mpeg_video_hevc_b_frame_qp);
    else
        return v4l2_ctrl_g_ctrl(channel->mpeg_video_h264_b_frame_qp);
}

static inline int
allegro_channel_get_min_qp(
    struct allegro_channel *channel, unsigned int codec)
{
    if (codec == V4L2_PIX_FMT_HEVC)
        return v4l2_ctrl_g_ctrl(channel->mpeg_video_hevc_min_qp);
    else
        return v4l2_ctrl_g_ctrl(channel->mpeg_video_h264_min_qp);
}

static inline int
allegro_channel_get_max_qp(
    struct allegro_channel *channel, unsigned int codec)
{
    if (codec == V4L2_PIX_FMT_HEVC)
        return v4l2_ctrl_g_ctrl(channel->mpeg_video_hevc_max_qp);
    else
        return v4l2_ctrl_g_ctrl(channel->mpeg_video_h264_max_qp);
}

static enum v4l2_mpeg_video_h264_level
select_minimum_h264_level(unsigned int width, unsigned int height)
{
    unsigned int pic_width_in_mb = DIV_ROUND_UP(width, SIZE_MACROBLOCK);
    unsigned int frame_height_in_mb = DIV_ROUND_UP(height, SIZE_MACROBLOCK);
    unsigned int frame_size_in_mb = pic_width_in_mb * frame_height_in_mb;
    enum v4l2_mpeg_video_h264_level level = V4L2_MPEG_VIDEO_H264_LEVEL_4_0;

    /*
     * The level limits are specified in Rec. ITU-T H.264 Annex A.3.1 and
     * also specify limits regarding bit rate and CBP size. Only approximate
     * the levels using the frame size.
     *
     * Level 5.1 allows up to 4k video resolution.
     */
    if (frame_size_in_mb <= 99)
        level = V4L2_MPEG_VIDEO_H264_LEVEL_1_0;
    else if (frame_size_in_mb <= 396)
        level = V4L2_MPEG_VIDEO_H264_LEVEL_1_1;
    else if (frame_size_in_mb <= 792)
        level = V4L2_MPEG_VIDEO_H264_LEVEL_2_1;
    else if (frame_size_in_mb <= 1620)
        level = V4L2_MPEG_VIDEO_H264_LEVEL_2_2;
    else if (frame_size_in_mb <= 3600)
        level = V4L2_MPEG_VIDEO_H264_LEVEL_3_1;
    else if (frame_size_in_mb <= 5120)
        level = V4L2_MPEG_VIDEO_H264_LEVEL_3_2;
    else if (frame_size_in_mb <= 8192)
        level = V4L2_MPEG_VIDEO_H264_LEVEL_4_0;
    else if (frame_size_in_mb <= 8704)
        level = V4L2_MPEG_VIDEO_H264_LEVEL_4_2;
    else if (frame_size_in_mb <= 22080)
        level = V4L2_MPEG_VIDEO_H264_LEVEL_5_0;
    else
        level = V4L2_MPEG_VIDEO_H264_LEVEL_5_1;

    return level;
}

static unsigned int h264_maximum_bitrate(enum v4l2_mpeg_video_h264_level level)
{
    switch (level) {
    case V4L2_MPEG_VIDEO_H264_LEVEL_1_0:
        return 64000;
    case V4L2_MPEG_VIDEO_H264_LEVEL_1B:
        return 128000;
    case V4L2_MPEG_VIDEO_H264_LEVEL_1_1:
        return 192000;
    case V4L2_MPEG_VIDEO_H264_LEVEL_1_2:
        return 384000;
    case V4L2_MPEG_VIDEO_H264_LEVEL_1_3:
        return 768000;
    case V4L2_MPEG_VIDEO_H264_LEVEL_2_0:
        return 2000000;
    case V4L2_MPEG_VIDEO_H264_LEVEL_2_1:
        return 4000000;
    case V4L2_MPEG_VIDEO_H264_LEVEL_2_2:
        return 4000000;
    case V4L2_MPEG_VIDEO_H264_LEVEL_3_0:
        return 10000000;
    case V4L2_MPEG_VIDEO_H264_LEVEL_3_1:
        return 14000000;
    case V4L2_MPEG_VIDEO_H264_LEVEL_3_2:
        return 20000000;
    case V4L2_MPEG_VIDEO_H264_LEVEL_4_0:
        return 20000000;
    case V4L2_MPEG_VIDEO_H264_LEVEL_4_1:
        return 50000000;
    case V4L2_MPEG_VIDEO_H264_LEVEL_4_2:
        return 50000000;
    case V4L2_MPEG_VIDEO_H264_LEVEL_5_0:
        return 135000000;
    case V4L2_MPEG_VIDEO_H264_LEVEL_5_1:
    default:
        return 240000000;
    }
}


static unsigned int h264_maximum_cpb_size(enum v4l2_mpeg_video_h264_level level)
{
    switch (level) {
    case V4L2_MPEG_VIDEO_H264_LEVEL_1_0:
        return 175;
    case V4L2_MPEG_VIDEO_H264_LEVEL_1B:
        return 350;
    case V4L2_MPEG_VIDEO_H264_LEVEL_1_1:
        return 500;
    case V4L2_MPEG_VIDEO_H264_LEVEL_1_2:
        return 1000;
    case V4L2_MPEG_VIDEO_H264_LEVEL_1_3:
        return 2000;
    case V4L2_MPEG_VIDEO_H264_LEVEL_2_0:
        return 2000;
    case V4L2_MPEG_VIDEO_H264_LEVEL_2_1:
        return 4000;
    case V4L2_MPEG_VIDEO_H264_LEVEL_2_2:
        return 4000;
    case V4L2_MPEG_VIDEO_H264_LEVEL_3_0:
        return 10000;
    case V4L2_MPEG_VIDEO_H264_LEVEL_3_1:
        return 14000;
    case V4L2_MPEG_VIDEO_H264_LEVEL_3_2:
        return 20000;
    case V4L2_MPEG_VIDEO_H264_LEVEL_4_0:
        return 25000;
    case V4L2_MPEG_VIDEO_H264_LEVEL_4_1:
        return 62500;
    case V4L2_MPEG_VIDEO_H264_LEVEL_4_2:
        return 62500;
    case V4L2_MPEG_VIDEO_H264_LEVEL_5_0:
        return 135000;
    case V4L2_MPEG_VIDEO_H264_LEVEL_5_1:
    default:
        return 240000;
    }
}

static enum v4l2_mpeg_video_hevc_level
select_minimum_hevc_level(unsigned int width, unsigned int height)
{
    unsigned int luma_picture_size = width * height;
    enum v4l2_mpeg_video_hevc_level level;

    if (luma_picture_size <= 36864)
        level = V4L2_MPEG_VIDEO_HEVC_LEVEL_1;
    else if (luma_picture_size <= 122880)
        level = V4L2_MPEG_VIDEO_HEVC_LEVEL_2;
    else if (luma_picture_size <= 245760)
        level = V4L2_MPEG_VIDEO_HEVC_LEVEL_2_1;
    else if (luma_picture_size <= 552960)
        level = V4L2_MPEG_VIDEO_HEVC_LEVEL_3;
    else if (luma_picture_size <= 983040)
        level = V4L2_MPEG_VIDEO_HEVC_LEVEL_3_1;
    else if (luma_picture_size <= 2228224)
        level = V4L2_MPEG_VIDEO_HEVC_LEVEL_4;
    else if (luma_picture_size <= 8912896)
        level = V4L2_MPEG_VIDEO_HEVC_LEVEL_5;
    else
        level = V4L2_MPEG_VIDEO_HEVC_LEVEL_6;

    return level;
}

static unsigned int hevc_maximum_bitrate(enum v4l2_mpeg_video_hevc_level level)
{
    /*
     * See Rec. ITU-T H.265 v5 (02/2018), A.4.2 Profile-specific level
     * limits for the video profiles.
     */
    switch (level) {
    case V4L2_MPEG_VIDEO_HEVC_LEVEL_1:
        return 128;
    case V4L2_MPEG_VIDEO_HEVC_LEVEL_2:
        return 1500;
    case V4L2_MPEG_VIDEO_HEVC_LEVEL_2_1:
        return 3000;
    case V4L2_MPEG_VIDEO_HEVC_LEVEL_3:
        return 6000;
    case V4L2_MPEG_VIDEO_HEVC_LEVEL_3_1:
        return 10000;
    case V4L2_MPEG_VIDEO_HEVC_LEVEL_4:
        return 12000;
    case V4L2_MPEG_VIDEO_HEVC_LEVEL_4_1:
        return 20000;
    case V4L2_MPEG_VIDEO_HEVC_LEVEL_5:
        return 25000;
    default:
    case V4L2_MPEG_VIDEO_HEVC_LEVEL_5_1:
        return 40000;
    }
}

static unsigned int hevc_maximum_cpb_size(enum v4l2_mpeg_video_hevc_level level)
{
    switch (level) {
    case V4L2_MPEG_VIDEO_HEVC_LEVEL_1:
        return 350;
    case V4L2_MPEG_VIDEO_HEVC_LEVEL_2:
        return 1500;
    case V4L2_MPEG_VIDEO_HEVC_LEVEL_2_1:
        return 3000;
    case V4L2_MPEG_VIDEO_HEVC_LEVEL_3:
        return 6000;
    case V4L2_MPEG_VIDEO_HEVC_LEVEL_3_1:
        return 10000;
    case V4L2_MPEG_VIDEO_HEVC_LEVEL_4:
        return 12000;
    case V4L2_MPEG_VIDEO_HEVC_LEVEL_4_1:
        return 20000;
    case V4L2_MPEG_VIDEO_HEVC_LEVEL_5:
        return 25000;
    default:
    case V4L2_MPEG_VIDEO_HEVC_LEVEL_5_1:
        return 40000;
    }
}

/*
 * Mailbox interface to send messages to the MCU.
 */

static u32 v4l2_pixelformat_to_mcu_format(u32 pixelformat)
{
    switch (pixelformat) {
    case V4L2_PIX_FMT_NV12:
        /* AL_420_8BITS: 0x100 -> NV12, 0x88 -> 8 bit */
        return 0x100 | 0x88;
    default:
        return -EINVAL;
    }
}

static u32 v4l2_colorspace_to_mcu_colorspace(enum v4l2_colorspace colorspace)
{
    switch (colorspace) {
    case V4L2_COLORSPACE_REC709:
        return 2;
    case V4L2_COLORSPACE_SMPTE170M:
        return 3;
    case V4L2_COLORSPACE_SMPTE240M:
        return 4;
    case V4L2_COLORSPACE_SRGB:
        return 7;
    default:
        /* UNKNOWN */
        return 0;
    }
}

static u8 v4l2_profile_to_mcu_profile(enum v4l2_mpeg_video_h264_profile profile)
{
    switch (profile) {
    case V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE:
    default:
        return 66;
    }
}

static u16 v4l2_level_to_mcu_level(enum v4l2_mpeg_video_h264_level level)
{
    switch (level) {
    case V4L2_MPEG_VIDEO_H264_LEVEL_1_0:
        return 10;
    case V4L2_MPEG_VIDEO_H264_LEVEL_1_1:
        return 11;
    case V4L2_MPEG_VIDEO_H264_LEVEL_1_2:
        return 12;
    case V4L2_MPEG_VIDEO_H264_LEVEL_1_3:
        return 13;
    case V4L2_MPEG_VIDEO_H264_LEVEL_2_0:
        return 20;
    case V4L2_MPEG_VIDEO_H264_LEVEL_2_1:
        return 21;
    case V4L2_MPEG_VIDEO_H264_LEVEL_2_2:
        return 22;
    case V4L2_MPEG_VIDEO_H264_LEVEL_3_0:
        return 30;
    case V4L2_MPEG_VIDEO_H264_LEVEL_3_1:
        return 31;
    case V4L2_MPEG_VIDEO_H264_LEVEL_3_2:
        return 32;
    case V4L2_MPEG_VIDEO_H264_LEVEL_4_0:
        return 40;
    case V4L2_MPEG_VIDEO_H264_LEVEL_4_1:
        return 41;
    case V4L2_MPEG_VIDEO_H264_LEVEL_4_2:
        return 42;
    case V4L2_MPEG_VIDEO_H264_LEVEL_5_0:
        return 50;
    case V4L2_MPEG_VIDEO_H264_LEVEL_5_1:
    default:
        return 51;
    }
}

static u8 hevc_profile_to_mcu_profile(enum v4l2_mpeg_video_hevc_profile profile)
{
    switch (profile) {
    default:
    case V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN:
        return 1;
    case V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_10:
        return 2;
    case V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_STILL_PICTURE:
        return 3;
    }
}

static u16 hevc_level_to_mcu_level(enum v4l2_mpeg_video_hevc_level level)
{
    switch (level) {
    case V4L2_MPEG_VIDEO_HEVC_LEVEL_1:
        return 10;
    case V4L2_MPEG_VIDEO_HEVC_LEVEL_2:
        return 20;
    case V4L2_MPEG_VIDEO_HEVC_LEVEL_2_1:
        return 21;
    case V4L2_MPEG_VIDEO_HEVC_LEVEL_3:
        return 30;
    case V4L2_MPEG_VIDEO_HEVC_LEVEL_3_1:
        return 31;
    case V4L2_MPEG_VIDEO_HEVC_LEVEL_4:
        return 40;
    case V4L2_MPEG_VIDEO_HEVC_LEVEL_4_1:
        return 41;
    case V4L2_MPEG_VIDEO_HEVC_LEVEL_5:
        return 50;
    default:
    case V4L2_MPEG_VIDEO_HEVC_LEVEL_5_1:
        return 51;
    }
}

static u8 hevc_tier_to_mcu_tier(enum v4l2_mpeg_video_hevc_tier tier)
{
    switch (tier) {
    default:
    case V4L2_MPEG_VIDEO_HEVC_TIER_MAIN:
        return 0;
    case V4L2_MPEG_VIDEO_HEVC_TIER_HIGH:
        return 1;
    }
}

static u32
v4l2_bitrate_mode_to_mcu_mode(enum v4l2_mpeg_video_bitrate_mode mode)
{
    switch (mode) {
    case V4L2_MPEG_VIDEO_BITRATE_MODE_VBR:
        return 2;
    case V4L2_MPEG_VIDEO_BITRATE_MODE_CBR:
    default:
        return 1;
    }
}

static u32 v4l2_cpb_size_to_mcu(unsigned int cpb_size, unsigned int bitrate)
{
    unsigned int cpb_size_kbit;
    unsigned int bitrate_kbps;

    /*
     * The mcu expects the CPB size in units of a 90 kHz clock, but the
     * channel follows the V4L2_CID_MPEG_VIDEO_H264_CPB_SIZE and stores
     * the CPB size in kilobytes.
     */
    cpb_size_kbit = cpb_size * BITS_PER_BYTE;
    bitrate_kbps = bitrate / 1000;

    return (cpb_size_kbit * 90000) / bitrate_kbps;
}

static s16 get_qp_delta(int minuend, int subtrahend)
{
    if (minuend == subtrahend)
        return -1;
    else
        return minuend - subtrahend;
}

static u32 allegro_channel_get_entropy_mode(
    struct allegro_channel *channel, unsigned int codec)
{
    /* HEVC always uses CABAC, but this has to be explicitly set */
    if (codec == V4L2_PIX_FMT_HEVC)
        return ALLEGRO_ENTROPY_MODE_CABAC;

    return ALLEGRO_ENTROPY_MODE_CAVLC;
}

static int fill_create_channel_param(struct allegro_channel *channel,
                     struct create_encode_channel_param *param)
{
    struct allegro_q_data *q_data_src =
                get_q_data(channel, V4L2_BUF_TYPE_VIDEO_OUTPUT);
    struct allegro_q_data *q_data_dst =
                get_q_data(channel, V4L2_BUF_TYPE_VIDEO_CAPTURE);

    unsigned int codec = q_data_dst->fourcc;
    unsigned int pixelformat = q_data_src->fourcc;

    int i_frame_qp = allegro_channel_get_i_frame_qp(channel, codec);
    int p_frame_qp = allegro_channel_get_p_frame_qp(channel, codec);
    int b_frame_qp = allegro_channel_get_b_frame_qp(channel, codec);
    int bitrate_mode = v4l2_ctrl_g_ctrl(channel->mpeg_video_bitrate_mode);
    unsigned int cpb_size = v4l2_ctrl_g_ctrl(channel->mpeg_video_cpb_size);

    param->width = q_data_src->width;
    param->height = q_data_src->height;
    param->format = v4l2_pixelformat_to_mcu_format(pixelformat);
    param->colorspace =
        v4l2_colorspace_to_mcu_colorspace(channel->colorspace);
    param->src_mode = 0x0;

    param->codec = codec;
    if (codec == V4L2_PIX_FMT_H264) {
        enum v4l2_mpeg_video_h264_profile profile;
        enum v4l2_mpeg_video_h264_level level;

        profile = v4l2_ctrl_g_ctrl(channel->mpeg_video_h264_profile);
        level = v4l2_ctrl_g_ctrl(channel->mpeg_video_h264_level);

        param->profile = v4l2_profile_to_mcu_profile(profile);
        param->constraint_set_flags = BIT(1);
        param->level = v4l2_level_to_mcu_level(level);
    } else {
        enum v4l2_mpeg_video_hevc_profile profile;
        enum v4l2_mpeg_video_hevc_level level;
        enum v4l2_mpeg_video_hevc_tier tier;
        profile = v4l2_ctrl_g_ctrl(channel->mpeg_video_hevc_profile);
        level = v4l2_ctrl_g_ctrl(channel->mpeg_video_hevc_level);
        tier = v4l2_ctrl_g_ctrl(channel->mpeg_video_hevc_tier);
        param->profile = hevc_profile_to_mcu_profile(profile);
        param->level = hevc_level_to_mcu_level(level);
        param->tier = hevc_tier_to_mcu_tier(tier);
    }

    param->log2_max_poc = LOG2_MAX_PIC_ORDER_CNT;
    param->log2_max_frame_num = channel->log2_max_frame_num;
    param->temporal_mvp_enable = channel->temporal_mvp_enable;

    param->dbf_ovr_en = channel->dbf_ovr_en;
    param->override_lf = channel->enable_deblocking_filter_override;
    param->enable_reordering = channel->enable_reordering;
    param->entropy_mode = allegro_channel_get_entropy_mode(channel, codec);
    param->rdo_cost_mode = 1;
    param->custom_lda = 1;
    param->lf = 1;
    param->lf_x_tile = channel->enable_loop_filter_across_tiles;
    param->lf_x_slice = channel->enable_loop_filter_across_slices;

    param->src_bit_depth = 8;

    param->beta_offset = BETA_OFFSET_DIV_2;
    param->tc_offset = TC_OFFSET_DIV_2;
    param->num_slices = 1;
    param->me_range[0] = channel->b_hrz_me_range;
    param->me_range[1] = channel->b_vrt_me_range;
    param->me_range[2] = channel->p_hrz_me_range;
    param->me_range[3] = channel->p_vrt_me_range;
    param->max_cu_size = channel->max_cu_size;
    param->min_cu_size = channel->min_cu_size;
    param->max_tu_size = channel->max_tu_size;
    param->min_tu_size = channel->min_tu_size;
    param->max_transfo_depth_intra = channel->max_transfo_depth_intra;
    param->max_transfo_depth_inter = channel->max_transfo_depth_inter;

    param->prefetch_auto = 0;
    param->prefetch_mem_offset = 0;
    param->prefetch_mem_size = 0;

    param->rate_control_mode = channel->frame_rc_enable ?
        v4l2_bitrate_mode_to_mcu_mode(bitrate_mode) : 0;

    param->cpb_size = v4l2_cpb_size_to_mcu(cpb_size, channel->bitrate_peak);
    /* Shall be ]0;cpb_size in 90 kHz units]. Use maximum value. */
    param->initial_rem_delay = param->cpb_size;
    param->framerate = DIV_ROUND_UP(channel->framerate.numerator,
                    channel->framerate.denominator);
    param->clk_ratio = channel->framerate.denominator == 1001 ? 1001 : 1000;
    param->target_bitrate = channel->bitrate;
    param->max_bitrate = channel->bitrate_peak;
    param->initial_qp = i_frame_qp;
    param->min_qp = allegro_channel_get_min_qp(channel, codec);
    param->max_qp = allegro_channel_get_max_qp(channel, codec);
    param->ip_delta = get_qp_delta(i_frame_qp, p_frame_qp);
    param->pb_delta = get_qp_delta(p_frame_qp, b_frame_qp);
    param->golden_ref = 0;
    param->golden_delta = 2;
    param->golden_ref_frequency = 10;
    param->rate_control_option = 0x00000000;

    param->num_pixel = param->width + param->height;
    param->max_psnr = 4200;
    param->max_pixel_value = 255;

    param->gop_ctrl_mode = 0x00000002;
    param->freq_idr = v4l2_ctrl_g_ctrl(channel->mpeg_video_gop_size);
    param->freq_lt = 0;
    param->gdr_mode = 0x00000000;
    param->gop_length = v4l2_ctrl_g_ctrl(channel->mpeg_video_gop_size);
    param->subframe_latency = 0x00000000;

    param->lda_factors[0] = 51;
    param->lda_factors[1] = 90;
    param->lda_factors[2] = 151;
    param->lda_factors[3] = 151;
    param->lda_factors[4] = 151;
    param->lda_factors[5] = 151;

    param->max_num_merge_cand = 5;

    return 0;
}

static int allegro_mcu_send_create_channel(struct allegro_dev *dev,
                       struct allegro_channel *channel)
{
    struct mcu_msg_create_channel msg;
    struct allegro_buffer *blob = &channel->config_blob;
    struct create_encode_channel_param param;
    size_t size;

    memset(&param, 0, sizeof(param));
    param.version = dev->fw_info->mailbox_version;
    fill_create_channel_param(channel, &param);
    allegro_alloc_buffer(dev, blob, sizeof(struct create_encode_channel_param));
    size = allegro_pack_encoder_config_blob(blob->vaddr, &param);

    memset(&msg, 0, sizeof(msg));

    msg.header.type = MCU_MSG_TYPE_CREATE_CHANNEL;
    msg.header.version = dev->fw_info->mailbox_version;
    msg.header.devtype = channel->inst_type;

    msg.user_id = channel->user_id;

    msg.blob = blob->vaddr;
    msg.blob_size = blob->size;
    msg.blob_mcu_addr = to_mcu_addr(dev, blob->paddr);

    allegro_mbox_send(dev->mbox_command, &msg);

    return 0;
}

static int allegro_mcu_send_destroy_channel(struct allegro_dev *dev,
                        struct allegro_channel *channel)
{
    struct mcu_msg_destroy_channel msg;

    memset(&msg, 0, sizeof(msg));

    msg.header.type = MCU_MSG_TYPE_DESTROY_CHANNEL;
    msg.header.version = dev->fw_info->mailbox_version;
    msg.header.devtype = channel->inst_type;

    msg.channel_id = channel->mcu_channel_id;

    allegro_mbox_send(dev->mbox_command, &msg);

    return 0;
}

static int allegro_mcu_send_put_stream_buffer(struct allegro_dev *dev,
                          struct allegro_channel *channel,
                          dma_addr_t paddr,
                          unsigned long size,
                          u64 dst_handle)
{
    struct mcu_msg_put_stream_buffer msg;

    memset(&msg, 0, sizeof(msg));

    msg.header.type = MCU_MSG_TYPE_PUT_STREAM_BUFFER;
    msg.header.version = dev->fw_info->mailbox_version;
    msg.header.devtype = channel->inst_type;

    msg.channel_id = channel->mcu_channel_id;
    msg.dma_addr = to_codec_addr(dev, paddr);
    msg.mcu_addr = to_mcu_addr(dev, paddr);
    msg.size = size;
    msg.offset = ENCODER_STREAM_OFFSET;
    /* copied to mcu_msg_encode_frame_response */
    msg.dst_handle = dst_handle;

    allegro_mbox_send(dev->mbox_command, &msg);

    return 0;
}

static int allegro_mcu_send_encode_frame(struct allegro_dev *dev,
                     struct allegro_channel *channel,
                     dma_addr_t src_y, dma_addr_t src_uv,
                     u64 src_handle)
{
    struct allegro_q_data *q_data;
    struct mcu_msg_encode_frame msg;

    memset(&msg, 0, sizeof(msg));

    q_data = get_q_data(channel, V4L2_BUF_TYPE_VIDEO_OUTPUT);

    msg.header.type = MCU_MSG_TYPE_ENCODE_FRAME;
    msg.header.version = dev->fw_info->mailbox_version;
    msg.header.devtype = channel->inst_type;

    msg.channel_id = channel->mcu_channel_id;
    msg.encoding_options = AL_OPT_FORCE_LOAD;
    msg.pps_qp = 26; /* qp are relative to 26 */
    msg.user_param = 0; /* copied to mcu_msg_encode_frame_response */
    /* src_handle is copied to mcu_msg_encode_frame_response */
    msg.src_handle = src_handle;
    msg.src_y = to_codec_addr(dev, src_y);
    msg.src_uv = to_codec_addr(dev, src_uv);
    msg.stride = q_data->bytesperline;
    msg.ep2 = 0x0;
    msg.ep2_v = to_mcu_addr(dev, msg.ep2);

    allegro_mbox_send(dev->mbox_command, &msg);

    return 0;
}

static int allegro_mcu_push_buffer_internal(
                    struct allegro_channel *channel,
                    enum mcu_msg_type type)
{
    struct allegro_dev *dev = channel->dev;
    struct mcu_msg_push_buffers_internal *msg;
    struct mcu_msg_push_buffers_internal_buffer *buffer;
    unsigned int num_buffers = 0;
    size_t size;
    struct allegro_buffer *al_buffer;
    struct list_head *list;
    int err;

    switch (type) {
    case MCU_MSG_TYPE_PUSH_BUFFER_REFERENCE:
        list = &channel->buffers_reference;
        break;
    case MCU_MSG_TYPE_PUSH_BUFFER_INTERMEDIATE:
        list = &channel->buffers_intermediate;
        break;
    default:
        return -EINVAL;
    }

    list_for_each_entry(al_buffer, list, head)
        num_buffers++;
    size = struct_size(msg, buffer, num_buffers);

    msg = kmalloc(size, GFP_KERNEL);
    if (!msg)
        return -ENOMEM;

    msg->header.type = type;
    msg->header.version = dev->fw_info->mailbox_version;
    msg->header.devtype = channel->inst_type;

    msg->channel_id = channel->mcu_channel_id;
    msg->num_buffers = num_buffers;

    buffer = msg->buffer;
    list_for_each_entry(al_buffer, list, head) {
        buffer->dma_addr = to_codec_addr(dev, al_buffer->paddr);
        buffer->mcu_addr = to_mcu_addr(dev, al_buffer->paddr);
        buffer->size = to_mcu_size(dev, al_buffer->size);
        buffer++;
    }

    err = allegro_mbox_send(dev->mbox_command, msg);

    kfree(msg);
    return err;
}

static int allegro_mcu_push_buffer_intermediate(struct allegro_channel *channel)
{
    enum mcu_msg_type type = MCU_MSG_TYPE_PUSH_BUFFER_INTERMEDIATE;

    return allegro_mcu_push_buffer_internal(channel, type);
}

static int allegro_mcu_push_buffer_reference(struct allegro_channel *channel)
{
    enum mcu_msg_type type = MCU_MSG_TYPE_PUSH_BUFFER_REFERENCE;

    return allegro_mcu_push_buffer_internal(channel, type);
}

static int allocate_buffers_internal(struct allegro_channel *channel,
                     struct list_head *list,
                     size_t n, size_t size)
{
    struct allegro_dev *dev = channel->dev;
    unsigned int i;
    int err;
    struct allegro_buffer *buffer, *tmp;

    for (i = 0; i < n; i++) {
        buffer = kmalloc(sizeof(*buffer), GFP_KERNEL);
        if (!buffer) {
            err = -ENOMEM;
            goto err;
        }
        INIT_LIST_HEAD(&buffer->head);
        buffer->id = i;

        err = allegro_alloc_buffer(dev, buffer, size);
        if (err)
            goto err;
        list_add(&buffer->head, list);
    }

    return 0;

err:
    list_for_each_entry_safe(buffer, tmp, list, head) {
        list_del(&buffer->head);
        allegro_free_buffer(dev, buffer);
        kfree(buffer);
    }
    return err;
}

static void destroy_buffers_internal(struct allegro_channel *channel,
                     struct list_head *list)
{
    struct allegro_dev *dev = channel->dev;
    struct allegro_buffer *buffer, *tmp;

    list_for_each_entry_safe(buffer, tmp, list, head) {
        list_del(&buffer->head);
        allegro_free_buffer(dev, buffer);
        kfree(buffer);
    }
}

static void destroy_reference_buffers(struct allegro_channel *channel)
{
    return destroy_buffers_internal(channel, &channel->buffers_reference);
}

static void destroy_intermediate_buffers(struct allegro_channel *channel)
{
    return destroy_buffers_internal(channel,
                    &channel->buffers_intermediate);
}

static int allocate_intermediate_buffers(struct allegro_channel *channel,
                     size_t n, size_t size)
{
    return allocate_buffers_internal(channel,
                     &channel->buffers_intermediate,
                     n, size);
}

static int allocate_reference_buffers(struct allegro_channel *channel,
                      size_t n, size_t size)
{
    return allocate_buffers_internal(channel,
                     &channel->buffers_reference,
                     n, PAGE_ALIGN(size));
}

static ssize_t allegro_h264_write_sps(struct allegro_channel *channel,
                      void *dest, size_t n)
{
    struct allegro_dev *dev = channel->dev;
    struct allegro_q_data *q_data;
    struct nal_h264_sps *sps;
    ssize_t size;
    unsigned int size_mb = SIZE_MACROBLOCK;
    /* Calculation of crop units in Rec. ITU-T H.264 (04/2017) p. 76 */
    unsigned int crop_unit_x = 2;
    unsigned int crop_unit_y = 2;
    enum v4l2_mpeg_video_h264_profile profile;
    enum v4l2_mpeg_video_h264_level level;
    unsigned int cpb_size;
    unsigned int cpb_size_scale;

    sps = kzalloc(sizeof(*sps), GFP_KERNEL);
    if (!sps)
        return -ENOMEM;

    q_data = get_q_data(channel, V4L2_BUF_TYPE_VIDEO_CAPTURE);

    profile = v4l2_ctrl_g_ctrl(channel->mpeg_video_h264_profile);
    level = v4l2_ctrl_g_ctrl(channel->mpeg_video_h264_level);

    sps->profile_idc = nal_h264_profile_from_v4l2(profile);
    sps->constraint_set0_flag = 0;
    sps->constraint_set1_flag = 1;
    sps->constraint_set2_flag = 0;
    sps->constraint_set3_flag = 0;
    sps->constraint_set4_flag = 0;
    sps->constraint_set5_flag = 0;
    sps->level_idc = nal_h264_level_from_v4l2(level);
    sps->seq_parameter_set_id = 0;
    sps->log2_max_frame_num_minus4 = LOG2_MAX_FRAME_NUM - 4;
    sps->pic_order_cnt_type = 0;
    sps->log2_max_pic_order_cnt_lsb_minus4 = LOG2_MAX_PIC_ORDER_CNT - 4;
    sps->max_num_ref_frames = 3;
    sps->gaps_in_frame_num_value_allowed_flag = 0;
    sps->pic_width_in_mbs_minus1 =
        DIV_ROUND_UP(q_data->width, size_mb) - 1;
    sps->pic_height_in_map_units_minus1 =
        DIV_ROUND_UP(q_data->height, size_mb) - 1;
    sps->frame_mbs_only_flag = 1;
    sps->mb_adaptive_frame_field_flag = 0;
    sps->direct_8x8_inference_flag = 1;
    sps->frame_cropping_flag =
        (q_data->width % size_mb) || (q_data->height % size_mb);
    if (sps->frame_cropping_flag) {
        sps->crop_left = 0;
        sps->crop_right = (round_up(q_data->width, size_mb) - q_data->width) / crop_unit_x;
        sps->crop_top = 0;
        sps->crop_bottom = (round_up(q_data->height, size_mb) - q_data->height) / crop_unit_y;
    }
    sps->vui_parameters_present_flag = 1;
    sps->vui.aspect_ratio_info_present_flag = 0;
    sps->vui.overscan_info_present_flag = 0;
    sps->vui.video_signal_type_present_flag = 1;
    sps->vui.video_format = 1;
    sps->vui.video_full_range_flag = 0;
    sps->vui.colour_description_present_flag = 1;
    sps->vui.colour_primaries = 5;
    sps->vui.transfer_characteristics = 5;
    sps->vui.matrix_coefficients = 5;
    sps->vui.chroma_loc_info_present_flag = 1;
    sps->vui.chroma_sample_loc_type_top_field = 0;
    sps->vui.chroma_sample_loc_type_bottom_field = 0;

    sps->vui.timing_info_present_flag = 1;
    sps->vui.num_units_in_tick = channel->framerate.denominator;
    sps->vui.time_scale = 2 * channel->framerate.numerator;

    sps->vui.fixed_frame_rate_flag = 1;
    sps->vui.nal_hrd_parameters_present_flag = 0;
    sps->vui.vcl_hrd_parameters_present_flag = 1;
    sps->vui.vcl_hrd_parameters.cpb_cnt_minus1 = 0;
    sps->vui.vcl_hrd_parameters.bit_rate_scale = 0;
    /* See Rec. ITU-T H.264 (04/2017) p. 410 E-53 */
    sps->vui.vcl_hrd_parameters.bit_rate_value_minus1[0] =
        channel->bitrate_peak / (1 << (6 + sps->vui.vcl_hrd_parameters.bit_rate_scale)) - 1;
    /* See Rec. ITU-T H.264 (04/2017) p. 410 E-54 */
    cpb_size = v4l2_ctrl_g_ctrl(channel->mpeg_video_cpb_size);
    cpb_size_scale = ffs(cpb_size) - 4;
    sps->vui.vcl_hrd_parameters.cpb_size_scale = cpb_size_scale;
    sps->vui.vcl_hrd_parameters.cpb_size_value_minus1[0] =
        (cpb_size * 1000) / (1 << (4 + cpb_size_scale)) - 1;
    sps->vui.vcl_hrd_parameters.cbr_flag[0] =
        !v4l2_ctrl_g_ctrl(channel->mpeg_video_frame_rc_enable);
    sps->vui.vcl_hrd_parameters.initial_cpb_removal_delay_length_minus1 = 31;
    sps->vui.vcl_hrd_parameters.cpb_removal_delay_length_minus1 = 31;
    sps->vui.vcl_hrd_parameters.dpb_output_delay_length_minus1 = 31;
    sps->vui.vcl_hrd_parameters.time_offset_length = 0;
    sps->vui.low_delay_hrd_flag = 0;
    sps->vui.pic_struct_present_flag = 1;
    sps->vui.bitstream_restriction_flag = 0;

    size = nal_h264_write_sps(&dev->plat_dev->dev, dest, n, sps);

    kfree(sps);

    return size;
}

static ssize_t allegro_h264_write_pps(struct allegro_channel *channel,
                      void *dest, size_t n)
{
    struct allegro_dev *dev = channel->dev;
    struct nal_h264_pps *pps;
    ssize_t size;

    pps = kzalloc(sizeof(*pps), GFP_KERNEL);
    if (!pps)
        return -ENOMEM;

    pps->pic_parameter_set_id = 0;
    pps->seq_parameter_set_id = 0;
    pps->entropy_coding_mode_flag = 0;
    pps->bottom_field_pic_order_in_frame_present_flag = 0;
    pps->num_slice_groups_minus1 = 0;
    pps->num_ref_idx_l0_default_active_minus1 = channel->num_ref_idx_l0 - 1;
    pps->num_ref_idx_l1_default_active_minus1 = channel->num_ref_idx_l1 - 1;
    pps->weighted_pred_flag = 0;
    pps->weighted_bipred_idc = 0;
    pps->pic_init_qp_minus26 = 0;
    pps->pic_init_qs_minus26 = 0;
    pps->chroma_qp_index_offset = 0;
    pps->deblocking_filter_control_present_flag = 1;
    pps->constrained_intra_pred_flag = 0;
    pps->redundant_pic_cnt_present_flag = 0;
    pps->transform_8x8_mode_flag = 0;
    pps->pic_scaling_matrix_present_flag = 0;
    pps->second_chroma_qp_index_offset = 0;

    size = nal_h264_write_pps(&dev->plat_dev->dev, dest, n, pps);

    kfree(pps);

    return size;
}

static ssize_t allegro_hevc_write_vps(struct allegro_channel *channel,
                      void *dest, size_t n)
{
    struct allegro_dev *dev = channel->dev;
    struct nal_hevc_vps *vps;
    struct nal_hevc_profile_tier_level *ptl;
    ssize_t size;
    unsigned int num_ref_frames = channel->num_ref_idx_l0;
    s32 profile = v4l2_ctrl_g_ctrl(channel->mpeg_video_hevc_profile);
    s32 level = v4l2_ctrl_g_ctrl(channel->mpeg_video_hevc_level);
    s32 tier = v4l2_ctrl_g_ctrl(channel->mpeg_video_hevc_tier);

    vps = kzalloc(sizeof(*vps), GFP_KERNEL);
    if (!vps)
        return -ENOMEM;

    vps->base_layer_internal_flag = 1;
    vps->base_layer_available_flag = 1;
    vps->temporal_id_nesting_flag = 1;

    ptl = &vps->profile_tier_level;
    ptl->general_profile_idc = nal_hevc_profile_from_v4l2(profile);
    ptl->general_profile_compatibility_flag[ptl->general_profile_idc] = 1;
    ptl->general_tier_flag = nal_hevc_tier_from_v4l2(tier);
    ptl->general_progressive_source_flag = 1;
    ptl->general_frame_only_constraint_flag = 1;
    ptl->general_level_idc = nal_hevc_level_from_v4l2(level);

    vps->sub_layer_ordering_info_present_flag = 0;
    vps->max_dec_pic_buffering_minus1[0] = num_ref_frames;
    vps->max_num_reorder_pics[0] = num_ref_frames;

    size = nal_hevc_write_vps(&dev->plat_dev->dev, dest, n, vps);

    kfree(vps);

    return size;
}

static ssize_t allegro_hevc_write_sps(struct allegro_channel *channel,
                      void *dest, size_t n)
{
    struct allegro_dev *dev = channel->dev;
    struct allegro_q_data *q_data;
    struct nal_hevc_sps *sps;
    struct nal_hevc_profile_tier_level *ptl;
    ssize_t size;
    unsigned int num_ref_frames = channel->num_ref_idx_l0;
    s32 profile = v4l2_ctrl_g_ctrl(channel->mpeg_video_hevc_profile);
    s32 level = v4l2_ctrl_g_ctrl(channel->mpeg_video_hevc_level);
    s32 tier = v4l2_ctrl_g_ctrl(channel->mpeg_video_hevc_tier);

    sps = kzalloc(sizeof(*sps), GFP_KERNEL);
    if (!sps)
        return -ENOMEM;

    q_data = get_q_data(channel, V4L2_BUF_TYPE_VIDEO_CAPTURE);

    sps->temporal_id_nesting_flag = 1;

    ptl = &sps->profile_tier_level;
    ptl->general_profile_idc = nal_hevc_profile_from_v4l2(profile);
    ptl->general_profile_compatibility_flag[ptl->general_profile_idc] = 1;
    ptl->general_tier_flag = nal_hevc_tier_from_v4l2(tier);
    ptl->general_progressive_source_flag = 1;
    ptl->general_frame_only_constraint_flag = 1;
    ptl->general_level_idc = nal_hevc_level_from_v4l2(level);

    sps->seq_parameter_set_id = 0;
    sps->chroma_format_idc = 1; /* Only 4:2:0 sampling supported */
    sps->pic_width_in_luma_samples = round_up(q_data->width, 8);
    sps->pic_height_in_luma_samples = round_up(q_data->height, 8);
    sps->conf_win_right_offset =
        sps->pic_width_in_luma_samples - q_data->width;
    sps->conf_win_bottom_offset =
        sps->pic_height_in_luma_samples - q_data->height;
    sps->conformance_window_flag =
        sps->conf_win_right_offset || sps->conf_win_bottom_offset;

    sps->log2_max_pic_order_cnt_lsb_minus4 = LOG2_MAX_PIC_ORDER_CNT - 4;

    sps->sub_layer_ordering_info_present_flag = 1;
    sps->max_dec_pic_buffering_minus1[0] = num_ref_frames;
    sps->max_num_reorder_pics[0] = num_ref_frames;

    sps->log2_min_luma_coding_block_size_minus3 =
        channel->min_cu_size - 3;
    sps->log2_diff_max_min_luma_coding_block_size =
        channel->max_cu_size - channel->min_cu_size;
    sps->log2_min_luma_transform_block_size_minus2 =
        channel->min_tu_size - 2;
    sps->log2_diff_max_min_luma_transform_block_size =
        channel->max_tu_size - channel->min_tu_size;
    sps->max_transform_hierarchy_depth_intra =
        channel->max_transfo_depth_intra;
    sps->max_transform_hierarchy_depth_inter =
        channel->max_transfo_depth_inter;

    sps->sps_temporal_mvp_enabled_flag = channel->temporal_mvp_enable;
    sps->strong_intra_smoothing_enabled_flag = channel->max_cu_size > 4;

    size = nal_hevc_write_sps(&dev->plat_dev->dev, dest, n, sps);

    kfree(sps);

    return size;
}

static ssize_t allegro_hevc_write_pps(struct allegro_channel *channel,
                      struct mcu_msg_encode_frame_response *msg,
                      void *dest, size_t n)
{
    struct allegro_dev *dev = channel->dev;
    struct nal_hevc_pps *pps;
    ssize_t size;
    int i;

    pps = kzalloc(sizeof(*pps), GFP_KERNEL);
    if (!pps)
        return -ENOMEM;

    pps->pps_pic_parameter_set_id = 0;
    pps->pps_seq_parameter_set_id = 0;

    if (msg->num_column > 1 || msg->num_row > 1) {
        pps->tiles_enabled_flag = 1;
        pps->num_tile_columns_minus1 = msg->num_column - 1;
        pps->num_tile_rows_minus1 = msg->num_row - 1;

        for (i = 0; i < msg->num_column; i++)
            pps->column_width_minus1[i] = msg->tile_width[i] - 1;

        for (i = 0; i < msg->num_row; i++)
            pps->row_height_minus1[i] = msg->tile_height[i] - 1;
    }

    pps->loop_filter_across_tiles_enabled_flag =
        channel->enable_loop_filter_across_tiles;
    pps->pps_loop_filter_across_slices_enabled_flag =
        channel->enable_loop_filter_across_slices;
    pps->deblocking_filter_control_present_flag = 1;
    pps->deblocking_filter_override_enabled_flag =
        channel->enable_deblocking_filter_override;
    pps->pps_beta_offset_div2 = BETA_OFFSET_DIV_2;
    pps->pps_tc_offset_div2 = TC_OFFSET_DIV_2;

    pps->lists_modification_present_flag = channel->enable_reordering;

    size = nal_hevc_write_pps(&dev->plat_dev->dev, dest, n, pps);

    kfree(pps);

    return size;
}

void allegro_channel_eos_event(struct allegro_channel *channel);

static void allegro_finish_encode_frame(struct allegro_channel *channel,
        struct mcu_msg_encode_frame_response *msg)
{
    struct allegro_dev *dev = channel->dev;
    struct allegro_q_data *q_data;
    struct vb2_v4l2_buffer *src_buf;
    struct vb2_v4l2_buffer *dst_buf;
    struct {
        u32 offset;
        u32 size;
    } *partition;
    enum vb2_buffer_state state = VB2_BUF_STATE_ERROR;
    char *curr;
    ssize_t len;
    ssize_t free;

    q_data = get_q_data(channel, V4L2_BUF_TYPE_VIDEO_CAPTURE);

    src_buf = allegro_get_buffer(channel, &channel->src_shadow_list,
                     msg->src_handle);
    if (!src_buf)
        v4l2_warn(&dev->v4l2_dev,
              "channel %d: invalid source buffer\n",
              channel->mcu_channel_id);

    dst_buf = allegro_get_buffer(channel, &channel->dst_shadow_list,
                     msg->dst_handle);
    if (!dst_buf)
        v4l2_warn(&dev->v4l2_dev,
              "channel %d: invalid stream buffer\n",
              channel->mcu_channel_id);

    if (!src_buf || !dst_buf)
        goto err;

    if (v4l2_m2m_is_last_draining_src_buf(channel->fh.m2m_ctx, src_buf)) {
        dst_buf->flags |= V4L2_BUF_FLAG_LAST;
        allegro_channel_eos_event(channel);
        v4l2_m2m_mark_stopped(channel->fh.m2m_ctx);
    }

    dst_buf->sequence = q_data->sequence++;

    if (msg->error_code & AL_ERROR) {
        v4l2_err(&dev->v4l2_dev,
             "channel %d: failed to encode frame: %s (%x)\n",
             channel->mcu_channel_id,
             allegro_err_to_string(msg->error_code),
             msg->error_code);
        goto err;
    }

    if (msg->partition_table_size != 1) {
        v4l2_warn(&dev->v4l2_dev,
              "channel %d: only handling first partition table entry (%d entries)\n",
              channel->mcu_channel_id, msg->partition_table_size);
    }

    if (msg->partition_table_offset +
        msg->partition_table_size * sizeof(*partition) >
        vb2_plane_size(&dst_buf->vb2_buf, 0)) {
        v4l2_err(&dev->v4l2_dev,
             "channel %d: partition table outside of dst_buf\n",
             channel->mcu_channel_id);
        goto err;
    }

    partition =
        vb2_plane_vaddr(&dst_buf->vb2_buf, 0) + msg->partition_table_offset;
    if (partition->offset + partition->size >
        vb2_plane_size(&dst_buf->vb2_buf, 0)) {
        v4l2_err(&dev->v4l2_dev,
             "channel %d: encoded frame is outside of dst_buf (offset 0x%x, size 0x%x)\n",
             channel->mcu_channel_id, partition->offset,
             partition->size);
        goto err;
    }

    v4l2_dbg(2, debug, &dev->v4l2_dev,
         "channel %d: encoded frame of size %d is at offset 0x%x\n",
         channel->mcu_channel_id, partition->size, partition->offset);

    /*
     * The payload must include the data before the partition offset,
     * because we will put the sps and pps data there.
     */
    vb2_set_plane_payload(&dst_buf->vb2_buf, 0,
                  partition->offset + partition->size);

    curr = vb2_plane_vaddr(&dst_buf->vb2_buf, 0);
    free = partition->offset;

    if (q_data->fourcc == V4L2_PIX_FMT_HEVC && msg->is_idr) {
        len = allegro_hevc_write_vps(channel, curr, free);
        if (len < 0) {
            v4l2_err(&dev->v4l2_dev,
                 "not enough space for video parameter set: %zd left\n",
                 free);
            goto err;
        }
        curr += len;
        free -= len;
        v4l2_dbg(1, debug, &dev->v4l2_dev,
             "channel %d: wrote %zd byte VPS nal unit\n",
             channel->mcu_channel_id, len);
    }

    if (msg->is_idr) {
        if (q_data->fourcc == V4L2_PIX_FMT_H264)
            len = allegro_h264_write_sps(channel, curr, free);
        else
            len = allegro_hevc_write_sps(channel, curr, free);
        if (len < 0) {
            v4l2_err(&dev->v4l2_dev,
                 "not enough space for sequence parameter set: %zd left\n",
                 free);
            goto err;
        }
        curr += len;
        free -= len;
        v4l2_dbg(1, debug, &dev->v4l2_dev,
             "channel %d: wrote %zd byte SPS nal unit\n",
             channel->mcu_channel_id, len);
    }

    if (msg->slice_type == AL_ENC_SLICE_TYPE_I) {
        if (q_data->fourcc == V4L2_PIX_FMT_H264)
            len = allegro_h264_write_pps(channel, curr, free);
        else
            len = allegro_hevc_write_pps(channel, msg, curr, free);
        if (len < 0) {
            v4l2_err(&dev->v4l2_dev,
                 "not enough space for picture parameter set: %zd left\n",
                 free);
            goto err;
        }
        curr += len;
        free -= len;
        v4l2_dbg(1, debug, &dev->v4l2_dev,
             "channel %d: wrote %zd byte PPS nal unit\n",
             channel->mcu_channel_id, len);
    }

    if (msg->slice_type != AL_ENC_SLICE_TYPE_I && !msg->is_idr) {
        dst_buf->vb2_buf.planes[0].data_offset = free;
        free = 0;
    } else {
        if (q_data->fourcc == V4L2_PIX_FMT_H264)
            len = nal_h264_write_filler(&dev->plat_dev->dev, curr, free);
        else
            len = nal_hevc_write_filler(&dev->plat_dev->dev, curr, free);
        if (len < 0) {
            v4l2_err(&dev->v4l2_dev,
                 "failed to write %zd filler data\n", free);
            goto err;
        }
        curr += len;
        free -= len;
        v4l2_dbg(2, debug, &dev->v4l2_dev,
             "channel %d: wrote %zd bytes filler nal unit\n",
             channel->mcu_channel_id, len);
    }

    if (free != 0) {
        v4l2_err(&dev->v4l2_dev,
             "non-VCL NAL units do not fill space until VCL NAL unit: %zd bytes left\n",
             free);
        goto err;
    }

    state = VB2_BUF_STATE_DONE;

    v4l2_m2m_buf_copy_metadata(src_buf, dst_buf, false);
    if (msg->is_idr)
        dst_buf->flags |= V4L2_BUF_FLAG_KEYFRAME;
    else
        dst_buf->flags |= V4L2_BUF_FLAG_PFRAME;

    v4l2_dbg(1, debug, &dev->v4l2_dev,
         "channel %d: encoded frame #%03d (%s%s, QP %d, %d bytes)\n",
         channel->mcu_channel_id,
         dst_buf->sequence,
         msg->is_idr ? "IDR, " : "",
         msg->slice_type == AL_ENC_SLICE_TYPE_I ? "I slice" :
         msg->slice_type == AL_ENC_SLICE_TYPE_P ? "P slice" : "unknown",
         msg->qp, partition->size);

err:
    if (src_buf)
        v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_DONE);

    if (dst_buf)
        v4l2_m2m_buf_done(dst_buf, state);
}

static int
allegro_handle_create_channel(struct allegro_dev *dev,
                  struct mcu_msg_create_channel_response *msg)
{
    struct allegro_channel *channel;
    struct create_encode_channel_param param;
    int err = 0;

    channel = allegro_find_channel_by_user_id(dev, msg->user_id);
    if (IS_ERR(channel)) {
        v4l2_warn(&dev->v4l2_dev,
              "received %s for unknown user %d\n",
              msg_type_name(msg->header.type),
              msg->user_id);
        return -EINVAL;
    }

    if (msg->error_code) {
        v4l2_err(&dev->v4l2_dev,
             "user %d: mcu failed to create channel: %s (%x)\n",
             channel->user_id,
             allegro_err_to_string(msg->error_code),
             msg->error_code);
        err = -EIO;
        goto out;
    }

    channel->mcu_channel_id = msg->channel_id;
    v4l2_dbg(1, debug, &dev->v4l2_dev,
         "user %d: channel has channel id %d\n",
         channel->user_id, channel->mcu_channel_id);

    err = allegro_unpack_encoder_config_blob(&param, msg,
                    channel->config_blob.vaddr);
    allegro_free_buffer(channel->dev, &channel->config_blob);
    if (err)
        return err;

    channel->num_ref_idx_l0 = param.num_ref_idx_l0;
    channel->num_ref_idx_l1 = param.num_ref_idx_l1;

    v4l2_dbg(1, debug, &dev->v4l2_dev,
         "channel %d: intermediate buffers: %d x %d bytes\n",
         channel->mcu_channel_id,
         msg->int_buffers_count, msg->int_buffers_size);
    err = allocate_intermediate_buffers(channel, msg->int_buffers_count,
                        msg->int_buffers_size);
    if (err) {
        v4l2_err(&dev->v4l2_dev,
             "channel %d: failed to allocate intermediate buffers\n",
             channel->mcu_channel_id);
        goto out;
    }
    err = allegro_mcu_push_buffer_intermediate(channel);
    if (err)
        goto out;

    v4l2_dbg(1, debug, &dev->v4l2_dev,
         "channel %d: reference buffers: %d x %d bytes\n",
         channel->mcu_channel_id,
         msg->rec_buffers_count, msg->rec_buffers_size);
    err = allocate_reference_buffers(channel, msg->rec_buffers_count,
                     msg->rec_buffers_size);
    if (err) {
        v4l2_err(&dev->v4l2_dev,
             "channel %d: failed to allocate reference buffers\n",
             channel->mcu_channel_id);
        goto out;
    }
    err = allegro_mcu_push_buffer_reference(channel);
    if (err)
        goto out;

out:
    channel->error = err;
    complete(&channel->completion);

    /* Handled successfully, error is passed via channel->error */
    return 0;
}

static int
allegro_handle_destroy_channel(struct allegro_dev *dev,
                   struct mcu_msg_destroy_channel_response *msg)
{
    struct allegro_channel *channel;

    channel = allegro_find_channel_by_channel_id(dev, msg->channel_id);
    if (IS_ERR(channel)) {
        v4l2_err(&dev->v4l2_dev,
             "received %s for unknown channel %d\n",
             msg_type_name(msg->header.type),
             msg->channel_id);
        return -EINVAL;
    }

    v4l2_dbg(2, debug, &dev->v4l2_dev,
         "user %d: vcu destroyed channel %d\n",
         channel->user_id, channel->mcu_channel_id);
    complete(&channel->completion);

    return 0;
}

static int
allegro_handle_encode_frame(struct allegro_dev *dev,
                struct mcu_msg_encode_frame_response *msg)
{
    struct allegro_channel *channel;

    channel = allegro_find_channel_by_channel_id(dev, msg->channel_id);
    if (IS_ERR(channel)) {
        v4l2_err(&dev->v4l2_dev,
             "received %s for unknown channel %d\n",
             msg_type_name(msg->header.type),
             msg->channel_id);
        return -EINVAL;
    }

    allegro_finish_encode_frame(channel, msg);

    return 0;
}

/**
 * allegro_channel_adjust() - Adjust channel parameters to current format
 * @channel: the channel to adjust
 *
 * Various parameters of a channel and their limits depend on the currently
 * set format. Adjust the parameters after a format change in one go.
 */
static void allegro_channel_adjust(struct allegro_channel *channel)
{
    struct allegro_dev *dev = channel->dev;
    struct allegro_q_data *q_data;
    u32 codec;
    struct v4l2_ctrl *ctrl;
    s64 min;
    s64 max;

    if (channel->inst_type == ALLEGRO_INST_DECODER)
        return;

    q_data = get_q_data(channel, V4L2_BUF_TYPE_VIDEO_CAPTURE);
    codec = q_data->fourcc;

    if (codec == V4L2_PIX_FMT_H264) {
        ctrl = channel->mpeg_video_h264_level;
        min = select_minimum_h264_level(q_data->width, q_data->height);
    } else {
        ctrl = channel->mpeg_video_hevc_level;
        min = select_minimum_hevc_level(q_data->width, q_data->height);
    }
    if (ctrl->minimum > min)
        v4l2_dbg(1, debug, &dev->v4l2_dev,
             "%s.minimum: %lld -> %lld\n",
             v4l2_ctrl_get_name(ctrl->id), ctrl->minimum, min);
    v4l2_ctrl_lock(ctrl);
    __v4l2_ctrl_modify_range(ctrl, min, ctrl->maximum,
                 ctrl->step, ctrl->default_value);
    v4l2_ctrl_unlock(ctrl);

    ctrl = channel->mpeg_video_bitrate;
    if (codec == V4L2_PIX_FMT_H264)
        max = h264_maximum_bitrate(v4l2_ctrl_g_ctrl(channel->mpeg_video_h264_level));
    else
        max = hevc_maximum_bitrate(v4l2_ctrl_g_ctrl(channel->mpeg_video_hevc_level));
    if (ctrl->maximum < max)
        v4l2_dbg(1, debug, &dev->v4l2_dev,
             "%s: maximum: %lld -> %lld\n",
             v4l2_ctrl_get_name(ctrl->id), ctrl->maximum, max);
    v4l2_ctrl_lock(ctrl);
    __v4l2_ctrl_modify_range(ctrl, ctrl->minimum, max,
                 ctrl->step, ctrl->default_value);
    v4l2_ctrl_unlock(ctrl);

    ctrl = channel->mpeg_video_bitrate_peak;
    v4l2_ctrl_lock(ctrl);
    __v4l2_ctrl_modify_range(ctrl, ctrl->minimum, max,
                 ctrl->step, ctrl->default_value);
    v4l2_ctrl_unlock(ctrl);

    v4l2_ctrl_activate(channel->mpeg_video_h264_profile,
               codec == V4L2_PIX_FMT_H264);
    v4l2_ctrl_activate(channel->mpeg_video_h264_level,
               codec == V4L2_PIX_FMT_H264);
    v4l2_ctrl_activate(channel->mpeg_video_h264_i_frame_qp,
               codec == V4L2_PIX_FMT_H264);
    v4l2_ctrl_activate(channel->mpeg_video_h264_max_qp,
               codec == V4L2_PIX_FMT_H264);
    v4l2_ctrl_activate(channel->mpeg_video_h264_min_qp,
               codec == V4L2_PIX_FMT_H264);
    v4l2_ctrl_activate(channel->mpeg_video_h264_p_frame_qp,
               codec == V4L2_PIX_FMT_H264);
    v4l2_ctrl_activate(channel->mpeg_video_h264_b_frame_qp,
               codec == V4L2_PIX_FMT_H264);

    v4l2_ctrl_activate(channel->mpeg_video_hevc_profile,
               codec == V4L2_PIX_FMT_HEVC);
    v4l2_ctrl_activate(channel->mpeg_video_hevc_level,
               codec == V4L2_PIX_FMT_HEVC);
    v4l2_ctrl_activate(channel->mpeg_video_hevc_tier,
               codec == V4L2_PIX_FMT_HEVC);
    v4l2_ctrl_activate(channel->mpeg_video_hevc_i_frame_qp,
               codec == V4L2_PIX_FMT_HEVC);
    v4l2_ctrl_activate(channel->mpeg_video_hevc_max_qp,
               codec == V4L2_PIX_FMT_HEVC);
    v4l2_ctrl_activate(channel->mpeg_video_hevc_min_qp,
               codec == V4L2_PIX_FMT_HEVC);
    v4l2_ctrl_activate(channel->mpeg_video_hevc_p_frame_qp,
               codec == V4L2_PIX_FMT_HEVC);
    v4l2_ctrl_activate(channel->mpeg_video_hevc_b_frame_qp,
               codec == V4L2_PIX_FMT_HEVC);

    if (codec == V4L2_PIX_FMT_H264)
        channel->log2_max_frame_num = LOG2_MAX_FRAME_NUM;
    channel->temporal_mvp_enable = true;
    channel->dbf_ovr_en = (codec == V4L2_PIX_FMT_H264);
    channel->enable_deblocking_filter_override = (codec == V4L2_PIX_FMT_HEVC);
    channel->enable_reordering = (codec == V4L2_PIX_FMT_HEVC);
    channel->enable_loop_filter_across_tiles = true;
    channel->enable_loop_filter_across_slices = true;

    if (codec == V4L2_PIX_FMT_H264) {
        channel->b_hrz_me_range = 8;
        channel->b_vrt_me_range = 8;
        channel->p_hrz_me_range = 16;
        channel->p_vrt_me_range = 16;
        channel->max_cu_size = ilog2(16);
        channel->min_cu_size = ilog2(8);
        channel->max_tu_size = ilog2(4);
        channel->min_tu_size = ilog2(4);
    } else {
        channel->b_hrz_me_range = 16;
        channel->b_vrt_me_range = 16;
        channel->p_hrz_me_range = 32;
        channel->p_vrt_me_range = 32;
        channel->max_cu_size = ilog2(32);
        channel->min_cu_size = ilog2(8);
        channel->max_tu_size = ilog2(32);
        channel->min_tu_size = ilog2(4);
    }
    channel->max_transfo_depth_intra = 1;
    channel->max_transfo_depth_inter = 1;
}

static void allegro_enc_message(struct allegro_dev *dev,
                   union mcu_msg_response *msg)
{
    switch (msg->header.type) {
    case MCU_MSG_TYPE_CREATE_CHANNEL:
        allegro_handle_create_channel(dev, &msg->create_channel);
        break;
    case MCU_MSG_TYPE_DESTROY_CHANNEL:
        allegro_handle_destroy_channel(dev, &msg->destroy_channel);
        break;
    case MCU_MSG_TYPE_ENCODE_FRAME:
        allegro_handle_encode_frame(dev, &msg->encode_frame);
        break;
    default:
        v4l2_warn(&dev->v4l2_dev,
              "%s: unknown message %s\n",
              __func__, msg_type_name(msg->header.type));
        break;
    }
}

static void allegro_enc_stop(struct allegro_channel *channel)
{
    struct allegro_dev *dev = channel->dev;
    unsigned long timeout;

    reinit_completion(&channel->completion);
    allegro_mcu_send_destroy_channel(dev, channel);
    timeout = wait_for_completion_timeout(&channel->completion,
                          msecs_to_jiffies(5000));
    if (timeout == 0)
        v4l2_warn(&dev->v4l2_dev,
              "channel %d: timeout while destroying\n",
              channel->mcu_channel_id);

    v4l2_ctrl_grab(channel->mpeg_video_h264_profile, false);
    v4l2_ctrl_grab(channel->mpeg_video_h264_level, false);
    v4l2_ctrl_grab(channel->mpeg_video_hevc_profile, false);
    v4l2_ctrl_grab(channel->mpeg_video_hevc_level, false);

    destroy_intermediate_buffers(channel);
    destroy_reference_buffers(channel);

    v4l2_ctrl_grab(channel->mpeg_video_h264_i_frame_qp, false);
    v4l2_ctrl_grab(channel->mpeg_video_h264_max_qp, false);
    v4l2_ctrl_grab(channel->mpeg_video_h264_min_qp, false);
    v4l2_ctrl_grab(channel->mpeg_video_h264_p_frame_qp, false);
    v4l2_ctrl_grab(channel->mpeg_video_h264_b_frame_qp, false);

    v4l2_ctrl_grab(channel->mpeg_video_hevc_tier, false);
    v4l2_ctrl_grab(channel->mpeg_video_hevc_i_frame_qp, false);
    v4l2_ctrl_grab(channel->mpeg_video_hevc_max_qp, false);
    v4l2_ctrl_grab(channel->mpeg_video_hevc_min_qp, false);
    v4l2_ctrl_grab(channel->mpeg_video_hevc_p_frame_qp, false);
    v4l2_ctrl_grab(channel->mpeg_video_hevc_b_frame_qp, false);

    v4l2_ctrl_grab(channel->mpeg_video_frame_rc_enable, false);
    v4l2_ctrl_grab(channel->mpeg_video_bitrate_mode, false);
    v4l2_ctrl_grab(channel->mpeg_video_bitrate, false);
    v4l2_ctrl_grab(channel->mpeg_video_bitrate_peak, false);
    v4l2_ctrl_grab(channel->mpeg_video_cpb_size, false);
    v4l2_ctrl_grab(channel->mpeg_video_gop_size, false);
}

static int allegro_enc_start(struct allegro_channel *channel)
{
    struct allegro_dev *dev = channel->dev;
    int timeout;

    v4l2_ctrl_grab(channel->mpeg_video_h264_profile, true);
    v4l2_ctrl_grab(channel->mpeg_video_h264_level, true);
    v4l2_ctrl_grab(channel->mpeg_video_h264_i_frame_qp, true);
    v4l2_ctrl_grab(channel->mpeg_video_h264_max_qp, true);
    v4l2_ctrl_grab(channel->mpeg_video_h264_min_qp, true);
    v4l2_ctrl_grab(channel->mpeg_video_h264_p_frame_qp, true);
    v4l2_ctrl_grab(channel->mpeg_video_h264_b_frame_qp, true);

    v4l2_ctrl_grab(channel->mpeg_video_hevc_profile, true);
    v4l2_ctrl_grab(channel->mpeg_video_hevc_level, true);
    v4l2_ctrl_grab(channel->mpeg_video_hevc_tier, true);
    v4l2_ctrl_grab(channel->mpeg_video_hevc_i_frame_qp, true);
    v4l2_ctrl_grab(channel->mpeg_video_hevc_max_qp, true);
    v4l2_ctrl_grab(channel->mpeg_video_hevc_min_qp, true);
    v4l2_ctrl_grab(channel->mpeg_video_hevc_p_frame_qp, true);
    v4l2_ctrl_grab(channel->mpeg_video_hevc_b_frame_qp, true);

    v4l2_ctrl_grab(channel->mpeg_video_frame_rc_enable, true);
    v4l2_ctrl_grab(channel->mpeg_video_bitrate_mode, true);
    v4l2_ctrl_grab(channel->mpeg_video_bitrate, true);
    v4l2_ctrl_grab(channel->mpeg_video_bitrate_peak, true);
    v4l2_ctrl_grab(channel->mpeg_video_cpb_size, true);
    v4l2_ctrl_grab(channel->mpeg_video_gop_size, true);

    reinit_completion(&channel->completion);
    allegro_mcu_send_create_channel(dev, channel);
    timeout = wait_for_completion_timeout(&channel->completion,
                          msecs_to_jiffies(5000));
    if (timeout == 0)
        channel->error = -ETIMEDOUT;
    if (channel->error)
        return channel->error;

    return 0;
}

static void allegro_enc_run(struct allegro_channel *channel)
{
    struct allegro_dev *dev = channel->dev;
    struct allegro_q_data *q_data;
    struct vb2_v4l2_buffer *src_buf;
    struct vb2_v4l2_buffer *dst_buf;
    dma_addr_t src_y;
    dma_addr_t src_uv;
    dma_addr_t dst_addr;
    unsigned long dst_size;
    u64 src_handle;
    u64 dst_handle;

    q_data = get_q_data(channel, V4L2_BUF_TYPE_VIDEO_OUTPUT);

    dst_buf = v4l2_m2m_dst_buf_remove(channel->fh.m2m_ctx);
    dst_addr = vb2_dma_contig_plane_dma_addr(&dst_buf->vb2_buf, 0);
    dst_size = vb2_plane_size(&dst_buf->vb2_buf, 0);
    dst_handle = allegro_put_buffer(channel, &channel->dst_shadow_list,
                    dst_buf);
    allegro_mcu_send_put_stream_buffer(dev, channel, dst_addr, dst_size,
                       dst_handle);

    src_buf = v4l2_m2m_src_buf_remove(channel->fh.m2m_ctx);
    src_buf->sequence = q_data->sequence++;
    src_y = vb2_dma_contig_plane_dma_addr(&src_buf->vb2_buf, 0);
    src_uv = src_y + (q_data->bytesperline * q_data->height);
    src_handle = allegro_put_buffer(channel, &channel->src_shadow_list,
                    src_buf);
    allegro_mcu_send_encode_frame(dev, channel, src_y, src_uv, src_handle);
}

extern struct v4l2_ctrl_ops allegro_ctrl_ops;

static void allegro_enc_init(struct allegro_channel *channel)
{
    struct v4l2_ctrl_handler *handler = &channel->ctrl_handler;
    struct allegro_q_data *q_data;
    u64 mask;
    unsigned int bitrate_max;
    unsigned int bitrate_def;
    unsigned int cpb_size_max;
    unsigned int cpb_size_def;

    q_data = get_q_data(channel, V4L2_BUF_TYPE_VIDEO_OUTPUT);

    channel->mpeg_video_h264_profile = v4l2_ctrl_new_std_menu(handler,
            &allegro_ctrl_ops,
            V4L2_CID_MPEG_VIDEO_H264_PROFILE,
            V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE, 0x0,
            V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE);
    mask = 1 << V4L2_MPEG_VIDEO_H264_LEVEL_1B;
    channel->mpeg_video_h264_level = v4l2_ctrl_new_std_menu(handler,
            &allegro_ctrl_ops,
            V4L2_CID_MPEG_VIDEO_H264_LEVEL,
            V4L2_MPEG_VIDEO_H264_LEVEL_5_1, mask,
            V4L2_MPEG_VIDEO_H264_LEVEL_5_1);
    channel->mpeg_video_h264_i_frame_qp =
        v4l2_ctrl_new_std(handler,
                  &allegro_ctrl_ops,
                  V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP,
                  0, 51, 1, 30);
    channel->mpeg_video_h264_max_qp =
        v4l2_ctrl_new_std(handler,
                  &allegro_ctrl_ops,
                  V4L2_CID_MPEG_VIDEO_H264_MAX_QP,
                  0, 51, 1, 51);
    channel->mpeg_video_h264_min_qp =
        v4l2_ctrl_new_std(handler,
                  &allegro_ctrl_ops,
                  V4L2_CID_MPEG_VIDEO_H264_MIN_QP,
                  0, 51, 1, 0);
    channel->mpeg_video_h264_p_frame_qp =
        v4l2_ctrl_new_std(handler,
                  &allegro_ctrl_ops,
                  V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP,
                  0, 51, 1, 30);
    channel->mpeg_video_h264_b_frame_qp =
        v4l2_ctrl_new_std(handler,
                  &allegro_ctrl_ops,
                  V4L2_CID_MPEG_VIDEO_H264_B_FRAME_QP,
                  0, 51, 1, 30);

    channel->mpeg_video_hevc_profile =
        v4l2_ctrl_new_std_menu(handler,
                       &allegro_ctrl_ops,
                       V4L2_CID_MPEG_VIDEO_HEVC_PROFILE,
                       V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN, 0x0,
                       V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN);
    channel->mpeg_video_hevc_level =
        v4l2_ctrl_new_std_menu(handler,
                       &allegro_ctrl_ops,
                       V4L2_CID_MPEG_VIDEO_HEVC_LEVEL,
                       V4L2_MPEG_VIDEO_HEVC_LEVEL_5_1, 0x0,
                       V4L2_MPEG_VIDEO_HEVC_LEVEL_5_1);
    channel->mpeg_video_hevc_tier =
        v4l2_ctrl_new_std_menu(handler,
                       &allegro_ctrl_ops,
                       V4L2_CID_MPEG_VIDEO_HEVC_TIER,
                       V4L2_MPEG_VIDEO_HEVC_TIER_HIGH, 0x0,
                       V4L2_MPEG_VIDEO_HEVC_TIER_MAIN);
    channel->mpeg_video_hevc_i_frame_qp =
        v4l2_ctrl_new_std(handler,
                  &allegro_ctrl_ops,
                  V4L2_CID_MPEG_VIDEO_HEVC_I_FRAME_QP,
                  0, 51, 1, 30);
    channel->mpeg_video_hevc_max_qp =
        v4l2_ctrl_new_std(handler,
                  &allegro_ctrl_ops,
                  V4L2_CID_MPEG_VIDEO_HEVC_MAX_QP,
                  0, 51, 1, 51);
    channel->mpeg_video_hevc_min_qp =
        v4l2_ctrl_new_std(handler,
                  &allegro_ctrl_ops,
                  V4L2_CID_MPEG_VIDEO_HEVC_MIN_QP,
                  0, 51, 1, 0);
    channel->mpeg_video_hevc_p_frame_qp =
        v4l2_ctrl_new_std(handler,
                  &allegro_ctrl_ops,
                  V4L2_CID_MPEG_VIDEO_HEVC_P_FRAME_QP,
                  0, 51, 1, 30);
    channel->mpeg_video_hevc_b_frame_qp =
        v4l2_ctrl_new_std(handler,
                  &allegro_ctrl_ops,
                  V4L2_CID_MPEG_VIDEO_HEVC_B_FRAME_QP,
                  0, 51, 1, 30);

    channel->mpeg_video_frame_rc_enable =
        v4l2_ctrl_new_std(handler,
                  &allegro_ctrl_ops,
                  V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE,
                  false, 0x1,
                  true, false);
    channel->mpeg_video_bitrate_mode = v4l2_ctrl_new_std_menu(handler,
            &allegro_ctrl_ops,
            V4L2_CID_MPEG_VIDEO_BITRATE_MODE,
            V4L2_MPEG_VIDEO_BITRATE_MODE_CBR, 0,
            V4L2_MPEG_VIDEO_BITRATE_MODE_CBR);

    if (q_data->fourcc == V4L2_PIX_FMT_H264) {
        bitrate_max = h264_maximum_bitrate(V4L2_MPEG_VIDEO_H264_LEVEL_5_1);
        bitrate_def = h264_maximum_bitrate(V4L2_MPEG_VIDEO_H264_LEVEL_5_1);
        cpb_size_max = h264_maximum_cpb_size(V4L2_MPEG_VIDEO_H264_LEVEL_5_1);
        cpb_size_def = h264_maximum_cpb_size(V4L2_MPEG_VIDEO_H264_LEVEL_5_1);
    } else {
        bitrate_max = hevc_maximum_bitrate(V4L2_MPEG_VIDEO_HEVC_LEVEL_5_1);
        bitrate_def = hevc_maximum_bitrate(V4L2_MPEG_VIDEO_HEVC_LEVEL_5_1);
        cpb_size_max = hevc_maximum_cpb_size(V4L2_MPEG_VIDEO_HEVC_LEVEL_5_1);
        cpb_size_def = hevc_maximum_cpb_size(V4L2_MPEG_VIDEO_HEVC_LEVEL_5_1);
    }
    channel->mpeg_video_bitrate = v4l2_ctrl_new_std(handler,
            &allegro_ctrl_ops,
            V4L2_CID_MPEG_VIDEO_BITRATE,
            0, bitrate_max, 1, bitrate_def);
    channel->mpeg_video_bitrate_peak = v4l2_ctrl_new_std(handler,
            &allegro_ctrl_ops,
            V4L2_CID_MPEG_VIDEO_BITRATE_PEAK,
            0, bitrate_max, 1, bitrate_def);
    channel->mpeg_video_cpb_size = v4l2_ctrl_new_std(handler,
            &allegro_ctrl_ops,
            V4L2_CID_MPEG_VIDEO_H264_CPB_SIZE,
            0, cpb_size_max, 1, cpb_size_def);
    channel->mpeg_video_gop_size = v4l2_ctrl_new_std(handler,
            &allegro_ctrl_ops,
            V4L2_CID_MPEG_VIDEO_GOP_SIZE,
            0, ALLEGRO_GOP_SIZE_MAX,
            1, ALLEGRO_GOP_SIZE_DEFAULT);
    v4l2_ctrl_new_std(handler,
              &allegro_ctrl_ops,
              V4L2_CID_MIN_BUFFERS_FOR_OUTPUT,
              1, 32,
              1, 1);

    v4l2_ctrl_cluster(3, &channel->mpeg_video_bitrate_mode);

    allegro_channel_adjust(channel);
}

const struct allegro_ops allegro_enc_ops = {
    .init = allegro_enc_init,
    .start = allegro_enc_start,
    .stop = allegro_enc_stop,
    .message = allegro_enc_message,
    .run = allegro_enc_run,
};