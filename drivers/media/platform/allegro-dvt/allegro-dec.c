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
#include <linux/v4l2-controls.h>
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



static int blk_count(u32 width, u32 height, u8 size)
{
#define ALLEGRO_BLK_16x16 4
#define ALLEGRO_BLK_32x32 5
#define ALLEGRO_BLK_64x64 6

    if (size < ALLEGRO_BLK_16x16)
        return -EINVAL;
    if (size > ALLEGRO_BLK_64x64)
        return -EINVAL;

    return ((width + (1 << size) - 1) << size) +
            ((height + (1 << size) - 1) << size);
}

/*
 * Mailbox interface to send messages to the MCU.
 */

static u8 h264_slice_type_to_mcu_slice_type(int type)
{
    switch (type) {
    case V4L2_H264_SLICE_TYPE_P:
        return AL_DEC_SLICE_P;
    case V4L2_H264_SLICE_TYPE_B:
        return AL_DEC_SLICE_B;
    default:
    case V4L2_H264_SLICE_TYPE_I:
        return AL_DEC_SLICE_I;
    case V4L2_H264_SLICE_TYPE_SP:
        return AL_DEC_SLICE_SP;
    case V4L2_H264_SLICE_TYPE_SI:
        return AL_DEC_SLICE_SI;
    }
}

static unsigned long allegro_next_picture_id(struct allegro_channel *channel)
{
    if (channel->decoder_picture_ids == ~0UL ||
        ffz(channel->decoder_picture_ids) >= 16)
        return -EBUSY;

    return ffz(channel->decoder_picture_ids);
}

static unsigned long allegro_next_mv_id(struct allegro_channel *channel)
{
    if (channel->decoder_mv_ids == ~0UL ||
        ffz(channel->decoder_mv_ids) >= 16)
        return -EBUSY;

    return ffz(channel->decoder_mv_ids);
}


static int fill_create_channel_param(struct allegro_channel *channel,
                     struct create_decode_channel_param *param)
{
#define AL_DEC_AVC_LOG2_MAX_CU_SIZE 4
#define AL_DEC_HEVC_LOG2_MAX_CU_SIZE 6

    struct allegro_q_data *q_data;

    q_data = get_q_data(channel, V4L2_BUF_TYPE_VIDEO_OUTPUT);

    param->width = 1280;//q_data->width;
    param->height = 720;//q_data->height;
    param->log2_max_cu_size = AL_DEC_AVC_LOG2_MAX_CU_SIZE;
    param->framerate = 24000;//DIV_ROUND_UP(channel->framerate.numerator,
                    //channel->framerate.denominator);
    param->clk_ratio = channel->framerate.denominator == 1001 ? 1001 : 1000;
    param->max_latency = 5;
    param->num_core = 0;
    param->ddr_width = AL_DEC_DDR_WIDTH_64;
    param->low_lat = 0;
    param->parallel_wpp = 0;
    param->disable_cache = 0;
    param->frame_buffer_compression = 0;
    param->use_early_callback = 0;
    param->fb_storage_mode = AL_DEC_FB_RASTER;
    param->codec = q_data->fourcc;
    param->max_slices = 2880;
    param->dec_unit = AL_DEC_UNIT_SLICE;
    param->buffer_output_mode = AL_DEC_OUTPUT_INTERNAL;

    return 0;
}

static int allegro_mcu_send_create_channel(struct allegro_dev *dev,
                       struct allegro_channel *channel)
{
    struct mcu_msg_create_channel msg;
    struct allegro_buffer *blob = &channel->config_blob;
    struct create_decode_channel_param param;
    size_t size;

    memset(&param, 0, sizeof(param));
    param.version = dev->fw_info->mailbox_version;
    fill_create_channel_param(channel, &param);
    allegro_alloc_buffer(dev, blob, sizeof(struct create_decode_channel_param));
    size = allegro_pack_decoder_config_blob(blob->vaddr, &param);

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

static void _allegro_write_ref_list(struct allegro_channel *channel,
                                const struct v4l2_h264_reference * ref_list,
                                u8 num_ref, u8 *pic_ids)
{
    struct vb2_queue *cap_q;
    const struct v4l2_ctrl_h264_decode_params *decode;
    unsigned int i;

    decode = channel->h264.decode_params;
    cap_q = v4l2_m2m_get_vq(channel->fh.m2m_ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE);

    for (i = 0; i < num_ref; i++) {
        const struct v4l2_h264_dpb_entry *dpb;
        int buf_idx;
        u8 dpb_idx;

        pic_ids[i] = 0;

        dpb_idx = ref_list[i].index;
        dpb = &decode->dpb[dpb_idx];

        if (!(dpb->flags & V4L2_H264_DPB_ENTRY_FLAG_ACTIVE))
            continue;

        buf_idx = vb2_find_timestamp(cap_q, dpb->reference_ts, 0);
        if (buf_idx < 0)
            continue;

        pic_ids[i] = dpb->pic_num;
    }
}

static void allegro_write_ref_list0(struct allegro_channel *channel,
                                struct decode_slice_param *param)
{
    const struct v4l2_ctrl_h264_slice_params *slice;

    slice = channel->h264.slice_params;

    _allegro_write_ref_list(channel,
                   slice->ref_pic_list0,
                   slice->num_ref_idx_l0_active_minus1 + 1,
                   param->pic_id_l0);
}


static void allegro_write_ref_list1(struct allegro_channel *channel,
                                struct decode_slice_param *param)
{
    const struct v4l2_ctrl_h264_slice_params *slice;

    slice = channel->h264.slice_params;

    _allegro_write_ref_list(channel,
                   slice->ref_pic_list1,
                   slice->num_ref_idx_l1_active_minus1 + 1,
                   param->pic_id_l1);
}

static void allegro_write_coloc_pic_id(struct allegro_channel *channel,
                                struct decode_slice_param *param)
{
    const struct v4l2_ctrl_h264_decode_params *decode;
    const struct v4l2_ctrl_h264_slice_params *slice;
    const struct v4l2_h264_dpb_entry *dpb;
    u8 dpb_idx;

    decode = channel->h264.decode_params;
    slice = channel->h264.slice_params;

    dpb_idx = slice->ref_pic_list1[0].index;
    dpb = &decode->dpb[dpb_idx];

    param->coloc_pic_id = dpb->pic_num;
}

static int
_allegro_fill_slice_param(struct allegro_channel *channel,
                   struct decode_slice_param *param,
                   unsigned long size)
{
    struct allegro_q_data *q_data =
                get_q_data(channel, V4L2_BUF_TYPE_VIDEO_OUTPUT);
    const struct v4l2_ctrl_h264_slice_params *slice =
                                channel->h264.slice_params;
    const struct v4l2_ctrl_h264_pps *pps = channel->h264.pps;
    u32 width_mbs = q_data->width / SIZE_MACROBLOCK;
    u32 height_map = q_data->height / SIZE_MACROBLOCK;

    memset(param, 0, sizeof(*param));
    param->cabac_init_idc = slice->cabac_init_idc;
    param->direct_spatial =
            slice->flags & V4L2_H264_SLICE_FLAG_DIRECT_SPATIAL_MV_PRED;
    param->cb_qp_offs = pps->chroma_qp_index_offset;
    param->cr_qp_offs = pps->second_chroma_qp_index_offset;
    param->slice_qp = pps->pic_init_qp_minus26 + slice->slice_qp_delta + 26;
    param->tc_offs_div2 = slice->slice_alpha_c0_offset_div2;
    param->beta_offs_div2 = slice->slice_beta_offset_div2;
    param->disable_loop_filter = slice->disable_deblocking_filter_idc & 0x1;
    param->x_slice_loop_filter = slice->disable_deblocking_filter_idc & 0x2;
    param->slice_id = channel->slice_id;

    param->first_lcu = slice->first_mb_in_slice;
    param->first_lcu_slice_segment = slice->first_mb_in_slice;
    param->first_lcu_slice = slice->first_mb_in_slice;

    param->num_ref_idx_l0_minus1 = slice->num_ref_idx_l0_active_minus1;
    param->num_ref_idx_l1_minus1 = slice->num_ref_idx_l1_active_minus1;

    param->num_lcu = width_mbs * height_map;

    param->weigthed_pred = param->weigthed_bi_pred = 0;

    if (slice->slice_type == V4L2_H264_SLICE_TYPE_P)
        param->weigthed_pred = (pps->flags & V4L2_H264_PPS_FLAG_WEIGHTED_PRED);
    else if (slice->slice_type == V4L2_H264_SLICE_TYPE_B &&
            pps->weighted_bipred_idc == 1)
        param->weigthed_pred = 1; /* WP_EXPLICIT */
    else if (slice->slice_type == V4L2_H264_SLICE_TYPE_B &&
            pps->weighted_bipred_idc == 2)
        param->weigthed_bi_pred = 1; /* WP_IMPLICIT */

    param->slice_hdrlen = 24;
    if (slice->header_bit_size % 8)
        param->slice_hdrlen = 16 + (slice->header_bit_size % 8);

    param->str_avail_size = size;

    param->next_slice_segment = (param->slice_id * 450) + 450;
    param->parsing_id = param->slice_id;
    param->valid_conceal = q_data->sequence ? 1 : 0;

    param->slice_type = h264_slice_type_to_mcu_slice_type(slice->slice_type);
    param->dependent_slice = false;
    //param->next_is_dependent = 1;

    param->last_slice =
            (param->next_slice_segment == param->num_lcu) ? 1 : 0;
    param->next_is_dependent =
            (param->next_slice_segment == param->num_lcu) ? 0 : 1;
    //param->next_slice_segment = param->num_lcu;

    allegro_write_coloc_pic_id(channel, param);

    if ((slice->slice_type == V4L2_H264_SLICE_TYPE_P) ||
        (slice->slice_type == V4L2_H264_SLICE_TYPE_SP) ||
        (slice->slice_type == V4L2_H264_SLICE_TYPE_B))
        allegro_write_ref_list0(channel, param);

    if (slice->slice_type == V4L2_H264_SLICE_TYPE_B)
        allegro_write_ref_list1(channel, param);

    return 0;
}

static void
allegro_fill_slice_param(struct allegro_channel *channel,
                        struct decode_slice_param *sp,
                        unsigned long size)
{
    const struct v4l2_ctrl_h264_slice_params *slice =
                                channel->h264.slice_params;
    struct allegro_q_data *q_data =
                        get_q_data(channel, V4L2_BUF_TYPE_VIDEO_OUTPUT);
    struct decode_slice_param *param;

    if (channel->fh.m2m_ctx->new_frame) {
        q_data->sequence++;
        channel->slice_id = 0;
    }
    else {
        channel->slice_id += 1;
    }

    param = &sp[channel->slice_id];

    _allegro_fill_slice_param(channel, param, size);
}


static int allegro_mcu_send_decode_frame(struct allegro_dev *dev,
                     struct allegro_channel *channel,
                     u8 frm_buf_id, u8 mv_buf_id,
                     dma_addr_t stream, unsigned long size,
                     dma_addr_t dst_y, dma_addr_t dst_uv,
                     dma_addr_t comp_data, dma_addr_t comp_map,
                     dma_addr_t list_ref, dma_addr_t poc, dma_addr_t mv,
                     dma_addr_t slice_p)
{
    struct allegro_q_data *q_data =
                        get_q_data(channel, V4L2_BUF_TYPE_VIDEO_OUTPUT);
    const struct v4l2_ctrl_h264_decode_params *decode =
                                channel->h264.decode_params;
    const struct v4l2_ctrl_h264_slice_params *slice =
                                channel->h264.slice_params;
    const struct v4l2_ctrl_h264_pps *pps = channel->h264.pps;
    const struct v4l2_ctrl_h264_sps *sps = channel->h264.sps;
    struct mcu_msg_decode_frame msg;
    u32 width_mbs, height_map;
    int codec;

    width_mbs = q_data->width / SIZE_MACROBLOCK;
    height_map = q_data->height / SIZE_MACROBLOCK;
    codec = q_data->fourcc;

    memset(&msg, 0, sizeof(msg));
    msg.header.type = MCU_MSG_TYPE_DECODE_SLICE;
    msg.header.version = dev->fw_info->mailbox_version;
    msg.header.devtype = channel->inst_type;
    msg.channel_id = channel->mcu_channel_id;
    msg.num_tile_columns = 1;
    msg.num_tile_rows = 1;

    msg.log2_max_tu_size =
            (pps->flags & V4L2_H264_PPS_FLAG_TRANSFORM_8X8_MODE) ? 3 : 2;
    msg.log2_max_cu_size = 4;
    msg.codec = codec;

    msg.pic_width = sps->pic_width_in_mbs_minus1 << 1;
    msg.pic_height = sps->pic_height_in_map_units_minus1 << 1;

    msg.entropy_mode = (pps->flags & V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE) ?
                    ALLEGRO_ENTROPY_MODE_CABAC : ALLEGRO_ENTROPY_MODE_CAVLC;
    msg.bit_depth_luma = sps->bit_depth_luma_minus8 + 8;
    msg.bit_depth_chroma = sps->bit_depth_chroma_minus8 + 8;
    msg.chroma_mode = sps->chroma_format_idc;

    msg.option_flags |= AL_DEC_OPT_INTRA_PCM;
    if (sps->flags & V4L2_H264_SPS_FLAG_QPPRIME_Y_ZERO_TRANSFORM_BYPASS)
        msg.option_flags |= AL_DEC_OPT_LOSSLESS;
    if (sps->flags & V4L2_H264_SPS_FLAG_DIRECT_8X8_INFERENCE)
        msg.option_flags |= AL_DEC_OPT_DIRECT8x8INFER;
    if (pps->flags & V4L2_H264_PPS_FLAG_SCALING_MATRIX_PRESENT) {
        msg.option_flags |= AL_DEC_OPT_ENABLE_SCL_LIST;
        msg.option_flags |= AL_DEC_OPT_LOAD_SCL_LIST;
    }
    if (pps->flags & V4L2_H264_PPS_FLAG_CONSTRAINED_INTRA_PRED)
        msg.option_flags |= AL_DEC_OPT_CONSTRAINED_INTRA_PRED;
    if (slice->slice_type == V4L2_H264_SLICE_TYPE_I)
        msg.option_flags |= AL_DEC_OPT_INTRA_ONLY;

    msg.lcu_pic_width = sps->pic_width_in_mbs_minus1 + 1;
    msg.lcu_pic_height = sps->pic_height_in_map_units_minus1 + 1;

    msg.current_poc = min_t(__s32, decode->top_field_order_cnt,
                decode->bottom_field_order_cnt);
    msg.frm_buf_id = frm_buf_id;
    msg.mv_buf_id = mv_buf_id;

    msg.pic_struct = AL_DEC_PS_FRM;
    msg.frame_num = q_data->sequence;
    msg.user_param[0] = 0;
    msg.user_param[1] = 0;

    msg.qp_prime_ts_min = 0;
    msg.num_ladf_intervals = 35;
    msg.ladf_lowest_interval_qp_offs = 69;
    msg.coloc_pic_id = 0xff;

    msg.addr_comp_data = to_codec_addr(dev, comp_data);
    msg.addr_comp_map = to_codec_addr(dev, comp_map);
    msg.addr_list_ref = to_codec_addr(dev, list_ref);
    msg.addr_stream = to_codec_addr(dev, stream);
    msg.stream_size = size;
    msg.addr_rec_y = to_codec_addr(dev, dst_y);
    msg.addr_rec_c1 = to_codec_addr(dev, dst_uv);
    msg.pitch = q_data->width;
    msg.addr_poc = to_codec_addr(dev, poc);
    msg.addr_mv = to_codec_addr(dev, mv);

    msg.blob_size = sizeof(struct decode_slice_param);
    msg.blob_mcu_addr = to_mcu_addr(dev, slice_p);

    allegro_mbox_send(dev->mbox_command, &msg);

    return 0;
}

static int allegro_mcu_send_search_sc(struct allegro_dev *dev,
                       struct allegro_channel *channel,
                       dma_addr_t stream, unsigned long ssize,
                       unsigned long offs, unsigned long avail,
                       dma_addr_t output, unsigned long osize)
{
    struct allegro_q_data *q_data;
    struct mcu_msg_search_sc msg;

    memset(&msg, 0, sizeof(msg));

    q_data = get_q_data(channel, V4L2_BUF_TYPE_VIDEO_OUTPUT);

    msg.header.type = MCU_MSG_TYPE_SEARCH_START_CODE;
    msg.header.version = dev->fw_info->mailbox_version;
    msg.header.devtype = channel->inst_type;

    msg.channel_id = channel->mcu_channel_id;
    msg.codec = q_data->fourcc;
    msg.max_size = osize >> 3;

    msg.addr_stream = to_codec_addr(dev, stream);
    msg.stream_size = ssize;
    msg.offset = offs;
    msg.avail_size = avail;
    msg.addr_output = to_codec_addr(dev, output);

    allegro_mbox_send(dev->mbox_command, &msg);

    return 0;
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

static struct allegro_buffer *
allegro_find_buffer_by_id(struct list_head *list, unsigned int id)
{
    struct allegro_buffer *buffer;

    list_for_each_entry(buffer, list, head) {
        if (buffer->id == id)
            return buffer;
    }

    return ERR_PTR(-EINVAL);
}

static void destroy_picture_buffers(struct allegro_channel *channel)
{
    destroy_buffers_internal(channel,
                    &channel->buffers_comp_data);
    destroy_buffers_internal(channel,
                    &channel->buffers_comp_map);
    destroy_buffers_internal(channel,
                    &channel->buffers_list_ref);
    destroy_buffers_internal(channel,
                    &channel->buffers_slice_p);
}

static void destroy_mv_buffers(struct allegro_channel *channel)
{
    destroy_buffers_internal(channel,
                    &channel->buffers_poc);
    destroy_buffers_internal(channel,
                    &channel->buffers_mv);
}

static int allocate_picture_buffers(struct allegro_channel *channel, size_t n)
{
#define ALLEGRO_LCU_INFO_SIZE     16   /* LCU compressed size + LCU offset */
#define ALLEGRO_AVC_LCU_CMP_SIZE  1120 /* for chroma 4:2:0 */
#define ALLEGRO_HEVC_LCU_CMP_SIZE 1088 /* for chroma 4:2:0 */

    int err;
    size_t size, blks;
    u32 w = 0, h = 0;

    blks = blk_count(w, h, ALLEGRO_BLK_16x16);

    size = blks * ALLEGRO_AVC_LCU_CMP_SIZE;
    size = 4032000;
    err = allocate_buffers_internal(channel,
                     &channel->buffers_comp_data,
                     n, size);
    if (err)
        return err;

    size = blks * ALLEGRO_LCU_INFO_SIZE;
    size = 57600;
    err = allocate_buffers_internal(channel,
                     &channel->buffers_comp_map,
                     n, size);
    if (err)
        return err;

    size = blks * ALLEGRO_LCU_INFO_SIZE * 2;
    size = 384;
    err = allocate_buffers_internal(channel,
                     &channel->buffers_list_ref,
                     n, size);
    if (err)
        return err;

    size = 6451200;//sizeof(struct decode_slice_param);
    err = allocate_buffers_internal(channel,
                     &channel->buffers_slice_p,
                     n, size);
    if (err)
        return err;

    return 0;
}
// PoolSclLst 12288
// PoolCompData 4032000
// PoolCompMap 57600
// PoolWP 737280
// PoolListRefAddr 384
// PoolVirtRefAddr 0
// POC 96
// MV 230400

static int allocate_mv_buffers(
    struct allegro_channel *channel, size_t n)
{
    int err;
    size_t size = 0;


    size = 16 * ALLEGRO_AVC_LCU_CMP_SIZE;
    size = 96;
    err = allocate_buffers_internal(channel,
                     &channel->buffers_poc,
                     n, size);
    if (err)
        return err;

    size = 16 * ALLEGRO_AVC_LCU_CMP_SIZE;
    size = 230400;
    err = allocate_buffers_internal(channel,
                     &channel->buffers_mv,
                     n, size);

    return err;
}

static int
allegro_handle_create_channel(struct allegro_dev *dev,
                  struct mcu_msg_create_channel_response *msg)
{
    struct allegro_channel *channel;
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


    allegro_free_buffer(channel->dev, &channel->config_blob);

    err = allocate_picture_buffers(channel, 16);
    if (err) {
        v4l2_err(&dev->v4l2_dev,
             "channel %d: failed to allocate picture buffers\n",
             channel->mcu_channel_id);
        goto out;
    }

    err = allocate_mv_buffers(channel, 16);
    if (err) {
        v4l2_err(&dev->v4l2_dev,
             "channel %d: failed to allocate mv buffers\n",
             channel->mcu_channel_id);
        goto out;
    }

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

static int allegro_finish_parse_frame(struct allegro_channel *channel,
                u32 frame_id, u32 parsing_id)
{
    struct allegro_dev *dev = channel->dev;
    enum vb2_buffer_state state = VB2_BUF_STATE_ERROR;
    struct allegro_q_data *q_data;
    struct vb2_v4l2_buffer *src_buf;
    struct vb2_v4l2_buffer *dst_buf;

    v4l2_dbg(2, debug, &dev->v4l2_dev,
         "channel %d: decode-frame PARSING - "
         "frame-id %d parse-id %d\n", channel->mcu_channel_id,
         frame_id, parsing_id);

    q_data = get_q_data(channel, V4L2_BUF_TYPE_VIDEO_CAPTURE);

    src_buf = v4l2_m2m_next_src_buf(channel->fh.m2m_ctx);
    dst_buf = v4l2_m2m_next_dst_buf(channel->fh.m2m_ctx);

    if (!src_buf || !dst_buf)
        goto err;

    if (!(src_buf->flags & V4L2_BUF_FLAG_M2M_HOLD_CAPTURE_BUF))
        return 0;

    state = VB2_BUF_STATE_DONE;

    v4l2_m2m_buf_copy_metadata(src_buf, dst_buf, false);

err:
    v4l2_m2m_buf_done_and_job_finish(dev->m2m_dev, channel->fh.m2m_ctx,
                     state);

    return 0;
}

void allegro_channel_eos_event(struct allegro_channel *channel);

static int allegro_finish_decode_frame(struct allegro_channel *channel,
                struct mcu_msg_decode_frame_response *msg)
{
    struct allegro_dev *dev = channel->dev;
    enum vb2_buffer_state state = VB2_BUF_STATE_ERROR;
    struct allegro_q_data *q_data;
    struct vb2_v4l2_buffer *src_buf;
    struct vb2_v4l2_buffer *dst_buf;

    v4l2_dbg(2, debug, &dev->v4l2_dev,
         "channel %d: decode-frame DECODE - "
         "bufid %d mvid %d state 0x%x\n",
         msg->channel_id, msg->d.frm_buf_id,
         msg->d.mv_buf_id, msg->d.pic_state);

    q_data = get_q_data(channel, V4L2_BUF_TYPE_VIDEO_CAPTURE);

    src_buf = v4l2_m2m_next_src_buf(channel->fh.m2m_ctx);
    dst_buf = v4l2_m2m_next_dst_buf(channel->fh.m2m_ctx);

    if (!src_buf || !dst_buf)
        goto err;

    if (v4l2_m2m_is_last_draining_src_buf(channel->fh.m2m_ctx, src_buf)) {
        dst_buf->flags |= V4L2_BUF_FLAG_LAST;
        allegro_channel_eos_event(channel);
        v4l2_m2m_mark_stopped(channel->fh.m2m_ctx);
    }

    dst_buf->sequence = q_data->sequence++;

    vb2_set_plane_payload(&dst_buf->vb2_buf, 0, q_data->sizeimage);

    clear_bit(msg->d.frm_buf_id, &channel->decoder_picture_ids);
    clear_bit(msg->d.mv_buf_id, &channel->decoder_mv_ids);

    if (msg->d.pic_state & AL_DEC_PIC_STATE_CMD_INVALID) {
        v4l2_err(&dev->v4l2_dev,
             "channel %d: failed to decode frame (err %x)\n",
             channel->mcu_channel_id, msg->d.pic_state);
        goto err;
    }

    state = VB2_BUF_STATE_DONE;

    v4l2_m2m_buf_copy_metadata(src_buf, dst_buf, false);

err:
    v4l2_m2m_buf_done_and_job_finish(dev->m2m_dev, channel->fh.m2m_ctx,
                     state);

    return 0;
}

static int
allegro_handle_decode_frame(struct allegro_dev *dev,
                struct mcu_msg_decode_frame_response *msg)
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

    if (msg->response_type == AL_DEC_RESPONSE_END_PARSING)
        return allegro_finish_parse_frame(channel,
                        msg->p.frame_id, msg->p.parsing_id);
    else if (msg->response_type == AL_DEC_RESPONSE_END_DECODING)
        return allegro_finish_decode_frame(channel, msg);

    return -EINVAL;
}

static void allegro_dec_message(struct allegro_dev *dev,
                   union mcu_msg_response *msg)
{
    switch (msg->header.type) {
    case MCU_MSG_TYPE_CREATE_CHANNEL:
        allegro_handle_create_channel(dev, &msg->create_channel);
        break;
    case MCU_MSG_TYPE_DESTROY_CHANNEL:
        allegro_handle_destroy_channel(dev, &msg->destroy_channel);
        break;
    case MCU_MSG_TYPE_DECODE_FRAME:
    case MCU_MSG_TYPE_DECODE_SLICE:
        allegro_handle_decode_frame(dev, &msg->decode_frame);
        break;
    default:
        v4l2_warn(&dev->v4l2_dev,
              "%s: unknown message %s\n",
              __func__, msg_type_name(msg->header.type));
        break;
    }
}

static void allegro_dec_stop(struct allegro_channel *channel)
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

    destroy_picture_buffers(channel);
    destroy_mv_buffers(channel);
}

static int allegro_dec_start(struct allegro_channel *channel)
{
    struct allegro_dev *dev = channel->dev;
    int timeout;

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

static int
allegro_prepare_next_buffers(struct allegro_channel *channel,
                        struct allegro_picture_buffers *bufs)
{
    struct allegro_buffer *slice_p;
    struct allegro_buffer *comp_data;
    struct allegro_buffer *comp_map;
    struct allegro_buffer *list_ref;
    struct allegro_buffer *poc;
    struct allegro_buffer *mv;
    u8 frm_buf_id = channel->frm_buf_id;
    u8 mv_buf_id = channel->mv_buf_id;

    if (channel->fh.m2m_ctx->new_frame) {
        //frm_buf_id = allegro_next_picture_id(channel);
        frm_buf_id = (frm_buf_id + 1) % 16;
        if (frm_buf_id < 0) {
            v4l2_err(&channel->dev->v4l2_dev,
                 "no free frm buffers available\n");
            return -ENOMEM;
        }

        //mv_buf_id = allegro_next_mv_id(channel);
        mv_buf_id = (mv_buf_id + 1) % 16;
        if (mv_buf_id < 0) {
            v4l2_err(&channel->dev->v4l2_dev,
                 "no free mv buffers available\n");
            return -ENOMEM;
        }

        set_bit(frm_buf_id, &channel->decoder_picture_ids);
        set_bit(mv_buf_id, &channel->decoder_mv_ids);
    }

    slice_p = allegro_find_buffer_by_id(
                    &channel->buffers_slice_p, frm_buf_id);
    comp_data = allegro_find_buffer_by_id(
                    &channel->buffers_comp_data, frm_buf_id);
    comp_map = allegro_find_buffer_by_id(
                    &channel->buffers_comp_map, frm_buf_id);
    list_ref = allegro_find_buffer_by_id(
                    &channel->buffers_list_ref, frm_buf_id);
    poc = allegro_find_buffer_by_id(
                    &channel->buffers_poc, mv_buf_id);
    mv = allegro_find_buffer_by_id(
                    &channel->buffers_mv, mv_buf_id);

    if (IS_ERR(comp_data) || IS_ERR(comp_map) ||
        IS_ERR(list_ref) || IS_ERR(poc) || IS_ERR(mv)) {
        v4l2_err(&channel->dev->v4l2_dev,
              "could not find decoder buffer(s) for frm %d mv %d\n",
              frm_buf_id, mv_buf_id);
        return -ENOMEM;
    }

    bufs->slice_p = slice_p;
    bufs->comp_data = comp_data;
    bufs->comp_map = comp_map;
    bufs->list_ref = list_ref;
    bufs->poc = poc;
    bufs->mv = mv;

    channel->frm_buf_id = frm_buf_id;
    channel->mv_buf_id = mv_buf_id;

    return 0;
}

static void _allegro_dec_run(struct allegro_channel *channel)
{
    struct allegro_dev *dev = channel->dev;
    struct allegro_q_data *q_data;
    struct vb2_v4l2_buffer *src_buf;
    struct vb2_v4l2_buffer *dst_buf;
    dma_addr_t src_addr;
    void *src_vaddr;
    void *dst_vaddr;
    unsigned long src_size;
    unsigned long dst_size;
    dma_addr_t dst_y;
    dma_addr_t dst_uv;
    struct allegro_picture_buffers bufs;
    struct decode_slice_param *sp;
    int ret;

    q_data = get_q_data(channel, V4L2_BUF_TYPE_VIDEO_CAPTURE);

    src_buf = v4l2_m2m_next_src_buf(channel->fh.m2m_ctx);
    dst_buf = v4l2_m2m_next_dst_buf(channel->fh.m2m_ctx);

    src_buf->sequence = q_data->sequence++;
    src_addr = vb2_dma_contig_plane_dma_addr(&src_buf->vb2_buf, 0);
    // src_vaddr = vb2_plane_vaddr(&src_buf->vb2_buf, 0);
    src_size = vb2_get_plane_payload(&src_buf->vb2_buf, 0);


    dst_y = vb2_dma_contig_plane_dma_addr(&dst_buf->vb2_buf, 0);
    dst_uv = dst_y + (q_data->bytesperline * q_data->height);
    dst_uv = dst_y + (1280 * 720);
    //dst_vaddr = vb2_plane_vaddr(&dst_buf->vb2_buf, 0);
    //dst_size = vb2_get_plane_payload(&dst_buf->vb2_buf, 0);
    //memset(dst_vaddr, 0, dst_size);

    ret = allegro_prepare_next_buffers(channel, &bufs);
    if (ret)
        return;

    sp = bufs.slice_p->vaddr;

    allegro_fill_slice_param(channel, sp, src_size);

    allegro_mcu_send_decode_frame(dev, channel, channel->frm_buf_id,
                     channel->mv_buf_id, src_addr, src_size, dst_y,
                     dst_uv, bufs.comp_data->paddr,
                     bufs.comp_map->paddr, bufs.list_ref->paddr,
                     bufs.poc->paddr, bufs.mv->paddr,
                     bufs.slice_p->paddr + channel->slice_id * sizeof(struct decode_slice_param));
}

static void allegro_dec_run(struct allegro_channel *channel)
{
    struct allegro_q_data *q_data;
    struct vb2_v4l2_buffer *src_buf;
    struct vb2_v4l2_buffer *dst_buf;

    q_data = get_q_data(channel, V4L2_BUF_TYPE_VIDEO_OUTPUT);

    src_buf = v4l2_m2m_next_src_buf(channel->fh.m2m_ctx);
    dst_buf = v4l2_m2m_next_dst_buf(channel->fh.m2m_ctx);

    /* Apply request(s) controls if needed. */
    if (src_buf->vb2_buf.req_obj.req)
        v4l2_ctrl_request_setup(src_buf->vb2_buf.req_obj.req,
                        &channel->ctrl_handler);

    switch(q_data->fourcc) {
    case V4L2_PIX_FMT_H264_SLICE:
        channel->h264.decode_params = allegro_find_control_data(channel,
            V4L2_CID_STATELESS_H264_DECODE_PARAMS);
        channel->h264.pps = allegro_find_control_data(channel,
            V4L2_CID_STATELESS_H264_PPS);
        channel->h264.scaling_matrix = allegro_find_control_data(channel,
            V4L2_CID_STATELESS_H264_SCALING_MATRIX);
        channel->h264.slice_params = allegro_find_control_data(channel,
            V4L2_CID_STATELESS_H264_SLICE_PARAMS);
        channel->h264.sps = allegro_find_control_data(channel,
            V4L2_CID_STATELESS_H264_SPS);
        channel->h264.pred_weights = allegro_find_control_data(channel,
            V4L2_CID_STATELESS_H264_PRED_WEIGHTS);
        break;

    case V4L2_PIX_FMT_HEVC_SLICE:
        channel->h265.sps = allegro_find_control_data(channel,
            V4L2_CID_MPEG_VIDEO_HEVC_SPS);
        channel->h265.pps = allegro_find_control_data(channel,
            V4L2_CID_MPEG_VIDEO_HEVC_PPS);
        channel->h265.slice_params = allegro_find_control_data(channel,
            V4L2_CID_MPEG_VIDEO_HEVC_SLICE_PARAMS);
        channel->h265.decode_params = allegro_find_control_data(channel,
            V4L2_CID_MPEG_VIDEO_HEVC_DECODE_PARAMS);
        break;

    default:
        break;
    }

    v4l2_m2m_buf_copy_metadata(src_buf, dst_buf, false);

    _allegro_dec_run(channel);

    /* Complete request(s) controls if needed. */
    v4l2_ctrl_request_complete(src_buf->vb2_buf.req_obj.req,
                &channel->ctrl_handler);
}

static void allegro_dec_init(struct allegro_channel *channel)
{
    INIT_LIST_HEAD(&channel->buffers_slice_p);
    INIT_LIST_HEAD(&channel->buffers_comp_data);
    INIT_LIST_HEAD(&channel->buffers_comp_map);
    INIT_LIST_HEAD(&channel->buffers_list_ref);
    INIT_LIST_HEAD(&channel->buffers_mv);
    INIT_LIST_HEAD(&channel->buffers_poc);

    channel->decoder_picture_ids = 0;
    channel->decoder_mv_ids = 0;
}

const struct allegro_ops allegro_dec_ops = {
    .init      = allegro_dec_init,
    .start     = allegro_dec_start,
    .stop      = allegro_dec_stop,
    .message   = allegro_dec_message,
    .run       = allegro_dec_run,
};