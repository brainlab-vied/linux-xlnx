// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 Pengutronix, Michael Tretter <kernel@pengutronix.de>
 *
 * Allegro DVT video encoder driver
 */

#ifndef __ALLEGRO_H__
#define __ALLEGRO_H__

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/firmware.h>
#include <linux/gcd.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/log2.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/xlnx-vcu.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
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

#include "allegro-mail.h"
#include "nal-h264.h"
#include "nal-hevc.h"

/*
 * Support up to 4k video streams. The hardware actually supports higher
 * resolutions, which are specified in PG252 June 6, 2018 (H.264/H.265 Video
 * Codec Unit v1.1) Chapter 3.
 */
#define ALLEGRO_WIDTH_MIN 128
#define ALLEGRO_WIDTH_DEFAULT 1920
#define ALLEGRO_WIDTH_MAX 3840
#define ALLEGRO_HEIGHT_MIN 64
#define ALLEGRO_HEIGHT_DEFAULT 1080
#define ALLEGRO_HEIGHT_MAX 2160

#define ALLEGRO_FRAMERATE_DEFAULT ((struct v4l2_fract) { 30, 1 })

#define ALLEGRO_GOP_SIZE_DEFAULT 25
#define ALLEGRO_GOP_SIZE_MAX 1000

/*
 * MCU Control Registers
 *
 * The Zynq UltraScale+ Devices Register Reference documents the registers
 * with an offset of 0x9000, which equals the size of the SRAM and one page
 * gap. The driver handles SRAM and registers separately and, therefore, is
 * oblivious of the offset.
 */
#define AL5_MCU_RESET                   0x0000
#define AL5_MCU_RESET_SOFT              BIT(0)
#define AL5_MCU_RESET_REGS              BIT(1)
#define AL5_MCU_RESET_MODE              0x0004
#define AL5_MCU_RESET_MODE_SLEEP        BIT(0)
#define AL5_MCU_RESET_MODE_HALT         BIT(1)
#define AL5_MCU_STA                     0x0008
#define AL5_MCU_STA_SLEEP               BIT(0)
#define AL5_MCU_WAKEUP                  0x000c

#define AL5_ICACHE_ADDR_OFFSET_MSB      0x0010
#define AL5_ICACHE_ADDR_OFFSET_LSB      0x0014
#define AL5_DCACHE_ADDR_OFFSET_MSB      0x0018
#define AL5_DCACHE_ADDR_OFFSET_LSB      0x001c

#define AL5_MCU_INTERRUPT               0x0100
#define AL5_ITC_CPU_IRQ_MSK             0x0104
#define AL5_ITC_CPU_IRQ_CLR             0x0108
#define AL5_ITC_CPU_IRQ_STA             0x010C
#define AL5_ITC_CPU_IRQ_STA_TRIGGERED   BIT(0)

#define AXI_ADDR_OFFSET_IP              0x0208

/*
 * The MCU accesses the system memory with a 2G offset compared to CPU
 * physical addresses.
 */
#define MCU_CACHE_OFFSET SZ_2G

#define SIZE_MACROBLOCK 16

#define ALLEGRO_ENTROPY_MODE_CAVLC 0
#define ALLEGRO_ENTROPY_MODE_CABAC 1

// TODO: need to check the functionality of this "debug" variable
// after moving it in this header file from "allegro-core.c",
// expecially due to "statis" declaration.
//
// Potential solutions in case of functionality issues:
// - remote "static" declaration
// - return the variable back in "allegro-core.c" and use as "extern"
// - create the special debug variables in the appropriate files/modules
static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-2)");


struct allegro_buffer {
        void *vaddr;
        dma_addr_t paddr;
        size_t size;
        struct list_head head;
};

struct allegro_dev;
struct allegro_channel;

struct allegro_mbox {
        struct allegro_dev *dev;
        unsigned int head;
        unsigned int tail;
        unsigned int data;
        size_t size;
        /* protect mailbox from simultaneous accesses */
        struct mutex lock;
};

struct allegro_encoder_buffer {
        unsigned int size;
        unsigned int color_depth;
        unsigned int num_cores;
        unsigned int clk_rate;
};

struct allegro_dev {
        struct v4l2_device v4l2_dev;
        struct video_device video_dev;
        struct v4l2_m2m_dev *m2m_dev;
        struct platform_device *plat_dev;

        /* mutex protecting vb2_queue structure */
        struct mutex lock;

        struct regmap *regmap;
        struct regmap *sram;
        struct regmap *settings;

        struct clk *clk_core;
        struct clk *clk_mcu;

        const struct fw_info *fw_info;
        struct allegro_buffer firmware;
        struct allegro_buffer suballocator;
        bool has_encoder_buffer;
        struct allegro_encoder_buffer encoder_buffer;

        struct completion init_complete;
        bool initialized;

        /* The mailbox interface */
        struct allegro_mbox *mbox_command;
        struct allegro_mbox *mbox_status;

        /*
         * The downstream driver limits the users to 64 users, thus I can use
         * a bitfield for the user_ids that are in use. See also user_id in
         * struct allegro_channel.
         */
        unsigned long channel_user_ids;
        struct list_head channels;
};

struct allegro_channel {
        struct allegro_dev *dev;
        struct v4l2_fh fh;
        struct v4l2_ctrl_handler ctrl_handler;

        unsigned int width;
        unsigned int height;
        unsigned int stride;
        struct v4l2_fract framerate;

        enum v4l2_colorspace colorspace;
        enum v4l2_ycbcr_encoding ycbcr_enc;
        enum v4l2_quantization quantization;
        enum v4l2_xfer_func xfer_func;

        u32 pixelformat;
        unsigned int sizeimage_raw;
        unsigned int osequence;

        u32 codec;
        unsigned int sizeimage_encoded;
        unsigned int csequence;

        bool frame_rc_enable;
        unsigned int bitrate;
        unsigned int bitrate_peak;

        struct allegro_buffer config_blob;

        unsigned int log2_max_frame_num;
        bool temporal_mvp_enable;

        bool enable_loop_filter_across_tiles;
        bool enable_loop_filter_across_slices;
        bool enable_deblocking_filter_override;
        bool enable_reordering;
        bool dbf_ovr_en;

        unsigned int num_ref_idx_l0;
        unsigned int num_ref_idx_l1;

        /* Maximum range for motion estimation */
        int b_hrz_me_range;
        int b_vrt_me_range;
        int p_hrz_me_range;
        int p_vrt_me_range;
        /* Size limits of coding unit */
        int min_cu_size;
        int max_cu_size;
        /* Size limits of transform unit */
        int min_tu_size;
        int max_tu_size;
        int max_transfo_depth_intra;
        int max_transfo_depth_inter;

        struct v4l2_ctrl *mpeg_video_h264_profile;
        struct v4l2_ctrl *mpeg_video_h264_level;
        struct v4l2_ctrl *mpeg_video_h264_i_frame_qp;
        struct v4l2_ctrl *mpeg_video_h264_max_qp;
        struct v4l2_ctrl *mpeg_video_h264_min_qp;
        struct v4l2_ctrl *mpeg_video_h264_p_frame_qp;
        struct v4l2_ctrl *mpeg_video_h264_b_frame_qp;

        struct v4l2_ctrl *mpeg_video_hevc_profile;
        struct v4l2_ctrl *mpeg_video_hevc_level;
        struct v4l2_ctrl *mpeg_video_hevc_tier;
        struct v4l2_ctrl *mpeg_video_hevc_i_frame_qp;
        struct v4l2_ctrl *mpeg_video_hevc_max_qp;
        struct v4l2_ctrl *mpeg_video_hevc_min_qp;
        struct v4l2_ctrl *mpeg_video_hevc_p_frame_qp;
        struct v4l2_ctrl *mpeg_video_hevc_b_frame_qp;

        struct v4l2_ctrl *mpeg_video_frame_rc_enable;
        struct { /* video bitrate mode control cluster */
                struct v4l2_ctrl *mpeg_video_bitrate_mode;
                struct v4l2_ctrl *mpeg_video_bitrate;
                struct v4l2_ctrl *mpeg_video_bitrate_peak;
        };
        struct v4l2_ctrl *mpeg_video_cpb_size;
        struct v4l2_ctrl *mpeg_video_gop_size;

        struct v4l2_ctrl *encoder_buffer;

        /* user_id is used to identify the channel during CREATE_CHANNEL */
        /* not sure, what to set here and if this is actually required */
        int user_id;
        /* channel_id is set by the mcu and used by all later commands */
        int mcu_channel_id;

        struct list_head buffers_reference;
        struct list_head buffers_intermediate;

        struct list_head source_shadow_list;
        struct list_head stream_shadow_list;
        /* protect shadow lists of buffers passed to firmware */
        struct mutex shadow_list_lock;

        struct list_head list;
        struct completion completion;

        unsigned int error;
};

struct allegro_m2m_buffer {
        struct v4l2_m2m_buffer buf;
        struct list_head head;
};

#define to_allegro_m2m_buffer(__buf) \
        container_of(__buf, struct allegro_m2m_buffer, buf)

struct fw_info {
        unsigned int id;
        unsigned int id_codec;
        char *version;
        unsigned int mailbox_cmd;
        unsigned int mailbox_status;
        size_t mailbox_size;
        enum mcu_msg_version mailbox_version;
        size_t suballocator_size;
};


static inline u32 to_mcu_addr(struct allegro_dev *dev, dma_addr_t phys)
{
        if (upper_32_bits(phys) || (lower_32_bits(phys) & MCU_CACHE_OFFSET))
                v4l2_warn(&dev->v4l2_dev,
                          "address %pad is outside mcu window\n", &phys);

        return lower_32_bits(phys) | MCU_CACHE_OFFSET;
}

static inline u32 to_mcu_size(struct allegro_dev *dev, size_t size)
{
        return lower_32_bits(size);
}

static inline u32 to_codec_addr(struct allegro_dev *dev, dma_addr_t phys)
{
        if (upper_32_bits(phys))
                v4l2_warn(&dev->v4l2_dev,
                          "address %pad cannot be used by codec\n", &phys);

        return lower_32_bits(phys);
}

static inline u64 ptr_to_u64(const void *ptr)
{
        return (uintptr_t)ptr;
}

/* Helper functions for channel and user operations */

unsigned long allegro_next_user_id(struct allegro_dev *dev);

struct allegro_channel *
allegro_find_channel_by_user_id(struct allegro_dev *dev,
                                unsigned int user_id);

struct allegro_channel *
allegro_find_channel_by_channel_id(struct allegro_dev *dev,
                                   unsigned int channel_id);

static inline bool channel_exists(struct allegro_channel *channel)
{
        return channel->mcu_channel_id != -1;
}

#define AL_ERROR                        0x80
#define AL_ERR_INIT_FAILED              0x81
#define AL_ERR_NO_FRAME_DECODED         0x82
#define AL_ERR_RESOLUTION_CHANGE        0x85
#define AL_ERR_NO_MEMORY                0x87
#define AL_ERR_STREAM_OVERFLOW          0x88
#define AL_ERR_TOO_MANY_SLICES          0x89
#define AL_ERR_BUF_NOT_READY            0x8c
#define AL_ERR_NO_CHANNEL_AVAILABLE     0x8d
#define AL_ERR_RESOURCE_UNAVAILABLE     0x8e
#define AL_ERR_NOT_ENOUGH_CORES         0x8f
#define AL_ERR_REQUEST_MALFORMED        0x90
#define AL_ERR_CMD_NOT_ALLOWED          0x91
#define AL_ERR_INVALID_CMD_VALUE        0x92

static inline const char *allegro_err_to_string(unsigned int err)
{
        switch (err) {
        case AL_ERR_INIT_FAILED:
                return "initialization failed";
        case AL_ERR_NO_FRAME_DECODED:
                return "no frame decoded";
        case AL_ERR_RESOLUTION_CHANGE:
                return "resolution change";
        case AL_ERR_NO_MEMORY:
                return "out of memory";
        case AL_ERR_STREAM_OVERFLOW:
                return "stream buffer overflow";
        case AL_ERR_TOO_MANY_SLICES:
                return "too many slices";
        case AL_ERR_BUF_NOT_READY:
                return "buffer not ready";
        case AL_ERR_NO_CHANNEL_AVAILABLE:
                return "no channel available";
        case AL_ERR_RESOURCE_UNAVAILABLE:
                return "resource unavailable";
        case AL_ERR_NOT_ENOUGH_CORES:
                return "not enough cores";
        case AL_ERR_REQUEST_MALFORMED:
                return "request malformed";
        case AL_ERR_CMD_NOT_ALLOWED:
                return "command not allowed";
        case AL_ERR_INVALID_CMD_VALUE:
                return "invalid command value";
        case AL_ERROR:
        default:
                return "unknown error";
        }
}

int allegro_alloc_buffer(struct allegro_dev *dev,
                                struct allegro_buffer *buffer, size_t size);

void allegro_free_buffer(struct allegro_dev *dev,
                                struct allegro_buffer *buffer);


void allegro_mcu_interrupt(struct allegro_dev *dev);

void allegro_handle_message(struct allegro_dev *dev,
                                   union mcu_msg_response *msg);

#endif // __ALLEGRO_H__

