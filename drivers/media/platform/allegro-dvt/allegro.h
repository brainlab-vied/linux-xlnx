/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2019-2020 Pengutronix, Michael Tretter <kernel@pengutronix.de>
 */

#ifndef __ALLEGRO_H__
#define __ALLEGRO_H__

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/videodev2.h>
#include <linux/wait.h>
#include <linux/clk.h>
#include <linux/regmap.h>
#include <linux/sizes.h>
#include <linux/bits.h>
#include <linux/firmware.h>
#include <linux/version.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-dma-contig.h>



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

#define SIZE_MACROBLOCK 16

/*
 * Support up to 4k video streams. The hardware actually supports higher
 * resolutions, which are specified in PG252 June 6, 2018 (H.264/H.265 Video
 * Codec Unit v1.1) Chapter 3.
 */
#define ALLEGRO_WIDTH_MIN 128
#define ALLEGRO_WIDTH_DEFAULT 1280 // 6.1 - 1920
#define ALLEGRO_WIDTH_MAX 3840
#define ALLEGRO_HEIGHT_MIN 64
#define ALLEGRO_HEIGHT_DEFAULT 720 // 6.1 - 1080
#define ALLEGRO_HEIGHT_MAX 2160

#define ALLEGRO_FRAMERATE_DEFAULT ((struct v4l2_fract) { 30, 1 })

#define ALLEGRO_GOP_SIZE_DEFAULT 25
#define ALLEGRO_GOP_SIZE_MAX 1000

#define ALLEGRO_ENTROPY_MODE_CAVLC 0
#define ALLEGRO_ENTROPY_MODE_CABAC 1

/*
 * The driver needs to reserve some space at the beginning of capture buffers,
 * because it needs to write SPS/PPS NAL units. The encoder writes the actual
 * frame data after the offset.
 */
#define ENCODER_STREAM_OFFSET SZ_128

/*
 * The MCU accesses the system memory with a 2G offset compared to CPU
 * physical addresses.
 */
#define MCU_CACHE_OFFSET SZ_2G

#define ALLEGRO_MAX_FORMATS	4

struct allegro_buffer {
	void *vaddr;
	dma_addr_t paddr;
	int id; // DW : new - field is not presetned in 6.1
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

enum {
	V4L2_M2M_SRC = 0,
	V4L2_M2M_DST = 1,
};

enum allegro_inst_type {
	ALLEGRO_INST_ENCODER,
	ALLEGRO_INST_DECODER,
};

union mcu_msg_response;

struct allegro_ops {
	void (*init)(struct allegro_channel *ch);
	int (*start)(struct allegro_channel *ch);
	void (*stop)(struct allegro_channel *ch);
	void (*run)(struct allegro_channel *ch);
	void (*message)(struct allegro_dev *dev, union mcu_msg_response *msg);
};

struct fw_info {
	unsigned int id;
	unsigned int id_codec;
	char *version;
	unsigned int mailbox_cmd;
	unsigned int mailbox_status;
	size_t mailbox_size;
	int mailbox_version;
	size_t suballocator_size;
};

struct allegro_devtype {
	const char *name;
	enum allegro_inst_type inst_type;
	const char *fwb;
	const char *fw;
	const struct fw_info *fwinfos;
	unsigned int num_fwinfos;
	const struct allegro_ctrl *ctrls;
	unsigned int num_ctrls;
	const struct allegro_ops *ops;
	u32 src_formats[ALLEGRO_MAX_FORMATS];
	u32 dst_formats[ALLEGRO_MAX_FORMATS];
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
#if 0 // DW_6-1
// introduced Oct 20 2021
// https://github.com/Xilinx/linux-xlnx/commit/b6707e770d832da586a4b42d4d45b3a91d5f98c2
	struct regmap *settings;
#endif // DW_6-1

#if 1 // DW_6-1
	struct clk *clk_core;
	struct clk *clk_mcu;
#endif // DW_6-1

	const struct fw_info *fw_info;
	struct allegro_buffer firmware;
	struct allegro_buffer suballocator;
#if 0 // DW_6-1 - disabled - Encoder special patch
// https://github.com/Xilinx/linux-xlnx/commit/98f1cbf65bf275cce2b9985a8d89cd4fc287a9cc
	bool has_encoder_buffer;
	struct allegro_encoder_buffer encoder_buffer;
#endif // DW_6-1

	struct completion init_complete;
#if 1 // DW
// useful fix from 6.1 - https://github.com/Xilinx/linux-xlnx/commit/dacc21d638c427a53448d91bd976ee6762822911
	bool initialized;
#endif // DW

	/* The mailbox interface */
	struct allegro_mbox *mbox_command;
	struct allegro_mbox *mbox_status;

	struct allegro_devtype *devtype; // DW_new - field is not presetned in 6.1
	const struct allegro_ops *ops;   // DW_new - field is not presetned in 6.1

#ifdef CONFIG_MEDIA_CONTROLLER
	struct media_device	mdev; // DW_new - field is not presetned in 6.1
#endif

	/*
	 * The downstream driver limits the users to 64 users, thus I can use
	 * a bitfield for the user_ids that are in use. See also user_id in
	 * struct allegro_channel.
	 */
	unsigned long channel_user_ids;
	struct list_head channels;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0)
	struct clk *pll_ref;
	struct clk *core_clk;
	struct clk *mcu_clk;
	struct regmap *logicore;
#endif
};

/* Per-queue, driver-specific private data */
struct allegro_q_data {
	unsigned int width;
	unsigned int height;
	unsigned int bytesperline;
	unsigned int sizeimage;
	unsigned int fourcc;
	unsigned int sequence;
};

struct allegro_ctrl {
	unsigned int codec;
	bool required;
	struct v4l2_ctrl_config cfg;
};

struct allegro_picture_buffers {
	struct allegro_buffer *slice_p;
	struct allegro_buffer *comp_data;
	struct allegro_buffer *comp_map;
	struct allegro_buffer *list_ref;
	struct allegro_buffer *poc;
	struct allegro_buffer *mv;
};

#define fh_to_channel(__fh) container_of(__fh, struct allegro_channel, fh)

struct allegro_channel {
	struct allegro_dev *dev;
	struct v4l2_fh fh;
	struct v4l2_ctrl_handler ctrl_handler;

	enum allegro_inst_type inst_type;
	const struct allegro_ops *ops;

	struct v4l2_fract framerate;

	enum v4l2_colorspace colorspace;
	enum v4l2_ycbcr_encoding ycbcr_enc;
	enum v4l2_quantization quantization;
	enum v4l2_xfer_func xfer_func;

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

	union {
		struct {
			const struct v4l2_ctrl_h264_decode_params *decode_params;
			const struct v4l2_ctrl_h264_pps	*pps;
			const struct v4l2_ctrl_h264_scaling_matrix *scaling_matrix;
			const struct v4l2_ctrl_h264_slice_params *slice_params;
			const struct v4l2_ctrl_h264_sps	*sps;
			const struct v4l2_ctrl_h264_pred_weights *pred_weights;
		} h264;
		struct {
			const struct v4l2_ctrl_hevc_sps	*sps;
			const struct v4l2_ctrl_hevc_pps	*pps;
			const struct v4l2_ctrl_hevc_slice_params *slice_params;
			const struct v4l2_ctrl_hevc_decode_params	*decode_params;
		} h265;
	};

	/* user_id is used to identify the channel during CREATE_CHANNEL */
	/* not sure, what to set here and if this is actually required */
	int user_id;
	/* channel_id is set by the mcu and used by all later commands */
	int mcu_channel_id;

	/* encoder internal buffers */
	struct list_head buffers_reference;
	struct list_head buffers_intermediate;

	/* decoder internal buffers */
	struct list_head buffers_slice_p;
	struct list_head buffers_comp_data;
	struct list_head buffers_comp_map;
	struct list_head buffers_list_ref;
	struct list_head buffers_poc;
	struct list_head buffers_mv;

	/* Compressed MVDs + header + residuals */
	struct allegro_buffer pool_comp_data[16];
	/* Compresseion map : LCU size + LCU offset pool buffer */
	struct allegro_buffer pool_comp_map[16];
	/* Weighted Pred Tables pool buffer */
	struct allegro_buffer pool_wp[16];
	/* Picture Reference List buffer */
	struct allegro_buffer pool_list_ref[16];

	/* Slice toggle management */

	/* Slice parameters */
	struct allegro_buffer pool_sp[16];
	/* Picture parameters */
	struct allegro_buffer pool_pp[16];
	/* Picture buffers */
	struct allegro_picture_buffers pool_pb[16];

	/* Decoder toggle buffer */

	/* Colocated POC buffer */
	struct allegro_buffer pool_poc[16];
	/* Motion Vector buffer */
	struct allegro_buffer pool_mv[16];

	u64 src_handles[16];
	u64 dst_handles[16];

	struct list_head src_shadow_list;
	struct list_head dst_shadow_list;
	/* protect shadow lists of buffers passed to firmware */
	struct mutex shadow_list_lock;

	struct list_head list;
	struct completion completion;

	unsigned int error;

	/* Source and destination queue data */
	struct allegro_q_data   q_data[2];

	unsigned long decoder_picture_ids;
	unsigned long decoder_mv_ids;
	unsigned int frm_buf_id;
	unsigned int mv_buf_id;
	unsigned int slice_id;

	dma_addr_t rec_bufs[16];
	dma_addr_t mv_bufs[16];
	dma_addr_t poc_bufs[16];
};

/* Logging helpers */
extern int debug;

static inline struct allegro_q_data *
get_q_data(struct allegro_channel *channel, enum v4l2_buf_type type)
{
	switch (type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		return &(channel->q_data[V4L2_M2M_SRC]);
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		return &(channel->q_data[V4L2_M2M_DST]);
	default:
		return NULL;
	}
}

/* Helper functions for mcu */

static inline u32 to_mcu_addr(struct allegro_dev *dev, dma_addr_t phys)
{
	//if (upper_32_bits(phys) || (lower_32_bits(phys) & MCU_CACHE_OFFSET))
	//	v4l2_warn(&dev->v4l2_dev,
	//		  "address %pad is outside mcu window\n", &phys);

	return lower_32_bits(phys) | MCU_CACHE_OFFSET;
}

static inline u32 to_mcu_size(struct allegro_dev *dev, size_t size)
{
	return lower_32_bits(size);
}

static inline u32 to_codec_addr(struct allegro_dev *dev, dma_addr_t phys)
{
	//if (upper_32_bits(phys))
	//	v4l2_warn(&dev->v4l2_dev,
	//		  "address %pad cannot be used by codec\n", &phys);

	return lower_32_bits(phys);
}

static inline u64 ptr_to_u64(const void *ptr)
{
	return (uintptr_t)ptr;
}

/*
 * Buffers that are used internally by the MCU.
 */

static inline int allegro_alloc_buffer(struct allegro_dev *dev,
				struct allegro_buffer *buffer, size_t size)
{
	buffer->vaddr = dma_alloc_coherent(&dev->plat_dev->dev, size,
					   &buffer->paddr, GFP_KERNEL);
	if (!buffer->vaddr)
		return -ENOMEM;
	buffer->size = size;

	return 0;
}

static inline void allegro_free_buffer(struct allegro_dev *dev,
				struct allegro_buffer *buffer)
{
	if (buffer->vaddr) {
		dma_free_coherent(&dev->plat_dev->dev, buffer->size,
				  buffer->vaddr, buffer->paddr);
		buffer->vaddr = NULL;
		buffer->size = 0;
	}
}

/* Helper functions for buffer operations */

struct allegro_m2m_buffer {
	struct v4l2_m2m_buffer buf;
	struct list_head head;
	u32 position;
};

#define to_allegro_m2m_buffer(__buf) \
	container_of(__buf, struct allegro_m2m_buffer, buf)

static inline struct allegro_m2m_buffer *
vb2_v4l2_to_allegro_buffer(const struct vb2_v4l2_buffer *p)
{
	return container_of(p, struct allegro_m2m_buffer, buf.vb);
}

static inline struct allegro_m2m_buffer *
vb2_to_allegro_buffer(const struct vb2_buffer *p)
{
	return vb2_v4l2_to_allegro_buffer(to_vb2_v4l2_buffer(p));
}

static inline u64 allegro_put_buffer(struct allegro_channel *channel,
			      struct list_head *list,
			      struct vb2_v4l2_buffer *buffer)
{
	struct v4l2_m2m_buffer *b = container_of(buffer,
						 struct v4l2_m2m_buffer, vb);
	struct allegro_m2m_buffer *shadow = to_allegro_m2m_buffer(b);

	mutex_lock(&channel->shadow_list_lock);
	list_add_tail(&shadow->head, list);
	mutex_unlock(&channel->shadow_list_lock);

	return ptr_to_u64(buffer);
}

static inline struct vb2_v4l2_buffer *
allegro_get_buffer(struct allegro_channel *channel,
		   struct list_head *list, u64 handle)
{
	struct allegro_m2m_buffer *shadow, *tmp;
	struct vb2_v4l2_buffer *buffer = NULL;

	mutex_lock(&channel->shadow_list_lock);
	list_for_each_entry_safe(shadow, tmp, list, head) {
		if (handle == ptr_to_u64(&shadow->buf.vb)) {
			buffer = &shadow->buf.vb;
			list_del_init(&shadow->head);
			break;
		}
	}
	mutex_unlock(&channel->shadow_list_lock);

	return buffer;
}

/* Helper functions for channel and user operations */

static inline unsigned long allegro_next_user_id(struct allegro_dev *dev)
{
	if (dev->channel_user_ids == ~0UL)
		return -EBUSY;

	return ffz(dev->channel_user_ids);
}

static inline struct allegro_channel *
allegro_find_channel_by_user_id(struct allegro_dev *dev,
				unsigned int user_id)
{
	struct allegro_channel *channel;

	list_for_each_entry(channel, &dev->channels, list) {
		if (channel->user_id == user_id)
			return channel;
	}

	return ERR_PTR(-EINVAL);
}

static inline struct allegro_channel *
allegro_find_channel_by_channel_id(struct allegro_dev *dev,
				   unsigned int channel_id)
{
	struct allegro_channel *channel;

	list_for_each_entry(channel, &dev->channels, list) {
		if (channel->mcu_channel_id == channel_id)
			return channel;
	}

	return ERR_PTR(-EINVAL);
}

static inline bool channel_exists(struct allegro_channel *channel)
{
	return channel->mcu_channel_id != -1;
}

/* Helper functions for mbox */

struct allegro_mbox *allegro_mbox_init(struct allegro_dev *dev,
					      unsigned int base, size_t size);

int allegro_mbox_write(struct allegro_mbox *mbox,
			      const u32 *src, size_t size);

ssize_t allegro_mbox_read(struct allegro_mbox *mbox,
				 u32 *dst, size_t nbyte);

int allegro_mbox_send(struct allegro_mbox *mbox, void *msg);

void allegro_mbox_notify(struct allegro_mbox *mbox);

#define AL_ERROR			0x80
#define AL_ERR_INIT_FAILED		0x81
#define AL_ERR_NO_FRAME_DECODED		0x82
#define AL_ERR_RESOLUTION_CHANGE	0x85
#define AL_ERR_NO_MEMORY		0x87
#define AL_ERR_STREAM_OVERFLOW		0x88
#define AL_ERR_TOO_MANY_SLICES		0x89
#define AL_ERR_BUF_NOT_READY		0x8c
#define AL_ERR_NO_CHANNEL_AVAILABLE	0x8d
#define AL_ERR_RESOURCE_UNAVAILABLE	0x8e
#define AL_ERR_NOT_ENOUGH_CORES		0x8f
#define AL_ERR_REQUEST_MALFORMED	0x90
#define AL_ERR_CMD_NOT_ALLOWED		0x91
#define AL_ERR_INVALID_CMD_VALUE	0x92

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

//static
void allegro_handle_message(struct allegro_dev *dev,
				   union mcu_msg_response *msg);

void *allegro_find_control_data(struct allegro_channel *channel, u32 id);


#endif /* __ALLEGRO_H__ */
