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
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
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

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0)
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/mfd/syscon/xlnx-vcu.h>
#include <linux/regmap.h>
#include <linux/clk.h>
#endif

#include "allegro.h"
#include "allegro-mail.h"
#include "nal-h264.h"
#include "nal-hevc.h"

#ifdef CONFIG_MEMORY_HOTPLUG
#define HOTPLUG_ALIGN 0x40000000
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0)
#define VCU_PLL_CLK			0x34
#define VCU_PLL_CLK_DEC			0x64
#define VCU_MCU_CLK			0x24
#define VCU_CORE_CLK			0x28
#define MHZ				1000000
#define FRAC				100
#endif


int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-2)");

extern struct allegro_ops allegro_enc_ops;
extern struct allegro_ops allegro_dec_ops;

static struct regmap_config allegro_regmap_config = {
	.name = "regmap",
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = 0xfff,
	.cache_type = REGCACHE_NONE,
};

static struct regmap_config allegro_sram_config = {
	.name = "sram",
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = 0x7fff,
	.cache_type = REGCACHE_NONE,
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0)
static struct regmap_config allegro_logicore_config = {
	.name = "logicore",
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = 0xfff,
	.cache_type = REGCACHE_NONE,
};
#endif

static const struct fw_info fw_info_enc[] = {
	{
		.id = 18296,
		.id_codec = 96272,
		.version = "v2018.2",
		.mailbox_cmd = 0x7800,
		.mailbox_status = 0x7c00,
		.mailbox_size = 0x400 - 0x8,
		.mailbox_version = MCU_MSG_VERSION_2018_2,
		.suballocator_size = SZ_16M,
	}, {
		.id = 14680,
		.id_codec = 126572,
		.version = "v2019.2",
		.mailbox_cmd = 0x7000,
		.mailbox_status = 0x7800,
		.mailbox_size = 0x800 - 0x8,
		.mailbox_version = MCU_MSG_VERSION_2019_2,
		.suballocator_size = SZ_32M,
	}, {
		.id = 17256,
		.id_codec = 138748,
		.version = "v2021.1",
		.mailbox_cmd = 0x7000,
		.mailbox_status = 0x7800,
		.mailbox_size = 0x800 - 0x8,
		.mailbox_version = MCU_MSG_VERSION_2021_1,
		.suballocator_size = SZ_32M,
	}, {
		.id = 17312,
		.id_codec = 141684,
		.version = "v2022.2",
		.mailbox_cmd = 0x7000,
		.mailbox_status = 0x7800,
		.mailbox_size = 0x800 - 0x8,
		.mailbox_version = MCU_MSG_VERSION_2022_2,
		.suballocator_size = SZ_32M,
	},
};

static const struct fw_info fw_info_dec[] = {
    {
        .id = 10966,
        .id_codec = 35680,
        .version = "v2018.2",
        .mailbox_cmd = 0x7800,
        .mailbox_status = 0x7c00,
        .mailbox_size = 0x400 - 0x8,
        .mailbox_version = MCU_MSG_VERSION_2018_2,
        .suballocator_size = SZ_16M,
    }, {
        .id = 13456,
        .id_codec = 36772,
        .version = "v2019.2",
        .mailbox_cmd = 0x7000,
        .mailbox_status = 0x7800,
        .mailbox_size = 0x800 - 0x8,
        .mailbox_version = MCU_MSG_VERSION_2019_2,
        .suballocator_size = SZ_32M,
    }, {
        .id = 13608,
        .id_codec = 40896,
        .version = "v2021.1",
        .mailbox_cmd = 0x7000,
        .mailbox_status = 0x7800,
        .mailbox_size = 0x800 - 0x8,
        .mailbox_version = MCU_MSG_VERSION_2021_1,
        .suballocator_size = SZ_32M,
    },{
        .id = 13768,
        .id_codec = 41452,
        .version = "v2022.2",
        .mailbox_cmd = 0x7000,
        .mailbox_status = 0x7800,
        .mailbox_size = 0x800 - 0x8,
        .mailbox_version = MCU_MSG_VERSION_2022_2,
        .suballocator_size = SZ_32M,
    },
};

static const struct allegro_ctrl ctrls_enc[] = {
	/*{
		.cfg = {
			.id = V4L2_CID_MPEG_VIDEO_H264_PROFILE,
			.min = V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE,
			.max = V4L2_MPEG_VIDEO_H264_PROFILE_HIGH,
			.menu_skip_mask =
			BIT(V4L2_MPEG_VIDEO_H264_PROFILE_EXTENDED),
			.def = V4L2_MPEG_VIDEO_H264_PROFILE_MAIN,
		},
		.required	= true,
	},*/
};

static const struct allegro_ctrl ctrls_dec[] = {
	{
		.cfg = {
			.id	= V4L2_CID_STATELESS_H264_DECODE_PARAMS,
		},
		.required	= true,
	}, {
		.cfg = {
			.id	= V4L2_CID_STATELESS_H264_SLICE_PARAMS,
		},
		.required	= true,
	}, {
		.cfg = {
			.id	= V4L2_CID_STATELESS_H264_SPS,
		},
		.required	= true,
	}, {
		.cfg = {
			.id	= V4L2_CID_STATELESS_H264_PPS,
		},
		.required	= true,
	}, {
		.cfg = {
			.id	= V4L2_CID_STATELESS_H264_SCALING_MATRIX,
		},
		.required	= false,
	}, {
		.cfg = {
			.id	= V4L2_CID_STATELESS_H264_PRED_WEIGHTS,
		},
		.required	= false,
	}, {
		.cfg = {
			.id	= V4L2_CID_STATELESS_H264_DECODE_MODE,
			.min = V4L2_STATELESS_H264_DECODE_MODE_SLICE_BASED,
			.def = V4L2_STATELESS_H264_DECODE_MODE_SLICE_BASED,
			.max = V4L2_STATELESS_H264_DECODE_MODE_SLICE_BASED,
		},
		.required	= false,
	}, {
		.cfg = {
			.id	= V4L2_CID_STATELESS_H264_START_CODE,
			.min = V4L2_STATELESS_H264_START_CODE_NONE,
			.def = V4L2_STATELESS_H264_START_CODE_NONE,
			.max = V4L2_STATELESS_H264_START_CODE_NONE,
		},
		.required	= false,
	}, {
		.cfg = {
			.id = V4L2_CID_MPEG_VIDEO_H264_PROFILE,
			.min = V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE,
			.max = V4L2_MPEG_VIDEO_H264_PROFILE_HIGH,
			.menu_skip_mask =
			~(BIT(V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE) |
			  BIT(V4L2_MPEG_VIDEO_H264_PROFILE_MAIN) |
			  BIT(V4L2_MPEG_VIDEO_H264_PROFILE_HIGH)),
			.def = V4L2_MPEG_VIDEO_H264_PROFILE_MAIN,
		},
		.required	= true,
	}, {
		.cfg = {
			.id = V4L2_CID_MPEG_VIDEO_H264_LEVEL,
			.min = V4L2_MPEG_VIDEO_H264_LEVEL_1_0,
			.max = V4L2_MPEG_VIDEO_H264_LEVEL_5_1,
			.menu_skip_mask = 0,
			.def = V4L2_MPEG_VIDEO_H264_LEVEL_5_1,
		},
		.required	= true,
	}, {
		.cfg = {
			.id	= V4L2_CID_MPEG_VIDEO_HEVC_SPS,
		},
		.required	= true,
	}, {
		.cfg = {
			.id	= V4L2_CID_MPEG_VIDEO_HEVC_PPS,
		},
		.required	= true,
	}, {
		.cfg = {
			.id	= V4L2_CID_MPEG_VIDEO_HEVC_SLICE_PARAMS,
		},
		.required	= true,
	}, {
		.cfg = {
			.id	= V4L2_CID_MPEG_VIDEO_HEVC_DECODE_MODE,
			.max	= V4L2_MPEG_VIDEO_HEVC_DECODE_MODE_SLICE_BASED,
			.def	= V4L2_MPEG_VIDEO_HEVC_DECODE_MODE_SLICE_BASED,
		},
		.required	= false,
	}, {
		.cfg = {
			.id	= V4L2_CID_MPEG_VIDEO_HEVC_START_CODE,
			.max	= V4L2_MPEG_VIDEO_HEVC_START_CODE_NONE,
			.def	= V4L2_MPEG_VIDEO_HEVC_START_CODE_NONE,
		},
		.required	= false,
	},
};


static unsigned int estimate_stream_size(unsigned int width,
					 unsigned int height)
{
	unsigned int offset = ENCODER_STREAM_OFFSET;
	unsigned int num_blocks = DIV_ROUND_UP(width, SIZE_MACROBLOCK) *
					DIV_ROUND_UP(height, SIZE_MACROBLOCK);
	unsigned int pcm_size = SZ_256;
	unsigned int partition_table = SZ_256;

	return round_up(offset + num_blocks * pcm_size + partition_table, 32);
}

static const struct fw_info *
allegro_get_firmware_info(struct allegro_dev *dev,
			  const struct firmware *fw,
			  const struct firmware *fw_codec)
{
	int i;
	unsigned int id = fw->size;
	unsigned int id_codec = fw_codec->size;
	const struct fw_info *fwinfos = dev->devtype->fwinfos;
	unsigned int num_fwinfos = dev->devtype->num_fwinfos;

	for (i = 0; i < num_fwinfos; i++)
		if (fwinfos[i].id == id &&
		    fwinfos[i].id_codec == id_codec)
			return &fwinfos[i];

	return NULL;
}

/*
 * Mailbox interface to send messages to the MCU.
 */

static void allegro_mcu_send_init(struct allegro_dev *dev,
				  dma_addr_t suballoc_dma, size_t suballoc_size)
{
	struct mcu_msg_init_request msg;

	memset(&msg, 0, sizeof(msg));

	msg.header.type = MCU_MSG_TYPE_INIT;
	msg.header.version = dev->fw_info->mailbox_version;

	msg.suballoc_dma = to_mcu_addr(dev, suballoc_dma);
	msg.suballoc_size = to_mcu_size(dev, suballoc_size);

	/* disable L2 cache */
	msg.l2_cache[0] = -1;
	msg.l2_cache[1] = -1;
	msg.l2_cache[2] = -1;

	allegro_mbox_send(dev->mbox_command, &msg);
}

static int allegro_mcu_wait_for_init_timeout(struct allegro_dev *dev,
					     unsigned long timeout_ms)
{
	unsigned long tmo;

	tmo = wait_for_completion_timeout(&dev->init_complete,
					  msecs_to_jiffies(timeout_ms));
	if (tmo == 0)
		return -ETIMEDOUT;

	reinit_completion(&dev->init_complete);
	return 0;
}

void allegro_channel_eos_event(struct allegro_channel *channel)
{
	const struct v4l2_event eos_event = {
		.type = V4L2_EVENT_EOS
	};

	v4l2_event_queue_fh(&channel->fh, &eos_event);
}

static int allegro_handle_init(struct allegro_dev *dev,
			       struct mcu_msg_init_response *msg)
{
	v4l2_dbg(1, debug, &dev->v4l2_dev, "MCU init done\n");

	complete(&dev->init_complete);

	return 0;
}

void allegro_handle_message(struct allegro_dev *dev,
				   union mcu_msg_response *msg)
{
	switch (msg->header.type) {
	case MCU_MSG_TYPE_INIT:
		allegro_handle_init(dev, &msg->init);
		break;
	default:
		dev->devtype->ops->message(dev, msg);
		break;
	}
}

static irqreturn_t allegro_hardirq(int irq, void *data)
{
	struct allegro_dev *dev = data;
	unsigned int status;

	regmap_read(dev->regmap, AL5_ITC_CPU_IRQ_STA, &status);
	if (!(status & AL5_ITC_CPU_IRQ_STA_TRIGGERED))
		return IRQ_NONE;

	regmap_write(dev->regmap, AL5_ITC_CPU_IRQ_CLR, status);

	return IRQ_WAKE_THREAD;
}

static irqreturn_t allegro_irq_thread(int irq, void *data)
{
	struct allegro_dev *dev = data;

	/*
	 * The firmware is initialized after the mailbox is setup. We further
	 * check the AL5_ITC_CPU_IRQ_STA register, if the firmware actually
	 * triggered the interrupt. Although this should not happen, make sure
	 * that we ignore interrupts, if the mailbox is not initialized.
	 */
	if (!dev->mbox_status)
		return IRQ_NONE;

	allegro_mbox_notify(dev->mbox_status);

	return IRQ_HANDLED;
}

static void allegro_copy_firmware(struct allegro_dev *dev,
				  const u8 * const buf, size_t size)
{
	int err = 0;

	v4l2_dbg(1, debug, &dev->v4l2_dev,
		 "copy mcu firmware (%zu B) to SRAM\n", size);
	err = regmap_bulk_write(dev->sram, 0x0, buf, size / 4);
	if (err)
		v4l2_err(&dev->v4l2_dev,
			 "failed to copy firmware: %d\n", err);
}

static void allegro_copy_fw_codec(struct allegro_dev *dev,
				  const u8 * const buf, size_t size)
{
	int err;
	dma_addr_t icache_offset, dcache_offset;

	/*
	 * The downstream allocates 600 KB for the codec firmware to have some
	 * extra space for "possible extensions." My tests were fine with
	 * allocating just enough memory for the actual firmware, but I am not
	 * sure that the firmware really does not use the remaining space.
	 */
	err = allegro_alloc_buffer(dev, &dev->firmware, size);
	if (err) {
		v4l2_err(&dev->v4l2_dev,
			 "failed to allocate %zu bytes for firmware\n", size);
		return;
	}

	v4l2_dbg(1, debug, &dev->v4l2_dev,
		 "copy codec firmware (%zd B) to phys %pad\n",
		 size, &dev->firmware.paddr);
	memcpy(dev->firmware.vaddr, buf, size);

	regmap_write(dev->regmap, AXI_ADDR_OFFSET_IP,
		     upper_32_bits(dev->firmware.paddr));

	icache_offset = dev->firmware.paddr - MCU_CACHE_OFFSET;
	v4l2_dbg(2, debug, &dev->v4l2_dev,
		 "icache_offset: msb = 0x%x, lsb = 0x%x\n",
		 upper_32_bits(icache_offset), lower_32_bits(icache_offset));
	regmap_write(dev->regmap, AL5_ICACHE_ADDR_OFFSET_MSB,
		     upper_32_bits(icache_offset));
	regmap_write(dev->regmap, AL5_ICACHE_ADDR_OFFSET_LSB,
		     lower_32_bits(icache_offset));

	dcache_offset =
	    (dev->firmware.paddr & 0xffffffff00000000ULL) - MCU_CACHE_OFFSET;
	v4l2_dbg(2, debug, &dev->v4l2_dev,
		 "dcache_offset: msb = 0x%x, lsb = 0x%x\n",
		 upper_32_bits(dcache_offset), lower_32_bits(dcache_offset));
	regmap_write(dev->regmap, AL5_DCACHE_ADDR_OFFSET_MSB,
		     upper_32_bits(dcache_offset));
	regmap_write(dev->regmap, AL5_DCACHE_ADDR_OFFSET_LSB,
		     lower_32_bits(dcache_offset));
}

static void allegro_free_fw_codec(struct allegro_dev *dev)
{
	allegro_free_buffer(dev, &dev->firmware);
}

/*
 * Control functions for the MCU
 */

static int allegro_mcu_enable_interrupts(struct allegro_dev *dev)
{
	return regmap_write(dev->regmap, AL5_ITC_CPU_IRQ_MSK, BIT(0));
}

static int allegro_mcu_disable_interrupts(struct allegro_dev *dev)
{
	return regmap_write(dev->regmap, AL5_ITC_CPU_IRQ_MSK, 0);
}

static int allegro_mcu_wait_for_sleep(struct allegro_dev *dev)
{
	unsigned long timeout;
	unsigned int status;

	timeout = jiffies + msecs_to_jiffies(100);
	while (regmap_read(dev->regmap, AL5_MCU_STA, &status) == 0 &&
	       status != AL5_MCU_STA_SLEEP) {
		if (time_after(jiffies, timeout))
			return -ETIMEDOUT;
		cpu_relax();
	}

	return 0;
}

static int allegro_mcu_start(struct allegro_dev *dev)
{
	unsigned long timeout;
	unsigned int status;
	int err;

	err = regmap_write(dev->regmap, AL5_MCU_WAKEUP, BIT(0));
	if (err)
		return err;

	timeout = jiffies + msecs_to_jiffies(100);
	while (regmap_read(dev->regmap, AL5_MCU_STA, &status) == 0 &&
	       status == AL5_MCU_STA_SLEEP) {
		if (time_after(jiffies, timeout))
			return -ETIMEDOUT;
		cpu_relax();
	}

	err = regmap_write(dev->regmap, AL5_MCU_WAKEUP, 0);
	if (err)
		return err;

	return 0;
}

static int allegro_mcu_reset(struct allegro_dev *dev)
{
	int err;

	/*
	 * Ensure that the AL5_MCU_WAKEUP bit is set to 0 otherwise the mcu
	 * does not go to sleep after the reset.
	 */
	err = regmap_write(dev->regmap, AL5_MCU_WAKEUP, 0);
	if (err)
		return err;

	err = regmap_write(dev->regmap,
			   AL5_MCU_RESET_MODE, AL5_MCU_RESET_MODE_SLEEP);
	if (err < 0)
		return err;

	err = regmap_write(dev->regmap, AL5_MCU_RESET, AL5_MCU_RESET_SOFT);
	if (err < 0)
		return err;

	return allegro_mcu_wait_for_sleep(dev);
}

static void allegro_destroy_channel(struct allegro_channel *channel)
{
	struct allegro_dev *dev = channel->dev;

	if (channel_exists(channel)) {
		dev->devtype->ops->stop(channel);
		channel->mcu_channel_id = -1;
	}

	if (channel->user_id != -1) {
		clear_bit(channel->user_id, &dev->channel_user_ids);
		channel->user_id = -1;
	}
}

/*
 * Create the MCU channel
 *
 * After the channel has been created, the picture size, format, colorspace
 * and framerate are fixed. Also the codec, profile, bitrate, etc. cannot be
 * changed anymore.
 *
 * The channel can be created only once. The MCU will accept source buffers
 * and stream buffers only after a channel has been created.
 */
static int allegro_create_channel(struct allegro_channel *channel)
{
	struct allegro_dev *dev = channel->dev;
	struct allegro_q_data *q_data;
	int ret;

	q_data = get_q_data(channel, V4L2_BUF_TYPE_VIDEO_CAPTURE);

	if (channel_exists(channel)) {
		v4l2_warn(&dev->v4l2_dev,
			  "channel already exists\n");
		return 0;
	}

	channel->user_id = allegro_next_user_id(dev);
	if (channel->user_id < 0) {
		v4l2_err(&dev->v4l2_dev,
			 "no free channels available\n");
		return -EBUSY;
	}
	set_bit(channel->user_id, &dev->channel_user_ids);

	v4l2_dbg(1, debug, &dev->v4l2_dev,
		 "user %d: creating channel (%4.4s, %dx%d@%d)\n",
		 channel->user_id,
		 (char *)&q_data->fourcc, q_data->width, q_data->height,
		 DIV_ROUND_UP(channel->framerate.numerator,
			      channel->framerate.denominator));

	ret = channel->ops->start(channel);
	if (ret) {
		v4l2_err(&dev->v4l2_dev,
			 "failed to create channel: %d\n", channel->user_id);
		goto err;
	}

	v4l2_dbg(1, debug, &dev->v4l2_dev,
		 "channel %d: accepting buffers\n",
		 channel->mcu_channel_id);

	return 0;

err:
	allegro_destroy_channel(channel);
	return ret;
}

static void allegro_set_default_params(struct allegro_channel *channel)
{
	struct allegro_dev *dev = channel->dev;
	unsigned int h, w, stride, usize, csize;

	h = ALLEGRO_HEIGHT_DEFAULT;
	w = ALLEGRO_WIDTH_DEFAULT;
	stride = round_up(w, 32);
	usize = stride * h * 3 / 2;
	csize = estimate_stream_size(stride, h);

	channel->framerate = ALLEGRO_FRAMERATE_DEFAULT;
	channel->colorspace = V4L2_COLORSPACE_REC709;
	channel->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	channel->quantization = V4L2_QUANTIZATION_DEFAULT;
	channel->xfer_func = V4L2_XFER_FUNC_DEFAULT;

	/* Default formats for output and input queues */
	channel->q_data[V4L2_M2M_SRC].fourcc = dev->devtype->src_formats[0];
	channel->q_data[V4L2_M2M_DST].fourcc = dev->devtype->dst_formats[0];
	channel->q_data[V4L2_M2M_SRC].width = w;
	channel->q_data[V4L2_M2M_SRC].height = h;
	channel->q_data[V4L2_M2M_DST].width = w;
	channel->q_data[V4L2_M2M_DST].height = h;

	if (channel->inst_type == ALLEGRO_INST_ENCODER) {
		channel->q_data[V4L2_M2M_SRC].bytesperline = stride;
		channel->q_data[V4L2_M2M_SRC].sizeimage = usize;
		channel->q_data[V4L2_M2M_DST].bytesperline = 0;
		channel->q_data[V4L2_M2M_DST].sizeimage = csize;
	} else {
		channel->q_data[V4L2_M2M_SRC].bytesperline = 0;
		channel->q_data[V4L2_M2M_SRC].sizeimage = csize;
		channel->q_data[V4L2_M2M_DST].bytesperline = stride;
		channel->q_data[V4L2_M2M_DST].sizeimage = usize;
	}
}

static int allegro_queue_setup(struct vb2_queue *vq,
			       unsigned int *nbuffers, unsigned int *nplanes,
			       unsigned int sizes[],
			       struct device *alloc_devs[])
{
	struct allegro_channel *channel = vb2_get_drv_priv(vq);
	struct allegro_dev *dev = channel->dev;
	struct allegro_q_data *q_data;
	unsigned int size;

	q_data = get_q_data(channel, vq->type);
	size = q_data->sizeimage;

	v4l2_dbg(2, debug, &dev->v4l2_dev,
		 "%s: queue setup[%s]: nplanes = %d size = %d\n",
		 V4L2_TYPE_IS_OUTPUT(vq->type) ? "output" : "capture",
		 *nplanes == 0 ? "REQBUFS" : "CREATE_BUFS", *nplanes, size);

	if (*nbuffers < vq->min_buffers_needed)
		*nbuffers = vq->min_buffers_needed;

	if (*nplanes)
		return sizes[0] < size ? -EINVAL : 0;

	*nplanes = 1;
	sizes[0] = size;

	return 0;
}

static int allegro_buf_prepare(struct vb2_buffer *vb)
{
	struct vb2_queue *vq = vb->vb2_queue;
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct allegro_channel *channel = vb2_get_drv_priv(vb->vb2_queue);
	struct allegro_q_data *q_data = get_q_data(channel, vq->type);
	struct allegro_dev *dev = channel->dev;

	if (V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type)) {
		if (vbuf->field == V4L2_FIELD_ANY)
			vbuf->field = V4L2_FIELD_NONE;
		if (vbuf->field != V4L2_FIELD_NONE) {
			v4l2_err(&dev->v4l2_dev,
				 "channel %d: unsupported field\n",
				 channel->mcu_channel_id);
			return -EINVAL;
		}
	}

	if (vb2_plane_size(vb, 0) < q_data->sizeimage)
		return -EINVAL;

	/*
	 * Buffer's bytesused must be written by driver for CAPTURE buffers.
	 * (for OUTPUT buffers, if userspace passes 0 bytesused, v4l2-core sets
	 * it to buffer length).
	 */
	if (V4L2_TYPE_IS_CAPTURE(vq->type))
		vb2_set_plane_payload(vb, 0, q_data->sizeimage);

	return 0;
}

static void allegro_buf_queue(struct vb2_buffer *vb)
{
	struct allegro_channel *channel = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct vb2_queue *q = vb->vb2_queue;
	struct allegro_q_data *q_data;

	q_data = get_q_data(channel, V4L2_BUF_TYPE_VIDEO_CAPTURE);

	if (V4L2_TYPE_IS_CAPTURE(q->type) &&
	    vb2_is_streaming(q) &&
	    v4l2_m2m_dst_buf_is_last(channel->fh.m2m_ctx)) {
		unsigned int i;

		for (i = 0; i < vb->num_planes; i++)
			vb->planes[i].bytesused = 0;

		vbuf->field = V4L2_FIELD_NONE;
		vbuf->sequence = q_data->sequence++;

		v4l2_m2m_last_buffer_done(channel->fh.m2m_ctx, vbuf);
		allegro_channel_eos_event(channel);
		return;
	}

	v4l2_m2m_buf_queue(channel->fh.m2m_ctx, vbuf);
}

static int allegro_buf_out_validate(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	vbuf->field = V4L2_FIELD_NONE;
	return 0;
}

static void allegro_buf_request_complete(struct vb2_buffer *vb)
{
	struct allegro_channel *channel = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_ctrl_request_complete(vb->req_obj.req, &channel->ctrl_handler);
}

static int allegro_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct allegro_channel *channel = vb2_get_drv_priv(q);
	struct allegro_dev *dev = channel->dev;
	struct allegro_q_data *q_data;

	q_data = get_q_data(channel, q->type);

	v4l2_dbg(2, debug, &dev->v4l2_dev,
		 "%s: start streaming\n",
		 V4L2_TYPE_IS_OUTPUT(q->type) ? "output" : "capture");

	v4l2_m2m_update_start_streaming_state(channel->fh.m2m_ctx, q);

	q_data->sequence = 0;
	return 0;
}

static void allegro_stop_streaming(struct vb2_queue *q)
{
	struct allegro_channel *channel = vb2_get_drv_priv(q);
	struct allegro_dev *dev = channel->dev;
	struct vb2_v4l2_buffer *buffer;
	struct allegro_m2m_buffer *shadow, *tmp;
	struct list_head *src_list = &channel->src_shadow_list;
	struct list_head *dst_list = &channel->dst_shadow_list;

	v4l2_dbg(2, debug, &dev->v4l2_dev,
		 "%s: stop streaming\n",
		 V4L2_TYPE_IS_OUTPUT(q->type) ? "output" : "capture");

	if (V4L2_TYPE_IS_OUTPUT(q->type)) {
		mutex_lock(&channel->shadow_list_lock);
		list_for_each_entry_safe(shadow, tmp, src_list, head) {
			list_del(&shadow->head);
			v4l2_m2m_buf_done(&shadow->buf.vb, VB2_BUF_STATE_ERROR);
		}
		mutex_unlock(&channel->shadow_list_lock);

		while ((buffer = v4l2_m2m_src_buf_remove(channel->fh.m2m_ctx))) {
			v4l2_ctrl_request_complete(buffer->vb2_buf.req_obj.req,
					   &channel->ctrl_handler);
			v4l2_m2m_buf_done(buffer, VB2_BUF_STATE_ERROR);
		}
	} else {
		mutex_lock(&channel->shadow_list_lock);
		list_for_each_entry_safe(shadow, tmp, dst_list, head) {
			list_del(&shadow->head);
			v4l2_m2m_buf_done(&shadow->buf.vb, VB2_BUF_STATE_ERROR);
		}
		mutex_unlock(&channel->shadow_list_lock);

		allegro_destroy_channel(channel);
		while ((buffer = v4l2_m2m_dst_buf_remove(channel->fh.m2m_ctx))) {
			v4l2_ctrl_request_complete(buffer->vb2_buf.req_obj.req,
					   &channel->ctrl_handler);
			v4l2_m2m_buf_done(buffer, VB2_BUF_STATE_ERROR);
		}
	}

	v4l2_m2m_update_stop_streaming_state(channel->fh.m2m_ctx, q);

	if (V4L2_TYPE_IS_OUTPUT(q->type) &&
	    v4l2_m2m_has_stopped(channel->fh.m2m_ctx))
		allegro_channel_eos_event(channel);
}

static const struct vb2_ops allegro_queue_ops = {
	.queue_setup = allegro_queue_setup,
	.buf_prepare = allegro_buf_prepare,
	.buf_queue = allegro_buf_queue,
	.buf_out_validate	= allegro_buf_out_validate,
	.buf_request_complete	= allegro_buf_request_complete,
	.start_streaming = allegro_start_streaming,
	.stop_streaming = allegro_stop_streaming,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
};

static int allegro_queue_init(void *priv,
			      struct vb2_queue *src_vq,
			      struct vb2_queue *dst_vq)
{
	int err;
	struct allegro_channel *channel = priv;

	src_vq->dev = &channel->dev->plat_dev->dev;
	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	src_vq->io_modes = VB2_DMABUF | VB2_MMAP;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->drv_priv = channel;
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->ops = &allegro_queue_ops;
	src_vq->buf_struct_size = sizeof(struct allegro_m2m_buffer);
	src_vq->min_buffers_needed = 1;
	src_vq->lock = &channel->dev->lock;
	if (channel->inst_type == ALLEGRO_INST_DECODER) {
		src_vq->supports_requests = true;
		src_vq->requires_requests = true;
	}
	err = vb2_queue_init(src_vq);
	if (err)
		return err;

	dst_vq->dev = &channel->dev->plat_dev->dev;
	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	dst_vq->io_modes = VB2_DMABUF | VB2_MMAP;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->drv_priv = channel;
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->ops = &allegro_queue_ops;
	dst_vq->buf_struct_size = sizeof(struct allegro_m2m_buffer);
	dst_vq->min_buffers_needed = 1;
	dst_vq->lock = &channel->dev->lock;
	err = vb2_queue_init(dst_vq);
	if (err)
		return err;

	return 0;
}

static int allegro_clamp_qp(struct allegro_channel *channel,
			    struct v4l2_ctrl *ctrl)
{
	struct v4l2_ctrl *next_ctrl;

	if (ctrl->id == V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP)
		next_ctrl = channel->mpeg_video_h264_p_frame_qp;
	else if (ctrl->id == V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP)
		next_ctrl = channel->mpeg_video_h264_b_frame_qp;
	else
		return 0;

	/* Modify range automatically updates the value */
	__v4l2_ctrl_modify_range(next_ctrl, ctrl->val, 51, 1, ctrl->val);

	return allegro_clamp_qp(channel, next_ctrl);
}

static int allegro_clamp_bitrate(struct allegro_channel *channel,
				 struct v4l2_ctrl *ctrl)
{
	struct v4l2_ctrl *ctrl_bitrate = channel->mpeg_video_bitrate;
	struct v4l2_ctrl *ctrl_bitrate_peak = channel->mpeg_video_bitrate_peak;

	if (ctrl->val == V4L2_MPEG_VIDEO_BITRATE_MODE_VBR &&
	    ctrl_bitrate_peak->val < ctrl_bitrate->val)
		ctrl_bitrate_peak->val = ctrl_bitrate->val;

	return 0;
}

static int allegro_try_ctrl(struct v4l2_ctrl *ctrl)
{
	struct allegro_channel *channel = container_of(ctrl->handler,
						       struct allegro_channel,
						       ctrl_handler);

	switch (ctrl->id) {
	case V4L2_CID_MPEG_VIDEO_BITRATE_MODE:
		allegro_clamp_bitrate(channel, ctrl);
		break;
	}

	return 0;
}

static int allegro_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct allegro_channel *channel = container_of(ctrl->handler,
						       struct allegro_channel,
						       ctrl_handler);
	struct allegro_dev *dev = channel->dev;

	v4l2_dbg(1, debug, &dev->v4l2_dev,
		 "s_ctrl: %s = %d\n", v4l2_ctrl_get_name(ctrl->id), ctrl->val);

	switch (ctrl->id) {
	case V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE:
		channel->frame_rc_enable = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_BITRATE_MODE:
		channel->bitrate = channel->mpeg_video_bitrate->val;
		channel->bitrate_peak = channel->mpeg_video_bitrate_peak->val;
		v4l2_ctrl_activate(channel->mpeg_video_bitrate_peak,
				   ctrl->val == V4L2_MPEG_VIDEO_BITRATE_MODE_VBR);
		break;
	case V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP:
	case V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP:
	case V4L2_CID_MPEG_VIDEO_H264_B_FRAME_QP:
		allegro_clamp_qp(channel, ctrl);
		break;
	}

	return 0;
}

const struct v4l2_ctrl_ops allegro_ctrl_ops = {
	.try_ctrl = allegro_try_ctrl,
	.s_ctrl = allegro_s_ctrl,
};

void *allegro_find_control_data(struct allegro_channel *channel, u32 id)
{
	struct v4l2_ctrl *ctrl;

	ctrl = v4l2_ctrl_find(&channel->ctrl_handler, id);
	return ctrl ? ctrl->p_cur.p : NULL;
}

static int allegro_ctrls_setup(struct allegro_channel *channel)
{
	struct allegro_dev *dev = channel->dev;
	struct v4l2_ctrl_handler *hdl = &channel->ctrl_handler;
	const struct allegro_ctrl *ctrls = dev->devtype->ctrls;
	int i, num_ctrls = dev->devtype->num_ctrls;

	v4l2_ctrl_handler_init(hdl, num_ctrls);
	if (hdl->error) {
		v4l2_err(&dev->v4l2_dev,
			 "Failed to initialize control handler\n");
		return hdl->error;
	}

	for (i = 0; i < num_ctrls; i++) {
		v4l2_ctrl_new_custom(hdl, &ctrls[i].cfg, NULL);
		if (hdl->error) {
			v4l2_err(&dev->v4l2_dev, "adding control (%d) failed %d\n",
				ctrls[i].cfg.id, hdl->error);
			v4l2_ctrl_handler_free(hdl);
			return hdl->error;
		}
	}

	channel->fh.ctrl_handler = hdl;
	return v4l2_ctrl_handler_setup(hdl);
}

static int allegro_request_validate(struct media_request *req)
{
	struct media_request_object *obj;
	struct allegro_channel *channel = NULL;
	unsigned int count;

	list_for_each_entry(obj, &req->objects, list) {
		struct vb2_buffer *vb;

		if (vb2_request_object_is_buffer(obj)) {
			vb = container_of(obj, struct vb2_buffer, req_obj);
			channel = vb2_get_drv_priv(vb->vb2_queue);
			break;
		}
	}

	if (!channel)
		return -ENOENT;

	count = vb2_request_buffer_cnt(req);
	if (!count) {
		v4l2_info(&channel->dev->v4l2_dev,
			  "No buffer was provided with the request\n");
		return -ENOENT;
	} else if (count > 1) {
		v4l2_info(&channel->dev->v4l2_dev,
			  "More than one buffer was provided with the request\n");
		return -EINVAL;
	}

	return vb2_request_validate(req);
}

static int allegro_open(struct file *file)
{
	struct video_device *vdev = video_devdata(file);
	struct allegro_dev *dev = video_get_drvdata(vdev);
	struct allegro_channel *channel = NULL;
	int ret;

	channel = kzalloc(sizeof(*channel), GFP_KERNEL);
	if (!channel)
		return -ENOMEM;

	v4l2_fh_init(&channel->fh, vdev);

	init_completion(&channel->completion);
	INIT_LIST_HEAD(&channel->src_shadow_list);
	INIT_LIST_HEAD(&channel->dst_shadow_list);
	mutex_init(&channel->shadow_list_lock);

	channel->dev = dev;
	channel->inst_type = dev->devtype->inst_type;
	channel->ops = dev->devtype->ops;

	allegro_set_default_params(channel);

	ret = allegro_ctrls_setup(channel);
	if (ret)
		goto error;

	channel->mcu_channel_id = -1;
	channel->user_id = -1;

	INIT_LIST_HEAD(&channel->buffers_reference);
	INIT_LIST_HEAD(&channel->buffers_intermediate);

	channel->fh.m2m_ctx = v4l2_m2m_ctx_init(dev->m2m_dev, channel,
						allegro_queue_init);

	if (IS_ERR(channel->fh.m2m_ctx)) {
		ret = PTR_ERR(channel->fh.m2m_ctx);
		goto error;
	}

	list_add(&channel->list, &dev->channels);
	file->private_data = &channel->fh;
	v4l2_fh_add(&channel->fh);

	if (channel->ops->init)
		channel->ops->init(channel);

	return 0;

error:
	v4l2_ctrl_handler_free(&channel->ctrl_handler);
	kfree(channel);
	return ret;
}

static int allegro_release(struct file *file)
{
	struct allegro_channel *channel = fh_to_channel(file->private_data);

	allegro_destroy_channel(channel);

	v4l2_m2m_ctx_release(channel->fh.m2m_ctx);

	list_del(&channel->list);

	v4l2_ctrl_handler_free(&channel->ctrl_handler);

	v4l2_fh_del(&channel->fh);
	v4l2_fh_exit(&channel->fh);

	kfree(channel);

	return 0;
}

static int allegro_querycap(struct file *file, void *fh,
			    struct v4l2_capability *cap)
{
	struct video_device *vdev = video_devdata(file);
	struct allegro_dev *dev = video_get_drvdata(vdev);

	const char *name = (dev->devtype->inst_type == ALLEGRO_INST_ENCODER) ?
				"Allegro DVT Video Encoder" : "Allegro DVT Video Decoder";

	strscpy(cap->driver, KBUILD_MODNAME, sizeof(cap->driver));
	strscpy(cap->card, name, sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
		 dev_name(&dev->plat_dev->dev));

	return 0;
}

static int allegro_try_pixelformat(struct allegro_channel *channel,
					struct v4l2_format *f)
{
	struct allegro_q_data *q_data;
	const u32 *formats;
	int i;

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT)
		formats = channel->dev->devtype->src_formats;
	else if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
		formats = channel->dev->devtype->dst_formats;
	else
		return -EINVAL;

	for (i = 0; i < ALLEGRO_MAX_FORMATS; i++) {
		if (formats[i] == f->fmt.pix.pixelformat) {
			f->fmt.pix.pixelformat = formats[i];
			return 0;
		}
	}

	/* Fall back to currently set pixelformat */
	q_data = get_q_data(channel, f->type);
	f->fmt.pix.pixelformat = q_data->fourcc;

	return 0;
}

static int allegro_try_fmt(struct allegro_channel *channel,
						struct v4l2_format *f)
{
	/* V4L2 specification suggests the driver corrects the format struct
	 * if any of the dimensions is unsupported */
	f->fmt.pix.field = V4L2_FIELD_NONE;

	/*
	 * The firmware of the Allegro codec handles the padding internally
	 * and expects the visual frame size when configuring a channel.
	 * Therefore, unlike other encoder drivers, this driver does not round
	 * up the width and height to macroblock alignment and does not
	 * implement the selection api.
	 */
	f->fmt.pix.width = clamp_t(__u32, f->fmt.pix.width,
				   ALLEGRO_WIDTH_MIN, ALLEGRO_WIDTH_MAX);
	f->fmt.pix.height = clamp_t(__u32, f->fmt.pix.height,
				    ALLEGRO_HEIGHT_MIN, ALLEGRO_HEIGHT_MAX);

	switch (f->fmt.pix.pixelformat) {
	case V4L2_PIX_FMT_NV12:
		/*
		 * Frame stride must be at least multiple of 8,
		 * but multiple of 16 for h.264
		 */
		f->fmt.pix.bytesperline = round_up(f->fmt.pix.width, 32);
		f->fmt.pix.sizeimage = f->fmt.pix.bytesperline *
					f->fmt.pix.height * 3 / 2;
		break;
	case V4L2_PIX_FMT_H264:
	case V4L2_PIX_FMT_H264_SLICE:
	case V4L2_PIX_FMT_HEVC:
	case V4L2_PIX_FMT_HEVC_SLICE:
		f->fmt.pix.bytesperline = 0;
		f->fmt.pix.sizeimage =
			estimate_stream_size(f->fmt.pix.width, f->fmt.pix.height);
		break;
	default:
		BUG();
	}

	return 0;
}

static int allegro_enum_fmt_vid(struct file *file, void *fh,
				struct v4l2_fmtdesc *f)
{
	struct video_device *vdev = video_devdata(file);
	struct allegro_dev *dev = video_get_drvdata(vdev);
	const u32 *formats;

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT)
		formats = dev->devtype->src_formats;
	else if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
		formats = dev->devtype->dst_formats;
	else
		return -EINVAL;

	if (f->index > ALLEGRO_MAX_FORMATS || formats[f->index] == 0)
		return -EINVAL;

	f->pixelformat = formats[f->index];

	return 0;
}

static int allegro_g_fmt_vid_cap(struct file *file, void *fh,
				 struct v4l2_format *f)
{
	struct allegro_q_data *q_data;
	struct allegro_channel *channel = fh_to_channel(fh);

	q_data = get_q_data(channel, f->type);
	if (!q_data)
		return -EINVAL;

	f->fmt.pix.field = V4L2_FIELD_NONE;
	f->fmt.pix.width = q_data->width;
	f->fmt.pix.height = q_data->height;
	f->fmt.pix.bytesperline = q_data->bytesperline;

	f->fmt.pix.colorspace = channel->colorspace;
	f->fmt.pix.ycbcr_enc = channel->ycbcr_enc;
	f->fmt.pix.quantization = channel->quantization;
	f->fmt.pix.xfer_func = channel->xfer_func;

	f->fmt.pix.pixelformat = q_data->fourcc;
	f->fmt.pix.sizeimage = q_data->sizeimage;

	return 0;
}

static int allegro_try_fmt_vid_cap(struct file *file, void *fh,
				   struct v4l2_format *f)
{
	struct allegro_channel *channel = fh_to_channel(fh);
	int ret;

	ret = allegro_try_pixelformat(channel, f);
	if (ret < 0)
		return ret;

	f->fmt.pix.colorspace = channel->colorspace;
	f->fmt.pix.xfer_func = channel->xfer_func;
	f->fmt.pix.ycbcr_enc = channel->ycbcr_enc;
	f->fmt.pix.quantization = channel->quantization;

	ret = allegro_try_fmt(channel, f);
	if (ret < 0)
		return ret;

	if (channel->inst_type == ALLEGRO_INST_DECODER) {
		f->fmt.pix.bytesperline = round_up(f->fmt.pix.width, 32);
		f->fmt.pix.height = round_up(f->fmt.pix.height, 32);
		f->fmt.pix.sizeimage = f->fmt.pix.bytesperline *
					       f->fmt.pix.height * 3 / 2;
	}

	return 0;
}

static int allegro_s_fmt(struct allegro_channel *channel,
					struct v4l2_format *f)
{
	struct allegro_dev *dev = channel->dev;
	struct allegro_q_data *q_data;
	struct vb2_queue *vq;

	vq = v4l2_m2m_get_vq(channel->fh.m2m_ctx, f->type);
	if (!vq)
		return -EINVAL;

	q_data = get_q_data(channel, f->type);
	if (!q_data)
		return -EINVAL;

	if (vb2_is_busy(vq)) {
		v4l2_err(&dev->v4l2_dev, "%s queue busy: %d\n",
			 v4l2_type_names[f->type], vq->num_buffers);
		return -EBUSY;
	}

	q_data->fourcc = f->fmt.pix.pixelformat;
	q_data->width = f->fmt.pix.width;
	q_data->height = f->fmt.pix.height;
	q_data->bytesperline = f->fmt.pix.bytesperline;
	q_data->sizeimage = f->fmt.pix.sizeimage;

	v4l2_dbg(2, debug, &dev->v4l2_dev,
		"Setting %s format, WxH: %dx%d, fmt: %4.4s\n",
		 v4l2_type_names[f->type], q_data->width, q_data->height,
		 (char *)&q_data->fourcc);

	return 0;
}

static int allegro_s_fmt_vid_cap(struct file *file, void *fh,
				 struct v4l2_format *f)
{
	struct allegro_channel *channel = fh_to_channel(fh);
	int err;

	err = allegro_try_fmt_vid_cap(file, fh, f);
	if (err)
		return err;

	err = allegro_s_fmt(channel, f);
	if (err)
		return err;

	if (channel->inst_type != ALLEGRO_INST_ENCODER)
		return 0;

	channel->colorspace = f->fmt.pix.colorspace;
	channel->xfer_func = f->fmt.pix.xfer_func;
	channel->ycbcr_enc = f->fmt.pix.ycbcr_enc;
	channel->quantization = f->fmt.pix.quantization;

	//allegro_channel_adjust(channel);

	return 0;
}

static int allegro_g_fmt_vid_out(struct file *file, void *fh,
				 struct v4l2_format *f)
{
	struct allegro_channel *channel = fh_to_channel(fh);
	struct allegro_q_data *q_data;

	q_data = get_q_data(channel, f->type);
	if (!q_data)
		return -EINVAL;

	f->fmt.pix.field = V4L2_FIELD_NONE;
	f->fmt.pix.pixelformat = q_data->fourcc;
	f->fmt.pix.width = q_data->width;
	f->fmt.pix.height = q_data->height;
	f->fmt.pix.bytesperline = q_data->bytesperline;

	f->fmt.pix.sizeimage = q_data->sizeimage;
	f->fmt.pix.colorspace = channel->colorspace;
	f->fmt.pix.ycbcr_enc = channel->ycbcr_enc;
	f->fmt.pix.quantization = channel->quantization;
	f->fmt.pix.xfer_func = channel->xfer_func;

	return 0;
}

static int allegro_try_fmt_vid_out(struct file *file, void *fh,
				   struct v4l2_format *f)
{
	struct allegro_channel *channel = fh_to_channel(fh);
	int ret;

	ret = allegro_try_pixelformat(channel, f);
	if (ret < 0)
		return ret;

	return allegro_try_fmt(channel, f);
}

static int allegro_s_fmt_vid_out(struct file *file, void *fh,
				 struct v4l2_format *f)
{
	struct allegro_channel *channel = fh_to_channel(fh);
	struct allegro_q_data *q_data;
	struct vb2_queue *vq;
	int err;

	q_data = get_q_data(channel, f->type);
	if (!q_data)
		return -EINVAL;

	err = allegro_try_fmt_vid_out(file, fh, f);
	if (err)
		return err;

	vq = v4l2_m2m_get_vq(channel->fh.m2m_ctx, f->type);

	switch (f->fmt.pix.pixelformat) {
	case V4L2_PIX_FMT_H264_SLICE:
	case V4L2_PIX_FMT_HEVC_SLICE:
		vq->subsystem_flags |=
			VB2_V4L2_FL_SUPPORTS_M2M_HOLD_CAPTURE_BUF;
		break;
	default:
		vq->subsystem_flags &=
			~VB2_V4L2_FL_SUPPORTS_M2M_HOLD_CAPTURE_BUF;
		break;
	}

	q_data->width = f->fmt.pix.width;
	q_data->height = f->fmt.pix.height;
	q_data->bytesperline = f->fmt.pix.bytesperline;
	q_data->sizeimage = f->fmt.pix.sizeimage;

	channel->colorspace = f->fmt.pix.colorspace;
	channel->ycbcr_enc = f->fmt.pix.ycbcr_enc;
	channel->quantization = f->fmt.pix.quantization;
	channel->xfer_func = f->fmt.pix.xfer_func;

	//allegro_channel_adjust(channel);

	return 0;
}

static int allegro_channel_cmd_stop(struct allegro_channel *channel)
{
	if (v4l2_m2m_has_stopped(channel->fh.m2m_ctx))
		allegro_channel_eos_event(channel);

	return 0;
}

static int allegro_channel_cmd_start(struct allegro_channel *channel)
{
	if (v4l2_m2m_has_stopped(channel->fh.m2m_ctx))
		vb2_clear_last_buffer_dequeued(&channel->fh.m2m_ctx->cap_q_ctx.q);

	return 0;
}

static int allegro_encoder_cmd(struct file *file, void *fh,
			       struct v4l2_encoder_cmd *cmd)
{
	struct allegro_channel *channel = fh_to_channel(fh);
	int err;

	err = v4l2_m2m_ioctl_try_encoder_cmd(file, fh, cmd);
	if (err)
		return err;

	err = v4l2_m2m_ioctl_encoder_cmd(file, fh, cmd);
	if (err)
		return err;

	if (cmd->cmd == V4L2_ENC_CMD_STOP)
		err = allegro_channel_cmd_stop(channel);

	if (cmd->cmd == V4L2_ENC_CMD_START)
		err = allegro_channel_cmd_start(channel);

	return err;
}

static int allegro_decoder_cmd(struct file *file, void *fh,
			       struct v4l2_decoder_cmd *cmd)
{
	struct allegro_channel *channel = fh_to_channel(fh);
	int err;

	err = v4l2_m2m_ioctl_stateless_try_decoder_cmd(file, fh, cmd);
	if (err)
		return err;

	err = v4l2_m2m_ioctl_stateless_decoder_cmd(file, fh, cmd);
	if (err)
		return err;

	if (cmd->cmd == V4L2_DEC_CMD_STOP)
		err = allegro_channel_cmd_stop(channel);

	if (cmd->cmd == V4L2_DEC_CMD_START)
		err = allegro_channel_cmd_start(channel);

	return err;
}

static int allegro_enum_framesizes(struct file *file, void *fh,
				   struct v4l2_frmsizeenum *fsize)
{
	struct allegro_channel *channel = fh_to_channel(fh);

	if (channel->inst_type != ALLEGRO_INST_ENCODER)
		return -ENOTTY;

	switch (fsize->pixel_format) {
	case V4L2_PIX_FMT_HEVC:
	case V4L2_PIX_FMT_H264:
	case V4L2_PIX_FMT_HEVC_SLICE:
	case V4L2_PIX_FMT_H264_SLICE:
	case V4L2_PIX_FMT_NV12:
		break;
	default:
		return -EINVAL;
	}

	if (fsize->index)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_CONTINUOUS;
	fsize->stepwise.min_width = ALLEGRO_WIDTH_MIN;
	fsize->stepwise.max_width = ALLEGRO_WIDTH_MAX;
	fsize->stepwise.step_width = 1;
	fsize->stepwise.min_height = ALLEGRO_HEIGHT_MIN;
	fsize->stepwise.max_height = ALLEGRO_HEIGHT_MAX;
	fsize->stepwise.step_height = 1;

	return 0;
}

static int allegro_ioctl_streamon(struct file *file, void *priv,
				  enum v4l2_buf_type type)
{
	struct v4l2_fh *fh = file->private_data;
	struct allegro_channel *channel = fh_to_channel(fh);
	int err;

	err = allegro_create_channel(channel);
	if (err)
		return err;

	return v4l2_m2m_streamon(file, fh->m2m_ctx, type);
}

static int allegro_g_parm(struct file *file, void *fh,
			  struct v4l2_streamparm *a)
{
	struct allegro_channel *channel = fh_to_channel(fh);
	struct v4l2_fract *timeperframe;

	if (a->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return -EINVAL;

	a->parm.output.capability = V4L2_CAP_TIMEPERFRAME;
	timeperframe = &a->parm.output.timeperframe;
	timeperframe->numerator = channel->framerate.denominator;
	timeperframe->denominator = channel->framerate.numerator;

	return 0;
}

static int allegro_s_parm(struct file *file, void *fh,
			  struct v4l2_streamparm *a)
{
	struct allegro_channel *channel = fh_to_channel(fh);
	struct v4l2_fract *timeperframe;
	int div;

	if (a->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return -EINVAL;

	a->parm.output.capability = V4L2_CAP_TIMEPERFRAME;
	timeperframe = &a->parm.output.timeperframe;

	if (timeperframe->numerator == 0 || timeperframe->denominator == 0)
		return allegro_g_parm(file, fh, a);

	div = gcd(timeperframe->denominator, timeperframe->numerator);
	channel->framerate.numerator = timeperframe->denominator / div;
	channel->framerate.denominator = timeperframe->numerator / div;

	return 0;
}

static int allegro_subscribe_event(struct v4l2_fh *fh,
				   const struct v4l2_event_subscription *sub)
{
	struct allegro_channel *channel = fh_to_channel(fh);

	switch (sub->type) {
	case V4L2_EVENT_EOS:
		return v4l2_event_subscribe(fh, sub, 0, NULL);
	case V4L2_EVENT_SOURCE_CHANGE:
		if (channel->inst_type == ALLEGRO_INST_DECODER)
			return v4l2_event_subscribe(fh, sub, 0, NULL);
		else
			return -EINVAL;
	default:
		return v4l2_ctrl_subscribe_event(fh, sub);
	}
}

static const struct v4l2_ioctl_ops allegro_ioctl_ops = {
	.vidioc_querycap 			= allegro_querycap,

	.vidioc_enum_fmt_vid_cap 	= allegro_enum_fmt_vid,
	.vidioc_g_fmt_vid_cap 		= allegro_g_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap 	= allegro_try_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap 		= allegro_s_fmt_vid_cap,

	.vidioc_enum_fmt_vid_out 	= allegro_enum_fmt_vid,
	.vidioc_g_fmt_vid_out 		= allegro_g_fmt_vid_out,
	.vidioc_try_fmt_vid_out 	= allegro_try_fmt_vid_out,
	.vidioc_s_fmt_vid_out 		= allegro_s_fmt_vid_out,

	.vidioc_reqbufs 		= v4l2_m2m_ioctl_reqbufs,
	.vidioc_expbuf 			= v4l2_m2m_ioctl_expbuf,
	.vidioc_querybuf 		= v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf 			= v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf 			= v4l2_m2m_ioctl_dqbuf,
	.vidioc_prepare_buf 	= v4l2_m2m_ioctl_prepare_buf,
	.vidioc_create_bufs 	= v4l2_m2m_ioctl_create_bufs,

	.vidioc_streamon 		= allegro_ioctl_streamon,
	.vidioc_streamoff 		= v4l2_m2m_ioctl_streamoff,

	.vidioc_try_encoder_cmd = v4l2_m2m_ioctl_try_encoder_cmd,
	.vidioc_encoder_cmd 	= allegro_encoder_cmd,

	.vidioc_try_decoder_cmd = v4l2_m2m_ioctl_stateless_try_decoder_cmd,
	.vidioc_decoder_cmd 	= allegro_decoder_cmd,

	.vidioc_enum_framesizes = allegro_enum_framesizes,

	.vidioc_g_parm		= allegro_g_parm,
	.vidioc_s_parm		= allegro_s_parm,

	.vidioc_subscribe_event = allegro_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

static const struct v4l2_file_operations allegro_fops = {
	.owner = THIS_MODULE,
	.open = allegro_open,
	.release = allegro_release,
	.poll = v4l2_m2m_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = v4l2_m2m_fop_mmap,
};

static int allegro_register_device(struct allegro_dev *dev)
{
	struct video_device *video_dev = &dev->video_dev;

	strscpy(video_dev->name, "allegro", sizeof(video_dev->name));
	video_dev->fops = &allegro_fops;
	video_dev->ioctl_ops = &allegro_ioctl_ops;
	video_dev->release = video_device_release_empty;
	video_dev->lock = &dev->lock;
	video_dev->v4l2_dev = &dev->v4l2_dev;
	video_dev->vfl_dir = VFL_DIR_M2M;
	video_dev->device_caps = V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING;
	video_set_drvdata(video_dev, dev);

	return video_register_device(video_dev, VFL_TYPE_VIDEO, 0);
}

static void allegro_device_run(void *priv)
{
	struct allegro_channel *channel = priv;
	//struct allegro_dev *dev = channel->dev;

	channel->ops->run(channel);

	//v4l2_m2m_job_finish(dev->m2m_dev, channel->fh.m2m_ctx);
	//v4l2_m2m_buf_done_and_job_finish(dev->m2m_dev,
	//				channel->fh.m2m_ctx, VB2_BUF_STATE_DONE);
}

static const struct v4l2_m2m_ops allegro_m2m_ops = {
	.device_run = allegro_device_run,
};

static const struct media_device_ops allegro_m2m_media_ops = {
	.req_validate	= allegro_request_validate,
	.req_queue	= v4l2_m2m_request_queue,
};

static int allegro_mcu_hw_init(struct allegro_dev *dev,
			       const struct fw_info *info)
{
	int err;

	dev->mbox_command = allegro_mbox_init(dev, info->mailbox_cmd,
					      info->mailbox_size);
	dev->mbox_status = allegro_mbox_init(dev, info->mailbox_status,
					     info->mailbox_size);
	if (IS_ERR(dev->mbox_command) || IS_ERR(dev->mbox_status)) {
		v4l2_err(&dev->v4l2_dev,
			 "failed to initialize mailboxes\n");
		return -EIO;
	}

	allegro_mcu_enable_interrupts(dev);

	/* The mcu sends INIT after reset. */
	allegro_mcu_start(dev);
	err = allegro_mcu_wait_for_init_timeout(dev, 5000);
	if (err < 0) {
		v4l2_err(&dev->v4l2_dev,
			 "mcu did not send INIT after reset\n");
		err = -EIO;
		goto err_disable_interrupts;
	}

	err = allegro_alloc_buffer(dev, &dev->suballocator,
				   info->suballocator_size);
	if (err) {
		v4l2_err(&dev->v4l2_dev,
			 "failed to allocate %zu bytes for suballocator\n",
			 info->suballocator_size);
		goto err_reset_mcu;
	}

	allegro_mcu_send_init(dev, dev->suballocator.paddr,
			      dev->suballocator.size);
	err = allegro_mcu_wait_for_init_timeout(dev, 5000);
	if (err < 0) {
		v4l2_err(&dev->v4l2_dev,
			 "mcu failed to configure sub-allocator\n");
		err = -EIO;
		goto err_free_suballocator;
	}

	return 0;

err_free_suballocator:
	allegro_free_buffer(dev, &dev->suballocator);
err_reset_mcu:
	allegro_mcu_reset(dev);
err_disable_interrupts:
	allegro_mcu_disable_interrupts(dev);

	return err;
}

static int allegro_mcu_hw_deinit(struct allegro_dev *dev)
{
	int err;

	err = allegro_mcu_reset(dev);
	if (err)
		v4l2_warn(&dev->v4l2_dev,
			  "mcu failed to enter sleep state\n");

	err = allegro_mcu_disable_interrupts(dev);
	if (err)
		v4l2_warn(&dev->v4l2_dev,
			  "failed to disable interrupts\n");

	allegro_free_buffer(dev, &dev->suballocator);

	return 0;
}

static void allegro_fw_callback(const struct firmware *fw, void *context)
{
	struct allegro_dev *dev = context;
	const char *fw_codec_name = dev->devtype->fw;
	const struct firmware *fw_codec;
	int err;

	if (!fw)
		return;

	v4l2_dbg(1, debug, &dev->v4l2_dev,
		 "requesting codec firmware '%s'\n", fw_codec_name);
	err = request_firmware(&fw_codec, fw_codec_name, &dev->plat_dev->dev);
	if (err)
		goto err_release_firmware;

	dev->fw_info = allegro_get_firmware_info(dev, fw, fw_codec);
	if (!dev->fw_info) {
		v4l2_err(&dev->v4l2_dev, "firmware is not supported\n");
		goto err_release_firmware_codec;
	}

	v4l2_info(&dev->v4l2_dev,
		  "using mcu firmware version '%s'\n", dev->fw_info->version);

	/* Ensure that the mcu is sleeping at the reset vector */
	err = allegro_mcu_reset(dev);
	if (err) {
		v4l2_err(&dev->v4l2_dev, "failed to reset mcu\n");
		goto err_release_firmware_codec;
	}

	allegro_copy_firmware(dev, fw->data, fw->size);
	allegro_copy_fw_codec(dev, fw_codec->data, fw_codec->size);

	err = allegro_mcu_hw_init(dev, dev->fw_info);
	if (err) {
		v4l2_err(&dev->v4l2_dev, "failed to initialize mcu\n");
		goto err_free_fw_codec;
	}

	dev->m2m_dev = v4l2_m2m_init(&allegro_m2m_ops);
	if (IS_ERR(dev->m2m_dev)) {
		v4l2_err(&dev->v4l2_dev, "failed to init mem2mem device\n");
		goto err_mcu_hw_deinit;
	}

#ifdef CONFIG_MEDIA_CONTROLLER
	if (dev->devtype->inst_type == ALLEGRO_INST_DECODER) {
		dev->mdev.dev = &dev->plat_dev->dev;
		strscpy(dev->mdev.model, "allegro-decoder", sizeof(dev->mdev.model));
		strscpy(dev->mdev.bus_info, "platform:allegro-decoder",
			sizeof(dev->mdev.bus_info));
		media_device_init(&dev->mdev);
		dev->mdev.ops = &allegro_m2m_media_ops;
		dev->v4l2_dev.mdev = &dev->mdev;
	}
#endif

	err = allegro_register_device(dev);
	if (err) {
		v4l2_err(&dev->v4l2_dev, "failed to register video device\n");
		goto err_m2m_release;
	}

	v4l2_dbg(1, debug, &dev->v4l2_dev,
		 "Video device registered as /dev/video%d\n",
		 dev->video_dev.num);

#ifdef CONFIG_MEDIA_CONTROLLER
	if (dev->devtype->inst_type == ALLEGRO_INST_DECODER) {
		err = v4l2_m2m_register_media_controller(dev->m2m_dev,
					&dev->video_dev, MEDIA_ENT_F_PROC_VIDEO_DECODER);
		if (err) {
			v4l2_err(&dev->v4l2_dev,
				 "failed to initialize V4L2 M2M media controller\n");
			goto err_video_unregister;
		}

		err = media_device_register(&dev->mdev);
		if (err) {
			v4l2_err(&dev->v4l2_dev, "failed to register media device\n");
			goto err_m2m_mc_unregister;
		}

		v4l2_dbg(1, debug, &dev->v4l2_dev,
			 "Media device registered as /dev/media%d\n",
			 dev->mdev.devnode->minor);
	}
#endif

	release_firmware(fw_codec);
	release_firmware(fw);

	return;

#ifdef CONFIG_MEDIA_CONTROLLER
err_m2m_mc_unregister:
	v4l2_m2m_unregister_media_controller(dev->m2m_dev);
#endif
err_video_unregister:
	video_unregister_device(&dev->video_dev);
err_m2m_release:
	v4l2_m2m_release(dev->m2m_dev);
	dev->m2m_dev = NULL;
err_mcu_hw_deinit:
	allegro_mcu_hw_deinit(dev);
err_free_fw_codec:
	allegro_free_fw_codec(dev);
err_release_firmware_codec:
	release_firmware(fw_codec);
err_release_firmware:
	release_firmware(fw);
}

static int allegro_firmware_request_nowait(struct allegro_dev *dev)
{
	const char *fw = dev->devtype->fwb;

	v4l2_dbg(1, debug, &dev->v4l2_dev,
		 "requesting firmware '%s'\n", fw);
	return request_firmware_nowait(THIS_MODULE, true, fw,
				       &dev->plat_dev->dev, GFP_KERNEL, dev,
				       allegro_fw_callback);
}

static void allegro_probe_mem_region(struct platform_device *pdev)
{
	struct device_node *mem_node;
	struct resource mem_res;
	unsigned long pgtable_padding;
	int ret;

	mem_node = of_parse_phandle(pdev->dev.of_node, "memory-region", 0);
	if (!mem_node)
		return;

	ret = of_address_to_resource(mem_node, 0, &mem_res);
	if (ret)
		goto node_put;

	ret = of_reserved_mem_device_init(&pdev->dev);
	if (ret) {
		dev_err(&pdev->dev,
			"failed to get shared dma pool: %d\n", ret);
		goto node_put;
	}

	dev_info(&pdev->dev, "using shared dma pool for allocation\n");

	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));
	if (ret) {
		dev_err(&pdev->dev, "dma_set_mask_and_coherent: %d\n", ret);
		of_reserved_mem_device_release(&pdev->dev);
		goto node_put;
	}

#ifdef CONFIG_MEMORY_HOTPLUG
	/* Hotplug requires 0x40000000 alignment so round to nearest multiple */
	if (resource_size(&mem_res) % HOTPLUG_ALIGN)
		pgtable_padding = HOTPLUG_ALIGN -
			(resource_size(&mem_res) % HOTPLUG_ALIGN);
	else
		pgtable_padding = 0;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)
	add_memory(0, mem_res.start, resource_size(&mem_res) +
		   pgtable_padding, MHP_NONE);
#else
	add_memory(0, mem_res.start, resource_size(&mem_res) +
		   pgtable_padding);
#endif
#endif

node_put:
	of_node_put(mem_node);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0)

int allegro_clk_setup(struct platform_device *pdev, struct allegro_dev *dev)
{
	u32 refclk, coreclk, mcuclk, inte, deci;
	int err;

	dev->pll_ref = devm_clk_get(&pdev->dev, "pll_ref");
	if (IS_ERR(dev->pll_ref)) {
		if (PTR_ERR(dev->pll_ref) != -EPROBE_DEFER)
			dev_err(&pdev->dev, "Could not get pll_ref clock\n");
		return PTR_ERR(dev->pll_ref);
	}

	dev->core_clk = devm_clk_get(&pdev->dev, "core_clk");
	if (IS_ERR(dev->core_clk)) {
		if (PTR_ERR(dev->core_clk) != -EPROBE_DEFER)
			dev_err(&pdev->dev, "Could not get core clock\n");
		return PTR_ERR(dev->core_clk);
	}

	dev->mcu_clk = devm_clk_get(&pdev->dev, "mcu_clk");
	if (IS_ERR(dev->mcu_clk)) {
		if (PTR_ERR(dev->mcu_clk) != -EPROBE_DEFER)
			dev_err(&pdev->dev, "Could not get mcu clock\n");
		return PTR_ERR(dev->mcu_clk);
	}

	regmap_read(dev->logicore, VCU_PLL_CLK, &inte);
	regmap_read(dev->logicore, VCU_PLL_CLK_DEC, &deci);
	regmap_read(dev->logicore, VCU_CORE_CLK, &coreclk);
	regmap_read(dev->logicore, VCU_MCU_CLK, &mcuclk);

	if (!mcuclk || !coreclk) {
		dev_err(&pdev->dev, "Invalid mcu and core clock data\n");
		return -EINVAL;
	}

	refclk = (inte * MHZ) + (deci * (MHZ / FRAC));
	coreclk *= MHZ;
	mcuclk *= MHZ;
	dev_dbg(&pdev->dev, "Ref clock from logicoreIP is %uHz\n", refclk);
	dev_dbg(&pdev->dev, "Core clock from logicoreIP is %uHz\n", coreclk);
	dev_dbg(&pdev->dev, "Mcu clock from logicoreIP is %uHz\n", mcuclk);

	err = clk_set_rate(dev->pll_ref, refclk);
	if (err)
		dev_warn(&pdev->dev, "failed to set logicoreIP refclk rate %d\n"
			 , err);

	err = clk_prepare_enable(dev->pll_ref);
	if (err) {
		dev_err(&pdev->dev, "failed to enable pll_ref clk source %d\n",
			err);
		return err;
	}

	err = clk_set_rate(dev->mcu_clk, mcuclk);
	if (err)
		dev_warn(&pdev->dev, "failed to set logicoreIP mcu clk rate "
			 "%d\n", err);

	err = clk_prepare_enable(dev->mcu_clk);
	if (err) {
		dev_err(&pdev->dev, "failed to enable mcu %d\n", err);
		goto error_mcu;
	}

	err = clk_set_rate(dev->core_clk, coreclk);
	if (err)
		dev_warn(&pdev->dev, "failed to set logicoreIP core clk rate "
			 "%d\n", err);

	err = clk_prepare_enable(dev->core_clk);
	if (err) {
		dev_err(&pdev->dev, "failed to enable core %d\n", err);
		goto error_core;
	}
	return 0;

error_core:
	clk_disable_unprepare(dev->mcu_clk);
error_mcu:
	clk_disable_unprepare(dev->pll_ref);

	return err;

}

int allegro_clk_cleanup(struct platform_device *pdev, struct allegro_dev *dev)
{
	clk_disable_unprepare(dev->core_clk);
	devm_clk_put(&pdev->dev, dev->core_clk);

	clk_disable_unprepare(dev->mcu_clk);
	devm_clk_put(&pdev->dev, dev->mcu_clk);

	clk_disable_unprepare(dev->pll_ref);
	devm_clk_put(&pdev->dev, dev->pll_ref);

	return 0;
}

#endif

static const struct allegro_devtype allegro_devdata[] = {
	[ALLEGRO_INST_ENCODER] = {
		.name = "allegro-enc",
		.inst_type = ALLEGRO_INST_ENCODER,
		.fwb = "al5e_b.fw",
		.fw = "al5e.fw",
		.fwinfos = fw_info_enc,
		.num_fwinfos = ARRAY_SIZE(fw_info_enc),
		.ctrls = ctrls_enc,
		.num_ctrls = ARRAY_SIZE(ctrls_enc),
		.ops = &allegro_enc_ops,
		.src_formats = {
			V4L2_PIX_FMT_NV12,
		},
		.dst_formats = {
			V4L2_PIX_FMT_H264,
			V4L2_PIX_FMT_HEVC,
		},
	},
	[ALLEGRO_INST_DECODER] = {
		.name = "allegro-dec",
		.inst_type = ALLEGRO_INST_DECODER,
		.fwb = "al5d_b.fw",
		.fw = "al5d.fw",
		.fwinfos = fw_info_dec,
		.num_fwinfos = ARRAY_SIZE(fw_info_dec),
		.ctrls = ctrls_dec,
		.num_ctrls = ARRAY_SIZE(ctrls_dec),
		.ops = &allegro_dec_ops,
		.src_formats = {
			V4L2_PIX_FMT_H264_SLICE,
			V4L2_PIX_FMT_HEVC_SLICE,
		},
		.dst_formats = {
			V4L2_PIX_FMT_NV12,
		},
	},
};

static const struct of_device_id allegro_dt_ids[] = {
	{ .compatible = "allegro,al5e-1.1",
	  .data = &allegro_devdata[ALLEGRO_INST_ENCODER] },
	{ .compatible = "allegro,al5d-1.1",
	  .data = &allegro_devdata[ALLEGRO_INST_DECODER] },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, allegro_dt_ids);

static int allegro_probe(struct platform_device *pdev)
{
	const struct of_device_id *of_id =
			of_match_device(of_match_ptr(allegro_dt_ids), &pdev->dev);
	struct allegro_dev *dev;
	struct resource *res;
	void __iomem *regs;
	int ret;
	int irq;

	if (!of_id)
		return -EINVAL;

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;
	dev->plat_dev = pdev;
	dev->devtype = (struct allegro_devtype *) of_id->data;

	init_completion(&dev->init_complete);
	INIT_LIST_HEAD(&dev->channels);

	mutex_init(&dev->lock);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "regs");
	if (!res) {
		dev_err(&pdev->dev,
			"regs resource missing from device tree\n");
		return -EINVAL;
	}
	regs = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!regs) {
		dev_err(&pdev->dev, "failed to map registers\n");
		return -ENOMEM;
	}
	dev->regmap = devm_regmap_init_mmio(&pdev->dev, regs,
					    &allegro_regmap_config);
	if (IS_ERR(dev->regmap)) {
		dev_err(&pdev->dev, "failed to init regmap\n");
		return PTR_ERR(dev->regmap);
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "sram");
	if (!res) {
		dev_err(&pdev->dev,
			"sram resource missing from device tree\n");
		return -EINVAL;
	}
	regs = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!regs) {
		dev_err(&pdev->dev, "failed to map sram\n");
		return -ENOMEM;
	}
	dev->sram = devm_regmap_init_mmio(&pdev->dev, regs,
					  &allegro_sram_config);
	if (IS_ERR(dev->sram)) {
		dev_err(&pdev->dev, "failed to init sram\n");
		return PTR_ERR(dev->sram);
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 15, 0)
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "logicore");
	if (!res) {
		dev_err(&pdev->dev,
			"logicore resource missing from device tree\n");
		return -EINVAL;
	}
	regs = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!regs) {
		dev_err(&pdev->dev, "failed to map logicore\n");
		return -ENOMEM;
	}
	dev->logicore = devm_regmap_init_mmio(&pdev->dev, regs,
					  &allegro_logicore_config);
	if (IS_ERR(dev->sram)) {
		dev_err(&pdev->dev, "failed to init sram\n");
		return PTR_ERR(dev->sram);
	}

	ret = allegro_clk_setup(pdev, dev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to setup clock");
		return ret;
	}
#endif

	allegro_probe_mem_region(pdev);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	ret = devm_request_threaded_irq(&pdev->dev, irq,
					allegro_hardirq,
					allegro_irq_thread,
					IRQF_SHARED, dev_name(&pdev->dev), dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to request irq: %d\n", ret);
		return ret;
	}

	ret = v4l2_device_register(&pdev->dev, &dev->v4l2_dev);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, dev);

	ret = allegro_firmware_request_nowait(dev);
	if (ret < 0) {
		v4l2_err(&dev->v4l2_dev,
			 "failed to request firmware: %d\n", ret);
		return ret;
	}

	return 0;
}

static int allegro_remove(struct platform_device *pdev)
{
	struct allegro_dev *dev = platform_get_drvdata(pdev);

#ifdef CONFIG_MEDIA_CONTROLLER
	if (dev->devtype->inst_type == ALLEGRO_INST_DECODER) {
		if (media_devnode_is_registered(dev->mdev.devnode)) {
			media_device_unregister(&dev->mdev);
			v4l2_m2m_unregister_media_controller(dev->m2m_dev);
			media_device_cleanup(&dev->mdev);
		}
	}
#endif

	video_unregister_device(&dev->video_dev);
	if (dev->m2m_dev)
		v4l2_m2m_release(dev->m2m_dev);
	allegro_mcu_hw_deinit(dev);
	allegro_free_fw_codec(dev);

	allegro_clk_cleanup(pdev, dev);

	of_reserved_mem_device_release(&pdev->dev);

	v4l2_device_unregister(&dev->v4l2_dev);

	return 0;
}

static struct platform_driver allegro_driver = {
	.probe = allegro_probe,
	.remove = allegro_remove,
	.driver = {
		.name = "allegro-dvt",
		.of_match_table = of_match_ptr(allegro_dt_ids),
	},
};

module_platform_driver(allegro_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Michael Tretter <kernel@pengutronix.de>");
MODULE_AUTHOR("Brainlab,VIED <deji.aribuki@brainlab.com>");
MODULE_DESCRIPTION("Allegro DVT codec driver");
