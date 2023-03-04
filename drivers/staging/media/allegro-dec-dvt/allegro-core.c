// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2023 Brainlab, VIED <deji.aribuki.ext@brainlab.com>
 *
 * Allegro DVT video decoder driver
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

#include <linux/version.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>

#include "allegro-mail.h"

#ifdef CONFIG_MEMORY_HOTPLUG
#define HOTPLUG_ALIGN 0x40000000
#endif

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

struct allegro_dev {
    struct v4l2_device v4l2_dev;
    struct video_device video_dev;
    struct v4l2_m2m_dev *m2m_dev;
    struct platform_device *plat_dev;

    /* mutex protecting vb2_queue structure */
    struct mutex lock;

    struct regmap *regmap;
    struct regmap *sram;

    const struct fw_info *fw_info;
    struct allegro_buffer firmware;
    struct allegro_buffer suballocator;

    struct completion init_complete;

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

#define fh_to_channel(__fh) container_of(__fh, struct allegro_channel, fh)

struct allegro_channel {
    struct allegro_dev *dev;
    struct v4l2_fh fh;
    struct v4l2_ctrl_handler ctrl_handler;

    struct allegro_buffer config_blob;

    unsigned int width;
    unsigned int height;
    u32 codec;

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

static const struct fw_info supported_firmware[] = {
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
        .mailbox_version = MCU_MSG_VERSION_2019_2,
        .suballocator_size = SZ_32M,
    },
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

static unsigned long allegro_next_user_id(struct allegro_dev *dev)
{
    if (dev->channel_user_ids == ~0UL)
        return -EBUSY;

    return ffz(dev->channel_user_ids);
}

static struct allegro_channel *
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

static struct allegro_channel *
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

#define AL_ERROR            0x80
#define AL_ERR_INIT_FAILED      0x81
#define AL_ERR_NO_FRAME_DECODED     0x82
#define AL_ERR_RESOLUTION_CHANGE    0x85
#define AL_ERR_NO_MEMORY        0x87
#define AL_ERR_STREAM_OVERFLOW      0x88
#define AL_ERR_TOO_MANY_SLICES      0x89
#define AL_ERR_BUF_NOT_READY        0x8c
#define AL_ERR_NO_CHANNEL_AVAILABLE 0x8d
#define AL_ERR_RESOURCE_UNAVAILABLE 0x8e
#define AL_ERR_NOT_ENOUGH_CORES     0x8f
#define AL_ERR_REQUEST_MALFORMED    0x90
#define AL_ERR_CMD_NOT_ALLOWED      0x91
#define AL_ERR_INVALID_CMD_VALUE    0x92

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

static const struct fw_info *
allegro_get_firmware_info(struct allegro_dev *dev,
              const struct firmware *fw,
              const struct firmware *fw_codec)
{
    int i;
    unsigned int id = fw->size;
    unsigned int id_codec = fw_codec->size;

    for (i = 0; i < ARRAY_SIZE(supported_firmware); i++)
        if (supported_firmware[i].id == id &&
            supported_firmware[i].id_codec == id_codec)
            return &supported_firmware[i];

    return NULL;
}

/*
 * Buffers that are used internally by the MCU.
 */

static int allegro_alloc_buffer(struct allegro_dev *dev,
                struct allegro_buffer *buffer, size_t size)
{
    buffer->vaddr = dma_alloc_coherent(&dev->plat_dev->dev, size,
                       &buffer->paddr, GFP_KERNEL);
    if (!buffer->vaddr)
        return -ENOMEM;
    buffer->size = size;

    return 0;
}

static void allegro_free_buffer(struct allegro_dev *dev,
                struct allegro_buffer *buffer)
{
    if (buffer->vaddr) {
        dma_free_coherent(&dev->plat_dev->dev, buffer->size,
                  buffer->vaddr, buffer->paddr);
        buffer->vaddr = NULL;
        buffer->size = 0;
    }
}

/*
 * Mailbox interface to send messages to the MCU.
 */

static void allegro_mcu_interrupt(struct allegro_dev *dev);
static void allegro_handle_message(struct allegro_dev *dev,
                   union mcu_msg_response *msg);

static struct allegro_mbox *allegro_mbox_init(struct allegro_dev *dev,
                          unsigned int base, size_t size)
{
    struct allegro_mbox *mbox;

    mbox = devm_kmalloc(&dev->plat_dev->dev, sizeof(*mbox), GFP_KERNEL);
    if (!mbox)
        return ERR_PTR(-ENOMEM);

    mbox->dev = dev;

    mbox->head = base;
    mbox->tail = base + 0x4;
    mbox->data = base + 0x8;
    mbox->size = size;
    mutex_init(&mbox->lock);

    regmap_write(dev->sram, mbox->head, 0);
    regmap_write(dev->sram, mbox->tail, 0);

    return mbox;
}

static int allegro_mbox_write(struct allegro_mbox *mbox,
                  const u32 *src, size_t size)
{
    struct regmap *sram = mbox->dev->sram;
    unsigned int tail;
    size_t size_no_wrap;
    int err = 0;
    int stride = regmap_get_reg_stride(sram);

    if (!src)
        return -EINVAL;

    if (size > mbox->size)
        return -EINVAL;

    mutex_lock(&mbox->lock);
    regmap_read(sram, mbox->tail, &tail);
    if (tail > mbox->size) {
        err = -EIO;
        goto out;
    }
    size_no_wrap = min(size, mbox->size - (size_t)tail);
    regmap_bulk_write(sram, mbox->data + tail,
              src, size_no_wrap / stride);
    regmap_bulk_write(sram, mbox->data,
              src + (size_no_wrap / sizeof(*src)),
              (size - size_no_wrap) / stride);
    regmap_write(sram, mbox->tail, (tail + size) % mbox->size);

out:
    mutex_unlock(&mbox->lock);

    return err;
}

static ssize_t allegro_mbox_read(struct allegro_mbox *mbox,
                 u32 *dst, size_t nbyte)
{
    struct {
        u16 length;
        u16 type;
    } __attribute__ ((__packed__)) *header;
    struct regmap *sram = mbox->dev->sram;
    unsigned int head;
    ssize_t size;
    size_t body_no_wrap;
    int stride = regmap_get_reg_stride(sram);

    regmap_read(sram, mbox->head, &head);
    if (head > mbox->size)
        return -EIO;

    /* Assume that the header does not wrap. */
    regmap_bulk_read(sram, mbox->data + head,
             dst, sizeof(*header) / stride);
    header = (void *)dst;
    size = header->length + sizeof(*header);
    if (size > mbox->size || size & 0x3)
        return -EIO;
    if (size > nbyte)
        return -EINVAL;

    /*
     * The message might wrap within the mailbox. If the message does not
     * wrap, the first read will read the entire message, otherwise the
     * first read will read message until the end of the mailbox and the
     * second read will read the remaining bytes from the beginning of the
     * mailbox.
     *
     * Skip the header, as was already read to get the size of the body.
     */
    body_no_wrap = min((size_t)header->length,
               (size_t)(mbox->size - (head + sizeof(*header))));
    regmap_bulk_read(sram, mbox->data + head + sizeof(*header),
             dst + (sizeof(*header) / sizeof(*dst)),
             body_no_wrap / stride);
    regmap_bulk_read(sram, mbox->data,
             dst + (sizeof(*header) + body_no_wrap) / sizeof(*dst),
             (header->length - body_no_wrap) / stride);

    regmap_write(sram, mbox->head, (head + size) % mbox->size);

    return size;
}

/**
 * allegro_mbox_send() - Send a message via the mailbox
 * @mbox: the mailbox which is used to send the message
 * @msg: the message to send
 */
static int allegro_mbox_send(struct allegro_mbox *mbox, void *msg)
{
    struct allegro_dev *dev = mbox->dev;
    ssize_t size;
    int err;
    u32 *tmp;

    tmp = kzalloc(mbox->size, GFP_KERNEL);
    if (!tmp) {
        err = -ENOMEM;
        goto out;
    }

    size = allegro_encode_mail(tmp, msg);

    err = allegro_mbox_write(mbox, tmp, size);
    kfree(tmp);
    if (err)
        goto out;

    allegro_mcu_interrupt(dev);

out:
    return err;
}

/**
 * allegro_mbox_notify() - Notify the mailbox about a new message
 * @mbox: The allegro_mbox to notify
 */
static void allegro_mbox_notify(struct allegro_mbox *mbox)
{
    struct allegro_dev *dev = mbox->dev;
    union mcu_msg_response *msg;
    ssize_t size;
    u32 *tmp;
    int err;

    msg = kmalloc(sizeof(*msg), GFP_KERNEL);
    if (!msg)
        return;

    msg->header.version = dev->fw_info->mailbox_version;

    tmp = kmalloc(mbox->size, GFP_KERNEL);
    if (!tmp)
        goto out;

    size = allegro_mbox_read(mbox, tmp, mbox->size);
    if (size < 0)
        goto out;

    err = allegro_decode_mail(msg, tmp);
    if (err)
        goto out;

    allegro_handle_message(dev, msg);

out:
    kfree(tmp);
    kfree(msg);
}

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

static int allegro_mcu_send_create_channel(struct allegro_dev *dev,
                       struct allegro_channel *channel)
{
    struct mcu_msg_create_channel msg;
    struct allegro_buffer *blob = &channel->config_blob;
    struct create_channel_param param;
    size_t size;

    memset(&param, 0, sizeof(param));
    param.version = dev->fw_info->mailbox_version;
    param.width = channel->width;
    param.height = channel->height;
    param.framerate = 60000;
    param.clk_ratio = 1000;
    param.max_latency = 2;
    param.num_core = 0;
    param.ddr_width = 32;
    param.low_lat = 0;
    param.parallel_wpp = 0;
    param.disable_cache = 0;
    param.frame_buffer_compression = 0;
    param.use_early_callback = 0;
    param.fb_storage_mode = 0;
    param.codec = channel->codec;
    param.max_slices = INT_MAX;
    param.dec_unit = 0; /*< use frame level decode */
    param.buffer_output_mode = 0;

    allegro_alloc_buffer(dev, blob, sizeof(struct create_channel_param));
    size = allegro_encode_config_blob(blob->vaddr, &param);

    memset(&msg, 0, sizeof(msg));

    msg.header.type = MCU_MSG_TYPE_CREATE_CHANNEL;
    msg.header.version = dev->fw_info->mailbox_version;

    msg.user_id = channel->user_id;

    msg.blob = blob->vaddr;
    msg.blob_size = size;
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

    msg.channel_id = channel->mcu_channel_id;

    allegro_mbox_send(dev->mbox_command, &msg);

    return 0;
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

static int allegro_handle_init(struct allegro_dev *dev,
                   struct mcu_msg_init_response *msg)
{
    complete(&dev->init_complete);

    return 0;
}

static int
allegro_handle_create_channel(struct allegro_dev *dev,
                  struct mcu_msg_create_channel_response *msg)
{
    struct allegro_channel *channel;
    int err = 0;
    struct create_channel_param param;

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

    printk("%s,%d: not implemented\n", __func__, __LINE__);

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

static void allegro_handle_message(struct allegro_dev *dev,
                   union mcu_msg_response *msg)
{
    switch (msg->header.type) {
    case MCU_MSG_TYPE_INIT:
        allegro_handle_init(dev, &msg->init);
        break;
    case MCU_MSG_TYPE_CREATE_CHANNEL:
        allegro_handle_create_channel(dev, &msg->create_channel);
        break;
    case MCU_MSG_TYPE_DESTROY_CHANNEL:
        allegro_handle_destroy_channel(dev, &msg->destroy_channel);
        break;
    default:
        v4l2_warn(&dev->v4l2_dev,
              "%s: unknown message %s\n",
              __func__, msg_type_name(msg->header.type));
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
            break;
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
            break;
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

static void allegro_mcu_interrupt(struct allegro_dev *dev)
{
    regmap_write(dev->regmap, AL5_MCU_INTERRUPT, BIT(0));
}

static void allegro_destroy_channel(struct allegro_channel *channel)
{
    struct allegro_dev *dev = channel->dev;
    unsigned long timeout;

    if (channel_exists(channel)) {
        reinit_completion(&channel->completion);
        allegro_mcu_send_destroy_channel(dev, channel);
        timeout = wait_for_completion_timeout(&channel->completion,
                              msecs_to_jiffies(5000));
        if (timeout == 0)
            v4l2_warn(&dev->v4l2_dev,
                  "channel %d: timeout while destroying\n",
                  channel->mcu_channel_id);

        channel->mcu_channel_id = -1;
    }

    // destroy_intermediate_buffers(channel);
    // destroy_reference_buffers(channel);

    // v4l2_ctrl_grab(channel->mpeg_video_h264_profile, false);
    // v4l2_ctrl_grab(channel->mpeg_video_h264_level, false);
    // v4l2_ctrl_grab(channel->mpeg_video_h264_i_frame_qp, false);
    // v4l2_ctrl_grab(channel->mpeg_video_h264_max_qp, false);
    // v4l2_ctrl_grab(channel->mpeg_video_h264_min_qp, false);
    // v4l2_ctrl_grab(channel->mpeg_video_h264_p_frame_qp, false);
    // v4l2_ctrl_grab(channel->mpeg_video_h264_b_frame_qp, false);

    // v4l2_ctrl_grab(channel->mpeg_video_hevc_profile, false);
    // v4l2_ctrl_grab(channel->mpeg_video_hevc_level, false);
    // v4l2_ctrl_grab(channel->mpeg_video_hevc_tier, false);
    // v4l2_ctrl_grab(channel->mpeg_video_hevc_i_frame_qp, false);
    // v4l2_ctrl_grab(channel->mpeg_video_hevc_max_qp, false);
    // v4l2_ctrl_grab(channel->mpeg_video_hevc_min_qp, false);
    // v4l2_ctrl_grab(channel->mpeg_video_hevc_p_frame_qp, false);
    // v4l2_ctrl_grab(channel->mpeg_video_hevc_b_frame_qp, false);

    // v4l2_ctrl_grab(channel->mpeg_video_frame_rc_enable, false);
    // v4l2_ctrl_grab(channel->mpeg_video_bitrate_mode, false);
    // v4l2_ctrl_grab(channel->mpeg_video_bitrate, false);
    // v4l2_ctrl_grab(channel->mpeg_video_bitrate_peak, false);
    // v4l2_ctrl_grab(channel->mpeg_video_cpb_size, false);
    // v4l2_ctrl_grab(channel->mpeg_video_gop_size, false);

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
    unsigned long timeout;

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
         "user %d: creating channel (%4.4s, %dx%d)\n",
         channel->user_id,
         (char *)&channel->codec, channel->width, channel->height);

    // v4l2_ctrl_grab(channel->mpeg_video_h264_profile, true);
    // v4l2_ctrl_grab(channel->mpeg_video_h264_level, true);
    // v4l2_ctrl_grab(channel->mpeg_video_h264_i_frame_qp, true);
    // v4l2_ctrl_grab(channel->mpeg_video_h264_max_qp, true);
    // v4l2_ctrl_grab(channel->mpeg_video_h264_min_qp, true);
    // v4l2_ctrl_grab(channel->mpeg_video_h264_p_frame_qp, true);
    // v4l2_ctrl_grab(channel->mpeg_video_h264_b_frame_qp, true);

    // v4l2_ctrl_grab(channel->mpeg_video_hevc_profile, true);
    // v4l2_ctrl_grab(channel->mpeg_video_hevc_level, true);
    // v4l2_ctrl_grab(channel->mpeg_video_hevc_tier, true);
    // v4l2_ctrl_grab(channel->mpeg_video_hevc_i_frame_qp, true);
    // v4l2_ctrl_grab(channel->mpeg_video_hevc_max_qp, true);
    // v4l2_ctrl_grab(channel->mpeg_video_hevc_min_qp, true);
    // v4l2_ctrl_grab(channel->mpeg_video_hevc_p_frame_qp, true);
    // v4l2_ctrl_grab(channel->mpeg_video_hevc_b_frame_qp, true);

    // v4l2_ctrl_grab(channel->mpeg_video_frame_rc_enable, true);
    // v4l2_ctrl_grab(channel->mpeg_video_bitrate_mode, true);
    // v4l2_ctrl_grab(channel->mpeg_video_bitrate, true);
    // v4l2_ctrl_grab(channel->mpeg_video_bitrate_peak, true);
    // v4l2_ctrl_grab(channel->mpeg_video_cpb_size, true);
    // v4l2_ctrl_grab(channel->mpeg_video_gop_size, true);

    reinit_completion(&channel->completion);
    allegro_mcu_send_create_channel(dev, channel);
    timeout = wait_for_completion_timeout(&channel->completion,
                          msecs_to_jiffies(5000));
    if (timeout == 0)
        channel->error = -ETIMEDOUT;
    if (channel->error)
        goto err;

    v4l2_dbg(1, debug, &dev->v4l2_dev,
         "channel %d: accepting buffers\n",
         channel->mcu_channel_id);

    return 0;

err:
    allegro_destroy_channel(channel);

    return channel->error;
}


static int allegro_open(struct file *file)
{
    struct video_device *vdev = video_devdata(file);
    struct allegro_dev *dev = video_get_drvdata(vdev);

    printk("%s,%d: not implemented\n", __func__, __LINE__);
    return 0;
}

static int allegro_release(struct file *file)
{
    printk("%s,%d: not implemented\n", __func__, __LINE__);

    return 0;
}

static int allegro_querycap(struct file *file, void *fh,
                struct v4l2_capability *cap)
{
    struct video_device *vdev = video_devdata(file);
    struct allegro_dev *dev = video_get_drvdata(vdev);

    strscpy(cap->driver, KBUILD_MODNAME, sizeof(cap->driver));
    strscpy(cap->card, "Allegro DVT Video Decoder", sizeof(cap->card));
    snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
         dev_name(&dev->plat_dev->dev));

    return 0;
}

static int allegro_enum_fmt_vid(struct file *file, void *fh,
                struct v4l2_fmtdesc *f)
{
    switch (f->type) {
    case V4L2_BUF_TYPE_VIDEO_CAPTURE:
        if (f->index >= 1)
            return -EINVAL;
        f->pixelformat = V4L2_PIX_FMT_NV12;
        break;
    case V4L2_BUF_TYPE_VIDEO_OUTPUT:
        if (f->index >= 2)
            return -EINVAL;
        if (f->index == 0)
            f->pixelformat = V4L2_PIX_FMT_H264;
        if (f->index == 1)
            f->pixelformat = V4L2_PIX_FMT_HEVC;
        break;
    default:
        return -EINVAL;
    }
    return 0;
}

static int allegro_g_fmt_vid_cap(struct file *file, void *fh,
                 struct v4l2_format *f)
{
    printk("%s: not implemented\n", __func__);

    return 0;
}

static int allegro_try_fmt_vid_cap(struct file *file, void *fh,
                   struct v4l2_format *f)
{
    printk("%s: not implemented\n", __func__);

    return 0;
}

static int allegro_s_fmt_vid_cap(struct file *file, void *fh,
                 struct v4l2_format *f)
{
    printk("%s: not implemented\n", __func__);

    return 0;
}

static int allegro_g_fmt_vid_out(struct file *file, void *fh,
                 struct v4l2_format *f)
{
    printk("%s: not implemented\n", __func__);

    return 0;
}


static int allegro_try_fmt_vid_out(struct file *file, void *fh,
                   struct v4l2_format *f)
{
    printk("%s: not implemented\n", __func__);

    return 0;
}


static int allegro_s_fmt_vid_out(struct file *file, void *fh,
                 struct v4l2_format *f)
{
    printk("%s: not implemented\n", __func__);

    return 0;
}

static int allegro_decoder_cmd(struct file *file, void *fh,
                   struct v4l2_decoder_cmd *cmd)
{
    int err;

    err = v4l2_m2m_ioctl_try_decoder_cmd(file, fh, cmd);
    if (err)
        return err;

    err = v4l2_m2m_ioctl_decoder_cmd(file, fh, cmd);
    if (err)
        return err;

    printk("%s: not implemented\n", __func__);

    return err;
}

static int allegro_ioctl_streamon(struct file *file, void *priv,
                  enum v4l2_buf_type type)
{
    struct v4l2_fh *fh = file->private_data;
    struct allegro_channel *channel = fh_to_channel(fh);
    int err;

    if (type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
        err = allegro_create_channel(channel);
        if (err)
            return err;
    }

    return v4l2_m2m_streamon(file, fh->m2m_ctx, type);
}

static int allegro_g_parm(struct file *file, void *fh,
              struct v4l2_streamparm *a)
{
    if (a->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
        return -EINVAL;

    printk("%s: not implemented\n", __func__);

    return 0;
}

static int allegro_s_parm(struct file *file, void *fh,
              struct v4l2_streamparm *a)
{
    if (a->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
        return -EINVAL;

    printk("%s: not implemented\n", __func__);

    return 0;
}

static int allegro_subscribe_event(struct v4l2_fh *fh,
                   const struct v4l2_event_subscription *sub)
{
    switch (sub->type) {
    case V4L2_EVENT_EOS:
        return v4l2_event_subscribe(fh, sub, 0, NULL);
    default:
        return v4l2_ctrl_subscribe_event(fh, sub);
    }
}

static const struct v4l2_ioctl_ops allegro_ioctl_ops = {
    .vidioc_querycap = allegro_querycap,
    .vidioc_enum_fmt_vid_cap = allegro_enum_fmt_vid,
    .vidioc_enum_fmt_vid_out = allegro_enum_fmt_vid,
    .vidioc_g_fmt_vid_cap = allegro_g_fmt_vid_cap,
    .vidioc_try_fmt_vid_cap = allegro_try_fmt_vid_cap,
    .vidioc_s_fmt_vid_cap = allegro_s_fmt_vid_cap,
    .vidioc_g_fmt_vid_out = allegro_g_fmt_vid_out,
    .vidioc_try_fmt_vid_out = allegro_try_fmt_vid_out,
    .vidioc_s_fmt_vid_out = allegro_s_fmt_vid_out,

    .vidioc_create_bufs = v4l2_m2m_ioctl_create_bufs,
    .vidioc_reqbufs = v4l2_m2m_ioctl_reqbufs,

    .vidioc_expbuf = v4l2_m2m_ioctl_expbuf,
    .vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
    .vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
    .vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
    .vidioc_prepare_buf = v4l2_m2m_ioctl_prepare_buf,

    .vidioc_streamon = allegro_ioctl_streamon,
    .vidioc_streamoff = v4l2_m2m_ioctl_streamoff,

    .vidioc_try_decoder_cmd = v4l2_m2m_ioctl_try_decoder_cmd,
    .vidioc_decoder_cmd = allegro_decoder_cmd,

    .vidioc_g_parm      = allegro_g_parm,
    .vidioc_s_parm      = allegro_s_parm,

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

    strscpy(video_dev->name, "allegro-decoder", sizeof(video_dev->name));
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

    printk("%s: not implemented\n", __func__);
}

static const struct v4l2_m2m_ops allegro_m2m_ops = {
    .device_run = allegro_device_run,
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
    const char *fw_codec_name = "al5d.fw";
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

    err = allegro_register_device(dev);
    if (err) {
        v4l2_err(&dev->v4l2_dev, "failed to register video device\n");
        goto err_m2m_release;
    }

    v4l2_dbg(1, debug, &dev->v4l2_dev,
         "allegro codec registered as /dev/video%d\n",
         dev->video_dev.num);

    release_firmware(fw_codec);
    release_firmware(fw);

    return;

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
    const char *fw = "al5d_b.fw";

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

static int allegro_probe(struct platform_device *pdev)
{
    struct allegro_dev *dev;
    struct resource *res, *sram_res;
    int ret;
    int irq;
    void __iomem *regs, *sram_regs;

    dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
    if (!dev)
        return -ENOMEM;
    dev->plat_dev = pdev;
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

    sram_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "sram");
    if (!sram_res) {
        dev_err(&pdev->dev,
            "sram resource missing from device tree\n");
        return -EINVAL;
    }
    sram_regs = devm_ioremap(&pdev->dev,
                 sram_res->start,
                 resource_size(sram_res));
    if (!sram_regs) {
        dev_err(&pdev->dev, "failed to map sram\n");
        return -ENOMEM;
    }
    dev->sram = devm_regmap_init_mmio(&pdev->dev, sram_regs,
                      &allegro_sram_config);
    if (IS_ERR(dev->sram)) {
        dev_err(&pdev->dev, "failed to init sram\n");
        return PTR_ERR(dev->sram);
    }

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

    video_unregister_device(&dev->video_dev);
    if (dev->m2m_dev)
        v4l2_m2m_release(dev->m2m_dev);
    allegro_mcu_hw_deinit(dev);
    allegro_free_fw_codec(dev);

    v4l2_device_unregister(&dev->v4l2_dev);

    return 0;
}

static const struct of_device_id allegro_dt_ids[] = {
    { .compatible = "allegro,al5d-1.1" },
    { /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, allegro_dt_ids);

static struct platform_driver allegro_driver = {
    .probe = allegro_probe,
    .remove = allegro_remove,
    .driver = {
        .name = "allegro-dec",
        .of_match_table = of_match_ptr(allegro_dt_ids),
    },
};

module_platform_driver(allegro_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Brainlab,VIED <deji.aribuki.ext@brainlab.com>");
MODULE_DESCRIPTION("Allegro DVT decoder driver");