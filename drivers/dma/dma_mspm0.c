/*
 * Copyright (c) 2024 EXALT Technologies.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>

#include <ti/driverlib/dl_dma.h>

#define DT_DRV_COMPAT ti_mspm0_dma

LOG_MODULE_REGISTER(dma_mspm0, LOG_LEVEL_DBG);

#define DMA_MSPM0_TIMEOUT K_SECONDS(1)

typedef void (*irq_func_t)(void);

struct dma_mspm0_channel {
	uint32_t id;
	struct dma_config *config;
	struct dma_block_config *curr_block;
	int curr_block_rem_bytes;
	int curr_num_blocks;
	int num_transfers;
	struct k_sem sync_sem;
	uint64_t total_bytes_written;
};

struct dma_mspm0_config {
	DMA_Regs *dma;
	irq_func_t irq_func;
};

struct dma_mspm0_data {
	struct dma_mspm0_channel *channels;
	const int num_channels;
	const int num_triggers;
};

static void dma_mspm0_get_int_ch_idx(uint32_t id, uint32_t *ch_id)
{
	switch (id) {
	case 0:
		*ch_id = DL_DMA_INTERRUPT_CHANNEL0;
		break;
	case 1:
		*ch_id = DL_DMA_INTERRUPT_CHANNEL1;
		break;
	case 2:
		*ch_id = DL_DMA_INTERRUPT_CHANNEL2;
		break;
	case 3:
		*ch_id = DL_DMA_INTERRUPT_CHANNEL3;
		break;
	case 4:
		*ch_id = DL_DMA_INTERRUPT_CHANNEL4;
		break;
	case 5:
		*ch_id = DL_DMA_INTERRUPT_CHANNEL5;
		break;
	case 6:
		*ch_id = DL_DMA_INTERRUPT_CHANNEL6;
		break;
	default:
		LOG_ERR("Invalid channel ID %d", id);
	}
}

static int dma_mspm0_get_data_width(uint32_t data_width, DL_DMA_WIDTH *mspm0_data_width)
{
	switch (data_width) {
	case 1:
		*mspm0_data_width = DL_DMA_WIDTH_BYTE;
		break;
	case 2:
		*mspm0_data_width = DL_DMA_WIDTH_HALF_WORD;
		break;
	case 4:
		*mspm0_data_width = DL_DMA_WIDTH_WORD;
		break;
	case 8:
		*mspm0_data_width = DL_DMA_WIDTH_LONG;
		break;
	default:
		LOG_ERR("Unsupported data width");
		return -EINVAL;
	}

	return 0;
}

static int dma_mspm0_get_addr_adj(uint16_t addr_adj, DL_DMA_INCREMENT *mspm0_addr_adj)
{
	switch (addr_adj) {
	case DMA_ADDR_ADJ_INCREMENT:
		*mspm0_addr_adj = DL_DMA_ADDR_INCREMENT;
		break;
	case DMA_ADDR_ADJ_DECREMENT:
		*mspm0_addr_adj = DL_DMA_ADDR_DECREMENT;
		break;
	case DMA_ADDR_ADJ_NO_CHANGE:
		*mspm0_addr_adj = DL_DMA_ADDR_UNCHANGED;
		break;
	default:
		LOG_ERR("Invalid address adjustment value");
		return -EINVAL;
	}

	return 0;
}

static void dma_mspm0_clear_err(const struct device *dev, struct dma_mspm0_channel *channel)
{
	const struct dma_mspm0_config *cfg = dev->config;
	DMA_Regs *dma = cfg->dma;
	uint32_t ch_id = 0;

	dma_mspm0_get_int_ch_idx(channel->id, &ch_id);
	DL_DMA_disableInterrupt(dma, ch_id);

	DL_DMA_disableChannel(dma, channel->id);

	k_sem_give(&channel->sync_sem);
}

static int dma_mspm0_configure_channel_block(const struct device *dev,
					     struct dma_mspm0_channel *channel)
{
	const struct dma_mspm0_config *cfg = dev->config;
	DMA_Regs *dma = cfg->dma;
	int ret = 0;

	DL_DMA_WIDTH src_width = DL_DMA_WIDTH_BYTE;
	DL_DMA_WIDTH dst_width = DL_DMA_WIDTH_BYTE;

	ret = dma_mspm0_get_data_width(channel->config->source_data_size, &src_width);
	if (ret != 0) {
		LOG_ERR("Failed to configure DMA channel due to data width error");
		dma_mspm0_clear_err(dev, channel);
		return ret;
	}

	ret = dma_mspm0_get_data_width(channel->config->dest_data_size, &dst_width);
	if (ret != 0) {
		LOG_ERR("Failed to configure DMA channel due to data width error");
		dma_mspm0_clear_err(dev, channel);
		return ret;
	}

	DL_DMA_setSrcWidth(dma, channel->id, src_width);
	DL_DMA_setDestWidth(dma, channel->id, dst_width);

	DL_DMA_INCREMENT src_inc = DL_DMA_ADDR_UNCHANGED;
	DL_DMA_INCREMENT dst_inc = DL_DMA_ADDR_UNCHANGED;

	ret = dma_mspm0_get_addr_adj(channel->curr_block->source_addr_adj, &src_inc);
	if (ret != 0) {
		LOG_ERR("Failed to configure DMA channel due to address adjustment error");
		dma_mspm0_clear_err(dev, channel);
		return ret;
	}

	ret = dma_mspm0_get_addr_adj(channel->curr_block->dest_addr_adj, &dst_inc);
	if (ret != 0) {
		LOG_ERR("Failed to configure DMA channel due to address adjustment error");
		dma_mspm0_clear_err(dev, channel);
		return ret;
	}

	DL_DMA_setSrcIncrement(dma, channel->id, src_inc);
	DL_DMA_setDestIncrement(dma, channel->id, dst_inc);

	DL_DMA_setSrcAddr(dma, channel->id, channel->curr_block->source_address);
	DL_DMA_setDestAddr(dma, channel->id, channel->curr_block->dest_address);

	/* The remaining bytes of each block should be set everytime a channel is configured with a
	 * new block */
	channel->curr_block_rem_bytes = channel->curr_block->block_size;
	channel->num_transfers =
		channel->config->source_burst_length / channel->config->source_data_size;

	DL_DMA_setTransferSize(dma, channel->id, channel->num_transfers);

	return 0;
}

static int dma_mspm0_verify_channel_id(const struct device *dev, uint32_t id)
{
	struct dma_mspm0_data *data = dev->data;

	if (id > data->num_channels) {
		LOG_ERR("Invalid channel %d used", id);
		return -EINVAL;
	}

	return 0;
}

static int dma_mspm0_verify_channel_config(const struct device *dev, uint32_t id, struct dma_config *config)
{
	struct dma_mspm0_data *data = (struct dma_mspm0_data *)dev->data;
	int ret;

	ret = dma_mspm0_verify_channel_id(dev, id);
	if (ret != 0) {
		LOG_ERR("Failed to configure channel with id %d", id);
		return ret;
	}

	if (config->dma_slot > data->num_triggers) {
		LOG_ERR("Invalid trigger source %d used", config->dma_slot);
		return -EINVAL;
	}

	if (config->channel_priority > 0) {
		LOG_ERR("Channel priority is not supported");
		return -EINVAL;
	}

	if (config->cyclic) {
		LOG_ERR("Cyclic mode is not supported");
		return -EINVAL;
	}

	if (config->source_handshake == 1 || config->dest_handshake == 1) {
		LOG_ERR("Software source/destination handshake is not supported");
		return -EINVAL;
	}

	if (config->source_data_size != config->dest_data_size) {
		LOG_ERR("Source and destination data size are not the same");
		return -EINVAL;
	}

	if (config->source_burst_length != config->dest_burst_length) {
		LOG_ERR("Source and destination burst length are not the same");
		return -EINVAL;
	}

	if (config->source_burst_length % config->source_data_size != 0) {
		LOG_ERR("Source burst length is not a multiple of source data size");
		return -EINVAL;
	}

	if (config->dest_burst_length % config->dest_data_size != 0) {
		LOG_ERR("Destination burst length is not a multiple of destination data size");
		return -EINVAL;
	}

	if (config->source_chaining_en) {
		LOG_ERR("Source chaining is not supported");
		return -EINVAL;
	}

	if (config->dest_chaining_en) {
		ret = dma_mspm0_verify_channel_id(dev, config->linked_channel);
		if (ret != 0) {
			LOG_ERR("Linked channel ID %d is incorrect", config->linked_channel);
			return ret;
		}
	}

	/* It is not possible to determine the channel that caused an error due to hardware
	 * limitation. Thus it is not possible to call the appropriate user callback in this
	 * situation */
	if (config->error_callback_dis == 0) {
		LOG_ERR("User callback on error is not supported");
		return -EINVAL;
	}

	struct dma_block_config *blk = config->head_block;

	/* Verify all blocks */
	for (size_t i = 0; i < config->block_count; i++) {
		if (blk == NULL) {
			LOG_ERR("Number of blocks provided doesn't match the blocks provided");
			return -EINVAL;
		}

		if (blk->block_size % config->source_burst_length != 0) {
			LOG_ERR("Block %d size is not a multiple of burst length", i);
			return -EINVAL;
		}

		if (blk->fifo_mode_control > 1) {
			LOG_ERR("Block %d FIFO mode control is not supported", i);
			return -EINVAL;
		}

		if (blk->source_reload_en || blk->dest_reload_en) {
			LOG_ERR("Block %d source and destination address reload is not supported",
				i);
			return -EINVAL;
		}

		if (blk->dest_scatter_en || blk->source_gather_en) {
			LOG_ERR("Block %d source gather/destination scatter is not supported", i);
			return -EINVAL;
		}

		if(blk->flow_control_mode) {
			LOG_ERR("Block %d flow control is not supported", i);
			return -EINVAL;
		}

		blk = blk->next_block;
	}

	return 0;
}

static int dma_mspm0_configure(const struct device *dev, uint32_t id, struct dma_config *config)
{
	const struct dma_mspm0_config *cfg = dev->config;
	struct dma_mspm0_data *data = (struct dma_mspm0_data *)dev->data;
	DMA_Regs *dma = cfg->dma;
	int ret = 0;

	ret = dma_mspm0_verify_channel_config(dev, id, config);
	if(ret !=0) {
		LOG_ERR("Invalid channel %d config", id);
		return ret;
	}

	/* Check if the channel is busy */
	if (k_sem_take(&data->channels[id].sync_sem, DMA_MSPM0_TIMEOUT) != 0) {
		return -EBUSY;
	}

	/* If chaining is enabled, the trigger source is an internal DMA channel rather than an
	 * external trigger */
	if (config->dest_chaining_en) {
		DL_DMA_setTrigger(dma, id, config->linked_channel, DL_DMA_TRIGGER_TYPE_INTERNAL);
	} else {
		DL_DMA_setTrigger(dma, id, config->dma_slot, DL_DMA_TRIGGER_TYPE_EXTERNAL);
	}

	DL_DMA_configMode(dma, id, DL_DMA_SINGLE_BLOCK_TRANSFER_MODE, DL_DMA_NORMAL_MODE);

	/* Prepare channel data */
	data->channels[id].config = config;
	data->channels[id].curr_block = config->head_block;
	data->channels[id].curr_num_blocks = config->block_count;
	data->channels[id].id = id;

	ret = dma_mspm0_configure_channel_block(dev, &data->channels[id]);
	if (ret != 0) {
		LOG_ERR("Failed to configure DMA channel");
		dma_mspm0_clear_err(dev, &data->channels[id]);
		return ret;
	}

	uint32_t ch_id = 0;

	dma_mspm0_get_int_ch_idx(id, &ch_id);
	DL_DMA_enableInterrupt(dma, ch_id);

	return 0;
}

static int dma_mspm0_reload(const struct device *dev, uint32_t id, uint32_t src, uint32_t dst,
			    size_t size)
{
	const struct dma_mspm0_config *cfg = dev->config;
	const struct dma_mspm0_data *data = dev->data;
	DMA_Regs *dma = cfg->dma;
	int ret;

	ret = dma_mspm0_verify_channel_id(dev, id);
	if (ret != 0) {
		LOG_ERR("Cannot reload channel with id %d", id);
		return ret;
	}

	/* Only enabled and configured channels can be reloaded */
	if (!DL_DMA_isChannelEnabled(dma, id)) {
		LOG_ERR("Channel %d is not configured, cannot be reloaded", id);
		return -EFAULT;
	}

	/* Disable channel temporarily and wait for channel to be disabled */
	DL_DMA_disableChannel(dma, id);
	while (DL_DMA_isChannelEnabled(dma, id))
		;

	DL_DMA_setSrcAddr(dma, id, src);
	DL_DMA_setDestAddr(dma, id, dst);
	DL_DMA_setTransferSize(dma, id, size / data->channels[id].config->source_data_size);

	DL_DMA_enableChannel(dma, id);

	return 0;
}

static int dma_mspm0_start(const struct device *dev, uint32_t id)
{
	const struct dma_mspm0_config *cfg = dev->config;
	DMA_Regs *dma = cfg->dma;
	int ret;

	ret = dma_mspm0_verify_channel_id(dev, id);
	if (ret != 0) {
		LOG_ERR("Cannot start channel with id %d", id);
		return ret;
	}

	DL_DMA_enableChannel(dma, id);

	return 0;
}

static int dma_mspm0_stop(const struct device *dev, uint32_t id)
{
	const struct dma_mspm0_config *cfg = dev->config;
	DMA_Regs *dma = cfg->dma;
	int ret;

	ret = dma_mspm0_verify_channel_id(dev, id);
	if (ret != 0) {
		LOG_ERR("Cannot stop channel with id %d", id);
		return ret;
	}

	// TODO remove the software trigger statement
	// DL_DMA_disableChannel(dma, id);
	DL_DMA_startTransfer(dma, id);

	return 0;
}

static int dma_mspm0_init(const struct device *dev)
{
	const struct dma_mspm0_config *cfg = dev->config;
	struct dma_mspm0_data *data = dev->data;
	cfg->irq_func();

	for (size_t i = 0; i < data->num_channels; i++) {
		k_sem_init(&data->channels[i].sync_sem, 1, 1);
	}

	return 0;
}

static int dma_mspm0_get_status(const struct device *dev, uint32_t id, struct dma_status *stat)
{
	const struct dma_mspm0_config *cfg = dev->config;
	const struct dma_mspm0_data *data = dev->data;
	DMA_Regs *dma = cfg->dma;
	int ret;

	ret = dma_mspm0_verify_channel_id(dev, id);
	if (ret != 0) {
		LOG_ERR("Cannot stop channel with id %d", id);
		return ret;
	}

	stat->busy = DL_DMA_isChannelEnabled(dma, id);
	stat->pending_length =
		DL_DMA_getTransferSize(dma, id) * data->channels[id].config->source_data_size;
	stat->total_copied = data->channels[id].total_bytes_written;

	return 0;
}

static void dma_mspm0_isr(const struct device *dev)
{
	const struct dma_mspm0_config *cfg = dev->config;
	const struct dma_mspm0_data *data = dev->data;
	DMA_Regs *dma = cfg->dma;
	struct dma_mspm0_channel *channel = NULL;
	int ret;

	switch (DL_DMA_getPendingInterrupt(dma)) {
	case DL_DMA_EVENT_IIDX_DMACH0:
		channel = &data->channels[0];
		break;
	case DL_DMA_EVENT_IIDX_DMACH1:
		channel = &data->channels[1];
		break;
	case DL_DMA_EVENT_IIDX_DMACH2:
		channel = &data->channels[2];
		break;
	case DL_DMA_EVENT_IIDX_DMACH3:
		channel = &data->channels[3];
		break;
	case DL_DMA_EVENT_IIDX_DMACH4:
		channel = &data->channels[4];
		break;
	case DL_DMA_EVENT_IIDX_DMACH5:
		channel = &data->channels[5];
		break;
	case DL_DMA_EVENT_IIDX_DMACH6:
		channel = &data->channels[6];
		break;
	case DL_DMA_EVENT_IIDX_ADDR_ERROR:
		LOG_ERR("DMA address error detected");
		return;
	case DL_DMA_EVENT_IIDX_DATA_ERROR:
		LOG_ERR("DMA data error detected");
		return;
	default:
		LOG_ERR("Unexpected interrupt received");
		return;
	}

	channel->curr_block_rem_bytes -= channel->config->source_burst_length;
	channel->total_bytes_written += channel->config->source_burst_length;

	/* If the current block still has remaining bytes to send, it is waiting for a trigger, the
	 * channel should be enabled again with the new number of bytes to transfer */
	if (channel->curr_block_rem_bytes > 0) {
		DL_DMA_setTransferSize(dma, channel->id, channel->num_transfers);
		DL_DMA_enableChannel(dma, channel->id);

		return;
	}

	channel->curr_num_blocks--;

	if (channel->curr_num_blocks > 0) {
		channel->curr_block = channel->curr_block->next_block;

		ret = dma_mspm0_configure_channel_block(dev, channel);
		if (ret != 0) {
			LOG_ERR("Failed to configure block %d",
				channel->config->block_count - channel->curr_num_blocks + 1);
			dma_mspm0_clear_err(dev, channel);
			return;
		}

		DL_DMA_enableChannel(dma, channel->id);
	}

	/* Call the user callback when end of transmission detected */
	if (channel->config->dma_callback != NULL) {
		if (channel->curr_num_blocks == 0) {
			channel->config->dma_callback(dev, channel->config->user_data, channel->id,
						      DMA_STATUS_COMPLETE);

		} else if (channel->config->complete_callback_en) {
			channel->config->dma_callback(dev, channel->config->user_data, channel->id,
						      DMA_STATUS_BLOCK);
		}
	}

	/* Disable the interrupt flag if all blocks were sent and release channel */
	if (channel->curr_num_blocks == 0) {
		uint32_t ch_id = 0;
		dma_mspm0_get_int_ch_idx(channel->id, &ch_id);

		DL_DMA_disableInterrupt(dma, ch_id);

		k_sem_give(&channel->sync_sem);
	}
}

static DEVICE_API(dma, dma_mspm0_api) = {
	.reload = dma_mspm0_reload,
	.config = dma_mspm0_configure,
	.start = dma_mspm0_start,
	.stop = dma_mspm0_stop,
	.get_status = dma_mspm0_get_status,
};

#define DMA_MSMP0_INIT(index)                                                                      \
	static struct dma_mspm0_channel channels_##index[DT_INST_PROP(index, dma_channels)] = {0}; \
	static void dma_mspm0_dma##index##_isr(void)                                               \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(index), DT_INST_IRQ(index, priority), dma_mspm0_isr,      \
			    DEVICE_DT_INST_GET(index), 0);                                         \
		irq_enable(DT_INST_IRQN(index));                                                   \
	}                                                                                          \
                                                                                                   \
	static const struct dma_mspm0_config dma_mspm0_config_##index = {                          \
		.dma = (DMA_Regs *)DT_INST_REG_ADDR(index),                                        \
		.irq_func = dma_mspm0_dma##index##_isr,                                            \
	};                                                                                         \
                                                                                                   \
	static struct dma_mspm0_data dma_mspm0_data_##index = {                                    \
		.channels = channels_##index,                                                      \
		.num_channels = DT_INST_PROP(index, dma_channels),                                 \
		.num_triggers = DT_INST_PROP(index, dma_requests),                                 \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(index, &dma_mspm0_init, PM_DEVICE_DT_INST_GET(index),                \
			      &dma_mspm0_data_##index, &dma_mspm0_config_##index, PRE_KERNEL_1,    \
			      CONFIG_DMA_INIT_PRIORITY, &dma_mspm0_api);

DT_INST_FOREACH_STATUS_OKAY(DMA_MSMP0_INIT)
