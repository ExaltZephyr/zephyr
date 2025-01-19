/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/kernel.h>

static const struct device *mspm0_dma = DEVICE_DT_GET(DT_NODELABEL(dma));

void my_callback(const struct device *dev, void *user_data, uint32_t channel, int status)
{
	printf("Finished transfering!\n");
}

int main(void)
{
	printk("Hello World! %s\n", CONFIG_BOARD_TARGET);

	int ret = 0;

	uint8_t src[] = {1, 2, 3, 4, 5, 6};
	uint8_t dst[] = {0, 0, 0, 0, 0, 0};

	struct dma_block_config blk1 = {0};
	struct dma_block_config blk2 = {0};
	struct dma_config cfg1 = {0};
	struct dma_config cfg2 = {0};

	blk2.block_size = 1;
	blk2.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	blk2.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	blk2.source_address = (uint32_t) &src[4];
	blk2.dest_address = (uint32_t) &dst[4];

	blk1.block_size = 1;
	blk1.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	blk1.source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	blk1.source_address = (uint32_t) src;
	blk1.dest_address = (uint32_t) dst;
	blk1.next_block = NULL;

	cfg2.block_count = 1;
	cfg2.source_burst_length = 1;
	cfg2.dest_burst_length = 1;
	cfg2.head_block = &blk2;
	cfg2.source_data_size = 1;
	cfg2.dest_data_size = 1;
	cfg2.dest_chaining_en = 1;
	cfg2.linked_channel = 0;
	cfg2.dma_callback = my_callback;
	cfg2.error_callback_dis = 1;

	cfg1.block_count = 1;
	cfg1.source_burst_length = 1;
	cfg1.dest_burst_length = 1;
	cfg1.head_block = &blk1;
	cfg1.dma_slot = 0;
	cfg1.source_data_size = 1;
	cfg1.dest_data_size = 1;
	cfg1.error_callback_dis = 1;
	// cfg1.dma_callback = my_callback;
	// cfg1.complete_callback_en = 1;

	ret = dma_config(mspm0_dma, 0, &cfg1);
	if(ret != 0) {
		printf("Failed to configure DMA channel 0\n");
		return 0;
	}

	ret = dma_config(mspm0_dma, 1, &cfg2);
	if(ret != 0) {
		printf("Failed to configure DMA channel 1\n");
		return 0;
	}

	dma_start(mspm0_dma, 1);
	dma_start(mspm0_dma, 0);

	dma_stop(mspm0_dma, 0);
	k_sleep(K_MSEC(50));
	// dma_stop(mspm0_dma, 0);
	// k_sleep(K_MSEC(50));
	// dma_stop(mspm0_dma, 0);
	// k_sleep(K_MSEC(50));
	// dma_stop(mspm0_dma, 0);

	k_sleep(K_MSEC(500));

	printk("dst = {%d, %d, %d, %d, %d, %d}\n", dst[0], dst[1], dst[2], dst[3], dst[4], dst[5]);

	return 0;
}
