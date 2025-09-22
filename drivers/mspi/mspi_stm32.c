/*
 * Copyright (c) 2025 EXALT Technologies.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * **************************************************************************
 * MSPI flash controller driver for stm32 serie with multi-SPI periherals
 * This driver is based on the stm32Cube HAL XSPI driver
 * **************************************************************************
 */
#define DT_DRV_COMPAT st_stm32_mspi_controller

#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/mspi.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include "mspi_stm32.h"

LOG_MODULE_REGISTER(mspi_stm32, CONFIG_MSPI_LOG_LEVEL);

static uint32_t mspi_stm32_hal_address_size(uint8_t address_length)
{
	if (address_length == 4U) {
		return HAL_XSPI_ADDRESS_32_BITS;
	}

	return HAL_XSPI_ADDRESS_24_BITS;
}

/**
 * @brief Gives a XSPI_RegularCmdTypeDef with all parameters set except Instruction, Address, NbData
 */
static XSPI_RegularCmdTypeDef mspi_stm32_prepare_cmd(uint8_t cfg_mode, uint8_t cfg_rate)
{
	/* Command empty structure */
	XSPI_RegularCmdTypeDef cmd_tmp = {0};

	cmd_tmp.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG;
	cmd_tmp.InstructionWidth = ((cfg_mode == MSPI_IO_MODE_OCTAL) ? HAL_XSPI_INSTRUCTION_16_BITS
								     : HAL_XSPI_INSTRUCTION_8_BITS);
	cmd_tmp.InstructionDTRMode =
		((cfg_rate == MSPI_DATA_RATE_DUAL) ? HAL_XSPI_INSTRUCTION_DTR_ENABLE
						   : HAL_XSPI_INSTRUCTION_DTR_DISABLE);
	cmd_tmp.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
	cmd_tmp.AddressDTRMode = ((cfg_rate == MSPI_DATA_RATE_DUAL) ? HAL_XSPI_ADDRESS_DTR_ENABLE
								    : HAL_XSPI_ADDRESS_DTR_DISABLE);
	cmd_tmp.DataDTRMode = ((cfg_rate == MSPI_DATA_RATE_DUAL) ? HAL_XSPI_DATA_DTR_ENABLE
								 : HAL_XSPI_DATA_DTR_DISABLE);
	/* AddressWidth must be set to 32bits for init and mem config phase */
	cmd_tmp.AddressWidth = HAL_XSPI_ADDRESS_32_BITS;
	cmd_tmp.DataDTRMode = ((cfg_rate == MSPI_DATA_RATE_DUAL) ? HAL_XSPI_DATA_DTR_ENABLE
								 : HAL_XSPI_DATA_DTR_DISABLE);
	cmd_tmp.DQSMode =
		((cfg_rate == MSPI_DATA_RATE_DUAL) ? HAL_XSPI_DQS_ENABLE : HAL_XSPI_DQS_DISABLE);
	cmd_tmp.SIOOMode = HAL_XSPI_SIOO_INST_EVERY_CMD;

	switch (cfg_mode) {
	case MSPI_IO_MODE_OCTAL: {
		cmd_tmp.InstructionMode = HAL_XSPI_INSTRUCTION_8_LINES;
		cmd_tmp.AddressMode = HAL_XSPI_ADDRESS_8_LINES;
		cmd_tmp.DataMode = HAL_XSPI_DATA_8_LINES;
		break;
	}
	case MSPI_IO_MODE_QUAD: {
		cmd_tmp.InstructionMode = HAL_XSPI_INSTRUCTION_4_LINES;
		cmd_tmp.AddressMode = HAL_XSPI_ADDRESS_4_LINES;
		cmd_tmp.DataMode = HAL_XSPI_DATA_4_LINES;
		break;
	}
	case MSPI_IO_MODE_DUAL: {
		cmd_tmp.InstructionMode = HAL_XSPI_INSTRUCTION_2_LINES;
		cmd_tmp.AddressMode = HAL_XSPI_ADDRESS_2_LINES;
		cmd_tmp.DataMode = HAL_XSPI_DATA_2_LINES;
		break;
	}
	default: {
		cmd_tmp.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
		cmd_tmp.AddressMode = HAL_XSPI_ADDRESS_1_LINE;
		cmd_tmp.DataMode = HAL_XSPI_DATA_1_LINE;
		break;
	}
	}

	return cmd_tmp;
}

/**
 * @brief Check if the flash is currently operating in memory-mapped mode.
 */
static bool stm32_xspi_is_memorymap(const struct device *dev)
{
	struct mspi_stm32_data *dev_data = dev->data;

	return READ_BIT(dev_data->hmspi.Instance->CR, XSPI_CR_FMODE) == XSPI_CR_FMODE;
}

/**
 * @brief Sets the device back in indirect mode.
 */
static int mspi_stm32_memmap_off(const struct device *controller)
{
	struct mspi_stm32_data *dev_data = controller->data;

	if (HAL_XSPI_Abort(&dev_data->hmspi) != HAL_OK) {
		LOG_ERR("MemMapped abort failed: %x\n",dev_data->hmspi.ErrorCode);
		return -EIO;
	}
	return 0;
}

/**
 * @brief Sets the device in Memory-Mapped mode.
 */
static int mspi_stm32_memmap_on(const struct device *controller)
{
	int ret;
	struct mspi_stm32_data *dev_data = controller->data;
	XSPI_MemoryMappedTypeDef s_MemMappedCfg;

	if(stm32_xspi_is_memorymap(controller)) {
		return 0;
	}

	/* Configure in MemoryMapped mode */
	if ((dev_data->dev_cfg.io_mode == MSPI_IO_MODE_SINGLE) &&

	    (mspi_stm32_hal_address_size(dev_data->dev_cfg.addr_length) ==
	     HAL_XSPI_ADDRESS_24_BITS)) {
		/* OPI mode and 3-bytes address size not supported by memory */
		LOG_ERR("MSPI_IO_MODE_SINGLE in 3Bytes addressing is not supported");
		return -EIO;
	}

	XSPI_RegularCmdTypeDef s_command =mspi_stm32_prepare_cmd(dev_data->dev_cfg.io_mode, dev_data->dev_cfg.data_rate);

	/* Initialize the read command */
	s_command.OperationType = HAL_XSPI_OPTYPE_READ_CFG;
	s_command.InstructionMode = (dev_data->dev_cfg.data_rate == MSPI_DATA_RATE_SINGLE)
					    ? ((dev_data->dev_cfg.io_mode == MSPI_IO_MODE_SINGLE)
						       ? HAL_XSPI_INSTRUCTION_1_LINE
						       : HAL_XSPI_INSTRUCTION_8_LINES)
					    : HAL_XSPI_INSTRUCTION_8_LINES;
	s_command.InstructionDTRMode = (dev_data->dev_cfg.data_rate == MSPI_DATA_RATE_SINGLE)
					       ? HAL_XSPI_INSTRUCTION_DTR_DISABLE
					       : HAL_XSPI_INSTRUCTION_DTR_ENABLE;
	s_command.InstructionWidth = (dev_data->dev_cfg.data_rate == MSPI_DATA_RATE_SINGLE)
					     ? ((dev_data->dev_cfg.io_mode == MSPI_IO_MODE_SINGLE)
							? HAL_XSPI_INSTRUCTION_8_BITS
							: HAL_XSPI_INSTRUCTION_16_BITS)
					     : HAL_XSPI_INSTRUCTION_16_BITS;
	s_command.Instruction = (dev_data->dev_cfg.data_rate == MSPI_DATA_RATE_SINGLE)
					? ((dev_data->dev_cfg.io_mode == MSPI_IO_MODE_SINGLE)
						   ? ((mspi_stm32_hal_address_size(
							       dev_data->ctx.xfer.addr_length) ==
						       HAL_XSPI_ADDRESS_24_BITS)
							      ? MSPI_STM32_CMD_READ_FAST
							      : MSPI_STM32_CMD_READ_FAST_4B)
						   : dev_data->dev_cfg.read_cmd)
					: MSPI_STM32_OCMD_DTR_RD;
	s_command.AddressMode = (dev_data->dev_cfg.data_rate == MSPI_DATA_RATE_SINGLE)
					? ((dev_data->dev_cfg.io_mode == MSPI_IO_MODE_SINGLE)
						   ? HAL_XSPI_ADDRESS_1_LINE
						   : HAL_XSPI_ADDRESS_8_LINES)
					: HAL_XSPI_ADDRESS_8_LINES;
	s_command.AddressDTRMode = (dev_data->dev_cfg.data_rate == MSPI_DATA_RATE_SINGLE)
					   ? HAL_XSPI_ADDRESS_DTR_DISABLE
					   : HAL_XSPI_ADDRESS_DTR_ENABLE;
	s_command.AddressWidth =
		(dev_data->dev_cfg.data_rate == MSPI_DATA_RATE_SINGLE)
			? mspi_stm32_hal_address_size(dev_data->ctx.xfer.addr_length)
			: HAL_XSPI_ADDRESS_32_BITS;
	s_command.DataMode = (dev_data->dev_cfg.data_rate == MSPI_DATA_RATE_SINGLE)
				     ? ((dev_data->dev_cfg.io_mode == MSPI_IO_MODE_SINGLE)
						? HAL_XSPI_DATA_1_LINE
						: HAL_XSPI_DATA_8_LINES)
				     : HAL_XSPI_DATA_8_LINES;
	s_command.DataDTRMode = (dev_data->dev_cfg.data_rate == MSPI_DATA_RATE_SINGLE)
					? HAL_XSPI_DATA_DTR_DISABLE
					: HAL_XSPI_DATA_DTR_ENABLE;
	s_command.DummyCycles = dev_data->ctx.xfer.rx_dummy;
	s_command.DQSMode = (dev_data->dev_cfg.data_rate == MSPI_DATA_RATE_SINGLE)
				    ? HAL_XSPI_DQS_DISABLE
				    : HAL_XSPI_DQS_ENABLE;

	#ifdef XSPI_CCR_SIOO
		s_command.SIOOMode = HAL_XSPI_SIOO_INST_EVERY_CMD;
	#endif /* XSPI_CCR_SIOO */

	ret = HAL_XSPI_Command(&dev_data->hmspi, &s_command, HAL_XSPI_TIMEOUT_DEFAULT_VALUE);
	if (ret != HAL_OK) {
		LOG_ERR("Failed to set memory mapped mode");
		return -EIO;
	}

	/* Initialize the program command */
	s_command.OperationType = HAL_XSPI_OPTYPE_WRITE_CFG;
	if (dev_data->dev_cfg.data_rate == MSPI_DATA_RATE_SINGLE) {
		s_command.Instruction =
			(dev_data->dev_cfg.io_mode == MSPI_IO_MODE_SINGLE)
				? ((mspi_stm32_hal_address_size(dev_data->ctx.xfer.addr_length) ==
				    HAL_XSPI_ADDRESS_24_BITS)
					   ? MSPI_STM32_CMD_PP
					   : MSPI_STM32_CMD_PP_4B)
				: MSPI_STM32_OCMD_PAGE_PRG;
	} else {
		s_command.Instruction = MSPI_STM32_OCMD_PAGE_PRG;
	}

	s_command.DQSMode = HAL_XSPI_DQS_DISABLE;
	ret = HAL_XSPI_Command(&dev_data->hmspi, &s_command, HAL_XSPI_TIMEOUT_DEFAULT_VALUE);
	if (ret != HAL_OK) {
		LOG_ERR("Failed to set memory mapped mode");
		return -EIO;
	}

	#ifdef XSPI_CR_NOPREF
		s_MemMappedCfg.NoPrefetchData = HAL_XSPI_AUTOMATIC_PREFETCH_ENABLE;
	#ifdef XSPI_CR_NOPREF_AXI
		s_MemMappedCfg.NoPrefetchAXI = HAL_XSPI_AXI_PREFETCH_DISABLE;
	#endif /* XSPI_CR_NOPREF_AXI */
	#endif /* XSPI_CR_NOPREF */

	/* Enable the memory-mapping */
	s_MemMappedCfg.TimeOutActivation = HAL_XSPI_TIMEOUT_COUNTER_DISABLE;
	ret = HAL_XSPI_MemoryMapped(&dev_data->hmspi, &s_MemMappedCfg);
	if (ret != HAL_OK) {
		LOG_ERR("Failed to enable memory mapped mode");
		return -EIO;
	}

	return 0;
}

static inline int mspi_context_lock(struct mspi_context *ctx, const struct mspi_dev_id *req,
				    const struct mspi_xfer *xfer, bool lockon)
{
	int ret = 0;
	if (k_sem_take(&ctx->lock, K_MSEC(xfer->timeout))) {
		return -EBUSY;
	}

	ctx->xfer = *xfer;
	ctx->packets_done = 0;
	ctx->packets_left = ctx->xfer.num_packet;
	return ret;
}

/**
 * Check if the MSPI bus is busy.
 *
 * @param controller MSPI emulation controller device.
 * @return true The MSPI bus is busy.
 * @return false The MSPI bus is idle.
 */
static inline bool mspi_is_inp(const struct device *controller)
{
	struct mspi_stm32_data *dev_data = controller->data;

	return (k_sem_count_get(&dev_data->ctx.lock) == 0);
}

/**
 * @brief Send a Command to the NOR and Receive/Transceive data if relevant in IT or DMA mode.
 *
 */
static int mspi_stm32_access(const struct device *dev, const struct mspi_xfer_packet *packet,
			     uint8_t access_mode)
{
	struct mspi_stm32_data *dev_data = dev->data;
	HAL_StatusTypeDef hal_ret;

	if(dev_data->xip_cfg.enable) {
		ARG_UNUSED(access_mode);

		if((packet->cmd == MSPI_STM32_CMD_WREN) || (packet->cmd == MSPI_STM32_OCMD_WREN) ||
		   (packet->cmd == MSPI_STM32_CMD_SE_4B) ||  (packet->cmd == MSPI_STM32_CMD_SE) ||
		   (packet->cmd == MSPI_STM32_OCMD_SE) ||
		   ((mspi_stm32_hal_address_size(dev_data->dev_cfg.addr_length) == HAL_XSPI_ADDRESS_24_BITS) && (dev_data->dev_cfg.io_mode == MSPI_IO_MODE_SINGLE))){
			LOG_DBG(" MSPI_IO_MODE_SINGLE in 3Bytes addressing is not supported in memory map mode, switching to indirect mode");
			if (stm32_xspi_is_memorymap(dev)) {
				hal_ret = mspi_stm32_memmap_off(dev);
				if (hal_ret != 0) {
					LOG_ERR("Failed to abort memory-mapped access");
					goto e_access;
				}
			}
			goto indirect;
		}

		if(packet->data_buf == NULL) {
			LOG_ERR("data buf is null :%x\n", packet->cmd);
			return -EIO;
		}

		if (!stm32_xspi_is_memorymap(dev)) {
				hal_ret = mspi_stm32_memmap_on(dev);
				if (hal_ret != 0) {
					LOG_ERR("Failed to set memory mapped");
					goto e_access;
				}
		}

		__ASSERT_NO_MSG(stm32_xspi_is_memorymap(dev));
			uintptr_t mmap_addr = dev_data->memmap_base_addr + packet->address;

		if(packet->dir == MSPI_RX) {
			LOG_INF("Memory-mapped read from 0x%08lx, len %zu and cmd:%x\n", mmap_addr, packet->num_bytes, packet->cmd);
			memcpy(packet->data_buf, (void *)mmap_addr, packet->num_bytes);
			HAL_Delay(1);
			printf("/////data after:\n");
			for (size_t i = 0; i < packet->num_bytes; i++) {
				printk("%02X ", packet->data_buf[i]);
			}
			printk("\n");
			goto e_access;
		} else {
			LOG_INF("Memory-mapped write from 0x%08lx, len %zu\n", mmap_addr, packet->num_bytes);
			memcpy((void *)mmap_addr,packet->data_buf, packet->num_bytes);
			HAL_Delay(1);
			goto e_access;
		}
	}

indirect:
	XSPI_RegularCmdTypeDef cmd =
		mspi_stm32_prepare_cmd(dev_data->dev_cfg.io_mode, dev_data->dev_cfg.data_rate);

	cmd.DataLength = packet->num_bytes;
	cmd.Instruction = packet->cmd;
	if (packet->dir == MSPI_TX) {
		cmd.DummyCycles = dev_data->ctx.xfer.tx_dummy;
	} else {
		cmd.DummyCycles = dev_data->ctx.xfer.rx_dummy;
	}
	cmd.Address = packet->address; /* AddressSize is 32bits in OPSI mode */
	cmd.AddressWidth = mspi_stm32_hal_address_size(dev_data->ctx.xfer.addr_length);
	if (cmd.DataLength == 0) {
		cmd.DataMode = HAL_XSPI_DATA_NONE;
	}

	if ((cmd.Instruction == MSPI_STM32_CMD_WREN) || (cmd.Instruction == MSPI_STM32_OCMD_WREN)) {
		/* Write Enable only accepts HAL_XSPI_ADDRESS_NONE */
		cmd.AddressMode = HAL_XSPI_ADDRESS_NONE;
	}

	LOG_DBG("MSPI access Instruction 0x%x", cmd.Instruction);

	hal_ret = HAL_XSPI_Command(&dev_data->hmspi, &cmd, HAL_XSPI_TIMEOUT_DEFAULT_VALUE);
	if (hal_ret != HAL_OK) {
		LOG_ERR("%d: Failed to send XSPI instruction", hal_ret);
		return -EIO;
	}

	if (packet->num_bytes == 0) {
		return 0;
	}

	if (packet->dir == MSPI_RX) {
		/* Receive the data */
		switch (access_mode) {
		case MSPI_ACCESS_SYNC:
			hal_ret = HAL_XSPI_Receive(&dev_data->hmspi, packet->data_buf,
						   HAL_XSPI_TIMEOUT_DEFAULT_VALUE);
			goto e_access;
		case MSPI_ACCESS_ASYNC:
			hal_ret = HAL_XSPI_Receive_IT(&dev_data->hmspi, packet->data_buf);
			break;
		default:
			/* Not correct */
			hal_ret = HAL_BUSY;
			break;
		}
	} else {
		/* Transmit the data */
		switch (access_mode) {
		case MSPI_ACCESS_SYNC:
			hal_ret = HAL_XSPI_Transmit(&dev_data->hmspi, packet->data_buf,
						    HAL_XSPI_TIMEOUT_DEFAULT_VALUE);
			goto e_access;
		case MSPI_ACCESS_ASYNC:
			hal_ret = HAL_XSPI_Transmit_IT(&dev_data->hmspi, packet->data_buf);
			break;
		default:
			/* Not correct */
			hal_ret = HAL_BUSY;
			break;
		}
	}

	if (hal_ret != HAL_OK) {
		LOG_ERR("%d: Failed to access data", hal_ret);
		return -EIO;
	}

	/* Lock again expecting the IRQ for end of Tx or Rx */
	if (k_sem_take(&dev_data->sync, K_FOREVER)) {
		LOG_ERR("%d: Failed to access data", hal_ret);
		return -EIO;
	}

e_access:
	LOG_DBG("Access %zu data at 0x%lx", packet->num_bytes, (long)(packet->address));

	return 0;
}

/* Start Automatic-Polling mode to wait until the memory is setting mask/value bit */
static int mspi_stm32_wait_auto_polling(const struct device *dev, uint8_t match_value,
					uint8_t match_mask, uint32_t timeout_ms)
{
	struct mspi_stm32_data *dev_data = dev->data;
	XSPI_AutoPollingTypeDef s_config;

	/* Set the match to check if the bit is Reset */
	s_config.MatchValue = match_value;
	s_config.MatchMask = match_mask;

	s_config.MatchMode = HAL_XSPI_MATCH_MODE_AND;
	s_config.IntervalTime = MSPI_STM32_AUTO_POLLING_INTERVAL;
	s_config.AutomaticStop = HAL_XSPI_AUTOMATIC_STOP_ENABLE;

	if (HAL_XSPI_AutoPolling_IT(&dev_data->hmspi, &s_config) != HAL_OK) {
		LOG_ERR("XSPI AutoPoll failed");
		return -EIO;
	}

	if (k_sem_take(&dev_data->sync, K_MSEC(timeout_ms)) != 0) {
		LOG_ERR("XSPI AutoPoll wait failed");
		HAL_XSPI_Abort(&dev_data->hmspi);
		k_sem_reset(&dev_data->sync);
		return -EIO;
	}

	return 0;
}

/*
 * Function to Read the status reg of the device
 * Send the RDSR command (according to io_mode/data_rate
 * Then set the Autopolling mode with match mask/value bit
 */
static int mspi_stm32_status_reg(const struct device *controller, const struct mspi_xfer *xfer)
{
	int ret = 0;
	struct mspi_stm32_data *dev_data = controller->data;
	struct mspi_context *ctx = &dev_data->ctx;

	if (xfer->num_packet == 0 || !xfer->packets) {
		LOG_ERR("Status Reg.: wrong parameters");
		return -EFAULT;
	}

	/* Lock with the expected timeout value = ctx->xfer.timeout */
	ret = mspi_context_lock(ctx, dev_data->dev_id, xfer, true);
	/** For async, user must make sure when cfg_flag = 0 the dummy and instr addr length
	 * in mspi_xfer of the two calls are the same if the first one has not finished yet.
	 */
	if (ret) {
		goto status_err;
	}

	XSPI_RegularCmdTypeDef cmd =
		mspi_stm32_prepare_cmd(dev_data->dev_cfg.io_mode, dev_data->dev_cfg.data_rate);
	if(dev_data->dev_cfg.io_mode == MSPI_IO_MODE_OCTAL){
		cmd.Instruction = MSPI_STM32_OCMD_RDSR ;
		cmd.DummyCycles = (dev_data->dev_cfg.data_rate == MSPI_DATA_RATE_DUAL)?
		MSPI_STM32_DUMMY_REG_OCTAL_DTR :MSPI_STM32_DUMMY_REG_OCTAL;
	}else{
		cmd.Instruction = MSPI_STM32_CMD_RDSR;
		cmd.AddressMode = HAL_XSPI_ADDRESS_NONE;
		cmd.DataMode = HAL_XSPI_DATA_1_LINE; /* 1-line DataMode for any non-OSPI transfer */
		/* DummyCycle to give to the mspi_stm32_read_access/mspi_stm32_write_access */
		cmd.DummyCycles = 0;
		cmd.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
	}
	cmd.Address = 0U;
	LOG_DBG("MSPI poll status reg.");

	XSPI_AutoPollingTypeDef s_config;

	/* Set the match to check if the bit is Reset */
	s_config.MatchValue = MSPI_STM32_WEL_MATCH;
	s_config.MatchMask = MSPI_STM32_WEL_MASK;

	s_config.MatchMode = HAL_XSPI_MATCH_MODE_AND;
	s_config.IntervalTime = MSPI_STM32_AUTO_POLLING_INTERVAL;
	s_config.AutomaticStop = HAL_XSPI_AUTOMATIC_STOP_ENABLE;

	if (stm32_xspi_is_memorymap(controller)) {
		/* Abort ongoing transfer to force CS high/BUSY deasserted */
		int hal_ret = mspi_stm32_memmap_off(controller);
		if (hal_ret != 0) {
			LOG_ERR("Failed to abort memory-mapped access before write");
		return  -1;
		}
	}

	if (HAL_XSPI_Command(&dev_data->hmspi, &cmd, HAL_XSPI_TIMEOUT_DEFAULT_VALUE)) {
		LOG_ERR("%d: Failed to send XSPI instruction", ret);
		ret = -EIO;
		goto status_err;
	}

	ret = mspi_stm32_wait_auto_polling(controller,
		MSPI_STM32_MEM_RDY_MATCH,
		MSPI_STM32_MEM_RDY_MASK,
		HAL_XSPI_TIMEOUT_DEFAULT_VALUE);

status_err:
	k_sem_give(&ctx->lock);
	return ret;
}

/*
 * This function Polls the WIP(Write In Progress) bit to become to 0
 * in cfg_mode SPI/OPI MSPI_IO_MODE_SINGLE or MSPI_IO_MODE_OCTAL
 * and cfg_rate transfer STR/DTR MSPI_DATA_RATE_SINGLE or MSPI_DATA_RATE_DUAL
 */
static int mspi_stm32_mem_ready(const struct device *dev, uint8_t cfg_mode, uint8_t cfg_rate)
{
	struct mspi_stm32_data *dev_data = dev->data;

	XSPI_RegularCmdTypeDef s_command = mspi_stm32_prepare_cmd(cfg_mode, cfg_rate);

	/* Configure automatic polling mode command to wait for memory ready */
	if (cfg_mode == MSPI_IO_MODE_OCTAL) {
		s_command.Instruction = MSPI_STM32_OCMD_RDSR;
		s_command.DummyCycles = (cfg_rate == MSPI_DATA_RATE_DUAL)
						? MSPI_STM32_DUMMY_REG_OCTAL_DTR
						: MSPI_STM32_DUMMY_REG_OCTAL;
	} else {
		s_command.Instruction = MSPI_STM32_CMD_RDSR;
		/* force 1-line InstructionMode for any non-OSPI transfer */
		s_command.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
		s_command.AddressMode = HAL_XSPI_ADDRESS_NONE;
		/* force 1-line DataMode for any non-OSPI transfer */
		s_command.DataMode = HAL_XSPI_DATA_1_LINE;
		s_command.DummyCycles = 0;
	}
	s_command.DataLength = ((cfg_rate == MSPI_DATA_RATE_DUAL) ? 2U : 1U);
	s_command.Address = 0U;

	if (HAL_XSPI_Command(&dev_data->hmspi, &s_command, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) !=
	    HAL_OK) {
		LOG_ERR("MSPI AutoPoll command failed");
		return -EIO;
	}
	/* Set the match to 0x00 to check if the WIP bit is Reset */
	LOG_DBG("MSPI read status reg MemRdy");
	return mspi_stm32_wait_auto_polling(dev,
		MSPI_STM32_MEM_RDY_MATCH,
		MSPI_STM32_MEM_RDY_MASK,
		HAL_XSPI_TIMEOUT_DEFAULT_VALUE);
}

/* Enables writing to the memory sending a Write Enable and wait it is effective */
static int mspi_stm32_write_enable(const struct device *dev, uint8_t cfg_mode, uint8_t cfg_rate)
{
	struct mspi_stm32_data *dev_data = dev->data;
	XSPI_RegularCmdTypeDef s_command = mspi_stm32_prepare_cmd(cfg_mode, cfg_rate);

	/* Initialize the write enable command */
	if (cfg_mode == MSPI_IO_MODE_OCTAL) {
		s_command.Instruction = MSPI_STM32_OCMD_WREN;
	} else {
		s_command.Instruction = MSPI_STM32_CMD_WREN;
		/* force 1-line InstructionMode for any non-OSPI transfer */
		s_command.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
	}
	s_command.AddressMode = HAL_XSPI_ADDRESS_NONE;
	s_command.DataMode = HAL_XSPI_DATA_NONE;
	s_command.DummyCycles = 0U;

	if (HAL_XSPI_Command(&dev_data->hmspi, &s_command, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) !=
	    HAL_OK) {
		LOG_ERR("MSPI flash write enable cmd failed");
		return -EIO;
	}
	LOG_DBG("MSPI write enable");

	/* New command to Configure automatic polling mode to wait for write enabling */
	if (cfg_mode == MSPI_IO_MODE_OCTAL) {
		s_command.Instruction = MSPI_STM32_OCMD_RDSR;
		s_command.AddressMode = HAL_XSPI_ADDRESS_8_LINES;
		s_command.DataMode = HAL_XSPI_DATA_8_LINES;
		s_command.DummyCycles = (cfg_rate == MSPI_DATA_RATE_DUAL)
						? MSPI_STM32_DUMMY_REG_OCTAL_DTR
						: MSPI_STM32_DUMMY_REG_OCTAL;
	} else {
		s_command.Instruction = MSPI_STM32_CMD_RDSR;
		/* force 1-line DataMode for any non-OSPI transfer */
		s_command.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
		s_command.AddressMode = HAL_XSPI_ADDRESS_1_LINE;
		s_command.DataMode = HAL_XSPI_DATA_1_LINE;
		s_command.DummyCycles = 0;

		/* DummyCycles remains 0 */
	}
	s_command.DataLength = (cfg_rate == MSPI_DATA_RATE_DUAL) ? 2U : 1U;
	s_command.Address = 0U;

	if (HAL_XSPI_Command(&dev_data->hmspi, &s_command, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) !=
	    HAL_OK) {
		LOG_ERR("MSPI config auto polling cmd failed");
		return -EIO;
	}
	LOG_DBG("MSPI read status reg");

	return mspi_stm32_wait_auto_polling(dev,
		MSPI_STM32_WREN_MATCH,
		MSPI_STM32_WREN_MASK,
		HAL_XSPI_TIMEOUT_DEFAULT_VALUE);
}

/* Write Flash configuration register 2 with new dummy cycles */
static int mspi_stm32_write_cfg2reg_dummy(const struct device *dev, uint8_t cfg_mode,
					  uint8_t cfg_rate)
{
	struct mspi_stm32_data *dev_data = dev->data;
	uint8_t transmit_data = MSPI_STM32_CR2_DUMMY_CYCLES_66MHZ;
	XSPI_RegularCmdTypeDef s_command = mspi_stm32_prepare_cmd(cfg_mode, cfg_rate);

	/* Initialize the writing of configuration register 2 */
	s_command.Instruction = (cfg_mode == MSPI_IO_MODE_SINGLE) ? MSPI_STM32_CMD_WR_CFGREG2
								  : MSPI_STM32_OCMD_WR_CFGREG2;
	s_command.Address = MSPI_STM32_REG2_ADDR3;
	s_command.DummyCycles = 0U;
	s_command.DataLength = (cfg_mode == MSPI_IO_MODE_SINGLE)
				       ? 1U
				       : ((cfg_rate == MSPI_DATA_RATE_DUAL) ? 2U : 1U);

	if (stm32_xspi_is_memorymap(dev)) {
		/* Abort ongoing transfer to force CS high/BUSY deasserted */
		int hal_ret = mspi_stm32_memmap_off(dev);
		if (hal_ret != 0) {
			LOG_ERR("Failed to abort memory-mapped access before write");
		return  -1;
		}
	}

	if (HAL_XSPI_Command(&dev_data->hmspi, &s_command, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) !=
	    HAL_OK) {
		LOG_ERR("MSPI transmit cmd");
		return -EIO;
	}

	if (HAL_XSPI_Transmit(&dev_data->hmspi, &transmit_data, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) !=
	    HAL_OK) {
		LOG_ERR("MSPI transmit ");
		return -EIO;
	}

	return 0;
}

/* Write Flash configuration register 2 with new single or octal SPI protocol */
static int mspi_stm32_write_cfg2reg_io(const struct device *dev, uint8_t cfg_mode, uint8_t cfg_rate,
				       uint8_t op_enable)
{
	int ret = 0;
	struct mspi_stm32_data *dev_data = dev->data;
	XSPI_RegularCmdTypeDef s_command = mspi_stm32_prepare_cmd(cfg_mode, cfg_rate);

	/* Initialize the writing of configuration register 2 */
	s_command.Instruction = (cfg_mode == MSPI_IO_MODE_SINGLE)
		? MSPI_STM32_CMD_WR_CFGREG2
		: MSPI_STM32_OCMD_WR_CFGREG2;
	s_command.Address = MSPI_STM32_REG2_ADDR1;
	s_command.DummyCycles = 0U;
	s_command.DataLength = (cfg_mode == MSPI_IO_MODE_SINGLE)
				       ? 1U
				       : ((cfg_rate == MSPI_DATA_RATE_DUAL) ? 2U : 1U);
	if (stm32_xspi_is_memorymap(dev)) {
		/* Abort ongoing transfer to force CS high/BUSY deasserted */
		ret = mspi_stm32_memmap_off(dev);
		if (ret != 0) {
			LOG_ERR("Failed to abort memory-mapped access before write");
			return  ret;
		}
	}

	if (HAL_XSPI_Command(&dev_data->hmspi, &s_command, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) !=
	    HAL_OK) {
		LOG_ERR("Write Flash configuration reg2 failed");
		return -EIO;
	}

	if (HAL_XSPI_Transmit(&dev_data->hmspi, &op_enable, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) !=
	    HAL_OK) {
		LOG_ERR("Write Flash configuration reg2 failed");
		return -EIO;
	}

	return ret;
}

/* Read Flash configuration register 2 with new single or octal SPI protocol */
static int mspi_stm32_read_cfg2reg(const struct device *dev, uint8_t cfg_mode, uint8_t cfg_rate,
				   uint8_t *value)
{
	int ret = 0;
	struct mspi_stm32_data *dev_data = dev->data;
	XSPI_RegularCmdTypeDef s_command = mspi_stm32_prepare_cmd(cfg_mode, cfg_rate);

	/* Initialize the writing of configuration register 2 */
	s_command.Instruction = (cfg_mode == MSPI_IO_MODE_SINGLE) ? MSPI_STM32_CMD_RD_CFGREG2
								  : MSPI_STM32_OCMD_RD_CFGREG2;
	s_command.Address = MSPI_STM32_REG2_ADDR1;
	s_command.DummyCycles =
		(cfg_mode == MSPI_IO_MODE_SINGLE)
			? 0U
			: ((cfg_rate == MSPI_DATA_RATE_DUAL)
				? MSPI_STM32_DUMMY_REG_OCTAL_DTR
			    : MSPI_STM32_DUMMY_REG_OCTAL);
	s_command.DataLength = (cfg_rate == MSPI_DATA_RATE_DUAL) ? 2U : 1U;
	if (stm32_xspi_is_memorymap(dev)) {
		/* Abort ongoing transfer to force CS high/BUSY deasserted */
		ret = mspi_stm32_memmap_off(dev);
		if (ret != 0) {
			LOG_ERR("Failed to abort memory-mapped access before write");
			return  ret;
		}
	}
	if (HAL_XSPI_Command(&dev_data->hmspi, &s_command, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) !=
	    HAL_OK) {
		LOG_ERR("Write Flash configuration reg2 failed");
		return -EIO;
	}

	if (HAL_XSPI_Receive(&dev_data->hmspi, value, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		LOG_ERR("Write Flash configuration reg2 failed");
		return -EIO;
	}

	return ret;
}

/* function to Send the command to configure the device according to the DTS */
static int mspi_stm32_config_mem(const struct device *dev, uint8_t cfg_mode, uint8_t cfg_rate)
{
	struct mspi_stm32_data *dev_data = dev->data;
	uint8_t reg[2];

	/* MSPI_IO_MODE_SINGLE/MSPI_DATA_RATE_SINGLE is already done */
	if ((cfg_mode == MSPI_IO_MODE_SINGLE) && (cfg_rate == MSPI_DATA_RATE_SINGLE)) {
		return 0;
	}

	/* The following sequence is given by the ospi/xspi stm32 driver but do not set WE */

	/* Write Configuration register 2 (with new dummy cycles) */
	if (mspi_stm32_write_cfg2reg_dummy(dev, MSPI_IO_MODE_SINGLE, MSPI_DATA_RATE_SINGLE) != 0) {
		LOG_ERR("XSPI write CFGR2 failed");
		return -EIO;
	}
	if (mspi_stm32_mem_ready(dev, MSPI_IO_MODE_SINGLE, MSPI_DATA_RATE_SINGLE) != 0) {
		LOG_ERR("XSPI autopolling failed");
		return -EIO;
	}
	if (mspi_stm32_write_enable(dev, MSPI_IO_MODE_SINGLE, MSPI_DATA_RATE_SINGLE) != 0) {
		LOG_ERR("XSPI write Enable 2 failed");
		return -EIO;
	}

	/* Write Configuration register 2 (with Octal I/O SPI protocol : choose STR or DTR) */
	uint8_t mode_enable = ((cfg_rate == MSPI_DATA_RATE_DUAL)
			? MSPI_STM32_CR2_DTR_OPI_EN
			: MSPI_STM32_CR2_STR_OPI_EN);
	if (mspi_stm32_write_cfg2reg_io(dev, MSPI_IO_MODE_SINGLE, MSPI_DATA_RATE_SINGLE,
					mode_enable) != 0) {
		LOG_ERR("XSPI write CFGR2 failed");
		return -EIO;
	}

	/* Wait that the configuration is effective and check that memory is ready */
	k_busy_wait(MSPI_STM32_WRITE_REG_MAX_TIME * USEC_PER_MSEC);

	/* Reconfigure the memory type of the peripheral */
	dev_data->hmspi.Init.MemoryType = HAL_XSPI_MEMTYPE_MACRONIX;
	dev_data->hmspi.Init.DelayHoldQuarterCycle = HAL_XSPI_DHQC_ENABLE;
	if (HAL_XSPI_Init(&dev_data->hmspi) != HAL_OK) {
		LOG_ERR("XSPI mem type MACRONIX failed");
		return -EIO;
	}

	if (mspi_stm32_mem_ready(dev, MSPI_IO_MODE_OCTAL, cfg_rate) != 0) {
		/* Check Flash busy ? */
		LOG_ERR("XSPI flash busy failed");
		return -EIO;
	}

	if (mspi_stm32_read_cfg2reg(dev, MSPI_IO_MODE_OCTAL, cfg_rate, reg) != 0) {
		/* Check the configuration has been correctly done on HAL_XSPI_REG2_ADDR1 */
		LOG_ERR("MSPI flash config read failed");
		return -EIO;
	}

	LOG_DBG("XSPI flash config is OCTO / %s", ((cfg_rate == MSPI_DATA_RATE_SINGLE) ?
		(char *)"STR" :
		(char *)"DTR"));

	return 0;
}

static void mspi_stm32_isr(const struct device *dev)
{
	struct mspi_stm32_data *dev_data = dev->data;

	HAL_XSPI_IRQHandler(&dev_data->hmspi);
}

#if !defined(CONFIG_SOC_SERIES_STM32H7X)
/* weak function required for HAL compilation */
__weak HAL_StatusTypeDef HAL_DMA_Abort_IT(DMA_HandleTypeDef *hdma)
{
	return HAL_OK;
}

/* weak function required for HAL compilation */
__weak HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *hdma)
{
	return HAL_OK;
}
#endif /* !CONFIG_SOC_SERIES_STM32H7X */

/*
 * Transfer Error callback.
 */
void HAL_XSPI_ErrorCallback(XSPI_HandleTypeDef *hmspi)
{
	struct mspi_stm32_data *dev_data = CONTAINER_OF(hmspi, struct mspi_stm32_data, hmspi);

	LOG_DBG("Error cb");

	k_sem_give(&dev_data->sync);
}

/*
 * Command completed callback.
 */
void HAL_XSPI_CmdCpltCallback(XSPI_HandleTypeDef *hmspi)
{
	struct mspi_stm32_data *dev_data = CONTAINER_OF(hmspi, struct mspi_stm32_data, hmspi);

	LOG_DBG("Cmd Cplt cb");

	k_sem_give(&dev_data->sync);
}

/*
 * Rx Transfer completed callback.
 */
void HAL_XSPI_RxCpltCallback(XSPI_HandleTypeDef *hmspi)
{
	struct mspi_stm32_data *dev_data = CONTAINER_OF(hmspi, struct mspi_stm32_data, hmspi);

	LOG_DBG("Rx Cplt cb");

	k_sem_give(&dev_data->sync);
}

/*
 * Tx Transfer completed callback.
 */
void HAL_XSPI_TxCpltCallback(XSPI_HandleTypeDef *hmspi)
{
	struct mspi_stm32_data *dev_data = CONTAINER_OF(hmspi, struct mspi_stm32_data, hmspi);

	LOG_DBG("Tx Cplt cb");

	dev_data->ctx.packets_done++;

	k_sem_give(&dev_data->sync);
}

/*
 * Status Match callback.
 */
void HAL_XSPI_StatusMatchCallback(XSPI_HandleTypeDef *hmspi)
{
	struct mspi_stm32_data *dev_data = CONTAINER_OF(hmspi, struct mspi_stm32_data, hmspi);

	LOG_DBG("Status Match cb");

	k_sem_give(&dev_data->sync);
}

/*
 * Timeout callback.
 */
void HAL_XSPI_TimeOutCallback(XSPI_HandleTypeDef *hmspi)
{
	struct mspi_stm32_data *dev_data = CONTAINER_OF(hmspi, struct mspi_stm32_data, hmspi);

	LOG_DBG("Timeout cb");

	k_sem_give(&dev_data->sync);
}

/**
 * Verify if the device with dev_id is on this MSPI bus.
 *
 * @param controller Pointer to the device structure for the driver instance.
 * @param dev_id Pointer to the device ID structure from a device.
 * @return 0 The device is on this MSPI bus.
 * @return -ENODEV The device is not on this MSPI bus.
 */
static inline int mspi_verify_device(const struct device *controller,
				     const struct mspi_dev_id *dev_id)
{
	const struct mspi_stm32_conf *cfg = controller->config;
	int device_index = cfg->mspicfg.num_periph;
	int ret = 0;

	if (cfg->mspicfg.num_ce_gpios != 0) {
		for (int i = 0; i < cfg->mspicfg.num_periph; i++) {
			if (dev_id->ce.port == cfg->mspicfg.ce_group[i].port &&
			    dev_id->ce.pin == cfg->mspicfg.ce_group[i].pin &&
			    dev_id->ce.dt_flags == cfg->mspicfg.ce_group[i].dt_flags) {
				device_index = i;
			}
		}

		if (device_index >= cfg->mspicfg.num_periph || device_index != dev_id->dev_idx) {
			LOG_ERR("%u, invalid device ID.", __LINE__);
			return -ENODEV;
		}
	} else {
		if (dev_id->dev_idx >= cfg->mspicfg.num_periph) {
			LOG_ERR("%u, invalid device ID.", __LINE__);
			return -ENODEV;
		}
	}

	return ret;
}

/**
 * Check and save dev_cfg to controller data->dev_cfg.
 *
 * @param controller Pointer to the device structure for the driver instance.
 * @param param_mask Macro definition of what to be configured in cfg.
 * @param dev_cfg The device runtime configuration for the MSPI controller.
 * @return 0 MSPI device configuration successful.
 * @return -Error MSPI device configuration fail.
 */
static int mspi_dev_cfg_save(const struct device *controller,
					  const enum mspi_dev_cfg_mask param_mask,
					  const struct mspi_dev_cfg *dev_cfg)
{
	const struct mspi_stm32_conf *cfg = controller->config;
	struct mspi_stm32_data *data = controller->data;

	if (param_mask & MSPI_DEVICE_CONFIG_CE_NUM) {
		data->dev_cfg.ce_num = dev_cfg->ce_num;
	}

	if (param_mask & MSPI_DEVICE_CONFIG_FREQUENCY) {
		if (dev_cfg->freq > MSPI_MAX_FREQ) {
			LOG_ERR("%u, freq is too large.", __LINE__);
			return -ENOTSUP;
		}
		data->dev_cfg.freq = dev_cfg->freq;
	}

	if (param_mask & MSPI_DEVICE_CONFIG_IO_MODE) {
		if (dev_cfg->io_mode >= MSPI_IO_MODE_MAX) {
			LOG_ERR("%u, Invalid io_mode.", __LINE__);
			return -EINVAL;
		}
		data->dev_cfg.io_mode = dev_cfg->io_mode;
	}

	if (param_mask & MSPI_DEVICE_CONFIG_DATA_RATE) {
		if (dev_cfg->data_rate >= MSPI_DATA_RATE_MAX) {
			LOG_ERR("%u, Invalid data_rate.", __LINE__);
			return -EINVAL;
		}
		data->dev_cfg.data_rate = dev_cfg->data_rate;
	}

	if (param_mask & MSPI_DEVICE_CONFIG_CPP) {
		if (dev_cfg->cpp > MSPI_CPP_MODE_3) {
			LOG_ERR("%u, Invalid cpp.", __LINE__);
			return -EINVAL;
		}
		data->dev_cfg.cpp = dev_cfg->cpp;
	}

	if (param_mask & MSPI_DEVICE_CONFIG_ENDIAN) {
		if (dev_cfg->endian > MSPI_XFER_BIG_ENDIAN) {
			LOG_ERR("%u, Invalid endian.", __LINE__);
			return -EINVAL;
		}
		data->dev_cfg.endian = dev_cfg->endian;
	}

	if (param_mask & MSPI_DEVICE_CONFIG_CE_POL) {
		if (dev_cfg->ce_polarity > MSPI_CE_ACTIVE_HIGH) {
			LOG_ERR("%u, Invalid ce_polarity.", __LINE__);
			return -EINVAL;
		}
		data->dev_cfg.ce_polarity = dev_cfg->ce_polarity;
	}

	if (param_mask & MSPI_DEVICE_CONFIG_DQS) {
		if (dev_cfg->dqs_enable && !cfg->mspicfg.dqs_support) {
			LOG_ERR("%u, DQS mode not supported.", __LINE__);
			return -ENOTSUP;
		}
		data->dev_cfg.dqs_enable = dev_cfg->dqs_enable;
	}

	if (param_mask & MSPI_DEVICE_CONFIG_RX_DUMMY) {
		data->dev_cfg.rx_dummy = dev_cfg->rx_dummy;
	}

	if (param_mask & MSPI_DEVICE_CONFIG_TX_DUMMY) {
		data->dev_cfg.tx_dummy = dev_cfg->tx_dummy;
	}

	if (param_mask & MSPI_DEVICE_CONFIG_READ_CMD) {
		data->dev_cfg.read_cmd = dev_cfg->read_cmd;
	}

	if (param_mask & MSPI_DEVICE_CONFIG_WRITE_CMD) {
		data->dev_cfg.write_cmd = dev_cfg->write_cmd;
	}

	if (param_mask & MSPI_DEVICE_CONFIG_CMD_LEN) {
		data->dev_cfg.cmd_length = dev_cfg->cmd_length;
	}

	if (param_mask & MSPI_DEVICE_CONFIG_ADDR_LEN) {
		data->dev_cfg.addr_length = dev_cfg->addr_length;
	}

	if (param_mask & MSPI_DEVICE_CONFIG_MEM_BOUND) {
		data->dev_cfg.mem_boundary = dev_cfg->mem_boundary;
	}

	if (param_mask & MSPI_DEVICE_CONFIG_BREAK_TIME) {
		data->dev_cfg.time_to_break = dev_cfg->time_to_break;
	}

	return 0;
}

/**
 * API implementation of mspi_dev_config : controller device specific configuration
 *
 * @param controller Pointer to the device structure for the driver instance.
 * @param dev_id Pointer to the device ID structure from a device.
 * @param param_mask Macro definition of what to be configured in cfg.
 * @param dev_cfg The device runtime configuration for the MSPI controller.
 *
 * @retval 0 if successful.
 * @retval -EINVAL invalid capabilities, failed to configure device.
 * @retval -ENOTSUP capability not supported by MSPI peripheral.
 */
static int mspi_stm32_dev_config(const struct device *controller, const struct mspi_dev_id *dev_id,
				 const enum mspi_dev_cfg_mask param_mask,
				 const struct mspi_dev_cfg *dev_cfg)
{
	const struct mspi_stm32_conf *cfg = controller->config;
	struct mspi_stm32_data *data = controller->data;
	int ret = 0;

	if (data->dev_id != dev_id) {
		if (k_mutex_lock(&data->lock, K_MSEC(CONFIG_MSPI_COMPLETION_TIMEOUT_TOLERANCE))) {
			LOG_ERR("MSPI config failed to access controller.");
			return -EBUSY;
		}

		ret = mspi_verify_device(controller, dev_id);
		if (ret) {
			goto e_return;
		}
	}

	if (mspi_is_inp(controller)) {
		ret = -EBUSY;
		goto e_return;
	}

	if (param_mask == MSPI_DEVICE_CONFIG_NONE && !cfg->mspicfg.sw_multi_periph) {
		/* Do nothing except obtaining the controller lock */
		data->dev_id = (struct mspi_dev_id *)dev_id;
		return ret;
	}

	/* Proceed step by step in configuration */
	if (param_mask & (MSPI_DEVICE_CONFIG_IO_MODE | MSPI_DEVICE_CONFIG_DATA_RATE)) {
		/* Going to set the XSPI mode and transfer rate */
		ret = mspi_stm32_config_mem(controller, dev_cfg->io_mode, dev_cfg->data_rate);
		if (ret) {
			goto e_return;
		}
		LOG_DBG("MSPI confg'd in %d / %d", dev_cfg->io_mode, dev_cfg->data_rate);
	}

	/*
	 * The SFDP is able to change the addr_length 4bytes or 3bytes
	 * this is reflected by the serial_cfg
	 */
	data->dev_id = (struct mspi_dev_id *)dev_id;
	/* Go on with other parameters if supported */
	if(mspi_dev_cfg_save(controller, param_mask, dev_cfg)){
		LOG_ERR("failed to change device cfg");
		return -1;
	}
e_return:
	k_mutex_unlock(&data->lock);

	return ret;
}

/**
 * API implementation of mspi_xip_config : XIP configuration
 *
 * @param controller Pointer to the device structure for the driver instance.
 * @param dev_id Pointer to the device ID structure from a device.
 * @param xip_cfg The controller XIP configuration for MSPI.
 *
 * @retval 0 if successful.
 * @retval -ESTALE device ID don't match, need to call mspi_dev_config first.
 */
static int mspi_stm32_xip_config(const struct device *controller, const struct mspi_dev_id *dev_id,
				 const struct mspi_xip_cfg *xip_cfg)
{
	struct mspi_stm32_data *dev_data = controller->data;
	int ret = 0;

	if (dev_id != dev_data->dev_id) {
		LOG_ERR("dev_id don't match");
		return -ESTALE;
	}

	if (!xip_cfg->enable) {
		/* This is for aborting */
		ret = mspi_stm32_memmap_off(controller);
	} else {
		ret = mspi_stm32_memmap_on(controller);
	}

	if (ret == 0) {
		dev_data->xip_cfg = *xip_cfg;
		LOG_INF("XIP configured %d", xip_cfg->enable);
	}
	return ret;
}

/**
 * API implementation of mspi_get_channel_status.
 *
 * @param controller Pointer to the device structure for the driver instance.
 * @param ch Not used.
 *
 * @retval 0 if successful.
 * @retval -EBUSY MSPI bus is busy
 */
static int mspi_stm32_get_channel_status(const struct device *controller, uint8_t ch)
{
	struct mspi_stm32_data *dev_data = controller->data;
	int ret = 0;

	ARG_UNUSED(ch);

	if (mspi_is_inp(controller) || (HAL_XSPI_GET_FLAG(&dev_data->hmspi, HAL_XSPI_FLAG_BUSY) == SET)) {
		ret = -EBUSY;
	}

	dev_data->dev_id = NULL;

	return ret;
}


static int mspi_stm32_pio_transceive(const struct device *controller, const struct mspi_xfer *xfer)
{
	int ret = 0;
	uint32_t packet_idx;
	struct mspi_stm32_data *dev_data = controller->data;
	struct mspi_context *ctx = &dev_data->ctx;
	const struct mspi_xfer_packet *packet;

	if (xfer->num_packet == 0 || !xfer->packets ||
	    xfer->timeout > CONFIG_MSPI_COMPLETION_TIMEOUT_TOLERANCE) {
		LOG_ERR("Transfer: wrong parameters");
		return -EFAULT;
	}

	/* DummyCycle to give to the mspi_stm32_read_access/mspi_stm32_write_access */
	ret = mspi_context_lock(ctx, dev_data->dev_id, xfer, true);
	/** For async, user must make sure when cfg_flag = 0 the dummy and instr addr length
	 * in mspi_xfer of the two calls are the same if the first one has not finished yet.
	 */
	if (ret) {
		goto pio_end;
	}

	while (ctx->packets_left > 0) {
		packet_idx = ctx->xfer.num_packet - ctx->packets_left;
		packet = &ctx->xfer.packets[packet_idx];
		/*
			* Always starts with a command,
			* then payload is given by the xfer->num_packet
			*/
		ret = mspi_stm32_access(controller, packet, (ctx->xfer.async == true) ? MSPI_ACCESS_ASYNC : MSPI_ACCESS_SYNC);

		ctx->packets_left--;
		if (ret) {
			ret = -EIO;
			goto pio_end;
		}
	}

pio_end:
	k_sem_give(&ctx->lock);
	return ret;
}

/**
 * API implementation of mspi_transceive.
 *
 * @param controller Pointer to the device structure for the driver instance.
 * @param dev_id Pointer to the device ID structure from a device.
 * @param xfer Pointer to the MSPI transfer started by dev_id.
 *
 * @retval 0 if successful.
 * @retval -ESTALE device ID don't match, need to call mspi_dev_config first.
 * @retval -Error transfer failed.
 */
static int mspi_stm32_transceive(const struct device *controller, const struct mspi_dev_id *dev_id,
				 const struct mspi_xfer *xfer)
{
	struct mspi_stm32_data *dev_data = controller->data;

	if (dev_id != dev_data->dev_id) {
		LOG_ERR("transceive : dev_id don't match");
		return -ESTALE;
	}

	/* Need to map the xfer to the data context */
	dev_data->ctx.xfer = *xfer;

	/*
	 * async + MSPI_PIO : Use callback on Irq if PIO
	 * sync + MSPI_PIO use timeout (mainly for NOR command and param
	 * MSPI_DMA : async/sync is meaningless with DMA (no DMA IT function)t
	 */
	if ((xfer->xfer_mode == MSPI_PIO) && ((xfer->packets->cmd == MSPI_STM32_OCMD_RDSR) ||
					      (xfer->packets->cmd == MSPI_STM32_CMD_RDSR))) {
		return mspi_stm32_status_reg(controller, xfer);
	}
	if (xfer->xfer_mode == MSPI_PIO) {
		return mspi_stm32_pio_transceive(controller, xfer);
	} else {
		return -EIO;
	}
}

/**
 * API implementation of mspi_config : controller configuration.
 *
 * @param spec Pointer to MSPI device tree spec.
 * @return 0 if successful.
 * @return -Error if fail.
 */
static int mspi_stm32_config(const struct mspi_dt_spec *spec)
{
	const struct mspi_cfg *config = &spec->config;
	const struct mspi_stm32_conf *dev_cfg = spec->bus->config;
	struct mspi_stm32_data *dev_data = spec->bus->data;

	uint32_t ahb_clock_freq;
	uint32_t prescaler = MSPI_STM32_CLOCK_PRESCALER_MIN;
	int ret = 0;

	/* Only Controller mode is supported */
	if (config->op_mode != MSPI_OP_MODE_CONTROLLER) {
		LOG_ERR("Only support MSPI controller mode.");
		return -ENOTSUP;
	}

	/* Check the max possible freq. */
	if (config->max_freq > MSPI_STM32_MAX_FREQ) {
		LOG_ERR("Max_freq %d too large.", config->max_freq);
		return -ENOTSUP;
	}

	if (config->duplex != MSPI_HALF_DUPLEX) {
		LOG_ERR("Only support half duplex mode.");
		return -ENOTSUP;
	}

	if (config->num_periph > MSPI_MAX_DEVICE) {
		LOG_ERR("Invalid MSPI peripheral number.");
		return -ENOTSUP;
	}

	/* Signals configuration */
	ret = pinctrl_apply_state(dev_cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("MSPI pinctrl setup failed");
		return ret;
	}

	if (dev_data->dev_cfg.dqs_enable && !dev_cfg->mspicfg.dqs_support) {
		LOG_ERR("MSPI dqs mismatch (not supported but enabled)");
		return -ENOTSUP;
	}

	if (!device_is_ready(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE))) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

	dev_cfg->irq_config();

	/* Max 3 domain clock are expected */
	if (dev_cfg->pclk_len > 3) {
		LOG_ERR("Could not select %d XSPI domain clock", dev_cfg->pclk_len);
		return -EIO;
	}

	/* Clock configuration */
	if (clock_control_on(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
			     (clock_control_subsys_t)&dev_cfg->pclken[0]) != 0) {
		LOG_ERR("Could not enable MSPI clock");
		return -EIO;
	}
	if (clock_control_get_rate(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
				   (clock_control_subsys_t)&dev_cfg->pclken[0],
				   &ahb_clock_freq) < 0) {
		LOG_ERR("Failed call clock_control_get_rate(pclken)");
		return -EIO;
	}

	/* Alternate clock config for peripheral if any */
	if (IS_ENABLED(MSPI_STM32_DOMAIN_CLOCK_SUPPORT) && (dev_cfg->pclk_len > 1)) {
		if (clock_control_configure(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
					    (clock_control_subsys_t)&dev_cfg->pclken[1],
					    NULL) != 0) {
			LOG_ERR("Could not select MSPI domain clock");
			return -EIO;
		}

		if (clock_control_get_rate(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
					   (clock_control_subsys_t)&dev_cfg->pclken[1],
					   &ahb_clock_freq) < 0) {
			LOG_ERR("Failed call clock_control_get_rate(pclken)");
			return -EIO;
		}
	}
	/* Clock domain corresponding to the IO-Mgr (XSPIM) */
	if (IS_ENABLED(MSPI_STM32_DOMAIN_CLOCK_SUPPORT) && (dev_cfg->pclk_len > 2)) {
		if (clock_control_on(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
				     (clock_control_subsys_t)&dev_cfg->pclken[2]) != 0) {
			LOG_ERR("Could not enable XSPI Manager clock");
			return -EIO;
		}
	}

	for (; prescaler <= MSPI_STM32_CLOCK_PRESCALER_MAX; prescaler++) {
		dev_data->dev_cfg.freq = MSPI_STM32_CLOCK_COMPUTE(ahb_clock_freq, prescaler);

		if (dev_data->dev_cfg.freq <= dev_cfg->mspicfg.max_freq) {
			break;
		}
	}
	__ASSERT_NO_MSG(prescaler >= MSPI_STM32_CLOCK_PRESCALER_MIN &&
			prescaler <= MSPI_STM32_CLOCK_PRESCALER_MAX);

	/* Initialize XSPI HAL structure completely */
	dev_data->hmspi.Init.ClockPrescaler = prescaler;
	/** The stm32 hal_mspi driver does not reduce DEVSIZE before writing the DCR1
	 * dev_data->hmspi.Init.MemorySize = find_lsb_set(dev_cfg->reg_size) - 2;
	 * dev_data->hmspi.Init.MemorySize is mandatory now (BUSY = 0) for HAL_XSPI Init
	 * give the value from the child node
	 */
#if defined(XSPI_DCR2_WRAPSIZE)
	dev_data->hmspi.Init.WrapSize = HAL_XSPI_WRAP_NOT_SUPPORTED;
#endif /* XSPI_DCR2_WRAPSIZE */
	/* STR mode else Macronix for DTR mode */
	if (dev_data->dev_cfg.data_rate == MSPI_DATA_RATE_DUAL) {
		dev_data->hmspi.Init.MemoryType = HAL_XSPI_MEMTYPE_MACRONIX;
		dev_data->hmspi.Init.DelayHoldQuarterCycle = HAL_XSPI_DHQC_ENABLE;
	} else {
		dev_data->hmspi.Init.MemoryType = HAL_XSPI_MEMTYPE_MICRON;
		dev_data->hmspi.Init.DelayHoldQuarterCycle = HAL_XSPI_DHQC_DISABLE;
	}
#if MSPI_STM32_DLYB_BYPASSED
	dev_data->hmspi.Init.DelayBlockBypass = HAL_XSPI_DELAY_BLOCK_BYPASS;
#else
	dev_data->hmspi.Init.DelayBlockBypass = HAL_XSPI_DELAY_BLOCK_ON;
#endif /* MSPI_STM32_DLYB_BYPASSED */

	if (HAL_XSPI_Init(&dev_data->hmspi) != HAL_OK) {
		LOG_ERR("MSPI Init failed");
		return -EIO;
	}

	LOG_DBG("MSPI Init'd");

#if defined(HAL_XSPIM_IOPORT_1) || defined(HAL_XSPIM_IOPORT_2)
	/* XSPI I/O manager init Function */
	XSPIM_CfgTypeDef mspi_mgr_cfg;

	if (dev_data->hmspi.Instance == XSPI1) {
		mspi_mgr_cfg.IOPort = HAL_XSPIM_IOPORT_1;
	} else if (dev_data->hmspi.Instance == XSPI2) {
		mspi_mgr_cfg.IOPort = HAL_XSPIM_IOPORT_2;
	}
	mspi_mgr_cfg.nCSOverride = HAL_XSPI_CSSEL_OVR_DISABLED;
	mspi_mgr_cfg.Req2AckTime = 1;

	if (HAL_XSPIM_Config(&dev_data->hmspi, &mspi_mgr_cfg, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) !=
	    HAL_OK) {
		LOG_ERR("XSPI M config failed");
		return -EIO;
	}

#endif /* XSPIM */

#if defined(DLYB_XSPI1) || defined(DLYB_XSPI2) || defined(DLYB_OCTOSPI1) || defined(DLYB_OCTOSPI2)
	/* XSPI delay block init Function */
	HAL_XSPI_DLYB_CfgTypeDef mspi_delay_block_cfg = {0};

	(void)HAL_XSPI_DLYB_GetClockPeriod(&dev_data->hmspi, &mspi_delay_block_cfg);
	/*  with DTR, set the PhaseSel/4 (empiric value from stm32Cube) */
	mspi_delay_block_cfg.PhaseSel /= 4;

	if (HAL_XSPI_DLYB_SetConfig(&dev_data->hmspi, &mspi_delay_block_cfg) != HAL_OK) {
		LOG_ERR("XSPI DelayBlock failed");
		return -EIO;
	}

	LOG_DBG("Delay Block Init");

#endif /* DLYB_ */

	if (!k_sem_count_get(&dev_data->ctx.lock)) {
		//dev_data->ctx.owner = NULL;
		k_sem_give(&dev_data->ctx.lock);
	}

	if (config->re_init) {
		k_mutex_unlock(&dev_data->lock);
	}

	LOG_INF("MSPI config'd");

	return 0;
}

/**
 * Set up a new controller and add its child to the list.
 *
 * @param dev MSPI emulation controller.
 *
 * @retval 0 if successful.
 */
static int mspi_stm32_init(const struct device *controller)
{
	const struct mspi_stm32_conf *cfg = controller->config;
	const struct mspi_dt_spec spec = {
		.bus = controller,
		.config = cfg->mspicfg,
	};

	return mspi_stm32_config(&spec);
}

static struct mspi_driver_api mspi_stm32_driver_api = {
	.config = mspi_stm32_config,
	.dev_config = mspi_stm32_dev_config,
	.xip_config = mspi_stm32_xip_config,
	.get_channel_status = mspi_stm32_get_channel_status,
	.transceive = mspi_stm32_transceive,
};

/* MSPI control config */
#define MSPI_CONFIG(n)                                                                             \
	{                                                                                          \
		.channel_num = 0, .op_mode = DT_ENUM_IDX_OR(n, op_mode, MSPI_OP_MODE_CONTROLLER),  \
		.duplex = DT_ENUM_IDX_OR(n, duplex, MSPI_HALF_DUPLEX),                             \
		.max_freq = DT_INST_PROP_OR(n, mspi_max_frequency, MSPI_STM32_MAX_FREQ),           \
		.dqs_support = DT_INST_PROP_OR(n, dqs_support, false),                             \
		.num_periph = DT_INST_CHILD_NUM(n),                                                \
		.sw_multi_periph = DT_INST_PROP_OR(n, software_multiperipheral, false),            \
	}

#define STM32_SMPI_IRQ_HANDLER(index)								\
	static void mspi_stm32_irq_config_func_##index(void)					\
	{											\
		IRQ_CONNECT(DT_INST_IRQN(index), DT_INST_IRQ(index, priority), mspi_stm32_isr,	\
			DEVICE_DT_INST_GET(index), 0);	\
		irq_enable(DT_INST_IRQN(index));	\
	}

#define MSPI_STM32_INIT(index)                                                              \
	static const struct stm32_pclken pclken_##index[] = STM32_DT_INST_CLOCKS(index);	\
	PINCTRL_DT_INST_DEFINE(index);								\
	\
	static struct gpio_dt_spec ce_gpios##n[] = MSPI_CE_GPIOS_DT_SPEC_INST_GET(n);             \
	STM32_SMPI_IRQ_HANDLER(index)								\
	static const struct mspi_stm32_conf mspi_stm32_dev_conf_##index = {	\
		.pclken = pclken_##index,	\
		.pclk_len = DT_INST_NUM_CLOCKS(index),	\
		.irq_config =  mspi_stm32_irq_config_func_##index,				\
		.mspicfg = MSPI_CONFIG(index),	\
		.pcfg = PINCTRL_DT_DEV_CONFIG_GET(DT_DRV_INST(index)),	\
		.mspicfg.num_ce_gpios  = ARRAY_SIZE(ce_gpios##n),				\
	};											\
	static struct mspi_stm32_data mspi_stm32_dev_data_##index = {                           \
		.hmspi = {                                                                          \
		.Instance = (XSPI_TypeDef *)DT_INST_REG_ADDR(index),                             \
		.Init = {                                                                        \
			.FifoThresholdByte = MSPI_STM32_FIFO_THRESHOLD,                              \
			.SampleShifting = (DT_INST_PROP(index, ssht_enable)                          \
					? HAL_XSPI_SAMPLE_SHIFT_HALFCYCLE                       \
					: HAL_XSPI_SAMPLE_SHIFT_NONE),                           \
			.ChipSelectHighTimeCycle = 1,                                                \
			.ClockMode = HAL_XSPI_CLOCK_MODE_0,                                          \
			.ChipSelectBoundary = 0,                                                     \
			.MemoryMode = HAL_XSPI_SINGLE_MEM,                                           \
			.MemorySize = 0x19,                                                          \
			.FreeRunningClock = HAL_XSPI_FREERUNCLK_DISABLE,                              \
		},                                                                               \
		},                                                                                   \
		.memmap_base_addr  = DT_REG_ADDR_BY_IDX(DT_DRV_INST(index), 1),	\
		.dev_id = index,                                                                     \
		.lock = Z_MUTEX_INITIALIZER(mspi_stm32_dev_data_##index.lock),                       \
		.sync = Z_SEM_INITIALIZER(mspi_stm32_dev_data_##index.sync, 0, 1),                   \
		.dev_cfg = {0},                                                                      \
		.xip_cfg = {0},                                                                      \
		.ctx.lock = Z_SEM_INITIALIZER(mspi_stm32_dev_data_##index.ctx.lock, 0, 1),           \
	};                                                                                       \
	\
	DEVICE_DT_INST_DEFINE(index, &mspi_stm32_init, NULL, &mspi_stm32_dev_data_##index,       \
				&mspi_stm32_dev_conf_##index, POST_KERNEL,                                 \
				CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &mspi_stm32_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MSPI_STM32_INIT)
