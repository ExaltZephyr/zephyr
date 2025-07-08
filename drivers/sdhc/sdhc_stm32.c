/*
 * Copyright (c) 2025 EXALT Technologies.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_stm32_sdhc

#include <zephyr/drivers/sdhc.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>


LOG_MODULE_REGISTER(sdhc_stm32, CONFIG_SDHC_LOG_LEVEL);
typedef void (*irq_config_func_t)(const struct device *port);

#define SD_TIMEOUT        ((uint32_t)0x00100000U)
#define SDHC_CMD_TIMEOUT  K_MSEC(200)

struct sdhc_stm32_config {
	uint16_t clk_div;
	bool hw_flow_control;
	unsigned int max_freq;             /* Max bus frequency in Hz*/
	unsigned int min_freq;             /* Min bus frequency in Hz*/
	uint8_t bus_width;                 /* Width of the SDIO bus (1-bit or 4-bit mode) */
	SD_HandleTypeDef *hsd;           /* Pointer to SD HAL handle */
	uint32_t power_delay_ms;      /* power delay prop for the host in milliseconds */
	const struct stm32_pclken *pclken;              /* Pointer to peripheral clock configuration */
	const struct pinctrl_dev_config *pcfg;               /* Pointer to pin control configuration */
	irq_config_func_t irq_config_func;    /* IRQ config function */
};

struct sdhc_stm32_data {
	struct k_mutex bus_mutex; 
	struct sdhc_io host_io;
	struct sdhc_host_props props;  /* currect host properties */
	struct k_sem cmd_sem;
};

void sdhc_stm32_log_err_type(SD_HandleTypeDef *hsd)
{
	uint32_t error_code = HAL_SD_GetError(hsd);

	// Timeout and busy errors
	if (error_code & (HAL_SD_ERROR_TIMEOUT | HAL_SD_ERROR_CMD_RSP_TIMEOUT)) {
		LOG_ERR("SDIO Timeout Error\n");
	}
	if (error_code & HAL_SD_ERROR_BUSY) {
		LOG_ERR("SDIO Busy Error\n");
	}

	// CRC-related errors
	if (error_code & (HAL_SD_ERROR_CMD_CRC_FAIL | HAL_SD_ERROR_DATA_CRC_FAIL | HAL_SD_ERROR_COM_CRC_FAILED)) {
		LOG_ERR("SDIO CRC Error (Command/Data/Communication)\n");
	}

	// FIFO underrun/overrun errors
	if (error_code & HAL_SD_ERROR_TX_UNDERRUN) {
		LOG_ERR("SDIO FIFO Transmit Underrun Error\n");
	}
	if (error_code & HAL_SD_ERROR_RX_OVERRUN) {
		LOG_ERR("SDIO FIFO Receive Overrun Error\n");
	}

	// Address-related errors
	if (error_code & (HAL_SD_ERROR_ADDR_MISALIGNED | HAL_SD_ERROR_ADDR_OUT_OF_RANGE)) {
		LOG_ERR("SDIO Address Error (Misaligned/Out of Range)\n");
	}

	// Block and erase errors
	if (error_code & (HAL_SD_ERROR_BLOCK_LEN_ERR | HAL_SD_ERROR_ERASE_SEQ_ERR | HAL_SD_ERROR_BAD_ERASE_PARAM | HAL_SD_ERROR_WP_ERASE_SKIP)) {
		LOG_ERR("SDIO Block/Erase Error\n");
	}

	// Card-related errors
	if (error_code & (HAL_SD_ERROR_WRITE_PROT_VIOLATION | HAL_SD_ERROR_LOCK_UNLOCK_FAILED | HAL_SD_ERROR_ILLEGAL_CMD)) {
		LOG_ERR("SDIO Card Access Error (Write Protect/Lock/Illegal Command)\n");
	}
	if (error_code & (HAL_SD_ERROR_CARD_ECC_FAILED | HAL_SD_ERROR_CARD_ECC_DISABLED)) {
		LOG_ERR("SDIO Card ECC Error (Failed/Disabled)\n");
	}
	if (error_code & HAL_SD_ERROR_CC_ERR) {
		LOG_ERR("SDIO Card Controller Error\n");
	}

	// Stream and overwrite errors
	if (error_code & (HAL_SD_ERROR_STREAM_READ_UNDERRUN | HAL_SD_ERROR_STREAM_WRITE_OVERRUN)) {
		LOG_ERR("SDIO Stream Error (Read Underrun/Write Overrun)\n");
	}
	if (error_code & HAL_SD_ERROR_CID_CSD_OVERWRITE) {
		LOG_ERR("SDIO CID/CSD Overwrite Error\n");
	}

	// Other general errors
	if (error_code & (HAL_SD_ERROR_GENERAL_UNKNOWN_ERR | HAL_SD_ERROR_ERASE_RESET | HAL_SD_ERROR_AKE_SEQ_ERR | HAL_SD_ERROR_REQUEST_NOT_APPLICABLE)) {
		LOG_ERR("SDIO General Error\n");
	}
	if (error_code & HAL_SD_ERROR_PARAM) {
		LOG_ERR("SDIO Parameter Error\n");
	}
	if (error_code & HAL_SD_ERROR_INVALID_VOLTRANGE) {
		LOG_ERR("SDIO Invalid Voltage Range Error\n");
	}
	if (error_code & HAL_SD_ERROR_UNSUPPORTED_FEATURE) {
		LOG_ERR("SDIO Unsupported Feature Error\n");
	}
	if (error_code & HAL_SD_ERROR_DMA) {
		LOG_ERR("SDIO DMA Error\n");
	}
	hsd->ErrorCode = HAL_SD_ERROR_NONE;
}

static int sdhc_stm32_write_blocks(const struct device *dev, struct sdhc_data *data){
	int ret;
	struct sdhc_stm32_data *dev_data = dev->data;
	const struct sdhc_stm32_config *config = dev->config;
	__ASSERT(data != NULL, "Data is required for this command.");

	#ifdef CONFIG_SDHC_STM32_DMA
		ret = HAL_SD_WriteBlocks_DMA(config->hsd, data->data, data->block_addr, data->blocks);
	#else
		ret = HAL_SD_WriteBlocks_IT(config->hsd, data->data, data->block_addr, data->blocks);
	#endif

	if (k_sem_take(&dev_data->cmd_sem, SDHC_CMD_TIMEOUT) != 0) {
		LOG_ERR("Failed to acquire Semaphore\n");
		return -ETIMEDOUT;
	}

	return ret;
}

static int sdhc_stm32_read_blocks(const struct device *dev, struct sdhc_data *data){
	int ret;
	struct sdhc_stm32_data *dev_data = dev->data;
	const struct sdhc_stm32_config *config = dev->config;

	#ifdef CONFIG_SDHC_STM32_DMA
		ret = HAL_SD_ReadBlocks_DMA(config->hsd, data->data, data->block_addr, data->blocks);
	#else
		ret = HAL_SD_ReadBlocks_IT(config->hsd, data->data, data->block_addr, data->blocks);
	#endif

	if (k_sem_take(&dev_data->cmd_sem, SDHC_CMD_TIMEOUT) != 0) {
		LOG_ERR("Failed to acquire Semaphore\n");
		return -ETIMEDOUT;
	}

	return ret;
}

static uint32_t sdhc_stm32_go_idle_state(const struct device *dev){
	uint32_t res;
	struct sdhc_stm32_data *dev_data = dev->data;
	const struct sdhc_stm32_config *config = dev->config;

	res = SDMMC_CmdGoIdleState(SDMMC1);
	if (res != HAL_OK) {
		printk("go to idle failed\n");
		sdhc_stm32_log_err_type(config->hsd);
		return res;
	}

	return 0;
}

static int sdhc_stm32_abort(const struct device *dev){
	int res;
	const struct sdhc_stm32_config *config = dev->config;
	res = HAL_SD_Abort(config->hsd);
	return res;
}

static int sdhc_stm32_erase_block(const struct device *dev, struct sdhc_data *data) {
	int res;
	struct sdhc_stm32_data *dev_data = dev->data;
	const struct sdhc_stm32_config *config = dev->config;

	res = HAL_SD_Erase(config->hsd, data->block_addr, (data->block_size + data->block_addr));
	if(res != HAL_OK) {
		return res;
	}

	if( !WAIT_FOR(HAL_SD_GetCardState(config->hsd) == HAL_SD_CARD_TRANSFER, SD_TIMEOUT, k_usleep(1))) {
		LOG_ERR("SD Card is busy now");
		return HAL_ERROR;
	}
	return res;
}

static int sdhc_stm32_request(const struct device *dev, struct sdhc_command *cmd, struct sdhc_data *data)
{
	int res;
	struct sdhc_stm32_data *dev_data = dev->data;
	const struct sdhc_stm32_config *config = dev->config;

	 __ASSERT(cmd != NULL, "Command is NULL.");

	if (k_mutex_lock(&dev_data->bus_mutex, K_MSEC(cmd->timeout_ms))) {
		return -EBUSY;
	}

	switch (cmd->opcode) {
		case SD_SEND_IF_COND:
			res = SDMMC_CmdOperCond(config->hsd->Instance);
			cmd->response[0] = SDMMC_GetResponse(config->hsd->Instance, SDMMC_RESP1);
			break;

		case SD_WRITE_SINGLE_BLOCK:
		case SD_WRITE_MULTIPLE_BLOCK:
			res = sdhc_stm32_write_blocks(dev, data);
			break;

		case SD_READ_SINGLE_BLOCK:
		case SD_READ_MULTIPLE_BLOCK:
			res = sdhc_stm32_read_blocks(dev, data);
			break;

		case SD_ERASE_BLOCK_OPERATION:
			res = sdhc_stm32_erase_block(dev, data);
			break;

		case SD_GO_IDLE_STATE:
			res = sdhc_stm32_go_idle_state(dev);
			break;

		case SD_SEND_CSD:
			res = SDMMC_CmdSendCSD(config->hsd->Instance, cmd->arg);
			if(res == 0){
				cmd->response[0] = SDMMC_GetResponse(config->hsd->Instance, SDMMC_RESP1);
				cmd->response[1] = SDMMC_GetResponse(config->hsd->Instance, SDMMC_RESP2);
				cmd->response[2] = SDMMC_GetResponse(config->hsd->Instance, SDMMC_RESP3);
				cmd->response[3] = SDMMC_GetResponse(config->hsd->Instance, SDMMC_RESP4);
			}
			break;

		case SD_SEND_RELATIVE_ADDR:
			res = SDMMC_CmdSetRelAdd(config->hsd->Instance,  (uint16_t *)&cmd->response);
			if (res == 0U) {
				/*
				* Restore RCA by reversing the double 16-bit right shift from
				* Zephyr subsys and SDMMC_CmdSetRelAdd
				*/
				cmd->response[0] = cmd->response[0] << 16;
			}
			break;

		case SD_STOP_TRANSMISSION:
			res = sdhc_stm32_abort(dev);
			break;

		case SD_SWITCH:
			//res = SD_SwitchSpeed(config->hsd, cmd->arg);
			break;

		case SD_APP_CMD:
			res = SDMMC_CmdAppCommand(config->hsd->Instance, cmd->arg);
			if(res) {
				LOG_ERR("Unsupported feature\n");
				res =  -ENOTSUP;
			} else {
			cmd->response[0] =  SDMMC_GetResponse(config->hsd->Instance, SDMMC_RESP1);
			}
			break;

		case SD_APP_SEND_OP_COND:
			res = SDMMC_CmdAppOperCommand(config->hsd->Instance, cmd->arg);
			cmd->response[0] =  SDMMC_GetResponse(config->hsd->Instance, SDMMC_RESP1);
			break;

		case SD_ALL_SEND_CID:
			res = SDMMC_CmdSendCID(config->hsd->Instance);
			if(res == 0) {
				cmd->response[0] = SDMMC_GetResponse(config->hsd->Instance, SDMMC_RESP1);
				cmd->response[1] = SDMMC_GetResponse(config->hsd->Instance, SDMMC_RESP2);
				cmd->response[2] = SDMMC_GetResponse(config->hsd->Instance, SDMMC_RESP3);
				cmd->response[3] = SDMMC_GetResponse(config->hsd->Instance, SDMMC_RESP4);
			}
			break;
		case SD_SELECT_CARD:
			res = SDMMC_CmdSelDesel(config->hsd->Instance, cmd->arg);
				cmd->response[0] = SDMMC_GetResponse(config->hsd->Instance, SDMMC_RESP1);
			
			break;
		case SD_APP_SEND_SCR:
			//res = SD_FindSCR(config->hsd, data->data);
			break;
		case SD_SET_BLOCK_SIZE:
			res = SDMMC_CmdBlockLength(config->hsd->Instance, (uint32_t)cmd->arg);
			break;
		default:
			res = HAL_ERROR;
			LOG_ERR("Unsupported Command: opcode :%d.", cmd->opcode);
			break;
	}

 	if (res != 0)
	{
		LOG_ERR("Command Failed, opcode:%d", cmd->opcode);
		sdhc_stm32_log_err_type(config->hsd);
	}

	k_mutex_unlock(&dev_data->bus_mutex);

	return res;
}

static int sdhc_stm32_get_card_present(const struct device *dev)
{
	struct sdhc_stm32_data *dev_data = dev->data;
	const struct sdhc_stm32_config *config = dev->config;

	HAL_SD_CardStateTypeDef card_state = HAL_SD_GetCardState(config->hsd);
	if((HAL_SD_GetError(config->hsd) == SDMMC_ERROR_CMD_RSP_TIMEOUT) || (card_state == HAL_SD_CARD_DISCONNECTED)) {
		sdhc_stm32_log_err_type(config->hsd);
		return false;
	}

	return true;
}

static int sdhc_stm32_reset(const struct device *dev)
{
	struct sdhc_stm32_data *data = dev->data;
	const struct sdhc_stm32_config *config = dev->config;

	/* Resetting Host controller */
	(void)SDMMC_PowerState_OFF(config->hsd->Instance);
	k_msleep(data->props.power_delay);
	(void)SDMMC_PowerState_ON(config->hsd->Instance);
	k_msleep(data->props.power_delay);

	/* Resetting card */
	return sdhc_stm32_go_idle_state(dev);
}

static int sdhc_stm32_card_busy(const struct device *dev)
{
	const struct sdhc_stm32_config *config = dev->config;

	return HAL_SD_GetState(config->hsd) == HAL_SD_STATE_BUSY;

}

static int sdhc_stm32_set_io(const struct device *dev, struct sdhc_io *ios)
{
	int res = 0;
	struct sdhc_stm32_data *data = dev->data;
	struct sdhc_host_props *props = &data->props;
	struct sdhc_io *host_io = &data->host_io;
	const struct sdhc_stm32_config *config = dev->config;

	if ((ios->bus_width != 0) && (host_io->bus_width != ios->bus_width)) {
		uint32_t bus_width_reg_value;

		if (ios->bus_width == SDHC_BUS_WIDTH8BIT) {
			bus_width_reg_value = SDMMC_BUS_WIDE_8B;
		} else if (ios->bus_width == SDHC_BUS_WIDTH4BIT) {
			bus_width_reg_value = SDMMC_BUS_WIDE_4B;
		} else {
			bus_width_reg_value = SDMMC_BUS_WIDE_1B;
		}

		MODIFY_REG(config->hsd->Instance->CLKCR, SDMMC_CLKCR_WIDBUS, bus_width_reg_value);
		host_io->bus_width = ios->bus_width;
	}

	if ((ios->clock != 0) && ( host_io->clock !=ios->clock) )
	{
		if ((ios->clock > props->f_max) || (ios->clock < props->f_min)) {
			LOG_ERR("Invalid clock frequency, domain (%u, %u)",
				props->f_min, props->f_max);
			res = -EINVAL;
			goto out;
		}
		uint32_t ClockDiv = (HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SDMMC)) / (2U * ios->clock);
		MODIFY_REG(config->hsd->Instance->CLKCR, SDMMC_CLKCR_CLKDIV, ClockDiv);

		host_io->clock = ios->clock;
		LOG_DBG("Clock set to %d", ios->clock);
	}

	if (ios->power_mode == SDHC_POWER_OFF) {
		(void)SDMMC_PowerState_OFF(config->hsd->Instance);
		k_msleep(data->props.power_delay);
	} else {
		(void)SDMMC_PowerState_ON(config->hsd->Instance);
		k_msleep(data->props.power_delay);
	}

out:
	return res;
}

static int sdhc_stm32_get_host_props(const struct device *dev, struct sdhc_host_props *props)
{
	struct sdhc_stm32_data *data = dev->data;

	memcpy(props, &data->props, sizeof(struct sdhc_host_props));

	return 0;
}

static const struct sdhc_driver_api sdhc_stm32_api = {
	.request = sdhc_stm32_request,
	.get_card_present = sdhc_stm32_get_card_present,
	.reset = sdhc_stm32_reset,
	.card_busy = sdhc_stm32_card_busy,
	.set_io = sdhc_stm32_set_io,
	.get_host_props = sdhc_stm32_get_host_props,
};

static int sdhc_stm32_activate(const struct device *dev)
{
	int ret;
	const struct sdhc_stm32_config *config = (struct sdhc_stm32_config *)dev->config;
	const struct device *const clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);

	if (!device_is_ready(clk)) {
		return -ENODEV;
	}

	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	if (DT_INST_NUM_CLOCKS(0) > 1) {
		if (clock_control_configure(clk, (clock_control_subsys_t)&config->pclken[1],
						NULL) != 0) {
		LOG_ERR("Failed to enable SDHC domain clock");
		return -EIO;
		}
	}

	if (clock_control_on(clk, (clock_control_subsys_t)&config->pclken[0]) != 0) {
		return -EIO;
	}

	return 0;
}

void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd)
{
	sdhc_stm32_log_err_type(hsd);
}

int sdhc_stm32_sd_init(const struct device *dev)
{
	struct sdhc_stm32_data *data = dev->data;
	const struct sdhc_stm32_config *config = dev->config;
	SD_HandleTypeDef *hsd = config->hsd;

	if (HAL_SD_DeInit(hsd) != HAL_OK) {
		LOG_ERR("Failed to de-initialize the SDHC device");
		return -EIO;
	}

	hsd->Init.ClockEdge = SDMMC_CLOCK_EDGE_FALLING;
	hsd->Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
	hsd->Init.ClockDiv = config->clk_div;

	if(config->hw_flow_control) {
		hsd->Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_ENABLE;
	} else {
		hsd->Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
	}

	if(data->host_io.bus_width == 4) {
		hsd->Init.BusWide = SDMMC_BUS_WIDE_4B;
	} else if(data->host_io.bus_width == 8) {
		hsd->Init.BusWide = SDMMC_BUS_WIDE_8B;
	} else {
		hsd->Init.BusWide = SDMMC_BUS_WIDE_1B;
	}

	if (HAL_SD_Init(hsd) != HAL_OK) {
		return -EIO;
	}
	return 0;

}

static void sdhc_stm32_init_props(const struct device *dev)
{
	const struct sdhc_stm32_config *sdhc_config = (const struct sdhc_stm32_config *)dev->config;
	struct sdhc_stm32_data *data = dev->data;
	struct sdhc_host_props *props = &data->props;

	memset(props, 0, sizeof(struct sdhc_host_props));

	props->f_min = sdhc_config->min_freq;
	props->f_max = sdhc_config->max_freq;
	props->power_delay = sdhc_config->power_delay_ms;
	props->host_caps.vol_330_support = true;
	// props->host_caps.vol_180_support = sdhc_config->support_1_8_v;
	props->host_caps.bus_8_bit_support = (sdhc_config->bus_width == SDHC_BUS_WIDTH8BIT);
	props->host_caps.bus_4_bit_support = (sdhc_config->bus_width == SDHC_BUS_WIDTH4BIT);
}

static int sdhc_stm32_init(const struct device *dev)
{
	int ret;
	struct sdhc_stm32_data *data = dev->data;
	const struct sdhc_stm32_config *config = dev->config;

	ret = sdhc_stm32_activate(dev);
	if (ret != 0) {
		LOG_ERR("Clock and GPIO could not be initialized for the SDHC module, err=%d", ret);
		return ret;
	}

	ret = sdhc_stm32_sd_init(dev);
	if (ret != 0) {
		sdhc_stm32_log_err_type(config->hsd);
		return ret;
	}

	sdhc_stm32_init_props(dev);

	k_mutex_init(&data->bus_mutex);
	k_sem_init(&data->cmd_sem, 0, K_SEM_MAX_LIMIT);

	config->irq_config_func(dev);

	return 0;
}

static void sdhc_stm32_event_isr(void *arg)
{
	const struct device     *dev = (const struct device *)arg;
	struct sdhc_stm32_data  *data = dev->data;
	const struct sdhc_stm32_config *config = dev->config;

	if (__HAL_SD_GET_FLAG(config->hsd,
				SDMMC_FLAG_DATAEND | SDMMC_FLAG_DCRCFAIL | SDMMC_FLAG_DTIMEOUT |
				SDMMC_FLAG_RXOVERR | SDMMC_FLAG_TXUNDERR)) {
		k_sem_give(&data->cmd_sem);
	}

	HAL_SD_IRQHandler(config->hsd);
}

#define STM32_SDHC_IRQ_CONNECT_AND_ENABLE(index)	\
	do {	\
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(index, event, irq),	\
			DT_INST_IRQ_BY_NAME(index, event, priority), sdhc_stm32_event_isr,	\
			DEVICE_DT_INST_GET(index), 0);	\
		irq_enable(DT_INST_IRQ_BY_NAME(index, event, irq));	\
	} while (false)

#define STM32_SDHC_IRQ_HANDLER_DECL(index)	\
	static void sdhc_stm32_irq_config_func_##index(const struct device *dev)

#define STM32_SDHC_IRQ_HANDLER_FUNCTION(index) .irq_config_func = sdhc_stm32_irq_config_func_##index,

#define STM32_SDHC_IRQ_HANDLER(index)	\
	static void sdhc_stm32_irq_config_func_##index(const struct device *dev)	\
	{	\
		STM32_SDHC_IRQ_CONNECT_AND_ENABLE(index);	\
	}

#define SDHC_STM32_INIT(index)	\
	STM32_SDHC_IRQ_HANDLER_DECL(index);	\
	\
	static const struct stm32_pclken pclken_##index[] = STM32_DT_INST_CLOCKS(index);	\
	\
	PINCTRL_DT_INST_DEFINE(index);	\
	\
	static SD_HandleTypeDef hsd_##index = {	\
		.Instance = (MMC_TypeDef *)DT_INST_REG_ADDR(index),	\
	};	\
	\
	static const struct sdhc_stm32_config sdhc_stm32_cfg_##index = {	\
		STM32_SDHC_IRQ_HANDLER_FUNCTION(index)	\
		.hsd = &hsd_##index,	\
		.pclken = pclken_##index,	\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),	\
		.bus_width = DT_INST_PROP(index, bus_width),	\
		.hw_flow_control= DT_INST_PROP_OR(index, hw_flow_control,0),	\
		.clk_div= DT_INST_PROP_OR(index, clk_div,4),	\
		.power_delay_ms = DT_INST_PROP_OR(inst, power_delay_ms, 500),	\
		.min_freq = DT_INST_PROP(index, min_bus_freq),	\
		.max_freq = DT_INST_PROP(index, max_bus_freq),	\
	};	\
	\
	static struct sdhc_stm32_data sdhc_stm32_data_##index = {	\
		.host_io = {	\
			.bus_width= DT_INST_PROP_OR(index, bus_width, 4),	\
		}	\
	};	\
	\
	DEVICE_DT_INST_DEFINE(index,	\
			&sdhc_stm32_init,	\
			NULL,	\
			&sdhc_stm32_data_##index,	\
			&sdhc_stm32_cfg_##index,	\
			POST_KERNEL,	\
			CONFIG_SDHC_INIT_PRIORITY,	\
			&sdhc_stm32_api);	\
	STM32_SDHC_IRQ_HANDLER(index)

DT_INST_FOREACH_STATUS_OKAY(SDHC_STM32_INIT)
