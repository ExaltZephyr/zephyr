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
#include <zephyr/drivers/gpio.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/pm/policy.h>


LOG_MODULE_REGISTER(sdhc_stm32, CONFIG_SDHC_LOG_LEVEL);
typedef void (*irq_config_func_t)(const struct device *port);

#define SD_TIMEOUT        ((uint32_t)0x00100000U)
#define SDHC_CMD_TIMEOUT  K_MSEC(200)

struct sdhc_stm32_config {
	bool hw_flow_control;              /* flag for enabling hardware flow control */
	bool support_1_8_v;                /* flag indicating support for 1.8V signaling */
	unsigned int max_freq;             /* Max bus frequency in Hz*/
	unsigned int min_freq;             /* Min bus frequency in Hz*/
	uint8_t bus_width;                 /* Width of the SDIO bus (1-bit or 4-bit mode) */
	uint16_t clk_div;
	uint32_t power_delay_ms;      /* power delay prop for the host in milliseconds */
	SD_HandleTypeDef *hsd;           /* Pointer to SD HAL handle */
	const struct stm32_pclken *pclken;              /* Pointer to peripheral clock configuration */
	const struct pinctrl_dev_config *pcfg;               /* Pointer to pin control configuration */
	struct gpio_dt_spec cd_gpio; 
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
	if (error_code & (HAL_SD_ERROR_TIMEOUT | HAL_SD_ERROR_CMD_RSP_TIMEOUT | HAL_SD_ERROR_DATA_TIMEOUT )) {
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
		LOG_ERR("SDIO General Error");
	}
	if (error_code & HAL_SD_ERROR_PARAM) {
		LOG_ERR("SDIO Parameter Error");
	}
	if (error_code & HAL_SD_ERROR_INVALID_VOLTRANGE) {
		LOG_ERR("SDIO Invalid Voltage Range Error");
	}
	if (error_code & HAL_SD_ERROR_UNSUPPORTED_FEATURE) {
		LOG_ERR("SDIO Unsupported Feature Error");
	}
	if (error_code & HAL_SD_ERROR_DMA) {
		LOG_ERR("SDIO DMA Error");
	}
	if (error_code & HAL_SD_ERROR_INVALID_CALLBACK) {
		LOG_ERR("SD Invalid Callback Error");
	}
	hsd->ErrorCode = HAL_SD_ERROR_NONE;
}

static int sdhc_stm32_write_blocks(const struct device *dev, struct sdhc_data *data){
	int ret;
	struct sdhc_stm32_data *dev_data = dev->data;
	const struct sdhc_stm32_config *config = dev->config;
	__ASSERT(data != NULL, "Data is required for this command.");

	if(!IS_ENABLED(CONFIG_SDHC_STM32_POLLING_MODE)){
		ret = HAL_SD_WriteBlocks_DMA(config->hsd, data->data, data->block_addr, data->blocks);
	} else {
		ret = HAL_SD_WriteBlocks(config->hsd, data->data, data->block_addr, data->blocks, data->timeout_ms);
	}

	if(!IS_ENABLED(CONFIG_SDHC_STM32_POLLING_MODE)){
		if (k_sem_take(&dev_data->cmd_sem, SDHC_CMD_TIMEOUT) != 0) {
			LOG_ERR("Failed to acquire Semaphore\n");
			return -ETIMEDOUT;
		}
	}

	return ret;
}

static int sdhc_stm32_read_blocks(const struct device *dev, struct sdhc_data *data){
	int ret;
	struct sdhc_stm32_data *dev_data = dev->data;
	const struct sdhc_stm32_config *config = dev->config;

	if(!IS_ENABLED(CONFIG_SDHC_STM32_POLLING_MODE)){
		ret = HAL_SD_ReadBlocks_DMA(config->hsd, data->data, data->block_addr, data->blocks);
	} else {
		ret = HAL_SD_ReadBlocks(config->hsd, data->data, data->block_addr, data->blocks, data->timeout_ms);
	}

	if(!IS_ENABLED(CONFIG_SDHC_STM32_POLLING_MODE)){
		if (k_sem_take(&dev_data->cmd_sem, SDHC_CMD_TIMEOUT) != 0) {
			LOG_ERR("Failed to acquire Semaphore\n");
			return -ETIMEDOUT;
		}
	}
	return ret;
}

static int sdhc_stm32_switch_to_1_8v(const struct device *dev){
	uint32_t res =0;
	struct sdhc_stm32_data *data = dev->data;
	const struct sdhc_stm32_config *config = dev->config;

	if(!data->props.host_caps.vol_180_support){
		LOG_ERR("Host does not support 1.8v signaling");
		return -ENOTSUP;
	}
	//config->hsd->SdCard.CardSpeed = CARD_ULTRA_HIGH_SPEED;

	/* Start switching procedue */
	config->hsd->Instance->POWER |= SDMMC_POWER_VSWITCHEN;

	res = SDMMC_CmdVoltageSwitch(config->hsd->Instance);
	if(res != 0) {
		LOG_ERR("CMD11 failed: %#x", res);
		return -EIO; 
	}

	LOG_INF("Successfully switched to 1.8V signaling");
	return 0;
}

static uint32_t sdhc_stm32_go_idle_state(const struct device *dev){
	uint32_t res;
	struct sdhc_stm32_data *dev_data = dev->data;
	const struct sdhc_stm32_config *config = dev->config;

	res = SDMMC_CmdGoIdleState(config->hsd->Instance);
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

static uint32_t sdhc_stm32_get_sd_status(SD_HandleTypeDef *hsd, uint32_t card_relative_address, uint32_t *card_status_resp)
{
	uint32_t res;

	/* Send Status command */
	res = SDMMC_CmdSendStatus(hsd->Instance, (uint32_t)(card_relative_address));
	if (res != HAL_SD_ERROR_NONE)
	{
		LOG_ERR("Get Card status failed\n");
		return res;
	}

	/* Get SD card status */
	*card_status_resp = SDMMC_GetResponse(hsd->Instance, SDMMC_RESP1);

	return 0;
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

	(void)pm_device_runtime_get(dev);
	/* Prevent the clocks to be stopped during the request */
	pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);

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
			res = SDMMC_CmdSetRelAdd(config->hsd->Instance, (uint16_t *)&cmd->response);
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
			res = SD_SwitchSpeed(config->hsd, cmd->arg);
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
			res = SD_FindSCR(config->hsd, data->data);
			break;
		case SD_SET_BLOCK_SIZE:
			res = SDMMC_CmdBlockLength(config->hsd->Instance, (uint32_t)cmd->arg);
			break;
		case SD_VOL_SWITCH:
		    res = sdhc_stm32_switch_to_1_8v(dev);
			break;

		case SD_SEND_STATUS:
			res = sdhc_stm32_get_sd_status(config->hsd, cmd->arg, &cmd->response[0]);
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

	pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
	(void)pm_device_runtime_put(dev);
	k_mutex_unlock(&dev_data->bus_mutex);

	return res;
}

static int sdhc_stm32_get_card_present(const struct device *dev)
{
    struct sdhc_stm32_data *dev_data = dev->data;

    const struct sdhc_stm32_config *config = dev->config;

    int res = 0;

    k_mutex_lock(&dev_data->bus_mutex, K_FOREVER);
    (void)pm_device_runtime_get(dev);
    pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);

    if (config->cd_gpio.port != NULL) {
        if (!device_is_ready(config->cd_gpio.port)) {
            LOG_ERR("Card detect GPIO not ready");
            res = -ENODEV;
            goto out;
        }
        res = gpio_pin_get_dt(&config->cd_gpio);
        LOG_INF("from gpio %d ",res);
        
    } else {
	    // Try CMD0 or CMD55/ACMD41 (send operation condition)
	    if (SDMMC_CmdGoIdleState(config->hsd->Instance) != 0) {
		    LOG_ERR("Card can't go to Idle State");
		    res = 1;
	    } else {
		    if (SDMMC_CmdOperCond(config->hsd->Instance) != 0) {
			    LOG_ERR("Card not responding to OC");
			    res = 0; // Not present
		    } else {
			    res = 1; // Present
		    }
	    }
    }

out:

    pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
    (void)pm_device_runtime_put(dev);
    k_mutex_unlock(&dev_data->bus_mutex);

    return res ;
}

static int sdhc_stm32_reset(const struct device *dev)
{
	int res = 0;
	struct sdhc_stm32_data *data = dev->data;
	const struct sdhc_stm32_config *config = dev->config;

	k_mutex_lock(&data->bus_mutex, K_FOREVER);
	(void)pm_device_runtime_get(dev);
	/* Prevent the clocks to be stopped during the request */
	pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);

	/* Resetting Host controller */
	(void)SDMMC_PowerState_OFF(config->hsd->Instance);
	k_msleep(data->props.power_delay);
	(void)SDMMC_PowerState_ON(config->hsd->Instance);
	k_msleep(data->props.power_delay);

	/* Resetting card */
	res = sdhc_stm32_go_idle_state(dev);

	pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
	(void)pm_device_runtime_put(dev);
	k_mutex_unlock(&data->bus_mutex);

	return res;
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

	k_mutex_lock(&data->bus_mutex, K_FOREVER);
	(void)pm_device_runtime_get(dev);
	/* Prevent the clocks to be stopped during the request */
	pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);

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
	pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
	(void)pm_device_runtime_put(dev);
	k_mutex_unlock(&data->bus_mutex);

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
	props->host_caps.vol_180_support = sdhc_config->support_1_8_v;
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

	if (config->cd_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&config->cd_gpio)) {
			LOG_ERR("GPIO port for carrier-detect pin is not ready");
			return -ENODEV;
		}
		int ret = gpio_pin_configure_dt(&config->cd_gpio, GPIO_INPUT| GPIO_PULL_UP);
		if (ret < 0) {
			LOG_ERR("Couldn't configure carrier-detect pin; (%d)", ret);
			return ret;
		}
	}

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

#ifdef CONFIG_PM_DEVICE
static int sdhc_stm32_suspend(const struct device *dev)
{
	int ret;
	const struct sdhc_stm32_config *cfg = (struct sdhc_stm32_config *)dev->config;
	const struct device *const clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);

	/* Disable device clock. */
	ret = clock_control_off(clk, (clock_control_subsys_t)(uintptr_t)&cfg->pclken[0]);
	if (ret < 0) {
		LOG_ERR("Failed to disable SDHC clock during PM suspend process");
		return ret;
	}

	/* Move pins to sleep state */
	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_SLEEP);
	if (ret == -ENOENT) {
		/* Warn but don't block suspend */
		LOG_WRN_ONCE("SDHC pinctrl sleep state not available");
		ret = 0;
	}

	return ret;
}

static int sdhc_stm32_pm_action(const struct device *dev, enum pm_device_action action)
{
	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		return sdhc_stm32_activate(dev);
	case PM_DEVICE_ACTION_SUSPEND:
		return sdhc_stm32_suspend(dev);
	default:
		return -ENOTSUP;
	}
}
#endif /* CONFIG_PM_DEVICE */

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
		.support_1_8_v = DT_INST_PROP(index, support_1_8_v),	\
		.min_freq = DT_INST_PROP(index, min_bus_freq),	\
		.max_freq = DT_INST_PROP(index, max_bus_freq),	\
		.cd_gpio = GPIO_DT_SPEC_GET_OR(DT_DRV_INST(index), cd_gpios, {0}),\
	};	\
	\
	static struct sdhc_stm32_data sdhc_stm32_data_##index = {	\
		.host_io = {	\
			.bus_width= DT_INST_PROP_OR(index, bus_width, 4),	\
		}	\
	};	\
	\
	PM_DEVICE_DT_INST_DEFINE(index, sdhc_stm32_pm_action);	\
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
