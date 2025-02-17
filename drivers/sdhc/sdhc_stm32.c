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
  uint16_t                          clk_div;
  bool                              hw_flow_control;
  uint32_t                          power_delay_ms;
	const struct stm32_pclken        *pclken;              /* Pointer to peripheral clock configuration */
	const struct pinctrl_dev_config  *pcfg;               /* Pointer to pin control configuration */
  irq_config_func_t                 irq_config_func;    /* IRQ config function */
};

struct sdhc_stm32_data {
  struct k_mutex     bus_mutex; 
  SD_HandleTypeDef   hsd;
  struct sdhc_io     host_io;
  struct k_sem       cmd_sem;
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

    __ASSERT(data != NULL, "Data is required for this command.");

    if(HAL_SD_Erase(&dev_data->hsd, data->block_addr, (data->block_addr+sizeof(data->data))) != HAL_OK)
    {
      return HAL_ERROR;
    }

    if(!WAIT_FOR(HAL_SD_GetCardState(&dev_data->hsd) == HAL_SD_CARD_TRANSFER, SD_TIMEOUT, k_usleep(1))) {
      LOG_ERR("SD Card is busy now");
      return HAL_ERROR;
    }

    #ifdef CONFIG_SDHC_STM32_DMA
      ret = HAL_SD_WriteBlocks_DMA(&dev_data->hsd, data->data, data->block_addr, data->blocks);
    #else
      ret = HAL_SD_WriteBlocks_IT(&dev_data->hsd, data->data, data->block_addr, data->blocks);
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

    #ifdef CONFIG_SDHC_STM32_DMA
        ret = HAL_SD_ReadBlocks_DMA(&dev_data->hsd, data->data, data->block_addr, data->blocks);
    #else
        ret = HAL_SD_ReadBlocks_IT(&dev_data->hsd, data->data, data->block_addr, data->blocks);
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

    res = SDMMC_CmdGoIdleState(SDMMC1);
    if (res != HAL_OK) {
        sdhc_stm32_log_err_type(&dev_data->hsd);
        return res;
    }

    res = HAL_SD_Init(&dev_data->hsd);
    if (res != HAL_OK)
    {
        sdhc_stm32_log_err_type(&dev_data->hsd);
        return res;
    }

     if(!WAIT_FOR(HAL_SD_GetCardState(&dev_data->hsd) == HAL_SD_CARD_TRANSFER, SD_TIMEOUT, k_usleep(1))) {
        LOG_ERR("SD Card is busy now");
        return HAL_ERROR;
     }

    return 0;
}

static int sdhc_stm32_abort(const struct device *dev){
    int res;
    struct sdhc_stm32_data *data = dev->data;
    res = HAL_SD_Abort_IT(&data->hsd);
    return res;
}

static int sdhc_stm32_erase_block(const struct device *dev, struct sdhc_data *data) {
    int res;
    struct sdhc_stm32_data *dev_data = dev->data;

    res = HAL_SD_Erase(&dev_data->hsd, data->block_addr, (data->block_size + data->block_addr));
    if(res != HAL_OK) {
      return res;
    }

    if( !WAIT_FOR(HAL_SD_GetCardState(&dev_data->hsd) == HAL_SD_CARD_TRANSFER, SD_TIMEOUT, k_usleep(1))) {
      LOG_ERR("SD Card is busy now");
      return HAL_ERROR;
    }
    return res;
}

static int sdhc_stm32_request(const struct device *dev, struct sdhc_command *cmd, struct sdhc_data *data)
{
   int res;
   struct sdhc_stm32_data *dev_data = dev->data;

   __ASSERT(cmd != NULL, "Command is NULL.");

     if( !WAIT_FOR(HAL_SD_GetCardState(&dev_data->hsd) == HAL_SD_CARD_TRANSFER, SD_TIMEOUT, k_usleep(1))) {
        LOG_ERR("SD Card is busy now");
        return HAL_ERROR;
     }

    k_mutex_lock(&dev_data->bus_mutex, K_FOREVER);

		switch (cmd->opcode) {
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
      case SD_STOP_TRANSMISSION:
        res = sdhc_stm32_abort(dev);
        break;
      default:
        res = HAL_ERROR;
        LOG_ERR("Unsupported Command.");
        break;
		}

    k_mutex_unlock(&dev_data->bus_mutex);
    if(res != HAL_OK) {
      sdhc_stm32_log_err_type(&dev_data->hsd);
    }

    return res;
  }

static int sdhc_stm32_get_card_present(const struct device *dev)
{
    struct sdhc_stm32_data *dev_data = dev->data;

    HAL_SD_CardStateTypeDef card_state = HAL_SD_GetCardState(&dev_data->hsd);
    if((HAL_SD_GetError(&dev_data->hsd) == SDMMC_ERROR_CMD_RSP_TIMEOUT) || (card_state == HAL_SD_CARD_DISCONNECTED)) {
      sdhc_stm32_log_err_type(&dev_data->hsd);
      return false;
    }

    return true;
}

static int sdhc_stm32_reset(const struct device *dev)
{
	return sdhc_stm32_go_idle_state(dev);
}

static int sdhc_stm32_card_busy(const struct device *dev)
{
     struct sdhc_stm32_data *data = dev->data;
    if(HAL_SD_GetState(&data->hsd) == HAL_SD_STATE_BUSY) {
      return true;
    }
    return false;

}

static int sdhc_stm32_set_io(const struct device *dev, struct sdhc_io *ios)
{
	struct sdhc_stm32_data *data = dev->data;
	struct sdhc_io *host_io = &data->host_io;
  int res = 0;

	if ((ios->bus_width != 0) && (host_io->bus_width != ios->bus_width)) {
    res = HAL_SD_ConfigWideBusOperation(&data->hsd, (uint32_t) ios->bus_width);
    //res =  HAL_SDIO_SetDataBusWidth(&data->hsd, (uint32_t) ios->bus_width);
    if(res != HAL_OK) {
      LOG_ERR("Failed to set data bus width as %d\n", ios->bus_width);
    } else {
      host_io->bus_width = ios->bus_width;
    }
	}

  if ((ios->clock != 0) && ( host_io->clock !=ios->clock) )
  {
      res = HAL_SD_ConfigSpeedBusOperation(&data->hsd, (uint32_t) ios->clock);
      //res = HAL_SDIO_ConfigFrequency(&data->hsd, (uint32_t) ios->clock);
      if(res != HAL_OK) {
        LOG_ERR("Failed to set clock as %d\n", ios->clock);
      } else {
        host_io->clock = ios->clock;
      }
  }

  if ((ios->signal_voltage !=0 ) && (host_io->signal_voltage != ios->signal_voltage) &&  (ios->signal_voltage == SD_VOL_1_8_V ) ) {
        res = SDMMC_CmdVoltageSwitch(data->hsd.Instance);

        if(res != HAL_SD_ERROR_NONE) {
          LOG_ERR("Failed to set signal voltage as %d\n", ios->signal_voltage);
        } else {
          host_io->signal_voltage = ios->signal_voltage;
        }
  }

  return res;
}

static int sdhc_stm32_get_host_props(const struct device *dev, struct sdhc_host_props *props)
{
	const struct sdhc_stm32_config *sdhc_config = (struct sdhc_stm32_config *)dev->config;
	memset(props, 0, sizeof(struct sdhc_host_props));

	props->f_min = SD_CLOCK_25MHZ;
	props->f_max = SD_CLOCK_208MHZ;
	props->power_delay = sdhc_config->power_delay_ms;
	props->host_caps.vol_330_support = true;

	props->host_caps.vol_180_support = true;
	props->host_caps.bus_8_bit_support = true;
  props->host_caps.bus_4_bit_support = true;
  props->host_caps.hs200_support = false;
  props->host_caps.hs400_support = false;
	props->is_spi = false;

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
    if (clock_control_configure(clk,
            (clock_control_subsys_t)&config->pclken[1],
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

void HAL_SD_AbortCallback(SD_HandleTypeDef *hsd) {
  LOG_INF("AbortCall");
}

int sdhc_stm32_sd_init(const struct device *dev)
{
  struct sdhc_stm32_data *data = dev->data;
  const struct sdhc_stm32_config *config = dev->config;
  SD_HandleTypeDef *hsd = &data->hsd;

  HAL_SD_DeInit(hsd);

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
  // int res = HAL_SD_ConfigSpeedBusOperation(&data->hsd, data->host_io.clock);
  // if(res != HAL_OK) {
  //   LOG_ERR("Configure Speed Failed.");
  // }
  return HAL_SD_Init(hsd);

}

static int sdhc_stm32_init(const struct device *dev)
{
  int ret;
  struct sdhc_stm32_data     *data = dev->data;
  const struct sdhc_stm32_config   *config = dev->config;

	sdhc_stm32_activate(dev);

  ret = sdhc_stm32_sd_init(dev);
  if (ret != 0) {
    sdhc_stm32_log_err_type(&data->hsd);
    return ret;
  }

  k_mutex_init(&data->bus_mutex);
  k_sem_init(&data->cmd_sem, 0, K_SEM_MAX_LIMIT);

  config->irq_config_func(dev);

	return 0;
}

static void i3c_stm32_event_isr(void *arg)
{
    const struct device     *dev = (const struct device *)arg;
    struct sdhc_stm32_data  *data = dev->data;

    if(__HAL_SD_GET_FLAG(&data->hsd, SDMMC_FLAG_DATAEND)) {
        k_sem_give(&data->cmd_sem);
    }

    if (__HAL_SD_GET_FLAG(&data->hsd, SDMMC_FLAG_DCRCFAIL | SDMMC_FLAG_DTIMEOUT | SDMMC_FLAG_RXOVERR | SDMMC_FLAG_TXUNDERR) || (data->hsd.ErrorCode != HAL_SD_ERROR_NONE))
    {
        k_sem_give(&data->cmd_sem);
    }

    HAL_SD_IRQHandler(&data->hsd);
}

#define STM32_SDHC_IRQ_CONNECT_AND_ENABLE(index)                                                    \
	do {                                                                                       \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(index, event, irq),                                \
			    DT_INST_IRQ_BY_NAME(index, event, priority), i3c_stm32_event_isr,      \
			    DEVICE_DT_INST_GET(index), 0);                                         \
		irq_enable(DT_INST_IRQ_BY_NAME(index, event, irq));                                \
	} while (false)

#define STM32_SDHC_IRQ_HANDLER_DECL(index)                                                          \
	static void sdhc_stm32_irq_config_func_##index(const struct device *dev)

#define STM32_SDHC_IRQ_HANDLER_FUNCTION(index) .irq_config_func = sdhc_stm32_irq_config_func_##index,

#define STM32_SDHC_IRQ_HANDLER(index)                                                               \
	static void sdhc_stm32_irq_config_func_##index(const struct device *dev)                    \
	{                                                                                          \
		STM32_SDHC_IRQ_CONNECT_AND_ENABLE(index);                                           \
	}

#define SDHC_STM32_INIT(index)						\
	STM32_SDHC_IRQ_HANDLER_DECL(index);  \
	static const struct stm32_pclken pclken_##index[] = STM32_DT_INST_CLOCKS(index);           \
	PINCTRL_DT_INST_DEFINE(index);                                                             \
	static const struct sdhc_stm32_config sdhc_stm32_cfg_##index = {                             \
		 STM32_SDHC_IRQ_HANDLER_FUNCTION(index).pclken = pclken_##index,                     \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),                                     \
    .hw_flow_control= DT_INST_PROP_OR(index, hw_flow_control,0),	\
    .clk_div= DT_INST_PROP_OR(index, clk_div,4),	\
    .power_delay_ms = DT_INST_PROP_OR(inst, power_delay_ms, 1),	\
	};                                                                                         \
	static struct sdhc_stm32_data sdhc_stm32_data_##index = {                             \
    .hsd = {   \
      .Instance = (MMC_TypeDef *)DT_INST_REG_ADDR(index),   \
    },   \
    .host_io = {   \
      .bus_width= DT_INST_PROP_OR(index, bus_width, 4),    \
      .clock= DT_INST_PROP_OR(index, clock_frequency, MHZ(12.5) ),    \
    }   \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(index,					\
			&sdhc_stm32_init,				\
			NULL,						\
			&sdhc_stm32_data_##index,						\
			&sdhc_stm32_cfg_##index,						\
			POST_KERNEL,					\
			CONFIG_SDHC_INIT_PRIORITY,			\
			&sdhc_stm32_api); \
  STM32_SDHC_IRQ_HANDLER(index)

DT_INST_FOREACH_STATUS_OKAY(SDHC_STM32_INIT)
