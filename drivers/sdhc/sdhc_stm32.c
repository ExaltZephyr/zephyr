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

LOG_MODULE_REGISTER(sdhc_stm32, CONFIG_SDHC_LOG_LEVEL);
typedef void (*irq_config_func_t)(const struct device *port);

#define DELAY_MS                    (10)
#define SD_TIMEOUT                  ((uint32_t)0x00100000U)
#define STM32_SDIO_F_MIN            (SDMMC_CLOCK_400KHZ)
#define STM32_SDIO_F_MAX            (MHZ(208))

struct sdhc_stm32_config {
  bool                              hw_flow_control;
  uint8_t                           bus_width;
  uint16_t                          clk_div;
  uint32_t                          power_delay_ms;
	const struct stm32_pclken        *pclken;
	const struct pinctrl_dev_config  *pcfg;
  irq_config_func_t                 irq_config_func;
};

struct sdhc_stm32_data {
  struct k_mutex       bus_mutex; 
  SDIO_HandleTypeDef   hsd;
  struct sdhc_io       host_io;
  uint32_t             cmd_index;
  struct k_sem         cmd_sem;
};

static HAL_StatusTypeDef SDIO_NoOpIdentifyCard(SD_HandleTypeDef *hsd)
{
    return HAL_OK;
}

static void i3c_stm32_event_isr(void *arg)
{
    const struct device     *dev = (const struct device *)arg;
    struct sdhc_stm32_data  *data = dev->data;

    if(__HAL_SDIO_GET_FLAG(&data->hsd, SDMMC_FLAG_DATAEND)) {
        k_sem_give(&data->cmd_sem);  
    }

    if (__HAL_SDIO_GET_FLAG(&data->hsd, SDMMC_FLAG_DCRCFAIL | SDMMC_FLAG_DTIMEOUT | SDMMC_FLAG_RXOVERR | SDMMC_FLAG_TXUNDERR) || (data->hsd.ErrorCode != HAL_SDIO_ERROR_NONE))
    {
        k_sem_give(&data->cmd_sem);
    }

    HAL_SDIO_IRQHandler(&data->hsd);
}

void sdhc_stm32_log_err_type(SDIO_HandleTypeDef *hsd)
{
    uint32_t error_code = HAL_SDIO_GetError(hsd);

    if (error_code & HAL_SDIO_ERROR_TIMEOUT) {
        LOG_ERR("SDIO Timeout Error\n");
    }

    if (error_code & HAL_SDIO_ERROR_DATA_TIMEOUT) {
        LOG_ERR("SDIO Data Timeout Error\n");
    }

    if (error_code & HAL_SDIO_ERROR_DATA_CRC_FAIL) {
        LOG_ERR("SDIO Data CRC Error\n");
    }

    if (error_code & HAL_SDIO_ERROR_TX_UNDERRUN) {
        LOG_ERR("SDIO FIFO Transmit Underrun Error\n");
    }

    if (error_code & HAL_SDIO_ERROR_RX_OVERRUN) {
        LOG_ERR("SDIO FIFO Receive Overrun Error\n");
    }

    // Invalid callback error
    if (error_code & HAL_SDIO_ERROR_INVALID_CALLBACK) {
        LOG_ERR("SDIO Invalid Callback Error\n");
    }

    hsd->ErrorCode = HAL_SDIO_ERROR_NONE;
}

int sdhc_stm32_sd_init(const struct device *dev)
{
  struct sdhc_stm32_data *data = dev->data;
  const struct sdhc_stm32_config *config = dev->config;
  SDIO_HandleTypeDef *hsd = &data->hsd;

  if (HAL_SDIO_DeInit(hsd) != HAL_OK) {
    LOG_ERR("Failed to de-initialize the SDIO device\n");
    return HAL_ERROR;
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

  return HAL_SDIO_Init(hsd);
}

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

	return ret;
}
static uint32_t sdhc_stm32_go_idle_state(const struct device *dev){
    uint32_t res;
    struct sdhc_stm32_data *data = dev->data;

    res = SDMMC_CmdGoIdleState(data->hsd.Instance);
    if (res != HAL_OK) {
        sdhc_stm32_log_err_type(&data->hsd);
        return res;
    }

    return res;
}

static int sdhc_stm32_request(const struct device *dev, struct sdhc_command *cmd, struct sdhc_data *data)
{
   int res;
   struct sdhc_stm32_data *dev_data = dev->data;

   __ASSERT(cmd != NULL, "Command is NULL.");

  if( !WAIT_FOR(HAL_SDIO_GetState(&dev_data->hsd) == HAL_SDIO_STATE_READY, SD_TIMEOUT, k_usleep(1))) {
    LOG_ERR("SD Card is busy now");
    return HAL_ERROR;
  }

  k_mutex_lock(&dev_data->bus_mutex, K_FOREVER);

  dev_data->cmd_index = cmd->opcode;
  switch (cmd->opcode) {
      case SD_GO_IDLE_STATE:
        res = sdhc_stm32_go_idle_state(dev);
        break;

      case SD_SELECT_CARD:
        res = SDMMC_CmdSelDesel(dev_data->hsd.Instance, cmd->arg );
        break;

      case SD_SEND_RELATIVE_ADDR:
        res = SDMMC_CmdSetRelAdd(dev_data->hsd.Instance, &cmd->response[0U]);
        break;

      case SDIO_SEND_OP_COND:
        res = SDMMC_CmdSendOperationcondition(dev_data->hsd.Instance, cmd->arg, &cmd->response[0]);
        break;

      case SDIO_RW_DIRECT:
        return res;
      break;

      default:
        res = -ENOTSUP;
        LOG_DBG("Unsupported Command, opcode:%d\n.",cmd->opcode);
        break;
  }

  k_mutex_unlock(&dev_data->bus_mutex);
  if(res != HAL_OK) {
    LOG_ERR("Command Failed, opcode:%d\n.",cmd->opcode);
    sdhc_stm32_log_err_type(&dev_data->hsd);
  }

    return res;
}

static int sdhc_stm32_set_io(const struct device *dev, struct sdhc_io *ios)
{
	struct sdhc_stm32_data *data = dev->data;
	struct sdhc_io *host_io = &data->host_io;
  int res = 0;

  if ((ios->clock != 0) && ( host_io->clock !=ios->clock) )
  {
      if ((ios->clock > STM32_SDIO_F_MAX) || (ios->clock < STM32_SDIO_F_MIN)) {
        LOG_ERR("Invalid Clock Frequency\n");
        return -EINVAL;
      }
      res = HAL_SDIO_ConfigFrequency(&data->hsd, (uint32_t) ios->clock);
      if(res != HAL_OK) {
        LOG_ERR("Failed to set clock as %d\n", ios->clock);
      } else {
        host_io->clock = ios->clock;
        LOG_INF("set clock as %d passed\n", ios->clock);
      }
      k_msleep(DELAY_MS);
  }

   if( (ios->signal_voltage == SD_VOL_1_8_V )) {
    res = SDMMC_CmdVoltageSwitch(data->hsd.Instance);
    if(res != HAL_SDIO_ERROR_NONE) {
      LOG_ERR("Failed to set signal voltage as %d\n", ios->signal_voltage);
    } else {
      host_io->signal_voltage = ios->signal_voltage;
      LOG_INF("set signal voltage as %d passed\n", ios->signal_voltage);
    }
  }

  if (ios->power_mode == SDHC_POWER_OFF) {
    (void)SDMMC_PowerState_OFF(data->hsd.Instance);
    k_msleep(DELAY_MS);
  } else if (ios->power_mode == SDHC_POWER_ON) {
    (void)SDMMC_PowerState_ON(data->hsd.Instance);
    k_msleep(DELAY_MS);
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
  props->host_caps.vol_300_support = false;
	props->host_caps.bus_8_bit_support = true;
  props->host_caps.bus_4_bit_support = true;
  props->host_caps.hs200_support = false;
  props->host_caps.hs400_support = false;
	props->host_caps.high_spd_support = true;
	props->host_caps.sdr50_support = true;
	props->host_caps.sdio_async_interrupt_support = true;
	props->is_spi = false;

	return HAL_OK;
}

static const struct sdhc_driver_api sdhc_stm32_api = {
	.request = sdhc_stm32_request,
  .set_io = sdhc_stm32_set_io,
  .get_host_props = sdhc_stm32_get_host_props,
};

static int sdhc_stm32_init(const struct device *dev)
{
  int ret;
  struct sdhc_stm32_data           *data = dev->data;
  const struct sdhc_stm32_config   *config = dev->config;

  config->irq_config_func(dev);

  HAL_SDIO_RegisterIdentifyCardCallback(&data->hsd, SDIO_NoOpIdentifyCard);

  ret = sdhc_stm32_activate(dev);
	if (ret != 0) {
		LOG_ERR("Clock and GPIO could not be initialized for the SDHC module, err=%d", ret);
		return ret;
	}

  ret = sdhc_stm32_sd_init(dev);
  if (ret != 0) {
    LOG_ERR("SDIO Init Failed\n");
    sdhc_stm32_log_err_type(&data->hsd);
    return ret;
  } else {
    LOG_INF("SDIO Init Passed Successfully.\n");
  }

	return ret;
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
    .bus_width= DT_INST_PROP(index, bus_width),    \
    .power_delay_ms = DT_INST_PROP_OR(inst, power_delay_ms, 500),	\
	};                                                                                         \
	static struct sdhc_stm32_data sdhc_stm32_data_##index = {                             \
    .hsd = {   \
      .Instance = (MMC_TypeDef *)DT_INST_REG_ADDR(index),   \
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
