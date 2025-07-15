#include <string.h>
#include <stdio.h>
#include <assert.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sdhc.h>
#include <zephyr/sd/sd.h>


LOG_MODULE_REGISTER(sd_sample, LOG_LEVEL_INF);

#define SD_TIMEOUT ((uint32_t)0x00100000U)

int sd_card_erase(const struct device *dev){

    uint32_t erase_address= (uint32_t)0x00000400U;

    struct sdhc_command cmd = {
        .opcode = SD_ERASE_BLOCK_OPERATION,
        .arg = 0,  // optional, depends on your implementation
    };
    

    struct sdhc_data data = {
        .block_addr = erase_address,
        .block_size = 512,
        .blocks = 1,
        .timeout_ms = 1000,
    };

    int ret = sdhc_request(dev,&cmd,&data);
    return ret;
}


int main(void)
{
	const struct device *sdmmc_dev = DEVICE_DT_GET(DT_NODELABEL(sdmmc1));

	if (!device_is_ready(sdmmc_dev)) {
		LOG_ERR("SDHC %s is not ready", sdmmc_dev->name);
		return -1;
	}

    int ret =0;
    while (true)
    {
        /* code */
        ret = sdhc_card_present(sdmmc_dev);
        if (!ret)
        {
            LOG_ERR("card not present");
        }else {
            LOG_INF("card is present");
        }
        k_sleep(K_SECONDS(1));
    }
    
    //ret =sd_card_erase(sdmmc_dev);
    // if(ret == 0){
    //     LOG_INF("Erase Successful");
    // } else {
    //     LOG_ERR("Erase failed with error: %d", ret);
    // }

	static struct sd_card card;
	if (sd_init(sdmmc_dev, &card)) {
		LOG_ERR("###failed in sd init");
		return -1;
	}
    else {
        LOG_INF("sd init passed");
    }
}
