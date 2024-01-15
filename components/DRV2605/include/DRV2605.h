#ifndef __DRV2605_H
#define __DRV2605_H

#include "DRV2605_REG.h"
#include "i2c-easy.h"
#include "freertos/FreeRTOS.h"
#include "driver/i2c.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_task_wdt.h"

#define DRV2605_ADDR 0x5A//0x5A=0b1011010,7bit
#define DRV2605_EN_PIN 35
#define I2C_MASTER_NUM I2C_NUM_0

#define TAG_DRV2605 "DRV2605"
esp_err_t DRV2605_Init(void);
esp_err_t DRV2605_Play(int Wave);

#endif //__DRV2605_H