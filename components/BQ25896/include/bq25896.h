#ifndef __BQ25896_H__
#define __BQ25896_H__

#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "i2c-easy.h"

void BQ25896_Init();
void BQ25896_Get_Status_Task(void *arg);
void BQ25896_OTG_en(int en);
#endif
