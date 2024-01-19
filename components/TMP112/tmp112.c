#include "tmp112.h"
TMP112_Config_t TMP112_Config_old;
void TMP112_Init(TMP112_Config_t TMP112_Config)
{
    uint16_t data[1] = {0};
    data[0] = (TMP112_Config.FaultQueue << FaultQueue_mask) | (TMP112_Config.Polarity << Polarity_mask) | (TMP112_Config.Thermostat << Thermostat_mask) | (TMP112_Config.ShutdownEn << ShutdownEn_mask) | (TMP112_Config.ConvertPeriod << ConvertPeriod_mask) | (TMP112_Config.Resolution << Resolution_mask);
    ESP_ERROR_CHECK(i2c_write_bytes(I2C_MASTER_NUM, TMP112_ADDR, TMP112_PTR_REG_Configuration, data[0], 2));
    data[0] = (int)(HighThreshold / 0.0625);
    if (TMP112_Config.Resolution == Resolution_12bit)
        data[0] = data[0] << 4;
    else if (TMP112_Config.Resolution == Resolution_13bit)
        data[0] = data[0] << 3;
    ESP_ERROR_CHECK(i2c_write_bytes(I2C_MASTER_NUM, TMP112_ADDR, TMP112_PTR_REG_HighThreshold, data[0], 2));

    data[0] = (int)(LowThreshold / 0.0625);
    if (TMP112_Config.Resolution == Resolution_12bit)
        data[0] = data[0] << 4;
    else if (TMP112_Config.Resolution == Resolution_13bit)
        data[0] = data[0] << 3;
    ESP_ERROR_CHECK(i2c_write_bytes(I2C_MASTER_NUM, TMP112_ADDR, TMP112_PTR_REG_LowThreshold, data[0], 2));
    memcpy(&TMP112_Config_old, &TMP112_Config, sizeof(TMP112_Config_t));
}

float TMP112_Get_One_Shot_Value()
{
    uint16_t data = 0;
    int check = 0;
    int count = 0;
    ESP_ERROR_CHECK(i2c_write_bit(I2C_MASTER_NUM, TMP112_ADDR, TMP112_PTR_REG_Configuration, OneShot_mask, 1));
    while (check == 0)
    {
        if (++count > 10)
        {
            return 888888;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
        ESP_ERROR_CHECK(i2c_read_bit(I2C_MASTER_NUM, TMP112_ADDR, TMP112_PTR_REG_Configuration, 7, &check));
    }
    ESP_ERROR_CHECK(i2c_read_bytes(I2C_MASTER_NUM, TMP112_ADDR, TMP112_PTR_REG_Temperature, &data, 2));
    if (TMP112_Config_old.Resolution == Resolution_12bit)
    {
        data = data >> 4;
        if ((data >> 11) == 1)
        {
            data = ~data;
            data *= -1;
        }
        else
        {
            ;
        }
    }
    else if (TMP112_Config_old.Resolution == Resolution_13bit)
    {
        data = data >> 3;
        if ((data >> 12) == 1)
        {
            data = ~data;
            data *= -1;
        }
        else
        {
            ;
        }
    }

    return data * 0.0625;
}