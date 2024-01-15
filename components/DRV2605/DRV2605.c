#include "DRV2605.h"

esp_err_t DRV2605_Init(void)
{
    //gpio init
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << DRV2605_EN_PIN);
    io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    vTaskDelay(1 / portTICK_PERIOD_MS);
    gpio_set_level(DRV2605_EN_PIN, 1);
    
    i2c_write_bit(I2C_MASTER_NUM, DRV2605_ADDR, DRV2605_REG_Mode, DRV2605_REG_Mode_Bit_Reset, 1);
    vTaskDelay(1 / portTICK_PERIOD_MS);
    i2c_write_byte(I2C_MASTER_NUM, DRV2605_ADDR, DRV2605_REG_Mode, 0x07);


    i2c_write_byte(I2C_MASTER_NUM,DRV2605_ADDR,DRV2605_REG_FeedbackCtrl,0xAA);

    i2c_write_byte(I2C_MASTER_NUM,DRV2605_ADDR,DRV2605_REG_RatedVoltage,0x50);
    i2c_write_byte(I2C_MASTER_NUM,DRV2605_ADDR,DRV2605_REG_ODClamp,0x32);

    i2c_write_bit(I2C_MASTER_NUM,DRV2605_ADDR,0x1E,5,1);
    i2c_write_bit(I2C_MASTER_NUM,DRV2605_ADDR,0x1E,4,1);

    i2c_write_bit(I2C_MASTER_NUM,DRV2605_ADDR,DRV2605_REG_Go,DRV2605_REG_Go_Bit_Go,1);
    ESP_LOGI(TAG_DRV2605,"DRV2605 Initializing,Auto Calibration Onward\n");
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    int result=0;
    i2c_read_bit(I2C_MASTER_NUM,DRV2605_ADDR,DRV2605_REG_Status,DRV2605_REG_Status_Bit_DiagnosticsResult,&result);
    if(result!=0)
    {
        ESP_LOGE(TAG_DRV2605,"DRV2605 Auto Cali Failed!!");
        return 0;
    }
    else
    {
        ESP_LOGI(TAG_DRV2605,"DRV2605 Auto Cali Passed!!");
    }

    i2c_write_byte(I2C_MASTER_NUM, DRV2605_ADDR, DRV2605_REG_Mode, DRV2605_REG_Mode_Bit_Mode_Int_trig);
    i2c_write_bit(I2C_MASTER_NUM,DRV2605_ADDR,DRV2605_REG_LibSel,0,0);
    i2c_write_bit(I2C_MASTER_NUM,DRV2605_ADDR,DRV2605_REG_LibSel,1,1);
    i2c_write_bit(I2C_MASTER_NUM,DRV2605_ADDR,DRV2605_REG_LibSel,2,1);//libsel
    gpio_set_level(DRV2605_EN_PIN, 0);
     return ESP_OK;
    // i2c_write_byte(I2C_MASTER_NUM,DRV2605_ADDR,0x1E,)
    // i2c_write_bits(I2C_MASTER_NUM,DRV2605_ADDR,0x1B,0,0);
    // i2c_write_bits(I2C_MASTER_NUM,DRV2605_ADDR,0x1C,2,3);//Sample time:300us
    // i2c_write_bits(I2C_MASTER_NUM,DRV2605_ADDR,0x1C,2,3);
}

// esp_err_t DRV2605_DISABLE(void)
// {
//     gpio_set_level(DRV2605_EN_PIN, 0);
// }

// esp_err_t DRV2605_ENABLE(void)
// {
//     gpio_set_level(DRV2605_EN_PIN, 1);
// }

esp_err_t DRV2605_Play(int Wave)
{
    gpio_set_level(DRV2605_EN_PIN, 1);
    vTaskDelay(1 / portTICK_PERIOD_MS);
    i2c_write_byte(I2C_MASTER_NUM, DRV2605_ADDR, DRV2605_REG_Mode, DRV2605_REG_Mode_Bit_Mode_Int_trig);
    
    i2c_write_byte(I2C_MASTER_NUM,DRV2605_ADDR,DRV2605_REG_WaveformSeq_1,Wave);
    i2c_write_bit(I2C_MASTER_NUM,DRV2605_ADDR,DRV2605_REG_Go,DRV2605_REG_Go_Bit_Go,1);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    gpio_set_level(DRV2605_EN_PIN, 0);
     return ESP_OK;
}