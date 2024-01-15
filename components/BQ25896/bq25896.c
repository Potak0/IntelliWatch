#include "bq25896.h"
#include "i2c-easy.h"
#define BQ25896_ADDR 0x6B
#define I2C_MASTER_NUM 0
#define I2C_MASTER_SDA_IO 22
#define I2C_MASTER_SCL_IO 23
static const char *BQ25896_TAG = "bq25896";
uint16_t BAT_VBUS_Status = 0;//11,bit 7
uint16_t BAT_CHG_Status = 0;//0B,bit 4-3, 0:Not Charging, 1:Pre-charge, 2:Fast Charging, 3:Charge Termination Done
uint16_t BAT_ADC_Status = 0;//02,bit 7
uint16_t BAT_OTG_Fault = 0;//0C,bit 6
uint16_t BAT_CHG_Fault = 0;//0C,bit 5-4
uint16_t BAT_BAT_Fault = 0;//0C,bit 3

float BAT_VBUS_Val = 0;//11,bit 6-0, 2.6+0.1x
float BAT_VSYS_Val = 0;//0F,bit 6-0, 2.304+0.02x
float BAT_VBAT_Val = 0;//0E,bit 6-0, 2.304+0.02x
float BAT_IBAT_Val = 0;//12,bit 6-0, 0.05x
void BQ25896_Init()
{
    ESP_ERROR_CHECK(i2c_write_byte(I2C_MASTER_NUM, BQ25896_ADDR, 0x00, 0x3F));
    ESP_ERROR_CHECK(i2c_write_byte(I2C_MASTER_NUM, BQ25896_ADDR, 0x04, 0b00001000));

}

void BQ25896_Get_Status_Task(void *arg)
{
    while(1)
    {
        ESP_ERROR_CHECK(i2c_write_bit(I2C_MASTER_NUM, BQ25896_ADDR, 0x02, 7, 1));//Start Conversion
        vTaskDelay(1000 / portTICK_RATE_MS);
        ESP_ERROR_CHECK(i2c_read_bit(I2C_MASTER_NUM, BQ25896_ADDR, 0x02,7, (uint8_t *)&BAT_ADC_Status));
        if(BAT_ADC_Status == 1)
        {
            ESP_LOGW(BQ25896_TAG, "ADC Conversion Not Ready");
            vTaskDelay(100 / portTICK_RATE_MS);
            continue;
        }
        int buf3=0;
        ESP_ERROR_CHECK(i2c_read_byte(I2C_MASTER_NUM, BQ25896_ADDR, 0x11,(uint8_t *)&buf3));
        BAT_VBUS_Status = buf3 >> 7;
        BAT_VBUS_Val = buf3 & 0x7F;
        BAT_VBUS_Val = BAT_VBUS_Val * 0.1 + 2.6;

        int buf1=0;
        int buf2=0;
        ESP_ERROR_CHECK(i2c_read_bit(I2C_MASTER_NUM, BQ25896_ADDR, 0x0B,4,(uint8_t *)&buf1));
        ESP_ERROR_CHECK(i2c_read_bit(I2C_MASTER_NUM, BQ25896_ADDR, 0x0B,3,(uint8_t *)&buf2));
        BAT_CHG_Status = buf1 << 1 | buf2;

        ESP_ERROR_CHECK(i2c_read_byte(I2C_MASTER_NUM, BQ25896_ADDR, 0x0F,(uint8_t *)&BAT_VSYS_Val));
        BAT_VSYS_Val = BAT_VSYS_Val*0.02+2.304;

        int buf111=0;
        ESP_ERROR_CHECK(i2c_read_byte(I2C_MASTER_NUM, BQ25896_ADDR, 0x0E,(uint8_t *)&buf111));

        BAT_VBAT_Val = buf111 & 0x7F;
        BAT_VBAT_Val = BAT_VBAT_Val*0.02+2.304;

        ESP_ERROR_CHECK(i2c_read_byte(I2C_MASTER_NUM, BQ25896_ADDR, 0x12,(uint8_t *)&BAT_IBAT_Val));
        BAT_IBAT_Val = BAT_IBAT_Val *0.05;

        // ESP_LOGI(BQ25896_TAG, "VBUS Status: %d  ;", BAT_VBUS_Status);
        // ESP_LOGI(BQ25896_TAG, "VBUS Val: %.4f  ;", BAT_VBUS_Val);
        // ESP_LOGI(BQ25896_TAG, "CHG Status: %d  ;", BAT_CHG_Status);
        // ESP_LOGI(BQ25896_TAG, "VBAT Val: %.4f  ;", BAT_VBAT_Val);

        // ESP_LOGI(BQ25896_TAG, "VSYS Val: %.4f  ;", BAT_VSYS_Val);
        // ESP_LOGI(BQ25896_TAG, "IBAT Val: %.4f  ;", BAT_IBAT_Val);
        // int fault=0;
        // ESP_ERROR_CHECK(i2c_read_byte(I2C_MASTER_NUM, BQ25896_ADDR, 0x0C,(uint8_t *)&fault));
        // if(fault !=0)
        // {
        //     ESP_LOGE(BQ25896_TAG, "Fault!!!!!!!!\n");
        //     break;
        // }
    }
    vTaskDelete(NULL);
}
void BQ25896_OTG_en(int en)
{
    ESP_ERROR_CHECK(i2c_write_bit(I2C_MASTER_NUM, BQ25896_ADDR, 0x03, 5, en));
}



