#ifndef __TMP112_H__
#define __TMP112_H__

#include "i2c-easy.h"

#define I2C_MASTER_NUM I2C_NUM_0

#define A0_PIN_CONN_GND 0
#define A0_PIN_CONN_VCC 1
#define A0_PIN_CONN_SDA 2
#define A0_PIN_CONN_SCL 3

#define A0_PIN_CONN A0_PIN_CONN_GND

#if A0_PIN_CONN == A0_PIN_CONN_GND
#define TMP112_ADDR 0b1001000
#elif A0_PIN_CONN == A0_PIN_CONN_VCC
#define TMP112_ADDR 0b1001001
#elif A0_PIN_CONN == A0_PIN_CONN_SDA
#define TMP112_ADDR 0b1001010
#elif A0_PIN_CONN == A0_PIN_CONN_SCL
#define TMP112_ADDR 0b1001011
#endif

#define TMP112_PTR_REG_Temperature 0b00
#define TMP112_PTR_REG_Configuration 0b01
#define TMP112_PTR_REG_LowThreshold 0b10
#define TMP112_PTR_REG_HighThreshold 0b11

#define FaultQueue_mask 11
#define FaultQueue_1 0b00
#define FaultQueue_2 0b01
#define FaultQueue_4 0b10
#define FaultQueue_6 0b11

#define Polarity_mask 10
#define Polarity_HighValid 1
#define Polarity_LowValid 0

#define Thermostat_mask 9
#define Thermostat_Comparator 0
#define Thermostat_Interrupt 1

#define ShutdownEn_mask 8
#define ShutdownEn_SHDN 1
#define ShutdownEn_NORM 0

#define ConvertPeriod_mask 6
#define ConvertPeriod_025 0b00
#define ConvertPeriod_1 0b01
#define ConvertPeriod_4 0b10
#define ConvertPeriod_8 0b11

#define Resolution_mask 4
#define Resolution_12bit 0
#define Resolution_13bit 1

#define OneShot_mask 7
#define Alert_mask 5
typedef struct TMP112_Config_t
{
    int FaultQueue;
    int Polarity;
    int Thermostat;
    int ShutdownEn;
    int ConvertPeriod;
    int Resolution;
    float HighThreshold;
    float LowThreshold;
} 
TMP112_Config_t TMP112_Config_old;
void TMP112_Init(TMP112_Config_t TMP112_Config);

float TMP112_Get_One_Shot_Value();

#endif //__TMP112_H__