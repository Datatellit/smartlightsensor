#include "ADC1Dev.h"

#include "sen_als.h"

// ALS sensor library
#define ALS_DATA_PORT                   GPIOD
#define ALS_DATA_PIN_ID                 GPIO_Pin_3

#define ALS_MA_NUM             10

uint8_t als_value;

bool als_checkData()
{
  enable_als();
  uint16_t lv_adcValue = adc_readMAValue(ALS_MA_NUM);
  // Scale from [0..4095] down to [0..100]
  als_value = (uint16_t)(lv_adcValue / 40.95 + 0.5);
  return TRUE;
}


