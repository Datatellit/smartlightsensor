#include "ADC1Dev.h"

#include "sen_als.h"

// ALS sensor library
#define ALS_DATA_PORT                   GPIOD
#define ALS_DATA_PIN_ID                 GPIO_Pin_3

#define ALS_MA_NUM             10

bool als_ready = FALSE;
bool als_alive = FALSE;
u8 als_value;

// Moving average
u8 als_mvPtr = 0;
u8 als_mvData[ALS_MA_NUM] = {0};
u16 als_mvSum = 0;

bool als_checkData()
{
  enable_als();
  als_mvSum = 0;
  for(uint8_t i=0;i<ALS_MA_NUM;i++)
  {
    uint16_t newData = adc_read();
    if(i != 0)
    {
      uint8_t level = (uint32_t)newData*100 / 4096;
      als_mvSum += level;
    }
  }
  
  als_value = als_mvSum / (ALS_MA_NUM-1);
    
  return TRUE;
}


