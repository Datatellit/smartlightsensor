#include "ADC1Dev.h"
#include "_global.h"

// ToDO: 
// 1. Change to Interupt mode
// 2. Average values and discard extreme samples

void ADC_Config()
{
  CLK_PeripheralClockConfig(CLK_Peripheral_ADC1 , ENABLE);    //ʹ��ADCʱ��
  
  ADC_Init(ADC1 ,
            ADC_ConversionMode_Single , //����ADC1�ǵ��β���
            ADC_Resolution_12Bit ,  //����ADC1Ϊ12BIT��ת������ 
            ADC_Prescaler_2         //����ADC1��ʱ��Ϊ2��Ƶ
           );
   /* ADC_ChannelCmd(ADC1 ,
                 ADC_Channel_19 ,   //ʹ��ADC1��ͨ��19
                 ENABLE
                 );*/
    ADC_Cmd(ADC1 , ENABLE);  //ʹ��ADC1  

}
void enable_als()
{
  ADC_ChannelCmd(ADC1, ADC_Channel_19, ENABLE);  //PD3
  ADC_ChannelCmd(ADC1, ADC_Channel_22, DISABLE); //PD0
  ADC_ChannelCmd(ADC1, ADC_Channel_21, DISABLE); //PD1,eq2
}
void enable_eq1()
{
  ADC_ChannelCmd(ADC1, ADC_Channel_19, DISABLE);
  ADC_ChannelCmd(ADC1, ADC_Channel_22, ENABLE); //PD0
  ADC_ChannelCmd(ADC1, ADC_Channel_21, DISABLE); //PD1
}
void enable_eq2()
{
  ADC_ChannelCmd(ADC1, ADC_Channel_19, DISABLE);
  ADC_ChannelCmd(ADC1, ADC_Channel_22, DISABLE); //PD0
  ADC_ChannelCmd(ADC1, ADC_Channel_21, ENABLE); //PD1
}

uint16_t adc_value=0;
uint16_t adc_read()
{ 
  ADC_SoftwareStartConv(ADC1);  //����ADC1��ʼת������
  
  while(ADC_GetFlagStatus(ADC1 , ADC_FLAG_EOC) == RESET);   //�ȴ�ADC��������
  ADC_ClearFlag(ADC1 , ADC_FLAG_EOC);  //���ת��������־
  
  adc_value = ADC_GetConversionValue(ADC1);  //����ADCת�����
  
  return adc_value;
}

#define COL_MA_NUM             10
void eq_checkData(uint16_t* eq1,uint16_t* eq2)
{
  enable_eq1();
  uint16_t eq1Sum = 0;
  for(uint8_t i=0;i<COL_MA_NUM;i++)
  {
    uint16_t newData = adc_read();
    if(i != 0)
    {
      eq1Sum += newData;
    }
  }
  *eq1 = eq1Sum / (COL_MA_NUM-1);
    
  enable_eq2();
  uint16_t eq2Sum = 0;
  for(uint8_t i=0;i<COL_MA_NUM;i++)
  {
    uint16_t newData = adc_read();
    if(i != 0)
    {
      eq2Sum += newData;
    }
  }
  *eq2 = eq2Sum / (COL_MA_NUM-1);
  
}