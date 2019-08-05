#include "_global.h"
#include "delay.h"
#include "rf24l01.h"
#include "timer4.h"
#include "button.h"
#include "MyMessage.h"
#include "ProtocolParser.h"
#include "sen_als.h"
#include "ADC1Dev.h"
#include "stm8l15x_rtc.h"
#include "led.h"
#include "UsartDev.h"
#include "XlightComBus.h"

/*
Xlight Remoter Program
License: MIT

Auther: Baoshi Sun
Email: bs.sun@datatellit.com, bs.sun@uwaterloo.ca
Github: https://github.com/sunbaoshi1975
Please visit xlight.ca for product details

RF24L01 connector pinout:
GND    VCC
CE     CSN
SCK    MOSI
MISO   IRQ

Connections:
  PB4 -> CE
  PC6 -> CSN (11 buttons)
  PB5 -> SCK
  PB6 -> MOSI
  PB7 -> MISO
  PD5 -> IRQ

*/

void ioinit()
{
  GPIO_Init(GPIOB , GPIO_Pin_0 , GPIO_Mode_Out_PP_High_Fast);
  //GPIO_Init(GPIOC , GPIO_Pin_4 , GPIO_Mode_In_FL_IT);
}

#define MAX_RF_FAILED_TIME              10      // Reset RF module when reach max failed times of sending


// Timeout
#define RTE_TM_CONFIG_MODE              12000  // timeout in config mode, about 120s (12000 * 10ms)


// Public variables
Config_t gConfig;

MyMessage_t sndMsg;
MyMessage_t rcvMsg;
uint8_t *psndMsg = (uint8_t *)&sndMsg;
uint8_t *prcvMsg = (uint8_t *)&rcvMsg;
bool gNeedSaveBackup = FALSE;
bool gIsStatusChanged = FALSE;
bool gIsConfigChanged = FALSE;
bool gResetRF = FALSE;
bool gResetNode = FALSE;

uint8_t _uniqueID[UNIQUE_ID_LEN];
uint8_t m_cntRFSendFailed = 0;
uint8_t gIsWakeup = 1;
uint8_t gIsInConfig = 0;
uint16_t gKeyLowPowerLeft = 0;
uint16_t gKeyLastInterval = 0;
bool bPowerOn = FALSE;
uint16_t gRedLefttime = 0;
uint16_t gGreenLefttime = 0;


/** 电池电量LEVEL */
typedef enum EQLevel
{
	LOWER = 0,		        //欠压
	NORMAL= 1,			//正常
        CHARGE= 2,                      //充电
        FULL  = 3                       //充满
}EQLevelType;

/** 工作状态 */
typedef enum WorkMode
{
	POWERDOWN = 0,		        //关机
	POWERUP   = 1,			//开机
}WorkModeType;

typedef enum ButtonMode
{
        NOTHING = 0,
	QUERY = 1,		        //关机
	SWITCH   = 2,			//开机
}ButtonModeType;


EQLevelType gCurrEQLevel = NORMAL;
WorkModeType gWorkMode = POWERUP;
ButtonModeType gButtonMode = NOTHING;
// collect interval
uint16_t mCollectInterval = 0;


uint8_t mutex;
uint16_t configMode_tick = 0;
void tmrProcess();


static void clock_init(void)
{
  CLK_DeInit();
  CLK_HSICmd(ENABLE);
  CLK_SYSCLKDivConfig(SYS_CLOCK_DIVIDER);
  CLK_PeripheralClockConfig(CLK_Peripheral_TIM4, ENABLE);
  //CLK_ClockSecuritySystemEnable();
}

void SetFlashlight(uint8_t _st)
{
#ifdef ENABLE_FLASHLIGHT_LASER
  if( _st == DEVICE_SW_ON ) {
    ledFlashLight(SET);
  } else if( _st == DEVICE_SW_OFF ) {
    ledFlashLight(RESET);
  } else {
    ledToggleFlashLight;
  }
#endif  
}

void SetLasterBeam(uint8_t _st)
{
#ifdef ENABLE_FLASHLIGHT_LASER
  if( _st == 1 ) {
    ledLaserPen(SET);
  } else if( _st == 0 ) {
    ledLaserPen(RESET);
  } else {
    ledToggleLaserPen;
  }
#endif  
}

// Blink LED to indicate starting
void LED_Blink(bool _flash, bool _fast) {
  if( _flash )
    SetFlashlight(1);
  else
    SetLasterBeam(1);
  //delay_ms(_fast ? 200 : 500);
  WaitMutex(_fast ? 0x4FFF : 0xBFFF);
  if( _flash )
    SetFlashlight(0);
  else
    SetLasterBeam(0);
  //delay_ms(_fast ? 200 : 500);
  WaitMutex(_fast ? 0x4FFF : 0xBFFF);
}


/**
  * @brief  configure GPIOs before entering low power
	* @caller lowpower_config
  * @param None
  * @retval None
  */  
void GPIO_LowPower_Config(void)
{
    /*GPIO_Init(GPIOB , GPIO_Pin_0 , GPIO_Mode_Out_PP_High_Fast);
    GPIO_WriteBit(GPIOB,GPIO_Pin_0,RESET);
    GPIO_Init(GPIOA, GPIO_Pin_2|GPIO_Pin_4, GPIO_Mode_In_FL_No_IT); //与设置成GPIO_Mode_Out_PP_Low_Slow 无差异
    GPIO_Init(GPIOA, GPIO_Pin_0|GPIO_Pin_1, GPIO_Mode_Out_PP_Low_Slow);  // 无效
    GPIO_Init(GPIOA, GPIO_Pin_3|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7, GPIO_Mode_Out_PP_Low_Slow);
    GPIO_Init(GPIOB, GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3, GPIO_Mode_Out_PP_Low_Slow);                     
    //GPIO_Init(GPIOC, GPIO_Pin_0, GPIO_Mode_Out_PP_High_Slow);  // 无效
    //GPIO_Init(GPIOC, GPIO_Pin_1, GPIO_Mode_Out_PP_High_Slow);  // 无效
    GPIO_Init(GPIOC, GPIO_Pin_4, GPIO_Mode_Out_PP_Low_Slow); 
    GPIO_Init(GPIOC, GPIO_Pin_3, GPIO_Mode_Out_PP_Low_Slow); 
    GPIO_Init(GPIOC, GPIO_Pin_5, GPIO_Mode_Out_PP_Low_Slow); 
    GPIO_Init(GPIOC, GPIO_Pin_7, GPIO_Mode_Out_PP_Low_Slow); 
    GPIO_Init(GPIOD, GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_3,GPIO_Mode_In_FL_No_IT);  //无效
    GPIO_Init(GPIOD, GPIO_Pin_2|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7, GPIO_Mode_Out_PP_Low_Slow);  */
    GPIO_Init(GPIOB , GPIO_Pin_0 , GPIO_Mode_Out_PP_High_Fast);
    GPIO_WriteBit(GPIOB,GPIO_Pin_0,RESET);
    GPIO_Init(GPIOA, GPIO_Pin_2|GPIO_Pin_4, GPIO_Mode_Out_PP_Low_Slow);
    GPIO_Init(GPIOA, GPIO_Pin_3|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7, GPIO_Mode_Out_PP_Low_Slow);
    GPIO_Init(GPIOB, GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3, GPIO_Mode_Out_PP_Low_Slow);
    GPIO_Init(GPIOC, GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2, GPIO_Mode_Out_PP_Low_Slow);
    GPIO_Init(GPIOC, GPIO_Pin_3, GPIO_Mode_Out_PP_Low_Slow); 
    //GPIO_Init(GPIOC, GPIO_Pin_4, GPIO_Mode_Out_PP_Low_Slow); 
    GPIO_Init(GPIOC, GPIO_Pin_5, GPIO_Mode_Out_PP_Low_Slow); 
    GPIO_Init(GPIOC, GPIO_Pin_7, GPIO_Mode_Out_PP_Low_Slow); 
    GPIO_Init(GPIOD, GPIO_Pin_2|GPIO_Pin_4|GPIO_Pin_5, GPIO_Mode_Out_PP_Low_Slow);
}

void RTC_Config()
{
  CLK_RTCClockConfig(CLK_RTCCLKSource_LSI, CLK_RTCCLKDiv_1);
  CLK_PeripheralClockConfig(CLK_Peripheral_RTC, ENABLE);
  RTC_WakeUpClockConfig(RTC_WakeUpClock_CK_SPRE_16bits);
  RTC_ITConfig(RTC_IT_WUT, ENABLE);
}


// Enter Low Power Mode, which can be woken up by external interupts
void lowpower_config(void) {
  // Set STM8 in low power
  PWR->CSR2 = 0x2;
  
  // Stop Timers
  TIM4_DeInit();
  
  // Set GPIO in low power
  GPIO_LowPower_Config();
  
  // RF24 Chip in low power
  RF24L01_DeInit();
  // TODO how process???
  /*while ((CLK->ICKCR & 0x04) != 0x00)
  {
    feed_wwdg();
  }*/
  
  ADC_DeInit(ADC1);
  
  // Stop peripheral clocks
  CLK_PeripheralClockConfig(CLK_Peripheral_TIM5, DISABLE);
  CLK_PeripheralClockConfig(CLK_Peripheral_TIM4, DISABLE);
  CLK_PeripheralClockConfig(CLK_Peripheral_TIM3, DISABLE);
  CLK_PeripheralClockConfig(CLK_Peripheral_TIM2, DISABLE);
  CLK_PeripheralClockConfig(CLK_Peripheral_TIM1, DISABLE);
  CLK_PeripheralClockConfig(CLK_Peripheral_I2C1, DISABLE);
  CLK_PeripheralClockConfig(CLK_Peripheral_SPI1, DISABLE);
  CLK_PeripheralClockConfig(CLK_Peripheral_USART1, DISABLE);
  CLK_PeripheralClockConfig(CLK_Peripheral_DAC, DISABLE);
  CLK_PeripheralClockConfig(CLK_Peripheral_ADC1, DISABLE);
  //CLK_PeripheralClockConfig(CLK_Peripheral_RTC, DISABLE);
  CLK_PeripheralClockConfig(CLK_Peripheral_LCD, DISABLE);
  CLK_PeripheralClockConfig(CLK_Peripheral_AES, DISABLE);
  
  //CLK_HSICmd(DISABLE);  
  //CLK_DeInit();
  RTC_Config();      // add 3uA
  RTC_SetWakeUpCounter(5);
  RTC_WakeUpCmd(ENABLE);
}

// Resume Normal Mode
void wakeup_config(void) {
  clock_init();
  timer_init();
  GPIO_WriteBit(GPIOB,GPIO_Pin_0,SET);
  ADC_Config(); 
  RF24L01_init();
  NRF2401_EnableIRQ();
  UpdateNodeAddress(NODEID_GATEWAY);
#ifdef DEBUG_LOG
  usart_config(9600);
#endif
}


/*// Save config to Flash
void SaveConfig()
{
#ifndef ENABLE_SDTM
  if( gIsConfigChanged ) {
    Flash_WriteBuf(FLASH_DATA_EEPROM_START_PHYSICAL_ADDRESS, (uint8_t *)&gConfig, sizeof(gConfig));
    gIsConfigChanged = FALSE;
  }
#endif  
}*/

// Save config to Flash
void SaveBackupConfig()
{
  if( gNeedSaveBackup ) {
    // Overwrite entire config bakup FLASH
    if(Flash_WriteDataBlock(BACKUP_CONFIG_BLOCK_NUM, (uint8_t *)&gConfig, sizeof(gConfig)))
    {
      gNeedSaveBackup = FALSE;
    }
  }
}

// Save status to Flash
void SaveStatusData()
{
    // Skip the first byte (version)
    uint8_t pData[50] = {0};
    uint16_t nLen = (uint16_t)(&(gConfig.nodeID)) - (uint16_t)(&gConfig);
    memcpy(pData, (uint8_t *)&gConfig, nLen);
    if(Flash_WriteDataBlock(STATUS_DATA_NUM, pData, nLen))
    {
      gIsStatusChanged = FALSE;
    }
}

// Save config to Flash
void SaveConfig()
{
  if( gIsStatusChanged ) {
    // Overwrite only Static & status parameters (the first part of config FLASH)
    SaveStatusData();
    gIsConfigChanged = TRUE;
  } 
  if( gIsConfigChanged ) {
    // Overwrite entire config FLASH
    if(Flash_WriteDataBlock(0, (uint8_t *)&gConfig, sizeof(gConfig)))
    {
      gIsStatusChanged = FALSE;
      gIsConfigChanged = FALSE;
      gNeedSaveBackup = TRUE;
      return;
    }
  } 
}

bool IsConfigInvalid() {
  return( gConfig.version > XLA_VERSION || gConfig.version < XLA_MIN_VER_REQUIREMENT 
       || /*!IS_VALID_REMOTE(gConfig.type)  || */gConfig.nodeID == 0
       || gConfig.rfPowerLevel > RF24_PA_MAX || gConfig.rfChannel > 127 || gConfig.rfDataRate > RF24_250KBPS );
}

bool isNodeIdInvalid(uint8_t nodeid)
{
  return( !IS_SENSOR_NODEID(nodeid)  );
}

/*// Load config from Flash
void LoadConfig()
{
    // Load the most recent settings from FLASH
    Flash_ReadBuf(FLASH_DATA_EEPROM_START_PHYSICAL_ADDRESS, (uint8_t *)&gConfig, sizeof(gConfig));
    if( IsConfigInvalid() ) {
      Flash_ReadBuf(BACKUP_CONFIG_ADDRESS, (uint8_t *)&gConfig, sizeof(gConfig));
      if( IsConfigInvalid() ) {
        memset(&gConfig, 0x00, sizeof(gConfig));
        gConfig.version = XLA_VERSION;
        gConfig.indDevice = 0;
        gConfig.present = 0;
        gConfig.inPresentation = 0;
        gConfig.enSDTM = 0;
        gConfig.rptTimes = 1;
        gConfig.nodeID = 130;
        gConfig.rfChannel = RF24_CHANNEL;
        gConfig.rfPowerLevel = RF24_PA_MAX;
        gConfig.rfDataRate = RF24_250KBPS;      
        memcpy(gConfig.NetworkID, RF24_BASE_RADIO_ID, ADDRESS_WIDTH);
      }
      gIsConfigChanged = TRUE;
    }
    else {
      uint8_t bytVersion;
      Flash_ReadBuf(BACKUP_CONFIG_ADDRESS, (uint8_t *)&bytVersion, sizeof(bytVersion));
      if( bytVersion != gConfig.version ) gNeedSaveBackup = TRUE;
    }
    // Load the most recent status from FLASH
    uint8_t pData[50] = {0};
    uint16_t nLen = (uint16_t)(&(gConfig.nodeID)) - (uint16_t)(&gConfig);
    Flash_ReadBuf(STATUS_DATA_ADDRESS, pData, nLen);
    if(pData[0] >= XLA_MIN_VER_REQUIREMENT && pData[0] <= XLA_VERSION)
    {
      memcpy(&gConfig,pData,nLen);
    }
    gConfig.rfChannel = 87;
}*/

void UpdateNodeAddress(uint8_t _tx) {
  memcpy(rx_addr, gConfig.NetworkID, ADDRESS_WIDTH);
  rx_addr[0] = gConfig.nodeID;
  memcpy(tx_addr, gConfig.NetworkID, ADDRESS_WIDTH);
  
  if( _tx == NODEID_RF_SCANNER ) {
    tx_addr[0] = NODEID_RF_SCANNER;
  } else {  
    tx_addr[0] = NODEID_GATEWAY;
  }
  RF24L01_setup(gConfig.rfChannel, gConfig.rfDataRate, gConfig.rfPowerLevel, BROADCAST_ADDRESS);     // With openning the boardcast pipe
}  

bool NeedUpdateRFAddress(uint8_t _dest) {
  bool rc = FALSE;
  if( sndMsg.header.destination == NODEID_RF_SCANNER && tx_addr[0] != NODEID_RF_SCANNER ) {
    UpdateNodeAddress(NODEID_RF_SCANNER);
    rc = TRUE;
  } else if( sndMsg.header.destination != NODEID_RF_SCANNER && tx_addr[0] != NODEID_GATEWAY ) {
    UpdateNodeAddress(NODEID_GATEWAY);
    rc = TRUE;
  }
  UpdateNodeAddress(NODEID_GATEWAY);
  return rc;
}


bool WaitMutex(uint32_t _timeout) {
  while(_timeout--) {
    if( mutex > 0 ) return TRUE;
    feed_wwdg();
  }
  return FALSE;
}

// reset rf
void ResetRFModule()
{
  if(gResetRF)
  {
    RF24L01_init();
    NRF2401_EnableIRQ();
    UpdateNodeAddress(NODEID_GATEWAY);
    gResetRF=FALSE;
  }
}


// Send message and switch back to receive mode
bool SendMyMessage() {
  if( bMsgReady ) {
    
    // Change tx destination if necessary
    NeedUpdateRFAddress(sndMsg.header.destination);
    
    uint8_t lv_tried = 0;
    uint16_t delay;
    while (lv_tried++ <= gConfig.rptTimes ) {
      feed_wwdg();
      mutex = 0;
      RF24L01_set_mode_TX();
      RF24L01_write_payload(psndMsg, PLOAD_WIDTH);
      WaitMutex(0x1FFFF);
      if (mutex == 1) {
        m_cntRFSendFailed = 0;
        break; // sent sccessfully
      } else if( m_cntRFSendFailed++ > MAX_RF_FAILED_TIME ) {
        // Reset RF module
        m_cntRFSendFailed = 0;
        //WWDG->CR = 0x80;
        // RF24 Chip in low power
        RF24L01_DeInit();
        delay = 0x1FFF;
        while(delay--)feed_wwdg();
        RF24L01_init();
        NRF2401_EnableIRQ();
        UpdateNodeAddress(NODEID_GATEWAY);
        continue;
      }
      
      //The transmission failed, Notes: mutex == 2 doesn't mean failed
      //It happens when rx address defers from tx address
      //asm("nop"); //Place a breakpoint here to see memory
      // Repeat the message if necessary
      uint16_t delay = 0xFFF;
      while(delay--)feed_wwdg();
    }
    
    // Switch back to receive mode
    bMsgReady = 0;
    RF24L01_set_mode_RX();
  }

  return(mutex > 0);
}

// Change LED or Laser to indecate execution of specific operation
void OperationIndicator() {
 //todo
}

uint16_t eqv;
void Check_eq()
{
  uint16_t eq1,eq2;
  eq_checkData(&eq1,&eq2);
  printnum(eq1);
  printlog("-");
  printnum(eq2);
  if(eq1 > eq2 + 250) 
  { //eqv1-eqv2 > 0.2v (eqv1 = eq1*3.3/4096)
    // 电池进电

    printlog("charge...");

    gCurrEQLevel = CHARGE;
    drv_led_off(LED_GREEN);
    drv_led_on(LED_RED);
  }
  else
  { // 电池不进电（插充电器但电池已充满或者没插充电器）
    printlog("normal...");

    eqv = (uint32_t)eq2*330 / 2048;
    printnum(eqv);
    if(eqv >= 460)
    { // 插充电器，电池已充满
      printlog("full...");
      drv_led_off(LED_RED);
      gCurrEQLevel = FULL;
    }
    else
    {
      printlog("not full...");
      drv_led_off(LED_GREEN);
      drv_led_off(LED_RED);
      gCurrEQLevel = NORMAL;
      if(eqv < LOWPOWER)
      {
        gCurrEQLevel = LOWER;
      }
    }
  }
}

int main( void ) {

  // Init clock, timer and button
  clock_init();
  timer_init();
  button_init();
  drv_led_init();
  // Go on only if NRF chip is presented
  RF24L01_init();
  while(!NRF24L01_Check());
  
  // Load config from Flash
  FLASH_DeInit();
  Read_UniqueID(_uniqueID, UNIQUE_ID_LEN);
  LoadConfig();
  // NRF_IRQ
  NRF2401_EnableIRQ();

  // Init Watchdog
  wwdg_init();
  
  TIM4_10ms_handler = tmrProcess;
  ADC_Config();
#ifdef DEBUG_LOG
  usart_config(9600);
#endif
  ////////////////PB0 control pin test code///////////////////////////
  ioinit();
  GPIO_WriteBit(GPIOB,GPIO_Pin_0,SET);
  ////////////////PB0 control pin test code///////////////////////////
  // Send Presentation Message
  Msg_Presentation();
  SendMyMessage();         // add 20uA,powerdown rf chip when rfdeinit,resolve

  printlog("start...");
  gIsInConfig = 1;
  while (1) {
       
    // Feed the Watchdog
    feed_wwdg();
    LedPortType led = LED_GREEN;
    ///////////////ALS collect,and send////////////////////////
    if(gIsWakeup)
    { // rtc timer wakeup
      gIsWakeup = 0;
      if(gWorkMode == POWERUP)
      {
        if(!bPowerOn)
        {
          bPowerOn = TRUE;
          wakeup_config();
        }   
        Check_eq();
        
        if(gCurrEQLevel == LOWER)
        {
          led = LED_RED;
        }
        drv_led_on(led);
        als_checkData();
        Msg_SenALS(als_value);
        SendMyMessage();
      }
    }
    ///////////////ALS collect,and send////////////////////////
    if(gIsInConfig)
    { // 正在配置中
      if(gKeyLowPowerLeft == 0)
      {
        gKeyLowPowerLeft = 600;
        drv_led_on(LED_GREEN);
      }

      ////////////rfscanner process///////////////////////////////
      ProcessOutputCfgMsg(); 
      // reset rf
      ResetRFModule();
      if(gResetNode)
      {
        gResetNode = FALSE;
      }
      ////////////rfscanner process/////////////////////////////// 
      // Save Config if Changed
      SaveConfig(); 
      // Save config into backup area
      SaveBackupConfig(); 
    }
    if(gKeyLowPowerLeft>0)
    { // 按键唤醒
       if(!bPowerOn)
      {
        bPowerOn = TRUE;
        wakeup_config();
      } 
    }
    if(gKeyLastInterval >= 500)
    { //按键超时
      gKeyLastInterval = 0;
      gKeyLowPowerLeft = 0;
      if(gWorkMode == POWERUP)
      {
        drv_led_off(LED_GREEN);
      }
      else
      {
        drv_led_off(LED_RED);
      }
    }
    if(gKeyLowPowerLeft == 0)
    {
        gKeyLastInterval = 0;
        if(gCurrEQLevel == NORMAL || gCurrEQLevel == LOWER)
        { // 非充电状态下，低功耗时必须保证灯灭
          printlog("enter low...");
          drv_led_off(LED_RED);
          drv_led_off(LED_GREEN);
        }
        else
        { // 充电状态下，根据充电状态灯常亮

          printlog("not low charge...");

          if(gCurrEQLevel == FULL)
          {    
            drv_led_off(LED_RED);
            drv_led_on(LED_GREEN);
          }
          else if(gCurrEQLevel == CHARGE)
          {
            drv_led_off(LED_GREEN);
            drv_led_on(LED_RED);
          }
        }
        gIsInConfig = 0;
        lowpower_config();
        halt();
        bPowerOn = FALSE;
    }
  }
}

void Button_Action()
{
  if(gKeyLastInterval >100 && gKeyLastInterval <=500)
  { //switch status
    gKeyLastInterval = 0;
    if(gWorkMode == POWERUP)
    {
      gWorkMode = POWERDOWN;
      drv_led_off(LED_GREEN);
    }
    else
    {
      gWorkMode = POWERUP;
      drv_led_off(LED_RED);
    }
  }
  else if(gKeyLastInterval >= 0 && gKeyLastInterval<=100)
  { // query status
    gButtonMode = QUERY;
  }
  if(gWorkMode == POWERUP)
  {
    drv_led_on(LED_GREEN);
  }
  else
  {
    drv_led_on(LED_RED);
  }
}

// Execute timer operations
void tmrProcess() {
  if(gKeyLowPowerLeft>0)
  {
    gKeyLowPowerLeft--;
  }
  if(gKeyLastInterval>=0)
  {
    gKeyLastInterval++;
  }
}

void RF24L01_IRQ_Handler() {
  tmrIdleDuration = 0;
  if(RF24L01_is_data_available()) {
    //Packet was received
    RF24L01_clear_interrupts();
    RF24L01_read_payload(prcvMsg, PLOAD_WIDTH);
    bMsgReady = ParseProtocol();
    return;
  }
 
  uint8_t sent_info;
  if (sent_info = RF24L01_was_data_sent()) {
    //Packet was sent or max retries reached
    RF24L01_clear_interrupts();
    mutex = sent_info; 
    return;
  }

   RF24L01_clear_interrupts();
}

/**
  * @brief RTC / CSS_LSE Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(RTC_CSSLSE_IRQHandler, 4)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
  RTC_ClearITPendingBit(RTC_IT_WUT); //中断标志位要清除
  gIsWakeup = 1;
}