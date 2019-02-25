#ifndef __GLOBAL_H
#define __GLOBAL_H

#include <stm8l15x.h> //Required for the stdint typedefs
#include "stdio.h"
#include "string.h"
#include "stm8l15x_conf.h"
#include "common.h"

// Xlight Application Identification
#define XLA_VERSION               0x20
#define XLA_ORGANIZATION          "xlight.ca"               // Default value. Read from EEPROM
#define XLA_PRODUCT_NAME          "XRemote"                 // Default value. Read from EEPROM

#define FULLPOWER        420
#define LOWPOWER         368


#define XLA_MIN_VER_REQUIREMENT   0x20
typedef struct
{
  // Static & status parameters
  UC version                  :8;           // Data version, other than 0xFF
  UC present                  :1;           // 0 - not present; 1 - present
  UC inPresentation           :1;           // whether in presentation
  UC inConfigMode             :1;           // whether in config mode
  UC reserved0                :5;
  
  // Configurable parameters
  UC nodeID;                                // Node ID for Remote on specific controller
  UC subID;                                 // SubID
  UC NetworkID[6];
  UC rfChannel;                             // RF Channel: [0..127]
  UC rfPowerLevel             :2;           // RF Power Level 0..3
  UC rfDataRate               :2;           // RF Data Rate [0..2], 0 for 1Mbps, or 1 for 2Mbps, 2 for 250kbs
  UC rptTimes                 :2;           // Sending message max repeat times [0..3]
  UC enSDTM                   :1;           // Simple Direct Test Mode Flag
  UC reserved1                :1;
  UC type;                                  // Type of Remote
  US token;                                 // Current token
  UC indDevice                :3;           // Current Device Index: [0..3]
  UC reserved2                :5;
} Config_t;

extern Config_t gConfig;
extern bool gIsChanged;
extern bool gNeedSaveBackup;
extern bool gIsStatusChanged;
extern bool gResetRF;
extern bool gResetNode;

extern uint8_t _uniqueID[UNIQUE_ID_LEN];
extern uint16_t gKeyLowPowerLeft;

bool WaitMutex(uint32_t _timeout);
void RF24L01_IRQ_Handler();
uint8_t ChangeCurrentDevice(uint8_t _newDev);
void UpdateNodeAddress(uint8_t _tx);
bool SendMyMessage();
void EraseCurrentDeviceInfo();
void ToggleSDTM();
void SetConfigMode(bool _sw, uint8_t _devIndex);
bool SayHelloToDevice(bool infinate);
void Button_Action();

//#define TEST
#ifdef TEST
#define     PC1_Low                GPIO_ResetBits(GPIOC, GPIO_Pin_1)
#define     PC3_Low                GPIO_ResetBits(GPIOC, GPIO_Pin_3)
#define     PC5_Low                GPIO_ResetBits(GPIOC, GPIO_Pin_5)
#define     PC1_High               GPIO_SetBits(GPIOC, GPIO_Pin_1)
#define     PC3_High               GPIO_SetBits(GPIOC, GPIO_Pin_3)
#define     PC5_High               GPIO_SetBits(GPIOC, GPIO_Pin_5)
#endif

#endif /* __GLOBAL_H */