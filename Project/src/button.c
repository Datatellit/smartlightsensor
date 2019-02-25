/*
 xlight remoter button functions

- Dimmer keys:
      PD2 -> keyUp
      PD3 -> keyDown
      PD0 -> keyLeft
      PD1 -> KeyRight
      PB0 -> KeyCenter

  - Functuon keys:
      PB1 -> Fn1
      PB2 -> Fn2
      PB3 -> Fn3
      //PD6 -> Fn4

LEDs
  Flashlight (White LED) -> PC1
  On/Off (Green LED) -> PC6
  Device Selection (Red LEDs) -> PC0 to PC3

*/

#include "_global.h"
#include "stm8l15x.h"
#include "stm8l15x_gpio.h"
#include "stm8l15x_exti.h"

#include "timer4.h"
#include "button.h"
#include "ProtocolParser.h"

//---------------------------------------------------
// PIN Map
//---------------------------------------------------
// Button pin map
#define BUTTONS_PORT1           (GPIOC)
#define BUTTON_PIN_SWITCH       (GPIO_Pin_4)

//---------------------------------------------------


// Get Button pin input
#define pinKeySwitch                ((BitStatus)(BUTTONS_PORT1->IDR & (uint8_t)BUTTON_PIN_SWITCH))

#define BUTTON_DEBONCE_DURATION                 3       // The unit is 10 ms, so the duration is 30 ms.
#define BUTTON_WAIT_2S                          100     // The unit is 10 ms, so the duration is 2 s.
#define BUTTON_WAIT_3S                          300     // The unit is 10 ms, so the duration is 3 s.
#define BUTTON_DOUBLE_BTN_DURATION              50      // The unit is 10 ms, so the duration is 500 ms.
#define BUTTON_DOUBLE_BTN_TRACK_DURATION        300     // The unit is 10 ms, so the duration is 3 s.


static button_timer_status_t  m_btn_timer_status[BTN_NUM] = {BUTTON_STATUS_INIT};
static bool detect_double_btn_press[BTN_NUM] = {FALSE};
static bool btn_is_pushed[BTN_NUM] = {FALSE};
static uint16_t btn_bit_postion[BTN_NUM];

static uint8_t m_timer_id_btn_detet[BTN_NUM];
static uint8_t m_timer_id_double_btn_detet[BTN_NUM];

static bool double_button_track = FALSE;
static uint8_t button_status = 0xFF;
static uint8_t button_first_detect_status = 0xFF;

static uint8_t m_timer_id_debonce_detet;

void app_button_event_handler(uint8_t _btn, button_event_t button_event);
void button_push(uint8_t _btn);
void button_release(uint8_t _btn);

static void btn_duration_timeout_handler(uint8_t _btn)
{
  // Assert button
  if( !IS_VALID_BUTTON(_btn) ) return;
  
  button_event_t button_event = BUTTON_INVALID;
  switch (m_btn_timer_status[_btn]) {
  case BUTTON_STATUS_INIT:
    break;
    
  case BUTTON_STATUS_LESS_2S:
    button_event = BUTTON_LONG_HOLD;
    timer_start(m_timer_id_btn_detet[_btn], BUTTON_WAIT_3S);
    m_btn_timer_status[_btn] = BUTTON_STATUS_MORE_2S;
    break;
    
  case BUTTON_STATUS_MORE_2S:
    button_event = BUTTON_VERY_LONG_HOLD;
    m_btn_timer_status[_btn] = BUTTON_STATUS_MORE_5S;
    break;
    
  case BUTTON_STATUS_MORE_5S:
    break;
    
  case BUTTON_STATUS_DOUBLE_TRACK:
    button_event = DOUBLE_BTN_TRACK;
    m_btn_timer_status[_btn] = BUTTON_STATUS_INIT;
    break;
    
  default:
    break;
  }
  
  if( button_event != BUTTON_INVALID ) {
    app_button_event_handler(_btn, button_event);
  }
}

void double_btn_timeout_handler(uint8_t _btn)
{
  // Assert button
  if( !IS_VALID_BUTTON(_btn) ) return;
  
  button_event_t button_event = BUTTON_SHORT_PRESS;
  detect_double_btn_press[_btn] = FALSE;
  m_btn_timer_status[_btn] = BUTTON_STATUS_INIT;
  timer_stop(m_timer_id_double_btn_detet[_btn]);
  app_button_event_handler(_btn, button_event);
}

void btn_debonce_timeout_handler(uint8_t _tag)
{
  uint8_t valid_button;
  uint8_t current_button;
  uint8_t changed_button;
  
  current_button = GPIO_ReadInputData(BUTTONS_PORT1);

  valid_button = ~(current_button ^ button_first_detect_status);    
  changed_button = ((current_button^button_status) & valid_button);
  button_status = current_button;
  
  // Scan all buttons
  uint8_t _btn;
  for( _btn = 0; _btn < BTN_NUM; _btn++ ) {    
    if ((changed_button & btn_bit_postion[_btn]) != 0)
    {
      timer_stop(m_timer_id_btn_detet[_btn]);
      if ((current_button & btn_bit_postion[_btn]) == 0)
      {
        // important
        gKeyLowPowerLeft = 500;
        button_push(_btn);
      }
      else
      {
        button_release(_btn);
      }
    }
  }
}

void button_init()
{
  uint8_t _btn;
  
  // Set button bit postion
  btn_bit_postion[keylstCenter] = BUTTON_PIN_SWITCH;

  GPIO_Init(BUTTONS_PORT1, BUTTON_PIN_SWITCH, GPIO_Mode_In_PU_IT);
  EXTI_DeInit();
  EXTI_SetPinSensitivity(EXTI_Pin_4, EXTI_Trigger_Rising_Falling);
  // Create all timers
  for( _btn = 0; _btn < BTN_NUM; _btn++ ) {
    timer_create(&m_timer_id_btn_detet[_btn], _btn, btn_duration_timeout_handler);
    timer_create(&m_timer_id_double_btn_detet[_btn], _btn, double_btn_timeout_handler);
  }
  timer_create(&m_timer_id_debonce_detet, 0, btn_debonce_timeout_handler);
}

void btn_short_button_press(uint8_t _btn)
{
  // Assert button
  if( !IS_VALID_BUTTON(_btn) ) return;
  switch( _btn ) {
  case keylstCenter:
    Button_Action();
    break;
    
  default:
    break;
  }
}

void btn_double_button_press(uint8_t _btn)
{
  // Assert button
  if( !IS_VALID_BUTTON(_btn) ) return;
  
  switch( _btn ) {

  }  
}

void btn_long_hold_button_press(uint8_t _btn)
{
  // Assert button
  if( !IS_VALID_BUTTON(_btn) ) return;
  
  switch( _btn ) {

  }  
}

void btn_long_button_press(uint8_t _btn)
{
  // Assert button
  if( !IS_VALID_BUTTON(_btn) ) return;
  
  switch( _btn ) {

  }
}

void btn_very_long_hold_button_press(uint8_t _btn)
{
  // Assert button
  if( !IS_VALID_BUTTON(_btn) ) return;
  
  switch( _btn ) {

  }
}

void btn_very_long_button_press(uint8_t _btn)
{
  // Assert button
  if( !IS_VALID_BUTTON(_btn) ) return;
  
  switch( _btn ) {

  }
}

void btn_double_long_hold_press(uint8_t _btn1, uint8_t _btn2)
{
  // Assert button
  if( !IS_VALID_BUTTON(_btn1) || !IS_VALID_BUTTON(_btn2) ) return;
}

void app_button_event_handler(uint8_t _btn, button_event_t button_event)
{
  uint8_t sec_btn = BTN_NUM;
  
  switch (button_event)
  {
  case BUTTON_INVALID:
    break;
    
  case BUTTON_SHORT_PRESS:
    btn_short_button_press(_btn);
    break;
    
  case BUTTON_DOUBLE_PRESS:
    btn_double_button_press(_btn);
    break;
    
  case BUTTON_LONG_HOLD:
    btn_long_hold_button_press(_btn);
    break;
    
  case BUTTON_LONG_PRESS:
    btn_long_button_press(_btn);
    break;
    
  case BUTTON_VERY_LONG_HOLD:
    btn_very_long_hold_button_press(_btn);
    break;
    
  case BUTTON_VERY_LONG_PRESS:
    btn_very_long_button_press(_btn);
    break;
    
  case DOUBLE_BTN_TRACK:
    /*if( btn_is_pushed[keylstFn1] ) sec_btn = keylstFn1;
    else if( btn_is_pushed[keylstFn2] ) sec_btn = keylstFn2;
    else if( btn_is_pushed[keylstFn3] ) sec_btn = keylstFn3;
    else if( btn_is_pushed[keylstFn4] ) sec_btn = keylstFn4;
    if( sec_btn < keylstDummy )
      btn_double_long_hold_press(_btn, sec_btn);*/
    break;
    
  default:
    break;
  }
}

// Only use button1_timer to track double button long hold.
void check_track_double_button(void)
{
  return;
}

void button_push(uint8_t _btn)
{
  // Assert button
  if( !IS_VALID_BUTTON(_btn) ) return;
  
  btn_is_pushed[_btn] = TRUE;
  check_track_double_button();
  
  if (double_button_track == FALSE)
  {
    m_btn_timer_status[_btn] = BUTTON_STATUS_LESS_2S;
    timer_start(m_timer_id_btn_detet[_btn], BUTTON_WAIT_2S);
  }
}

void button_release(uint8_t _btn)
{
  // Assert button
  if( !IS_VALID_BUTTON(_btn) ) return;
  
  btn_is_pushed[_btn] = FALSE;
  button_event_t button_event = BUTTON_INVALID;
  
  check_track_double_button();
  
  switch (m_btn_timer_status[_btn])
  {
  case BUTTON_STATUS_INIT:
    break;
    
  case BUTTON_STATUS_LESS_2S:
    if (detect_double_btn_press[_btn] == FALSE)
    {
      detect_double_btn_press[_btn] = TRUE;
      timer_start(m_timer_id_double_btn_detet[_btn], BUTTON_DOUBLE_BTN_DURATION);  // 500ms
    }
    else
    {
      button_event = BUTTON_DOUBLE_PRESS;
      detect_double_btn_press[_btn] = FALSE;
      timer_stop(m_timer_id_double_btn_detet[_btn]);
    }
    break;
    
  case BUTTON_STATUS_MORE_2S:
    button_event = BUTTON_LONG_PRESS;
    break;
    
  case BUTTON_STATUS_MORE_5S:
    button_event = BUTTON_VERY_LONG_PRESS;
    break;
    
  default:
    break;
  }
  
  m_btn_timer_status[_btn] = BUTTON_STATUS_INIT;
  if (button_event != BUTTON_INVALID) {
    app_button_event_handler(_btn, button_event);
  }
}

void button_event_handler(uint8_t _pin)
{
  gKeyLowPowerLeft = 500;
  button_first_detect_status = GPIO_ReadInputData(BUTTONS_PORT1);
  timer_start(m_timer_id_debonce_detet, BUTTON_DEBONCE_DURATION);
}
