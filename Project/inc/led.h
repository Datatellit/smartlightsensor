#ifndef	__DRV_LED_H__
#define __DRV_LED_H__


#include "stm8l15x_gpio.h"


//LEDӲ������
#define LED_RED_GPIO_PORT			GPIOD								
#define LED_RED_GPIO_PIN			GPIO_Pin_7

#define LED_GREEN_GPIO_PORT			GPIOD							
#define LED_GREEN_GPIO_PIN			GPIO_Pin_6


/** LED���� */
typedef enum LedPort
{
	LED_RED = 0,		        //��ɫLED
	LED_GREEN			//��ɫLED
}LedPortType;


void drv_led_init( void );
void drv_led_on( LedPortType LedPort );
void drv_led_off( LedPortType LedPort );
void drv_led_flashing( LedPortType LedPort );

//��ɫLED��������
#define led_red_on( )				drv_led_on( LED_RED )
#define led_red_off( )				drv_led_off( LED_RED )
#define led_red_flashing( )			drv_led_flashing( LED_RED )
//��ɫLED��������
#define led_green_on( )				drv_led_on( LED_GREEN )
#define led_green_off( )			drv_led_off( LED_GREEN )
#define led_green_flashing( )		        drv_led_flashing( LED_GREEN )


#endif
