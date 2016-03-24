#include <stdio.h>
#include "LED.h"

#ifdef DEV_BUILD
  //LED definitions for devboard
  #define LED_BITS          (0xFF)
  #define LED_NUM_MAX       7
  #define LED_NUM_MIN       0
  #define LED_REG(r)        (P7##r)
#else
  //LED definitions for ACDS board hardware
  #define LED_BITS          (0x0F)
  #define LED_NUM_MAX       4
  #define LED_NUM_MIN       0
  #define LED_REG(r)        (P11##r)

#endif


void init_LED(void){
  LED_REG(OUT)&=~LED_BITS;
  LED_REG(DIR)|= LED_BITS;
  LED_REG(SEL0)&=~LED_BITS;
}

//turn off all LED's
void LEDs_clear(void){
  LED_REG(OUT)&=~LED_BITS;
}
  
//turn on an LED
void LED_on(int LED){
  if(LED>LED_NUM_MAX || LED<LED_NUM_MIN){
    printf("Error: can\'t turn on unknown LED number %i\r\n",LED);
    return;
  }
  LED_REG(OUT)|=1<<LED;
}

//Turn off an LED
void LED_off(int LED){
  if(LED>LED_NUM_MAX || LED<LED_NUM_MIN){
    printf("Error: can\'t turn on unknown LED number %i\r\n",LED);
    return;
  }
  LED_REG(OUT)&=~(1<<LED);
}

//Toggle an LED
void LED_toggle(int LED){
  if(LED>LED_NUM_MAX || LED<LED_NUM_MIN){
    printf("Error: can\'t turn on unknown LED number %i\r\n",LED);
    return;
  }
  LED_REG(OUT)^=1<<LED;
}
