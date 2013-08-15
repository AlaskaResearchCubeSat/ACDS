#include <stdio.h>
#include "LED.h"

void init_LED(void){
  //1st pair of LED's
  P2SEL&=~(BIT0|BIT1);
  P2OUT&=~(BIT0|BIT1);
  P2DIR|=BIT0|BIT1;
  //2nd pair of LED's
  P4SEL&=~(BIT0|BIT1);
  P4OUT&=~(BIT0|BIT1);
  P4DIR|=BIT0|BIT1;
}

//turn off all LED's
void LEDs_clear(void){
    //1st pair of LED's
  P2OUT&=~(BIT0|BIT1);
  //2nd pair of LED's
  P4OUT&=~(BIT0|BIT1);
}
  
//turn on an LED
void LED_on(int LED){
  switch(LED){
    case 1:
      P4OUT|=BIT1;
    break;
    case 2:
      P4OUT|=BIT0;
    break;
    case 3:
      P2OUT|=BIT1;
    break;
    case 4:
      P2OUT|=BIT0;
    break;
    default:
      printf("Error: can\'t turn on unknown LED number %i\r\n",LED);
    break;
  }
}

//Turn off an LED
void LED_off(int LED){
  switch(LED){
    case 1:
      P4OUT&=~BIT1;
    break;
    case 2:
      P4OUT&=~BIT0;
    break;
    case 3:
      P2OUT&=~BIT1;
    break;
    case 4:
      P2OUT&=~BIT0;
    break;
    default:
      printf("Error: can\'t turn off unknown LED number %i\r\n",LED);
    break;
  }
}
