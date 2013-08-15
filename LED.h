#ifndef __LED_H
#define __LED_H

#include <msp430.h>

void init_LED(void);
void LED_on(int LED);
void LED_off(int LED);
void LEDs_clear(void);

#define ERR_LED_on()  LED_on(4)
#define ERR_LED_off()  LED_off(4)

#endif

