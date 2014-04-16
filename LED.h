#ifndef __LED_H
#define __LED_H

#include <msp430.h>

void init_LED(void);
void LED_on(int LED);
void LED_off(int LED);
void LEDs_clear(void);
void LED_toggle(int LED);

#define ERR_LED_on()       LED_on(4)
#define ERR_LED_off()      LED_off(4)

#define PWR_LED_on()       LED_on(3)
#define PWR_LED_off()      LED_off(3)

#define FLIP_LED_on()      LED_on(2)
#define FLIP_LED_off()     LED_off(2)
#define FLIP_LED_toggle()  LED_toggle(2)

#endif

