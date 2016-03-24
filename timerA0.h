#ifndef __TIMER_A0_H
#define __TIMER_A0_H

//use majority function so the timer
//can be read while it is running
short readTA0(void);

//setup timer A to run off 32.768kHz xtal
void init_timerA0(void);

//start timer A in continuous mode
void start_timerA0(void);

#endif
  