#ifndef __TIMER_A_H
#define __TIMER_A_H

//use majority function so the timer
//can be read while it is running
short readTA(void);

//setup timer A to run off 32.768kHz xtal
void init_timerA(void);

//start timer A in continuous mode
void start_timerA(void);

#endif
  