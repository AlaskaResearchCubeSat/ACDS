#include <ctl.h>
#include <msp430.h>

//read the timer until the same result is read twice
//this is so the timer can be read while it is running
short readTA0(void){
    short last,next;
    int count=0;
    //capture TAR
    last=TA0R;
    do{
        next=TA0R;
        count++;
    }while(next!=last || count>5);
    //return value
    return next;
}


//setup timer A to run off 32.768kHz xtal
void init_timerA0(void){
  TA0CTL=TASSEL__ACLK|TACLR;
}

//start timer A in continuous mode
void start_timerA0(void){
  TA0CTL|=MC__CONTINUOUS;
}
