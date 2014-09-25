#include <ctl_api.h>
#include <msp430.h>

//read the timer until the same result is read twice
//this is so the timer can be read while it is running
short readTA(void){
    short last,next;
    int count=0;
    //capture TAR
    last=TAR;
    do{
        next=TAR;
        count++;
    }while(next!=last || count>5);
    //return value
    return next;
}
