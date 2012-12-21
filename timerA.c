#include <ctl_api.h>
#include <msp430.h>

//use majority function so the timer
//can be read while it is running
short readTA(void){
  int a=TAR,b=TAR,c=TAR;
  return (a&b)|(a&c)|(b&c);
}
