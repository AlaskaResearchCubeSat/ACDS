#include <string.h>
#include "stackcheck.h"

//initialize stack memory so that stack size can be tracked
void stackInit(unsigned *stack,size_t size){
  memset(stack,STACK_INIT_VAL,size);  // write known values into the stack
  // put marker values at the words before/after the stack
  stack[0]=STACK_GUARD;                          //before stack
  stack[size/sizeof(unsigned)-1]=STACK_GUARD;     //after stack
}

//check guard words on stack for overflow
int stackcheck_guard(unsigned *stack,size_t size){
  if(stack[0]!=STACK_GUARD || stack[size/sizeof(unsigned)-1]!=STACK_GUARD){
    //stack overflow
    return 1;
  }
  //no detected stack overflow
  return 0;
}

//return the number of bytes left in the stack
int stackcheck_unused(unsigned *stack,size_t size){
  unsigned char *p;
  int i;
  //check the end guard byte
  if(stack[0]!=STACK_GUARD){
    //stack overflow!
    return -1;
  }
  //start counting from the last byte in the stack
  p=(unsigned char*)&stack[1];
  for(i=0;i<size-sizeof(unsigned)*2;i++){
    //check if value is initialized value
     if(p[i]!=STACK_INIT_VAL){
        //value has changed
        break;
     }
  }
  //return result
  return i;
}
