#ifndef __STACKCHECK_H
#define __STACKCHECK_H

  #define STACK_GUARD     0xfeed
  #define STACK_INIT_VAL  0xCD

  //initialize stack memory so that stack size can be tracked
  void stackInit(unsigned *stack,size_t size);
  //check guard words on stack for overflow
  int stackcheck_guard(unsigned *stack,size_t size);
  
  //return the number of bytes left in the stack
  int stackcheck_unused(unsigned *stack,size_t size);

#endif
  