#include <msp430.h>
#include <ctl.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <ARCbus.h>
#include <stdlib.h>
#include <Error.h>
#include "timerA0.h"
#include <terminal.h>
#include <SDlib.h>
#include <UCA2_uart.h>
#include <i2c.h>
#include "LED.h"
#include "torquers.h"
#include "ACDS.h"
#include "stackcheck.h"
#include "sensor-interface.h"
#include "log.h"
#include "ACDSerr.h"
#include "pins.h"

CTL_TASK_t tasks[3];

//stacks for tasks
unsigned stack1[1+356+1];          
unsigned stack2[1+512+1];
unsigned stack3[1+300+1];   

//print stack bytes remaining
/*int stackCmd(char **argv,unsigned short agrc){
  unsigned *(stacks[])={stack1,stack2,stack3};
  size_t sizes[]={sizeof(stack1),sizeof(stack2),sizeof(stack3)};
  char *(names[])={"stack1","stack2","stack3"};
  int i,rem;
  for(i=0;i<3;i++){
    //get remaining bytes in stack
    rem=stackcheck_unused(stacks[i],sizes[i]);
    //check for overflow
    if(rem==-1){
      //print overflow message
      printf("Stack Overflow for %s!!\r\n",names[i]);
    }else{
      //print remaining bytes
      printf("%4i out of %4i bytes remaining in %s. %2.0f%%\r\n",rem,sizes[i]-2*sizeof(unsigned),names[i],100*(sizes[i]-2*sizeof(unsigned)-rem)/(float)sizes[i]);
    }
  }
  return 0;
}*/

//make printf and friends use async
int __putchar(int c){
  return UCA2_TxChar(c);
}

//make printf and friends use async
int __getchar(void){
  return UCA2_Getc();
}

int main(void){
  //DO this first
  ARC_setup(); 
  
  //====================[register error handler]====================
  err_register_handler(ACDS_ERROR_MIN,ACDS_ERROR_MAX,ACDS_err_decode,ERR_FLAGS_SUBSYSTEM);

  //==================[setup subsystem Peripherals]==================

  //setup mmc interface
  mmcInit_msp();

  //setup torquer driver pins
  driverInit();
  //setup comparitor pins
  torque_fb_init();
  
  //initialize logging 
  log_init();

  //setup LED's
  init_LED();
  
  //turn power LED
  PWR_LED_on();

  //initialize UART
  UCA2_init_UART(UART_PORT,UART_TX_PIN_NUM,UART_RX_PIN_NUM);

  //initialize sensor I2C
  initI2C(SEN_PORT,SEN_SDA_PIN_NUM,SEN_SCL_PIN_NUM);

  //initialize magnetometer S/R pin
  mag_init();

  //setup command parse for ACDS
  BUS_register_cmd_callback(&ACDS_parse);
  
  //TESTING: set log level to report everything by default
  set_error_level(0);

  //setup bus interface
  initARCbus(BUS_ADDR_ACDS);

  //initialize events for ACDS
  ctl_events_init(&ACDS_evt,0);

  //initialize stacks
  stackInit(stack1,sizeof(stack1));
  stackInit(stack2,sizeof(stack2));
  stackInit(stack3,sizeof(stack3));

  //create tasks
  ctl_task_run(&tasks[0],BUS_PRI_NORMAL,ACDS_events,NULL,"ACDS",sizeof(stack1)/sizeof(stack1[0])-2,stack1+1,0);
  ctl_task_run(&tasks[1],BUS_PRI_LOW,terminal,"ACDS Test Program","terminal",sizeof(stack2)/sizeof(stack2[0])-2,stack2+1,0);
  ctl_task_run(&tasks[2],BUS_PRI_HIGH,sub_events,NULL,"sub_events",sizeof(stack3)/sizeof(stack3[0])-2,stack3+1,0);
  
  //start timer A0
  start_timerA0();
  
  //seed random number generator
  //TODO: figure out which timer should be used
  srand(TA0R);
  
  mainLoop();
}

