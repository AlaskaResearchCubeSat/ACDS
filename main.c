#include <msp430.h>
#include <ctl_api.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <ARCbus.h>
#include "UCA1_uart.h"
#include "timerA.h"
#include "terminal.h"

CTL_TASK_t tasks[3];

//stacks for tasks
unsigned stack1[1+256+1];          
unsigned stack2[1+512+1];
unsigned stack3[1+64+1];   

CTL_EVENT_SET_t cmd_parse_evt;

unsigned char buffer[80];

//handle subsystem specific commands
int SUB_parseCmd(unsigned char src,unsigned char cmd,unsigned char *dat,unsigned short len){
  int i;
  switch(cmd){
    //Handle Print String Command
    case 6:
      //check packet length
      if(len>sizeof(buffer)){
        //return error
        return ERR_PK_LEN;
      }
      //copy to temporary buffer
      for(i=0;i<len;i++){
        buffer[i]=dat[i];
      }
      //terminate string
      buffer[i]=0;
      //set event
      ctl_events_set_clear(&cmd_parse_evt,0x01,0);
      //Return Success
      return RET_SUCCESS;
  }
  //Return Error
  return ERR_UNKNOWN_CMD;
}

void cmd_parse(void *p) __toplevel{
  unsigned int e;
  //init event
  ctl_events_init(&cmd_parse_evt,0);
  for(;;){
    e=ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&cmd_parse_evt,0x01,CTL_TIMEOUT_NONE,0);
    if(e&0x01){
      //print message
      printf("%s\r\n",buffer);
    }
  }
}


void sub_events(void *p) __toplevel{
  unsigned int e,len;
  int i;
  unsigned char buf[10],*ptr;
  for(;;){
    e=ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&SUB_events,SUB_EV_ALL,CTL_TIMEOUT_NONE,0);
    if(e&SUB_EV_PWR_OFF){
      //print message
      puts("System Powering Down\r");
    }
    if(e&SUB_EV_PWR_ON){
      //print message
      puts("System Powering Up\r");
    }
    if(e&SUB_EV_SEND_STAT){
      //send status
      //puts("Sending status\r");
      //setup packet 
      //TODO: put actual command for subsystem response
      ptr=BUS_cmd_init(buf,20);
      //TODO: fill in telemitry data
      //send command
      BUS_cmd_tx(BUS_ADDR_CDH,buf,0,0,SEND_FOREGROUND);
    }
    if(e&SUB_EV_TIME_CHECK){
      printf("time ticker = %li\r\n",get_ticker_time());
    }
    if(e&SUB_EV_SPI_DAT){
      puts("SPI data recived:\r");
      //get length
      len=arcBus_stat.spi_stat.len;
      //print out data
      for(i=0;i<len;i++){
        //printf("0x%02X ",rx[i]);
        printf("%03i ",arcBus_stat.spi_stat.rx[i]);
      }
      printf("\r\n");
    }
    if(e&SUB_EV_SPI_ERR_CRC){
      puts("SPI bad CRC\r");
    }
  }
}

int main(void){
  //DO this first
  ARC_setup(); 
  
  //setup system specific peripherals

  //setup mmc interface
  //mmcInit_msp();
 
 
  //setup UCA1 UART
  init_UCA1_UART();
  
  //setup P7 for LED's
  P7OUT=0x00;
  P7DIR=0xFF;

  //setup P8 for output
  P8OUT=0x00;
  P8DIR=0xFF;
  P8SEL=0x00;
  
  //setup bus interface
  initARCbus(*((char*)0x01000));
  
  //print informative initial message
  printf("\rARC Bus Test Program\r\n");

  //print out I2C address
  printf("I2C address = 0x%02X\r\n",*(unsigned char*)0x01000);

  //initialize stacks
  memset(stack1,0xcd,sizeof(stack1));  // write known values into the stack
  stack1[0]=stack1[sizeof(stack1)/sizeof(stack1[0])-1]=0xfeed; // put marker values at the words before/after the stack
  
  memset(stack2,0xcd,sizeof(stack2));  // write known values into the stack
  stack2[0]=stack2[sizeof(stack2)/sizeof(stack2[0])-1]=0xfeed; // put marker values at the words before/after the stack
    
  memset(stack3,0xcd,sizeof(stack3));  // write known values into the stack
  stack3[0]=stack3[sizeof(stack3)/sizeof(stack3[0])-1]=0xfeed; // put marker values at the words before/after the stack

  //create tasks
  ctl_task_run(&tasks[0],1,cmd_parse,NULL,"cmd_parse",sizeof(stack1)/sizeof(stack1[0])-2,stack1+1,0);
  ctl_task_run(&tasks[1],2,terminal,NULL,"terminal",sizeof(stack2)/sizeof(stack2[0])-2,stack2+1,0);
  ctl_task_run(&tasks[2],10,sub_events,NULL,"sub_events",sizeof(stack3)/sizeof(stack3[0])-2,stack3+1,0);
  
  mainLoop();
}

