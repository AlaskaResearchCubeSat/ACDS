
#include <msp430.h>
#include <string.h>
#include <stdio.h>
#include <ctl_api.h>
#include "UCA1_uart.h"

void init_UCA1_UART(void){
 //init queues
  ctl_byte_queue_init(&UCA1_TxBuf.queue,UCA1_TxBuf.buf,UCA1_TX_SIZE);
  ctl_byte_queue_init(&UCA1_RxBuf.queue,UCA1_RxBuf.buf,UCA1_RX_SIZE);
  UCA1_TxBuf.done=0;
   //setup UCA1 for USB-UART operation
  UCA1CTL1=UCSWRST;
  UCA1CTL0=0;
  UCA1CTL1|=UCSSEL_1;
  //UCA1CTL1|=UCSSEL_2;
  
  //set baud rate to 9600
  UCA1BR0=3;
  UCA1BR1=0;
  UCA1MCTL=UCBRS_3;
  
  //set baud rate to 38400
  //UCA1BR0=26;
  //UCA1BR1=0;
  //UCA1MCTL=UCBRF_1|UCOS16;
  
  //set baud rate to 57600
  //UCA1BR0=17;
  //UCA1BR1=0;
  //UCA1MCTL=UCBRF_6|UCBRS_0|UCOS16;
  //setup pins
  P3SEL|=BIT6|BIT7;
  //take UCA1 out of reset mode
  UCA1CTL1&=~UCSWRST;
  //enable interrupts
  UC1IE|=UCA1TXIE|UCA1RXIE;
}

//queue byte to get transmitted
int UCA1_TxChar(unsigned char c){
  unsigned int t;
  int res=c;
  //disable interrupt
  int en=ctl_global_interrupts_disable();
  //check if transmitting
  if(UCA1_TxBuf.done){
    //bypass queue for first byte if not transmitting
    UCA1TXBUF=c;
    //clear done flag
    UCA1_TxBuf.done=0;
  //queue byte
  }else{
    ctl_byte_queue_post(&UCA1_TxBuf.queue,c,CTL_TIMEOUT_NONE,0);
  }
  //enable interrupt
  if(en){
    ctl_global_interrupts_enable();
  }
  //return result
  return res;
}

//get byte from buffer
int UCA1_Getc(void){
  unsigned char c;
  //recive a byte from the queue
  //TODO: posibly add timeout and the posibility to return EOF
  ctl_byte_queue_receive(&UCA1_RxBuf.queue,&c,CTL_TIMEOUT_NONE,0);
  //return byte from queue
  return c;
}

int UCA1_CheckKey(void){
  unsigned char c;
  if(ctl_byte_queue_receive_nb(&UCA1_RxBuf.queue,&c)){
    return c;
  }else{
    return EOF;
  }
}

