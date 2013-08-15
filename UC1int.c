#include <msp430.h>
#include <ctl.h>
#include "UCA1_uart.h"

//buffers for UCA1 UART
struct UART_Tx UCA1_TxBuf;
struct UART_Rx UCA1_RxBuf;

//UART TX ISR called to transmit UART data
void UC1_TX(void) __ctl_interrupt[USCIAB1TX_VECTOR]{
  unsigned char flags=UC1IFG&(UC1IE);
//=============[UART Transmit Handler]===============
  if(flags&UCA1TXIFG){
    unsigned char c;
    if (ctl_byte_queue_receive_nb(&UCA1_TxBuf.queue,&c)==0){
      //buffer empty disable TX
      UCA1_TxBuf.done=1;
      UC1IFG&=~UCA1TXIFG;
    }else{
      //send char to UART
      UCA1TXBUF=c;
    }
  }
//==============================================================
}

// receive UART ISR
void UC1_rx(void) __ctl_interrupt[USCIAB1RX_VECTOR]{
  unsigned char flags=UC1IFG&(UC1IE);
//==============[UART Receive Handler]==========================
  if(flags&UCA1RXIFG){
    //read a byte from UART
    unsigned char c=UCA1RXBUF;
    //put byte in queue, if no room too darn bad
    ctl_byte_queue_post_nb(&UCA1_RxBuf.queue,c);
    //TODO: raise error if no room
  }
//==============================================================
}
