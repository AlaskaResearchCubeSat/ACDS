#ifndef __UCA1_UART_H
#define __UCA1_UART_H

#include <ctl.h>

//define size for RX and TX buffers
#define UCA1_RX_SIZE   (8)
#define UCA1_TX_SIZE   (1024)

  //TX buffer type
  struct Tx{
    CTL_BYTE_QUEUE_t queue;
    char done;
    unsigned char buf[UCA1_TX_SIZE];
  };
  
  //RX buffer type
  struct Rx{
    CTL_BYTE_QUEUE_t queue;
    unsigned char buf[UCA1_RX_SIZE];
  };

  //queue a byte to be transfered over the UART
  int UCA1_TxChar(unsigned char c);
  //Get a byte from the receive queue
  int UCA1_Getc(void);
  //initialize UCA0 for UART Usage
  void init_UCA1_UART(void);
  //get character from port without blocking
  int UCA1_CheckKey(void);
  //check if UART is transmitting or receiving
  #define UCA1_CheckBusy()  (UCA1STAT&UCBUSY)

  //buffer structures
  extern struct Tx UCA1_TxBuf;
  extern struct Rx UCA1_RxBuf;

#endif
  
