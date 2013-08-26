#include <msp430.h>
#include <ctl.h>
#include <stdio.h>
#include <ARCbus.h>
#include <string.h>
#include <terminal.h>
#include "SensorDataInterface.h"
#include "ACDS.h"
#include "LED.h"
#include "vector.h"
#include "algorithm.h"
#include "torquers.h"


//spesifications for the terminal
const TERM_SPEC async_term={"ACDS Test Program ready",async_Getc};

void sub_events(void *p) __toplevel{
  unsigned int e,len;
  int i;
  extern CTL_TASK_t tasks[3];
  unsigned char buf[10],*ptr;
  for(;;){
    e=ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&SUB_events,SUB_EV_ALL|SUB_EV_ASYNC_OPEN|SUB_EV_ASYNC_CLOSE,CTL_TIMEOUT_NONE,0);
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
      BUS_cmd_tx(BUS_ADDR_CDH,buf,0,0,BUS_I2C_SEND_FOREGROUND);
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
      //free buffer
      BUS_free_buffer_from_event();
    }
    if(e&SUB_EV_SPI_ERR_CRC){
      puts("SPI bad CRC\r");
    }
    if(e&SUB_EV_SPI_ERR_CRC){
      //puts("SPI bad CRC\r");
      P7OUT|=BIT7;
    }
    /*if(e&SUB_EV_ASYNC_OPEN){
      extern unsigned char async_addr;
      unsigned *stack2;
      //setup closed event
      async_setup_close_event(&SUB_events,SUB_EV_ASYNC_CLOSE);
      //print message
      printf("Async Opened from 0x%02X\r\n",async_addr);
      //setup UART terminal        
      ctl_task_run(&tasks[1],BUS_PRI_NORMAL,terminal,(void*)&async_term,"terminal",sizeof(stack2)/sizeof(stack2[0])-2,stack2+1,0);
      //async_close();
    }
    if(e&SUB_EV_ASYNC_CLOSE){
      //kill off async terminal
      ctl_task_remove(&tasks[1]);
    }*/
  }
}

long magData[2];

//convert ADC value to voltage
float ADCtoV(long adc){
  //TODO: maybe allow other references
  return adc*3.3/(2*65535.0);
}

  
  //gain of magnetomitor amplifier
  //#define AMP_GAIN    (2.49e6/5.1e3)    // V/V0
  #define AMP_GAIN    (64)    // V/V
  //#define AMP_GAIN    (5.11e6/5.1e3)    // V/V
  //sensitivity of magnetomitor
  #define MAG_SENS    (1e-3)            // mV/V/Gauss

//compute ADC value to magnetic field value in gauss
float ADCtoGauss(long adc){
  return  ADCtoV(adc)/(AMP_GAIN*MAG_SENS);
}


//convert returned data from 16bit LTC24xx ADC into a signed long integer
long adc16Val(unsigned char *dat){
  long val;
  short sig,msb;
  //extract magnitude bits from data
  //val=(((unsigned long)dat[0])<<(16-6))|(((unsigned long)dat[1])<<(8-6))|((unsigned long)dat[2]>>6);
  val=(((unsigned long)dat[0])<<16)|(((unsigned long)dat[1])<<8)|((unsigned long)dat[2]);
  val>>=6;
  //check sign bit
  sig=!!(val&(0x20000));
  //check MSB bit
  msb=!!(val&(0x10000));
  //remove MSB and sig bits
  val&=~0x30000;
  //check for negative values
  if(!sig){
    val|=0xFFFF0000;
  }
  //check for positive overflow
  if(msb && sig && val!=0){
    return 65536;
  }

  //check for negative overflow
  if(!msb && !sig && val!=0){
    return -65536;
  }


  return val;
}


CTL_EVENT_SET_t ACDS_evt;

//handle ACDS specific commands
int SUB_parseCmd(unsigned char src,unsigned char cmd,unsigned char *dat,unsigned short len){
  int i;
  switch(cmd){
    case CMD_ACDS_STAT:
      //need to send status set event
      ctl_events_set_clear(&ACDS_evt,ACDS_EVT_SEND_STAT,0);
    return RET_SUCCESS;
    case CMD_MAG_DATA:
      memcpy(magData,dat,sizeof(magData));
      //sensor data recieved set event
      ctl_events_set_clear(&ACDS_evt,ACDS_EVT_DAT_REC,0);
    return RET_SUCCESS;
  }
  //Return Error
  return ERR_UNKNOWN_CMD;
}

//calibration data for X and Y axis
float calX[3]={1.178493850E-004,5.062495770E-006,-7.474086836E-001},calY[3]={2.192687380E-006,1.180955870E-004,1.009666236E-001};

float applyCal(long *raw,float *cal){
  float val;
  val=raw[0]*cal[0]+raw[1]*cal[1]+cal[2];
  return val;
}

int ACDS_mode=ACDS_HOLD_MODE;

int cal_stat=0;

//return index from torquer status
int stat2idx(int stat){
  //const int tbl[4]={2,3,1,0};
  const int tbl[4]={3,0,2,1};
  //check stat so that array bounds are not exceeded
  if(stat>=4 || stat<0){
    //return -1 in case of error
    return -1;
  }
  return tbl[stat];
}

//get index for the current torquer status in the calibration table
int calIdx(void){
  int idx,xidx,yidx,zidx;
  extern TQ_SET tq_big;
  //get index from status values
  xidx=stat2idx(tq_big.c.x.status);
  yidx=stat2idx(tq_big.c.y.status);
  zidx=stat2idx(tq_big.c.z.status);
  //check for error
  if(xidx==-1 || yidx==-1 || zidx==-1){
    //error, return invalid
    return -1;
  }
  idx=xidx+4*yidx+16*zidx;
  return idx;
}

//table of torquer flips to cover all possible torquer states
  //assume torquers have been initialized
  const struct{
    int num[3],dir[3];
  } flipTable[64]={
                  //num    dir
                  {{2,0,0},{M_PLUS, 0      ,0      }},//1
                  {{1,0,0},{M_MINUS,0      ,0      }},//2
                  {{2,0,0},{M_MINUS,0      ,0      }},//3
                  {{1,2,0},{M_PLUS ,M_PLUS ,0      }},//4
                          
                  {{2,0,0},{M_PLUS ,0      ,0      }},//5
                  {{1,0,0},{M_MINUS,0      ,0      }},//6
                  {{2,0,0},{M_MINUS,0      ,0      }},//7
                  {{1,1,0},{M_PLUS ,M_MINUS,0      }},//8
                                  
                  {{2,0,0},{M_PLUS ,0      ,0      }},//9
                  {{1,0,0},{M_MINUS,0      ,0      }},//10
                  {{2,0,0},{M_MINUS,0      ,0      }},//11
                  {{1,2,0},{M_PLUS ,M_MINUS,0      }},//12
                                          
                  {{2,0,0},{M_PLUS ,0      ,0      }},//13
                  {{1,0,0},{M_MINUS,0      ,0      }},//14
                  {{2,0,0},{M_MINUS,0      ,0      }},//14
                  {{1,1,2},{M_PLUS ,M_PLUS ,M_PLUS }},//15
                  
                  {{2,0,0},{M_PLUS, 0      ,0      }},//16
                  {{1,0,0},{M_MINUS,0      ,0      }},//17
                  {{2,0,0},{M_MINUS,0      ,0      }},//18
                  {{1,2,0},{M_PLUS ,M_PLUS ,0      }},//19
                          
                  {{2,0,0},{M_PLUS ,0      ,0      }},//20
                  {{1,0,0},{M_MINUS,0      ,0      }},//21
                  {{2,0,0},{M_MINUS,0      ,0      }},//22
                  {{1,1,0},{M_PLUS ,M_MINUS,0      }},//23
                                  
                  {{2,0,0},{M_PLUS ,0      ,0      }},//24
                  {{1,0,0},{M_MINUS,0      ,0      }},//25
                  {{2,0,0},{M_MINUS,0      ,0      }},//26
                  {{1,2,0},{M_PLUS ,M_MINUS,0      }},//27
                                          
                  {{2,0,0},{M_PLUS ,0      ,0      }},//28
                  {{1,0,0},{M_MINUS,0      ,0      }},//29
                  {{2,0,0},{M_MINUS,0      ,0      }},//30
                  {{1,1,1},{M_PLUS ,M_PLUS ,M_MINUS}},//31
                  
                  {{2,0,0},{M_PLUS, 0      ,0      }},//32
                  {{1,0,0},{M_MINUS,0      ,0      }},//33
                  {{2,0,0},{M_MINUS,0      ,0      }},//34
                  {{1,2,0},{M_PLUS ,M_PLUS ,0      }},//35
                          
                  {{2,0,0},{M_PLUS ,0      ,0      }},//36
                  {{1,0,0},{M_MINUS,0      ,0      }},//37
                  {{2,0,0},{M_MINUS,0      ,0      }},//38
                  {{1,1,0},{M_PLUS ,M_MINUS,0      }},//39
                                  
                  {{2,0,0},{M_PLUS ,0      ,0      }},//40
                  {{1,0,0},{M_MINUS,0      ,0      }},//41
                  {{2,0,0},{M_MINUS,0      ,0      }},//42
                  {{1,2,0},{M_PLUS ,M_MINUS,0      }},//43
                                          
                  {{2,0,0},{M_PLUS ,0      ,0      }},//44
                  {{1,0,0},{M_MINUS,0      ,0      }},//45
                  {{2,0,0},{M_MINUS,0      ,0      }},//46
                  {{1,1,2},{M_PLUS ,M_PLUS ,M_MINUS}},//47

                  
                  {{2,0,0},{M_PLUS, 0      ,0      }},//48
                  {{1,0,0},{M_MINUS,0      ,0      }},//49
                  {{2,0,0},{M_MINUS,0      ,0      }},//50
                  {{1,2,0},{M_PLUS ,M_PLUS ,0      }},//51
                          
                  {{2,0,0},{M_PLUS ,0      ,0      }},//52
                  {{1,0,0},{M_MINUS,0      ,0      }},//53
                  {{2,0,0},{M_MINUS,0      ,0      }},//54
                  {{1,1,0},{M_PLUS ,M_MINUS,0      }},//55
                                  
                  {{2,0,0},{M_PLUS ,0      ,0      }},//56
                  {{1,0,0},{M_MINUS,0      ,0      }},//57
                  {{2,0,0},{M_MINUS,0      ,0      }},//58
                  {{1,2,0},{M_PLUS ,M_MINUS,0      }},//59
                                          
                  {{2,0,0},{M_PLUS ,0      ,0      }},//60
                  {{1,0,0},{M_MINUS,0      ,0      }},//61
                  {{2,0,0},{M_MINUS,0      ,0      }},//62
                  {{1,1,1},{M_PLUS ,M_PLUS ,M_PLUS }},//63
                  };
 
typedef VEC_INT CAL_TBL[64];
CAL_TBL tmpCal;

calibrate(long *dat){
  int idx;
  //get index for table
  idx=calIdx();
  //print out reading
  printf("% 4i\t% 4i\t% 4i\t",dat[0],dat[1],0);
  //check index
  if(idx<0){
    printf("Error illegal index %i in iteration %i\r\n",idx,cal_stat);
  }else{
    tmpCal[idx].c.x=dat[0];
    tmpCal[idx].c.y=dat[1];
    tmpCal[idx].c.z=0;
  }
  print_torquer_stat_code(TQ_SET_BIG);
  //flip torquers
  drive_torquers(TQ_SET_BIG,flipTable[cal_stat].num,flipTable[cal_stat].dir);
  //increment counter
  cal_stat++;
  printf("\r\n");
  //check if calibration is complete
  if(cal_stat>=64){
    //put ACDS into hold mode
    ACDS_mode=ACDS_HOLD_MODE;
    //reset cal_stat
    cal_stat=0;
    //set event to say that it is complete
    ctl_events_set_clear(&ACDS_evt,ADCS_EVT_CAL_COMPLETE,0);
  }
}

void ACDS_events(void *p) __toplevel{
  unsigned int e;
  const VEC zero={0,0,0};
  VEC Flux;
  //init event
  ctl_events_init(&ACDS_evt,0);
  for(;;){
    //wait for events
    e=ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&ACDS_evt,ACDS_EVT_ALL,CTL_TIMEOUT_NONE,0);
    //status event
    if(e&ACDS_EVT_SEND_STAT){
      //TODO: send status
      printf("Fixme: send status info\r\n");
    }
    //magnetometer data event
    if(e&ACDS_EVT_DAT_REC){
      //calculate flux Vector
      Flux.c.x=applyCal(magData,calX);
      Flux.c.y=applyCal(magData,calY);
      Flux.c.z=0;
      switch(ACDS_mode){
        case ACDS_MODE_1:
          //run B-dot algorithm
          bdot(&Flux,32768);
        break;
        case ACDS_MODE_2:
        break;
        case ACDS_MODE_3:
        break;
        case ACDS_CAL_MODE:
          calibrate(magData);
        break;
        case ACDS_HOLD_MODE:
          //flip torquers
          setTorque(&zero,TQ_SET_BIG);
        break;
      }
    }
  }
}
