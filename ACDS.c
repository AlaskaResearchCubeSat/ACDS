#include <msp430.h>
#include <ctl.h>
#include <stdio.h>
#include <ARCbus.h>
#include <string.h>
#include <terminal.h>
#include <stdlib.h>
#include <math.h> //needed for NAN
#include "SensorDataInterface.h"
#include "ACDS.h"
#include "LED.h"
#include "vector.h"
#include "algorithm.h"
#include "torquers.h"
#include "ACDSerr.h"
#include <SDlib.h>
#include "corrections.h"
#include "log.h"

ACDS_STAT status;
    
typedef struct{
        CTL_MUTEX_t lock;
        int action;
        union {
            unsigned long sector;
        } parm;
    }SPI_DATA_ACTION;
    
SPI_DATA_ACTION spi_action;

void sub_events(void *p) __toplevel{
  unsigned int e,len;
  int i,resp;
  extern CTL_TASK_t tasks[3];
  unsigned short time=32768,count=0;
  unsigned char buf[BUS_I2C_HDR_LEN+sizeof(ACDS_STAT)+BUS_I2C_CRC_LEN],*ptr;
  for(;;){
    e=ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&SUB_events,SUB_EV_ALL|SUB_EV_ASYNC_OPEN|SUB_EV_ASYNC_CLOSE,CTL_TIMEOUT_NONE,0);
    if(e&SUB_EV_PWR_OFF){
        //print message
        puts("System Powering Down\r\n");
        //set ACDS mode
        ACDS_mode=ACDS_IDLE_MODE;
        //setup command
        ptr=BUS_cmd_init(buf,CMD_MAG_SAMPLE_CONFIG);
        //set command
        *ptr++=MAG_SAMPLE_STOP;
        //send packet
        resp=BUS_cmd_tx(BUS_ADDR_LEDL,buf,1,0,BUS_I2C_SEND_FOREGROUND);
        //check result
        if(resp<0){
            printf("Error communicating with LEDL : %s\r\n",BUS_error_str(resp));
        }
    }
    if(e&SUB_EV_PWR_ON){
        //print message
        puts("System Powering Up\r\n");
        //set init ACDS mode
        ACDS_mode=ACDS_INIT_MODE;
        //setup command
        ptr=BUS_cmd_init(buf,CMD_MAG_SAMPLE_CONFIG);
        //set command
        *ptr++=MAG_SAMPLE_START;
        //set time MSB
        *ptr++=time>>8;
        //set time LSB
        *ptr++=time;
        //set count
        *ptr++=count;
        //send packet
        resp=BUS_cmd_tx(BUS_ADDR_LEDL,buf,4,0,BUS_I2C_SEND_FOREGROUND);
        //check result
        if(resp<0){
            printf("Error communicating with LEDL : %s\r\n",BUS_error_str(resp));
        }
    }
    if(e&SUB_EV_SEND_STAT){
      //send status
      puts("Sending status\r\n");
      //setup packet 
      ptr=BUS_cmd_init(buf,CMD_ACDS_STAT);
      //fill in telemitry data
      for(i=0;i<sizeof(ACDS_STAT);i++){
        ptr[i]=((unsigned char*)(&status))[i];
      }
      //wait to avoid conflicts
      ctl_timeout_wait(ctl_get_current_time()+50);
      //send command
      resp=BUS_cmd_tx(BUS_ADDR_CDH,buf,sizeof(ACDS_STAT),0,BUS_I2C_SEND_FOREGROUND);
      if(resp!=RET_SUCCESS){
        printf("Failed to send status %s\r\n",BUS_error_str(resp));
        ERR_LED_on();
      }else{
        STAT_LED_toggle();
      }
    }
    if(e&SUB_EV_TIME_CHECK){
      printf("time ticker = %li\r\n",get_ticker_time());
    }
    if(e&SUB_EV_SPI_DAT){
      puts("SPI data recived:\r\n");
      if(!ctl_mutex_lock(&spi_action.lock,CTL_TIMEOUT_DELAY,2048)){
          puts("\tUnable to get SPI data action\r\n");
          //free buffer
          BUS_free_buffer_from_event();
          continue;
      }
      //get length
      len=arcBus_stat.spi_stat.len;
      switch(spi_action.action){
          case SPI_DAT_ACTION_INVALID:
            puts("\tInvalid SPI data action\r\n");
          break;
          case SPI_DAT_ACTION_SD_WRITE:
            //check packet length
            if(len%512!=0){
                printf("\tError : SD packet is not a whole number of sectors\r\n");
                break;
            }
            //calculate number of sectors
            len=len/512;
            if(len==1){
                resp=mmcWriteBlock(spi_action.parm.sector,(unsigned char*)arcBus_stat.spi_stat.rx);
            }else{
                resp=mmcWriteMultiBlock(spi_action.parm.sector,(unsigned char*)arcBus_stat.spi_stat.rx,len);
            }
            if(resp){
                printf("Unable to write SPI data to SD card : %s\r\n",SD_error_str(resp));
            }
          break;
          default:
            printf("\tUnknown SPI data action %i\r\n",spi_action.action);
            //print out data
            for(i=0;i<len;i++){
                //printf("0x%02X ",rx[i]);
                printf("%03i ",arcBus_stat.spi_stat.rx[i]);
            }
            printf("\r\n");
          break;
        }
      //clear action
      spi_action.action=SPI_DAT_ACTION_INVALID;
      //unlock action
      ctl_mutex_unlock(&spi_action.lock);
      //free buffer
      BUS_free_buffer_from_event();
    }
    if(e&SUB_EV_SPI_ERR_CRC){
      puts("SPI bad CRC\r\n");
    }
    /*if(e&SUB_EV_ASYNC_OPEN){
      extern unsigned char async_addr;
      unsigned *stack2;
      //setup closed event
      async_setup_close_event(&SUB_events,SUB_EV_ASYNC_CLOSE);
      //print message
      printf("Async Opened from 0x%02X\r\n",async_addr);
      //setup UART terminal        
      ctl_task_run(&tasks[1],BUS_PRI_NORMAL,terminal,"ACDS Test Program ready","terminal",sizeof(stack2)/sizeof(stack2[0])-2,stack2+1,0);
      //async_close();
    }
    if(e&SUB_EV_ASYNC_CLOSE){
      //kill off async terminal
      ctl_task_remove(&tasks[1]);
    }*/
  }
}

MAG_DAT magData;


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
  unsigned long sector;
  switch(cmd){
    case CMD_MAG_DATA:
      //check packet length
      if(len!=sizeof(magData)){
        //length incorrect, report error and exit
        report_error(ERR_LEV_ERROR,ACDS_ERR_SRC_SENSORS,ACDS_ERR_SEN_BAD_PACKET_LENGTH,len);
        return ERR_PK_LEN;
      }
      memcpy(&magData,dat,sizeof(magData));
      //sensor data recieved set event
      ctl_events_set_clear(&ACDS_evt,ACDS_EVT_DAT_REC,0);
    return RET_SUCCESS;
    case CMD_SPI_DATA_ACTION:
        if(len==0){
            return ERR_PK_LEN;
        }
        switch(dat[0]){
            case SPI_DAT_ACTION_SD_WRITE:
                if(len!=5){
                    return ERR_PK_LEN;
                }
                sector=((unsigned long)dat[1]<<24)|((unsigned long)dat[2]<<16)|((unsigned long)dat[3]<<8)|((unsigned long)dat[4]);
                //get lock on action
                if(!ctl_mutex_lock(&spi_action.lock,CTL_TIMEOUT_DELAY,10)){
                    return ERR_SPI_BUSY;
                }
                spi_action.action=dat[0];
                spi_action.parm.sector=sector;
                ctl_mutex_unlock(&spi_action.lock);
            break;
            default:
                return ERR_UNKNOWN_CMD;
        }
    return RET_SUCCESS;
  }
  //Return Error
  return ERR_UNKNOWN_CMD;
}


int ACDS_mode=ACDS_HOLD_MODE;



void ACDS_events(void *p) __toplevel{
  unsigned int e;
  int i,xnum,ynum,znum;
  const VEC zero={0,0,0};
  VEC Flux,mag;
  CPOINT pt;
  //initialize status
  memset(status.mag,0,sizeof(status.mag));
  memset(status.tqstat,0,sizeof(status.tqstat));
  memset(status.flips,0,sizeof(status.flips));
  iquat_zero(&status.attitude);
  ivec_zero(&status.rates);
  //init event
  ctl_events_init(&ACDS_evt,0);
  //check correction data status
  read_cor_stat();
  //setup data logging
  log_start();
  //endless loop
  for(;;){
    //wait for events
    e=ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&ACDS_evt,ACDS_EVT_ALL,CTL_TIMEOUT_NONE,0);
    //magnetometer data event
    if(e&ACDS_EVT_DAT_REC){
      //clear flux
      Flux.c.x=Flux.c.y=Flux.c.z=0;
      //clear axes sample number
      xnum=ynum=znum=0;
      //apply correction
      for(i=0;i<6;i++){
          //check that data was read and correction is valid
          if(magData.flags&(1<<(i*2)) && magData.flags&(1<<(i*2+1)) && cor_stat&(1<<i)){
              //apply correction
              applyCor(&pt,&magData.meas[i],i);
              switch(i){
                  //X+ axis
                  case MAG_X_PLUS_IDX:
                    Flux.c.y+=pt.c.a;
                    ynum++;
                    Flux.c.z-=pt.c.b;
                    znum++;
                  break;
                  //X- axis
                  case MAG_X_MINUS_IDX:
                    Flux.c.y-=pt.c.a;
                    ynum++;
                    Flux.c.z-=pt.c.b;
                    znum++;
                  break;
                  //Y+ axis
                  case MAG_Y_PLUS_IDX:
                    Flux.c.x-=pt.c.a;
                    xnum++;
                    Flux.c.z-=pt.c.b;
                    znum++;
                    break;
                  //Y- axis
                  case MAG_Y_MINUS_IDX:
                    Flux.c.x+=pt.c.a;
                    xnum++;
                    Flux.c.z-=pt.c.b;
                    znum++;
                  break;
                  //Z+ axis
                  case MAG_Z_PLUS_IDX:
                    Flux.c.y+=pt.c.a;
                    xnum++;
                    Flux.c.x+=pt.c.b;
                    ynum++;
                  break;
                  //Z- axis
                  case MAG_Z_MINUS_IDX:
                    Flux.c.x-=pt.c.a;
                    xnum++;
                    Flux.c.y+=pt.c.b;
                    ynum++;
                  break;
              }
          }
      }
      //calculate average
      Flux.c.x=xnum>0?Flux.c.x/xnum:__float32_nan;
      Flux.c.y=ynum>0?Flux.c.y/ynum:__float32_nan;
      Flux.c.z=znum>0?Flux.c.z/znum:__float32_nan;
      //set flux in status packet
      status.mag[0]=32767/2*Flux.elm[0];
      status.mag[1]=32767/2*Flux.elm[1];
      status.mag[2]=32767/2*Flux.elm[2];
      //set flux vector
      vec_cp(&acds_dat.dat.acds_dat.flux,&Flux);      
      //set mode
      acds_dat.dat.acds_dat.mode=ACDS_mode;
      //do things based on mode
      switch(ACDS_mode){
        case ACDS_MODE_1:
            //print flux vector
            //vecPrint("Flux",&Flux);
            //run B-dot algorithm
            bdot(&Flux,32768);  
            //print out new torquer status
            //printf("New Torquer Status:\r\n");
            //print_torquer_status();
        break;
        case ACDS_MODE_2:
        break;
        case ACDS_MODE_3:
        break;
        case ACDS_HOLD_MODE:
          //flip torquers
          setTorque(&zero);
        break;
        case ACDS_COMMAND_MODE:
            /*for(i=0;i<6;i++){
                if(magData.flags&(1<<(i*2)) && magData.flags&(1<<(i*2+1))){
                    if(cor_stat&(1<<i)){
                        //apply correction
                        applyCor(&pt,&magData.meas[i],i);
                        printf("%i : %f %f\r\n",i,pt.c.a,pt.c.b);
                    }else{
                        //print error
                        printf("%i : --- ---\r\n",i);
                    }
                }else{
                    //print error
                    printf("%i : ### ###\r\n",i);
                }
            }*/
            //printf("Fux = %f %f %f\r\n",Flux.c.x,Flux.c.y,Flux.c.z);
        break;
        case ACDS_IDLE_MODE:
            //do nothing in idle mode
        break;
        case ACDS_INIT_MODE:
            //check if torquers are initialized
            if(!checkTorqueInit){
                //flip torquers
                setTorque(&zero);  
            }else{
                //switch to mode 1
                ACDS_mode=ACDS_MODE_1;
            }
        break;
                
      }
      //save status
      tqstat2stat(status.tqstat);
      ctl_events_set_clear(&ACDS_evt,ADCS_EVD_COMMAND_SENSOR_READ,0);
      //write log data
      log_store_data(&acds_dat);
    }
  }
}
