#include <msp430.h>
#include <ctl.h>
#include <stdio.h>
#include <ARCbus.h>
#include <string.h>
#include <terminal.h>
#include <limits.h>
#include <stdlib.h>
#include <timerA0.h>
#include <math.h> //needed for NAN
#include "SensorDataInterface.h"
#include "ACDS.h"
#include "LED.h"
#include "vector.h"
#include "algorithm.h"
#include "torquers.h"
#include "ACDSerr.h"
#include <SDlib.h>
#include <crc.h>
#include "corrections.h"
#include "log.h"
    
typedef struct{
        CTL_MUTEX_t lock;
        int action;
        union {
            unsigned long sector;
        } parm;
    }SPI_DATA_ACTION;
    
SPI_DATA_ACTION spi_action;
    
float setpoint_dat[3];
unsigned char gain_dat[1+3*sizeof(float)];


//count and period to determine mag timeout
MAG_TIME mag_time;

SD_block_addr SD_read_addr;
   
//Convert magnetometer values to integers 
//Use smallest value 
int int_mag(SCL val){
    //check for NAN
    if(isnan(val)){
        //return INT_MIN
        return INT_MIN;
    }
    //scale floating point value
    val*=32767/2.0;
    //check for positave overflow
    if(val>INT_MAX){
        //return int max
        return INT_MAX;
    }    
    //check for negitave overflow
    if(val<(INT_MIN+1)){
        //return minimum value
        return (INT_MIN+1);
    }
    //return scaled value as an integer
    return ((int)val);
}

void make_status(ACDS_STAT *dest){    
  //get torquer status
  tqstat2stat(dest->tqstat);
  //get magnetic flux and convert to integer flux
  dest->mag[0]=int_mag(acds_dat.dat.acds_dat.flux.elm[0]);
  dest->mag[1]=int_mag(acds_dat.dat.acds_dat.flux.elm[1]);
  dest->mag[2]=int_mag(acds_dat.dat.acds_dat.flux.elm[2]);
  //copy mode
  dest->mode=acds_dat.dat.acds_dat.mode;
  //copy flips
  dest->flips[0]=acds_dat.dat.acds_dat.flips[0];
  dest->flips[1]=acds_dat.dat.acds_dat.flips[1];
  dest->flips[2]=acds_dat.dat.acds_dat.flips[2];
  //set attitude
  //TODO: figure out what to do here
  iquat_zero(&dest->attitude);
  //set rates from gyro
  ivec_cp(&dest->rates,&acds_dat.dat.acds_dat.gyro);
}

int mag_sample_stop(void* buf){
    unsigned char *ptr;
    //set ACDS mode
    ACDS_mode=ACDS_IDLE_MODE;
    //setup command
    ptr=BUS_cmd_init(buf,CMD_MAG_SAMPLE_CONFIG);
    //set command
    *ptr++=MAG_SAMPLE_STOP;
    //stop LEDL timeout
    mag_timeout_stop();
    //send packet
    return BUS_cmd_tx(BUS_ADDR_LEDL,buf,1,0);
}

int mag_sample_start(void* buf,unsigned short time,unsigned char count){
    unsigned char *ptr;
    int resp;
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
    //set timeout structure
    mag_time.T=time;
    mag_time.n=count+1;
    //send packet
    resp=BUS_cmd_tx(BUS_ADDR_LEDL,buf,4,0);
    //if command was sent successfully then start timeout timer
    if(resp==RET_SUCCESS){
        //restart timeout timer
        mag_timeout_reset();
    }
    //return response
    return resp;
}

int mag_sample_single(void* buf){
    unsigned char *ptr;
    int resp;
    //setup command
    ptr=BUS_cmd_init(buf,CMD_MAG_SAMPLE_CONFIG);
    //set command
    *ptr++=MAG_SINGLE_SAMPLE;
    //send packet
    resp=BUS_cmd_tx(BUS_ADDR_LEDL,buf,1,0);
    //if command was sent successfully then start timeout timer
    if(resp==RET_SUCCESS){
        //restart timeout timer
        mag_timeout_reset();
    }
    //return response
    return resp;
}

void sub_events(void *p) __toplevel{
  unsigned int e,len;
  int i,resp;
  extern CTL_TASK_t tasks[3];
  ACDS_STAT status;
  unsigned char buf[BUS_I2C_HDR_LEN+sizeof(ACDS_STAT)+BUS_I2C_CRC_LEN],*ptr;
  for(;;){
    e=ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&SUB_events,SUB_EV_ALL|SUB_EV_ASYNC_OPEN|SUB_EV_ASYNC_CLOSE,CTL_TIMEOUT_NONE,0);
    if(e&SUB_EV_PWR_OFF){
        //print message
        puts("System Powering Down\r\n");
        //send stop sampling packet
        resp=mag_sample_stop(buf);
        //check result
        if(resp<0){
            report_error(ERR_LEV_ERROR,ACDS_ERR_SRC_SUBSYSTEM,ACDS_ERR_SUB_LEDL_STOP,resp);
        }
    }
    if(e&SUB_EV_PWR_ON){
        //print message
        puts("System Powering Up\r\n");
        //set init ACDS mode
        ACDS_mode=ACDS_INIT_MODE;
        //send start sampling packet
        resp= mag_sample_start(buf,32768,0);
        //check result
        if(resp<0){
            report_error(ERR_LEV_ERROR,ACDS_ERR_SRC_SUBSYSTEM,ACDS_ERR_SUB_LEDL_START,resp);
        }
    }
    if(e&SUB_EV_SEND_STAT){
      //send status
      //puts("Sending status\r\n");
      //get status data
      make_status(&status);
      //setup packet 
      ptr=BUS_cmd_init(buf,CMD_ACDS_STAT);
      //fill in telemitry data
      for(i=0;i<sizeof(ACDS_STAT);i++){
        ptr[i]=((unsigned char*)(&status))[i];
      }
      //send command
      resp=BUS_cmd_tx(BUS_ADDR_CDH,buf,sizeof(ACDS_STAT),0);
      if(resp!=RET_SUCCESS){
        //report error
        report_error(ERR_LEV_ERROR,ACDS_ERR_SRC_SUBSYSTEM,ACDS_ERR_SUB_STAT_TX,resp);
        //turn on error LED
        ERR_LED_on();
      }else{
        STAT_LED_toggle();
      }
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
    if(e&SUB_EV_SPI_ERR_BUSY){
        puts("SPI packet lost\r\n");
    }
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

//command data for ACDS settings
signed char settings_dat[10];

CTL_EVENT_SET_t ACDS_evt;

int ACDS_parse_cmd(unsigned char src,unsigned char cmd,unsigned char *dat,unsigned short len,unsigned char flags);
CMD_PARSE_DAT ACDS_parse={ACDS_parse_cmd,CMD_PARSE_ADDR0|CMD_PARSE_GC_ADDR,BUS_PRI_NORMAL,NULL};

//handle ACDS specific commands
int ACDS_parse_cmd(unsigned char src,unsigned char cmd,unsigned char *dat,unsigned short len,unsigned char flags){
  int i;
  unsigned long block_id;
  unsigned long sector;
  signed char *parms;
  switch(cmd){
    case CMD_MAG_DATA:
      //check packet length
      if(len!=sizeof(magData)){
        //length incorrect, report error and exit
        report_error(ERR_LEV_ERROR,ACDS_ERR_SRC_SENSORS,ACDS_ERR_SEN_BAD_PACKET_LENGTH,len);
        return ERR_PK_LEN;
      }
      memcpy(&magData,dat,sizeof(magData));
      //sensor data received set event
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
    case CMD_ACDS_READ_BLOCK:
      if(len!=3){
        return ERR_PK_LEN;
      }
      block_id =((unsigned long)dat[0])<<16;
      block_id|=((unsigned long)dat[1])<<8;
      block_id|=((unsigned long)dat[2]);
      //check range
      if(block_id>LOG_IDX_MAX){
        //index is out of range
        return ERR_PK_BAD_PARM;
      }
      //set SD address
      SD_read_addr=LOG_ADDR_START+block_id;
      //trigger event
      ctl_events_set_clear(&ACDS_evt,ACDS_EVT_SEND_DAT,0);
      return RET_SUCCESS;
    case CMD_ACDS_CONFIG:
      //check length
      if(len<1){
        return ERR_PK_LEN;
      }
      //parameters start at 1
      parms=(signed char*)&dat[1];
      //see what to configure
      switch(dat[0]){
        case ACDS_CONFIG_FLIP:
          //x y z
          //check length
          if(len!=4){
            return ERR_PK_LEN;
          }
          //loop through the arguments
          for(i=0;i<3;i++){
            //check range:
            //  torquers are numbered 1-T_NUM_AXIS_TQ
            //  zero is for no action
            //  sign indicates flip direction
            if(parms[i]>T_NUM_AXIS_TQ || parms[i]<-1*T_NUM_AXIS_TQ){
              //incorrect parameter
              return ERR_PK_BAD_PARM;
            }
            //copy data
            settings_dat[i],parms[i];
          }
          //set event
          ctl_events_set_clear(&ACDS_evt,ACDS_EVT_FLIP,0);
        return RET_SUCCESS;
        case ACDS_CONFIG_SET_TQ:
          //check length
          if(len!=4){
            return ERR_PK_LEN;
          }
          //loop through the arguments
          for(i=0;i<3;i++){
            //check range of possible torques
            if(parms[i]>T_NUM_AXIS_TQ/2 || parms[1]<-1*T_NUM_AXIS_TQ/2){
              //incorrect parameter
              return ERR_PK_BAD_PARM;
            }
            //copy data
            settings_dat[i]=parms[i];
          }
          //set event
          ctl_events_set_clear(&ACDS_evt,ACDS_EVT_TQ_SET,0);
        return RET_SUCCESS;
        case ACDS_CONFIG_DRIVE:
          //check length
          if(len!=4){
            return ERR_PK_LEN;
          }
          //check axis
          if(parms[0]<0 || parms[0]>=3){
              //incorrect axis
              return ERR_PK_BAD_PARM;
          }
          //check torquer number
          if(parms[1]<=0 || parms[1]>T_NUM_AXIS_TQ){
              //incorrect torquer number
              return ERR_PK_BAD_PARM;
          }
          //check torquer direction
          if(parms[2]!=M_MINUS && parms[2]!=M_PLUS){
              //incorrect direction
              return ERR_PK_BAD_PARM;
          }
          //copy data
          memcpy(settings_dat,parms,3);
          //set event
          ctl_events_set_clear(&ACDS_evt,ACDS_EVT_DRIVE,0);
        return RET_SUCCESS;
        case ACDS_CONFIG_TQ_INIT:
          //check length
          if(len!=1){
            return ERR_PK_LEN;
          }
          //set event
          ctl_events_set_clear(&ACDS_evt,ACDS_EVT_TQ_INIT,0);
        return RET_SUCCESS;
        case ACDS_CONFIG_GAIN:
          //check length
          if(len!=(1+1+3*sizeof(float))){
            return ERR_PK_LEN;
          }
          //check gain type
          switch(parms[0]){
            case 'a':
              //set Ka
            break;
            case 'm':
              //set Km
            break;
            case 'b':
              //set Kb
            break;
            default:
              //unknown gain type
              return ERR_PK_BAD_PARM;
          }
          //copy data into staging
          memcpy(&gain_dat,parms,1+3*sizeof(float));
          //set event
          ctl_events_set_clear(&ACDS_evt,ACDS_EVT_WRITE_GAIN,0);
        return RET_SUCCESS;
        case ACDS_CONFIG_FILTER:
          //TODO: do things
        return RET_SUCCESS;
        case ACDS_CONFIG_SETPOINT:
          //check length
          if(len!=(1+3*sizeof(float))){
            return ERR_PK_LEN;
          }
          //copy data
          memcpy(&setpoint_dat,parms,3*sizeof(float));
          //set event
          ctl_events_set_clear(&ACDS_evt,ACDS_EVT_WRITE_SETPOINT,0);
        return RET_SUCCESS;
      }
      //unknown 
      return ERR_PK_BAD_PARM;
  }
  //Return Error
  return ERR_UNKNOWN_CMD;
}


int ACDS_mode=ACDS_HOLD_MODE;



void ACDS_events(void *p) __toplevel{
  unsigned int e;
  int num[3],dir[3];
  signed char tmp;
  VEC T;
  int i,xnum,ynum,znum,resp;
  const VEC zero={0,0,0};
  ACDS_SETTINGS_STORE *tmp_settings;
  VEC Flux,mag;
  CPOINT pt;
  unsigned char *buffer;
  VEC *dest;
  unsigned char buf[BUS_I2C_HDR_LEN+3+BUS_I2C_CRC_LEN],*ptr;
  //init event
  ctl_events_init(&ACDS_evt,0);
  //check correction data status
  read_cor_stat();
  //pause to let error log to be initialized first
  ctl_timeout_wait(ctl_get_current_time()+2048);
  //setup data logging
  log_start();
  //set last field value to NaN
  acds_dat.dat.acds_dat.flux.c.x=__float32_nan;
  acds_dat.dat.acds_dat.flux.c.y=__float32_nan;
  acds_dat.dat.acds_dat.flux.c.z=__float32_nan;
  //set version
  acds_dat.version=ACDS_LOG_VERSION;
  //endless loop
  for(;;){
    //wait for events
    e=ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&ACDS_evt,ACDS_EVT_ALL,CTL_TIMEOUT_NONE,0);
    //magnetometer data event
    if(e&ACDS_EVT_DAT_REC){
      //reset data timeout
      mag_timeout_reset();
      //clear LEDL timeout event
      e&=~ACDS_EVT_DAT_TIMEOUT;
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
                    if(!(ACDS_settings.dat.settings.blacklist&(1<<(2*i)))){
                        Flux.c.y-=pt.c.a;
                        ynum++;
                    }
                    if(!(ACDS_settings.dat.settings.blacklist&(1<<(2*i+1)))){
                        Flux.c.z-=pt.c.b;
                        znum++;
                    }
                  break;
                  //X- axis
                  case MAG_X_MINUS_IDX:
                    if(!(ACDS_settings.dat.settings.blacklist&(1<<(2*i)))){
                        Flux.c.y+=pt.c.a;
                        ynum++;
                    }
                    if(!(ACDS_settings.dat.settings.blacklist&(1<<(2*i+1)))){
                        Flux.c.z-=pt.c.b;
                        znum++;
                    }
                  break;
                  //Y+ axis
                  case MAG_Y_PLUS_IDX:
                    if(!(ACDS_settings.dat.settings.blacklist&(1<<(2*i)))){
                        Flux.c.x+=pt.c.a;
                        xnum++;
                    }
                    if(!(ACDS_settings.dat.settings.blacklist&(1<<(2*i+1)))){
                        Flux.c.z-=pt.c.b;
                        znum++;
                    }
                    break;
                  //Y- axis
                  case MAG_Y_MINUS_IDX:
                    if(!(ACDS_settings.dat.settings.blacklist&(1<<(2*i)))){
                        Flux.c.x-=pt.c.a;
                        xnum++;
                    }
                    if(!(ACDS_settings.dat.settings.blacklist&(1<<(2*i+1)))){
                        Flux.c.z-=pt.c.b;
                        znum++;
                    }
                  break;
                  //Z+ axis
                  case MAG_Z_PLUS_IDX:
                    if(!(ACDS_settings.dat.settings.blacklist&(1<<(2*i)))){
                        Flux.c.y+=pt.c.a;
                        xnum++;
                    }
                    if(!(ACDS_settings.dat.settings.blacklist&(1<<(2*i+1)))){
                        Flux.c.x-=pt.c.b;
                        ynum++;
                    }
                  break;
                  //Z- axis
                  case MAG_Z_MINUS_IDX:
                    if(!(ACDS_settings.dat.settings.blacklist&(1<<(2*i)))){
                        Flux.c.x-=pt.c.a;
                        xnum++;
                    }
                    if(!(ACDS_settings.dat.settings.blacklist&(1<<(2*i+1)))){
                        Flux.c.y+=pt.c.b;
                        ynum++;
                    }
                  break;
              }
          }
      }
      //calculate average
      Flux.c.x=xnum>0?Flux.c.x/xnum:__float32_nan;
      Flux.c.y=ynum>0?Flux.c.y/ynum:__float32_nan;
      Flux.c.z=znum>0?Flux.c.z/znum:__float32_nan;
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
            //printf("Fux = %f %f %f\r\n",Flux.c.x,Flux.c.y,Flux.c.z);
        break;
        case ACDS_IDLE_MODE:
            //do nothing in idle mode
            //send stop sampling packet
            resp=mag_sample_stop(buf);
            //check result
            if(resp<0){
                report_error(ERR_LEV_ERROR,ACDS_ERR_SRC_SUBSYSTEM,ACDS_ERR_SUB_LEDL_STOP,resp);
            }
        break;
        case ACDS_INIT_MODE:
            //check if torquers are initialized
            if(!checkTorqueInit()){
                //flip torquers
                setTorque(&zero);  
            }else{
                //switch to mode 1
                ACDS_mode=ACDS_MODE_1;
            }
        break;
                
      }
      //set raw mag data
      memcpy(&acds_dat.dat.acds_dat.raw_mag,&magData,sizeof(magData));
      //set flux vector
      vec_cp(&acds_dat.dat.acds_dat.flux,&Flux);      
      //set mode
      acds_dat.dat.acds_dat.mode=ACDS_mode;
      //check for command mode
      if(ACDS_mode==ACDS_COMMAND_MODE){
          //send event for command, after structure has been filled
          ctl_events_set_clear(&ACDS_evt,ADCS_EVT_COMMAND_SENSOR_READ,0);
      }
      //write log data
      resp=log_store_data(&acds_dat);
      if(resp){
          report_error(ERR_LEV_ERROR,ACDS_ERR_SRC_ALGORITHM,ACDS_ERR_ALG_LOG_FAIL,resp);
      }
    }
    if(e&ACDS_EVT_DAT_TIMEOUT){
        //TODO : do something
        //puts("LEDL timeout\r");
        ERR_LED_on();
    }
    if(e&ACDS_EVT_SEND_DAT){
      buffer=BUS_get_buffer(CTL_TIMEOUT_DELAY,500);
      if(buffer!=NULL){
        //set block 
        buffer[0]=SPI_ACDS_DAT;
        buffer[1]=BUS_ADDR_ACDS;
        //read block into buffer
        resp=mmcReadBlock(SD_read_addr,buffer+2);
        //check response
        if(resp==RET_SUCCESS){
          //send data to COMM
          resp=BUS_SPI_txrx(BUS_ADDR_COMM,buffer,NULL,sizeof(LOG_DAT_STORE) + 2);
          //check result
          if(resp!=RET_SUCCESS){
            //report error
            report_error(ERR_LEV_ERROR,ACDS_ERR_SRC_I2C_CMD,ACDS_ERR_I2C_SPI_DAT,resp);
          }
        }else{
          //report error
          report_error(ERR_LEV_ERROR,ACDS_ERR_SRC_I2C_CMD,ACDS_ERR_I2C_READ_DAT,resp);
        }
        //done with buffer, free it
        BUS_free_buffer();
      }else{
          //buffer not free, report error
          report_error(ERR_LEV_ERROR,ACDS_ERR_SRC_I2C_CMD,ACDS_ERR_I2C_BUFFER_BUSY,resp);
      }
    }
    if(e&ACDS_EVT_WRITE_SETPOINT){
      //get buffer, set a timeout of ~500 ms
      buffer=BUS_get_buffer(CTL_TIMEOUT_DELAY,500);
      //check for error
      if(buffer!=NULL){
        //set temporary settings pointer
        tmp_settings=(ACDS_SETTINGS_STORE*)buffer;
        //copy settings into temp buffer
        memcpy(tmp_settings,&ACDS_settings,sizeof(ACDS_SETTINGS_STORE));
        //read setpoint
        for(i=0;i<3;i++){
          //get value
          tmp_settings->dat.settings.Omega_CMD.elm[i]=gain_dat[i];  
        }
        //set magic
        tmp_settings->magic=ACDS_SETTINGS_MAGIC;
        //set CRC
        tmp_settings->crc=crc16((void*)&tmp_settings->dat,sizeof(ACDS_SETTINGS));
        //write values to flash
        if((resp=flash_write((void*)&ACDS_settings,tmp_settings,sizeof(ACDS_SETTINGS_STORE)))!=RET_SUCCESS){
          //flash write failed, report error
          report_error(ERR_LEV_ERROR,ACDS_ERR_SRC_I2C_CMD,ACDS_ERR_I2C_FLASH_WRITE,resp);
        }
        //free buffer
        BUS_free_buffer();
      }else{
          //buffer not free, report error
          report_error(ERR_LEV_ERROR,ACDS_ERR_SRC_I2C_CMD,ACDS_ERR_I2C_BUFFER_BUSY,resp);
      }
    }
    if(e&ACDS_EVT_WRITE_GAIN){
      //get buffer, set a timeout of ~500 ms
      buffer=BUS_get_buffer(CTL_TIMEOUT_DELAY,500);
      //check for error
      if(buffer!=NULL){
        //set temporary settings pointer
        tmp_settings=(ACDS_SETTINGS_STORE*)buffer;
        //coppy settings into temp buffer
        memcpy(tmp_settings,&ACDS_settings,sizeof(ACDS_SETTINGS_STORE));
        //determine which gian to set
        switch(gain_dat[0]){
          case 'a':
            dest=&(tmp_settings->dat.settings.Ka);
          break;
          case 'm':
            dest=&(tmp_settings->dat.settings.Km);
          break;
          case 'b':
            dest=&(tmp_settings->dat.settings.Kb);
          break;
          default:
            buffer=NULL;
        }
        //store values in temp buffer
        memcpy(dest,&gain_dat[1],sizeof(VEC));
        //set magic
        tmp_settings->magic=ACDS_SETTINGS_MAGIC;
        //set CRC
        tmp_settings->crc=crc16((void*)&tmp_settings->dat,sizeof(ACDS_SETTINGS));
        //write values to flash
        if(flash_write((void*)&ACDS_settings,tmp_settings,sizeof(ACDS_SETTINGS_STORE))!=RET_SUCCESS){
           //flash write failed, report error
          report_error(ERR_LEV_ERROR,ACDS_ERR_SRC_I2C_CMD,ACDS_ERR_I2C_BUFFER_BUSY,resp);
        }
        //free buffer
        BUS_free_buffer();  
      }else{
          //buffer not free, report error
          report_error(ERR_LEV_ERROR,ACDS_ERR_SRC_I2C_CMD,ACDS_ERR_I2C_BUFFER_BUSY,resp);
      }
    }
    if(e&ACDS_EVT_FLIP){
      for(i=0;i<3;i++){
        if(settings_dat[i]==0){
          num[i]=0;
          dir[i]=0;
        }else{
          if(settings_dat[i]<0){
            num[i]=-1*settings_dat[i];
            dir[i]=M_MINUS;
          }else{
            num[i]=settings_dat[i];
            dir[i]=M_PLUS;
          }
        }
      }
      //drive torquers
      drive_torquers(num,dir);
    }
    if(e&ACDS_EVT_TQ_SET){
      //calculate requested torque
      for(i=0;i<3;i++){
        //multiply by torque of torquer
        T.elm[i]=settings_dat[i]*M_CmdLim_b;
      }
      //set torques
      setTorque(&T);
    }
    if(e&ACDS_EVT_TQ_INIT){
      //initialize torquers
      torqueInit();
    }
    if(e&ACDS_EVT_DRIVE){
      //set num to zero
      memset(num,0,sizeof(num));
      //set dir to zero
      memset(dir,0,sizeof(dir));
      //check axis, just to be sure
      tmp=settings_dat[0];
      if(tmp>=0 && tmp<3){
        //copy data
        num[tmp]=settings_dat[1];
        dir[tmp]=settings_dat[2];
        //drive torquers
        drive_torquers(num,dir);
      }
    }
  }
}

//running interrupt count 
static short int_count;

//reset timeout timer
void mag_timeout_reset(void){
  //disable timer interrupt
  TA0CCTL1=0;
  //initialize interrupt count
  int_count=mag_time.n;
  //set interupt time
  TA0CCR1=readTA0()+mag_time.T;
  //clear event flag
  ctl_events_set_clear(&ACDS_evt,0,ACDS_EVT_DAT_TIMEOUT);
  //enable timer interrupt
  TA0CCTL1=CCIE;
}

//stop timeout timer
void mag_timeout_stop(void){
    //disable interrupt
    TA0CCTL1=0;
    //clear event flag
    ctl_events_set_clear(&ACDS_evt,0,ACDS_EVT_DAT_TIMEOUT);
}

//Timer A1 interrupt
void ACDS_timer(void) __ctl_interrupt[TIMER0_A0_VECTOR]{
  switch(TA0IV){
    //CCR1 : used for sensor data timeout
    case TA0IV_TACCR1:
      //setup next interrupt
      TA0CCR1+=mag_time.T;
      //decremint count
      int_count--;
      if(int_count<=0){
        ctl_events_set_clear(&ACDS_evt,ACDS_EVT_DAT_TIMEOUT,0);
        int_count=mag_time.n;
      }
    break;
    //CCR2 : Unused
    case TA0IV_TACCR2:
    break;
    //TA0INT : unused
    case TA0IV_TAIFG:
    break;
  }
}
