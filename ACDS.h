#ifndef __ACDS_H
#define __ACDS_H

  #include "torquers.h"
  #include "quat.h"
  
  //modes
  enum{ACDS_MODE_1=1,ACDS_MODE_2,ACDS_MODE_3,ACDS_HOLD_MODE,ACDS_COMMAND_MODE,ACDS_IDLE_MODE,ACDS_INIT_MODE};
  
  //events in ACDS_evt
  enum{ACDS_EVT_DAT_REC=0x0002,ADCS_EVT_CAL_COMPLETE=0x0100,ADCS_EVD_COMMAND_SENSOR_READ=0x1000,ACDS_EVT_DAT_TIMEOUT=0x0001};
    
  //structure for beacon data  
  typedef struct{
      short mag[3];
      unsigned char mode;
      unsigned char tqstat[3];
      unsigned short flips[3];
      IQUAT attitude;
      VEC_INT rates;
  }ACDS_STAT;
    
  //Interrupt timing for magnetometer
  typedef struct{
      unsigned short T;       //period
      unsigned short n;       //count
  }MAG_TIME;
  
  //events for ACDS task
  extern CTL_EVENT_SET_t ACDS_evt;
  
  //mode for ACDS system
  extern int ACDS_mode;
  
  //all events for ACDS
#define ACDS_EVT_ALL  (ACDS_EVT_DAT_REC|ACDS_EVT_DAT_TIMEOUT)
  
  void sub_events(void *p);
  void ACDS_events(void *p);

  void make_status(ACDS_STAT *dest);
  void mag_timeout_reset(void);
  void mag_timeout_stop(void);

  int mag_sample_start(void* buf,unsigned short time,unsigned char count);
  int mag_sample_stop(void* buf);
  int mag_sample_single(void* buf);

#endif
  