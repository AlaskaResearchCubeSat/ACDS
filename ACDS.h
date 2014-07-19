#ifndef __ACDS_H
#define __ACDS_H

  #include "torquers.h"
  #include "quat.h"
  
  //modes
  enum{ACDS_MODE_1=1,ACDS_MODE_2,ACDS_MODE_3,ACDS_HOLD_MODE,ACDS_COMMAND_MODE,ACDS_IDLE_MODE,ACDS_INIT_MODE};
  
  //events in ACDS_evt
  enum{ACDS_EVT_DAT_REC=0x0002,ADCS_EVT_CAL_COMPLETE=0x0100,ADCS_EVD_COMMAND_SENSOR_READ=0x1000};
    
  //structure for beacon data  
  typedef struct{
      short mag[3];
      char gyro[3];
      unsigned char tqstat[3];
      unsigned short flips[3];
      unsigned short flags;
      IQUAT attitude;
      VEC rates;
  }ACDS_STAT;
    
  //events for ACDS task
  extern CTL_EVENT_SET_t ACDS_evt;
  
  //mode for ACDS system
  extern int ACDS_mode;
  
  extern ACDS_STAT status;
  
  //all events for ACDS
#define ACDS_EVT_ALL  (ACDS_EVT_DAT_REC)
  
  void sub_events(void *p);
  void ACDS_events(void *p);

#endif
  