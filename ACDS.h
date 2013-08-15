#ifndef __ACDS_H
#define __ACDS_H


//events for ACDS task
extern CTL_EVENT_SET_t ACDS_evt;

//mode for ACDS system
extern int ACDS_mode;

//modes
enum{ACDS_MODE_1=1,ACDS_MODE_2,ACDS_MODE_3,ACDS_CAL_MODE,ACDS_HOLD_MODE};

//events in ACDS_evt
enum{ACDS_EVT_SEND_STAT=0x0001,ACDS_EVT_DAT_REC=0x0002,ADCS_EVT_CAL_COMPLETE=0x0100};

//all events for ACDS
#define ACDS_EVT_ALL  (ACDS_EVT_SEND_STAT|ACDS_EVT_DAT_REC)

void sub_events(void *p);
void ACDS_events(void *p);

#endif
  