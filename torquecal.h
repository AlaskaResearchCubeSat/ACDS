#ifndef __TORQUE_CAL_H
  #define __TORQUE_CAL_H
  #include "vector.h"

  typedef VEC_INT CAL_TBL[64];
  
  #define CAL_MAGIC 0xA5A0
  
  
  typedef struct{
    unsigned short magic;
    CAL_TBL cal;
    //add some bytes so that nothing else fits into this section
    unsigned char fill[126];
  }FLASH_CAL;
  
  VEC_INT *correct(VEC_INT *m);
  int calIdx(void);
  int clrCal(void);
  int writeCal(const CAL_TBL *newCal);
  int dumpCalCmd(char **argv,unsigned short argc);
  int writeCalCmd(char **argv,unsigned short argc);
  int calCmd(char **argv,unsigned short argc);
  int calClearCmd(char **argv,unsigned short argc);
  int calTstCmd(char **argv,unsigned short argc);
  
#endif
  