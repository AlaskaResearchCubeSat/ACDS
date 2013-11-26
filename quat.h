#ifndef __QUAT_H
#define __QUAT_H
//quaternion library
//include vector.h for SCL
#include "vector.h"

typedef union{
  struct {
    SCL a,b,c,d;
  }c;
  SCL elm[4];
} QUAT;


QUAT* quat_scale(SCL s,QUAT* a);
QUAT* quat_cp(QUAT* dest,const QUAT* src);
QUAT* quat_zero(QUAT* q);
QUAT* quat_unit(QUAT* q);
QUAT* quat_ascale(SCL s,QUAT* a);


//printing functions
void quatPrint(const char * name,const QUAT *q);


#endif
