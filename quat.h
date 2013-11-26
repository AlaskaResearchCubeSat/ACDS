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

//integer quaternion
//used for export
typedef struct{
  short a,b,c,d;
}IQUAT;


QUAT* quat_cp(QUAT* dest,const QUAT* src);
QUAT* quat_zero(QUAT* q);
QUAT* quat_unit(QUAT* q);
QUAT* quat_ascale(SCL s,QUAT* a);
QUAT* quat_scale(SCL s,QUAT* a);
QUAT* quat_mul(QUAT* dest,const QUAT* src);

//conversion functions
IQUAT* quat2iquat(IQUAT *dest,const QUAT* src);
QUAT* iquat2quat(QUAT *dest,const IQUAT* src);

//integer functions
//only basic opperations allowed
IQUAT* iquat_cp(IQUAT* dest,const IQUAT* src);
IQUAT* iquat_zero(IQUAT* q);
IQUAT* iquat_unit(IQUAT* q);

//printing functions
void quatPrint(const char * name,const QUAT *q);
void iquatPrint(const char * name,const IQUAT *q);


#endif
