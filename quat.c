
#include <msp430.h>
#include <stdio.h>
#include "quat.h"
#include "output_type.h"

//print out a quaternion
void quatPrint(const char * name,const QUAT *q){
  if(output_type==HUMAN_OUTPUT){
      //for human output print name and line return
      printf("%s\t%f\t%f\t%f\t%f\r\n",name,q->c.a,q->c.b,q->c.c,q->c.d);
  }else{
    //for machine output only print values separated by tabs
    printf("%f\t%f\t%f\t%f\t",name,q->c.a,q->c.b,q->c.c,q->c.d);
  }
}
  
//print out an integer quaternion
void iquatPrint(const char * name,const IQUAT *q){
  QUAT tmp;
  //convert to float quaternion and print
  quatPrint(name,iquat2quat(&tmp,q));
}
  
//scale factor for integer quaternion elements
//when representing rotations quaternions can take on +1 through -1
#define IQUAT_SCL   32000 

//convert float quaternion to integer quaternion
IQUAT* quat2iquat(IQUAT *dest,const QUAT* src){
    dest->a=IQUAT_SCL*src->c.a;
    dest->b=IQUAT_SCL*src->c.b;
    dest->c=IQUAT_SCL*src->c.c;
    dest->d=IQUAT_SCL*src->c.d;
    return dest;
}

//convert integer quaternion to float quaternion
QUAT* iquat2quat(QUAT *dest,const IQUAT* src){
    dest->c.a=src->a/((SCL)IQUAT_SCL);
    dest->c.b=src->b/((SCL)IQUAT_SCL);
    dest->c.c=src->c/((SCL)IQUAT_SCL);
    dest->c.d=src->d/((SCL)IQUAT_SCL);
    return dest;
}

//scale quaternion
QUAT* quat_scale(SCL s,QUAT* a){
    a->c.a*=s;
    a->c.b*=s;
    a->c.c*=s;
    a->c.d*=s;
    return a;
}

//quaternion antiscale, divide by s
QUAT* quat_ascale(SCL s,QUAT* a){
    a->c.a/=s;
    a->c.b/=s;
    a->c.c/=s;
    a->c.d/=s;
    return a;
}

//multiply dest*src with quaternion math
QUAT* quat_mul(QUAT* dest,const QUAT* src){
    SCL p1,p2,p3;
    p1        = dest->c.a*src->c.a - dest->c.b*src->c.b - dest->c.c*src->c.c - dest->c.d*src->c.d;
    p2        = dest->c.a*src->c.b + dest->c.b*src->c.a + dest->c.c*src->c.d - dest->c.d*src->c.c;
    p3        = dest->c.a*src->c.c - dest->c.b*src->c.d + dest->c.c*src->c.a + dest->c.d*src->c.b;
    dest->c.d = dest->c.a*src->c.d + dest->c.b*src->c.c - dest->c.c*src->c.b + dest->c.d*src->c.a;
    dest->c.a = p1;
    dest->c.b = p2;
    dest->c.c = p3;
    return dest;
}

//quaternion copy
QUAT* quat_cp(QUAT* dest,const QUAT* src){
    dest->c.a=src->c.a;
    dest->c.b=src->c.b;
    dest->c.c=src->c.c;
    dest->c.d=src->c.d;
    return dest;
}

//set all quaternion elements to zero
QUAT* quat_zero(QUAT* q){
    q->c.a=0;
    q->c.b=0;
    q->c.c=0;
    q->c.d=0;
    return q;
}

//set to unit quaternion
QUAT* quat_unit(QUAT* q){
    q->c.a=1;
    q->c.b=0;
    q->c.c=0;
    q->c.d=0;
    return q;
}

//integer operations

//integer quaternion copy
IQUAT* iquat_cp(IQUAT* dest,const IQUAT* src){
    dest->a=src->a;
    dest->b=src->b;
    dest->c=src->c;
    dest->d=src->d;
    return dest;
}

//set all integer quaternion elements to zero
IQUAT* iquat_zero(IQUAT* q){
    q->a=0;
    q->b=0;
    q->c=0;
    q->d=0;
    return q;
}

//set to unit quaternion
IQUAT* iquat_unit(IQUAT* q){
    q->a=IQUAT_SCL;
    q->b=0;
    q->c=0;
    q->d=0;
    return q;
}
