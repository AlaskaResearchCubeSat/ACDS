
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

//set all quaternion elements to zero
QUAT* quat_unit(QUAT* q){
    q->c.a=0;
    q->c.b=0;
    q->c.c=0;
    q->c.d=1;
    return q;
}
