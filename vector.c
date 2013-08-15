
#include <msp430.h>
#include <stdio.h>
#include "vector.h"
#include "output_type.h"

//print out a vector
void vecPrint(const char * name,const VEC *v){
  if(output_type==HUMAN_OUTPUT){
      //for human output print name and line return
      printf("%s\t%f\t%f\t%f\r\n",name,v->c.x,v->c.y,v->c.z);
  }else{
    //for machine output only print values separated by tabs
    printf("%f\t%f\t%f\t",v->c.x,v->c.y,v->c.z);
  }
}

//print out an integer vector
void ivecPrint(const char * name,const VEC_INT *v){
  if(output_type==HUMAN_OUTPUT){
      //for human output print name and line return
      printf("%s\t%i\t%i\t%i\r\n",name,v->c.x,v->c.y,v->c.z);
  }else{
      //for machine output only print values separated by tabs
      printf("%i\t%i\t%i\t",v->c.x,v->c.y,v->c.z);
  }
}

//vector cross product
VEC* vec_cross(VEC* a,const VEC* b){
    SCL xt,yt,zt;
    xt=a->c.y*b->c.z - a->c.z*b->c.y;
    yt=a->c.z*b->c.x - a->c.x*b->c.z;
    zt=a->c.x*b->c.y - a->c.y*b->c.x;
    a->c.x=xt;
    a->c.y=yt;
    a->c.z=zt;
    return a;
}

//vector dot product
SCL vec_dot(const VEC* a,const VEC* b){
    return a->c.x*b->c.x + a->c.y*b->c.y + a->c.z*b->c.z;
}

//magnitude of vector squared
SCL vec_magsq(const VEC *v){
  return (v->c.x)*(v->c.x) + (v->c.y)*(v->c.y) + (v->c.z)*(v->c.z);
}


//vector addition
VEC* vec_sum(VEC* a,const VEC* b){
    a->c.x+=b->c.x;
    a->c.y+=b->c.y;
    a->c.z+=b->c.z;
    return a;
}

//vector subtraction
VEC* vec_dif(VEC* a,const VEC* b){
    a->c.x-=b->c.x;
    a->c.y-=b->c.y;
    a->c.z-=b->c.z;
    return a;
}

//scale vector
VEC* vec_scale(SCL s,VEC* a){
    a->c.x*=s;
    a->c.y*=s;
    a->c.z*=s;
    return a;
}

//vector antiscale, divide by s
VEC* vec_ascale(SCL s,VEC* a){
    a->c.x/=s;
    a->c.y/=s;
    a->c.z/=s;
    return a;
}

//element by element multiplication
VEC* vec_eemul(VEC* a,const VEC* b){
    a->c.x*=b->c.x;
    a->c.y*=b->c.y;
    a->c.z*=b->c.z;
    return a;
}

//vector copy
VEC* vec_cp(VEC* dest,const VEC* src){
    dest->c.x=src->c.x;
    dest->c.y=src->c.y;
    dest->c.z=src->c.z;
    return dest;
}

//set all vector elements to zero
VEC* vec_zero(VEC* v){
    v->c.x=0;
    v->c.y=0;
    v->c.z=0;
    return v;
}

//integer versions

//magnitude of integer vector squared
SCL_INT ivec_magsq(const VEC_INT* v){
  return (v->c.x)*(v->c.x) + (v->c.y)*(v->c.y) + (v->c.z)*(v->c.z);
}

//integer vector copy
VEC_INT* ivec_cp(VEC_INT* dest,const VEC_INT* src){
    dest->c.x=src->c.x;
    dest->c.y=src->c.y;
    dest->c.z=src->c.z;
    return dest;
}

//integer vector subtraction
VEC_INT* ivec_dif(VEC_INT* a,const VEC_INT* b){
    a->c.x-=b->c.x;
    a->c.y-=b->c.y;
    a->c.z-=b->c.z;
    return a;
}

//used for crossproduct
//returns a1*b1-a2*b2 but uses the MACS mode of the HW multiplier
//this results in 32bit intermediate precision for a more accurate result (hopefully)
short cmul(short a1,short b1,short a2,short b2){
  //initialize result
  RESHI=0;
  RESLO=0;
  MACS=a1;
  OP2=b1;
  MACS=-a2;
  OP2=b2;
  return RESLO;
}

//vector cross product
//use the MAC mode of the hardware multiplier
VEC_INT* ivec_cross(VEC_INT* a,const VEC_INT* b){
  //temp variables for Y and Y
  SCL_INT xt,yt;
  //xt=a->c.y*b->c.z - a->c.z*b->c.y;
  xt=cmul(a->c.y,b->c.z,a->c.z,b->c.y);
  //y component
  //yt=a->c.z*b->c.x - a->c.x*b->c.z;
  yt=cmul(a->c.z,b->c.x,a->c.x,b->c.z);
  //Z can be set directly because the other components are calculated
  a->c.z=cmul(a->c.x,b->c.y,a->c.y,b->c.x);
  a->c.x=xt;
  a->c.y=yt;
  //a->c.z=a->c.x*b->c.y - a->c.y*b->c.x;
  return a;
}

//convert an integer vector to a vector
VEC* ivec2vec(VEC* dest,const VEC_INT* src){
  dest->c.x=src->c.x;
  dest->c.y=src->c.y;
  dest->c.z=src->c.z;
  return dest;
}
