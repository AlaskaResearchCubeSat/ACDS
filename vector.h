#ifndef __VECTOR_H
#define __VECTOR_H

//Vector math library

//scaler and vector component definitions
//typedef unsigned short SCL;
//typedef float SCL;
typedef double SCL;
typedef short SCL_INT;

/*typedef struct {
  SCL x,y,z;
} VEC;*/

typedef union{
  struct {
    SCL x,y,z;
  }c;
  SCL elm[3];
} VEC;

typedef union{
  struct {
    SCL_INT x,y,z;
  }c;
  SCL_INT elm[3];
} VEC_INT;


VEC* vec_cross(VEC* a,const VEC* b);
SCL vec_dot(const VEC* a,const VEC* b);
VEC* vec_scale(SCL s,VEC* a);
VEC* vec_sum(VEC* a,const VEC* b);
VEC* vec_dif(VEC* a,const VEC* b);
VEC* vec_eemul(VEC* a,const VEC* b);
VEC* vec_cp(VEC* dest,const VEC* src);
VEC* vec_zero(VEC* v);
VEC* vec_ascale(SCL s,VEC* a);
SCL vec_magsq(const VEC *v);

//intiger vector opperations
SCL_INT ivec_magsq(const VEC_INT* v);
VEC* ivec2vec(VEC* dest,const VEC_INT* src);
VEC_INT* ivec_cross(VEC_INT* a,const VEC_INT* b);
SCL_INT ivec_dot(const VEC_INT* a,const VEC_INT* b);
VEC_INT* ivec_scale(SCL_INT s,VEC_INT* a);
VEC_INT* ivec_sum(VEC_INT* a,const VEC_INT* b);
VEC_INT* ivec_dif(VEC_INT* a,const VEC_INT* b);
VEC_INT* ivec_eemul(VEC_INT* a,const VEC_INT* b);
VEC_INT* ivec_cp(VEC_INT* dest,const VEC_INT* src);
VEC_INT* ivec_zero(VEC_INT* v);
VEC_INT* ivec_ascale(SCL_INT s,VEC_INT* a);

//printing functions
void vecPrint(const char * name,const VEC *v);
void ivecPrint(const char * name,const VEC_INT *v);


#endif
