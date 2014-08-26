#include "filter.h"


float filter(const IIR_FILTER *coef,float *z,float x){
    float y;
    int i,n;
    //calculate number of loops
    n=coef->nb-1;                                 //subtract 1 for first calculation
    n=((coef->na>n)?coef->na:n)-1;                //subtract 1 for last calculation
    //calculate output
    y=coef->b[0]*x+z[0];
    //calculate memory
    for(i=0;i<n;i++){
        z[i]=coef->b[i+1]*x+z[i+1]-coef->a[i]*y;
    }
    //do the last bit
    z[i]=coef->b[i+1]*x-coef->a[i]*y;
    //return result
    return y;
}
