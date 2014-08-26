#ifndef __FILTER_H
#define __FILTER_H



#define FILTER_MAX_B        5
#define FILTER_MAX_A        (FILTER_MAX_B-1)

//constants for filter status
enum {FILTER_OFF,FILTER_ON};

typedef struct{
    unsigned short status;
    unsigned short na,nb;
    float a[FILTER_MAX_A];
    float b[FILTER_MAX_B];
}IIR_FILTER;

float filter(const IIR_FILTER *coeff,float *z,float x);

#endif
    