#ifndef __ALGORITHM_H
#define __ALGORITHM_H

#include "vector.h"
#include "bias.h"
#include "torquers.h"
#include "filter.h"

#define ACDS_SETTINGS_MAGIC       0xAA53

typedef struct{
    //detumble gain
    VEC Ka;
    //alignment gain
    VEC Km;  
    //B-dot gain
    VEC Kb;
    //programed rates (rad/sec)?
    VEC Omega_CMD;
    //black list SPBs
    short blacklist;
}ACDS_SETTINGS;

typedef struct{
    unsigned short magic;
    union{
        ACDS_SETTINGS settings;
        char pad[60];
    }dat;
    unsigned short crc;
}ACDS_SETTINGS_STORE;

extern IIR_FILTER bdot_filter;

//current mode of operation
extern unsigned short mode;
//allow mode switching logic to function
extern unsigned short upgrade;

extern float lat,lat_old;

extern const ACDS_SETTINGS_STORE ACDS_settings;

enum{MODE_NO_UPGRADE=0,MODE_UPGRADE=1};

//calculate rotation rates
//VEC* rateCalc(VEC* dest,short mag[3][3],const VEC* Flux);
VEC* rateCalc(VEC* dest,const VEC_INT *B0,const VEC_INT *B1,const VEC_INT *B2);
//bdot algorithm
void bdot(const VEC *FluxVector,unsigned short step);
//foce the ACDS into a particular mode
short forceMode(unsigned short new_mode,unsigned short new_upgrade);
//setpoint command
int setpointCmd(char **argv,unsigned short argc);
//gain command
int gainCmd(char **argv,unsigned short argc);
//write data to flash
//TODO : move this to a more appropriat file
int flash_write(void *dest,const void *src,int size);

#endif
