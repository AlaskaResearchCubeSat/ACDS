#ifndef __ALGORITHM_H
#define __ALGORITHM_H

#include "vector.h"
#include "bias.h"


//current mode of operation
extern unsigned short mode;
//allow mode switching logic to function
extern unsigned short upgrade;

extern float lat,lat_old;

extern VEC Ka,Km;
extern VEC Omega_CMD;

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

#endif
