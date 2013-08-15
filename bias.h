#ifndef __BIAS_H
#define __BIAS_H

#include "vector.h"

extern const VEC zero_bias;
extern const VEC SE_bias;
extern const VEC NE_bias;
extern const VEC NP_bias;
extern const VEC hyst_bias;
extern const VEC RG_cor_bias;
extern const VEC PGR_cor_bias;


//Magnetic Dipole Command Limit (amperes-m^2)
#define    M_CmdLim_s     (0.00011)
#define    M_CmdLim_b     (0.022)

enum {UNKNOWN_WIN=0,NP_WIN,SE_WIN,SP_WIN,NE_WIN,N45_WIN};

#endif
