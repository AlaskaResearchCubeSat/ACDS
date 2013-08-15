
#include "bias.h"

//no Bias
const VEC zero_bias={0,0,0};
//Southbound Equatorial Bias
const VEC SE_bias={0,-2*M_CmdLim_s,0};
//Northbound Equatorial bias
const VEC NE_bias={0,2*M_CmdLim_s,0};
//North Polar Bias
const VEC NP_bias={-2*M_CmdLim_s,0,0};
//Bias for Hysteresis mode
const VEC hyst_bias={0,2*M_CmdLim_b,0};
//Retrograde Correction Bias
const VEC RG_cor_bias={-2.0*M_CmdLim_s,0,0};
//Prograde Reversed Correction Bias
const VEC PGR_cor_bias={-2.0*M_CmdLim_s,0,0};
