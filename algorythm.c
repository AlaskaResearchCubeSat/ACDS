#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "algorithm.h"
#include "vector.h"
#include "bias.h"
#include "torquers.h"
#include "terminal.h"
#include "output_type.h"

const float pi=3.14159265358979323846;

//current ACDS data
ACDS_DAT acds_dat;

//outputs values of {2,1,-2}*qval according to Figure 14 in Mench 2011
//may be combined into torquer firing routine
VEC* quantize(VEC* val,SCL qval){
  //quantize X axis
  if(val->c.x>qval){
    val->c.x=2*qval;
  }else if(val->c.x < -qval){
    val->c.x=-2*qval;
  }else{
    val->c.x=0;
  }
  //quantize Y axis
  if(val->c.y>qval){
    val->c.y=2*qval;
  }else if(val->c.y<-qval){
    val->c.y=-2*qval;
  }else{
    val->c.y=0;
  }
  //quantize Z axis
  if(val->c.z>qval){
    val->c.z=2*qval;
  }else if(val->c.z<-qval){
    val->c.z=-2*qval;
  }else{
    val->c.z=0;
  }
  return val;
}

//these come from Cranks simulation
const float lat_trigger=90-10,offset=10,eq_window=15;
float lat,lat_old;

//determines if the CubeSat is within one of the 3 bias windows
short get_window(float lat){
    //check for north polar window
    if( lat < (lat_trigger-offset) && lat > lat_old){
        return NP_WIN;
    }
    //check for South bound equatorial window
    if(lat < (eq_window/2+offset) && lat > -(eq_window/2-offset) && lat < lat_old){
        return SE_WIN;
    }
    //check for North bound equatorial window
    if(lat > -(eq_window/2+offset) && lat < (eq_window/2-offset) && lat > lat_old){
        return NE_WIN;
    }
    //window for retrograde check
    if(lat > 45 && lat < 50 && lat > lat_old){
        return N45_WIN;
    }
    return UNKNOWN_WIN;
}

//prevents generating a dipole moment in the opposite direction of the bias
VEC* biasFix(VEC* cmd,const VEC* bias){
    //X - axis
    /* X - axis only biased in negative direction this is not needed
    if(cmd->c.x<0 && bias->c.x>0){
        cmd->c.x=0;
    }*/
    if(cmd->c.x>0 && bias->c.x<0){
        cmd->c.x=0;
    }
    //Y - axis
    if(cmd->c.y<0 && bias->c.y>0){
        cmd->c.y=0;
    }
    if(cmd->c.y>0 && bias->c.y<0){
        cmd->c.y=0;
    }
    /* Z - axis is unused so this is not needed
    //Z - axis
    if(cmd->c.z<0 && bias->c.z>0){
        cmd->c.z=0;
    }
    if(cmd->c.z>0 && bias->c.z<0){
        cmd->c.z=0;
    }*/
    return cmd;
}

//current mode of operation
unsigned short mode=1;

//allow mode switching logic to function
unsigned short upgrade=MODE_UPGRADE;

#pragma constseg("INFO_D")
#pragma zeroedseg("INFO_D")
const ACDS_SETTINGS_STORE ACDS_settings={ACDS_SETTINGS_MAGIC,{{
                                    //detumble gain
                                    {10000*0.00024,10000*0.00024,10000*0.00015},
                                    //alignment gain
                                    {10000*0.00024,10000*0.00024,10000*0.00015},
                                    //B-dot gain
                                    {200.0,200.0,200.0},
                                    //programed rates (rad/sec)?
                                    {0,0,0.0011}
                                    }},
                                    //CRC not correct
                                    0};
#pragma constseg(default)
#pragma zeroedseg(default)

//edit angular rate setpoint
int setpointCmd(char **argv,unsigned short argc){
  VEC tmp;
  char *end;
  int i;
  //check number of arguments
  if(argc!=3 && argc!=0){
    printf("Error : %s requiors 0 or 3 arguments but %i given.\r\n",argv[0],argc);
    return -2;
  }
  //if 3 arguments given, parse and set
  if(argc==3){
    for(i=0;i<3;i++){
      //get value
      tmp.elm[i]=strtof(argv[i+1],&end);
      //check if value parsed
      if(end==argv[i+1]){
          printf("Error : could not parse element \"%s\".\r\n",argv[i+1]);
          return 2;
      }
      //check for unknown sufix
      if(*end!=0){
        printf("Error : unknown sufix \"%s\" at end of element \"%s\"\r\n",end,argv[i+1]);
        return 3;
      }   
    }
    //set value
    vec_cp(&Omega_CMD,&tmp);
  }
  //print value
  vecPrint("Omega_cmd",&Omega_CMD);
  if(output_type==MACHINE_OUTPUT){
    printf("\r\n");
  }
  return 0;
}

//set detumble and alignment gains
int gainCmd(char **argv,unsigned short argc){
  VEC tmp,*dest;
  char *end;
  int i;
  //chekc number of arguments
  if(argc!=4 && argc!=0 && argc!=2){
    printf("Error : %s requiors 0, 4 or 2  arguments but %i given.\r\n",argv[0],argc);
    return -2;
  }
  //if zero arguments given print both gains
  if(argc==0){      
    //detumble gain
    vecPrint("Ka",&ACDS_settings.dat.settings.Ka);
    if(output_type==MACHINE_OUTPUT){
        printf("\r\n");
    }
    //alignment gain
    vecPrint("Km",&ACDS_settings.dat.settings.Km);
    if(output_type==MACHINE_OUTPUT){
        printf("\r\n");
    }
    //alignment gain
    vecPrint("Kb",&ACDS_settings.dat.settings.Kb);
    if(output_type==MACHINE_OUTPUT){
        printf("\r\n");
    }
    return 0;
  }
  //determine which gian to set
  if(!strcmp(argv[1],"Ka")){
    dest=&ACDS_settings.dat.settings.Ka;
  }else if(!strcmp(argv[1],"Km")){
    dest=&ACDS_settings.dat.settings.Km;
  }else if(!strcmp(argv[1],"Kb")){
    dest=&ACDS_settings.dat.settings.Kb;
  }else{
    printf("Error : Unknown Gain \"%s\" \r\n",argv[1]);
    return 5;
  }
  //read values
  for(i=0;i<((argc==4)?3:1);i++){
    //get value
    printf("i = %i\r\n",i);
    tmp.elm[i]=strtof(argv[i+2],&end);
    //check if value parsed correctly
    if(end==argv[i+2]){
        printf("Error : could not parse element \"%s\".\r\n",argv[i+2]);
        return 2;
    }
    //check for unknown suffix
    if(*end!=0){
      printf("Error : unknown sufix \"%s\" at end of element \"%s\"\r\n",end,argv[i+2]);
      return 3;
    }   
  }
  //if 2 arguments given set all components the same
  if(argc==2){
    tmp.c.y=tmp.c.x;
    tmp.c.z=tmp.c.x;
  }
  //store values
  vec_cp(dest,&tmp);
  //print values
  vecPrint(argv[1],dest);
  if(output_type==MACHINE_OUTPUT){
    printf("\r\n");
  }
  return 0;
}

//used to count cycles for mode switching
static unsigned short trip=0;

//called to force a particular mode
short forceMode(unsigned short new_mode,unsigned short new_upgrade){
  switch(new_mode){
    case 0:
      //TODO: neutralize small torquers
      mode=0;
      trip=0;
    break;
    case 1:
      //TODO: neutralize small torquers
      mode=1;
    break;
    case 2:
      //TODO: neutralize large torquers
      mode=2;
      trip=0;
    break;
    case 3:
      //TODO: neutralize large torquers
      mode=3;
    break;
    default:
      printf("Error : can\'t force unknown mode %i\r\n",new_mode);
    return 0;
  }
  //display new mode
  printf("Forcing Mode %i\r\n",new_mode);
  if(new_upgrade==MODE_UPGRADE){
    upgrade=MODE_UPGRADE;
  }else{
    upgrade=MODE_NO_UPGRADE;
  }
  return 1;
}

static VEC oldFlux;
static short oldFluxValid=0;

//run the B-dot control algorithm
void bdot(const VEC *FluxVector,unsigned short step){
  //commanded dipole moments
  VEC M_cmd;
  //check if old flux is valid
  if(oldFluxValid){
    //compute B-dot
    vec_cp(&M_cmd,FluxVector);
    vec_dif(&M_cmd,&oldFlux);
  }else{
    //use zero vector for B-dot
    vec_zero(&M_cmd);
  }
  //set mode 1
  acds_dat.mode=1;
  //save magnetic field derivative
  vec_cp(&acds_dat.mdat.mode1.B_dot,&M_cmd);
  //apply gain
  vec_eemul(&M_cmd,&ACDS_settings.dat.settings.Kb);
  //save commanded dipole moment
  vec_cp(&acds_dat.M_cmd,&M_cmd);
  //flip torquers
  setTorque(&M_cmd);
  //save status
  get_stat(&acds_dat.tq_stat);
  //save old flux
  vec_cp(&oldFlux,FluxVector);
  oldFluxValid=1;
}


void print_acds_dat(const ACDS_DAT *dat){
    //print ACDS mode
    if(output_type==HUMAN_OUTPUT){
        printf("Mode : %i\r\n",dat->mode);
    }else{
        printf("M%i\t",dat->mode);
    }
    //print magnetic flux vector
    vecPrint("Flux",&dat->flux);
    //print mode specific data
    switch(dat->mode){
        case 1:
            vecPrint("B-dot",&dat->mdat.mode1.B_dot);
        break;
    }
    //print commanded dipole moment
    vecPrint("Mcmd",&dat->M_cmd);
    //print new torquer status
    if(output_type==HUMAN_OUTPUT){
        print_tqstat(&dat->tq_stat);
    }else{
        print_tqstat_code(&dat->tq_stat);
        //print trailing newline
        printf("\r\n");
    }
}
