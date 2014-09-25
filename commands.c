#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <msp430.h>
#include <ctl_api.h>
#include <terminal.h>
#include <ARCbus.h>
#include <Error.h>
#include <SDlib.h>
#include <errno.h>
#include <commandLib.h>
#include <math.h>
#include <limits.h>
#include <crc.h>
#include "torquers.h"
#include "output_type.h"
#include "SensorDataInterface.h"
#include "algorithm.h"
#include "stackcheck.h"
#include "ACDS.h"
#include "LED.h"
#include "IGRF/igrf.h"
#include "corrections.h"
#include "log.h"
#include "crc.h"

//define printf formats
#define HEXOUT_STR    "%02X "
#define ASCIIOUT_STR  "%c"

int current_set=TQ_SET_BIG;


//Set torque in each axis
int setTorqueCmd(char **argv,unsigned short argc){
  VEC T;
  int i;
  char *end;
  // three arguments are accepted, X axis, Y axis and Z axis torques
  if(argc!=3){
    printf("Error : %s requires 3 arguments\r\n",argv[0]);
    return -1;
  }
  for(i=0;i<3;i++){
    //get torques
    T.elm[i]=strtof(argv[i+1],&end);
    if(end==argv[i+2]){
      printf("Error : could not parse torquer number \'%s\'\r\n",argv[i+1]);
      return -1;
    }else if(*end!=0){
      printf("Error : unknown suffix \'%s\' on torque \'%s\'\r\n",end,argv[i+1]);
      return -2;
    }
  }
  /*T.c.x=readTorque(argv[1]);
  T.c.y=readTorque(argv[2]);
  T.c.z=readTorque(argv[3]);*/
  //print old status
  printf("Previous Torquer Status:\r\n");
  print_torquer_status();
  //current_set torques
  setTorque(&T);
  //print status
  printf("New Torquer Status:\r\n");
  print_torquer_status();
  return 0;
}

//flip a torquer in each axis
int flipCmd(char **argv,unsigned short argc){
  int num[3]={0,0,0},dir[3]={0,0,0};
  int i,set=TQ_SET_BIG;
  unsigned long tnum;
  char *end;
  const char axis[]={'X','Y','Z'};
  // three arguments are accepted, X axis, Y axis and Z axis torques
  if(argc!=3){
    printf("Error : %s requires 3 arguments\r\n",argv[0]);
    return -1;
  }
  //get torquers
  for(i=0;i<3;i++){
    //check for no flip represented by a zero
    if(!strcmp("0",argv[i+1])){
      dir[i]=0;
      num[i]=0;
    }else{
      //get torquer direction
      if(argv[i+1][0]=='+'){
        dir[i]=M_PLUS;
      }else if(argv[i+1][0]=='-'){
        dir[i]=M_MINUS;
      }else{
        printf("Error : \'%c\' is not a valid torquer direction for %c-axis\r\n",argv[3][0],axis[i]);
        return -1;
      }
      //get torquer number
      tnum=strtoul(argv[i+1]+1,&end,10);
      if(end==argv[2]){
        printf("Error : could not parse \'%s\' for %c-axis torquer number\r\n",argv[i+1]+1,axis[i]);
        return -1;
      }else if(*end!=0){
        printf("Error : unknown suffix \'%s\' for %c-axis torquer number\r\n",end,axis[i]);
        return -2;
      }else if(tnum>T_NUM_AXIS_TQ){
        printf("Error : torquer number %lu for %c-axis is too large. maximum torquer number is %u\r\n",tnum,axis[i],T_NUM_AXIS_TQ);
        return -3;
      }
      num[i]=tnum;
      //check if not flipping a torquer
      if(tnum==0){
        //clear direction
        dir[i]=0;
      }
    }
  }
  if(output_type==HUMAN_OUTPUT){
    //print old status
    printf("Previous Torquer Status:\r\n");
    print_torquer_status();
  }
  //drive torquers 
  drive_torquers(num,dir);
  if(output_type==HUMAN_OUTPUT){
    //print status
    printf("New Torquer Status:\r\n");
    print_torquer_status();
  }else{
    print_torquer_stat_code();
    printf("\r\n");
  }
  return 0;
}

//drive a torquer
int driveCmd(char **argv,unsigned short argc){
  int num[3]={0,0,0},dir[3]={0,0,0};
  int axis=0;
  unsigned long tnum;
  char *end;
  //three arguments are accepted: axis, torquer direction
  if(argc!=3){
    printf("Error : %s requires 3 arguments\r\n",argv[0]);
    return -1;
  }
  //get axis to drive
  if(!strcmp("X",argv[1])){
    axis=X_AXIS;
  }else if(!strcmp("Y",argv[1])){
    axis=Y_AXIS;
  }else if(!strcmp("Z",argv[1])){
    axis=Z_AXIS;
  }else{
    printf("Error : unknown axis \'%s\'\r\n",argv[1]);
    return -1;
  }
  //get torquer number
  tnum=strtoul(argv[2],&end,10);
  if(end==argv[2]){
    printf("Error : could not parse torquer number \'%s\'\r\n",argv[2]);
    return -1;
  }else if(*end!=0){
    printf("Error : unknown suffix \'%s\' for torquer number\r\n",end);
    return -2;
  }else if(tnum>T_NUM_AXIS_TQ){
    printf("Error : torquer number %lu is too large. maximum torquer number is %u\r\n",tnum,T_NUM_AXIS_TQ);
    return -3;
  }
  num[axis]=tnum;
  //get torque direction
  if(strlen(argv[3])!=1){
    printf("Error parsing torquer direction\r\n");
    return -1;
  }
  if(argv[3][0]=='+'){
    dir[axis]=M_PLUS;
  }else if(argv[3][0]=='-'){
    dir[axis]=M_MINUS;
  }else{
    printf("Error : \'%s\' is not a valid torquer direction\r\n",argv[3]);
    return -1;
  }
  //print old status
  printf("Previous Torquer Status:\r\n");
  print_torquer_status();  
  //drive torquer 
  drive_torquers(num,dir);
  //print new status
  printf("New Torquer Status:\r\n");
  print_torquer_status();
  return 0;
}

int tqstatCmd(char **argv,unsigned short argc){
  int set=TQ_SET_NONE,i;
  if(argc>2){
    printf("Error : %s takes no arguments\r\n",argv[0]);
    return -1;
  }
  //print torquer status
  print_torquer_status();
  return 0;
}

int statcodeCmd(char **argv,unsigned short argc){
  int set=TQ_SET_NONE,i;
  if(argc>2){
    printf("Error : %s takes no arguments\r\n",argv[0]);
    return -1;
  }
  //print torquer status
  print_torquer_stat_code();
  printf("\r\n");
  return 0;
}

int initCmd(char **argv,unsigned short argc){
  torqueInit();
  return 0;
}

int reinitCmd(char **argv,unsigned short argc){
  torqueReinit();
  return 0;
}

int compCmd(char **argv,unsigned short argc){
  unsigned char fb;
  int i;
  const char axis[]={'X','Y','Z'};
  fb=get_torquer_fb();
  printf("fb = %u\r\n",fb);
  for(i=0;i<3;i++){
    printf("%c-axis charged    : %s\r\n"
           "%c-axis discharged : %s\r\n",axis[i],(fb&0x02)?"yes":"no",axis[i],(fb&0x01)?"yes":"no");
    fb>>=2;
  }
  return 0;
}

//basically same as drive command but used for the purpose of testing 
int tstCmd(char **argv,unsigned short argc){
  int num[3]={0,0,0},dir[3]={0,0,0};
  int axis=0,err;
  extern TQ_SET tq_stat;
  //three arguments are accepted: axis, torquer direction
  if(argc!=2){
    printf("Error : %s requires 2 arguments\r\n",argv[0]);
    return -1;
  }
  //get axis to drive
  if(!strcmp("X",argv[1])){
    axis=X_AXIS;
  }else if(!strcmp("Y",argv[1])){
    axis=Y_AXIS;
  }else if(!strcmp("Z",argv[1])){
    axis=Z_AXIS;
  }else{
    printf("Error : unknown axis \'%s\'\r\n",argv[1]);
    return -1;
  }
  //get torquer number
  if(strlen(argv[2])!=1){
    printf("Error parsing torquer number\r\n");
    return -1;
  }
  if(argv[2][0]=='1'){
    num[axis]=1;
  }else if(argv[2][0]=='2'){
    num[axis]=2;
  }else{
    printf("Error : \'%s\' is not a valid torquer number\r\n",argv[2]);
    return -1;
  }
  //direction is oposite of current direction
  //get direction of torquer
  dir[axis]=(tq_stat.elm[axis].status>>(num[axis]-1))&1;
  //Toggle direction
  if(dir[axis]==0){
    //Torquer was minus, set to plus
    dir[axis]=M_PLUS;
  }else{
    //torquer was plus, set to minus
    dir[axis]=M_MINUS;
  }
  //drive torquer 
  err=drive_torquers(num,dir);
  //print message based on the error that is returned
  switch(err){
    case RET_SUCCESS:
      printf("Success\r\n");
    break;
    case TQ_ERR_BAD_SET:
    case TQ_ERR_BAD_TORQUER:
      //this should not happen, arguments are checked
      printf("Internal Error\r\n");
    break;
    case TQ_ERR_COMP:
      printf("Error : Comparitor\r\n");
    break;
    case TQ_ERR_CAP:
      printf("Error : Capacitor\r\n");
    break;
    case TQ_ERR_BAD_CONNECTION:
      printf("Error : Bad Connection\r\n");
    break;
    default:
      printf("Unknown Error %i\r\n",err);
    break;
  }
  return err;
}

//tell LEDL to start reading magnetomitors
int sensorRunCmd(char **argv,unsigned short argc){
  unsigned short time=32768,count=0;
  unsigned char buff[BUS_I2C_HDR_LEN+3+BUS_I2C_CRC_LEN],*ptr;
  int res;
  if(argc==2){
    time=atoi(argv[1]);
    count=atoi(argv[2]);
  }else if(argc!=0){
      printf("Error : %s takes 0 or 2 arguments\r\n",argv[0]);
      return -1;
  }
  //setup sampling
  res=mag_sample_start(buff,time,count);
  //check result
  if(res<0){
    printf("Error communicating with LEDL : %s\r\n",BUS_error_str(res));
    //return error
    return 1;
  } 
  return 0;
}

//tell LEDL to stop reading magnetomitors
int sensorStopCmd(char **argv,unsigned short argc){
  unsigned char buff[BUS_I2C_HDR_LEN+1+BUS_I2C_CRC_LEN],*ptr;
  int res;
  //send stop command to LEDL
  res=mag_sample_stop(buff);
  //check result
  if(res<0){
    printf("Error communicating with LEDL : %s\r\n",BUS_error_str(res));
    //return error
    return 1;
  } 
  return 0;
}

int clrErrCmd(char **argv,unsigned short argc){
  ERR_LED_off();
  return 0;
}

short output_type=HUMAN_OUTPUT;
//switch output type
int outputTypeCmd(char **argv,unsigned short argc){
    //check to see if one argument given
    if(argc>1){
      printf("Error : too many arguments\r\n");
      return -1;
    }
    //if no arguments given print output type
    if(argc==0){
      switch(output_type){
        case HUMAN_OUTPUT:
          printf("human\r\n");
        break;
        case MACHINE_OUTPUT:
          printf("machine\r\n");
        break;
        default:
          printf("Error : internal error\r\n");
          return 1;
      }
      return 0;
    }
    //check argument for output type
    if(!strcmp("human",argv[1])){
      output_type=HUMAN_OUTPUT;
    }else if(!strcmp("machine",argv[1])){
      output_type=MACHINE_OUTPUT;
    }else{
      //unknown output type print error
      printf("Error : unknown output type \'%s\'.\r\n",argv[1]);
      return -1;
    }
    return 0;
}

#define PI 3.14159265358979323846
#define RAD2DEG (180.0/PI)

int shval3Cmd(char **argv,unsigned short argc){
  float year,lat,lon,alt;
  VEC field;
  char *end;
  int nmax;
  //check for proper number of arguments
  if(argc!=4){
    printf("Error: %s takes exactly 4 arguments\r\n",argv[0]);
    return -1;
  }
  //parse arguments
  year=strtof(argv[1],&end);
  alt=strtof(argv[2],&end);
  lat=strtof(argv[3],&end);
  lon=strtof(argv[4],&end);
  //turn on LED's
  P7OUT|=BIT0|BIT1;
  //extrapolate model to desired date
  nmax=extrapsh(year);
  P7OUT&=~BIT1;
  P7OUT|=BIT2;
  //calculate magnetic field
  shval3(lat,lon,alt,nmax,&field);
  P7OUT&=~(BIT0|BIT2);
  //print
  printf("%f %f %f\r\n",field.c.x,field.c.y,field.c.z);
  return 0;
}

int tst_IGRF_cmd(char **argv,unsigned short argc){
    VEC field;
    int nmax;
    P7OUT|=BIT0|BIT1;
    //extrapolate model to desired date
    nmax=extrapsh(2013);
    P7OUT&=~BIT1;
    P7OUT|=BIT2;
    //calculate magnetic field
    shval3(64.9261111/RAD2DEG,-147.4958333/RAD2DEG,6371.2,nmax,&field);
    //shval3(1.13317,-2.57429,6371.2,nmax,&field);
    //shval3(64.9261111/RAD2DEG,-147.4958333/RAD2DEG,6771.2,nmax,&field);
    P7OUT&=~(BIT0|BIT2);
    //print
    printf("x = %f\r\ny = %f\r\nz = %f\r\n",field.c.x,field.c.y,field.c.z);
    return 0;
}

int statusCmd(char **argv,unsigned short argc){
  int i;
  ACDS_STAT status;
  //get status data
  make_status(&status);
  printf("Status Size = %u\r\n",sizeof(ACDS_STAT));
  //print status data
  printf   ("Mode   \t%i\r\n",status.mode);
  printf   ("mag     \t%i, %i, %i\r\n",status.mag[0],status.mag[1],status.mag[2]);
  printf   ("tqstat  \t0x%02X, 0x%02X, 0x%02X\r\n",status.tqstat[0],status.tqstat[1],status.tqstat[2]);
  printf   ("flips   \t%u, %u, %u\r\n",status.flips[0],status.flips[1],status.flips[2]);
  iquatPrint("attitude",&status.attitude);
  ivecPrint ("rates   ",&status.rates);
  return 0;
}  

int randomTorqueCmd(char **argv,unsigned short argc){
  VEC T;
  int i;
  for(i=0;i<3;i++){
    //generate random torque
    T.elm[i]=(rand()%5-2)*2;
  }
  printf("Torque = (%+i,%+i,%+i)\r\n",(int)T.c.x,(int)T.c.y,(int)T.c.z);
  //print old status
  printf("Previous Torquer Status:\r\n");
  print_torquer_status();
  //current_set torques
  setTorque(&T);
  //print status
  printf("New Torquer Status:\r\n");
  print_torquer_status();
  return 0;
}

enum{COR_X_BASE=0,COR_Y_BASE=2,COR_Z_BASE=4,COR_PLUS_OFFSET=0,COR_MINUS_OFFSET=1};

const char * const (cor_axis_names[])={"X+","X-","Y+","Y-","Z+","Z-"};

int unpackCmd(char **argv,unsigned short argc){
    unsigned long sector;
    unsigned char *buffer=NULL;
    C_AXIS *dest;
    int ret;
    unsigned short check_c,check_s;
    int i,resp,idx;
    if(argc<1){
        printf("Error: too few arguments\r\n");
        return -1;
    }
    if(argc>1){
        printf("Error: too many arguments\r\n");
        return -2;
    }
    //read sector
    if(1!=sscanf(argv[1],"%lu",&sector)){
      //print error
      printf("Error parsing sector \"%s\"\r\n",argv[1]);
      return -3;
    }
    //get buffer, set a timeout of 2 secconds
    buffer=BUS_get_buffer(CTL_TIMEOUT_DELAY,2048);
    //check for error
    if(buffer==NULL){
        printf("Error : Timeout while waiting for buffer.\r\n");
        return -1;
    }
    //read block
    resp=mmcReadBlock(sector,(unsigned char*)buffer);
    //check if block was read
    if(resp){
        printf("%s\r\n",SD_error_str(resp));
        //free buffer
        BUS_free_buffer();
        //return
        return resp;
    }
    //calculate check
    for(i=0,check_c=0;i<510;i++){
        check_c+=buffer[i];
    }
    //read check
    check_s=buffer[510]|(((unsigned short)buffer[511])<<8);
    //print out header
    printf("%c%c%c%c%c%c\r\n",buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5]);
    //check check
    if(check_c!=check_s){
        printf("Error: checksum failed. stored = %u calculated = %u\r\n",check_s,check_c);
        BUS_free_buffer();
        return 1;
    }
    //check header
    if(buffer[0]!='C' || buffer[1]!='O' ||buffer[2]!='R' ||buffer[3]!=' '){
        printf("Error: incorrect header\r\n");
        BUS_free_buffer();
        return 2;
    }
    switch(buffer[4]){
        case 'X':
            //X-axis
            idx=COR_X_BASE;
        break;
        case 'Y':
            //Y-axis
            idx=COR_Y_BASE;
        break;
        case 'Z':
            //Z-axis
            idx=COR_Z_BASE;
        break;
        default:
            printf("Error : unknown axis \'%c\'\r\n",buffer[4]);
    }
    switch(buffer[5]){
        case '+':
            //+ direction
            idx+=COR_PLUS_OFFSET;
        break;
        case '-':
            //- direction
            idx+=COR_MINUS_OFFSET;
        break;
        default:
            printf("Error : unknown direction \'%c\'\r\n",buffer[5]);
    }
    dest=(C_AXIS*)(buffer+512);
    printf("Unpacking correction data for the %s axis\r\n",cor_axis_names[idx]);
    //get correction values
    dest->scl[0]=*((float*)(buffer+ 6));
    dest->scl[1]=*((float*)(buffer+10));
    dest->scl[2]=*((float*)(buffer+14));
    dest->scl[3]=*((float*)(buffer+18));
    dest->baseOS.c.a=*((float*)(buffer+22));
    dest->baseOS.c.b=*((float*)(buffer+26));
    //get X offsets
    for(i=0;i<16;i++){
        dest->osX[i].c.a=*((float*)(buffer+ 30+4*i));
        dest->osX[i].c.b=*((float*)(buffer+222+4*i));
    }
    //get Y offsets
    for(i=0;i<16;i++){
        dest->osY[i].c.a=*((float*)(buffer+ 30+16*4+4*i));
        dest->osY[i].c.b=*((float*)(buffer+222+16*4+4*i));
    }
    //get Z offsets
    for(i=0;i<16;i++){
        dest->osZ[i].c.a=*((float*)(buffer+ 30+16*4*2+4*i));
        dest->osZ[i].c.b=*((float*)(buffer+222+16*4*2+4*i));
    }
    printf("Writing Corrections data\r\n");
    ret=write_correction_dat(idx,dest);
    if(ret==RET_SUCCESS){
        printf("Correction Data Written\r\n");
    }else{
        printf("Error writing correction data %i returned\r\n",ret);
    }
    //free buffer
    BUS_free_buffer();
    return 0;
}

int corchkCmd(char **argv,unsigned short argc){
    int i,ret;
    for(i=0;i<6;i++){
        ret=check_cor(i);
        if(ret==0){
            printf("%s correction data present\r\n",cor_axis_names[i]);
        }else{
            printf("%s corrections data is invalid\r\n",cor_axis_names[i]);
        }
    }
    return 0;
}
    
int dummycorCmd(char **argv,unsigned short argc){
    unsigned char *buffer=NULL;
    C_AXIS *dest;
    int ret;
    unsigned short check_c,check_s,all=0;
    int i,resp,idx;
    if(argc<1){
        printf("Error: too few arguments\r\n");
        return -1;
    }
    if(argc>1){
        printf("Error: too many arguments\r\n");
        return -2;
    }
    for(i=0,idx=-1;i<6;i++){
        if(!strcmp(argv[1],cor_axis_names[i])){
            idx=i;
            break;
        }
    }
    if(idx==-1){
        if(!strcmp(argv[1],"all")){
            all=1;
            idx=0;
        }
    }
    if(idx==-1){
        if(1!=sscanf(argv[1],"%u",&idx)){
            //print error
            printf("Error parsing index \"%s\"\r\n",argv[1]);
            return -3;
        }
    }
    //sanity check index
    if(idx>=6){
      printf("Error : index too large\r\n");
      return -5;
    }
    //get buffer, set a timeout of 2 secconds
    buffer=BUS_get_buffer(CTL_TIMEOUT_DELAY,2048);
    //check for error
    if(buffer==NULL){
        printf("Error : Timeout while waiting for buffer.\r\n");
        return -1;
    }
    dest=(C_AXIS*)(buffer);
    for(;idx<6;idx++){
        //zero all data
        memset(dest,0,sizeof(C_AXIS));
        //set scale factors to datasheet nominal
        dest->scl[0]=dest->scl[3]=1/(2*65535*1e-3*95.3);
        //set cross axis factors to +/-1%
        dest->scl[1]=-0.01*dest->scl[0];
        dest->scl[2]=0.01*dest->scl[3];
    
        printf("Writing dummy Corrections data for %s axis\r\n",cor_axis_names[idx]);
        ret=write_correction_dat(idx,dest);
        if(ret==RET_SUCCESS){
            printf("Correction Data Written\r\n");
        }else{
            printf("Error writing correction data %i returned\r\n",ret);
            break;
        }
        //exit loop if only erasing one axis
        if(!all)break;
    }
    //free buffer
    BUS_free_buffer();
    return 0;
}

int dumpcorCmd(char **argv,unsigned short argc){
    const C_AXIS *src;
    int i,idx;
    //check for too few arguments
    if(argc<1){
        printf("Error: too few arguments\r\n");
        return -1;
    }
    //check for too many arguments
    if(argc>1){
        printf("Error: too many arguments\r\n");
        return -2;
    }
    //look for symbolic axis name
    for(i=0,idx=-1;i<6;i++){
        if(!strcmp(argv[1],cor_axis_names[i])){
            idx=i;
            break;
        }
    }
    //check if axis name found
    if(idx==-1){
        //parse numerical axis name
        if(1!=sscanf(argv[1],"%u",&idx)){
            //print error
            printf("Error parsing index \"%s\"\r\n",argv[1]);
            return -3;
        }
    }
    //sanity check index
    if(idx>=6){
      printf("Error : index too large\r\n");
      return -5;
    }
    //check correction validity
    if(RET_SUCCESS!=check_cor(idx)){
        printf("%s corrections data is invalid\r\n",cor_axis_names[idx]);
    }
    //get pointer to source data
    src=&correction_data[idx].dat.cor;
    // print out scale factors
    for(i=0;i<4;i++){
        printf("scl[%i] = %E\r\n",i,src->scl[i]);
    }
    //print base offsets
    printf("Base Offsets:\r\n%.2f %.2f\r\n",src->baseOS.c.a,src->baseOS.c.b);
    
    printf("X offsets:\r\n");
    //print offsets for X torquers
    for(i=0;i<16;i++){
        printf("%.2f %.2f\r\n",src->osX[i].c.a,src->osX[i].c.b);
    }
    
    printf("Y offsets:\r\n");
    //print offsets for Z torquers
    for(i=0;i<16;i++){
        printf("%.2f %.2f\r\n",src->osY[i].c.a,src->osY[i].c.b);
    }
    
    printf("Z offsets:\r\n");
    //print offsets for Z torquers
    for(i=0;i<16;i++){
        printf("%.2f %.2f\r\n",src->osZ[i].c.a,src->osZ[i].c.b);
    }
    //DONE!
    return 0;
}

//correction test command given hypothetical mag values it 
int ctstCmd(char **argv,unsigned short argc){
    CPOINT meas;
    MAG_POINT mag;
    int idx=-1,i;
    char *end;
    //check number of arguments
    if(argc!=3){
        printf("Error : %s requires 3 arguments but %i given\r\n",argv[0],argc);
        return -1;
    }
    //figure out which axis is being usesd
    for(i=0;i<6;i++){
        if(!strcmp(argv[1],cor_axis_names[i])){
            idx=i;
            break;
        }
    }
    //check if symbolic name was found
    if(idx==-1){
        //read numeric name
        if(1!=sscanf(argv[1],"%u",&idx)){
            //print error
            printf("Error parsing axis \"%s\"\r\n",argv[1]);
            return -3;
        }
    }
    //parse first value
    mag.c.a=strtol(argv[2],&end,10);
    if(end==argv[2]){
        printf("Error : could not parse \"%s\"\r\n",argv[2]);
        return -4;
    }
    if(*end!=0){
        printf("Error : unknown suffix \"%s\" found while paresing \"%s\"\r\n",end,argv[2]);
        return -5;
    }
    //parse second value
    mag.c.b=strtol(argv[3],&end,10);
    if(end==argv[3]){
        printf("Error : could not parse \"%s\"\r\n",argv[3]);
        return -4;
    }
    if(*end!=0){
        printf("Error : unknown suffix \"%s\" found while paresing \"%s\"\r\n",end,argv[3]);
        return -5;
    }
    //print values and axis
    printf("Correcting measurements for the %s axis:\r\n%i %i\r\n",cor_axis_names[idx],mag.c.a,mag.c.b);
    //apply correction and check if full correction was applied
    if(RET_SUCCESS!=applyCor(&meas,&mag,idx)){
        //print warning
        printf("warning : invalid torquer status, calibration incomplete\r\n");
    }
    //check valididity of the correction
    if(RET_SUCCESS!=check_cor(idx)){
        //print warning
        printf("warning : invalid correction for %s axis\r\n",cor_axis_names[idx]);
    }
    //print measurments
    printf("Corrected measurements for the %s axis:\r\n%f %f\r\n",cor_axis_names[idx],meas.c.a,meas.c.b);
    return 0;
}

int magCmd(char **argv,unsigned short argc){
    const char *term=(output_type==MACHINE_OUTPUT)?"\t":"\r\n";
    int single=0,print_sdata=0,print_all=0,single_axis=-1,print_raw=0;
    unsigned short time=32768,count=0;
    int i,j,res,timeout=0;
    CTL_EVENT_SET_t e;
    extern MAG_DAT magData;
    CPOINT pt;
    unsigned char buff[BUS_I2C_HDR_LEN+3+BUS_I2C_CRC_LEN],*ptr;
    //parse arguments
    for(i=1;i<=argc;i++){
        if(!strcmp("single",argv[i])){
            single=1;
        }else if(!strcmp("sdata",argv[i])){
            print_sdata=1;
        }else if(!strcmp("all",argv[i])){
            print_all=1;
            print_sdata=1;
        }else if(!strcmp("raw",argv[i])){
            print_raw=1;
        }else{                        
            //look for symbolic axis name
            for(j=0;j<6;j++){
                if(!strcmp(argv[i],cor_axis_names[j])){
                    single_axis=j;
                    break;
                }
            }
            if(single_axis==-1){
                printf("Error Unknown argument \'%s\'.\r\n",argv[i]);
                return -1;
            }
        }
    }
    //set ACDS mode
    ACDS_mode=ACDS_COMMAND_MODE;
    //check if reading a single sample
    if(single){
        //read single measurement
        res=mag_sample_single(buff);
    }else{
        //start reading magnetometer data
        res=mag_sample_start(buff,time,count);
    }
    //check result
    if(res<0){
        printf("Error communicating with LEDL : %s\r\n",BUS_error_str(res));
        //return error
        return 1;
    }
    //refresh correction data status
    read_cor_stat();
    if(single){
        do{
            //wait for measurement
            e=ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&ACDS_evt,ADCS_EVD_COMMAND_SENSOR_READ,CTL_TIMEOUT_DELAY,2048);
            if(!e){
                //send packet again
                res=BUS_cmd_tx(BUS_ADDR_LEDL,buff,1,0,BUS_I2C_SEND_FOREGROUND);     
                //increse timeout count
                timeout++;
            }
        }while(!e && timeout<5);
        if(!e){
            printf("Error : timeout while waiting for sensor data\r\n");
            return 2;
        }
        if(single_axis==-1){
            //print out flux values   
            vecPrint("Flux",&acds_dat.dat.acds_dat.flux);
            //print out individual sensor data
            if(print_sdata){
                //print seperator
                if(output_type==HUMAN_OUTPUT){   
                    printf("========================================================================================\r\n");
                }
                //loop through all SPBs
                for(i=0;i<6;i++){
                    //check for valid measurements
                    if(magData.flags&(1<<(i*2)) && magData.flags&(1<<(i*2+1))){
                        if(print_raw){
                            printf(" %s : % i % i%s",cor_axis_names[i],magData.meas[i].c.a,magData.meas[i].c.b,term);
                        }else{
                            //check for correction data
                            if(cor_stat&(1<<i)){
                                //apply correction
                                applyCor(&pt,&magData.meas[i],i);
                                //print result
                                printf(" %s : %f %f%s",cor_axis_names[i],pt.c.a,pt.c.b,term);
                            }else{
                                //print error
                                printf(" %s : --- ---%s",cor_axis_names[i],term);
                            }
                        }
                    }else if(print_all){
                        //print error
                        printf(" %s : ### ###%s",cor_axis_names[i],term);
                    }
                }
            }
            if(output_type==MACHINE_OUTPUT){   
                printf("\r\n");
            }
        }else{
            if(print_raw){
                printf("% i\t% i\r\n",magData.meas[single_axis].c.a,magData.meas[single_axis].c.b);
            }else{
                applyCor(&pt,&magData.meas[single_axis],single_axis);
                //print result
                printf("%f\t%f\r\n",pt.c.a,pt.c.b);
            }
        }
    }else{
        printf("Reading Magnetometer, press any key to stop\r\n");
        //run while no keys pressed
        while(async_CheckKey()==EOF){
            //wait for data from LEDL
            e=ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&ACDS_evt,ADCS_EVD_COMMAND_SENSOR_READ,CTL_TIMEOUT_DELAY,1800);
            //check if data was received
            if(e&ADCS_EVD_COMMAND_SENSOR_READ){
                if(single_axis==-1){
                    vecPrint("Flux",&acds_dat.dat.acds_dat.flux);
                    //print out individual sensor data
                    if(print_sdata){ 
                        //print seperator
                        if(output_type==HUMAN_OUTPUT){   
                            printf("========================================================================================\r\n");
                        }
                        //loop through all SPBs
                        for(i=0;i<6;i++){
                            //check for valid measurements
                            if(magData.flags&(1<<(i*2)) && magData.flags&(1<<(i*2+1))){
                                if(print_raw){
                                    printf(" %s : % i % i%s",cor_axis_names[i],magData.meas[i].c.a,magData.meas[i].c.b,term);
                                }else{
                                    //check for correction data
                                    if(cor_stat&(1<<i)){
                                        //apply correction
                                        applyCor(&pt,&magData.meas[i],i);
                                        //print result
                                        printf(" %s : % f\t% f%s",cor_axis_names[i],pt.c.a,pt.c.b,term);
                                    }else{
                                        //print error
                                        printf(" %s : --- ---%s",cor_axis_names[i],term);
                                    }
                                }
                            }else if(print_all){
                                //print error
                                printf(" %s : ### ###%s",cor_axis_names[i],term);
                            }
                        }
                        //print seperator                   
                        if(output_type==HUMAN_OUTPUT){   
                            printf("========================================================================================\r\n");
                        }
                    }
                    if(output_type==MACHINE_OUTPUT){   
                        printf("\r\n");
                    }
                }else{
                    if(print_raw){
                        printf("% i\t% i\r\n",magData.meas[single_axis].c.a,magData.meas[single_axis].c.b);
                    }else{
                        applyCor(&pt,&magData.meas[single_axis],single_axis);
                        //print result
                        printf("%f\t%f\r\n",pt.c.a,pt.c.b);
                    }
                }
                //message recived, reduce timeout count
                if(timeout>-10){
                    timeout--;
                }
            }else{
                //message timeout, increase timeout count
                timeout+=4;
                //check if too many time outs have happened
                if(timeout>20){
                    //print error 
                    printf("Error : timeout while waiting for sensor data\r\n");            
                    //exit loop
                    break;
                }
                //if not aborting, print a warning
                printf("Warning : timeout while waiting for sensor data\r\n"); 
            }
        }
        //send stop sample command
        res=mag_sample_stop(buff);
        //check result
        if(res<0){
            printf("Error communicating with LEDL : %s\r\n",BUS_error_str(res));
            //return error
            return 1;
        }
        //clear event flag
        ctl_events_set_clear(&ACDS_evt,0,ADCS_EVD_COMMAND_SENSOR_READ);
        //wait for straggalers
        ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&ACDS_evt,ADCS_EVD_COMMAND_SENSOR_READ,CTL_TIMEOUT_DELAY,900);
    }
    return 0;
}

int modeCmd(char **argv,unsigned short argc){
    unsigned short time=32768,count=0,mode;
    int i,res,timeout=0;
    char *end;
    CTL_EVENT_SET_t e;
    unsigned char buff[BUS_I2C_HDR_LEN+3+BUS_I2C_CRC_LEN],*ptr;
    //check number of arguments
    if(argc!=1){
        printf("Error : %s requires 1 argument but %i given\r\n",argv[0],argc);
        return 1;
    }
    //parse arguments
    mode=strtoul(argv[1],&end,10);
    if(end==argv[1]){
      printf("Error : could not parse torquer number \'%s\'\r\n",argv[1]);
      return -1;
    }else if(*end!=0){
      printf("Error : unknown suffix \'%s\' on torque \'%s\'\r\n",end,argv[1]);
      return -2;
    }        
    //set last field value to NaN
    acds_dat.dat.acds_dat.flux.c.x=__float32_nan;
    acds_dat.dat.acds_dat.flux.c.y=__float32_nan;
    acds_dat.dat.acds_dat.flux.c.z=__float32_nan;
    //set ACDS mode
    ACDS_mode=mode;
    //send sample start command
    res=mag_sample_start(buff,time,count);
    //check result
    if(res<0){
        printf("Error communicating with LEDL : %s\r\n",BUS_error_str(res));
        //return error
        return 1;
    }
    //refresh correction data status
    read_cor_stat();
    //print message
    printf("Running ACDS in mode %i, press any key to stop\r\n",mode);
    //run while no keys pressed
    while(async_CheckKey()==EOF){
        //wait for data from LEDL
        e=ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&ACDS_evt,ADCS_EVD_COMMAND_SENSOR_READ,CTL_TIMEOUT_DELAY,1800);
        //check if data was received
        if(e&ADCS_EVD_COMMAND_SENSOR_READ){
            if(output_type==HUMAN_OUTPUT){
                printf("\r\n=========================================================================\r\n");
            }
            print_log_dat(&acds_dat.dat.acds_dat);
            //message recived, reduce timeout count
            if(timeout>-10){
                timeout--;
            }
        }else{
            //message timeout, increase timeout count
            timeout+=4;
            //check if too many time outs have happened
            if(timeout>20){
                //print error 
                printf("Error : timeout while waiting for sensor data\r\n");            
                //exit loop
                break;
            }
            //if not aborting, print a warning
            printf("Warning : timeout while waiting for sensor data\r\n"); 
        }
    }
    //send stop command
    res=mag_sample_stop(buff);
    //check result
    if(res<0){
        printf("Error communicating with LEDL : %s\r\n",BUS_error_str(res));
        //return error
        return 1;
    }
    return 0;
}

//erase corection data command
int erase_cor_Cmd(char **argv,unsigned short argc){
    int ret;
    unsigned short all=0;
    int i,resp,idx;
    if(argc<1){
        printf("Error: too few arguments\r\n");
        return -1;
    }
    if(argc>1){
        printf("Error: too many arguments\r\n");
        return -2;
    }
    for(i=0,idx=-1;i<6;i++){
        if(!strcmp(argv[1],cor_axis_names[i])){
            idx=i;
            break;
        }
    }
    if(idx==-1){
        if(!strcmp(argv[1],"all")){
            all=1;
            idx=0;
        }
    }
    if(idx==-1){
        if(1!=sscanf(argv[1],"%u",&idx)){
            //print error
            printf("Error parsing index \"%s\"\r\n",argv[1]);
            return -3;
        }
    }
    //sanity check index
    if(idx>=6){
      printf("Error : index too large\r\n");
      return -5;
    }
    for(;idx<6;idx++){
        ret=erase_correction_dat(idx);
        if(ret){
            printf("Error : failed to erase data for the %s axis\r\n",cor_axis_names[idx]);
        }else{
            printf("%s axis data erased\r\n",cor_axis_names[idx]);
        } 
        //exit loop if only erasing one axis
        if(!all)break;
    }
}

//test stat2idx function
int stat2idx_Cmd(char **argv,unsigned short argc){    
    int axis;
    if(argc<1){
        printf("Error : too few arguments\r\n");
        return -1;
    }
    if(argc>1){
        printf("Error : too many arguments\r\n");
        return -2;
    }
    axis=atoi(argv[1]);
    printf("Idx = %i\r\n",stat2Idx(axis));
    return 0;
}

int build_Cmd(char **argv,unsigned short argc){
    #ifdef DEV_BUILD
        printf("Development Board Build\r\n");
    #else
        printf("Standard build\r\n");
    #endif
}
    
int data_log_Cmd(char **argv,unsigned short argc){
    unsigned short num=0;
    unsigned long tmp;
    int resp;
    char *end;
    if(argc>=1){
        //check for actions
        if(!strcmp(argv[1],"clear")){
            resp=clear_log();
            if(resp){
                printf("Error erasing log : %s\r\n",SD_error_str(resp));
            }else{
                printf("Data log erase successfully!\r\n");
            }
            return 0;
        }
        //parse number
        tmp=strtoul(argv[1],&end,10);
        if(end==argv[1]){
            //print error
            printf("Error parsing num \"%s\"\r\n",argv[1]);
            return -1;
        }
        if(*end){
            printf("Error : unknown suffix \"%s\" while parsing num \"%s\".\r\n",end,argv[1]);
            return -2;
        }
        //saturate
        if(tmp>USHRT_MAX){
            num=USHRT_MAX;
        }else{
             num=tmp;
        }
    }
    log_replay(num);
}
    
//print number of the first available sector on the SD card
int first_free_sectorCmd(char **argv,unsigned short argc){
    printf("%li\r\n",(long)SD_FIRST_FREE_ADDR);
    return 0;
}

int blacklist_Cmd(char **argv,unsigned short argc){
    int i,j,found;
    enum{BLACKLIST_RM,BLACKLIST_ADD,BLACKLIST_SET,BLACKLIST_SHOW,BLACKLIST_CLEAR};
    int action=BLACKLIST_SHOW;
    short blacklist=0;
    unsigned char *buffer=NULL;
    ACDS_SETTINGS_STORE *tmp_settings;
    if(argc>0){
        if(!strcmp("rm",argv[1])){
            action=BLACKLIST_RM;
        }else if(!strcmp("add",argv[1])){
            action=BLACKLIST_ADD;
        }else if(!strcmp("set",argv[1])){
            action=BLACKLIST_SET;
        }else if(!strcmp("show",argv[1])){
            action=BLACKLIST_SHOW;
        }else if(!strcmp("clear",argv[1])){
            action=BLACKLIST_CLEAR;
        }else{
            printf("Error : unknown action %s\r\n",argv[1]);
            return 1;
        }
    }
    if(action!=BLACKLIST_SHOW){
        //check number of arguments
        //only BLACKLIST_CLEAR is alowed to have no extra arguments
        if(argc<2 && action!=BLACKLIST_CLEAR){
            printf("Error : too few arguments\r\n");
            return 1;
        }
        //parse arguments
        for(i=2;i<=argc;i++){
            //compare first two chars to get SPB
            for(j=0,found=0;j<12;j++){
                if(!strncmp(cor_axis_names[j],argv[i],2)){
                    found=1;
                    break;
                }
            }
            //check if SPB found
            if(!found){
                //check if all is given
                if(!strcmp("all",argv[i])){
                    blacklist|=(1<<12)-1;
                    continue;
                }
                //unknown axis, give error
                printf("Error : could not parse %s, unknown SPB\r\n",argv[i]);
                return 2;
            }
            if(argv[i][2]==0){
                //add whole SPB
                blacklist|=3<<(j*2);
            }else if(argv[i][2]=='a' && argv[i][3]==0){
                //add a-axis sensor
                blacklist|=1<<(j*2);
            }else if(argv[i][2]=='b' && argv[i][3]==0){
                blacklist|=1<<(j*2+1);
            }else{
                printf("Error : could not parse %s, unknown SPB axis\r\n",argv[i]);
                return 4;
            }
        }
        //get buffer, set a timeout of 2 secconds
        buffer=BUS_get_buffer(CTL_TIMEOUT_DELAY,2048);
        //check for error
        if(buffer==NULL){
            printf("Error : Timeout while waiting for buffer.\r\n");
            return -1;
        }
        //set temporary settings pointer
        tmp_settings=(ACDS_SETTINGS_STORE*)buffer;
        //copy settings into temp buffer
        memcpy(tmp_settings,&ACDS_settings,sizeof(ACDS_SETTINGS_STORE));
        //check action and set new blacklist
        switch(action){
            case BLACKLIST_RM:
                tmp_settings->dat.settings.blacklist&=~blacklist;
            break;
            case BLACKLIST_ADD:
                tmp_settings->dat.settings.blacklist|=blacklist;
            break;
            case BLACKLIST_SET:
                tmp_settings->dat.settings.blacklist=blacklist;
            break;
            case BLACKLIST_CLEAR:
                tmp_settings->dat.settings.blacklist=blacklist;
            break;
            default:
                printf("Internal Error : incorrect action %i\r\n",action);
                //free buffer
                BUS_free_buffer();  
                return 3;
        }
        //set magic
        tmp_settings->magic=ACDS_SETTINGS_MAGIC;
        //set CRC
        tmp_settings->crc=crc16((void*)&tmp_settings->dat,sizeof(ACDS_SETTINGS));
        //write values to flash
        if(flash_write((void*)&ACDS_settings,tmp_settings,sizeof(ACDS_SETTINGS_STORE))!=RET_SUCCESS){
            printf("Error : could not write settings\r\n");
            //free buffer
            BUS_free_buffer();  
            return -1;
        }
        //free buffer
        BUS_free_buffer();  
    }
    if(ACDS_settings.dat.settings.blacklist==0){
        //nothing in blacklist, print empty
        printf("empty");
    }else{
        //check blacklist for entries
        for(i=0;i<6;i++){
            if(ACDS_settings.dat.settings.blacklist&(1<<(2*i))){
                printf("%sa ",cor_axis_names[i]);
            }
            if(ACDS_settings.dat.settings.blacklist&(1<<(2*i+1))){
                printf("%sb ",cor_axis_names[i]);
            }
        }
    }
    printf("\r\n");
    return 0;
}

//gets a line from input and checks for overflow
char *getline(char *dest,size_t size){
    int i;
    for(i=0;i<size;i++){
        //get char from input
        dest[i]=getchar();
        //check for newline or null
        if(dest[i]=='\r' || dest[i]=='\n' || dest[i]=='\0'){
            //write null terminator
            dest[i]='\0';
            //exit loop
            return dest;
        }
    }
    return NULL;
}

//write filter to flash
void filter_write(FILTER_STORE *src){
    //compute CRC only on filter structure
    src->crc=crc16(&src->dat.filter,sizeof(IIR_FILTER));
    //write data to flash
    if(flash_write((void*)&bdot_filter,src,sizeof(FILTER_STORE))!=RET_SUCCESS){
        printf("Error : could not write filter\r\n");
    }
}

int filter_Cmd(char **argv,unsigned short argc){
    int i,checksum,checksum_rec;
    char buffer[FILTER_MAX_B*20],*ptr;
    unsigned short crc;
    FILTER_STORE tmp_filt;
    if(argc>0){
        //process arguments
        if(!strcmp("off",argv[1])){
            //coppy data to RAM
            memcpy(&tmp_filt,&bdot_filter,sizeof(FILTER_STORE));
            //turn filter off    
            tmp_filt.dat.filter.status=FILTER_OFF;
            //write new filter
            filter_write(&tmp_filt);
        }else if(!strcmp("on",argv[1])){
            //coppy data to RAM
            memcpy(&tmp_filt,&bdot_filter,sizeof(FILTER_STORE));
            //turn filter on
            tmp_filt.dat.filter.status=FILTER_ON;
            //write new filter
            filter_write(&tmp_filt);
        }else if(!strcmp("new",argv[1])){
            //upload new filter
            printf("Ready for filter upload na=%i nb=%i\r\n",FILTER_MAX_A,FILTER_MAX_B);
            //get line check if buffer overflowed
            if(!getline(buffer,sizeof(buffer))){
                //print error
                printf("Error : buffer overflow\r\n");
                //exit
                return -2;
            }
            //clear errno
            errno=0;
            //parse data
            for(i=0,ptr=buffer;i<FILTER_MAX_A;i++){
                tmp_filt.dat.filter.a[i]=strtof(ptr,&ptr);
                //check for end
                if(*ptr=='\0'){
                    break;
                }
                //skip comma seperator
                if(*ptr!=','){
                    break;
                }
                ptr++;
            }
            //check if all elements were parsed
            if((i+1)!=FILTER_MAX_A){
                printf("Error : too few elements in a. found %i expected %i\r\n",i,FILTER_MAX_A);
                return -4;
            }
            
            //get line check if buffer overflowed
            if(!getline(buffer,sizeof(buffer))){
                //print error
                printf("Error : buffer overflow\r\n");
                //exit
                return -2;
            }
            //parse data
            for(i=0,ptr=buffer;i<FILTER_MAX_B;i++){
                tmp_filt.dat.filter.b[i]=strtof(ptr,&ptr);
                //check for end
                if(*ptr=='\0'){
                    break;
                }
                //skip comma seperator
                if(*ptr!=','){
                    break;
                }
                ptr++;
            }
            //check if all elements were parsed
            if((i+1)!=FILTER_MAX_B){
                printf("Error : too few elements in b. found %i expected %i\r\n",i,FILTER_MAX_B);
                return -4;
            }
            //check if an error occured
            if(errno){
                printf("Error : could not parse data %i returned\r\n",errno);
                return -3;
            }
            
            //get line check if buffer overflowed
            if(!getline(buffer,sizeof(buffer))){
                //print error
                printf("Error : buffer overflow\r\n");
                //exit
                return -2;
            }
            //parse filter orders
            if(2!=sscanf(buffer,"na = %u nb = %u",&tmp_filt.dat.filter.na,&tmp_filt.dat.filter.nb)){
                //print error
                printf("Error : unable to parse filter orders\r\n");
                return -5;
            }
            //check filter orders
            if(tmp_filt.dat.filter.na>FILTER_MAX_A){
                printf("Error : na is too large\r\n");
                return -6;
            }
            
            if(tmp_filt.dat.filter.nb>FILTER_MAX_B){
                printf("Error : nb is too large\r\n");
                return -6;
            }
            
            //success!! fill in values in structure
            tmp_filt.magic=FILTER_MAGIC;
            tmp_filt.dat.filter.status=bdot_filter.dat.filter.status;
            filter_write(&tmp_filt);
            //print success
            printf("Filter transfer complete!\r\n");
            //done exit so we don't print
            return 0;
        }else if(!strcmp("show",argv[1])){
            //fall through to printing
        }else{
            printf("Error : unknown argument \"%s\"\r\n",argv[1]);
            return 1;
        }
    }
    //check filter magic
    if(bdot_filter.magic!=FILTER_MAGIC){
        printf("Filter magic incorrect. Read %04X expected %04X\r\n",bdot_filter.magic,FILTER_MAGIC);
    }
    //check crc
    crc=crc16(&bdot_filter.dat.filter,sizeof(IIR_FILTER));
    if(bdot_filter.crc!=crc){
        printf("Filter CRC incorrect. Read %04X calculated %04X\r\n",bdot_filter.crc,crc);
    }
    //print filter status
    switch(bdot_filter.dat.filter.status){
        case FILTER_ON:
            printf("Filter is on\r\n");
        break;    
        case FILTER_OFF:
            printf("Filter is off\r\n");
        break;    
        default:
            printf("unknown filter status = %i\r\n",bdot_filter.dat.filter.status);
        break;
    }
    //print filter coefficents
    printf("a[%i] = ",bdot_filter.dat.filter.na);
    for(i=0;i<bdot_filter.dat.filter.na && i<FILTER_MAX_A;i++){
        printf("%f\t",bdot_filter.dat.filter.a[i]);
    }
    printf("\r\n""b[%i] = ",bdot_filter.dat.filter.nb);
    for(i=0;i<bdot_filter.dat.filter.nb && i<FILTER_MAX_B;i++){
        printf("%f\t",bdot_filter.dat.filter.b[i]);
    }
    printf("\r\n");
    return 0;
}
    

//table of commands with help
const CMD_SPEC cmd_tbl[]={{"help"," [command]\r\n\t""get a list of commands or help on a spesific command.",helpCmd},
                     CTL_COMMANDS,ARC_COMMANDS,ERROR_COMMANDS,
                     #ifndef DEV_BUILD
                     MMC_COMMANDS,MMC_DREAD_COMMAND,
                     #endif
                     {"flip","[X Y Z]\r\n\t""Flip a torquer in each axis.",flipCmd},
                     {"setTorque"," Xtorque Ytorque Ztorque\r\n\tFlip torquers to set the torque in the X, Y and Z axis",setTorqueCmd},
                     {"drive"," axis num dir\r\n\tdrive a torquer in the given axis in a given direction",driveCmd},
                     {"tqstat","\r\n\t""Print torquer status",tqstatCmd},
                     {"statcode","\r\n\t""Print torquer status in machine readable form",statcodeCmd},
                     {"init","\r\n\t""initialize torquers",initCmd},
                     {"reinit","\r\n\t""Set torquers to initialized state",reinitCmd},
                     {"comp","\r\n\t""print feedback comparitor status",compCmd},
                     {"tst","\r\n\t""axis num dir\r\n\t""do a test flip of given torquer to see if it is connected",tstCmd},
                     {"srun","[time count]\r\n\t""tell LEDL to start taking sensor data.",sensorRunCmd},
                     {"sstop","\r\n\t""tell LEDL to stop taking sensor data.",sensorStopCmd},
                     {"gain","type [g1 g2 g3]\r\n\t""set gain of algorithm",gainCmd},
                     {"setpoint","s1 s2 s3\r\n\t""get/set setpoint for rates",setpointCmd},
                     {"log","[level]\r\n\t""get/set log level",logCmd},
                     {"clrerr","\r\n\t""Clear error LED",clrErrCmd},
                     {"output","[output type]\r\n\tchange output between human and machine readable",outputTypeCmd},
                     {"tstIGRF","\r\n\t""Test IGRF conversion",tst_IGRF_cmd},
                     {"shval3","year alt lat long\r\n\t""IGRF conversion",shval3Cmd},
                     {"status","\r\n\t""Print status information",statusCmd},
                     {"rndt","\r\n\t""Set torquers to random torque",randomTorqueCmd},
                     {"unpack","sector""\r\n\t""Unpack calibration/correction data stored in a given sector",unpackCmd},
                     {"corchk","\r\n\t""Check correction data for all axis\r\n",corchkCmd},
                     {"dummycor","idx""\r\n\t""write corrections data for the given index",dummycorCmd},
                     {"dcor","idx""\r\n\t""write corrections data for the given index",dumpcorCmd},
                     {"ctst","axis aval bval""\r\n\t""apply corrections to a set of measurments",ctstCmd},
                     {"mag","[raw single]""\r\n\t""read data from magnetomiters",magCmd},
                     {"mode","mode""\r\n\t""run ACDS in given mode",modeCmd},
                     {"ecor","idx""\r\n\t""erase correction data for the given SPB",erase_cor_Cmd},
                     {"stat2idx","axis""\r\n\t""test stat2idx function",stat2idx_Cmd},
                     {"build","""\r\n\t""print build",build_Cmd},
                     {"dlog","[num]""\r\n\t""replay log data",data_log_Cmd},
                     {"ffsector","""\r\n\t""Print the address of the first free sector on the SD card",first_free_sectorCmd},
                     {"blacklist","[rm|add|set|show] ""\r\n\t""show/edit SPB blacklist",blacklist_Cmd},
                     {"filter","[show|on|off|new] ""\r\n\t""Edit filter and filter settings",filter_Cmd},
                     //end of list
                     {NULL,NULL,NULL}};
