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
#include "torquers.h"
#include "output_type.h"
#include "SensorDataInterface.h"
#include "algorithm.h"
#include "stackcheck.h"
#include "ACDS.h"
#include "LED.h"
#include "IGRF/igrf.h"
#include "corrections.h"


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
  
  ptr=BUS_cmd_init(buff,CMD_MAG_SAMPLE_CONFIG);
  //set command
  *ptr++=MAG_SAMPLE_START;
  //set time MSB
  *ptr++=time>>8;
  //set time LSB
  *ptr++=time;
  //set count
  *ptr++=count;
  //send packet
  res=BUS_cmd_tx(BUS_ADDR_LEDL,buff,4,0,BUS_I2C_SEND_FOREGROUND);
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
  ptr=BUS_cmd_init(buff,CMD_MAG_SAMPLE_CONFIG);
  //set command
  *ptr++=MAG_SAMPLE_STOP;
  //send packet
  res=BUS_cmd_tx(BUS_ADDR_LEDL,buff,1,0,BUS_I2C_SEND_FOREGROUND);
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
  printf("Status Size = %u\r\n",sizeof(ACDS_STAT));
  printf("Quat Size = %u\r\n",sizeof(QUAT));
  printf("Vec Size = %u\r\n",sizeof(VEC));
  //print status data
  printf   ("mag     \t%i, %i, %i\r\n",status.mag[0],status.mag[1],status.mag[2]);
  printf   ("gyro    \t%i, %i, %i\r\n",status.gyro[0],status.gyro[1],status.gyro[2]);
  printf   ("tqstat  \t0x%02X, 0x%02X, 0x%02X\r\n",status.tqstat[0],status.tqstat[1],status.tqstat[2]);
  printf   ("flips   \t%u, %u, %u\r\n",status.flips[0],status.flips[1],status.flips[2]);
  printf   ("flags   \t0x%04X\r\n",status.flags);
  iquatPrint("attitude",&status.attitude);
  vecPrint ("rates   ",&status.rates);
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

const char *(cor_axis_names[])={"X+","X-","Y+","Y-","Z+","Z-"};

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
    for(i=0,idx=-1;i<6;i++){
        if(!strcmp(argv[1],cor_axis_names[i])){
            idx=i;
            break;
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
    mag.c.a=strtof(argv[2],&end);
    if(end==argv[2]){
        printf("Error : could not parse \"%s\"\r\n",argv[2]);
        return -4;
    }
    if(*end!=0){
        printf("Error : unknown suffix \"%s\" found while paresing \"%s\"\r\n",end,argv[2]);
        return -5;
    }
    //parse second value
    mag.c.b=strtof(argv[3],&end);
    if(end==argv[3]){
        printf("Error : could not parse \"%s\"\r\n",argv[3]);
        return -4;
    }
    if(*end!=0){
        printf("Error : unknown suffix \"%s\" found while paresing \"%s\"\r\n",end,argv[3]);
        return -5;
    }
    //print values and axis
    printf("Correcting measurements for the %s axis:\r\n%f %f\r\n",cor_axis_names[idx],mag.c.a,mag.c.b);
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
    int single=0;
    unsigned short time=32768,count=0;
    int i,res;
    CTL_EVENT_SET_t e;
    unsigned char buff[BUS_I2C_HDR_LEN+3+BUS_I2C_CRC_LEN],*ptr;
    //parse arguments
    for(i=1;i<=argc;i++){
        if(!strcmp("single",argv[i])){
            single=1;
        }else{
            printf("Error Unknown argument \'%s\'.\r\n",argv[i]);
            return -1;
        }
    }
    //set ACDS mode
    ACDS_mode=ACDS_COMMAND_MODE;
    //setup command
    ptr=BUS_cmd_init(buff,CMD_MAG_SAMPLE_CONFIG);
    //check if reading a single sample
    if(single){
        //set command
        *ptr++=MAG_SINGLE_SAMPLE;
        //send packet
        res=BUS_cmd_tx(BUS_ADDR_LEDL,buff,1,0,BUS_I2C_SEND_FOREGROUND);
    }else{
        //set command
        *ptr++=MAG_SAMPLE_START;
        //set time MSB
        *ptr++=time>>8;
        //set time LSB
        *ptr++=time;
        //set count
        *ptr++=count;
        //send packet
        res=BUS_cmd_tx(BUS_ADDR_LEDL,buff,4,0,BUS_I2C_SEND_FOREGROUND);
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
        //wait for measurement
        e=ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&ACDS_evt,ADCS_EVD_COMMAND_SENSOR_READ,CTL_TIMEOUT_DELAY,2048);
        if(!e){
            printf("Error : Timeout while waiting for data\r\n");
            return 2;
        }
    }else{
        printf("Reading Magnetometer, press any key to stop\r\n");
        //get keypress
        getchar();
        //setup command
        ptr=BUS_cmd_init(buff,CMD_MAG_SAMPLE_CONFIG);
        //set command
        *ptr++=MAG_SAMPLE_STOP;
        //send packet
        res=BUS_cmd_tx(BUS_ADDR_LEDL,buff,1,0,BUS_I2C_SEND_FOREGROUND);
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
    //set ACDS mode
    ACDS_mode=mode;
    //setup command
    ptr=BUS_cmd_init(buff,CMD_MAG_SAMPLE_CONFIG);
    //set command
    *ptr++=MAG_SAMPLE_START;
    //set time MSB
    *ptr++=time>>8;
    //set time LSB
    *ptr++=time;
    //set count
    *ptr++=count;
    //send packet
    res=BUS_cmd_tx(BUS_ADDR_LEDL,buff,4,0,BUS_I2C_SEND_FOREGROUND);
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
            printf("\r\n=========================================================================\r\n");
            print_acds_dat(&acds_dat);
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
    //setup command
    ptr=BUS_cmd_init(buff,CMD_MAG_SAMPLE_CONFIG);
    //set command
    *ptr++=MAG_SAMPLE_STOP;
    //send packet
    res=BUS_cmd_tx(BUS_ADDR_LEDL,buff,1,0,BUS_I2C_SEND_FOREGROUND);
    //check result
    if(res<0){
        printf("Error communicating with LEDL : %s\r\n",BUS_error_str(res));
        //return error
        return 1;
    }
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
                     //end of list
                     {NULL,NULL,NULL}};
