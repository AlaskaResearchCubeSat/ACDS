#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <msp430.h>
#include <ctl_api.h>
#include <terminal.h>
#include <ARCbus.h>
#include <Error.h>
#include "torquers.h"
#include "output_type.h"
#include "SensorDataInterface.h"
#include "algorithm.h"
#include "stackcheck.h"
#include "ACDS.h"
#include "LED.h"

int current_set=TQ_SET_BIG;

//helper function to parse I2C address
//if res is true reject reserved addresses
unsigned char getI2C_addr(char *str,short res){
  unsigned long addr;
  unsigned char tmp;
  char *end;
  //attempt to parse a numeric address
  addr=strtol(str,&end,0);
  //check for errors
  if(end==str){
    //check for symbolic matches
    if(!strcmp(str,"LEDL")){
      return BUS_ADDR_LEDL;
    }else if(!strcmp(str,"ACDS")){
      return BUS_ADDR_ACDS;
    }else if(!strcmp(str,"COMM")){
      return BUS_ADDR_COMM;
    }else if(!strcmp(str,"IMG")){
      return BUS_ADDR_IMG;
    }else if(!strcmp(str,"CDH")){
      return BUS_ADDR_CDH;
    }else if(!strcmp(str,"GC")){
      return BUS_ADDR_GC;
    }
    //not a known address, error
    printf("Error : could not parse address \"%s\".\r\n",str);
    return 0xFF;
  }
  if(*end!=0){
    printf("Error : unknown sufix \"%s\" at end of address\r\n",end);
    return 0xFF;
  }
  //check address length
  if(addr>0x7F){
    printf("Error : address 0x%04lX is not 7 bits.\r\n",addr);
    return 0xFF;
  }
  //check for reserved address
  tmp=0x78&addr;
  if((tmp==0x00 || tmp==0x78) && res){
    printf("Error : address 0x%02lX is reserved.\r\n",addr);
    return 0xFF;
  }
  //return address
  return addr;
}

//set priority for tasks on the fly
int priorityCmd(char **argv,unsigned short argc){
  extern CTL_TASK_t *ctl_task_list;
  int i,found=0;
  CTL_TASK_t *t=ctl_task_list;
  if(argc<1 || argc>2){
    printf("Error: %s takes one or two arguments, but %u are given.\r\n",argv[0],argc);
    return -1;
  }
  while(t!=NULL){
    if(!strcmp(t->name,argv[1])){
      found=1;
      //match found, break
      break;
    }
    t=t->next;
  }
  //check that a task was found
  if(found==0){
      //no task found, return
      printf("Error: could not find task named %s.\r\n",argv[1]);
      return -3;
  }
  //print original priority
  printf("\"%s\" priority = %u\r\n",t->name,t->priority);
  if(argc==2){
      unsigned char val=atoi(argv[2]);
      if(val==0){
        printf("Error: invalid priority.\r\n");
        return -2;
      }
      //set priority
      ctl_task_set_priority(t,val);
      //print original priority
      printf("new \"%s\" priority = %u\r\n",t->name,t->priority);
  }
  return 0;
}

//get/set ctl_timeslice_period
int timesliceCmd(char **argv,unsigned short argc){
  if(argc>1){
    printf("Error: too many arguments.\r\n");
    return 0;
  }
  //if one argument given then set otherwise get
  if(argc==1){
    int en;
    CTL_TIME_t val=atol(argv[1]);
    //check value
    if(val==0){
      printf("Error: bad value.\r\n");
      return -1;
    }
    //disable interrupts so that opperation is atomic
    en=ctl_global_interrupts_set(0);
    ctl_timeslice_period=val;
    ctl_global_interrupts_set(en);
  }
  printf("ctl_timeslice_period = %ul\r\n",ctl_timeslice_period);
  return 0;
}

//return state name
const char *stateName(unsigned char state){
  switch(state){
    case CTL_STATE_RUNNABLE:
      return "CTL_STATE_RUNNABLE";
    case CTL_STATE_TIMER_WAIT:
      return "CTL_STATE_TIMER_WAIT";
    case CTL_STATE_EVENT_WAIT_ALL:
      return "CTL_STATE_EVENT_WAIT_ALL";
    case CTL_STATE_EVENT_WAIT_ALL_AC:
      return "CTL_STATE_EVENT_WAIT_ALL_AC";
    case CTL_STATE_EVENT_WAIT_ANY:
      return "CTL_STATE_EVENT_WAIT_ANY";
    case CTL_STATE_EVENT_WAIT_ANY_AC:
      return "CTL_STATE_EVENT_WAIT_ANY_AC";
    case CTL_STATE_SEMAPHORE_WAIT:
      return "CTL_STATE_SEMAPHORE_WAIT";
    case CTL_STATE_MESSAGE_QUEUE_POST_WAIT:
      return "CTL_STATE_MESSAGE_QUEUE_POST_WAIT";
    case CTL_STATE_MESSAGE_QUEUE_RECEIVE_WAIT:
      return "CTL_STATE_MESSAGE_QUEUE_RECEIVE_WAIT";
    case CTL_STATE_MUTEX_WAIT:
      return "CTL_STATE_MUTEX_WAIT";
    case CTL_STATE_SUSPENDED:
      return "CTL_STATE_SUSPENDED";
    default:
      return "unknown state";
  }
}

//print the status of all tasks in a table
int statsCmd(char **argv,unsigned short argc){
  extern CTL_TASK_t *ctl_task_list;
  int i;
  CTL_TASK_t *t=ctl_task_list;
  //format string
  const char *fmt="%-10s\t%u\t\t%c%-28s\t%lu\r\n";
  //print out nice header
  printf("\r\nName\t\tPriority\tState\t\t\t\tTime\r\n--------------------------------------------------------------------\r\n");
  //loop through tasks and print out info
  while(t!=NULL){
    printf(fmt,t->name,t->priority,(t==ctl_task_executing)?'*':' ',stateName(t->state),t->execution_time);
    t=t->next;
  }
  //add a blank line after table
  printf("\r\n");
  return 0;
}

//print current time
int timeCmd(char **argv,unsigned short argc){
  printf("time ticker = %li\r\n",get_ticker_time());
  return 0;
}


//get torque from string
float readTorque(const char*tstr){
  //only consider 1 char long strings
  if(strlen(tstr)!=1){
    return 0;
  }
  switch(tstr[0]){
    case '+':
      return 1;
    case '-':
      return -1;
    case '0':
      return 0;
    default:
      //unknown torque
      return 0;
  }
}

//Set torque in each axis
int setTorqueCmd(char **argv,unsigned short argc){
  VEC T;
  // three arguments are accepted, X axis, Y axis and Z axis torques
  if(argc!=3){
    printf("Error : %s requires 3 arguments\r\n",argv[0]);
    return -1;
  }
  //get torques
  T.c.x=readTorque(argv[1]);
  T.c.y=readTorque(argv[2]);
  T.c.z=readTorque(argv[3]);
  //print old status
  printf("Previous Torquer Status:\r\n");
  print_torquer_status(current_set);
  //current_set torques
  setTorque(&T,current_set);
  //print status
  printf("New Torquer Status:\r\n");
  print_torquer_status(current_set);
  return 0;
}

//flip a torquer in each axis
int flipCmd(char **argv,unsigned short argc){
  int num[3]={0,0,0},dir[3]={0,0,0};
  int i,set=TQ_SET_BIG;
  const char axis[]={'X','Y','Z'};
  // three arguments are accepted, X axis, Y axis and Z axis torques
  if(argc!=3){
    printf("Error : %s requires 3 arguments\r\n",argv[0]);
    return -1;
  }
  //get torquers
  for(i=0;i<3;i++){
    //get torquer number
    if(strlen(argv[i+1]+1)!=1){
      printf("Error parsing torquer number \"%s\" in %c-axis\r\n",argv[i+1],axis[i]);
      return -1;
    }
    if(argv[i+1][1]=='1'){
      num[i]=1;
    }else if(argv[i+1][1]=='2'){
      num[i]=2;
    }else if(argv[i+1][1]=='0'){
      num[i]=0;    ///no torquer to flip
    }else{
      printf("Error : \'%s\' is not a valid torquer number for %c-axis\r\n",argv[i+1]+1,axis[i]);
      return -1;
    }
    //get torquer direction
    if(argv[i+1][0]=='+'){
      dir[i]=M_PLUS;
    }else if(argv[i+1][0]=='-'){
      dir[i]=M_MINUS;
    }else if(argv[i+1][0]=='0' && num[i]==0){
      dir[i]=0;
    }else{
      printf("Error : \'%c\' is not a valid torquer direction for %c-axis\r\n",argv[3][0],axis[i]);
      return -1;
    }
    
  }
  if(output_type==HUMAN_OUTPUT){
    //print old status
    printf("Previous Torquer Status:\r\n");
    print_torquer_status(current_set);
  }
  //drive torquers 
  drive_torquers(current_set,num,dir);
  if(output_type==HUMAN_OUTPUT){
    //print status
    printf("New Torquer Status:\r\n");
    print_torquer_status(current_set);
  }else{
    print_torquer_stat_code(current_set);
    printf("\r\n");
  }
  return 0;
}

//drive a torquer
int driveCmd(char **argv,unsigned short argc){
  int num[3]={0,0,0},dir[3]={0,0,0};
  int axis=0;
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
  print_torquer_status(current_set);  
  //drive torquer 
  drive_torquers(current_set,num,dir);
  //print new status
  printf("New Torquer Status:\r\n");
  print_torquer_status(current_set);
  return 0;
}

int tqsetCmd(char **argv,unsigned short argc){
  if(argc>2){
    printf("Error : %s requires 1 or 2 arguments\r\n",argv[0]);
    return -1;
  }
  if(argc==1){
    if(!strcmp(argv[1],"B")){
        current_set=TQ_SET_BIG;
    }else if(!strcmp(argv[1],"S")){
      current_set=TQ_SET_SMALL;
    }else{
      printf("Error : could not parse argument \"%s\"\r\n",argv[1]);
      return -2;
    }
  }
  switch(current_set){
    case TQ_SET_SMALL:
      printf("\t""Small Torquers\r\n");
    break;
    case TQ_SET_BIG:
      printf("\t""Large Torquers\r\n");
    break;
    default:
    //this should never happen
      printf("Error : unknown torquer set.\r\n");
      return -1;
  }
  return 0;
}

int tqstatCmd(char **argv,unsigned short argc){
  int set=TQ_SET_NONE,i;
  if(argc>2){
    printf("Error : %s requires 1 or 2 arguments\r\n",argv[0]);
    return -1;
  }
  if(argc==1){
    if(!strcmp(argv[1],"B")){
        set=TQ_SET_BIG;
    }else if(!strcmp(argv[1],"S")){
        set=TQ_SET_SMALL;
    }else if(!strcmp(argv[1],"current")){
        set=TQ_SET_NONE;
    }else if(!strcmp(argv[1],"all")){
        set=TQ_SET_ALL;
    }else{
      printf("Error : could not parse argument \"%s\"\r\n",argv[1]);
      return -2;
    }
  }
  if(set==TQ_SET_ALL){
    for(i=0,set=TQ_SET_BIG;i<2;i++,set=TQ_SET_SMALL){
      print_torquer_status(set);
    }
  }else{
    //check if printing current set
    if(set==TQ_SET_NONE){
      set=current_set;
    }
    //print torquer status
    print_torquer_status(set);
  }
  return 0;
}

int initCmd(char **argv,unsigned short argc){
  torqueInit();
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
  extern TQ_SET tq_big,tq_small;
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
  if(current_set==TQ_SET_BIG){
    dir[axis]=(tq_big.elm[axis].status>>(num[axis]-1))&1;
  }else{
    dir[axis]=(tq_small.elm[axis].status>>(num[axis]-1))&1;
  }
  //Toggle direction
  if(dir[axis]==0){
    //Torquer was minus, set to plus
    dir[axis]=M_PLUS;
  }else{
    //torquer was plus, set to minus
    dir[axis]=M_MINUS;
  }
  //drive torquer 
  err=drive_torquers(current_set,num,dir);
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

//magnetomitor interface
int magCmd(char **argv,unsigned short argc){
  printf("TODO: write \"%s\" command\r\n",argv[0]);
  return -1;
}

enum{CAL_LOAD,CAL_DUMP,CAL_WRITE,CAL_TST};
  
float magCal[4];
float torqueCal[64];  

//calibration data command
/*int calCmd(char **argv,unsigned short argc){
  int action;
  if(argc!=1){
    printf("Error : %s requires only one argument\r\n",argv[0]);
    return -1;
  }
  if(!strcmp(argv[1],"load")){
    action=CAL_LOAD;
  }else if(!strcmp(argv[1],"dump")){
    action=CAL_DUMP;
  }else if(!strcmp(argv[1],"write")){
    action=CAL_DUMP;
  }else if(!strcmp(argv[1],"test")){
    action=CAL_DUMP;
  }else{
    printf("Unknown action \"%s\"\r\n",argv[1]);
    return -2;
  }
  return 0;
}*/


//set which errors are logged
int logCmd(char **argv,unsigned short argc){
  const unsigned char logLevels[]={ERR_LEV_DEBUG,ERR_LEV_INFO,ERR_LEV_WARNING,ERR_LEV_ERROR,ERR_LEV_CRITICAL};
  const char *(levelNames[])={"debug","info","warn","error","critical"};
  int found,i;
  unsigned char level;
  //check for too many arguments
  if(argc>1){
    printf("Error : %s takes 0 or 1 arguments\r\n",argv[0]);
    return -1;
  }
  //check if argument given
  if(argc==1){
    if(!strcmp("levels",argv[1])){
      //print a list of level names
      for(i=0;i<sizeof(logLevels)/sizeof(logLevels[0]);i++){
        printf("% 3u - %s\r\n",logLevels[i],levelNames[i]);
       }
       return 0;
    }
    //check for matching level names
    for(i=0;i<sizeof(logLevels)/sizeof(logLevels[0]);i++){
      if(!strcmp(levelNames[i],argv[1])){
        //match found
        found=1;
        //set log level
        level=logLevels[i];
        //done
        break;
      }
    }
    //check if there was a matching name
    if(!found){
      //get convert to integer
      level=atoi(argv[1]);
    }
    //set log level
    set_error_level(level);
  }
  //print (new) log level
  printf("Log level = %u\r\n",get_error_level());
  return 0;
}


int stackCmd(char **argv,unsigned short agrc);

int clearCmd(char **argv,unsigned short argc){
  resetTorqueStatus();
  return 0;
}

int calCmd(char **argv,unsigned short argc){
  unsigned short time=32768,count=0;
  unsigned char buff[BUS_I2C_HDR_LEN+3+BUS_I2C_CRC_LEN],*ptr;
  int res;
  unsigned int e;
  VEC T={0,0,0};
  extern int cal_stat;
  //reset calibration state
  cal_stat=0;
  ACDS_mode=ACDS_CAL_MODE;
  //reset torquers to initial status
  resetTorqueStatus();

  setTorque(&T,TQ_SET_BIG);
  setTorque(&T,TQ_SET_BIG);
  //wait
  ctl_timeout_wait(ctl_get_current_time()+2048);
  //send sample command
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
  //wait until calibration is complete before returning
  e=ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&ACDS_evt,ADCS_EVT_CAL_COMPLETE,CTL_TIMEOUT_DELAY,2500000);
  if(e!=ADCS_EVT_CAL_COMPLETE){
    printf("Error : timeout\r\n");
  }else{
    printf("Calibration complete\r\n");
  }
  //setup for sending stop command
  ptr=BUS_cmd_init(buff,CMD_MAG_SAMPLE_CONFIG);
  //set command
  *ptr++=MAG_SAMPLE_STOP;
  //send packet
  res=BUS_cmd_tx(BUS_ADDR_LEDL,buff,1,0,BUS_I2C_SEND_FOREGROUND);
  //check result
  if(res<0){
    printf("Error stopping data collection : %s\r\n",BUS_error_str(res));
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

//print the status of each tasks stack
int stackstatsCmd(char **argv,unsigned short argc){
  extern CTL_TASK_t *ctl_task_list;
  int i;
  CTL_TASK_t *t=ctl_task_list;
  //format string
  const char *fmt="%-10s\t%lp\t%lp\t%li\r\n";
  //print out nice header
  printf("\r\nName\tPointer\tStart\tRemaining\r\n--------------------------------------------------------------------\r\n");
  //loop through tasks and print out info
  while(t!=NULL){
    printf(fmt,t->name,t->stack_pointer,t->stack_start,t->stack_pointer-t->stack_start);
    t=t->next;
  }
  //add a blank line after table
  printf("\r\n");
  return 0;
}

int replayCmd(char **argv,unsigned short argc){
  error_log_replay();
  return 0;
}

//table of commands with help
const CMD_SPEC cmd_tbl[]={{"help"," [command]\r\n\t""get a list of commands or help on a spesific command.",helpCmd},
                     {"priority"," task [priority]\r\n\t""Get/set task priority.",priorityCmd},
                     {"timeslice"," [period]\r\n\t""Get/set ctl_timeslice_period.",timesliceCmd},
                     {"stats","\r\n\t""Print task status",statsCmd},
                     {"stack","\r\n\t""Print task stack status",stackstatsCmd},
                     {"time","\r\n\t""Return current time.",timeCmd},
                     {"flip","[X Y Z]\r\n\t""Flip a torquer in each axis.",flipCmd},
                     {"setTorque"," Xtorque Ytorque Ztorque\r\n\tFlip torquers to set the torque in the X, Y and Z axis",setTorqueCmd},
                     {"drive"," axis num dir\r\n\tdrive a torquer in the given axis in a given direction",driveCmd},
                     {"tqset","[B|S]\r\n\t""Set/Get current torquer set",tqsetCmd},
                     {"tqstat","[B|S|all|current]\r\n\t""Get torquer status",tqstatCmd},
                     {"init","\r\n\t""initialize torquers",initCmd},
                     {"comp","\r\n\t""print feedback comparitor status",compCmd},
                     {"tst","\r\n\t""axis num dir\r\n\t""do a test flip of given torquer to see if it is connected",tstCmd},
                     {"srun","[time count]\r\n\t""tell LEDL to start taking sensor data.",sensorRunCmd},
                     {"sstop","\r\n\t""tell LEDL to stop taking sensor data.",sensorStopCmd},
                     {"gain","type [g1 g2 g3]\r\n\t""set gain of algorithm",gainCmd},
                     {"mag","\r\n\t""Read From The magnetomitor",magCmd},
                     {"cal","load|dump|write|test\r\n\t""do things with the magnetometer calibration",calCmd},
                     {"log","[level]\r\n\t""get/set log level",logCmd},
                     {"stack","\r\n\t""",stackCmd},
                     {"clear","\r\n\t""",clearCmd},
                     {"clrerr","\r\n\t""Clear error LED",clrErrCmd},
                     {"output","[output type]\r\n\tchange output between human and machine readable",outputTypeCmd},
                     {"clear","\r\n\t""Clear all saved errors on the SD card",clearCmd},
                     {"replay","\r\n\t""Replay errors from log",replayCmd},
                     //end of list
                     {NULL,NULL,NULL}};