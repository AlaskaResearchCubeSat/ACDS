#include <msp430.h>
#include <stdio.h>
#include <string.h>   //for memset
#include <ctl.h>
#include <ARCbus.h>
#include <Error.h>
#include <limits.h>
#include "vector.h"
#include "torquers.h"
#include "output_type.h"
#include "ACDSerr.h"
#include "ACDS.h"
#include "LED.h"
#include "bias.h"

//structures to track torquer status
TQ_SET tq_stat;

//charge time for torquer capacitor in ms (or so)
unsigned short chargeTime=1*1024;
CTL_TIME_t lastFlip=0;

//values for status
static const int  err_mask[4]={T_STAT_ERR_1,T_STAT_ERR_2,T_STAT_ERR_3,T_STAT_ERR_4};
static const int init_mask[4]={T_STAT_UNINIT_1,T_STAT_UNINIT_2,T_STAT_UNINIT_3,T_STAT_UNINIT_4};
static const int stat_mask[4]={T_STAT_1,T_STAT_2,T_STAT_3,T_STAT_4};
      
const char ax_char[3]={'X','Y','Z'};

//check if torquers have been initialized
//return 1 if torquers have been initialized
short checkTorqueInit(void){
  return !(tq_stat.c.x.status&(T_STAT_UNINIT_1|T_STAT_UNINIT_2|T_STAT_UNINIT_3|T_STAT_UNINIT_4) || tq_stat.c.y.status&(T_STAT_UNINIT_1|T_STAT_UNINIT_2|T_STAT_UNINIT_3|T_STAT_UNINIT_4) || tq_stat.c.z.status&(T_STAT_UNINIT_1|T_STAT_UNINIT_2|T_STAT_UNINIT_3|T_STAT_UNINIT_4));
}

void get_stat(TQ_SET *dest){
    memcpy(dest,&tq_stat,sizeof(TQ_SET));
}

//give correction value index based on status of a given index
int stat2Idx(int idx){
    int stat=tq_stat.elm[idx].status;
    //check for uninitialized torquers
    if(stat&T_STAT_INIT_MASK){
        return -1;
    }
    //return status with only torquer bit present
    return stat&T_STAT_TQ_MASK;
}

//generate beacon status info
void tqstat2stat(unsigned char *dest){
  int i;
  //get info for each axis
  for(i=0;i<3;i++){
    //get data from each axis
    dest[i]=tq_stat.elm[i].status;
  }
}

//sets torquer status back to uninitialized state
void resetTorqueStatus(void){
  //big
  memset(&tq_stat,0,sizeof(TQ_SET));
  tq_stat.c.x.status=T_STAT_UNINIT_1|T_STAT_UNINIT_2|T_STAT_UNINIT_3|T_STAT_UNINIT_4;
  tq_stat.c.y.status=T_STAT_UNINIT_1|T_STAT_UNINIT_2|T_STAT_UNINIT_3|T_STAT_UNINIT_4;
  tq_stat.c.z.status=T_STAT_UNINIT_1|T_STAT_UNINIT_2|T_STAT_UNINIT_3|T_STAT_UNINIT_4;
}

//initialize torquer status
void torqueInit(void){
  VEC T={0,0,0};
  
  //reset torquer status to unknown
  resetTorqueStatus();

  printf("Initializing torquers\r\n");
  //drive torquers
  if(output_type==HUMAN_OUTPUT){
    printf("Previous Torquer Status:\r\n");
    print_torquer_status();
  }
  setTorque(&T);
  if(output_type==HUMAN_OUTPUT){
    printf("New Torquer Status:\r\n");
    print_torquer_status();
  }
  setTorque(&T);
  if(output_type==HUMAN_OUTPUT){
    printf("New Torquer Status:\r\n");
    print_torquer_status();
  }
  setTorque(&T);
  if(output_type==HUMAN_OUTPUT){
    printf("New Torquer Status:\r\n");
    print_torquer_status();
  }
  setTorque(&T);
  if(output_type==HUMAN_OUTPUT){
    printf("Final Torquer Status:\r\n");
    print_torquer_status();
  }
}

//set torquers to initialized state
void torqueReinit(void){
  int i,j;
  int num[3],dir[3],target,flip;
  
  //check if torquers have been initialized
  if(!checkTorqueInit()){
    if(output_type==HUMAN_OUTPUT){
      printf("Torquers Not initialized\r\n");
    }
    torqueInit();
    return;
  }
  
  //print out status
  if(output_type==HUMAN_OUTPUT){
    printf("Previous Torquer Status:\r\n");
    print_torquer_status();
  }
  for(i=0;i<4;i++){
    target=i>=2?M_MINUS:M_PLUS;
    flip=0;
    for(j=0;j<3;j++){
      //check direction
      if(target==M_PLUS){
        //flip if torquer in the - direction
        dir[j]=tq_stat.elm[j].status&stat_mask[i]?0:M_PLUS;
      }else{
        //flip if torquer in the + direction
        dir[j]=tq_stat.elm[j].status&stat_mask[i]?M_MINUS:0;
      }
      if(dir[j]){
        //set torquer number to flip
        num[j]=i+1;
        flip=1;
      }else{
        //no flip needed, zero torquer number
        num[j]=0;
      }
    }
    if(flip){
      //drive torquers
      drive_torquers(num,dir);
      //print status
      if(output_type==HUMAN_OUTPUT){
        printf("New Torquer Status:\r\n");
        print_torquer_status();
      }
    }
  }
  //print status
  if(output_type==HUMAN_OUTPUT){
    printf("Final Torquer Status:\r\n");
    print_torquer_status();
  }
}

//init pins for torquer drivers
void driverInit(void){
//skip init for DEV_BUILD
#ifndef DEV_BUILD
  //set all driver pins low
  X_DRV_PORT1&=~TQ_DRV_PINS;
  X_DRV_PORT2&=~TQ_DRV_PINS;
  Y_DRV_PORT1&=~TQ_DRV_PINS;
  Y_DRV_PORT2&=~TQ_DRV_PINS;
  Z_DRV_PORT1&=~TQ_DRV_PINS;
  Z_DRV_PORT2&=~TQ_DRV_PINS;
  //set pins to GPIO function
  X_DRV_SEL1&=~TQ_DRV_PINS;
  X_DRV_SEL2&=~TQ_DRV_PINS;
  Y_DRV_SEL1&=~TQ_DRV_PINS;
  Y_DRV_SEL2&=~TQ_DRV_PINS;
  Z_DRV_SEL1&=~TQ_DRV_PINS;
  Z_DRV_SEL2&=~TQ_DRV_PINS;
  //set driver pins as outputs
  X_DRV_DIR1|=TQ_DRV_PINS;
  X_DRV_DIR2|=TQ_DRV_PINS;
  Y_DRV_DIR1|=TQ_DRV_PINS;
  Y_DRV_DIR2|=TQ_DRV_PINS;
  Z_DRV_DIR1|=TQ_DRV_PINS;
  Z_DRV_DIR2|=TQ_DRV_PINS;
  //TESTING: setup test pin
  P5OUT&=~BIT1;
  P5SEL&=~BIT1;
  P5DIR|= BIT1;
#endif
  //reset torquer status to unknown
  resetTorqueStatus();
}

//initialize torquer feedback pins
void torque_fb_init(void){
//skip init for DEV_BUILD
#ifndef DEV_BUILD
    //X-axis
    TQ_FB_X_OUT|=TQ_FB_PIN_MASK;
    TQ_FB_X_DIR&=~TQ_FB_PIN_MASK;
    TQ_FB_X_SEL&=~TQ_FB_PIN_MASK;
    TQ_FB_X_REN|=TQ_FB_PIN_MASK;
    //Y-axis
    TQ_FB_Y_OUT|=TQ_FB_PIN_MASK;
    TQ_FB_Y_DIR&=~TQ_FB_PIN_MASK;
    TQ_FB_Y_SEL&=~TQ_FB_PIN_MASK;
    TQ_FB_Y_REN|=TQ_FB_PIN_MASK;
    //Z-axis
    TQ_FB_Z_OUT|=TQ_FB_PIN_MASK;
    TQ_FB_Z_DIR&=~TQ_FB_PIN_MASK;
    TQ_FB_Z_SEL&=~TQ_FB_PIN_MASK;
    TQ_FB_Z_REN|=TQ_FB_PIN_MASK;
#endif
}

//get feedback from all axis
//results are as follows:
//Bits 7 & 6  Unused, should be zero
//Bit 5       Z-axis charged
//Bit 4       Z-axis discharged
//Bit 3       Y-axis charged
//Bit 2       Y-axis discharged
//Bit 1       X-axis charged
//Bit 0       X-axis discharged
unsigned char get_torquer_fb(void){
  unsigned char fb;
  //get X-axis
  fb=TQ_FB_X&TQ_FB_PIN_MASK;
  //Get Y-axis, shift and add to result
  fb|=((TQ_FB_Y&TQ_FB_PIN_MASK)<<2);
  //get Z-axis, shift and add to result
  fb|=((TQ_FB_Z&TQ_FB_PIN_MASK)<<4);
  return fb;
}

//TODO: fix this for 4 torquer set code
//determine which torquer should be flipped based on what direction the flip is and which torquer was last flipped
int choseTorquer(int stat,int last,int dir){
  int i,pos,tq,j,tp,tlast;
  //first check for uninitialized torquers
  if(stat&T_STAT_INIT_MASK){
    //return first torquer that is not initialized
    for(i=0;i<4;i++){
      if(stat&init_mask[i]){
        return i+1;
      }
    }
    //all torquers initialized????
    return 0;
  }
  
  if(dir==M_PLUS){
    //find a - torquer to flip
    for(i=0,pos=-1;i<4;i++){
      //take the first one that comes along
      //TODO : find the torquer flipped least recently
      if(!(stat&stat_mask[i])){
        //find when the current torquer was last flipped
        for(j=0,tp=5,tlast=last;tlast;j++,tlast>>=4){
          if((tlast&0x000F)==i+1){
            tp=j+1;
            break;
          }
        }
        //check if torquer was last flipped least recently
        if(tp>pos){
          //set new best torquer
          pos=tp;
          tq=i+1;
        }
      }
    }
    return tq;
  }else{
    //find a + torquer to flip
    for(i=0,pos=-1;i<4;i++){
      //take the first that comes along
      //TODO : find the torquer flipped least recently
      if(stat&stat_mask[i]){
        //find when the current torquer was last flipped
        for(j=0,tp=5,tlast=last;tlast;j++,tlast>>=4){
          //look for torquer in last flipped
          if((tlast&0x000F)==i+1){
            tp=j+1;
            break;
          }
        }
        //check if torquer was last flipped least recently
        if(tp>pos){
          //set new best torquer
          pos=tp;
          tq=i+1;
        }
      }
    }
    return tq;
  }
  //unknown inputs
  return 0;
}

//convert torquer status bits into a torque string for printing
const char *stat_to_torque(int st){
  int c,tq;
  //check for error
  if(st&T_STAT_ERR_MASK){
    return "E!";
  }
  //check if torquers are not initialized
  if(st&T_STAT_INIT_MASK){
    return "?";
  }
  //mask out all but torque bits
  st&=T_STAT_TQ_MASK;
  //count bits set
  for(c=0;st;c++){
    st&=st-1;
  }
  //calculate torque
  tq=2*c-4;
  //use switch to return constant value
  switch(tq){
    case -4:
      return "-4";
    case -2:
      return "-2";
    case 0:
      return "0";
    case 2:
      return "2";
    case 4:
      return "4";
    default:
      //error unknown torque
      return "X";
  }
}

//return error string from status
const char *stat_err(int st){
  //only check for whole axis errors
  switch(st&(T_STAT_COMP_ERR|T_STAT_CAP_ERR)){
    //check for only capacitor error
    case T_STAT_CAP_ERR:
      return "Cap";
    //check for only comparitor errror
    case T_STAT_COMP_ERR:
      return "Comp";
    //check for both status and comparitor error
    case T_STAT_CAP_ERR|T_STAT_COMP_ERR:
      return "Cap, Comp";
    //no error
    default:
      return "";
  }
}

//direction status of torquer 1
char torquer_dir(int st,int n){
  //check range of n
  if(n<1 || n>4){
    //TODO: report error?
    return 'X';
  }
  //check for errors
  if(st&err_mask[n-1]){
    return '!';
  }
  //check if initialized
  if(st&init_mask[n-1]){
    return '?';
  }
  //return status
  if(st&stat_mask[n-1]){
    return '+';
  }else{
    return '-';
  }
}
  
 #define getLast(s,n)   ((((s).last)&(0x000F<<(n*4)))>>(n*4))

//print the staus of a torquer set for easy reading by a human
void print_torquer_status(void){
  //print status
  print_tqstat(&tq_stat);
}
    
void print_tqstat(const TQ_SET *stat){
  //print status
  printf("\t""Axis\t""M\t""status\t""last0\t""last1\t""last2\t""last3\t""error\r\n"
        "\tx\t%s\t%c%c%c%c\t%i\t%i\t%i\t%i\t%s\r\n"
        "\ty\t%s\t%c%c%c%c\t%i\t%i\t%i\t%i\t%s\r\n"
        "\tz\t%s\t%c%c%c%c\t%i\t%i\t%i\t%i\t%s\r\n",
        stat_to_torque(stat->c.x.status),torquer_dir(stat->c.x.status,1),torquer_dir(stat->c.x.status,2),torquer_dir(stat->c.x.status,3),torquer_dir(stat->c.x.status,4),getLast(stat->c.x,0),getLast(stat->c.x,1),getLast(stat->c.x,2),getLast(stat->c.x,3),stat_err(stat->c.x.status),
        stat_to_torque(stat->c.y.status),torquer_dir(stat->c.y.status,1),torquer_dir(stat->c.y.status,2),torquer_dir(stat->c.y.status,3),torquer_dir(stat->c.y.status,4),getLast(stat->c.y,0),getLast(stat->c.y,1),getLast(stat->c.y,2),getLast(stat->c.y,3),stat_err(stat->c.y.status),
        stat_to_torque(stat->c.z.status),torquer_dir(stat->c.z.status,1),torquer_dir(stat->c.z.status,2),torquer_dir(stat->c.z.status,3),torquer_dir(stat->c.z.status,4),getLast(stat->c.z,0),getLast(stat->c.z,1),getLast(stat->c.z,2),getLast(stat->c.z,3),stat_err(stat->c.z.status));
}

//print the status of a torquer set so that it can be easily parsed on the other end by software
void print_torquer_stat_code(void){
  //print status
  print_tqstat_code(&tq_stat);
}
  
void print_tqstat_code(const TQ_SET *stat){
  //print status
  printf("%c%c%c%c\t%c%c%c%c\t%c%c%c%c\t%i\t%i\t%i\t",
      torquer_dir(stat->c.x.status,1),torquer_dir(stat->c.x.status,2),torquer_dir(stat->c.x.status,3),torquer_dir(stat->c.x.status,4),
      torquer_dir(stat->c.y.status,1),torquer_dir(stat->c.y.status,2),torquer_dir(stat->c.y.status,3),torquer_dir(stat->c.y.status,4),
      torquer_dir(stat->c.z.status,1),torquer_dir(stat->c.z.status,2),torquer_dir(stat->c.z.status,3),torquer_dir(stat->c.z.status,4),
      stat->c.x.last,stat->c.y.last,stat->c.z.last);
}

//TODO: fix for four torquers
//set the torque of the torquer set given by set
int setTorque(const VEC *T){
  int flip[3]={0,0,0},dir[3]={0,0,0},tmp,rt=0,i;
  int stat,set,num;
  //loop through torquer axis and set torque
  for(i=0;i<3;i++){
    if(T->elm[i]<-3*M_CmdLim_b){
      //set torque to -4
      num=0;
    }else if(T->elm[i]>3*M_CmdLim_b){
      //set torque to +4
      num=4;
    }else if(T->elm[i]<-1*M_CmdLim_b){
      //set torque to -2
      num=1;
    }else if(T->elm[i]>1*M_CmdLim_b){
      //set torque to +2
      num=3;
    }else{
      //set torque to 0
      num=2;
    }
    //printf("%c-axis torque = %i. num = %i\r\n",ax_char[i],(int)T->elm[i],num);
    //get torquer status
    stat=tq_stat.elm[i].status;
    //mask out all but status of initialized torquers
    stat=(stat&T_STAT_TQ_MASK)&(((~stat)&T_STAT_INIT_MASK)>>T_STAT_TQ_INIT_SHIFT);
    //count number of torquers set
    for(set=0;stat;set++){
      stat&=stat-1;
    }
    //check if set torque is desired torque
    if(set==num){
      //check if torquers are uninitialized
      if(tq_stat.elm[i].status&T_STAT_INIT_MASK){
        //check last
        /*if(tq_stat.elm[i].last&0x01){
          //even, flip in + direction
          dir[i]=M_PLUS;
        }else{
          //odd, flip in - direction
          dir[i]=M_MINUS;
        }*/
        dir[i]=M_MINUS;
        //determine which torquer should be flipped
        flip[i]=choseTorquer(tq_stat.elm[i].status,tq_stat.elm[i].last,dir[i]);
        //report information message
        report_error(ERR_LEV_INFO,ACDS_ERR_SRC_TORQUERS,TQ_INFO_FLIP,i|(flip[i]<<4));        
      }
    }else if(num>set){
        //too few torquers flipped in the + direction, flip one
        dir[i]=M_PLUS;
        //determine which torquer should be flipped
        flip[i]=choseTorquer(tq_stat.elm[i].status,tq_stat.elm[i].last,dir[i]);
        //report information message
        report_error(ERR_LEV_INFO,ACDS_ERR_SRC_TORQUERS,TQ_INFO_FLIP,i|(flip[i]<<4));
    }else{
       //too few torquers flipped in the - direction, flip one
       dir[i]=M_MINUS;
      //determine which torquer should be flipped
      flip[i]=choseTorquer(tq_stat.elm[i].status,tq_stat.elm[i].last,dir[i]);
      //report information message
      report_error(ERR_LEV_INFO,ACDS_ERR_SRC_TORQUERS,TQ_INFO_FLIP,i|(flip[i]<<4));
    }
  }
  //drive toruqers
  return drive_torquers(flip,dir);
}
 
//drive one torquer in each axis
int drive_torquers(const int* num,const int* dir){
  volatile unsigned char *(port[3]);
  volatile unsigned char *const ports[3][2]={{&X_DRV_PORT1,&X_DRV_PORT2},{&Y_DRV_PORT1,&Y_DRV_PORT2},{&Z_DRV_PORT1,&Z_DRV_PORT2}};
  unsigned char val[3]={0,0,0};
  unsigned char p_old,fb1,fb2;
  CTL_TIME_t ct;
  int i,d,rtval=RET_SUCCESS;
  //calculate mask values
  for(i=0;i<3;i++){
    //check which torquer is being flipped and use appropriate port
    if(num[i]<=2){
      //use first port
      port[i]=ports[i][0];
    }else{
      //use second port
      port[i]=ports[i][1];
    }
    //get direction
    d=dir[i];
    //flip direction for X-axis
    if(i==0){
      d*=-1;
    }
    //set mask based on torquer and direction
    switch(num[i]){
      case 1:
      case 3:
        //drive torquer 1
        if(d==M_PLUS){
          //drive 0 high 1 low
          val[i]=TQ_OUT0_H|TQ_OUT1_L;
        }else if(d==M_MINUS){
          //drive 1 high 0 low
          val[i]=TQ_OUT1_H|TQ_OUT0_L;
        }else{
          val[i]=0;
          report_error(ERR_LEV_ERROR,ACDS_ERR_SRC_TORQUERS,TQ_ERR_BAD_DIR,d);
          //TODO: perhaps abort here
        }
      break;
      case 2:
      case 4:
        //drive torquer 2
        if(d==M_PLUS){
          //drive 2 high, 1 low
          val[i]=TQ_OUT2_H|TQ_OUT1_L;
        }else if(d==M_MINUS){
          //drive 1 high, 2 low
          val[i]=TQ_OUT1_H|TQ_OUT2_L;
        }else{
          val[i]=0;
          report_error(ERR_LEV_ERROR,ACDS_ERR_SRC_TORQUERS,TQ_ERR_BAD_DIR,d);
          //TODO: perhaps abort here
        }
      break;
      case 0:
           //no torquer to flip
      break;
      default:
        //unknown torquer, return
        report_error(ERR_LEV_ERROR,ACDS_ERR_SRC_TORQUERS,TQ_ERR_BAD_TORQUER,num[i]);
        return TQ_ERR_BAD_TORQUER;
     }
    //set new status
    if(dir[i]==M_PLUS){
      //set direction
      tq_stat.elm[i].status|=stat_mask[num[i]-1];
      //shift last torquer
      tq_stat.elm[i].last<<=4;
      //set last torquer
      tq_stat.elm[i].last|=num[i];
      //clear init flag
      tq_stat.elm[i].status&=~init_mask[num[i]-1];
    }else if(dir[i]==M_MINUS){
      //set direction
      tq_stat.elm[i].status&=~stat_mask[num[i]-1];
      //shift last torquer
      tq_stat.elm[i].last<<=4;
      //set last torquer
      tq_stat.elm[i].last|=num[i];
      //clear init flag
      tq_stat.elm[i].status&=~init_mask[num[i]-1];
    }
   }
   //elevate priority so torquer flip is not interrupted
   //TODO: perhaps do this with an interrupt? also get priorities straight (just had to say that)
   //but really some sort of plan is needed for task priority. this priority should just be quite high
   //but it should not spend too much time at high priority
   p_old=ctl_task_set_priority(ctl_task_executing,200);
   //check if torquer is ready for flip
   //TODO: should this really be in flight code??
   //it seems like it could cause problems if timing gets off
   ct=ctl_get_current_time();
   if((ct-lastFlip)<(chargeTime-10)){
    ctl_timeout_wait(lastFlip+chargeTime);
   }
   //skip torquer feedback for DEV_BUILD
   #ifndef DEV_BUILD
     //get torquer feedback before
     fb1=get_torquer_fb();
     //TESTING: set test pin high while torquer is flipped
     P5OUT|=BIT1;
   #else
    //fake feedback for DEV_BUILD
    fb1=0x2A;
   #endif
   //skip driving torquers for DEV build
   #ifndef DEV_BUILD
     //Set outputs
     for(i=0;i<3;i++){
       //Clear output pins
       *port[i]&=~TQ_DRV_PINS;
       //drive torquer
       *port[i]|=val[i];
     }
   #endif
   //delay while torquer flipped
   ctl_timeout_wait(ctl_get_current_time()+2);
   //skip driving torquers for DEV build
   #ifndef DEV_BUILD
     for(i=0;i<3;i++){
       //stop driving
       *port[i]&=~TQ_DRV_PINS;
     }
   #endif
   //skip test pin for DEV_BUILD
   #ifndef DEV_BUILD
     //TESTING: set test pin low after torquers have been flipped
     P5OUT&=~BIT1;
   #endif
   //get flip time
   lastFlip=ctl_get_current_time();
   //skip torquer feedback for DEV_BUILD
   #ifndef DEV_BUILD
     //get torquer feedback after
     fb2=get_torquer_fb();
   #else
     //pretend everything is good
     fb2=(num[0]?0x01:0x02)|(num[1]?0x04:0x08)|(num[2]?0x10:0x20);
   #endif
   //restore old priority
   ctl_task_set_priority(ctl_task_executing,p_old);
   //toggle LED
   FLIP_LED_toggle();
   //TODO: save feedback for ground analysis
   report_error(ERR_LEV_INFO,ACDS_ERR_SRC_TORQUERS,TQ_INFO_TQFB,((fb1<<8)|fb2));
   //printf("fb1 0x%02X\r\nfb2 0x%02X\r\n",fb1,fb2);
   //Look at torquer feedback for each axis and add error bits to status if nessissary
   //TODO: think deep thoughts about what error bits should be set here
   //special care should be taken so that functionality is not limited if comparitors function poorly
   for(i=0;i<3;i++,fb1>>=2,fb2>>=2){
     //check if torquer was flipped in this axis
     if(num[i]!=0){
       //increment flips
       if(status.flips[i]++==USHRT_MAX){
         //saturate
         status.flips[i]=USHRT_MAX;
       }
     }  
     //check feedback from before flip
     switch(fb1&0x3){
       //check if discharged
       case 0x01:
       //check if not charged or discharged
       case 0x00:
          tq_stat.elm[i].status|=T_STAT_CAP_ERR;
          //report error
          report_error(ERR_LEV_ERROR,ACDS_ERR_SRC_TORQUERS,TQ_ERR_CAP,i);
          //check which torquer was flipped
          if(num[i]!=0){
            //Cap not charged, flip questionable
            tq_stat.elm[i].status|=init_mask[num[i]-1];
          }
          rtval=TQ_ERR_CAP;
       break;
       //check if charged
       case 0x02:
        //everything good
       break;
       //error with comparitor
       default:
         tq_stat.elm[i].status|=T_STAT_COMP_ERR;
         //report error
         report_error(ERR_LEV_ERROR,ACDS_ERR_SRC_TORQUERS,TQ_ERR_COMP,i);
       break;
     }
     //check feedback from after flip
     switch(fb2&0x3){
       //check if not charged or discharged
       case 0x00:
       //check if charged
       case 0x02:
        //check if torquer should have been flipped
        if(num[i]!=0){ 
           //error flipping torquer, set error flag
           tq_stat.elm[i].status|=err_mask[num[i]-1];
           //Cap not discharged, flip questionable
           //TODO: figure out what to do here
           //tq_stat.elm[i].status|=int_mask[num[i]-1];
           //report error
           report_error(ERR_LEV_ERROR,ACDS_ERR_SRC_TORQUERS,TQ_ERR_BAD_CONNECTION,i|(num[i]<<4));
           //check if already returning an error
           if(rtval==RET_SUCCESS){
            //only give bad connection error if there are no other errors
            rtval=TQ_ERR_BAD_CONNECTION;
          }
        }
       break;
       //check if discharged
       case 0x01:
        if(num[i]!=0){
          //everything good
          break;
        }
        //no torquer was flipped so no discharge should have happened, flag error
        //TODO: is this really the correct error here?
        tq_stat.elm[i].status|=T_STAT_CAP_ERR;
        rtval=TQ_ERR_CAP;
        //report error
        report_error(ERR_LEV_ERROR,ACDS_ERR_SRC_TORQUERS,TQ_ERR_CAP,i);
       break;
       //error with comparitor
       default:
         tq_stat.elm[i].status|=T_STAT_COMP_ERR;
         //report error
         report_error(ERR_LEV_ERROR,ACDS_ERR_SRC_TORQUERS,TQ_ERR_COMP,i);
         rtval=TQ_ERR_COMP;
       break;
     }
   }
   return rtval;
}

