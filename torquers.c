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

//structures to track torquer status
TQ_SET tq_big,tq_small;

//charge time for torquer capacitor in ms (or so)
unsigned short chargeTime=1*1024;
CTL_TIME_t lastFlip=0;

//check if torquers have been initialized
//return 1 if torquers have been initialized
short checkTorqueInit(void){
  return !(tq_big.c.x.status&(T_STAT_UNINIT_2|T_STAT_UNINIT_1) || tq_big.c.y.status&(T_STAT_UNINIT_2|T_STAT_UNINIT_1) || tq_big.c.z.status&(T_STAT_UNINIT_2|T_STAT_UNINIT_1));
}

//generate beacon status info
void tqstat2stat(unsigned char *dest){
  int i;
  //get info for each axis
  for(i=0;i<3;i++){
    //combine data from torquer sets
    dest[i]=((T_STAT_TQ_MASK&tq_big.elm[i].status)<<4)|(T_STAT_TQ_MASK&tq_small.elm[i].status);
  }
}

//sets torquer status back to uninitialized state
void resetTorqueStatus(void){
  //big
  memset(&tq_big,0,sizeof(tq_big));
  tq_big.c.x.status=T_STAT_UNINIT_2|T_STAT_UNINIT_1;
  tq_big.c.y.status=T_STAT_UNINIT_2|T_STAT_UNINIT_1;
  tq_big.c.z.status=T_STAT_UNINIT_2|T_STAT_UNINIT_1;
  //small
  memset(&tq_small,0,sizeof(tq_small));
  tq_small.c.x.status=T_STAT_UNINIT_2|T_STAT_UNINIT_1;
  tq_small.c.y.status=T_STAT_UNINIT_2|T_STAT_UNINIT_1;
  tq_small.c.z.status=T_STAT_UNINIT_2|T_STAT_UNINIT_1;
}

//initialize torquer status
void torqueInit(void){
  VEC T={0,0,0};
  int set,i;
  
  //reset torquer status to unknown
  resetTorqueStatus();

  printf("Initializing torquers\r\n");
  for(set=TQ_SET_BIG,i=0;i<2;i++,set=TQ_SET_SMALL){
    //drive torquers
    if(output_type==HUMAN_OUTPUT){
      printf("Previous Torquer Status:\r\n");
      print_torquer_status(set);
    }
    setTorque(&T,set);
    if(output_type==HUMAN_OUTPUT){
      printf("New Torquer Status:\r\n");
      print_torquer_status(set);
    }
    setTorque(&T,set);
    if(output_type==HUMAN_OUTPUT){
      printf("Final Torquer Status:\r\n");
      print_torquer_status(set);
    }
  }
}

//init pins for torquer drivers
void driverInit(void){
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
  //reset torquer status to unknown
  resetTorqueStatus();
}

//initialize torquer feedback pins
void torque_fb_init(void){
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

//determine which torquer should be flipped based on what direction the flip is and which torquer was last flipped
int choseTorquer(int stat,int last,int dir){
  //check if torquers have been initialized
  if(!(stat&(T_STAT_UNINIT_1|T_STAT_UNINIT_2))){
     //strip off error bits
     //TODO: handle error bits appropratly: should a non working torquer be flipped or not flipped?
     stat&=~(T_STAT_ERR_1|T_STAT_ERR_2);
     //if both torquers are the same flip the one that was flipped least recently  
     if(stat==(T_STAT_1|T_STAT_2) || stat==0){
      return (last==1)?2:1;
    //otherwise there is only one choice of torquer
    }else if(stat==T_STAT_1){
        return (dir==M_PLUS)?2:1;
    }else if(stat==T_STAT_2){
        return (dir==M_PLUS)?1:2;
    }
  }else{
    //check which torquer was not initialized and flip that one
    //if one torquer has an error try the other
    if(stat&T_STAT_UNINIT_1 && !(stat&T_STAT_ERR_1)){
      return 1;
    }else if(stat&T_STAT_UNINIT_2 && !(stat&T_STAT_ERR_2)){
      return 2;
    }else{
      //otherwise flip the least recently flipped torquer
      return (last==1)?2:1;
    }
  }
  //TODO: do something with error bits here
  //something is not right, extra bits are set here
  //TODO: probably could take corrective action
  report_error(ERR_LEV_ERROR,ACDS_ERR_SRC_TORQUERS,TQ_ERROR_INVALID_STATUS,stat);
  //unknown inputs
  return 0;
}

//convert torquer status bits into a torque string for printing
const char *stat_to_torque(int st){
  //mask out large error bits
  st&=~(T_STAT_CAP_ERR|T_STAT_COMP_ERR);
  //check status
  switch(st){
    case 0:
      return "-1";
    case T_STAT_1:
    case T_STAT_2:
      return "0";
    case T_STAT_1|T_STAT_2: 
      return "1";
  }
  //check for error
  if(st&T_STAT_ERR_MASK){
    return "E!";
  }
  //unknown torque status
  return "?";
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
char torquer_dir_1(int st){
  //check for errors
  if(st&T_STAT_ERR_1){
    return '!';
  }
  //check if initialized
  if(st&T_STAT_UNINIT_1){
    return '?';
  }
  //return status
  if(st&T_STAT_1){
    return '+';
  }else{
    return '-';
  }
}
//direction status of torquer 2
char torquer_dir_2(int st){
  //check for errors
  if(st&T_STAT_ERR_2){
    return '!';
  }
  //check if initialized
  if(st&T_STAT_UNINIT_2){
    return '?';
  }
  //return status
  if(st&T_STAT_2){
    return '+';
  }else{
    return '-';
  }
}

//print the staus of a torquer set for easy reading by a human
void print_torquer_status(int set){
  TQ_SET *p;
  //get the current torque of the set given by set
  switch(set){
    case TQ_SET_BIG:
      p=&tq_big;
      printf("Large Torquers:\r\n");
    break;
    case TQ_SET_SMALL:
      p=&tq_small;
      printf("Small Torquers:\r\n");
    break;
    default:
      return;
    break;
  }
  //print status
  printf(     "\t""Axis\t""M\t""status\t""last\t""error\r\n"
              "\tx\t%s\t%c%c\t%i\t%s\r\n"
              "\ty\t%s\t%c%c\t%i\t%s\r\n"
              "\tz\t%s\t%c%c\t%i\t%s\r\n",
              stat_to_torque(p->c.x.status),torquer_dir_1(p->c.x.status),torquer_dir_2(p->c.x.status),p->c.x.last,stat_err(p->c.x.status),
              stat_to_torque(p->c.y.status),torquer_dir_1(p->c.y.status),torquer_dir_2(p->c.y.status),p->c.y.last,stat_err(p->c.y.status),
              stat_to_torque(p->c.z.status),torquer_dir_1(p->c.z.status),torquer_dir_2(p->c.z.status),p->c.z.last,stat_err(p->c.z.status));
}

//print the status of a torquer set so that it can be easily parsed on the other end by software
void print_torquer_stat_code(int set){
  TQ_SET *p;
    //get the current torque of the set given by set
  switch(set){
    case TQ_SET_BIG:
      p=&tq_big;
      printf("B");
    break;
    case TQ_SET_SMALL:
      p=&tq_small;
      printf("S");
    break;
    default:
      return;
    break;
  }
  //print status
  printf("\t%c%c\t%c%c\t%c%c\t%i\t%i\t%i\t",
              (p->c.x.status&T_STAT_UNINIT_1)?'?':((p->c.x.status&T_STAT_1)?'+':'-'),(p->c.x.status&T_STAT_UNINIT_2)?'?':((p->c.x.status&T_STAT_2)?'+':'-'),
              (p->c.y.status&T_STAT_UNINIT_1)?'?':((p->c.y.status&T_STAT_1)?'+':'-'),(p->c.y.status&T_STAT_UNINIT_2)?'?':((p->c.y.status&T_STAT_2)?'+':'-'),
              (p->c.z.status&T_STAT_UNINIT_1)?'?':((p->c.z.status&T_STAT_1)?'+':'-'),(p->c.z.status&T_STAT_UNINIT_2)?'?':((p->c.z.status&T_STAT_2)?'+':'-'),
              p->c.x.last,p->c.y.last,p->c.z.last);
}

//set the torque of the torquer set given by set
int setTorque(const VEC *T,int set){
  TQ_SET *current;
  const char ax_name[3]={'X','Y','Z'};
  int flip[3]={0,0,0},dir[3]={0,0,0},tmp,rt=0,i;
  //get the current torque of the set given by set
  switch(set){
    case TQ_SET_BIG:
      current=&tq_big;
    break;
    case TQ_SET_SMALL:
      current=&tq_small;
    break;
    default:
      //report error
      report_error(ERR_LEV_ERROR,ACDS_ERR_SRC_TORQUERS,TQ_ERR_BAD_SET,set);
      return -1;
    break;
  }
  //TODO: try to get this into some sort of loop to make the code easier to read and maintain
  for(i=0;i<3;i++){
    //figure out which torquers need to be flipped
    //look at current dipole moment
    switch(current->elm[i].status){
      case 0:
        if(T->elm[i]>=0){
          //set direction
          dir[i]=M_PLUS;
          //determine which torquer should be flipped
          flip[i]=choseTorquer(current->elm[i].status,current->elm[i].last,dir[i]);
          //report information message
          report_error(ERR_LEV_INFO,ACDS_ERR_SRC_TORQUERS,TQ_INFO_FLIP,i|(flip[i]<<4));
        }
      break;
      case T_STAT_1:
      case T_STAT_2:
        if(T->elm[i]>0){
          //set direction
          dir[i]=M_PLUS;
          //determine which torquer should be flipped
          flip[i]=choseTorquer(current->elm[i].status,current->elm[i].last,dir[i]);
          //report information message
          report_error(ERR_LEV_INFO,ACDS_ERR_SRC_TORQUERS,TQ_INFO_FLIP,i|(flip[i]<<4));
        }else if(T->elm[i]<0){
          //set direction
          dir[i]=M_MINUS;
          //determine which torquer should be flipped
          flip[i]=choseTorquer(current->elm[i].status,current->elm[i].last,dir[i]);
          //report information message
          report_error(ERR_LEV_INFO,ACDS_ERR_SRC_TORQUERS,TQ_INFO_FLIP,i|(flip[i]<<4));
        }
      break;
      case T_STAT_1|T_STAT_2:  
        if(T->elm[i]<=0){
          //set direction
          dir[i]=M_MINUS;
          //determine which torquer should be flipped
          flip[i]=choseTorquer(current->elm[i].status,current->elm[i].last,dir[i]);
          //report information message
          report_error(ERR_LEV_INFO,ACDS_ERR_SRC_TORQUERS,TQ_INFO_FLIP,i|(flip[i]<<4));
        }
      break;
      default:
        if(T->elm[i]>0){
          //set direction
          dir[i]=M_PLUS;
          //determine which torquer should be flipped
          flip[i]=choseTorquer(current->elm[i].status,current->elm[i].last,dir[i]);
          //report information message
          report_error(ERR_LEV_INFO,ACDS_ERR_SRC_TORQUERS,TQ_INFO_FLIP,i|(flip[i]<<4));
        }else if(T->elm[i]<0){
          //set direction
          dir[i]=M_MINUS;
          //determine which torquer should be flipped
          flip[i]=choseTorquer(current->elm[i].status,current->elm[i].last,dir[i]);
          //report information message
          report_error(ERR_LEV_INFO,ACDS_ERR_SRC_TORQUERS,TQ_INFO_FLIP,i|(flip[i]<<4));
        }else{
          //zero torque desired
          //check if one status is known 
          if(current->elm[i].status&(~(current->elm[i].status>>2))&0x03){
            //set direction
            dir[i]=M_MINUS;
            //determine which torquer should be flipped
            flip[i]=choseTorquer(current->elm[i].status,current->elm[i].last,dir[i]);
          }else{
            //set direction
            dir[i]=M_PLUS;
            //determine which torquer should be flipped
            flip[i]=choseTorquer(current->elm[i].status,current->elm[i].last,dir[i]);
          }
          //report information message
          report_error(ERR_LEV_INFO,ACDS_ERR_SRC_TORQUERS,TQ_INFO_FLIP,i|(flip[i]<<4));
        } 
    }
  }
  //drive toruqers
  return drive_torquers(set,flip,dir);
}
 
//drive one torquer in each axis
int drive_torquers(int set,const int* num,const int* dir){
  volatile unsigned char *(port[3]);
  unsigned char val[3]={0,0,0};
  unsigned char p_old,fb1,fb2;
  CTL_TIME_t ct;
  TQ_SET *current;
  int i,d,rtval=RET_SUCCESS;
  //get the current torque of the set given by set
  switch(set){
    case TQ_SET_BIG:
      current=&tq_big;
      port[0]=&X_DRV_PORT1;
      port[1]=&Y_DRV_PORT1;
      port[2]=&Z_DRV_PORT1;
    break;
    case TQ_SET_SMALL:
      current=&tq_small;
      port[0]=&X_DRV_PORT2;
      port[1]=&Y_DRV_PORT2;
      port[2]=&Z_DRV_PORT2;
    break;
    default:
      //Report error
      report_error(ERR_LEV_ERROR,ACDS_ERR_SRC_TORQUERS,TQ_ERR_BAD_SET,set);
      return TQ_ERR_BAD_SET;
    break;
  }
  for(i=0;i<3;i++){
    //get direction
    d=dir[i];
    //flip direction for X-axis
    if(i==0){
      d*=-1;
    }
    //setup
    switch(num[i]){
      case 1:
        //drive torquer 1
        if(d==M_PLUS){
          //drive 0 high 1 low
          val[i]=TQ_OUT0_H|TQ_OUT1_L;
        }else if(d==M_MINUS){
          //drive 1 high 0 low
          val[i]=TQ_OUT1_H|TQ_OUT0_L;
        }
      break;
      case 2:
        //drive torquer 2
        if(d==M_PLUS){
          //drive 2 high, 1 low
          val[i]=TQ_OUT2_H|TQ_OUT1_L;
        }else if(d==M_MINUS){
          //drive 1 high, 2 low
          val[i]=TQ_OUT1_H|TQ_OUT2_L;
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
      current->elm[i].status|=1<<(num[i]-1);
      current->elm[i].last=num[i];
    }else if(dir[i]==M_MINUS){
      current->elm[i].status&=~(1<<(num[i]-1));
      current->elm[i].last=num[i];
    }
    current->elm[i].status&=~(T_STAT_UNINIT_1<<(num[i]-1));
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
   //get torquer feedback before
   fb1=get_torquer_fb();
   //TESTING: set test pin high while torquer is flipped
   P5OUT|=BIT1;
   //Set outputs
   for(i=0;i<3;i++){
     //Clear output pins
     *port[i]&=~TQ_DRV_PINS;
     //drive torquer
     *port[i]|=val[i];
   }
   //delay while torquer flipped
   ctl_timeout_wait(ctl_get_current_time()+2);
   for(i=0;i<3;i++){
     //stop driving
     *port[i]&=~TQ_DRV_PINS;
   }
   //TESTING: set test pin low after torquers have been flipped
   P5OUT&=~BIT1;
   //get flip time
   lastFlip=ctl_get_current_time();
   //get torquer feedback after
   fb2=get_torquer_fb();
   //restore old priority
   ctl_task_set_priority(ctl_task_executing,p_old);
   //TODO: save feedback for ground analysis
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
          current->elm[i].status|=T_STAT_CAP_ERR;
          //report error
          report_error(ERR_LEV_ERROR,ACDS_ERR_SRC_TORQUERS,TQ_ERR_CAP,i);
          //check which torquer was flipped
          if(num[i]==1){
            //Cap not charged, flip questionable
            current->elm[i].status|=T_STAT_UNINIT_1;
          }else if(num[i]==2){
            //Cap not charged, flip questionable
            current->elm[i].status|=T_STAT_UNINIT_2;
          }
          rtval=TQ_ERR_CAP;
       break;
       //check if charged
       case 0x02:
        //everything good
       break;
       //error with comparitor
       default:
         current->elm[i].status|=T_STAT_COMP_ERR;
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
        //check which torquer was flipped
        if(num[i]==1){
          //error flipping torquer #1
          current->elm[i].status|=T_STAT_ERR_1;
          //Cap not discharged, flip questionable
          //current->elm[i].status|=T_STAT_UNINIT_1;
        }else if(num[i]==2){
          //error flipping torquer #2
          current->elm[i].status|=T_STAT_ERR_2;
          //Cap not discharged, flip questionable
          current->elm[i].status|=T_STAT_UNINIT_2;
        }
        //check if torquer should have been flipped
        if(num[i]!=0){
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
        current->elm[i].status|=T_STAT_CAP_ERR;
        rtval=TQ_ERR_CAP;
        //report error
        report_error(ERR_LEV_ERROR,ACDS_ERR_SRC_TORQUERS,TQ_ERR_CAP,i);
       break;
       //error with comparitor
       default:
         current->elm[i].status|=T_STAT_COMP_ERR;
         //report error
         report_error(ERR_LEV_ERROR,ACDS_ERR_SRC_TORQUERS,TQ_ERR_COMP,i);
         rtval=TQ_ERR_COMP;
       break;
     }
   }
   return rtval;
}

