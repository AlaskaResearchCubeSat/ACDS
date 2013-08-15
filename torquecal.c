#include <msp430.h>
#include <string.h>
#include <stdio.h>
#include "torquecal.h"
#include "torquers.h"
#include "adc.h"
#include "terminal.h"
#include "timer.h"
#include "batt.h"

FLASH_CAL calibration __at ".cal";
//FLASH_CAL calibration;

//need to get big torquer status for calibration
extern TQ_SET tq_big;

int clrCal(void){
  unsigned short state;
  //make sure that calibration data fills a section
  if(sizeof(FLASH_CAL)!=512){
    return 1;
  }
  //erase calibration section
  //first disable watchdog
  WDTCTL = WDTPW|WDTHOLD;
  //unlock flash memory
  FCTL3=FWKEY;
  //setup flash for erase
  FCTL1=FWKEY|ERASE;
  //dummy write to indicate which segment to erase
  calibration.magic=0;
  //lock flash
  FCTL3=FWKEY|LOCK;
  //TODO: enable WDT if needed
}

int writeCal(const CAL_TBL *newCal){
  unsigned short state;
  //make sure that calibration data fills a section
  if(sizeof(FLASH_CAL)!=512){
    return 1;
  }
  //disable watchdog during flash operation
  WDTCTL = WDTPW|WDTHOLD;
  //first erase calibration section
  //unlock flash memory
  FCTL3=FWKEY;
  //setup flash for erase
  FCTL1=FWKEY|ERASE;
  //dummy write to indicate which segment to erase
  calibration.magic=0;
  //enable writing
  FCTL1=FWKEY|WRT;
  //write magic
  calibration.magic=CAL_MAGIC;
  //write data to flash
  memcpy(calibration.cal,newCal,sizeof(CAL_TBL));
  //disable writing
  FCTL1=FWKEY;
  //lock flash
  FCTL3=FWKEY|LOCK;
  //TODO: enable WDT if needed
  return 0;
}

//return index from torquer status
int stat2idx(int stat){
  //const int tbl[4]={2,3,1,0};
  const int tbl[4]={3,0,2,1};
  //check stat so that array bounds are not exceeded
  if(stat>=4 || stat<0){
    //return -1 in case of error
    return -1;
  }
  return tbl[stat];
}

//get index for the current torquer status in the calibration table
int calIdx(void){
  int idx,xidx,yidx,zidx;
  //get index from status values
  xidx=stat2idx(tq_big.c.x.status);
  yidx=stat2idx(tq_big.c.y.status);
  zidx=stat2idx(tq_big.c.z.status);
  //check for error
  if(xidx==-1 || yidx==-1 || zidx==-1){
    //error, return invalid
    return -1;
  }
  idx=xidx+4*yidx+16*zidx;
  return idx;
}

VEC_INT *correct(VEC_INT *m){
  int idx=calIdx();
  //check for error
  if(idx<0){
    return NULL;
  }
  //check if calibration is valid
  if(calibration.magic==CAL_MAGIC){
    //subtract calibration factor
    ivec_dif(m,&calibration.cal[idx]);
  }
  return m;
}

//dump calibration data to the terminal
int dumpCalCmd(char **argv,unsigned short argc){
  const char *(c_strs[3])={"{","{%i,%i,%i},\r\n","}\r\n"};
  const char *(tab_strs[3])={"X\tY\tZ\r\n","%i\t%i\t%i\r\n",""};
  int i;
  const char **strs;
  //check for arguments
  if(argc==0){
    //default to C output type
    strs=c_strs;
  }else{
    //check for tab output
    if(!strcmp("tab",argv[1])){
      strs=tab_strs;
    //check for C output
    }else if(!strcmp("C",argv[1])){
      strs=c_strs;
    //unknown output type
    }else{
      printf("Error : unknown argument \'%s\'.\r\n",argv[1]);
      return -1;
    }
  }
  //check to see if calibration is valid
  if(calibration.magic!=CAL_MAGIC){
    //calibration is not valid, abort
    printf("Error calibration not valid\r\n");
    return 1;
  }
  //output first line
  printf(strs[0]);
  for(i=0;i<64;i++){
    //print data
    printf(strs[1],calibration.cal[i].c.x,calibration.cal[i].c.y,calibration.cal[i].c.z);
  }
  //print ending
  printf(strs[2]);
  return 1;
}
  
//default calibration to avoid having to run the calibration procedure command every time the code is loaded  
const CAL_TBL default_cal={{352,917,0},
{193,282,0},
{6,-308,0},
{159,328,0},
{27,981,0},
{-140,380,0},
{-324,-212,0},
{-172,389,0},
{-234,1063,0},
{-389,428,0},
{-565,-1024,0},
{-960,288,0},
{9,320,0},
{288,0,0},
{2,-1024,0},
{256,16,0},
{272,16,0},
{42,16,0},
{-72,-1276,0},
{1,0,0},
{2,132,0},
{-496,128,0},
{-496,4,0},
{-491,129,0},
{-508,16,0},
{0,128,0},
{5,-960,0},
{-1024,264,0},
{8,384,0},
{256,0,0},
{1,-1024,0},
{256,1,0},
{256,49,0},
{16,0,0},
{0,-1280,0},
{0,48,0},
{0,144,0},
{-480,128,0},
{-464,3,0},
{-491,128,0},
{-504,2,0},
{0,136,0},
{0,-960,0},
{-958,257,0},
{8,384,0},
{256,48,0},
{0,-1008,0},
{23,24,0},
{256,1,0},
{4,16,0},
{0,-1408,0},
{34,0,0},
{0,132,0},
{-496,0,0},
{-480,0,0},
{-496,130,0},
{-495,9,0},
{0,104,0},
{0,-1020,0},
{-1022,261,0},
{0,388,0},
{289,0,0},
{2,-1024,0},
{26,0,0}};
  
//writes default calibration into memory
int writeCalCmd(char **argv,unsigned short argc){
  //write default data
  const CAL_TBL *newCal=&default_cal;
  int st;     //status from write function
  //write calibration
  st=writeCal(newCal);
  //output status
  if(st==0){
    printf("New Calibration written successfully\r\n");
  }else if(st>0){
    printf("Internal Error\r\n");
  }else{
    printf("Error writing calibration\r\n");
  }
  return st;    
}
    
//clear calibration data command
int calClearCmd(char **argv,unsigned short argc){
  int st=clrCal();
  //output status
  if(st==0){
    printf("calibration cleared\r\n");
  }else{
    printf("Error clearing calibration data\r\n");
  }
}
  
  //table of torquer flips to cover all possible torquer states
  //assume torquers have been initialized
  const struct{
    int num[3],dir[3];
  } flipTable[64]={
                  //num    dir
                  {{2,0,0},{M_PLUS, 0      ,0      }},//1
                  {{1,0,0},{M_MINUS,0      ,0      }},//2
                  {{2,0,0},{M_MINUS,0      ,0      }},//3
                  {{1,2,0},{M_PLUS ,M_PLUS ,0      }},//4
                          
                  {{2,0,0},{M_PLUS ,0      ,0      }},//5
                  {{1,0,0},{M_MINUS,0      ,0      }},//6
                  {{2,0,0},{M_MINUS,0      ,0      }},//7
                  {{1,1,0},{M_PLUS ,M_MINUS,0      }},//8
                                  
                  {{2,0,0},{M_PLUS ,0      ,0      }},//9
                  {{1,0,0},{M_MINUS,0      ,0      }},//10
                  {{2,0,0},{M_MINUS,0      ,0      }},//11
                  {{1,2,0},{M_PLUS ,M_MINUS,0      }},//12
                                          
                  {{2,0,0},{M_PLUS ,0      ,0      }},//13
                  {{1,0,0},{M_MINUS,0      ,0      }},//14
                  {{2,0,0},{M_MINUS,0      ,0      }},//14
                  {{1,1,2},{M_PLUS ,M_PLUS ,M_PLUS }},//15
                  
                  {{2,0,0},{M_PLUS, 0      ,0      }},//16
                  {{1,0,0},{M_MINUS,0      ,0      }},//17
                  {{2,0,0},{M_MINUS,0      ,0      }},//18
                  {{1,2,0},{M_PLUS ,M_PLUS ,0      }},//19
                          
                  {{2,0,0},{M_PLUS ,0      ,0      }},//20
                  {{1,0,0},{M_MINUS,0      ,0      }},//21
                  {{2,0,0},{M_MINUS,0      ,0      }},//22
                  {{1,1,0},{M_PLUS ,M_MINUS,0      }},//23
                                  
                  {{2,0,0},{M_PLUS ,0      ,0      }},//24
                  {{1,0,0},{M_MINUS,0      ,0      }},//25
                  {{2,0,0},{M_MINUS,0      ,0      }},//26
                  {{1,2,0},{M_PLUS ,M_MINUS,0      }},//27
                                          
                  {{2,0,0},{M_PLUS ,0      ,0      }},//28
                  {{1,0,0},{M_MINUS,0      ,0      }},//29
                  {{2,0,0},{M_MINUS,0      ,0      }},//30
                  {{1,1,1},{M_PLUS ,M_PLUS ,M_MINUS}},//31
                  
                  {{2,0,0},{M_PLUS, 0      ,0      }},//32
                  {{1,0,0},{M_MINUS,0      ,0      }},//33
                  {{2,0,0},{M_MINUS,0      ,0      }},//34
                  {{1,2,0},{M_PLUS ,M_PLUS ,0      }},//35
                          
                  {{2,0,0},{M_PLUS ,0      ,0      }},//36
                  {{1,0,0},{M_MINUS,0      ,0      }},//37
                  {{2,0,0},{M_MINUS,0      ,0      }},//38
                  {{1,1,0},{M_PLUS ,M_MINUS,0      }},//39
                                  
                  {{2,0,0},{M_PLUS ,0      ,0      }},//40
                  {{1,0,0},{M_MINUS,0      ,0      }},//41
                  {{2,0,0},{M_MINUS,0      ,0      }},//42
                  {{1,2,0},{M_PLUS ,M_MINUS,0      }},//43
                                          
                  {{2,0,0},{M_PLUS ,0      ,0      }},//44
                  {{1,0,0},{M_MINUS,0      ,0      }},//45
                  {{2,0,0},{M_MINUS,0      ,0      }},//46
                  {{1,1,2},{M_PLUS ,M_PLUS ,M_MINUS}},//47

                  
                  {{2,0,0},{M_PLUS, 0      ,0      }},//48
                  {{1,0,0},{M_MINUS,0      ,0      }},//49
                  {{2,0,0},{M_MINUS,0      ,0      }},//50
                  {{1,2,0},{M_PLUS ,M_PLUS ,0      }},//51
                          
                  {{2,0,0},{M_PLUS ,0      ,0      }},//52
                  {{1,0,0},{M_MINUS,0      ,0      }},//53
                  {{2,0,0},{M_MINUS,0      ,0      }},//54
                  {{1,1,0},{M_PLUS ,M_MINUS,0      }},//55
                                  
                  {{2,0,0},{M_PLUS ,0      ,0      }},//56
                  {{1,0,0},{M_MINUS,0      ,0      }},//57
                  {{2,0,0},{M_MINUS,0      ,0      }},//58
                  {{1,2,0},{M_PLUS ,M_MINUS,0      }},//59
                                          
                  {{2,0,0},{M_PLUS ,0      ,0      }},//60
                  {{1,0,0},{M_MINUS,0      ,0      }},//61
                  {{2,0,0},{M_MINUS,0      ,0      }},//62
                  {{1,1,1},{M_PLUS ,M_PLUS ,M_PLUS }},//63
                  };
 
extern short step;

//calibrate torquers
//make magnetic field readings for all combinations of torquer status and store in a table
int calCmd(char **argv,unsigned short argc){
  //table of measurements
  CAL_TBL tmpCal;
  //torquer set to use
  int set= TQ_SET_BIG;
  int i,idx,st;
  char c;
  //readings from magnetometer
  VEC_INT B0,B1,B2;
  //initialize calibration table
  memset(tmpCal,0x80,sizeof(tmpCal));
  //initialize torquers
  torqueInit();
  //print starting message
  printf("Calibrating Torquers Please Wait\r\n");
  //set step to the first step
  step=0;
  //clear ADC flag
  adcRdy=0;
  //setup timer for event timing
  TACCR1=readTA()+32768;
  TACCTL1=CCIE;
  //run until abort detected
  for(i=0;i<64;){
    //check for ADC samples
    if(adcRdy){
      //store samples    
      switch(step){
        case 2:
          //set sample
          B0.c.x=sample[0];
          B0.c.y=sample[1];
        break;
        case 4:
          //reset sample
          B0.c.x=B0.c.x-sample[0];
          B0.c.x*=-1;    //X - axis is flipped
          B0.c.y=B0.c.y-sample[1];
          B0.c.z=0;//no z-axis sensor
          //print out reading
          if(output_type==HUMAN_OUTPUT){
            printf("1\t% 4i\t% 4i\t% 4i\r\n",B0.c.x,B0.c.y,B0.c.z);
          }else{
            printf("% 4i\t% 4i\t% 4i\t",B0.c.x,B0.c.y,B0.c.z);
          }
        break;
        case 6:
          //set sample
          B1.c.x=sample[0];
          B1.c.y=sample[1];
        break;
        case 8:
          //reset sample
          B1.c.x=B1.c.x-sample[0];
          B1.c.x*=-1;   //X - axis is flipped
          B1.c.y=B1.c.y-sample[1];
          B1.c.z=0;//no z-axis sensor
          //print out reading
          if(output_type==HUMAN_OUTPUT){
            printf("2\t% 4i\t% 4i\t% 4i\r\n",B1.c.x,B1.c.y,B1.c.z);
          }else{
            printf("% 4i\t% 4i\t% 4i\t",B1.c.x,B1.c.y,B1.c.z);
          }
        break;
        case 10:
          //set sample
          B2.c.x=sample[0];
          B2.c.y=sample[1];
        break;
        case 0:
          //reset sample
          B2.c.x=B2.c.x-sample[0];
          B2.c.x*=-1;   //X - axis is flipped
          B2.c.y=B2.c.y-sample[1];
          B2.c.z=0;//no z-axis sensor
          //get index for table
          idx=calIdx();
          //print out reading
          if(output_type==HUMAN_OUTPUT){
            printf("3\t% 4i\t% 4i\t% 4i\r\n",B2.c.x,B2.c.y,B2.c.z);
          }else{
            printf("% 4i\t% 4i\t% 4i\t",B2.c.x,B2.c.y,B2.c.z);
          }
          //check index
          if(idx<0){
            printf("Error illegal index %i in iteration %i\r\n",idx,i);
          }else{
            ivec_cp(&tmpCal[idx],&B1);
          }
          if(output_type==HUMAN_OUTPUT){
            printf("New Torquer Status:\r\n");
            print_torquer_status(set);
          }else{
            print_torquer_stat_code(set);
          }
          //flip torquers
          drive_torquers(set,flipTable[i].num,flipTable[i].dir);
          //increment counter
          i++;
          if(output_type==MACHINE_OUTPUT){
              printf("\r\n");
          }
        break;
      }
      adcRdy=0;
    }
    //check for keys
    if((c=chooseChar())!=EOF){
      //check for ^C
      if(c==0x03 || c=='Q' ||c=='q'){
        break;
      }
    }
    //go to low power mode to wait for next sample
    LPM0;
  }
  //stop timer interrupt
  TACCTL1=0;
  if(c!=0x03){
    st=writeCal((const CAL_TBL*)&tmpCal);
    if(st==0){
      printf("Calibration Updated Successfully\r\n");
    }else{
      printf("Error writing calibration\r\n");
    }
  }
  return 0;
}
 
int calTstCmd(char **argv,unsigned short argc){
  int set= TQ_SET_BIG;
  int i;
  char c;
  VEC Flux;
  VEC_INT B0,B1,B2;
  //initialize torquers
  torqueInit();
  //print starting message
  printf("Testing Torquer Calibration Please Wait\r\n");
  //set step to the first step
  step=0;
  //clear ADC flag
  adcRdy=0;
  //setup timer for event timing
  TACCR1=readTA()+32768;
  TACCTL1=CCIE;
  //run until ^C key is pressed
  for(i=0;i<64;){
    //check for ADC samples
    if(adcRdy){
      //store samples    
      switch(step){
        case 2:
          //set sample
          B0.c.x=sample[0];
          B0.c.y=sample[1];
        break;
        case 4:
          //reset sample
          B0.c.x=B0.c.x-sample[0];
          B0.c.x*=-1;    //X - axis is flipped
          B0.c.y=B0.c.y-sample[1];
          B0.c.z=0;//no z-axis sensor
          //correct for torquers
          correct(&B0);
          //print out reading
          if(output_type==HUMAN_OUTPUT){
            printf("1\t% 4i\t% 4i\t% 4i\r\n",B0.c.x,B0.c.y,B0.c.z);
          }else{
            printf("%4i\t%4i\t%4i\t",B0.c.x,B0.c.y,B0.c.z);
          }
        break;
        case 6:
          //set sample
          B1.c.x=sample[0];
          B1.c.y=sample[1];
        break;
        case 8:
          //reset sample
          B1.c.x=B1.c.x-sample[0];
          B1.c.x*=-1;   //X - axis is flipped
          B1.c.y=B1.c.y-sample[1];
          B1.c.z=0;//no z-axis sensor
          //correct for torquers
          correct(&B1);
          //print out reading
          if(output_type==HUMAN_OUTPUT){
            printf("2\t% 4i\t% 4i\t% 4i\r\n",B1.c.x,B1.c.y,B1.c.z);
          }else{
            printf("%4i\t%4i\t%4i\t",B1.c.x,B1.c.y,B1.c.z);
          }
        break;
        case 10:
          //set sample
          B2.c.x=sample[0];
          B2.c.y=sample[1];
        break;
        case 0:
          //reset sample
          B2.c.x=B2.c.x-sample[0];   
          B2.c.x*=-1;   //X - axis is flipped
          B2.c.y=B2.c.y-sample[1];
          B2.c.z=0;//no z-axis sensor
          //correct for torquers
          correct(&B2);
          //print out reading
          if(output_type==HUMAN_OUTPUT){
            printf("3\t% 4i\t% 4i\t% 4i\r\n",B2.c.x,B2.c.y,B2.c.z);
          }else{
            printf("%4i\t%4i\t%4i\t",B2.c.x,B2.c.y,B2.c.z);
          }
          if(output_type==HUMAN_OUTPUT){
            printf("New Torquer Status:\r\n");
            print_torquer_status(set);
          }else{
            print_torquer_stat_code(set);
          }
          //flip torquers
          drive_torquers(set,flipTable[i].num,flipTable[i].dir);
          //increment counter
          i++;
          if(output_type==MACHINE_OUTPUT){
              printf("\r\n");
          }
        break;
      }
      adcRdy=0;
    }
    //check for keys
    if((c=chooseChar())!=EOF){
      //check for ^C
      if(c==0x03 || c=='Q' ||c=='q'){
        break;
      }
    }
    //go to low power mode to wait for next sample
    LPM0;
  }
  //stop timer interrupt
  TACCTL1=0;
  return 0;
}
              
