#ifndef __TORQUERS_H
#define __TORQUERS_H

#include "vector.h"

//tracking information for a single axis
typedef struct{
  int status;
  unsigned short last;
}TQ_AXIS;

//set of torquers
typedef union{
  struct {
    TQ_AXIS x,y,z;
  }c;
  TQ_AXIS elm[3];
}TQ_SET;

//Pin mask for torquer pins
#define TQ_DRV_PINS  (BIT2|BIT3|BIT4|BIT5|BIT6|BIT7)

#define TQ_DRV_d0    (BIT2)
#define TQ_DRV_d1    (BIT3)
#define TQ_DRV_d2    (BIT4)
#define TQ_DRV_d3    (BIT5)
#define TQ_DRV_d4    (BIT6)
#define TQ_DRV_d5    (BIT7)

#define TQ_OUT0_H    (TQ_DRV_d1)
#define TQ_OUT0_L    (TQ_DRV_d0)
#define TQ_OUT1_H    (TQ_DRV_d3)
#define TQ_OUT1_L    (TQ_DRV_d2)
#define TQ_OUT2_H    (TQ_DRV_d5)
#define TQ_OUT2_L    (TQ_DRV_d4)

//define ports for x-axis torquers
#define X1_DRV_PORT  P8OUT
#define X2_DRV_PORT  P7OUT

#define X1_DRV_DIR   P8DIR
#define X2_DRV_DIR   P7DIR

#define X1_DRV_SEL0  P8SEL0
#define X2_DRV_SEL0  P7SEL0
#define X1_DRV_SEL1  P8SEL0
#define X2_DRV_SEL1  P7SEL0

//define ports for y-axis torquers
#define Y1_DRV_PORT  P6OUT
#define Y2_DRV_PORT  P10OUT

#define Y1_DRV_DIR   P6DIR
#define Y2_DRV_DIR   P10DIR

#define Y1_DRV_SEL0   P6SEL0
#define Y2_DRV_SEL0   P10SEL0
#define Y1_DRV_SEL1   P6SEL1
#define Y2_DRV_SEL1   P10SEL0

//define ports for z-axis torquers
#define Z1_DRV_PORT  P9OUT
#define Z2_DRV_PORT  P5OUT

#define Z1_DRV_DIR  P9DIR
#define Z2_DRV_DIR  P5DIR

#define Z1_DRV_SEL0  P9SEL0
#define Z2_DRV_SEL0  P5SEL0
#define Z1_DRV_SEL1  P9SEL0
#define Z2_DRV_SEL1  P5SEL1

//defines for feedback pins
#define TQ_FB_PIN_MASK  (BIT0|BIT1)

//definitions for feedback bits
#define TQ_FB_X_MASK    (BIT0|BIT1)
#define TQ_FB_Y_MASK    (BIT2|BIT3)
#define TQ_FB_Z_MASK    (BIT4|BIT5)

//Definitions for feedback ports
#define TQ_FB_X     (P8IN)
#define TQ_FB_X_OUT (P8OUT)
#define TQ_FB_X_DIR (P8DIR)
#define TQ_FB_X_REN (P8REN)
#define TQ_FB_X_SEL0 (P8SEL0)
#define TQ_FB_X_SEL1 (P8SEL0)

#define TQ_FB_Y     (P9IN)
#define TQ_FB_Y_OUT (P9OUT)
#define TQ_FB_Y_DIR (P9DIR)
#define TQ_FB_Y_REN (P9REN)
#define TQ_FB_Y_SEL0 (P9SEL0)
#define TQ_FB_Y_SEL1 (P9SEL0)

#define TQ_FB_Z     (P7IN)
#define TQ_FB_Z_OUT (P7OUT)
#define TQ_FB_Z_DIR (P7DIR)
#define TQ_FB_Z_REN (P7REN)
#define TQ_FB_Z_SEL0 (P7SEL0)
#define TQ_FB_Z_SEL1 (P7SEL0)


#define T_STAT_ERR_MASK     (T_STAT_ERR_1|T_STAT_ERR_2|T_STAT_ERR_2|T_STAT_ERR_4| T_STAT_CAP_ERR|T_STAT_COMP_ERR)
#define T_STAT_TQ_MASK      (T_STAT_1|T_STAT_2|T_STAT_3|T_STAT_4)
#define T_STAT_INIT_MASK    (T_STAT_UNINIT_1|T_STAT_UNINIT_2|T_STAT_UNINIT_3|T_STAT_UNINIT_4)

#define T_STAT_TQ_INIT_SHIFT  4

//number of torquers in each axis
#define T_NUM_AXIS_TQ         4

//status bits
enum{T_STAT_1=1<<0,T_STAT_2=1<<1,T_STAT_3=1<<2,T_STAT_4=1<<3,
     T_STAT_UNINIT_1=1<<4,T_STAT_UNINIT_2=1<<5,T_STAT_UNINIT_3=1<<6,T_STAT_UNINIT_4=1<<7,
     T_STAT_ERR_1=1<<8,T_STAT_ERR_2=1<<9,T_STAT_ERR_3=1<<10,T_STAT_ERR_4=1<<11,
     T_STAT_CAP_ERR=1<<12,T_STAT_COMP_ERR=1<<13};
//Directions
enum{M_MINUS=-1,M_PLUS=1};
//Axis
enum{X_AXIS=0,Y_AXIS,Z_AXIS};
//Torquer sets
enum {TQ_SET_ALL=-2,TQ_SET_NONE=-1,TQ_SET_BIG=1};
//errors from torquer sets
enum{TQ_ERR_BAD_SET=-1,TQ_ERR_BAD_TORQUER=-2,TQ_ERR_BAD_DIR=-3,TQ_ERR_COMP=1,TQ_ERR_CAP=2,TQ_ERR_BAD_CONNECTION=3,TQ_INFO_FLIP,TQ_ERROR_INVALID_STATUS,TQ_INFO_TQFB};

int setTorque(const VEC *T);
int drive_torquer(int axis,unsigned char num,unsigned char dir);
void torqueInit(void);
void torqueReinit(void);
void driverInit(void);
int drive_torquers(const int* num,const int* dir);
void print_torquer_status(void);
void print_torquer_stat_code(void);
short checkTorqueInit(void);
void torque_fb_init(void);
void resetTorqueStatus(void);

unsigned char get_torquer_fb(void);
void tqstat2stat(unsigned char *dest);

int stat2Idx(int idx);

void get_stat(TQ_SET *dest);

void print_tqstat(const TQ_SET *stat);
void print_tqstat_code(const TQ_SET *stat);

#endif
