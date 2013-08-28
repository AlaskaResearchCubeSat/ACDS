#ifndef __TORQUERS_H
#define __TORQUERS_H

#include "vector.h"

//tracking information for a single axis
typedef struct{
  int status;
  int last;
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
#define X_DRV_PORT1  P7OUT
#define X_DRV_PORT2  P5OUT

#define X_DRV_DIR1  P7DIR
#define X_DRV_DIR2  P5DIR

#define X_DRV_SEL1  P7SEL
#define X_DRV_SEL2  P5SEL

//define ports for y-axis torquers
#define Y_DRV_PORT1  P2OUT
#define Y_DRV_PORT2  P8OUT

#define Y_DRV_DIR1   P2DIR
#define Y_DRV_DIR2   P8DIR

#define Y_DRV_SEL1   P2SEL
#define Y_DRV_SEL2   P8SEL

//define ports for z-axis torquers
#define Z_DRV_PORT1  P4OUT
#define Z_DRV_PORT2  P6OUT

#define Z_DRV_DIR1  P4DIR
#define Z_DRV_DIR2  P6DIR

#define Z_DRV_SEL1  P4SEL
#define Z_DRV_SEL2  P6SEL

//defines for feedback pins
#define TQ_FB_PIN_MASK  (BIT0|BIT1)

//definitions for feedback bits
#define TQ_FB_X_MASK    (BIT0|BIT1)
#define TQ_FB_Y_MASK    (BIT2|BIT3)
#define TQ_FB_Z_MASK    (BIT4|BIT5)

//Definitions for feedback ports
#define TQ_FB_X     (P7IN)
#define TQ_FB_X_OUT (P7OUT)
#define TQ_FB_X_DIR (P7DIR)
#define TQ_FB_X_REN (P7REN)
#define TQ_FB_X_SEL (P7SEL)

#define TQ_FB_Y     (P8IN)
#define TQ_FB_Y_OUT (P8OUT)
#define TQ_FB_Y_DIR (P8DIR)
#define TQ_FB_Y_REN (P8REN)
#define TQ_FB_Y_SEL (P8SEL)

#define TQ_FB_Z     (P6IN)
#define TQ_FB_Z_OUT (P6OUT)
#define TQ_FB_Z_DIR (P6DIR)
#define TQ_FB_Z_REN (P6REN)
#define TQ_FB_Z_SEL (P6SEL)


#define T_STAT_ERR_MASK     (T_STAT_ERR_1|T_STAT_ERR_2|T_STAT_CAP_ERR|T_STAT_COMP_ERR)
#define T_STAT_TQ_MASK      (T_STAT_1|T_STAT_2|T_STAT_UNINIT_1|T_STAT_UNINIT_2)

//status bits
enum{T_STAT_1=1<<0,T_STAT_2=1<<1,T_STAT_UNINIT_1=1<<2,T_STAT_UNINIT_2=1<<3,T_STAT_ERR_1=1<<4,T_STAT_ERR_2=1<<5,T_STAT_CAP_ERR=1<<6,T_STAT_COMP_ERR=1<<7};
//Directions
enum{M_MINUS=-1,M_PLUS=1};
//Axis
enum{X_AXIS=0,Y_AXIS,Z_AXIS};
//Torquer sets
enum {TQ_SET_ALL=-2,TQ_SET_NONE=-1,TQ_SET_BIG=1,TQ_SET_SMALL=2};
//errors from torquer sets
enum{TQ_ERR_BAD_SET=-1,TQ_ERR_BAD_TORQUER=-2,TQ_ERR_COMP=1,TQ_ERR_CAP=2,TQ_ERR_BAD_CONNECTION=3,TQ_INFO_FLIP,TQ_ERROR_INVALID_STATUS};

int setTorque(const VEC *T,int set);
int drive_torquer(int axis,int set,unsigned char num,unsigned char dir);
void torqueInit(void);
void driverInit(void);
int drive_torquers(int set,const int* num,const int* dir);
void print_torquer_status(int set);
void print_torquer_stat_code(int set);
short checkTorqueInit(void);
void torque_fb_init(void);
void resetTorqueStatus(void);

unsigned char get_torquer_fb(void);

#endif
