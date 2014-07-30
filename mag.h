#ifndef __MAG_H
#define __MAG_H

    
    //magnetometer point
    typedef union{
      struct {
        short a,b;
      }c;
      short elm[2];
    } MAG_POINT;

    typedef struct{
        unsigned short flags;
        MAG_POINT meas[6];
    }MAG_DAT; 

    //flags for magFLags
#define MAG_FLAGS_X0      (1<<(2*2+0))      //Y+ board a-axis
#define MAG_FLAGS_X1      (1<<(2*3+0))      //Y- board a-axis
#define MAG_FLAGS_X2      (1<<(2*4+1))      //Z+ board b-axis
#define MAG_FLAGS_X3      (1<<(2*5+1))      //Z- board b-axis
#define MAG_FLAGS_X       (MAG_FLAGS_X0|MAG_FLAGS_X1|MAG_FLAGS_X2|MAG_FLAGS_X3)

#define MAG_FLAGS_Y0      (1<<(2*0+0))      //X+ board a-axis
#define MAG_FLAGS_Y1      (1<<(2*1+0))      //X- board a-axis
#define MAG_FLAGS_Y2      (1<<(2*4+0))      //Z+ board a-axis
#define MAG_FLAGS_Y3      (1<<(2*5+0))      //Z- board a-axis
#define MAG_FLAGS_Y       (MAG_FLAGS_Y0|MAG_FLAGS_Y1|MAG_FLAGS_Y2|MAG_FLAGS_Y3)

#define MAG_FLAGS_Z0      (1<<(2*0+1))      //X+ board b-axis    
#define MAG_FLAGS_Z1      (1<<(2*1+1))      //X- board b-axis
#define MAG_FLAGS_Z2      (1<<(2*2+1))      //Y+ board b-axis
#define MAG_FLAGS_Z3      (1<<(2*3+1))      //Y- board b-axis
#define MAG_FLAGS_Z       (MAG_FLAGS_Z0|MAG_FLAGS_Z1|MAG_FLAGS_Z2|MAG_FLAGS_Z3)

#endif
    