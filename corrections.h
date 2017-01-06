#ifndef __CORRECTIONS_H
    #define __CORRECTIONS_H
    
    #include "vector.h"
    #include "sensor-interface.h"
    
    #define COR_MAGIC       0xAA43
    
    //return values for write_correction_dat
    enum{WR_COR_ERROR_WRITE_ERROR=-1,WR_COR_ERROR_CRC_MSIMATCH=-2,COR_CHK_ERROR_MAGIC=-3,COR_CHK_ERROR_CRC=-4,WR_COR_ERROR_ERASE_FAIL=-5};
        
    //status flags for correction data
    enum{COR_ST_X_MINUS=1<<0,COR_ST_X_PLUS=1<<1,COR_ST_Y_MINUS=1<<2,COR_ST_Y_PLUS=1<<3,COR_ST_Z_MINUS=1<<4,COR_ST_Z_PLUS=1<<5};
   
    
    //corection point
    typedef union{
      struct {
        SCL a,b;
      }c;
      SCL elm[2];
    } CPOINT;
    
    //corrections structure
    typedef struct{
        SCL scl[4];
        CPOINT baseOS,osX[16],osY[16],osZ[16];
    } C_AXIS;
    
    typedef struct{
        unsigned short magic;
        union {
            C_AXIS cor;
            unsigned char pad[508];
        }dat;
        unsigned short crc;
    }COR_STORE;

    //correction data
    extern const COR_STORE correction_data[6];

    //correction data status
    extern unsigned char cor_stat;
    
    //write corrections data to flash
    short write_correction_dat(int idx,const C_AXIS *dat);
    //check if corrections data is present
    int check_cor(int idx);
    //read correction data status for all axes
    void read_cor_stat(void);
    //apply corrections to measurement
    int applyCor(CPOINT *dest,const MAG_POINT *src,int idx);
    //erase corrections data for a given SPB index
    short erase_correction_dat(int idx);

#endif
    