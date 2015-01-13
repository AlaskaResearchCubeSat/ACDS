#ifndef __LOG_H
#define __LOG_H
    #include "vector.h"
    #include "quat.h"
    #include "torquers.h"
    #include "mag.h"
    #include "filter.h"
    #include <error.h>
    #include <ARCbus.h>
    
    #define ACDS_LOG_MAGIC      0xAA72
    
    #define ACDS_LOG_VERSION    5
    
    //structure for log data
    typedef struct{
        //ACDS mode
        unsigned char mode;
        //flags, decide what these are for
        unsigned char flags;
        //torquer feedback
        unsigned char fb1,fb2;
        //magnetic flux vector
        VEC flux;
        //commanded dipole moments
        VEC M_cmd;
        //torquer status
        TQ_SET tq_stat;
        //running torquer flip count
        unsigned short flips[3];
        //raw magnetometer data
        MAG_DAT raw_mag;
        //gyro readings
        VEC_INT gyro;
        //mode defined data
        union{
            struct{
                //magnetic field derivative
                VEC B_dot;
                //filter memory
                float z_xmag[FILTER_MAX_A],z_ymag[FILTER_MAX_A],z_zmag[FILTER_MAX_A];
            } mode1;
            struct{
                //bias window
                short window;
            } mode2;
            struct{
                //bias window
                short window;
            } mode3;
        }mdat;
    }LOG_DAT;

    typedef struct{
        unsigned short magic;
        //block number incremented each time a block is stored
        unsigned short number;
        //version, changes when structure gets updated
        unsigned char version;
        //flags, decide what these are for
        unsigned char flags;
        //captures the time the block was stored
        ticker time;
        union{
            //data to log
            LOG_DAT acds_dat;
            //padding 
            unsigned char pad[500];
        }dat;
        unsigned short crc;
    }LOG_DAT_STORE;

    //Address range for ERROR data on the SD card
    enum{LOG_ADDR_START=ERR_ADDR_END+1,LOG_ADDR_END=LOG_ADDR_START+500,SD_FIRST_FREE_ADDR};

    enum{ACDS_LOG_ERR_UNINIT=-15};
    
    //flags for log blocks
    enum{LOG_FLAGS_FIRST=1<<0};
    
    //initial state for flags
    #define LOG_INIT_FLAGS          LOG_FLAGS_FIRST

    //ACDS status
    extern LOG_DAT_STORE acds_dat;

    //print data structure
    void print_log_dat(const LOG_DAT *dat);
    
    void log_init(void);
    
    int clear_log(void);
    
    void log_init(void);
    
    void log_start(void);
    
    int log_store_data(LOG_DAT_STORE *data);
    
    void log_replay(unsigned short num);

#endif
    