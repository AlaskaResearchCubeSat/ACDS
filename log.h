#ifndef __LOG_H
#define __LOG_H
    #include "vector.h"
    #include "quat.h"
    #include "torquers.h"
    #include <error.h>
    #include <ARCbus.h>
    
    #define ACDS_LOG_MAGIC      0xAA72
    
    #define ACDS_LOG_VERSION    2
    
    //structure for log data
    typedef struct{
        unsigned char mode;
        unsigned char flags;
        VEC flux;
        VEC M_cmd;
        TQ_SET tq_stat;
        union{
            struct{
                VEC B_dot;
            
            } mode1;
            struct{
                short window;
            } mode2;
            struct{
                short window;
            } mode3;
        }mdat;
    }LOG_DAT;

    typedef struct{
        unsigned short magic;
        unsigned char version;
        unsigned char flags;
        unsigned short number;
        ticker time;
        union{
            LOG_DAT acds_dat;
            unsigned char pad[500];
        }dat;
        unsigned short crc;
    }LOG_DAT_STORE;

    //Address range for ERROR data on the SD card
    enum{LOG_ADDR_START=ERR_ADDR_END+1,LOG_ADDR_END=LOG_ADDR_START+500};

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
    