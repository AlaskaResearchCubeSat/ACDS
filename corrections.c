
#include <crc.h>
#include <ctl.h>
#include <msp430.h>
#include <ARCbus.h>
#include <string.h>
#include "corrections.h"

#pragma constseg("COR")
#pragma zeroedseg("COR")
const COR_STORE correction_data[6];
#pragma constseg(default)
#pragma zeroedseg(default)

short write_correction_dat(int idx,const C_AXIS *dat){
    int en;
    unsigned short crc;
    //pointer to destination that is not constant so compiler does not complain
    COR_STORE *dest=(COR_STORE*)&correction_data[idx];
    //first compute CRC
    crc=crc16((const unsigned char*)dat,sizeof(C_AXIS));
    //disable interrupts
    en = ctl_global_interrupts_set(0);
    //disable watchdog
    WDT_STOP();
    //unlock flash memory
    FCTL3=FWKEY;
    //setup flash for erase
    FCTL1=FWKEY|ERASE;
    //dummy write to indicate which segment to erase
    dest->magic=0;
    //enable writing
    FCTL1=FWKEY|WRT;
    //write magic
    dest->magic=COR_MAGIC;
    //write settings
    memcpy(&dest->dat.cor,dat,sizeof(C_AXIS));
    //write CRC
    dest->crc=crc;
    //disable writing
    FCTL1=FWKEY;
    //lock flash
    FCTL3=FWKEY|LOCK;
    //Kick WDT to restart it
    WDT_KICK();
    //re-enable interrupts if enabled before
    if (en){
      ctl_global_interrupts_enable();
    }
    //check for errors
    if(dest->magic!=COR_MAGIC){
        return WR_COR_ERROR_WRITE_ERROR;
    }
    if(crc!=crc16((const unsigned char*)&dest->dat.cor,sizeof(C_AXIS)) || crc!=dest->crc){
        return WR_COR_ERROR_CRC_MSIMATCH;
    }
    return RET_SUCCESS;
}
