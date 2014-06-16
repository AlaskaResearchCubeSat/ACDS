
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
    int en,i;
    unsigned short crc;
    unsigned short *s,*d;
    //pointer to destination that is not constant so compiler does not complain
    COR_STORE *dest=(COR_STORE*)&correction_data[idx];
    //first compute CRC
    crc=crc16((const unsigned char*)dat,sizeof(C_AXIS));
    //disable interrupts
    en = BUS_stop_interrupts();
    //disable watchdog
    WDT_STOP();
    //unlock flash memory
    FCTL3=FWKEY;
    //setup flash for erase
    FCTL1=FWKEY|ERASE;
    //dummy write to indicate which segment to erase
    dest->magic=0;
    //lock the flash again
    FCTL3=FWKEY|LOCK;
    //check fail flag
    if(FCTL3&FAIL || dest->magic!=0xFFFF){
        //re-enable interrupts if enabled before
        BUS_restart_interrupts(en);
        return WR_COR_ERROR_ERASE_FAIL;
    }  
    
    //unlock flash memory
    FCTL3=FWKEY; 
    //enable writing
    FCTL1=FWKEY|WRT;
    //write magic
    dest->magic=COR_MAGIC;
    //write CRC
    dest->crc=crc;
    //disable writing
    FCTL1=FWKEY;
    //lock flash
    FCTL3=FWKEY|LOCK;
    
    for(i=0,s=(unsigned short*)dat,d=(unsigned short*)&dest->dat.cor;i<(sizeof(C_AXIS)+1)/2;i++){
        //unlock flash memory
        FCTL3=FWKEY; 
        //enable writing
        FCTL1=FWKEY|WRT;
        //write settings
        *d++=*s++;
        //disable writing
        FCTL1=FWKEY;
        //lock flash
        FCTL3=FWKEY|LOCK;
    }
    
    //unlock flash memory
    FCTL3=FWKEY; 
    //enable writing
    FCTL1=FWKEY|WRT;
    //write CRC
    dest->crc=crc;
    //disable writing
    FCTL1=FWKEY;
    //lock flash
    FCTL3=FWKEY|LOCK;
    //re-enable interrupts if enabled before
    BUS_restart_interrupts(en);
    //check for errors
    if(dest->magic!=COR_MAGIC){
        return WR_COR_ERROR_WRITE_ERROR;
    }
    if(crc!=crc16((const unsigned char*)&dest->dat.cor,sizeof(C_AXIS)) || crc!=dest->crc){
        return WR_COR_ERROR_CRC_MSIMATCH;
    }
    return RET_SUCCESS;
}

int check_cor(int idx){
    if(correction_data[idx].magic!=COR_MAGIC){
        return COR_CHK_ERROR_MAGIC;
    }
    if(correction_data[idx].crc!=crc16((const unsigned char*)&correction_data[idx].dat.cor,sizeof(C_AXIS))){
        return COR_CHK_ERROR_CRC;
    }
    return RET_SUCCESS;
}
