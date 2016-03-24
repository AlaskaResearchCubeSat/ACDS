
#include <crc.h>
#include <ctl.h>
#include <msp430.h>
#include <ARCbus.h>
#include <string.h>
#include "torquers.h"
#include "corrections.h"

#pragma constseg("COR")
#pragma zeroedseg("COR")
const COR_STORE correction_data[6];
#pragma constseg(default)
#pragma zeroedseg(default)

//status flags for correction data
unsigned char cor_stat=0;

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
    //check that the first and last bytes were erased
    if(dest->magic!=0xFFFF || dest->crc!=0xFFFF){
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
    //write data
    memcpy(&dest->dat.cor,dat,sizeof(C_AXIS));
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

short erase_correction_dat(int idx){
    int en;
    //pointer to destination that is not constant so compiler does not complain
    COR_STORE *dest=(COR_STORE*)&correction_data[idx];
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
    //re-enable interrupts if enabled before
    BUS_restart_interrupts(en);
    //check that the first and last bytes were erased
    if(dest->magic!=0xFFFF || dest->crc!=0xFFFF){
        //return error
        return WR_COR_ERROR_ERASE_FAIL;
    }  
    return RET_SUCCESS;
}

//read correction data status for all axes
void read_cor_stat(void){
    int i;
    //loop through all sets
    for(i=0;i<6;i++){
        //check correction
        check_cor(i);
    }
}

int check_cor(int idx){
    //check magic
    if(correction_data[idx].magic!=COR_MAGIC){
        //clear status bit
        cor_stat&=~(1<<idx);
        //return error
        return COR_CHK_ERROR_MAGIC;
    }
    //check CRC
    if(correction_data[idx].crc!=crc16((const unsigned char*)&correction_data[idx].dat.cor,sizeof(C_AXIS))){
        //clear status bit
        cor_stat&=~(1<<idx);
        //return error
        return COR_CHK_ERROR_CRC;
    }
    //set status bit
    cor_stat|=(1<<idx);
    //return success
    return RET_SUCCESS;
}

int applyCor(CPOINT *dest,const MAG_POINT *src,int idx){
    const C_AXIS *cor=&correction_data[idx].dat.cor;
    SCL a,b;
    int xidx,yidx,zidx,rt;
    //get idx for all axis
    xidx=stat2Idx(X_AXIS);
    yidx=stat2Idx(Y_AXIS);
    zidx=stat2Idx(Z_AXIS);
    //compute base measurements
    a=cor->scl[0]*src->c.a+cor->scl[1]*src->c.b+cor->baseOS.c.a;
    b=cor->scl[2]*src->c.a+cor->scl[3]*src->c.b+cor->baseOS.c.b;
    //check for valid index
    if(xidx>=0 && yidx>=0 && zidx>=0){
        //compute first measurements
        a+=cor->osX[xidx].c.a+cor->osY[yidx].c.a+cor->osZ[zidx].c.a;
        b+=cor->osX[xidx].c.b+cor->osY[yidx].c.b+cor->osZ[zidx].c.b;
        //success!
        rt=RET_SUCCESS;
    }else{
        //error with correction, only base offsets used
        rt=TQ_ERROR_INVALID_STATUS;
    } 
    //store measurement
    dest->c.a=a;
    dest->c.b=b;
    //return result
    return rt;
}
