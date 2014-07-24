#include <stdio.h>
#include <SDlib.h>
#include <ctl.h>
#include <ARCbus.h>
#include <crc.h>
#include "log.h"
#include "output_type.h"


//current ACDS data
LOG_DAT_STORE acds_dat;

static SD_blolck_addr current_log_block;

unsigned short next_log_idx;

void print_log_dat(const LOG_DAT *dat){
    //print ACDS mode
    if(output_type==HUMAN_OUTPUT){
        printf("Mode : %i\r\n",dat->mode);
    }else{
        printf("M%i\t",dat->mode);
    }
    //print magnetic flux vector
    vecPrint("Flux",&dat->flux);
    //print mode specific data
    switch(dat->mode){
        case 1:
            vecPrint("B-dot",&dat->mdat.mode1.B_dot);
        break;
    }
    //print commanded dipole moment
    vecPrint("Mcmd",&dat->M_cmd);
    //print new torquer status
    if(output_type==HUMAN_OUTPUT){
        print_tqstat(&dat->tq_stat);
    }else{
        print_tqstat_code(&dat->tq_stat);
        //print trailing newline
        printf("\r\n");
    }
}


void log_init(void){
    next_log_idx=0;
    current_log_block=-1;
    acds_dat.flags=LOG_INIT_FLAGS;
    acds_dat.magic=ACDS_LOG_MAGIC;
}

void log_start(void){
    int resp,found,i;
    SD_blolck_addr addr,found_addr;
    LOG_DAT_STORE *blk;
    unsigned char *buf;
    unsigned short number;
    //lock card so that initialization is not interrupted
    resp=mmcLock(CTL_TIMEOUT_DELAY,4096);
    //check if card was locked
    if(resp==MMC_SUCCESS){
      //initialize the card
      resp=mmcInit_card();
      //check if card was initialized
      if(resp==MMC_SUCCESS){
        //get buffer 
        buf=BUS_get_buffer(CTL_TIMEOUT_DELAY,100);
        //check if buffer acquired
        if(buf){
          //look for previous logs on the SD card
          for(addr=LOG_ADDR_START,found_addr=0,found=0,number=0;addr<LOG_ADDR_END;addr++){
            //read block
            resp=mmcReadBlock(addr,buf);
            //check for error
            if(resp==MMC_SUCCESS){
              //check for valid error block
              blk=(LOG_DAT_STORE*)buf;
              //check signature values
              if(blk->magic==ACDS_LOG_MAGIC){
                //TODO: check CRC?
                //check block number is greater then found block
                if(blk->number>=number){
                  found_addr=addr;
                  found=1;
                  number=blk->number;
                }
              }
            }else{
                //read failed
                //TODO: handle error 
            }
          }
        }
        //TODO: check for errors
        //check if an address was found
        if(found){
          //set error address
          current_log_block=found_addr+1;
          //check for wraparound
          if(current_log_block>LOG_ADDR_END){
            current_log_block=LOG_ADDR_START;
          }
          //set number
          acds_dat.number=number+1;
        }else{
          //set address to first address
          current_log_block=LOG_ADDR_START;
          //set number to zero
          acds_dat.number=0;
        }
        //done using buffer
        BUS_free_buffer();
        //done using card, unlock
        mmcUnlock();
      }else{
        //could not lock SD card
        //TODO: handle error
      }
    }else{
      //could not init card
      //TODO: handle error
    }
}

int log_store_data(LOG_DAT_STORE *data){
    int result;
    if(current_log_block==-1){
        //log has not been initialized
        return ACDS_LOG_ERR_UNINIT;
    }
    //compute CRC
    data->crc=crc16(data,sizeof(LOG_DAT_STORE)-sizeof(data->crc));
    //write block
    result=mmcWriteBlock(current_log_block,(unsigned char*)data);
    //increment number regardless
    data->number++;
    //check if block got written
    if(result==RET_SUCCESS){
        //set error address
        current_log_block=current_log_block+1;
        //check for wraparound
        if(current_log_block>LOG_ADDR_END){
            current_log_block=LOG_ADDR_START;
        }
    }
    data->flags&=~LOG_FLAGS_FIRST;
    //return result from write
    return result;
}

int clear_log(void){
    return mmcErase(LOG_ADDR_START,LOG_ADDR_END);
    next_log_idx=0;
    current_log_block=LOG_ADDR_START;
    acds_dat.flags=LOG_INIT_FLAGS;
}

//print data from the log starting with the most recent ones
//print only errors with a level greater than level up to a maximum of num errors 
void log_replay(unsigned short num){
  unsigned short count=0;
    SD_blolck_addr start,addr;
    LOG_DAT_STORE *store;
    unsigned short number=acds_dat.number;
    unsigned char *buf;
    int i,resp,last=0;
    if(addr==-1){
        printf("Error : data log not initialized\r\n");
        return;
    }
    //lock card so that we are not interrupted
    resp=mmcLock(CTL_TIMEOUT_DELAY,10);
    //check if card was locked
    if(resp!=MMC_SUCCESS){
        printf("Error : Failed to lock SD card : %s\r\n",SD_error_str(resp));
    }
    //get buffer 
    buf=BUS_get_buffer(CTL_TIMEOUT_DELAY,100);
    //set start
    start=current_log_block-1;
    //check for wraparound
    if(start<LOG_ADDR_START){
        start=LOG_ADDR_END;
    }
    //set address to start
    addr=start;
    //check if buffer acquired
    if(buf){
        for(;;){
            //read block
            resp=mmcReadBlock(addr,buf);
            //check for error
            if(resp==MMC_SUCCESS){
                //check for valid error block
                store=(LOG_DAT_STORE*)buf;
                //check signature values
                if(store->magic==ACDS_LOG_MAGIC){
                    //check CRC
                    if(store->crc==crc16(store,sizeof(LOG_DAT_STORE)-sizeof(store->crc))){
                        if(number!=store->number){
                            //print message
                            printf("Missing block(s) expected #%u got #%u\r\n",number,store->number);
                            //update number
                            number=store->number;
                        }
                        //print block
                        print_log_dat(&store->dat.acds_dat);
                        //check if we are counting
                        if(num!=0){
                            //increment count
                            count++;
                            //check if enough blocks have been printed
                            if(count>=num){
                                //done!
                                break;
                            }
                        }
                    }else{
                        //block CRC is not valid, print error
                        printf("Error : invalid block CRC\r\n");
                    }
                }else{
                    //check if this block is expected to be the last
                    if(last){
                      //exit loop
                      break;
                    }
                    //block header is not valid
                    printf("Error : invalid block header\r\n");
                }
            }else{
                //error reading from SD card
                printf("Error : failed to read from SD card : %s\r\n",SD_error_str(resp));
                //exit loop to prevent further errors
                break; 
            }
            //check if this should be the last block
            if(number==0){
                //set flag so code can exit silently next time
                last=1;
            }
            //next block should have a lower number decrement
            number--;
            //check if address is at the beginning
            if(addr<=LOG_ADDR_START){
                //set address to the end
                addr=LOG_ADDR_END;
            }else{
                //decrement address
                addr--;
            }
            //check if there are more errors to display
            if(addr==start){
                //replay complete, exit
                break;
            }
        }
        //free buffer
        BUS_free_buffer();
    }else{
        printf("Error : failed to get buffer\r\n");
    }
    //unlock card
    mmcUnlock();
}
