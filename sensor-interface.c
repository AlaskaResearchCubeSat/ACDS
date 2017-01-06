#include <msp430.h>
#include <ctl.h>
#include <ARCbus.h>
#include <i2c.h>
#include <string.h>
//TESTING: include stdio
#include <stdio.h>
#include <Error.h>
#include "timerA0.h"
#include "LTC24XX.h"
#include "sensor-interface.h"
#include "pins.h"
#include "ACDSerr.h"
#include "ACDS.h"
#include <limits.h>


CTL_EVENT_SET_t sens_ev;

//the time that the ADC can next be sampled
CTL_TIME_t adc_ready_time=153;

unsigned short mag_ADC_gain=1;

//setup P8 for magnetomitor usage
void mag_init(void){ 
  //set output low
  MAG_SR_OUT&=~MAG_SR_PIN;
  //use GPIO functions
  MAG_SR_SEL0&=~MAG_SR_PIN;
  MAG_SR_SEL1&=~MAG_SR_PIN;
  //set as output
  MAG_SR_DIR|=MAG_SR_PIN;
}

//TESTING: trigger a sensor read
void trigger_read(void){
  ctl_events_set_clear(&sens_ev,SENS_EV_READ,0);
}
  

//convert ADC value to voltage
float ADCtoV(short adc){
  //TODO: maybe allow other references
  return ((float)adc)*3.3/(2*65535.0);
}

//compute ADC value to magnetic field value in gauss
float ADCtoGauss(long adc){
  return  ADCtoV(adc)/(AMP_GAIN*MAG_SENS);
}



MAG_POINT magMem[6];
unsigned short magFlags;

//LED stuff
void meas_LED_on(void){
  P7OUT|=BIT1;
}

void meas_LED_off(void){
  P7OUT&=~BIT1;
}

void sens_err_LED_on(void){
  P7OUT|=BIT0;
}

void sens_err_LED_off(void){
  P7OUT&=~BIT0;
}

void com_err_LED_on(void){
  P7OUT|=BIT4;
}

void com_err_LED_off(void){
  P7OUT&=~BIT4;
}

//address for magnetometer        X+   X-   Y+   Y-   Z+   Z-
const unsigned char mag_addrs[6]={0x14,0x16,0x26,0x34,0x25,0x24};

//take a reading from the magnetometer ADC
short do_conversion(void){
  long aval,bval;
  unsigned char rxbuf[4],txbuf[4];
  int res,i;
  CTL_TIME_t ct;
  //get current time
  ct=ctl_get_current_time();
  //check if ADC is ready
  if((ct-adc_ready_time)<-3){
    //wait for ADC to be ready
    ctl_timeout_wait(adc_ready_time);
  }
  //turn on LED while measuring
  meas_LED_on();
  //generate set pulse
  MAG_SR_OUT|=MAG_SR_PIN;
  //delay for pulse
  //__delay_cycles(16000);
  ctl_timeout_wait(ctl_get_current_time()+2);
  //configure for first conversion use channel 0
  txbuf[0]=LTC24XX_PRE|LTC24XX_EN|MAG_A_CH;
  #ifdef MAG_ADC_GAIN
    txbuf[1]=LTC24xx_EN2|LTC24xx_FA|MAG_ADC_GAIN;                   
  #else
    txbuf[1]=LTC24xx_EN2|LTC24xx_FA;
    switch(mag_ADC_gain){
      case 1:
        txbuf[1]|=LTC24xx_GAIN1;
      break;
      case 4:
        txbuf[1]|=LTC24xx_GAIN4;
      break;
      case 8:
        txbuf[1]|=LTC24xx_GAIN8;
      break;
      case 16:
        txbuf[1]|=LTC24xx_GAIN16;
      break;
      case 32:
        txbuf[1]|=LTC24xx_GAIN32;
      break;
      case 64:
        txbuf[1]|=LTC24xx_GAIN64;
      break;
      case 128:
        txbuf[1]|=LTC24xx_GAIN128;
      break;
      case 264:
        txbuf[1]|=LTC24xx_GAIN264;
      break;
      default:
        report_error(ERR_LEV_ERROR,ACDS_ERR_SRC_SENSORS,ACDS_ERR_SEN_BAD_GAIN,mag_ADC_gain);
        return 1;
    }
  #endif
  //send setup message to global address
  if((res=i2c_tx(LTC24XX_GLOBAL_ADDR,txbuf,2))<0){
    //perhaps a conversion is in progress, wait for it to complete
    ctl_timeout_wait(ctl_get_current_time()+153);    //wait about 150ms
    //report a warning
    report_error(ERR_LEV_WARNING,ACDS_ERR_SRC_SENSORS,ACDS_ERR_SEN_SETUP,res);
    //try sending again
    if((res=i2c_tx(LTC24XX_GLOBAL_ADDR,txbuf,2))<0){
      //report error
      report_error(ERR_LEV_ERROR,ACDS_ERR_SRC_SENSORS,ACDS_ERR_SEN_SETUP,res);
      //turn on error LED
      sens_err_LED_on();
      //turn LED off, done measuring
      meas_LED_off();
      //clear flags
      magFlags=0;
      //TODO : provide real error codes
      return 2;
    }
  }
  //wait for conversion to complete
  ctl_timeout_wait(ctl_get_current_time()+153);    //wait about 150ms
  //config for next conversion use channel 1
  txbuf[0]=LTC24XX_PRE|LTC24XX_EN|MAG_B_CH;
  for(i=0;i<6;i++){
    //read in data and start next conversion
    if((res=i2c_txrx(mag_addrs[i],txbuf,1,rxbuf,3))<0){
      //report error
      report_error(ERR_LEV_WARNING,ACDS_ERR_SRC_SENSORS,ACDS_ERR_SEN_CONV_READ,res);
      //turn on error LED
      sens_err_LED_on();
      //clear value
      magMem[i].c.a=0;
      //clear valid flag
      magFlags&=~(1<<(2*i));
    }else{
      //get result
      magMem[i].c.a=adc16Val(rxbuf);
      //set valid flag
      magFlags|= (1<<(2*i));
    }
  }
  //wait for conversion to complete
  ctl_timeout_wait(ctl_get_current_time()+153);    //wait about 150ms
  for(i=0;i<6;i++){
    //read in data
    if((res=i2c_rx(mag_addrs[i],rxbuf,3))<0){
      //report error
      report_error(ERR_LEV_WARNING,ACDS_ERR_SRC_SENSORS,ACDS_ERR_SEN_CONV_READ,res);
      //turn on error LED
      sens_err_LED_on();
      //clear value
      magMem[i].c.b=0;
      //clear valid flag
      magFlags&=~(1<<(2*i+1));
    }else{
      //get result
      magMem[i].c.b=adc16Val(rxbuf);
      //set valid flag
      magFlags|= (1<<(2*i+1));
    }
  }
  //generate reset pulse
  MAG_SR_OUT&=~MAG_SR_PIN;
  //turn LED off, done measuring
  meas_LED_off();
  //get time that ADC can next be read
  adc_ready_time=ctl_get_current_time()+153;
  //check for data in all 3 axis
  if((magFlags&MAG_FLAGS_X) && (magFlags&MAG_FLAGS_Y) && (magFlags&MAG_FLAGS_Z)){
    //TODO: provide real return codes
    return 0;
  }else{
    //report error
    report_error(ERR_LEV_ERROR,ACDS_ERR_SRC_SENSORS,ACDS_ERR_SEN_INSUFFICIENT_DATA,magFlags);
    //TODO: provide real return codes
    return 0;
  }
}


//take a reading from the magnetomitor ADC
short single_sample(unsigned short addr,short *dest){
  unsigned char rxbuf[4],txbuf[4];
  int res;
  CTL_TIME_t ct;
  //generate set pulse
  MAG_SR_OUT|=MAG_SR_PIN;
  //delay for pulse
  ctl_timeout_wait(ctl_get_current_time()+2);
  //get current time
  ct=ctl_get_current_time();
  //check if ADC is ready
  if((ct-adc_ready_time)<-3){
    ctl_timeout_wait(adc_ready_time);
  }
  //turn on LED while measuring
  meas_LED_on();
  //configure for first conversion convert in the A-axis
  txbuf[0]=LTC24XX_PRE|LTC24XX_EN|MAG_A_CH;
  #ifdef MAG_ADC_GAIN
    txbuf[1]=LTC24xx_EN2|LTC24xx_FA|MAG_ADC_GAIN;                   
  #else
    txbuf[1]=LTC24xx_EN2|LTC24xx_FA;
    switch(mag_ADC_gain){
      case 1:
        txbuf[1]|=LTC24xx_GAIN1;
      break;
      case 4:
        txbuf[1]|=LTC24xx_GAIN4;
      break;
      case 8:
        txbuf[1]|=LTC24xx_GAIN8;
      break;
      case 16:
        txbuf[1]|=LTC24xx_GAIN16;
      break;
      case 32:
        txbuf[1]|=LTC24xx_GAIN32;
      break;
      case 64:
        txbuf[1]|=LTC24xx_GAIN64;
      break;
      case 128:
        txbuf[1]|=LTC24xx_GAIN128;
      break;
      case 264:
        txbuf[1]|=LTC24xx_GAIN264;
      break;
      default:
        report_error(ERR_LEV_ERROR,ACDS_ERR_SRC_SENSORS,ACDS_ERR_SEN_BAD_GAIN,mag_ADC_gain);
        //generate reset pulse
        MAG_SR_OUT&=~MAG_SR_PIN;
        return 1;
    }
  #endif  
  if((res=i2c_tx(addr,txbuf,2))<0){
    if(res==I2C_ERR_NACK){
      //perhaps a conversion is in progress, wait for it to complete
      ctl_timeout_wait(ctl_get_current_time()+153);    //wait about 150ms
      //report a warning
      report_error(ERR_LEV_WARNING,ACDS_ERR_SRC_SENSORS,ACDS_ERR_SEN_SETUP,res);
      //try sending again
      res=i2c_tx(addr,txbuf,2);
    }
    if(res<0){
      //report error
      report_error(ERR_LEV_ERROR,ACDS_ERR_SRC_SENSORS,ACDS_ERR_SEN_SETUP,res);
      //turn on error LED
      sens_err_LED_on();
      //generate reset pulse
      MAG_SR_OUT&=~MAG_SR_PIN;
      //turn LED off, done measuring
      meas_LED_off();
      //TODO : provide real error codes
      return 2;
    }
  }
  //wait for conversion to complete
  ctl_timeout_wait(ctl_get_current_time()+153);    //wait about 150ms
  //config for next conversion in the B-axis
  txbuf[0]=LTC24XX_PRE|LTC24XX_EN|MAG_B_CH;
  //read in data and start next conversion
  if((res=i2c_txrx(addr,txbuf,1,rxbuf,3))<0){
    //report error
    report_error(ERR_LEV_ERROR,ACDS_ERR_SRC_SENSORS,ACDS_ERR_SEN_CONV_READ,res);
    //turn on error LED
    sens_err_LED_on();
    //generate reset pulse
    MAG_SR_OUT&=~MAG_SR_PIN;
    //turn LED off, done measuring
    meas_LED_off();
    //TODO : provide real error codes
    return 2;
  }
  //save result
  dest[0]=adc16Val(rxbuf);
  //wait for conversion to complete
  ctl_timeout_wait(ctl_get_current_time()+153);    //wait about 150ms
  //read in data
  if((res=i2c_rx(addr,rxbuf,3))<0){
    //report error
    report_error(ERR_LEV_ERROR,ACDS_ERR_SRC_SENSORS,ACDS_ERR_SEN_CONV_READ,res);
    //turn on error LED
    sens_err_LED_on();
    //generate reset pulse
    MAG_SR_OUT&=~MAG_SR_PIN;
    //turn LED off, done measuring
    meas_LED_off();
    //TODO : provide real error codes
    return 2;
  }
  //save result
  dest[1]=adc16Val(rxbuf);
  //generate reset pulse
  MAG_SR_OUT&=~MAG_SR_PIN;
  //turn LED off, done measuring
  meas_LED_off();
  //get time that ADC can next be read
  adc_ready_time=ctl_get_current_time()+153;
  //TODO: provide real return codes
  return 0;
}

//take a reading from the magnetometer ADC
/*short do_conversion(void){
  long aval[2],bval[2];
  unsigned char rxbuf[4],txbuf[4];
  unsigned short addr=0x14;
  int res;
  //turn on LED while measuring
  meas_LED_on();
  //generate set pulse
  MAG_SR_OUT|=MAG_SR_PIN;
  //delay for pulse
  //__delay_cycles(16000);
  ctl_timeout_wait(ctl_get_current_time()+2);
  //configure for first conversion use channel 0
  txbuf[0]=LTC24XX_PRE|LTC24XX_EN|MAG_A_CH;
  txbuf[1]=LTC24xx_EN2|LTC24xx_SPD|LTC24xx_FA|MAG_ADC_GAIN;                   
  if((res=i2c_tx(addr,txbuf,2))<0){
    //report error
    report_error(ERR_LEV_ERROR,ACDS_ERR_SRC_SENSORS,ACDS_ERR_SEN_SETUP,res);
    //turn on error LED
    sens_err_LED_on();
    //turn LED off, done measuring
    meas_LED_off();
    //TODO : provide real error codes
    return 2;
  }
  //wait for conversion to complete
  //TODO: convert to events wait
  //__delay_cycles(1091200);
  ctl_timeout_wait(ctl_get_current_time()+78);    //wait about 75ms
  //config for next conversion use channel 1
  txbuf[0]=LTC24XX_PRE|LTC24XX_EN|MAG_B_CH;
  //read in data and start next conversion
  if((res=i2c_txrx(addr,txbuf,1,rxbuf,3))<0){
    //report error
    report_error(ERR_LEV_ERROR,ACDS_ERR_SRC_SENSORS,ACDS_ERR_SEN_CONV_READ,res);
    //turn on error LED
    sens_err_LED_on();
    //turn LED off, done measuring
    meas_LED_off();
    //TODO : provide real error codes
    return 2;
  }
  //get result
  aval[0]=adc16Val(rxbuf);
  //wait for conversion to complete
  //TODO: convert to events wait
  //__delay_cycles(1091200);
  ctl_timeout_wait(ctl_get_current_time()+78);    //wait about 75ms
  //generate reset pulse
  MAG_SR_OUT&=~MAG_SR_PIN;
  //delay for pulse
  //TODO: convert to events wait
  //__delay_cycles(16000);
  ctl_timeout_wait(ctl_get_current_time()+2);
  //configure for first conversion use channel 0
  txbuf[0]=LTC24XX_PRE|LTC24XX_EN|MAG_A_CH;
  //read in data and setup next conversion
  if((res=i2c_txrx(addr,txbuf,1,rxbuf,3))<0){
    //report error
    report_error(ERR_LEV_ERROR,ACDS_ERR_SRC_SENSORS,ACDS_ERR_SEN_CONV_READ,res);
    //turn on error LED
    sens_err_LED_on();
    //turn LED off, done measuring
    meas_LED_off();
    //TODO : provide real error codes
    return 2;
  }
  //print result
  bval[0]=adc16Val(rxbuf);                  
  //wait for conversion to complete
  //TODO: convert to events wait
  //__delay_cycles(1091200);
  ctl_timeout_wait(ctl_get_current_time()+78);    //wait about 75ms
  //config for next conversion use channel 1
  txbuf[0]=LTC24XX_PRE|LTC24XX_EN|MAG_B_CH;
  //read in data and start next conversion
  if((res=i2c_txrx(addr,txbuf,1,rxbuf,3))<0){
    //report error
    report_error(ERR_LEV_ERROR,ACDS_ERR_SRC_SENSORS,ACDS_ERR_SEN_CONV_READ,res);
    //turn on error LED
    sens_err_LED_on();
    //turn LED off, done measuring
    meas_LED_off();
    //TODO : provide real error codes
    return 2;
  }
  //print result
  aval[1]=adc16Val(rxbuf);
  //wait for conversion to complete
  //TODO: convert to events wait
  //__delay_cycles(1091200);
  ctl_timeout_wait(ctl_get_current_time()+78);    //wait about 75ms
  //read in data
  if((res=i2c_rx(addr,rxbuf,3))<0){
    //report error
    report_error(ERR_LEV_ERROR,ACDS_ERR_SRC_SENSORS,ACDS_ERR_SEN_CONV_READ,res);
    //turn on error LED
    sens_err_LED_on();
    //turn LED off, done measuring
    meas_LED_off();
    //TODO : provide real error codes
    return 2;
  }
  //print result
  bval[1]=adc16Val(rxbuf);
  magMem[0]=aval[0]-aval[1];
  magMem[1]=bval[0]-bval[1];
  //turn LED off, done measuring
  meas_LED_off();
  //TODO: provide real return codes
  return 0;
}*/

unsigned char mag_tx_addr=BUS_ADDR_ACDS;

void ACDS_sensor_interface(void *p) __toplevel{
    unsigned int e;
    unsigned char buff[BUS_I2C_HDR_LEN+sizeof(magMem)+2+BUS_I2C_CRC_LEN],*ptr;
    int i,j,res;
    //initialize event set
    ctl_events_init(&sens_ev,0);
    //stop sensors
    mag_sample_stop();
    //event loop
    for(;;){
        e=ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&sens_ev,SENS_EV_READ,CTL_TIMEOUT_NONE,0);
        if(e&SENS_EV_READ){
            //read data from magnetometers
            res=do_conversion();
            //check result
            if(res==0){
                for(i=0;i<6;i++){
                    //print out values from first axis
                    if(magFlags&(1<<(i*2))){
                        //value good, print
                        printf("%- 11i\t",magMem[i].c.a);
                    }else{
                        //value not good, print error
                        printf(" %-11s\t","Error");
                    }
                    //print out values form b-axis
                    if(magFlags&(1<<(i*2+1))){
                    //value good, print
                    printf("%- 11i\r\n",magMem[i].c.b);
                    }else{
                        //value not good, print error
                        printf(" %-11s\r\n","Error");
                    }
                }
                //TODO: set event
            }
        }
    }
}

//count and period to determine mag sampling
MAG_TIME mag_time;

//running interrupt count 
static short int_count;

//start sampling timer for magnetometer 
void mag_sample_start(unsigned short time,short count){
  //set period
  mag_time.T=time;
  //set count
  mag_time.n=count;
  //initialize interrupt count
  int_count=count;
  //set interrupt time
  TA0CCR1=readTA0()+time;
  //enable interrupt
  TA0CCTL1=CCIE;
  //turn off error LED's
  com_err_LED_off();
  sens_err_LED_off();
}

//stop sampling magnetometer
void mag_sample_stop(void){
  //disable interrupt
  TA0CCTL1=0;
  //turn off error LED's
  com_err_LED_off();
  sens_err_LED_off();
}

void sensors_single_sample(void){
    //stop sensors
    mag_sample_stop();
    //send a sensor event
    ctl_events_set_clear(&sens_ev,SENS_EV_READ,0);
}


//Timer A1 interrupt
void ACDS_timer(void) __ctl_interrupt[TIMER0_A0_VECTOR]{
  switch(TA0IV){
    //CCR1 : used for sensor data timeout
    case TA0IV_TACCR1:
      //setup next interrupt
      TA0CCR1+=mag_time.T;
      //decremint count
      int_count--;
      if(int_count<=0){
        ctl_events_set_clear(&sens_ev,SENS_EV_READ,0);
        int_count=mag_time.n;
      }
    break;
    //CCR2 : Unused
    case TA0IV_TACCR2:
    break;
    //TA0INT : unused
    case TA0IV_TAIFG:
    break;
  }
}
