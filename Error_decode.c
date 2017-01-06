#include <Error.h>
#include <commandLib.h>
#include <SDlib.h>
#include <ARCbus.h>
#include <i2c.h>
#include "ACDSerr.h"
#include "torquers.h"

static char axis(unsigned short ax){
  switch(ax){
    case 0:
      return 'X';
    case 1:
      return 'Y';
    case 2:
      return 'Z';
    default:
      return '?';
  }
}

//decode errors from ACDS system
const char *ACDS_err_decode(char buf[150], unsigned short source,int err, unsigned short argument){
  switch(source){
    case ERR_SRC_CMD:
      switch(err){
      case CMD_ERR_RESET:
        return "Command Line : Commanded reset";
      }
    break;
    case ACDS_ERR_SRC_TORQUERS:
      switch(err){
        TQ_ERR_BAD_SET:
          sprintf(buf,"Torquers : Bad Set %i",argument);
        return buf;
        case TQ_ERR_BAD_TORQUER:
          sprintf(buf,"Torquers : Bad Torquer %i",argument);
        return buf;
        case TQ_ERR_BAD_DIR:
          sprintf(buf,"Torquers : Bad Direction %i",argument);
        return buf;
        case TQ_ERR_COMP:
          sprintf(buf,"Torquers : comparitor error for %c-axis",axis(argument));
        return buf;
        case TQ_ERR_CAP:
          sprintf(buf,"Torquers : capacitor error for %c-axis",axis(argument));
        return buf;
        case TQ_ERR_BAD_CONNECTION:
          sprintf(buf,"Torquers : Bad connection for torquer #%i on %c-axis",(argument>>4),axis(argument&0x0F));
        return buf;
        case TQ_INFO_FLIP:
          sprintf(buf,"Torquers : Flipping torquer #%i on %c-axis",(argument>>4),axis(argument&0x0F));
        return buf;
        case TQ_ERROR_INVALID_STATUS:
          sprintf(buf,"Torquers : Invalid Status %u",argument);
        return buf;
        case TQ_INFO_TQFB:
          sprintf(buf,"Torquers : fb1 = 0x%02X  fb2 = 0x%02X",(argument>>8),(argument&0xFF));
        return buf;
        default:
          sprintf(buf,"Torquers : Unknown Error #%i, argument = %i",err,argument);
        return buf;
      }
    break;
    case ACDS_ERR_SRC_ALGORITHM:
        switch(err){
            case ACDS_ERR_ALG_LOG_FAIL:
                sprintf(buf,"Algorithm : Failed to write log : %s (%i)",SD_error_str(argument),argument);
            return buf;
        }
    break;
    case ACDS_ERR_SRC_SENSORS:
      switch(err){
        case ACDS_ERR_SEN_SETUP:
          sprintf(buf,"Sensor I2C : Setup %s",I2C_error_str(argument));
        return buf;
        case ACDS_ERR_SEN_CONV_READ:
          sprintf(buf,"Sensor I2C : Conversion Result %s",I2C_error_str(argument));
        return buf;
        case ACDS_ERR_SEN_BAD_GAIN:
          sprintf(buf,"Sensor I2C : Invalid Gain %u",argument);
        return buf;
        case ACDS_ERR_SEN_INSUFFICIENT_DATA:
          sprintf(buf,"Sensor I2C : Insufficient Data, Flags = 0x%04X",argument);
        return buf;
        default:
          sprintf(buf,"Sensors : Unknown Error #%i, argument = %i",err,argument);
        return buf;
      }
    break;
    case ACDS_ERR_SRC_SUBSYSTEM:
        switch(err){
            case ACDS_ERR_SUB_LEDL_START:
                sprintf(buf,"Sybsystem : Error starting sensor reading : %s (%i)",BUS_error_str(argument),argument);
            break;
            case ACDS_ERR_SUB_LEDL_STOP:
                sprintf(buf,"Sybsystem : Error starting sensor reading : %s (%i)",BUS_error_str(argument),argument);
            break;
            case ACDS_ERR_SUB_STAT_TX:
                sprintf(buf,"Sybsystem : Error Sending Status: %s (%i)",BUS_error_str(argument),argument);
            default:
              sprintf(buf,"Subsystem : Unknown Error #%i, argument = %i",err,argument);
            return buf;
        }
    case ACDS_ERR_SRC_I2C_CMD:
        switch(err){
            case ACDS_ERR_I2C_FLASH_WRITE:
                return "I2C command : flash write failed";
            case ACDS_ERR_I2C_BUFF_BUSY:
                return "I2C command : buffer busy";
            case ACDS_ERR_I2C_SPI_DAT:
              sprintf(buf,"I2C command : Error sending data block : %s (%i)",BUS_error_str(argument),argument);
            return buf;
            case ACDS_ERR_I2C_READ_DAT:
              sprintf(buf,"I2C command : Error reading data block : %s (%i)",SD_error_str(argument),argument);
            return buf;
            case ACDS_ERR_I2C_BUFFER_BUSY:
              return "I2C command : Error Buffer Busy";
            default:
              sprintf(buf,"I2C command : Unknown Error #%i, argument = %i",err,argument);
            return buf;
        }
  }
  sprintf(buf,"source = %i, error = %i, argument = %i",source,err,argument);
  return buf;
}
