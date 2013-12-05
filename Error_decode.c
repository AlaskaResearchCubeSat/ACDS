#include <Error.h>
#include <commandLib.h>
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
char *err_decode(char buf[150], unsigned short source,int err, unsigned short argument){
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
        default:
          sprintf(buf,"Torquers : Unknown Error #%i, argument = %i",err,argument);
        return buf;
      }
    break;
    case ACDS_ERR_SRC_ALGORITHM:
    break;
    case ACDS_ERR_SRC_SENSORS:
      switch(err){
        case ACDS_ERR_SEN_BAD_PACKET_LENGTH:
          sprintf(buf,"Sensors : Bad Packet Length, len = %i",argument);
        default:
          sprintf(buf,"Sensors : Unknown Error #%i, argument = %i",err,argument);
        return buf;
      }
    break;
  }
  sprintf(buf,"source = %i, error = %i, argument = %i",source,err,argument);
  return buf;
}
