#ifndef __ACDS_ERROR_H
#define __ACDS_ERROR_H
#include <commandLib.h>
#include <Error.h>

enum{ACDS_ERR_SRC_TORQUERS=ERR_SRC_CMD+1,ACDS_ERR_SRC_ALGORITHM,ACDS_ERR_SRC_SENSORS};

//sensor errors
enum{ACDS_ERR_SEN_BAD_PACKET_LENGTH=0};
    
//algorithm errors
enum{ACDS_ERR_ALG_LOG_FAIL=0};


#endif
  