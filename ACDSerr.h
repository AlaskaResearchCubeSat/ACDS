#ifndef __ACDS_ERROR_H
#define __ACDS_ERROR_H
#include <commandLib.h>
#include <Error.h>

enum{ACDS_ERR_SRC_TORQUERS=ERR_SRC_CMD+1,ACDS_ERR_SRC_ALGORITHM,ACDS_ERR_SRC_SENSORS,ACDS_ERR_SRC_SUBSYSTEM,
     ACDS_ERR_END};

#define ACDS_ERROR_MIN  ERR_SRC_CMD
#define ACDS_ERROR_MAX  (ACDS_ERR_END-1)

//sensor errors
enum{ACDS_ERR_SEN_BAD_PACKET_LENGTH=0};
    
//algorithm errors
enum{ACDS_ERR_ALG_LOG_FAIL=0};
    
//subsystem errors
enum{ACDS_ERR_SUB_LEDL_START=0,ACDS_ERR_SUB_LEDL_STOP,ACDS_ERR_SUB_STAT_TX};

const char *ACDS_err_decode(char buf[150], unsigned short source,int err, unsigned short argument);


#endif
  