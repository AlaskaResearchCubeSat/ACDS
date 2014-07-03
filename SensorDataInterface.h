#ifndef __SENSOR_DATA_INTERFACE_H
#define __SENSOR_DATA_INTERFACE_H


  //command for magnetomitor data
  //TODO: set this better
  enum{CMD_MAG_DATA=20,CMD_MAG_SAMPLE_CONFIG};
  
  enum{MAG_SAMPLE_START,MAG_SAMPLE_STOP,MAG_SINGLE_SAMPLE};
      
  enum{MAG_X_PLUS_IDX=0,MAG_X_MINUS_IDX=1,MAG_Y_PLUS_IDX=2,MAG_Y_MINUS_IDX=3,MAG_Z_PLUS_IDX=4,MAG_Z_MINUS_IDX=5};
    
#endif
      