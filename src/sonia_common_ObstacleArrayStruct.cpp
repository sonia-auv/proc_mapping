//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// sonia_common_ObstacleArrayStruct.cpp
//
// Code generation for function 'sonia_common_ObstacleArrayStruct'
//

// Include files
#include "sonia_common_ObstacleArrayStruct.h"
#include "proc_mapping_types.h"
#include "rt_nonfinite.h"
#include "sonia_common_ObstacleInfoStruct.h"
#include "std_msgs_HeaderStruct.h"
#include "coder_array.h"
#include <string.h>

// Function Definitions
void sonia_common_ObstacleArrayStruct(sonia_common_ObstacleArrayStruct_T *msg)
{
  static const char cv[26]{'s', 'o', 'n', 'i', 'a', '_', 'c', 'o', 'm',
                           'm', 'o', 'n', '/', 'O', 'b', 's', 't', 'a',
                           'c', 'l', 'e', 'A', 'r', 'r', 'a', 'y'};
  sonia_common_ObstacleInfoStruct_T t5_Obstacles;
  //  Message struct definition for sonia_common/ObstacleArray
  std_msgs_HeaderStruct(&msg->Header);
  sonia_common_ObstacleInfoStruct(&t5_Obstacles);
  for (int i{0}; i < 26; i++) {
    msg->MessageType[i] = cv[i];
  }
  msg->Obstacles.set_size(1);
  msg->Obstacles[0] = t5_Obstacles;
  msg->Obstacles.set_size(0);
  //(msg);
}

// End of code generation (sonia_common_ObstacleArrayStruct.cpp)
