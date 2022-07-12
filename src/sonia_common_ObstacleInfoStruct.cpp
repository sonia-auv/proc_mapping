//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// sonia_common_ObstacleInfoStruct.cpp
//
// Code generation for function 'sonia_common_ObstacleInfoStruct'
//

// Include files
#include "sonia_common_ObstacleInfoStruct.h"
#include "geometry_msgs_PoseStruct.h"
#include "proc_mapping_types.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <string.h>

// Function Definitions
void sonia_common_ObstacleInfoStruct(sonia_common_ObstacleInfoStruct_T *msg)
{
  static const char cv[25]{'s', 'o', 'n', 'i', 'a', '_', 'c', 'o', 'm',
                           'm', 'o', 'n', '/', 'O', 'b', 's', 't', 'a',
                           'c', 'l', 'e', 'I', 'n', 'f', 'o'};
  //  Message struct definition for sonia_common/ObstacleInfo
  geometry_msgs_PoseStruct(&msg->Pose);
  for (int i{0}; i < 25; i++) {
    msg->MessageType[i] = cv[i];
  }
  msg->Name.set_size(1, 0);
  msg->IsValid = false;
  msg->Confidence = 0.0F;
  //(msg);
}

// End of code generation (sonia_common_ObstacleInfoStruct.cpp)
