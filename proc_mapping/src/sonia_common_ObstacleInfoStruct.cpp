//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: sonia_common_ObstacleInfoStruct.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 31-Jul-2022 13:03:34
//

// Include Files
#include "sonia_common_ObstacleInfoStruct.h"
#include "geometry_msgs_PoseStruct.h"
#include "proc_mapping_types.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <string.h>

// Function Definitions
//
// Message struct definition for sonia_common/ObstacleInfo
//
// Arguments    : sonia_common_ObstacleInfoStruct_T *msg
// Return Type  : void
//
void sonia_common_ObstacleInfoStruct(sonia_common_ObstacleInfoStruct_T *msg)
{
  static const char b_cv[25]{'s', 'o', 'n', 'i', 'a', '_', 'c', 'o', 'm',
                             'm', 'o', 'n', '/', 'O', 'b', 's', 't', 'a',
                             'c', 'l', 'e', 'I', 'n', 'f', 'o'};
  geometry_msgs_PoseStruct(&msg->Pose);
  for (int i{0}; i < 25; i++) {
    msg->MessageType[i] = b_cv[i];
  }
  msg->Name.set_size(1, 0);
  msg->IsValid = false;
  msg->Confidence = 0.0F;
  //(msg);
}

//
// File trailer for sonia_common_ObstacleInfoStruct.cpp
//
// [EOF]
//
