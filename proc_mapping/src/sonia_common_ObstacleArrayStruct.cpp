//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: sonia_common_ObstacleArrayStruct.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 31-Jul-2022 13:03:34
//

// Include Files
#include "sonia_common_ObstacleArrayStruct.h"
#include "proc_mapping_types.h"
#include "rt_nonfinite.h"
#include "sonia_common_ObstacleInfoStruct.h"
#include "std_msgs_HeaderStruct.h"
#include "coder_array.h"
#include <string.h>

// Function Definitions
//
// Message struct definition for sonia_common/ObstacleArray
//
// Arguments    : sonia_common_ObstacleArrayStruct_T *msg
// Return Type  : void
//
void sonia_common_ObstacleArrayStruct(sonia_common_ObstacleArrayStruct_T *msg)
{
  static const char b_cv[26]{'s', 'o', 'n', 'i', 'a', '_', 'c', 'o', 'm',
                             'm', 'o', 'n', '/', 'O', 'b', 's', 't', 'a',
                             'c', 'l', 'e', 'A', 'r', 'r', 'a', 'y'};
  sonia_common_ObstacleInfoStruct_T t2_Obstacles;
  std_msgs_HeaderStruct(&msg->Header);
  sonia_common_ObstacleInfoStruct(&t2_Obstacles);
  for (int i{0}; i < 26; i++) {
    msg->MessageType[i] = b_cv[i];
  }
  msg->Obstacles.set_size(1);
  msg->Obstacles[0] = t2_Obstacles;
  msg->Obstacles.set_size(0);
  //(msg);
}

//
// File trailer for sonia_common_ObstacleArrayStruct.cpp
//
// [EOF]
//
