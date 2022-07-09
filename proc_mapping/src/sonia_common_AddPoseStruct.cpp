//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// sonia_common_AddPoseStruct.cpp
//
// Code generation for function 'sonia_common_AddPoseStruct'
//

// Include files
#include "sonia_common_AddPoseStruct.h"
#include "geometry_msgs_PointStruct.h"
#include "geometry_msgs_Vector3Struct.h"
#include "proc_mapping_types.h"
#include "rt_nonfinite.h"
#include <string.h>

// Function Definitions
void sonia_common_AddPoseStruct(sonia_common_AddPoseStruct_T *msg)
{
  static const char b_cv[20]{'s', 'o', 'n', 'i', 'a', '_', 'c', 'o', 'm', 'm',
                             'o', 'n', '/', 'A', 'd', 'd', 'P', 'o', 's', 'e'};
  //  Message struct definition for sonia_common/AddPose
  for (int i{0}; i < 20; i++) {
    msg->MessageType[i] = b_cv[i];
  }
  msg->Position = geometry_msgs_PointStruct();
  msg->Orientation = geometry_msgs_Vector3Struct();
  msg->Frame = 0U;
  msg->Speed = 0U;
  msg->Fine = 0.0;
  msg->Rotation = false;
  //(msg);
}

// End of code generation (sonia_common_AddPoseStruct.cpp)
