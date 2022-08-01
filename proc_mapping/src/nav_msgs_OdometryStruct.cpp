//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: nav_msgs_OdometryStruct.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 01-Aug-2022 08:26:09
//

// Include Files
#include "nav_msgs_OdometryStruct.h"
#include "geometry_msgs_PoseWithCovarianceStruct.h"
#include "geometry_msgs_TwistWithCovarianceStruct.h"
#include "proc_mapping_types.h"
#include "rt_nonfinite.h"
#include "std_msgs_HeaderStruct.h"
#include "coder_array.h"
#include <string.h>

// Function Definitions
//
// Message struct definition for nav_msgs/Odometry
//
// Arguments    : nav_msgs_OdometryStruct_T *msg
// Return Type  : void
//
void nav_msgs_OdometryStruct(nav_msgs_OdometryStruct_T *msg)
{
  static const char b_cv[17]{'n', 'a', 'v', '_', 'm', 's', 'g', 's', '/',
                             'O', 'd', 'o', 'm', 'e', 't', 'r', 'y'};
  std_msgs_HeaderStruct(&msg->Header);
  geometry_msgs_PoseWithCovarianceStruct(&msg->Pose);
  geometry_msgs_TwistWithCovarianceStruct(&msg->Twist);
  for (int i{0}; i < 17; i++) {
    msg->MessageType[i] = b_cv[i];
  }
  msg->ChildFrameId.set_size(1, 0);
  //(msg);
}

//
// File trailer for nav_msgs_OdometryStruct.cpp
//
// [EOF]
//
