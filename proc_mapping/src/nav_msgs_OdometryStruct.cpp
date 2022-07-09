//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// nav_msgs_OdometryStruct.cpp
//
// Code generation for function 'nav_msgs_OdometryStruct'
//

// Include files
#include "nav_msgs_OdometryStruct.h"
#include "geometry_msgs_PoseWithCovarianceStruct.h"
#include "geometry_msgs_TwistWithCovarianceStruct.h"
#include "proc_mapping_types.h"
#include "rt_nonfinite.h"
#include "std_msgs_HeaderStruct.h"
#include "coder_array.h"
#include <string.h>

// Function Definitions
void nav_msgs_OdometryStruct(nav_msgs_OdometryStruct_T *msg)
{
  static const char b_cv[17]{'n', 'a', 'v', '_', 'm', 's', 'g', 's', '/',
                             'O', 'd', 'o', 'm', 'e', 't', 'r', 'y'};
  //  Message struct definition for nav_msgs/Odometry
  std_msgs_HeaderStruct(&msg->Header);
  geometry_msgs_PoseWithCovarianceStruct(&msg->Pose);
  geometry_msgs_TwistWithCovarianceStruct(&msg->Twist);
  for (int i{0}; i < 17; i++) {
    msg->MessageType[i] = b_cv[i];
  }
  msg->ChildFrameId.set_size(1, 0);
  //(msg);
}

// End of code generation (nav_msgs_OdometryStruct.cpp)
