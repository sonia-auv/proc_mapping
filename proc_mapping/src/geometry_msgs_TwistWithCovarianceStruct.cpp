//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: geometry_msgs_TwistWithCovarianceStruct.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 01-Aug-2022 08:26:09
//

// Include Files
#include "geometry_msgs_TwistWithCovarianceStruct.h"
#include "geometry_msgs_TwistStruct.h"
#include "proc_mapping_types.h"
#include "rt_nonfinite.h"
#include <cstring>
#include <string.h>

// Function Definitions
//
// Message struct definition for geometry_msgs/TwistWithCovariance
//
// Arguments    : geometry_msgs_TwistWithCovarianceStruct_T *msg
// Return Type  : void
//
void geometry_msgs_TwistWithCovarianceStruct(
    geometry_msgs_TwistWithCovarianceStruct_T *msg)
{
  static const char b_cv[33]{'g', 'e', 'o', 'm', 'e', 't', 'r', 'y', '_',
                             'm', 's', 'g', 's', '/', 'T', 'w', 'i', 's',
                             't', 'W', 'i', 't', 'h', 'C', 'o', 'v', 'a',
                             'r', 'i', 'a', 'n', 'c', 'e'};
  for (int i{0}; i < 33; i++) {
    msg->MessageType[i] = b_cv[i];
  }
  geometry_msgs_TwistStruct(&msg->Twist);
  std::memset(&msg->Covariance[0], 0, 36U * sizeof(double));
  //(msg);
}

//
// File trailer for geometry_msgs_TwistWithCovarianceStruct.cpp
//
// [EOF]
//
