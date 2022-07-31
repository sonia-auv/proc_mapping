//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: geometry_msgs_TwistStruct.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 31-Jul-2022 13:03:34
//

// Include Files
#include "geometry_msgs_TwistStruct.h"
#include "geometry_msgs_Vector3Struct.h"
#include "proc_mapping_types.h"
#include "rt_nonfinite.h"
#include <string.h>

// Function Definitions
//
// Message struct definition for geometry_msgs/Twist
//
// Arguments    : geometry_msgs_TwistStruct_T *msg
// Return Type  : void
//
void geometry_msgs_TwistStruct(geometry_msgs_TwistStruct_T *msg)
{
  static const char b_cv[19]{'g', 'e', 'o', 'm', 'e', 't', 'r', 'y', '_', 'm',
                             's', 'g', 's', '/', 'T', 'w', 'i', 's', 't'};
  for (int i{0}; i < 19; i++) {
    msg->MessageType[i] = b_cv[i];
  }
  msg->Linear = geometry_msgs_Vector3Struct();
  msg->Angular = geometry_msgs_Vector3Struct();
  //(msg);
}

//
// File trailer for geometry_msgs_TwistStruct.cpp
//
// [EOF]
//
