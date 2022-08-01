//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: geometry_msgs_QuaternionStruct.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 01-Aug-2022 08:26:09
//

// Include Files
#include "geometry_msgs_QuaternionStruct.h"
#include "proc_mapping_types.h"
#include "rt_nonfinite.h"
#include <string.h>

// Function Definitions
//
// Message struct definition for geometry_msgs/Quaternion
//
// Arguments    : void
// Return Type  : geometry_msgs_QuaternionStruct_T
//
geometry_msgs_QuaternionStruct_T geometry_msgs_QuaternionStruct()
{
  static const geometry_msgs_QuaternionStruct_T b_msg{
      {'g', 'e', 'o', 'm', 'e', 't', 'r', 'y', '_', 'm', 's', 'g', 's',
       '/', 'Q', 'u', 'a', 't', 'e', 'r', 'n', 'i', 'o', 'n'}, // MessageType
      0.0,                                                     // X
      0.0,                                                     // Y
      0.0,                                                     // Z
      0.0                                                      // W
  };
  geometry_msgs_QuaternionStruct_T msg;
  msg = b_msg;
  //(&b_msg);
  return msg;
}

//
// File trailer for geometry_msgs_QuaternionStruct.cpp
//
// [EOF]
//
