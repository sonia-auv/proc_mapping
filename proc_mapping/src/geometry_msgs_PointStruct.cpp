//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: geometry_msgs_PointStruct.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 01-Aug-2022 08:26:09
//

// Include Files
#include "geometry_msgs_PointStruct.h"
#include "proc_mapping_types.h"
#include "rt_nonfinite.h"
#include <string.h>

// Function Definitions
//
// Message struct definition for geometry_msgs/Point
//
// Arguments    : void
// Return Type  : geometry_msgs_PointStruct_T
//
geometry_msgs_PointStruct_T geometry_msgs_PointStruct()
{
  static const geometry_msgs_PointStruct_T b_msg{
      {'g', 'e', 'o', 'm', 'e', 't', 'r', 'y', '_', 'm', 's', 'g', 's', '/',
       'P', 'o', 'i', 'n', 't'}, // MessageType
      0.0,                       // X
      0.0,                       // Y
      0.0                        // Z
  };
  geometry_msgs_PointStruct_T msg;
  msg = b_msg;
  //(&b_msg);
  return msg;
}

//
// File trailer for geometry_msgs_PointStruct.cpp
//
// [EOF]
//
