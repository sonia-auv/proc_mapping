//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: std_msgs_BoolStruct.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 01-Aug-2022 08:26:09
//

// Include Files
#include "std_msgs_BoolStruct.h"
#include "proc_mapping_types.h"
#include "rt_nonfinite.h"
#include <string.h>

// Function Definitions
//
// Message struct definition for std_msgs/Bool
//
// Arguments    : void
// Return Type  : std_msgs_BoolStruct_T
//
std_msgs_BoolStruct_T std_msgs_BoolStruct()
{
  static const std_msgs_BoolStruct_T b_msg{
      {'s', 't', 'd', '_', 'm', 's', 'g', 's', '/', 'B', 'o', 'o',
       'l'}, // MessageType
      false  // Data
  };
  std_msgs_BoolStruct_T msg;
  msg = b_msg;
  //(&b_msg);
  return msg;
}

//
// File trailer for std_msgs_BoolStruct.cpp
//
// [EOF]
//
