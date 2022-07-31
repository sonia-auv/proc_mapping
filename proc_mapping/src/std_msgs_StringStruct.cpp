//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: std_msgs_StringStruct.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 31-Jul-2022 13:03:34
//

// Include Files
#include "std_msgs_StringStruct.h"
#include "proc_mapping_types.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <string.h>

// Function Definitions
//
// Message struct definition for std_msgs/String
//
// Arguments    : std_msgs_StringStruct_T *msg
// Return Type  : void
//
void std_msgs_StringStruct(std_msgs_StringStruct_T *msg)
{
  static const char b_cv[15]{'s', 't', 'd', '_', 'm', 's', 'g', 's',
                             '/', 'S', 't', 'r', 'i', 'n', 'g'};
  for (int i{0}; i < 15; i++) {
    msg->MessageType[i] = b_cv[i];
  }
  msg->Data.set_size(1, 0);
  //(msg);
}

//
// File trailer for std_msgs_StringStruct.cpp
//
// [EOF]
//
