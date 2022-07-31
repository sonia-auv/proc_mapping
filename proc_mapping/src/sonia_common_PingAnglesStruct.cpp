//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: sonia_common_PingAnglesStruct.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 31-Jul-2022 13:03:34
//

// Include Files
#include "sonia_common_PingAnglesStruct.h"
#include "proc_mapping_types.h"
#include "rt_nonfinite.h"
#include "std_msgs_HeaderStruct.h"
#include <string.h>

// Function Definitions
//
// Message struct definition for sonia_common/PingAngles
//
// Arguments    : sonia_common_PingAnglesStruct_T *msg
// Return Type  : void
//
void sonia_common_PingAnglesStruct(sonia_common_PingAnglesStruct_T *msg)
{
  static const char b_cv[23]{'s', 'o', 'n', 'i', 'a', '_', 'c', 'o',
                             'm', 'm', 'o', 'n', '/', 'P', 'i', 'n',
                             'g', 'A', 'n', 'g', 'l', 'e', 's'};
  for (int i{0}; i < 23; i++) {
    msg->MessageType[i] = b_cv[i];
  }
  std_msgs_HeaderStruct(&msg->Header);
  msg->Heading = 0.0;
  msg->Elevation = 0.0;
  msg->Frequency = 0U;
  msg->Snr = 0U;
  //(msg);
}

//
// File trailer for sonia_common_PingAnglesStruct.cpp
//
// [EOF]
//
