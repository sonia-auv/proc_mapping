//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: sensor_msgs_PointFieldStruct.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 01-Aug-2022 08:26:09
//

// Include Files
#include "sensor_msgs_PointFieldStruct.h"
#include "proc_mapping_types.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <string.h>

// Function Definitions
//
// Message struct definition for sensor_msgs/PointField
//
// Arguments    : sensor_msgs_PointFieldStruct_T *msg
// Return Type  : void
//
void sensor_msgs_PointFieldStruct(sensor_msgs_PointFieldStruct_T *msg)
{
  static const char b_cv[22]{'s', 'e', 'n', 's', 'o', 'r', '_', 'm',
                             's', 'g', 's', '/', 'P', 'o', 'i', 'n',
                             't', 'F', 'i', 'e', 'l', 'd'};
  for (int i{0}; i < 22; i++) {
    msg->MessageType[i] = b_cv[i];
  }
  msg->INT8 = 1U;
  msg->UINT8 = 2U;
  msg->INT16 = 3U;
  msg->UINT16 = 4U;
  msg->INT32 = 5U;
  msg->UINT32 = 6U;
  msg->FLOAT32 = 7U;
  msg->FLOAT64 = 8U;
  msg->Name.set_size(1, 0);
  msg->Offset = 0U;
  msg->Datatype = 0U;
  msg->Count = 0U;
  //(msg);
}

//
// File trailer for sensor_msgs_PointFieldStruct.cpp
//
// [EOF]
//
