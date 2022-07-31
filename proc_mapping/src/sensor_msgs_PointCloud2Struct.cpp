//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: sensor_msgs_PointCloud2Struct.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 31-Jul-2022 13:03:34
//

// Include Files
#include "sensor_msgs_PointCloud2Struct.h"
#include "proc_mapping_types.h"
#include "rt_nonfinite.h"
#include "sensor_msgs_PointFieldStruct.h"
#include "std_msgs_HeaderStruct.h"
#include "coder_array.h"
#include <string.h>

// Function Definitions
//
// Message struct definition for sensor_msgs/PointCloud2
//
// Arguments    : sensor_msgs_PointCloud2Struct_T *msg
// Return Type  : void
//
void sensor_msgs_PointCloud2Struct(sensor_msgs_PointCloud2Struct_T *msg)
{
  static const char b_cv[23]{'s', 'e', 'n', 's', 'o', 'r', '_', 'm',
                             's', 'g', 's', '/', 'P', 'o', 'i', 'n',
                             't', 'C', 'l', 'o', 'u', 'd', '2'};
  sensor_msgs_PointFieldStruct_T t3_Fields;
  std_msgs_HeaderStruct(&msg->Header);
  sensor_msgs_PointFieldStruct(&t3_Fields);
  for (int i{0}; i < 23; i++) {
    msg->MessageType[i] = b_cv[i];
  }
  msg->Height = 0U;
  msg->Width = 0U;
  msg->Fields.set_size(1);
  msg->Fields[0] = t3_Fields;
  msg->IsBigendian = false;
  msg->PointStep = 0U;
  msg->RowStep = 0U;
  msg->Data.set_size(0);
  msg->IsDense = false;
  msg->Fields.set_size(0);
  //(msg);
}

//
// File trailer for sensor_msgs_PointCloud2Struct.cpp
//
// [EOF]
//
