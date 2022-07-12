//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// sensor_msgs_CompressedImageStruct.cpp
//
// Code generation for function 'sensor_msgs_CompressedImageStruct'
//

// Include files
#include "sensor_msgs_CompressedImageStruct.h"
#include "proc_mapping_types.h"
#include "rt_nonfinite.h"
#include "std_msgs_HeaderStruct.h"
#include "coder_array.h"
#include <string.h>

// Function Definitions
void sensor_msgs_CompressedImageStruct(sensor_msgs_CompressedImageStruct_T *msg)
{
  static const char cv[27]{'s', 'e', 'n', 's', 'o', 'r', '_', 'm', 's',
                           'g', 's', '/', 'C', 'o', 'm', 'p', 'r', 'e',
                           's', 's', 'e', 'd', 'I', 'm', 'a', 'g', 'e'};
  //  Message struct definition for sensor_msgs/CompressedImage
  std_msgs_HeaderStruct(&msg->Header);
  for (int i{0}; i < 27; i++) {
    msg->MessageType[i] = cv[i];
  }
  msg->Format.set_size(1, 0);
  msg->Data.set_size(0);
  //(msg);
}

// End of code generation (sensor_msgs_CompressedImageStruct.cpp)
