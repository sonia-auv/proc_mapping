//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// geometry_msgs_TwistStruct.cpp
//
// Code generation for function 'geometry_msgs_TwistStruct'
//

// Include files
#include "geometry_msgs_TwistStruct.h"
#include "geometry_msgs_Vector3Struct.h"
#include "proc_mapping_types.h"
#include "rt_nonfinite.h"
#include <string.h>

// Function Definitions
void geometry_msgs_TwistStruct(geometry_msgs_TwistStruct_T *msg)
{
  static const char cv[19]{'g', 'e', 'o', 'm', 'e', 't', 'r', 'y', '_', 'm',
                           's', 'g', 's', '/', 'T', 'w', 'i', 's', 't'};
  //  Message struct definition for geometry_msgs/Twist
  for (int i{0}; i < 19; i++) {
    msg->MessageType[i] = cv[i];
  }
  msg->Linear = geometry_msgs_Vector3Struct();
  msg->Angular = geometry_msgs_Vector3Struct();
  //(msg);
}

// End of code generation (geometry_msgs_TwistStruct.cpp)
