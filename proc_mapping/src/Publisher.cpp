//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// Publisher.cpp
//
// Code generation for function 'Publisher'
//

// Include files
#include "Publisher.h"
#include "proc_mapping_types.h"
#include "rt_nonfinite.h"
#include "sensor_msgs_PointCloud2Struct.h"
#include "sonia_common_AddPoseStruct.h"
#include "mlroscpp_pub.h"
#include <string.h>

// Function Definitions
namespace coder {
namespace ros {
Publisher *Publisher::init()
{
  static const char topic[20]{'/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p', 'p',
                              'i', 'n', 'g', '/', 'o', 'u', 't', 'p', 'u', 't'};
  Publisher *obj;
  sensor_msgs_PointCloud2Struct_T r;
  obj = this;
  for (int i{0}; i < 20; i++) {
    obj->TopicName[i] = topic[i];
  }
  obj->BufferSize = 1.0;
  obj->IsLatching = true;
  sensor_msgs_PointCloud2Struct(&r);
  obj->PublisherHelper =
      std::unique_ptr<MATLABPublisher<sensor_msgs::PointCloud2,
                                      sensor_msgs_PointCloud2Struct_T>>(
          new MATLABPublisher<sensor_msgs::PointCloud2,
                              sensor_msgs_PointCloud2Struct_T>()); //();
  MATLABPUBLISHER_createPublisher(obj->PublisherHelper, &obj->TopicName[0],
                                  20.0, obj->BufferSize, obj->IsLatching);
  return obj;
}

b_Publisher *b_Publisher::init()
{
  static const char topic[25]{'/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p',
                              'p', 'i', 'n', 'g', '/', 'o', 'u', 't', 'p',
                              'u', 't', '_', 'p', 'o', 's', 'e'};
  b_Publisher *obj;
  sonia_common_AddPoseStruct_T unusedExpr;
  obj = this;
  for (int i{0}; i < 25; i++) {
    obj->TopicName[i] = topic[i];
  }
  obj->BufferSize = 1.0;
  obj->IsLatching = true;
  sonia_common_AddPoseStruct(&unusedExpr);
  obj->PublisherHelper = std::unique_ptr<
      MATLABPublisher<sonia_common::AddPose, sonia_common_AddPoseStruct_T>>(
      new MATLABPublisher<sonia_common::AddPose,
                          sonia_common_AddPoseStruct_T>()); //();
  MATLABPUBLISHER_createPublisher(obj->PublisherHelper, &obj->TopicName[0],
                                  25.0, obj->BufferSize, obj->IsLatching);
  return obj;
}

} // namespace ros
} // namespace coder

// End of code generation (Publisher.cpp)
