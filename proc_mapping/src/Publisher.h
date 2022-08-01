//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Publisher.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 01-Aug-2022 08:26:09
//

#ifndef PUBLISHER_H
#define PUBLISHER_H

// Include Files
#include "rtwtypes.h"
#include "mlroscpp_pub.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
namespace coder {
namespace ros {
class Publisher {
public:
  Publisher *init();
  char TopicName[20];
  double BufferSize;
  bool IsLatching;

private:
  std::unique_ptr<MATLABPublisher<sensor_msgs::PointCloud2,
                                  sensor_msgs_PointCloud2Struct_T>>
      PublisherHelper;
};

class b_Publisher {
public:
  b_Publisher *init();
  char TopicName[28];
  double BufferSize;
  bool IsLatching;
  std::unique_ptr<MATLABPublisher<sonia_common::ObstacleArray,
                                  sonia_common_ObstacleArrayStruct_T>>
      PublisherHelper;
};

} // namespace ros
} // namespace coder

#endif
//
// File trailer for Publisher.h
//
// [EOF]
//
