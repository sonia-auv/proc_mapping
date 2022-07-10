//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// Subscriber.h
//
// Code generation for function 'Subscriber'
//

#ifndef SUBSCRIBER_H
#define SUBSCRIBER_H

// Include files
#include "proc_mapping_types.h"
#include "rtwtypes.h"
#include "coder_array.h"
#include "mlroscpp_sub.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
namespace coder {
namespace ros {
class Subscriber {
public:
  Subscriber *init();
  void callback();
  double get_MessageCount() const;
  void get_LatestMessage(::coder::array<char, 2U> &lastSubMsg_Data) const;
  char TopicName[19];
  double BufferSize;
  double MessageCount;

private:
  std::unique_ptr<MATLABSubscriber<std_msgs::String, std_msgs_StringStruct_T>>
      SubscriberHelper;
  std_msgs_StringStruct_T MsgStruct;
  bool IsInitialized;
};

class b_Subscriber {
public:
  b_Subscriber *init();
  void callback();
  double get_MessageCount() const;
  char TopicName[18];
  double BufferSize;
  double MessageCount;

private:
  std::unique_ptr<MATLABSubscriber<std_msgs::Bool, std_msgs_BoolStruct_T>>
      SubscriberHelper;
  std_msgs_BoolStruct_T MsgStruct;
  bool IsInitialized;
};

class c_Subscriber {
public:
  c_Subscriber *init();
  void callback();
  double get_MessageCount() const;
  char TopicName[26];
  double BufferSize;
  double MessageCount;

private:
  std::unique_ptr<MATLABSubscriber<std_msgs::Bool, std_msgs_BoolStruct_T>>
      SubscriberHelper;
  std_msgs_BoolStruct_T MsgStruct;
  bool IsInitialized;
};

class d_Subscriber {
public:
  d_Subscriber *init();
  void callback();
  double get_MessageCount() const;
  void get_LatestMessage(
      geometry_msgs_PoseWithCovarianceStruct_T *lastSubMsg_Pose) const;
  char TopicName[20];
  double BufferSize;
  double MessageCount;

private:
  std::unique_ptr<
      MATLABSubscriber<nav_msgs::Odometry, nav_msgs_OdometryStruct_T>>
      SubscriberHelper;
  nav_msgs_OdometryStruct_T MsgStruct;
};

class e_Subscriber {
public:
  e_Subscriber *init();
  void callback();
  double get_MessageCount() const;
  void get_LatestMessage(
      unsigned int *lastSubMsg_Height, unsigned int *lastSubMsg_Width,
      ::coder::array<sensor_msgs_PointFieldStruct_T, 1U> &lastSubMsg_Fields,
      unsigned int *lastSubMsg_PointStep,
      ::coder::array<unsigned char, 1U> &lastSubMsg_Data) const;
  char TopicName[28];
  double BufferSize;
  double MessageCount;

private:
  std::unique_ptr<MATLABSubscriber<sensor_msgs::PointCloud2,
                                   sensor_msgs_PointCloud2Struct_T>>
      SubscriberHelper;
  sensor_msgs_PointCloud2Struct_T MsgStruct;
  bool IsInitialized;
};

class f_Subscriber {
public:
  f_Subscriber *init();
  void callback();
  double get_MessageCount() const;
  char TopicName[40];
  double BufferSize;
  double MessageCount;

private:
  std::unique_ptr<MATLABSubscriber<sensor_msgs::CompressedImage,
                                   sensor_msgs_CompressedImageStruct_T>>
      SubscriberHelper;
  sensor_msgs_CompressedImageStruct_T MsgStruct;
};

} // namespace ros
} // namespace coder

#endif
// End of code generation (Subscriber.h)
