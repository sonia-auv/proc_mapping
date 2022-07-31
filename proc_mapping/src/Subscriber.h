//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Subscriber.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 31-Jul-2022 13:03:34
//

#ifndef SUBSCRIBER_H
#define SUBSCRIBER_H

// Include Files
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
  char TopicName[25];
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
  b_Subscriber *b_init();
  void b_callback();
  char TopicName[24];
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
  c_Subscriber *b_init();
  void b_callback();
  char TopicName[32];
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
  unsigned short get_LatestMessage() const;
  char TopicName[25];
  double BufferSize;
  double MessageCount;

private:
  std::unique_ptr<MATLABSubscriber<std_msgs::UInt16, std_msgs_UInt16Struct_T>>
      SubscriberHelper;
  std_msgs_UInt16Struct_T MsgStruct;
  bool IsInitialized;
};

class g_Subscriber {
public:
  g_Subscriber *init();
  void callback();
  double get_MessageCount() const;
  void get_LatestMessage(double *lastSubMsg_Heading,
                         double *lastSubMsg_Elevation,
                         unsigned short *lastSubMsg_Frequency,
                         unsigned short *lastSubMsg_Snr) const;
  char TopicName[31];
  double BufferSize;
  double MessageCount;

private:
  std::unique_ptr<MATLABSubscriber<sonia_common::PingAngles,
                                   sonia_common_PingAnglesStruct_T>>
      SubscriberHelper;
  sonia_common_PingAnglesStruct_T MsgStruct;
  bool IsInitialized;
};

} // namespace ros
} // namespace coder

#endif
//
// File trailer for Subscriber.h
//
// [EOF]
//
