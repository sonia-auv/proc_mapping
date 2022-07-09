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
  char TopicName[24];
  double BufferSize;
  double MessageCount;

private:
  std::unique_ptr<MATLABSubscriber<std_msgs::Bool, std_msgs_BoolStruct_T>>
      SubscriberHelper;
  std_msgs_BoolStruct_T MsgStruct;
  bool IsInitialized;
};

class b_Subscriber {
public:
  b_Subscriber *init();
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

class c_Subscriber {
public:
  c_Subscriber *init();
  void callback();
  double get_MessageCount() const;
  void get_LatestMessage(double *lastSubMsg_Pose_Pose_Position_X,
                         double *lastSubMsg_Pose_Pose_Position_Y,
                         double *lastSubMsg_Pose_Pose_Position_Z,
                         double *lastSubMsg_Pose_Pose_Orientation_X,
                         double *lastSubMsg_Pose_Pose_Orientation_Y,
                         double *lastSubMsg_Pose_Pose_Orientation_Z,
                         double *lastSubMsg_Pose_Pose_Orientation_W) const;
  char TopicName[20];
  double BufferSize;
  double MessageCount;

private:
  std::unique_ptr<
      MATLABSubscriber<nav_msgs::Odometry, nav_msgs_OdometryStruct_T>>
      SubscriberHelper;
  nav_msgs_OdometryStruct_T MsgStruct;
};

class d_Subscriber {
public:
  d_Subscriber *init();
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

class e_Subscriber {
public:
  e_Subscriber *init();
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
