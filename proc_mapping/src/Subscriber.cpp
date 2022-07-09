//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// Subscriber.cpp
//
// Code generation for function 'Subscriber'
//

// Include files
#include "Subscriber.h"
#include "nav_msgs_OdometryStruct.h"
#include "proc_mapping_data.h"
#include "proc_mapping_types.h"
#include "rt_nonfinite.h"
#include "sensor_msgs_CompressedImageStruct.h"
#include "sensor_msgs_PointCloud2Struct.h"
#include "std_msgs_BoolStruct.h"
#include "coder_array.h"
#include "mlroscpp_sub.h"
#include <stdio.h>
#include <string.h>

// Function Definitions
namespace coder {
namespace ros {
void Subscriber::callback()
{
  MessageCount = get_MessageCount() + 1.0;
  if (IsInitialized) {
    bundleStarted = MsgStruct.Data;
    //  Initial variables
    //  SET
    //  Initial variables
    //  GET
    if (bundleStarted) {
      printf("INFO : proc mapping : Bundle record started \n");
      fflush(stdout);
    } else {
      printf("INFO : proc mapping : Bundle record stopped \n");
      fflush(stdout);
    }
  }
}

void e_Subscriber::callback()
{
  MessageCount = get_MessageCount() + 1.0;
}

void b_Subscriber::callback()
{
  MessageCount = get_MessageCount() + 1.0;
  if (IsInitialized) {
    //  Initial variables
    //  SET
    newClearBundleMsg = true;
  }
}

void d_Subscriber::callback()
{
  MessageCount = get_MessageCount() + 1.0;
  if (IsInitialized) {
    //  Initial variables
    //  SET
    newSonarMsg = true;
  }
}

void c_Subscriber::callback()
{
  MessageCount = get_MessageCount() + 1.0;
}

double c_Subscriber::get_MessageCount() const
{
  return MessageCount;
}

double e_Subscriber::get_MessageCount() const
{
  return MessageCount;
}

double d_Subscriber::get_MessageCount() const
{
  return MessageCount;
}

double b_Subscriber::get_MessageCount() const
{
  return MessageCount;
}

double Subscriber::get_MessageCount() const
{
  return MessageCount;
}

void d_Subscriber::get_LatestMessage(
    unsigned int *lastSubMsg_Height, unsigned int *lastSubMsg_Width,
    ::coder::array<sensor_msgs_PointFieldStruct_T, 1U> &lastSubMsg_Fields,
    unsigned int *lastSubMsg_PointStep,
    ::coder::array<unsigned char, 1U> &lastSubMsg_Data) const
{
  int loop_ub;
  MATLABSUBSCRIBER_lock(SubscriberHelper);
  lastSubMsg_Fields.set_size(MsgStruct.Fields.size(0));
  loop_ub = MsgStruct.Fields.size(0);
  for (int i{0}; i < loop_ub; i++) {
    lastSubMsg_Fields[i] = MsgStruct.Fields[i];
  }
  lastSubMsg_Data.set_size(MsgStruct.Data.size(0));
  loop_ub = MsgStruct.Data.size(0);
  for (int i{0}; i < loop_ub; i++) {
    lastSubMsg_Data[i] = MsgStruct.Data[i];
  }
  MATLABSUBSCRIBER_unlock(SubscriberHelper);
  *lastSubMsg_Height = MsgStruct.Height;
  *lastSubMsg_Width = MsgStruct.Width;
  *lastSubMsg_PointStep = MsgStruct.PointStep;
}

void c_Subscriber::get_LatestMessage(
    double *lastSubMsg_Pose_Pose_Position_X,
    double *lastSubMsg_Pose_Pose_Position_Y,
    double *lastSubMsg_Pose_Pose_Position_Z,
    double *lastSubMsg_Pose_Pose_Orientation_X,
    double *lastSubMsg_Pose_Pose_Orientation_Y,
    double *lastSubMsg_Pose_Pose_Orientation_Z,
    double *lastSubMsg_Pose_Pose_Orientation_W) const
{
  MATLABSUBSCRIBER_lock(SubscriberHelper);
  *lastSubMsg_Pose_Pose_Position_X = MsgStruct.Pose.Pose.Position.X;
  *lastSubMsg_Pose_Pose_Position_Y = MsgStruct.Pose.Pose.Position.Y;
  *lastSubMsg_Pose_Pose_Position_Z = MsgStruct.Pose.Pose.Position.Z;
  *lastSubMsg_Pose_Pose_Orientation_X = MsgStruct.Pose.Pose.Orientation.X;
  *lastSubMsg_Pose_Pose_Orientation_Y = MsgStruct.Pose.Pose.Orientation.Y;
  *lastSubMsg_Pose_Pose_Orientation_Z = MsgStruct.Pose.Pose.Orientation.Z;
  *lastSubMsg_Pose_Pose_Orientation_W = MsgStruct.Pose.Pose.Orientation.W;
  MATLABSUBSCRIBER_unlock(SubscriberHelper);
}

Subscriber *Subscriber::init()
{
  static const char topic[24]{'/', 'p', 'r', 'o', 'c', '_', 'm', 'a',
                              'p', 'p', 'i', 'n', 'g', '/', 's', 't',
                              'a', 'r', 't', '_', 's', 't', 'o', 'p'};
  Subscriber *obj;
  obj = this;
  obj->IsInitialized = false;
  for (int i{0}; i < 24; i++) {
    obj->TopicName[i] = topic[i];
  }
  obj->BufferSize = 1.0;
  obj->MessageCount = 0.0;
  obj->MsgStruct = std_msgs_BoolStruct();
  auto structPtr = (&obj->MsgStruct);
  obj->SubscriberHelper =
      std::unique_ptr<MATLABSubscriber<std_msgs::Bool, std_msgs_BoolStruct_T>>(
          new MATLABSubscriber<std_msgs::Bool, std_msgs_BoolStruct_T>(
              structPtr, [this] { this->callback(); })); //();
  MATLABSUBSCRIBER_createSubscriber(obj->SubscriberHelper, &obj->TopicName[0],
                                    24.0, obj->BufferSize);
  obj->callback();
  obj->IsInitialized = true;
  return obj;
}

d_Subscriber *d_Subscriber::init()
{
  static const char topic[28]{'/', 'p', 'r', 'o', 'v', 'i', 'd', 'e', 'r', '_',
                              's', 'o', 'n', 'a', 'r', '/', 'p', 'o', 'i', 'n',
                              't', '_', 'c', 'l', 'o', 'u', 'd', '2'};
  d_Subscriber *obj;
  obj = this;
  obj->IsInitialized = false;
  for (int i{0}; i < 28; i++) {
    obj->TopicName[i] = topic[i];
  }
  obj->BufferSize = 1.0;
  obj->MessageCount = 0.0;
  sensor_msgs_PointCloud2Struct(&obj->MsgStruct);
  auto structPtr = (&obj->MsgStruct);
  obj->SubscriberHelper =
      std::unique_ptr<MATLABSubscriber<sensor_msgs::PointCloud2,
                                       sensor_msgs_PointCloud2Struct_T>>(
          new MATLABSubscriber<sensor_msgs::PointCloud2,
                               sensor_msgs_PointCloud2Struct_T>(
              structPtr, [this] { this->callback(); })); //();
  MATLABSUBSCRIBER_createSubscriber(obj->SubscriberHelper, &obj->TopicName[0],
                                    28.0, obj->BufferSize);
  obj->callback();
  obj->IsInitialized = true;
  return obj;
}

c_Subscriber *c_Subscriber::init()
{
  static const char topic[20]{'/', 'p', 'r', 'o', 'c', '_', 'n', 'a', 'v', '/',
                              'a', 'u', 'v', '_', 's', 't', 'a', 't', 'e', 's'};
  c_Subscriber *obj;
  obj = this;
  for (int i{0}; i < 20; i++) {
    obj->TopicName[i] = topic[i];
  }
  obj->BufferSize = 1.0;
  obj->MessageCount = 0.0;
  nav_msgs_OdometryStruct(&obj->MsgStruct);
  auto structPtr = (&obj->MsgStruct);
  obj->SubscriberHelper = std::unique_ptr<
      MATLABSubscriber<nav_msgs::Odometry, nav_msgs_OdometryStruct_T>>(
      new MATLABSubscriber<nav_msgs::Odometry, nav_msgs_OdometryStruct_T>(
          structPtr, [this] { this->callback(); })); //();
  MATLABSUBSCRIBER_createSubscriber(obj->SubscriberHelper, &obj->TopicName[0],
                                    20.0, obj->BufferSize);
  obj->callback();
  return obj;
}

e_Subscriber *e_Subscriber::init()
{
  static const char topic[40]{'/', 'c', 'a', 'm', 'e', 'r', 'a', '_', 'a', 'r',
                              'r', 'a', 'y', '/', 'f', 'r', 'o', 'n', 't', '/',
                              'i', 'm', 'a', 'g', 'e', '_', 'r', 'a', 'w', '/',
                              'c', 'o', 'm', 'p', 'r', 'e', 's', 's', 'e', 'd'};
  e_Subscriber *obj;
  obj = this;
  for (int i{0}; i < 40; i++) {
    obj->TopicName[i] = topic[i];
  }
  obj->BufferSize = 1.0;
  obj->MessageCount = 0.0;
  sensor_msgs_CompressedImageStruct(&obj->MsgStruct);
  auto structPtr = (&obj->MsgStruct);
  obj->SubscriberHelper =
      std::unique_ptr<MATLABSubscriber<sensor_msgs::CompressedImage,
                                       sensor_msgs_CompressedImageStruct_T>>(
          new MATLABSubscriber<sensor_msgs::CompressedImage,
                               sensor_msgs_CompressedImageStruct_T>(
              structPtr, [this] { this->callback(); })); //();
  MATLABSUBSCRIBER_createSubscriber(obj->SubscriberHelper, &obj->TopicName[0],
                                    40.0, obj->BufferSize);
  obj->callback();
  return obj;
}

b_Subscriber *b_Subscriber::init()
{
  static const char topic[26]{'/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p',
                              'p', 'i', 'n', 'g', '/', 'c', 'l', 'e', 'a',
                              'r', '_', 'b', 'u', 'n', 'd', 'l', 'e'};
  b_Subscriber *obj;
  obj = this;
  obj->IsInitialized = false;
  for (int i{0}; i < 26; i++) {
    obj->TopicName[i] = topic[i];
  }
  obj->BufferSize = 1.0;
  obj->MessageCount = 0.0;
  obj->MsgStruct = std_msgs_BoolStruct();
  auto structPtr = (&obj->MsgStruct);
  obj->SubscriberHelper =
      std::unique_ptr<MATLABSubscriber<std_msgs::Bool, std_msgs_BoolStruct_T>>(
          new MATLABSubscriber<std_msgs::Bool, std_msgs_BoolStruct_T>(
              structPtr, [this] { this->callback(); })); //();
  MATLABSUBSCRIBER_createSubscriber(obj->SubscriberHelper, &obj->TopicName[0],
                                    26.0, obj->BufferSize);
  obj->callback();
  obj->IsInitialized = true;
  return obj;
}

} // namespace ros
} // namespace coder

// End of code generation (Subscriber.cpp)
