//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Subscriber.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 01-Aug-2022 08:26:09
//

// Include Files
#include "Subscriber.h"
#include "nav_msgs_OdometryStruct.h"
#include "proc_mapping_data.h"
#include "proc_mapping_types.h"
#include "rt_nonfinite.h"
#include "sensor_msgs_PointCloud2Struct.h"
#include "sonia_common_PingAnglesStruct.h"
#include "std_msgs_BoolStruct.h"
#include "std_msgs_StringStruct.h"
#include "std_msgs_UInt16Struct.h"
#include "coder_array.h"
#include "mlroscpp_sub.h"
#include <stdio.h>
#include <string.h>

// Function Definitions
//
// Arguments    : void
// Return Type  : void
//
namespace coder {
namespace ros {
void c_Subscriber::b_callback()
{
  MessageCount = get_MessageCount() + 1.0;
  if (IsInitialized) {
    //  Initial variables
    //  SET
    b_newClearBundleMsg = true;
  }
}

//
// Arguments    : void
// Return Type  : void
//
void b_Subscriber::b_callback()
{
  MessageCount = get_MessageCount() + 1.0;
  if (IsInitialized) {
    //  Initial variables
    //  SET
    b_bundleStarted = false;
    printf("INFO : proc mapping : hydro : Bundle record stopped \n");
    fflush(stdout);
  }
}

//
// Arguments    : void
// Return Type  : void
//
void Subscriber::callback()
{
  MessageCount = get_MessageCount() + 1.0;
  if (IsInitialized) {
    //  Initial variables
    //  SET
    bundleStarted = true;
    printf("INFO : proc mapping : sonar : Bundle record started \n");
    fflush(stdout);
  }
}

//
// Arguments    : void
// Return Type  : void
//
void b_Subscriber::callback()
{
  MessageCount = get_MessageCount() + 1.0;
  if (IsInitialized) {
    //  Initial variables
    //  SET
    bundleStarted = false;
    printf("INFO : proc mapping : sonar : Bundle record stopped \n");
    fflush(stdout);
  }
}

//
// Arguments    : void
// Return Type  : void
//
void c_Subscriber::callback()
{
  MessageCount = get_MessageCount() + 1.0;
  if (IsInitialized) {
    //  Initial variables
    //  SET
    newClearBundleMsg = true;
  }
}

//
// Arguments    : void
// Return Type  : void
//
void e_Subscriber::callback()
{
  MessageCount = get_MessageCount() + 1.0;
  if (IsInitialized) {
    //  Initial variables
    //  SET
    newSonarMsg = true;
  }
}

//
// Arguments    : void
// Return Type  : void
//
void f_Subscriber::callback()
{
  MessageCount = get_MessageCount() + 1.0;
  if (IsInitialized) {
    //  Initial variables
    //  SET
    b_bundleStarted = true;
    printf("INFO : proc mapping : hydro : Bundle record started \n");
    fflush(stdout);
  }
}

//
// Arguments    : void
// Return Type  : void
//
void d_Subscriber::callback()
{
  MessageCount = get_MessageCount() + 1.0;
}

//
// Arguments    : void
// Return Type  : void
//
void g_Subscriber::callback()
{
  MessageCount = get_MessageCount() + 1.0;
  if (IsInitialized) {
    //  Initial variables
    //  SET
    newHydroMsg = true;
  }
}

//
// Arguments    : void
// Return Type  : double
//
double g_Subscriber::get_MessageCount() const
{
  return MessageCount;
}

//
// Arguments    : void
// Return Type  : double
//
double c_Subscriber::get_MessageCount() const
{
  return MessageCount;
}

//
// Arguments    : void
// Return Type  : double
//
double b_Subscriber::get_MessageCount() const
{
  return MessageCount;
}

//
// Arguments    : void
// Return Type  : double
//
double f_Subscriber::get_MessageCount() const
{
  return MessageCount;
}

//
// Arguments    : void
// Return Type  : double
//
double d_Subscriber::get_MessageCount() const
{
  return MessageCount;
}

//
// Arguments    : void
// Return Type  : double
//
double e_Subscriber::get_MessageCount() const
{
  return MessageCount;
}

//
// Arguments    : void
// Return Type  : double
//
double Subscriber::get_MessageCount() const
{
  return MessageCount;
}

//
// Arguments    : void
// Return Type  : b_Subscriber *
//
b_Subscriber *b_Subscriber::b_init()
{
  static const char topic[24]{'/', 'p', 'r', 'o', 'c', '_', 'm', 'a',
                              'p', 'p', 'i', 'n', 'g', '/', 'h', 'y',
                              'd', 'r', 'o', '/', 's', 't', 'o', 'p'};
  b_Subscriber *obj;
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
  obj->b_callback();
  obj->IsInitialized = true;
  return obj;
}

//
// Arguments    : void
// Return Type  : c_Subscriber *
//
c_Subscriber *c_Subscriber::b_init()
{
  static const char topic[32]{'/', 'p', 'r', 'o', 'c', '_', 'm', 'a',
                              'p', 'p', 'i', 'n', 'g', '/', 'h', 'y',
                              'd', 'r', 'o', '/', 'c', 'l', 'e', 'a',
                              'r', '_', 'b', 'u', 'n', 'd', 'l', 'e'};
  c_Subscriber *obj;
  obj = this;
  obj->IsInitialized = false;
  for (int i{0}; i < 32; i++) {
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
                                    32.0, obj->BufferSize);
  obj->b_callback();
  obj->IsInitialized = true;
  return obj;
}

//
// Arguments    : geometry_msgs_PoseWithCovarianceStruct_T *lastSubMsg_Pose
// Return Type  : void
//
void d_Subscriber::get_LatestMessage(
    geometry_msgs_PoseWithCovarianceStruct_T *lastSubMsg_Pose) const
{
  MATLABSUBSCRIBER_lock(SubscriberHelper);
  *lastSubMsg_Pose = MsgStruct.Pose;
  MATLABSUBSCRIBER_unlock(SubscriberHelper);
}

//
// Arguments    : void
// Return Type  : unsigned short
//
unsigned short f_Subscriber::get_LatestMessage() const
{
  unsigned short lastSubMsg_Data;
  MATLABSUBSCRIBER_lock(SubscriberHelper);
  lastSubMsg_Data = MsgStruct.Data;
  MATLABSUBSCRIBER_unlock(SubscriberHelper);
  return lastSubMsg_Data;
}

//
// Arguments    : double *lastSubMsg_Heading
//                double *lastSubMsg_Elevation
//                unsigned short *lastSubMsg_Frequency
//                unsigned short *lastSubMsg_Snr
// Return Type  : void
//
void g_Subscriber::get_LatestMessage(double *lastSubMsg_Heading,
                                     double *lastSubMsg_Elevation,
                                     unsigned short *lastSubMsg_Frequency,
                                     unsigned short *lastSubMsg_Snr) const
{
  MATLABSUBSCRIBER_lock(SubscriberHelper);
  *lastSubMsg_Heading = MsgStruct.Heading;
  *lastSubMsg_Elevation = MsgStruct.Elevation;
  *lastSubMsg_Frequency = MsgStruct.Frequency;
  *lastSubMsg_Snr = MsgStruct.Snr;
  MATLABSUBSCRIBER_unlock(SubscriberHelper);
}

//
// Arguments    : ::coder::array<char, 2U> &lastSubMsg_Data
// Return Type  : void
//
void Subscriber::get_LatestMessage(
    ::coder::array<char, 2U> &lastSubMsg_Data) const
{
  int loop_ub;
  MATLABSUBSCRIBER_lock(SubscriberHelper);
  lastSubMsg_Data.set_size(1, MsgStruct.Data.size(1));
  loop_ub = MsgStruct.Data.size(1);
  for (int i{0}; i < loop_ub; i++) {
    lastSubMsg_Data[i] = MsgStruct.Data[i];
  }
  MATLABSUBSCRIBER_unlock(SubscriberHelper);
}

//
// Arguments    : unsigned int *lastSubMsg_Height
//                unsigned int *lastSubMsg_Width
//                ::coder::array<sensor_msgs_PointFieldStruct_T, 1U>
//                &lastSubMsg_Fields unsigned int *lastSubMsg_PointStep
//                ::coder::array<unsigned char, 1U> &lastSubMsg_Data
// Return Type  : void
//
void e_Subscriber::get_LatestMessage(
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

//
// Arguments    : void
// Return Type  : b_Subscriber *
//
b_Subscriber *b_Subscriber::init()
{
  static const char topic[24]{'/', 'p', 'r', 'o', 'c', '_', 'm', 'a',
                              'p', 'p', 'i', 'n', 'g', '/', 's', 'o',
                              'n', 'a', 'r', '/', 's', 't', 'o', 'p'};
  b_Subscriber *obj;
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

//
// Arguments    : void
// Return Type  : c_Subscriber *
//
c_Subscriber *c_Subscriber::init()
{
  static const char topic[32]{'/', 'p', 'r', 'o', 'c', '_', 'm', 'a',
                              'p', 'p', 'i', 'n', 'g', '/', 's', 'o',
                              'n', 'a', 'r', '/', 'c', 'l', 'e', 'a',
                              'r', '_', 'b', 'u', 'n', 'd', 'l', 'e'};
  c_Subscriber *obj;
  obj = this;
  obj->IsInitialized = false;
  for (int i{0}; i < 32; i++) {
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
                                    32.0, obj->BufferSize);
  obj->callback();
  obj->IsInitialized = true;
  return obj;
}

//
// Arguments    : void
// Return Type  : e_Subscriber *
//
e_Subscriber *e_Subscriber::init()
{
  static const char topic[28]{'/', 'p', 'r', 'o', 'v', 'i', 'd', 'e', 'r', '_',
                              's', 'o', 'n', 'a', 'r', '/', 'p', 'o', 'i', 'n',
                              't', '_', 'c', 'l', 'o', 'u', 'd', '2'};
  e_Subscriber *obj;
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

//
// Arguments    : void
// Return Type  : d_Subscriber *
//
d_Subscriber *d_Subscriber::init()
{
  static const char topic[20]{'/', 'p', 'r', 'o', 'c', '_', 'n', 'a', 'v', '/',
                              'a', 'u', 'v', '_', 's', 't', 'a', 't', 'e', 's'};
  d_Subscriber *obj;
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

//
// Arguments    : void
// Return Type  : f_Subscriber *
//
f_Subscriber *f_Subscriber::init()
{
  static const char topic[25]{'/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p',
                              'p', 'i', 'n', 'g', '/', 'h', 'y', 'd', 'r',
                              'o', '/', 's', 't', 'a', 'r', 't'};
  f_Subscriber *obj;
  obj = this;
  obj->IsInitialized = false;
  for (int i{0}; i < 25; i++) {
    obj->TopicName[i] = topic[i];
  }
  obj->BufferSize = 1.0;
  obj->MessageCount = 0.0;
  obj->MsgStruct = std_msgs_UInt16Struct();
  auto structPtr = (&obj->MsgStruct);
  obj->SubscriberHelper = std::unique_ptr<
      MATLABSubscriber<std_msgs::UInt16, std_msgs_UInt16Struct_T>>(
      new MATLABSubscriber<std_msgs::UInt16, std_msgs_UInt16Struct_T>(
          structPtr, [this] { this->callback(); })); //();
  MATLABSUBSCRIBER_createSubscriber(obj->SubscriberHelper, &obj->TopicName[0],
                                    25.0, obj->BufferSize);
  obj->callback();
  obj->IsInitialized = true;
  return obj;
}

//
// Arguments    : void
// Return Type  : Subscriber *
//
Subscriber *Subscriber::init()
{
  static const char topic[25]{'/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p',
                              'p', 'i', 'n', 'g', '/', 's', 'o', 'n', 'a',
                              'r', '/', 's', 't', 'a', 'r', 't'};
  Subscriber *obj;
  obj = this;
  obj->IsInitialized = false;
  for (int i{0}; i < 25; i++) {
    obj->TopicName[i] = topic[i];
  }
  obj->BufferSize = 1.0;
  obj->MessageCount = 0.0;
  std_msgs_StringStruct(&obj->MsgStruct);
  auto structPtr = (&obj->MsgStruct);
  obj->SubscriberHelper = std::unique_ptr<
      MATLABSubscriber<std_msgs::String, std_msgs_StringStruct_T>>(
      new MATLABSubscriber<std_msgs::String, std_msgs_StringStruct_T>(
          structPtr, [this] { this->callback(); })); //();
  MATLABSUBSCRIBER_createSubscriber(obj->SubscriberHelper, &obj->TopicName[0],
                                    25.0, obj->BufferSize);
  obj->callback();
  obj->IsInitialized = true;
  return obj;
}

//
// Arguments    : void
// Return Type  : g_Subscriber *
//
g_Subscriber *g_Subscriber::init()
{
  static const char topic[21]{'/', 'p', 'r', 'o', 'c', '_', 'h',
                              'y', 'd', 'r', 'o', 'p', 'h', 'o',
                              'n', 'e', '/', 'p', 'i', 'n', 'g'};
  g_Subscriber *obj;
  obj = this;
  obj->IsInitialized = false;
  for (int i{0}; i < 21; i++) {
    obj->TopicName[i] = topic[i];
  }
  obj->BufferSize = 1.0;
  obj->MessageCount = 0.0;
  sonia_common_PingAnglesStruct(&obj->MsgStruct);
  auto structPtr = (&obj->MsgStruct);
  obj->SubscriberHelper =
      std::unique_ptr<MATLABSubscriber<sonia_common::PingAngles,
                                       sonia_common_PingAnglesStruct_T>>(
          new MATLABSubscriber<sonia_common::PingAngles,
                               sonia_common_PingAnglesStruct_T>(
              structPtr, [this] { this->callback(); })); //();
  MATLABSUBSCRIBER_createSubscriber(obj->SubscriberHelper, &obj->TopicName[0],
                                    21.0, obj->BufferSize);
  obj->callback();
  obj->IsInitialized = true;
  return obj;
}

} // namespace ros
} // namespace coder

//
// File trailer for Subscriber.cpp
//
// [EOF]
//
