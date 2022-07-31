#include "ros_structmsg_conversion.h"


// Conversions between geometry_msgs_PointStruct_T and geometry_msgs::Point

void struct2msg(geometry_msgs::Point* msgPtr, geometry_msgs_PointStruct_T const* structPtr)
{
  const std::string rosMessageType("geometry_msgs/Point");

  msgPtr->x =  structPtr->X;
  msgPtr->y =  structPtr->Y;
  msgPtr->z =  structPtr->Z;
}

void msg2struct(geometry_msgs_PointStruct_T* structPtr, geometry_msgs::Point const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Point");

  structPtr->X =  msgPtr->x;
  structPtr->Y =  msgPtr->y;
  structPtr->Z =  msgPtr->z;
}


// Conversions between geometry_msgs_PoseStruct_T and geometry_msgs::Pose

void struct2msg(geometry_msgs::Pose* msgPtr, geometry_msgs_PoseStruct_T const* structPtr)
{
  const std::string rosMessageType("geometry_msgs/Pose");

  struct2msg(&msgPtr->orientation, &structPtr->Orientation);
  struct2msg(&msgPtr->position, &structPtr->Position);
}

void msg2struct(geometry_msgs_PoseStruct_T* structPtr, geometry_msgs::Pose const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Pose");

  msg2struct(&structPtr->Orientation, &msgPtr->orientation);
  msg2struct(&structPtr->Position, &msgPtr->position);
}


// Conversions between geometry_msgs_PoseWithCovarianceStruct_T and geometry_msgs::PoseWithCovariance

void struct2msg(geometry_msgs::PoseWithCovariance* msgPtr, geometry_msgs_PoseWithCovarianceStruct_T const* structPtr)
{
  const std::string rosMessageType("geometry_msgs/PoseWithCovariance");

  convertFromStructPrimitiveArray(msgPtr->covariance, structPtr->Covariance);
  struct2msg(&msgPtr->pose, &structPtr->Pose);
}

void msg2struct(geometry_msgs_PoseWithCovarianceStruct_T* structPtr, geometry_msgs::PoseWithCovariance const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/PoseWithCovariance");

  convertToStructPrimitiveArray(structPtr->Covariance, msgPtr->covariance);
  msg2struct(&structPtr->Pose, &msgPtr->pose);
}


// Conversions between geometry_msgs_QuaternionStruct_T and geometry_msgs::Quaternion

void struct2msg(geometry_msgs::Quaternion* msgPtr, geometry_msgs_QuaternionStruct_T const* structPtr)
{
  const std::string rosMessageType("geometry_msgs/Quaternion");

  msgPtr->w =  structPtr->W;
  msgPtr->x =  structPtr->X;
  msgPtr->y =  structPtr->Y;
  msgPtr->z =  structPtr->Z;
}

void msg2struct(geometry_msgs_QuaternionStruct_T* structPtr, geometry_msgs::Quaternion const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Quaternion");

  structPtr->W =  msgPtr->w;
  structPtr->X =  msgPtr->x;
  structPtr->Y =  msgPtr->y;
  structPtr->Z =  msgPtr->z;
}


// Conversions between geometry_msgs_TwistStruct_T and geometry_msgs::Twist

void struct2msg(geometry_msgs::Twist* msgPtr, geometry_msgs_TwistStruct_T const* structPtr)
{
  const std::string rosMessageType("geometry_msgs/Twist");

  struct2msg(&msgPtr->angular, &structPtr->Angular);
  struct2msg(&msgPtr->linear, &structPtr->Linear);
}

void msg2struct(geometry_msgs_TwistStruct_T* structPtr, geometry_msgs::Twist const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Twist");

  msg2struct(&structPtr->Angular, &msgPtr->angular);
  msg2struct(&structPtr->Linear, &msgPtr->linear);
}


// Conversions between geometry_msgs_TwistWithCovarianceStruct_T and geometry_msgs::TwistWithCovariance

void struct2msg(geometry_msgs::TwistWithCovariance* msgPtr, geometry_msgs_TwistWithCovarianceStruct_T const* structPtr)
{
  const std::string rosMessageType("geometry_msgs/TwistWithCovariance");

  convertFromStructPrimitiveArray(msgPtr->covariance, structPtr->Covariance);
  struct2msg(&msgPtr->twist, &structPtr->Twist);
}

void msg2struct(geometry_msgs_TwistWithCovarianceStruct_T* structPtr, geometry_msgs::TwistWithCovariance const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/TwistWithCovariance");

  convertToStructPrimitiveArray(structPtr->Covariance, msgPtr->covariance);
  msg2struct(&structPtr->Twist, &msgPtr->twist);
}


// Conversions between geometry_msgs_Vector3Struct_T and geometry_msgs::Vector3

void struct2msg(geometry_msgs::Vector3* msgPtr, geometry_msgs_Vector3Struct_T const* structPtr)
{
  const std::string rosMessageType("geometry_msgs/Vector3");

  msgPtr->x =  structPtr->X;
  msgPtr->y =  structPtr->Y;
  msgPtr->z =  structPtr->Z;
}

void msg2struct(geometry_msgs_Vector3Struct_T* structPtr, geometry_msgs::Vector3 const* msgPtr)
{
  const std::string rosMessageType("geometry_msgs/Vector3");

  structPtr->X =  msgPtr->x;
  structPtr->Y =  msgPtr->y;
  structPtr->Z =  msgPtr->z;
}


// Conversions between nav_msgs_OdometryStruct_T and nav_msgs::Odometry

void struct2msg(nav_msgs::Odometry* msgPtr, nav_msgs_OdometryStruct_T const* structPtr)
{
  const std::string rosMessageType("nav_msgs/Odometry");

  convertFromStructPrimitiveArray(msgPtr->child_frame_id, structPtr->ChildFrameId);
  struct2msg(&msgPtr->header, &structPtr->Header);
  struct2msg(&msgPtr->pose, &structPtr->Pose);
  struct2msg(&msgPtr->twist, &structPtr->Twist);
}

void msg2struct(nav_msgs_OdometryStruct_T* structPtr, nav_msgs::Odometry const* msgPtr)
{
  const std::string rosMessageType("nav_msgs/Odometry");

  convertToStructPrimitiveArray(structPtr->ChildFrameId, msgPtr->child_frame_id);
  msg2struct(&structPtr->Header, &msgPtr->header);
  msg2struct(&structPtr->Pose, &msgPtr->pose);
  msg2struct(&structPtr->Twist, &msgPtr->twist);
}


// Conversions between ros_TimeStruct_T and ros::Time

void struct2msg(ros::Time* msgPtr, ros_TimeStruct_T const* structPtr)
{
  const std::string rosMessageType("ros_time/Time");

  msgPtr->nsec =  structPtr->Nsec;
  msgPtr->sec =  structPtr->Sec;
}

void msg2struct(ros_TimeStruct_T* structPtr, ros::Time const* msgPtr)
{
  const std::string rosMessageType("ros_time/Time");

  structPtr->Nsec =  msgPtr->nsec;
  structPtr->Sec =  msgPtr->sec;
}


// Conversions between sensor_msgs_PointCloud2Struct_T and sensor_msgs::PointCloud2

void struct2msg(sensor_msgs::PointCloud2* msgPtr, sensor_msgs_PointCloud2Struct_T const* structPtr)
{
  const std::string rosMessageType("sensor_msgs/PointCloud2");

  convertFromStructPrimitiveArray(msgPtr->data, structPtr->Data);
  convertFromStructNestedArray(msgPtr->fields, structPtr->Fields);
  struct2msg(&msgPtr->header, &structPtr->Header);
  msgPtr->height =  structPtr->Height;
  msgPtr->is_bigendian =  structPtr->IsBigendian;
  msgPtr->is_dense =  structPtr->IsDense;
  msgPtr->point_step =  structPtr->PointStep;
  msgPtr->row_step =  structPtr->RowStep;
  msgPtr->width =  structPtr->Width;
}

void msg2struct(sensor_msgs_PointCloud2Struct_T* structPtr, sensor_msgs::PointCloud2 const* msgPtr)
{
  const std::string rosMessageType("sensor_msgs/PointCloud2");

  convertToStructPrimitiveArray(structPtr->Data, msgPtr->data);
  convertToStructNestedArray(structPtr->Fields, msgPtr->fields);
  msg2struct(&structPtr->Header, &msgPtr->header);
  structPtr->Height =  msgPtr->height;
  structPtr->IsBigendian =  msgPtr->is_bigendian;
  structPtr->IsDense =  msgPtr->is_dense;
  structPtr->PointStep =  msgPtr->point_step;
  structPtr->RowStep =  msgPtr->row_step;
  structPtr->Width =  msgPtr->width;
}


// Conversions between sensor_msgs_PointFieldStruct_T and sensor_msgs::PointField

void struct2msg(sensor_msgs::PointField* msgPtr, sensor_msgs_PointFieldStruct_T const* structPtr)
{
  const std::string rosMessageType("sensor_msgs/PointField");

  msgPtr->count =  structPtr->Count;
  msgPtr->datatype =  structPtr->Datatype;
  convertFromStructPrimitiveArray(msgPtr->name, structPtr->Name);
  msgPtr->offset =  structPtr->Offset;
}

void msg2struct(sensor_msgs_PointFieldStruct_T* structPtr, sensor_msgs::PointField const* msgPtr)
{
  const std::string rosMessageType("sensor_msgs/PointField");

  structPtr->Count =  msgPtr->count;
  structPtr->Datatype =  msgPtr->datatype;
  convertToStructPrimitiveArray(structPtr->Name, msgPtr->name);
  structPtr->Offset =  msgPtr->offset;
}


// Conversions between sonia_common_ObstacleArrayStruct_T and sonia_common::ObstacleArray

void struct2msg(sonia_common::ObstacleArray* msgPtr, sonia_common_ObstacleArrayStruct_T const* structPtr)
{
  const std::string rosMessageType("sonia_common/ObstacleArray");

  struct2msg(&msgPtr->header, &structPtr->Header);
  convertFromStructNestedArray(msgPtr->obstacles, structPtr->Obstacles);
}

void msg2struct(sonia_common_ObstacleArrayStruct_T* structPtr, sonia_common::ObstacleArray const* msgPtr)
{
  const std::string rosMessageType("sonia_common/ObstacleArray");

  msg2struct(&structPtr->Header, &msgPtr->header);
  convertToStructNestedArray(structPtr->Obstacles, msgPtr->obstacles);
}


// Conversions between sonia_common_ObstacleInfoStruct_T and sonia_common::ObstacleInfo

void struct2msg(sonia_common::ObstacleInfo* msgPtr, sonia_common_ObstacleInfoStruct_T const* structPtr)
{
  const std::string rosMessageType("sonia_common/ObstacleInfo");

  msgPtr->confidence =  structPtr->Confidence;
  msgPtr->is_valid =  structPtr->IsValid;
  convertFromStructPrimitiveArray(msgPtr->name, structPtr->Name);
  struct2msg(&msgPtr->pose, &structPtr->Pose);
}

void msg2struct(sonia_common_ObstacleInfoStruct_T* structPtr, sonia_common::ObstacleInfo const* msgPtr)
{
  const std::string rosMessageType("sonia_common/ObstacleInfo");

  structPtr->Confidence =  msgPtr->confidence;
  structPtr->IsValid =  msgPtr->is_valid;
  convertToStructPrimitiveArray(structPtr->Name, msgPtr->name);
  msg2struct(&structPtr->Pose, &msgPtr->pose);
}


// Conversions between sonia_common_PingAnglesStruct_T and sonia_common::PingAngles

void struct2msg(sonia_common::PingAngles* msgPtr, sonia_common_PingAnglesStruct_T const* structPtr)
{
  const std::string rosMessageType("sonia_common/PingAngles");

  msgPtr->elevation =  structPtr->Elevation;
  msgPtr->frequency =  structPtr->Frequency;
  struct2msg(&msgPtr->header, &structPtr->Header);
  msgPtr->heading =  structPtr->Heading;
  msgPtr->snr =  structPtr->Snr;
}

void msg2struct(sonia_common_PingAnglesStruct_T* structPtr, sonia_common::PingAngles const* msgPtr)
{
  const std::string rosMessageType("sonia_common/PingAngles");

  structPtr->Elevation =  msgPtr->elevation;
  structPtr->Frequency =  msgPtr->frequency;
  msg2struct(&structPtr->Header, &msgPtr->header);
  structPtr->Heading =  msgPtr->heading;
  structPtr->Snr =  msgPtr->snr;
}


// Conversions between std_msgs_BoolStruct_T and std_msgs::Bool

void struct2msg(std_msgs::Bool* msgPtr, std_msgs_BoolStruct_T const* structPtr)
{
  const std::string rosMessageType("std_msgs/Bool");

  msgPtr->data =  structPtr->Data;
}

void msg2struct(std_msgs_BoolStruct_T* structPtr, std_msgs::Bool const* msgPtr)
{
  const std::string rosMessageType("std_msgs/Bool");

  structPtr->Data =  msgPtr->data;
}


// Conversions between std_msgs_HeaderStruct_T and std_msgs::Header

void struct2msg(std_msgs::Header* msgPtr, std_msgs_HeaderStruct_T const* structPtr)
{
  const std::string rosMessageType("std_msgs/Header");

  convertFromStructPrimitiveArray(msgPtr->frame_id, structPtr->FrameId);
  msgPtr->seq =  structPtr->Seq;
  struct2msg(&msgPtr->stamp, &structPtr->Stamp);
}

void msg2struct(std_msgs_HeaderStruct_T* structPtr, std_msgs::Header const* msgPtr)
{
  const std::string rosMessageType("std_msgs/Header");

  convertToStructPrimitiveArray(structPtr->FrameId, msgPtr->frame_id);
  structPtr->Seq =  msgPtr->seq;
  msg2struct(&structPtr->Stamp, &msgPtr->stamp);
}


// Conversions between std_msgs_StringStruct_T and std_msgs::String

void struct2msg(std_msgs::String* msgPtr, std_msgs_StringStruct_T const* structPtr)
{
  const std::string rosMessageType("std_msgs/String");

  convertFromStructPrimitiveArray(msgPtr->data, structPtr->Data);
}

void msg2struct(std_msgs_StringStruct_T* structPtr, std_msgs::String const* msgPtr)
{
  const std::string rosMessageType("std_msgs/String");

  convertToStructPrimitiveArray(structPtr->Data, msgPtr->data);
}


// Conversions between std_msgs_UInt16Struct_T and std_msgs::UInt16

void struct2msg(std_msgs::UInt16* msgPtr, std_msgs_UInt16Struct_T const* structPtr)
{
  const std::string rosMessageType("std_msgs/UInt16");

  msgPtr->data =  structPtr->Data;
}

void msg2struct(std_msgs_UInt16Struct_T* structPtr, std_msgs::UInt16 const* msgPtr)
{
  const std::string rosMessageType("std_msgs/UInt16");

  structPtr->Data =  msgPtr->data;
}

