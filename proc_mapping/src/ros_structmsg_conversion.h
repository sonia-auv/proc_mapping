#ifndef _ROS_STRUCTMSG_CONVERSION_H_
#define _ROS_STRUCTMSG_CONVERSION_H_

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <ros/time.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sonia_common/ObstacleArray.h>
#include <sonia_common/ObstacleInfo.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include "proc_mapping_types.h"
#include "mlroscpp_msgconvert_utils.h"


void struct2msg(geometry_msgs::Point* msgPtr, geometry_msgs_PointStruct_T const* structPtr);
void msg2struct(geometry_msgs_PointStruct_T* structPtr, geometry_msgs::Point const* msgPtr);

void struct2msg(geometry_msgs::Pose* msgPtr, geometry_msgs_PoseStruct_T const* structPtr);
void msg2struct(geometry_msgs_PoseStruct_T* structPtr, geometry_msgs::Pose const* msgPtr);

void struct2msg(geometry_msgs::PoseWithCovariance* msgPtr, geometry_msgs_PoseWithCovarianceStruct_T const* structPtr);
void msg2struct(geometry_msgs_PoseWithCovarianceStruct_T* structPtr, geometry_msgs::PoseWithCovariance const* msgPtr);

void struct2msg(geometry_msgs::Quaternion* msgPtr, geometry_msgs_QuaternionStruct_T const* structPtr);
void msg2struct(geometry_msgs_QuaternionStruct_T* structPtr, geometry_msgs::Quaternion const* msgPtr);

void struct2msg(geometry_msgs::Twist* msgPtr, geometry_msgs_TwistStruct_T const* structPtr);
void msg2struct(geometry_msgs_TwistStruct_T* structPtr, geometry_msgs::Twist const* msgPtr);

void struct2msg(geometry_msgs::TwistWithCovariance* msgPtr, geometry_msgs_TwistWithCovarianceStruct_T const* structPtr);
void msg2struct(geometry_msgs_TwistWithCovarianceStruct_T* structPtr, geometry_msgs::TwistWithCovariance const* msgPtr);

void struct2msg(geometry_msgs::Vector3* msgPtr, geometry_msgs_Vector3Struct_T const* structPtr);
void msg2struct(geometry_msgs_Vector3Struct_T* structPtr, geometry_msgs::Vector3 const* msgPtr);

void struct2msg(nav_msgs::Odometry* msgPtr, nav_msgs_OdometryStruct_T const* structPtr);
void msg2struct(nav_msgs_OdometryStruct_T* structPtr, nav_msgs::Odometry const* msgPtr);

void struct2msg(ros::Time* msgPtr, ros_TimeStruct_T const* structPtr);
void msg2struct(ros_TimeStruct_T* structPtr, ros::Time const* msgPtr);

void struct2msg(sensor_msgs::CompressedImage* msgPtr, sensor_msgs_CompressedImageStruct_T const* structPtr);
void msg2struct(sensor_msgs_CompressedImageStruct_T* structPtr, sensor_msgs::CompressedImage const* msgPtr);

void struct2msg(sensor_msgs::PointCloud2* msgPtr, sensor_msgs_PointCloud2Struct_T const* structPtr);
void msg2struct(sensor_msgs_PointCloud2Struct_T* structPtr, sensor_msgs::PointCloud2 const* msgPtr);

void struct2msg(sensor_msgs::PointField* msgPtr, sensor_msgs_PointFieldStruct_T const* structPtr);
void msg2struct(sensor_msgs_PointFieldStruct_T* structPtr, sensor_msgs::PointField const* msgPtr);

void struct2msg(sonia_common::ObstacleArray* msgPtr, sonia_common_ObstacleArrayStruct_T const* structPtr);
void msg2struct(sonia_common_ObstacleArrayStruct_T* structPtr, sonia_common::ObstacleArray const* msgPtr);

void struct2msg(sonia_common::ObstacleInfo* msgPtr, sonia_common_ObstacleInfoStruct_T const* structPtr);
void msg2struct(sonia_common_ObstacleInfoStruct_T* structPtr, sonia_common::ObstacleInfo const* msgPtr);

void struct2msg(std_msgs::Bool* msgPtr, std_msgs_BoolStruct_T const* structPtr);
void msg2struct(std_msgs_BoolStruct_T* structPtr, std_msgs::Bool const* msgPtr);

void struct2msg(std_msgs::Header* msgPtr, std_msgs_HeaderStruct_T const* structPtr);
void msg2struct(std_msgs_HeaderStruct_T* structPtr, std_msgs::Header const* msgPtr);

void struct2msg(std_msgs::String* msgPtr, std_msgs_StringStruct_T const* structPtr);
void msg2struct(std_msgs_StringStruct_T* structPtr, std_msgs::String const* msgPtr);


#endif
