/**
 * \file	raw_map_test.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	06/02/2016
 *
 * \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
 *
 * \section LICENSE
 *
 * This file is part of S.O.N.I.A. software.
 *
 * S.O.N.I.A. software is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S.O.N.I.A. software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.
 */

#include <gtest/gtest.h>
#include "ros/ros.h"
#include <thread>
#include <opencv2/opencv.hpp>
#include <pcl/common/transforms.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include "nav_msgs/Odometry.h"
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Quaternion.h>
#include "lib_atlas/maths.h"
#include "lib_atlas/pattern/runnable.h"
#include <eigen3/Eigen/Geometry>



#include "proc_mapping/interpreter/raw_map.h"

static const std::string node_prefix("proc_mapping");

class OdometryEmulator : public atlas::Runnable {
 public:
  explicit OdometryEmulator() {
    ros::NodeHandle n("proc_navigation");
    publisher_ = n.advertise<nav_msgs::Odometry>("Odometry", 100);
  }

  void Run() override
  {
    ros::Rate loop_rate(100);
    nav_msgs::Odometry msg;

    float yaw = 3*M_PI/2, pitch = M_PI/8, roll = M_PI/4;
    float x = 0.0f, y = 0.0f, z = 0.0f;
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle ;
    q.normalize();
    geometry_msgs::Quaternion quaternion;
    quaternion.w = q.w();
    quaternion.x = q.x();
    quaternion.y = q.y();
    quaternion.z = q.z();
    msg.pose.pose.orientation = quaternion;
    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.position.z = z;


    while (!MustStop()) {
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = "NED";
      publisher_.publish(msg);
      ros::spinOnce();
      loop_rate.sleep();
    }
  }

 private:
  ros::Publisher publisher_;
};

class SonarEmulator : public atlas::Runnable  {
 public:
  explicit SonarEmulator() {
    ros::NodeHandle n("sonar_node");
    publisher_ = n.advertise<sensor_msgs::PointCloud2>("point_cloud2", 100);
  }

  void Run() override
  {
    ros::Rate loop_rate(10);

    std::vector<std::pair<float, uint8_t> > intensity_bins(400);
    float bin_distance_step = 0.02f;

    for (size_t i = 0; i < intensity_bins.size(); ++i) {
      intensity_bins[i].first = bin_distance_step * (float)(i + 1);
      intensity_bins[i].second = 0;
      if( i > 100 && i <150){
        intensity_bins[i].second = 255;
      }
    }

    sensor_msgs::PointCloud2 point_cloud_msg_;
    // - Copy ROS header
    point_cloud_msg_.header.stamp = ros::Time::now();

    // - TODO: Wtf is height and width
    point_cloud_msg_.width = intensity_bins.size();
    point_cloud_msg_.height = 1;
    // - Fields: x, y, z, intensity
    // - Fields describe the binary blob in data
    point_cloud_msg_.fields.resize(4);
    point_cloud_msg_.fields[0].name = "x";
    point_cloud_msg_.fields[1].name = "y";
    point_cloud_msg_.fields[2].name = "z";
    point_cloud_msg_.fields[3].name = "intensity";
    // - Offset from the beginning of the point struct in bytes
    uint offset = 0;
    for (size_t i = 0; i < point_cloud_msg_.fields.size(); ++i, offset += 4) {
      point_cloud_msg_.fields[i].offset = offset;
      point_cloud_msg_.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
      point_cloud_msg_.fields[i].count = 1;
    }
    // - Offset per point of data (x, y, z, intensity)
    point_cloud_msg_.point_step = offset;
    // - length of the row TODO: is it ok?
    point_cloud_msg_.row_step = point_cloud_msg_.width;
    point_cloud_msg_.data.resize(point_cloud_msg_.point_step *
        point_cloud_msg_.row_step);
    point_cloud_msg_.is_bigendian = false;
    point_cloud_msg_.is_dense = false;
    // 135 225
    float angle = 135;
    bool ascending = true;
    point_cloud_msg_.header.frame_id = "SUB";
    while (!MustStop()) {

      // - Centered at 0 degree. 180 degree is the middle of the sonar scanline
      float delta_x = bin_distance_step *
          cos(atlas::DegToRad(angle-180.0f));
      float delta_y = bin_distance_step *
          sin(atlas::DegToRad(angle-180.0f));

      // - try with distance * cos (theta)
      float coordinate_x = 0;
      float coordinate_y = 0;
      float coordinate_z = 0;
      for (size_t i = 0; i < intensity_bins.size();
           ++i, coordinate_x += delta_x, coordinate_y += delta_y) {

        float bin_intensity = (float) (intensity_bins[i].second) / 255.0f;
        memcpy(&point_cloud_msg_.data[i * point_cloud_msg_.point_step +
                   point_cloud_msg_.fields[0].offset],
               &coordinate_x, sizeof(float));
        memcpy(&point_cloud_msg_.data[i * point_cloud_msg_.point_step +
                   point_cloud_msg_.fields[1].offset],
               &coordinate_y, sizeof(float));
        memcpy(&point_cloud_msg_.data[i * point_cloud_msg_.point_step +
                   point_cloud_msg_.fields[2].offset],
               &coordinate_z, sizeof(float));
        memcpy(&point_cloud_msg_.data[i * point_cloud_msg_.point_step +
                   point_cloud_msg_.fields[3].offset],
               &bin_intensity, sizeof(float));
      }

      if( angle >= 225)
      {
        ascending = false;
      }
      else if( angle <= 135 )
      {
        ascending = true;
      }
      if( ascending )
      {
        angle ++;
      } else{
        angle --;
      }
      publisher_.publish(point_cloud_msg_);
      ros::spinOnce();
      loop_rate.sleep();
    }
  }

 private:
  ros::Publisher publisher_;
};

TEST(BasicMapping, core_test) {

  SonarEmulator sonarEmulator;
  OdometryEmulator odometryEmulator;
  sonarEmulator.Start();
  odometryEmulator.Start();
  ros::NodeHandlePtr nh(new ros::NodeHandle(node_prefix.c_str()));
  proc_mapping::RawMap rawMap(nh);

  while(ros::ok())
  {
    ros::spinOnce();
  }

}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, node_prefix.c_str());

  return RUN_ALL_TESTS();
}
