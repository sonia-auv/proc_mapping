/**
 * \file	OdometrySimulator.cc
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
#include "nav_msgs/Odometry.h"
#include "lib_atlas/maths.h"
#include <eigen3/Eigen/Geometry>


class OdometryEmulator {
public:
  explicit OdometryEmulator(const std::string &node_name = "proc_navigation",
                            const std::string &topic_name = "odom"):
      x_(0.0f), y_(0.0f), z_(0.0f),
      yaw_(0.0f), pitch_(0.0f), roll_(0.0f)
  {
    ros::NodeHandle n(node_name);
    publisher_ = n.advertise<nav_msgs::Odometry>(topic_name, 100);
    SetPosition(0,0,0,0,0,0);
  }

  ~OdometryEmulator()
  {
    if( ! stop_publishing_ )
    {
      StopPublishing();
    }
    publisher_.shutdown();
  }

  inline void StartPublishing()
  {
    stop_publishing_ = true;
    publishing_thread_ = std::thread(std::bind(&OdometryEmulator::PublishScanlineThreadFunction, this));
  }
  inline void StopPublishing()
  {
    stop_publishing_ = true;
    publishing_thread_.join();
  }
  inline void
  SetPosition(float x, float y, float z, float roll, float pitch, float yaw)
  {
    x_ = x;
    y_ = y;
    z_ = z;
    roll_ = roll;
    pitch_ = pitch;
    yaw_ = yaw;
    Eigen::AngleAxisd rollAngle(roll_, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch_, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw_, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    q.normalize();

    msg_.pose.pose.orientation.w = q.w();
    msg_.pose.pose.orientation.x = q.x();
    msg_.pose.pose.orientation.y = q.y();
    msg_.pose.pose.orientation.z = q.z();
    msg_.pose.pose.position.x = x_;
    msg_.pose.pose.position.y = y_;
    msg_.pose.pose.position.z = z_;
    msg_.twist.twist.linear.x = x_;
    msg_.twist.twist.linear.y = y_;
    msg_.twist.twist.linear.z = z_;
    msg_.twist.twist.angular.x = roll_;
    msg_.twist.twist.angular.y = pitch_;
    msg_.twist.twist.angular.z = yaw_;
  }
  inline void PublishScanlineThreadFunction() {
    ros::Rate loop_rate(100);
    msg_.header.frame_id = "NED";
    while (!stop_publishing_) {
      msg_.header.stamp = ros::Time::now();
      publisher_.publish(msg_);
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
private:
  ros::Publisher publisher_;
  std::thread publishing_thread_;
  bool stop_publishing_;
  float x_, y_, z_, yaw_, pitch_, roll_;
  nav_msgs::Odometry msg_;

};
