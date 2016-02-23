/**
 * \file	raw_map_interpreter.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	07/02/2016
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

#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/common/transforms.h>
#include <tf/transform_datatypes.h>
#include "proc_mapping/interpreter/raw_map.h"

namespace proc_mapping {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
RawMap::RawMap(const ros::NodeHandlePtr &nh) ATLAS_NOEXCEPT
    : atlas::Subject<cv::Mat>(),
      nh_(nh),
      points2_sub_(),
      odom_sub_(),
      sonar_threshold_(32),
      hit_count_(0) {
  std::string points_topic;
  std::string odometry_topic;

  nh->param<std::string>("/proc_mapping/topics/points_topic", points_topic,
                         "/Scanline_parser/point_cloud2");
  nh->param<std::string>("/proc_mapping/topics/odometry", odometry_topic,
                         "/proc_navigation/Odometry");

  int w, h;
  double r;
  nh->param<int>("/proc_mapping/map/width", w, 20);
  nh->param<int>("/proc_mapping/map/height", h, 20);
  // - MUST BE EQUAL TO SONAR's RESOLUTION
  nh->param<double>("/proc_mapping/map/resolution", r, 0.02);
  SetMapParameters(static_cast<uint32_t>(w), static_cast<uint32_t>(h), r);

  points2_sub_ =
      nh_->subscribe(points_topic, 100, &RawMap::PointCloudCallback, this);
  odom_sub_ = nh_->subscribe(odometry_topic, 100, &RawMap::OdomCallback, this);

  Start();
}

//------------------------------------------------------------------------------
//
RawMap::~RawMap() ATLAS_NOEXCEPT {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void RawMap::PointCloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr &msg_in) ATLAS_NOEXCEPT {
  last_pcl_ = msg_in;
  ++hit_count_;
  if (hit_count_ >= sonar_threshold_) {
    new_pcl_ready_ = true;
  }
}

//------------------------------------------------------------------------------
//
void RawMap::OdomCallback(const nav_msgs::Odometry::ConstPtr &odo_in)
    ATLAS_NOEXCEPT {
  last_odom_ = odo_in;
  if (!first_odom_received_) {
    world_.x_0 = last_odom_->pose.pose.position.x;
    world_.y_0 = last_odom_->pose.pose.position.y;
  }
  first_odom_received_ = true;
}

//------------------------------------------------------------------------------
//
void RawMap::Run() {
  while (IsRunning()) {
    if (new_pcl_ready_ && first_odom_received_) {
      // See http://pointclouds.org/documentation/tutorials/matrix_transform.php
      // for more details on the transformation algo.
      Eigen::Affine3f transform = Eigen::Affine3f::Identity();

      // Applying odometry translation to the transformation
      transform.translation() << last_odom_->pose.pose.position.x,
          last_odom_->pose.pose.position.y, 0.0;
      // We want to center the submarine, then substract the initial position to
      // the current one.
      transform.translation() << -world_.x_0, -world_.y_0, 0.0;
      // Finally, let us center the origin of the coordinate system to the
      // center of the map.
      transform.translation() << world_.width / 2, world_.height / 2, 0.0;

      // The same rotation matrix as before; tetha radians arround Z axis
      transform.rotate(Eigen::AngleAxisf(
          static_cast<float>(last_odom_->pose.pose.orientation.z),
          Eigen::Vector3f::UnitZ()));

      ProcessPointCloud(last_pcl_, transform);
      new_pcl_ready_ = false;
      Notify(pixel_.map);
    }
  }
}

//------------------------------------------------------------------------------
//
void RawMap::SetMapParameters(const size_t &w, const size_t &h,
                              const double &r) ATLAS_NOEXCEPT {
  world_.width = w;
  world_.height = h;

  pixel_.width = static_cast<uint32_t>(w / r);
  pixel_.height = static_cast<uint32_t>(h / r);
  pixel_.resolution = r;

  // - TODO: Add resolution service request from sonar. Must match sonar's resolution.
  cv::Size size(static_cast<uint32_t>(pixel_.width),
                static_cast<uint32_t>(pixel_.height));
  pixel_.map = cv::Mat(static_cast<int>(pixel_.width),
                       static_cast<int>(pixel_.height), CV_8UC1);
  pixel_.map.setTo(cv::Scalar(0));
}

//------------------------------------------------------------------------------
//
void RawMap::ProcessPointCloud(const sensor_msgs::PointCloud2::ConstPtr &msg,
                               const Eigen::Affine3f &t) {
  float x, y, z, intensity;
  for (unsigned int i = sonar_threshold_;
       i < msg->data.size() && i + 3 * sizeof(float) < msg->data.size();
       i += msg->point_step) {
    memcpy(&x, &msg->data[i], sizeof(float));
    memcpy(&y, &msg->data[i + sizeof(float)], sizeof(float));
    memcpy(&z, &msg->data[i + 2 * sizeof(float)], sizeof(float));
    memcpy(&intensity, &msg->data[i + 3 * sizeof(float)], sizeof(float));

    Eigen::Vector3f p_wcs(x, y, 0);
    Eigen::Vector3f p_wcs_transformed = t * p_wcs;
    Eigen::Vector3i p_pcs(
        static_cast<int>(p_wcs_transformed(0) / pixel_.resolution),
        static_cast<int>(p_wcs_transformed(1) / pixel_.resolution), 0);

    UpdateMat(p_pcs(0), p_pcs(1), (static_cast<uint8_t>(255.0 * intensity)));
    
  }
}

inline void RawMap::UpdateMat(int x, int y, uchar intensity){
  pixel_.map.at<uchar>(x, y) = static_cast<uchar>((intensity + pixel_.map.at<uchar>(x, y)) / 2);
}

}  // namespace proc_mapping
