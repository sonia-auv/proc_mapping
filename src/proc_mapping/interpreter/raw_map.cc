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

  nh->param<std::string>("topics/points_topic", points_topic,
                         "/Scanline_parser/point_cloud2");
  nh->param<std::string>("topics/odometry", odometry_topic,
                         "/proc_navigation/Odometry");

  int w, h;
  double r;
  nh->param<int>("map/width", w, 30);
  nh->param<int>("map/height", h, 30);
  nh->param<double>("map/resolution", r, 0.125);
  SetMapParameters(static_cast<uint32_t>(w), static_cast<uint32_t>(h), r);

  points2_sub_ = nh_->subscribe(points_topic, 100, &RawMap::PointCloudCallback, this);
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
  if(hit_count_ >= sonar_threshold_) {
    new_pcl_ready_ = true;
  }
}

void RawMap::OdomCallback(const nav_msgs::Odometry::ConstPtr &odo_in)
    ATLAS_NOEXCEPT {
  last_odom_ = odo_in;
  first_odom_received_ = true;
}

void RawMap::Run() {
  while (IsRunning()) {
    if (new_pcl_ready_ && first_odom_received_) {
      // See http://pointclouds.org/documentation/tutorials/matrix_transform.php
      // for more details on the transformation algo.
      Eigen::Affine3f transform = Eigen::Affine3f::Identity();

      // Define a translation of 2.5 meters on the x axis.
      transform.translation() << 2.5, 0.0, 0.0;
      // The same rotation matrix as before; tetha radians arround Z axis
      transform.rotate(Eigen::AngleAxisf(
          static_cast<float>(last_odom_->pose.pose.orientation.z),
          Eigen::Vector3f::UnitZ()));

      // Executing the transformation
      pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
      pcl::PointCloud<pcl::PointXYZ> source_cloud;
      pcl::fromROSMsg(*last_pcl_, source_cloud);
      // You can either apply transform_1 or transform_2; they are the same
      pcl::transformPointCloud(source_cloud, transformed_cloud, transform);

      // Adding the transformed cloud to the original point cloud.
      world_.cloud += transformed_cloud;
      ConvertWorldToPixelCCS(world_, pixel_);
      new_pcl_ready_ = false;
      Notify(pixel_.map);
    }
  }
}

//------------------------------------------------------------------------------
//
void RawMap::SetMapParameters(const size_t &w, const size_t &h, const double &r) ATLAS_NOEXCEPT {
  world_.width = w;
  world_.height = h;

  pixel_.width = static_cast<uint32_t>(w/r);
  pixel_.height = static_cast<uint32_t>(h/r);
  pixel_.resolution = r;

  cv::Size size(static_cast<uint32_t>(pixel_.width), static_cast<uint32_t>(pixel_.height));
  pixel_.map = cv::Mat(pixel_.width, pixel_.height, CV_8UC1);
  pixel_.map.setTo(cv::Scalar(0));
}

//------------------------------------------------------------------------------
//
void RawMap::ConvertWorldToPixelCCS(const WorldCCS &world, PixelCCS &pixel) ATLAS_NOEXCEPT {
  float x, y, z, intensity;
  for(unsigned int i = 0; i < world.cloud.size() && i + 3 * sizeof(float) < world.cloud.size(); i+=last_pcl_->point_step){
    memcpy(&x, &world.cloud[i ], sizeof(float));
    memcpy(&y, &world.cloud[i + sizeof(float)], sizeof(float));
    memcpy(&z, &world.cloud[i + 2*sizeof(float)], sizeof(float));
    memcpy(&intensity, &world.cloud[i + 3*sizeof(float)], sizeof(float));

    int u = static_cast<int>(x/pixel_.resolution);
    int v = static_cast<int>(y/pixel_.resolution);

    pixel_.map.at<uchar>(u, v) =
        static_cast<uchar>((static_cast<uint8_t >(255.0 * intensity) +
                pixel_.map.at<uchar>(u, v))/2);
  }
}

}  // namespace proc_mapping
