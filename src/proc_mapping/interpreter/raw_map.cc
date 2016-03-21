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

#include "proc_mapping/interpreter/raw_map.h"
#include <opencv/cv.h>
#include <pcl/common/transforms.h>
#include <tf/transform_datatypes.h>
#include <opencv2/highgui/highgui.hpp>

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
      point_cloud_threshold_(0),
      hit_count_(0),
      tile_generator_(0),
      new_pcl_ready_(false) {
  std::string points_topic;
  std::string odometry_topic;

  nh->param<std::string>("/proc_mapping/topics/points_topic", points_topic,
                         "/sonar_node/point_cloud2");
  nh->param<std::string>("/proc_mapping/topics/odometry", odometry_topic,
                         "/proc_navigation/Odometry");

  int w, h, scanlines_per_tile;
  double r, sonar_threshold;
  nh->param<int>("/proc_mapping/map/width", w, 20);
  nh->param<int>("/proc_mapping/map/height", h, 20);
  // - MUST BE EQUAL TO SONAR's RESOLUTION
  nh->param<double>("/proc_mapping/map/resolution", r, 0.02);
  nh->param<double>("/proc_mapping/map/sonar_threshold", sonar_threshold, 0.5);
  nh->param<double>("/proc_mapping/map/sub_initial_x",
                    world_.sub.initialPosition.x, 15);
  nh->param<double>("/proc_mapping/map/sub_initial_y",
                    world_.sub.initialPosition.y, 15);
  nh->param<int>("/proc_mapping/tile/number_of_scanlines", scanlines_per_tile,
                 100);

  tile_generator_.SetScanlinePerTile(scanlines_per_tile);
  SetMapParameters(static_cast<uint32_t>(w), static_cast<uint32_t>(h), r);
  SetPointCloudThreshold(sonar_threshold, r);

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
  new_pcl_ready_ = true;
}

//------------------------------------------------------------------------------
//
void RawMap::OdomCallback(const nav_msgs::Odometry::ConstPtr &odo_in)
    ATLAS_NOEXCEPT {
  // - Generate 3x3 transformation matrix from Quaternions
  auto orientation = &odo_in.get()->pose.pose.orientation;
  tf::Quaternion q(orientation->x, orientation->y, orientation->z,
                   orientation->w);
  tf::Matrix3x3 m(q);
  // - Get YPR from transformation Matrix
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  // - Set all odometry values that will be use in the cv::mat update thread
  world_.sub.yaw = M_PI - yaw;
  world_.sub.pitch = pitch;
  world_.sub.roll = roll;
  world_.sub.position.x = odo_in.get()->pose.pose.position.x;
  world_.sub.position.y = odo_in.get()->pose.pose.position.y;
}

//------------------------------------------------------------------------------
//
void RawMap::Run() {
  while (IsRunning()) {
    if (new_pcl_ready_ and last_pcl_) {
      ProcessPointCloud(last_pcl_);
      new_pcl_ready_ = false;

      pixel_.map.copyTo(displayMap);

      PointXY<int> sub_coord = CoordinateToPixel(world_.sub.position);
      sub_coord.x =
          sub_coord
              .x /*+ static_cast<int>(world_.sub.initialPosition.x / pixel_.resolution)*/;
      sub_coord.y =
          sub_coord
              .y /*+ static_cast<int>(world_.sub.initialPosition.y / pixel_.resolution)*/;
      cv::Point sub(sub_coord.x, sub_coord.y);
      ROS_INFO("%d, %d", sub.x, sub.y);
      cv::circle(displayMap, sub, 4, CV_RGB(255, 255, 255), -1);

      cv::imshow(" ", displayMap);
      cv::waitKey(1);
      // Notify(pixel_.map);
    }
  }
}

//------------------------------------------------------------------------------
//
void RawMap::SetPointCloudThreshold(double sonar_threshold, double resolution) {
  uint32_t numberOfPoints = static_cast<uint32_t>(sonar_threshold / resolution);
  // - TODO: Remove hard coded factor of 16. This 16 is the value in
  // msg->point_step
  point_cloud_threshold_ = 16 * numberOfPoints;
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

  // - TODO: Add resolution service request from sonar. Must match sonar's
  // resolution.
  pixel_.map = cv::Mat(static_cast<int>(pixel_.width),
                       static_cast<int>(pixel_.height), CV_8UC1);
  pixel_.map.setTo(cv::Scalar(0));
  displayMap = cv::Mat(static_cast<int>(pixel_.width),
                       static_cast<int>(pixel_.height), CV_8UC1);
  displayMap.setTo(cv::Scalar(0));
  // - Keeps the number of hits for each pixels
  pixel_.number_of_hits_.resize(pixel_.width * pixel_.height, 0);
}

//------------------------------------------------------------------------------
//
void RawMap::ProcessPointCloud(const sensor_msgs::PointCloud2::ConstPtr &msg) {
  float x, y, z, intensity;
  double yaw = world_.sub.yaw;
  // --
  // Computes cosinus and sinus outside of the loop.
  double cosRotationFactor = cos(yaw);
  double sinRotationFactor = sin(yaw);

  // --
  // TODO: Add sonar service for range, resolution, min, max

  uint32_t last_bin_index =
      static_cast<uint32_t>(msg->data.size() / msg->point_step) - 1;

  for (uint32_t i = point_cloud_threshold_;
       i < msg->data.size() && i + 3 * sizeof(float) < msg->data.size();
       i += msg->point_step) {
    memcpy(&x, &msg->data[i], sizeof(float));
    memcpy(&y, &msg->data[i + sizeof(float)], sizeof(float));
    memcpy(&z, &msg->data[i + 2 * sizeof(float)], sizeof(float));
    memcpy(&intensity, &msg->data[i + 3 * sizeof(float)], sizeof(float));

    PointXY<int> bin_coordinate = CoordinateToPixel(
        Transform(x, y, cosRotationFactor, sinRotationFactor));
    UpdateMat(bin_coordinate, (static_cast<uint8_t>(255.0 * intensity)));

    // -- Tile Generator logic
    if (i == point_cloud_threshold_ || i == last_bin_index) {
      tile_generator_.UpdateTileBoundaries(bin_coordinate);
    }

    if (tile_generator_.IsTileReadyForProcess()) {
      // tile_generator_.GetTile();
    }
  }
}

}  // namespace proc_mapping
