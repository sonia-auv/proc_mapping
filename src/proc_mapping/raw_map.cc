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

#include "proc_mapping/raw_map.h"
#include <opencv/cv.h>
#include <pcl/common/transforms.h>
#include <opencv2/highgui/highgui.hpp>

namespace proc_mapping {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
RawMap::RawMap(const ros::NodeHandlePtr &nh)
    : atlas::Subject<cv::Mat>(),
      nh_(nh),
      points2_sub_(),
      odom_sub_(),
      point_cloud_threshold_(0),
      hit_count_(0),
      new_pcl_ready_(false),
      last_pcl_(nullptr),
      scanlines_per_tile_(0),
      is_map_ready_for_process_(false) {
  std::string point_cloud_topic;
  std::string odometry_topic;

  nh->param<std::string>("/proc_mapping/topics/point_cloud2", point_cloud_topic,
                         "/provider_sonar/point_cloud2");
  nh->param<std::string>("/proc_mapping/topics/odometry", odometry_topic,
                         "/proc_navigation/odometry");

  int n_bin, w, h;
  float range;
  double r, sonar_threshold;
  nh->param<int>("/provider_sonar/sonar/n_bins_", n_bin, 400);
  nh->param<float>("/provider_sonar/sonar/range_", range, 10.0);

  nh->param<int>("/proc_mapping/map/width", w, 30);
  nh->param<int>("/proc_mapping/map/height", h, 30);
  nh->param<double>("/proc_mapping/map/sonar_threshold", sonar_threshold, 1.0);
  nh->param<double>("/proc_mapping/map/sub_initial_x", sub_.initial_position.x,
                    15.0);
  nh->param<double>("/proc_mapping/map/sub_initial_y", sub_.initial_position.y,
                    15.0);
  nh->param<int>("/proc_mapping/tile/number_of_scanlines", scanlines_per_tile_,
                 1);

  // Resolution is equal to the range of the sonar divide by the number of bin
  // of a scanline.
  r = range / n_bin;

  SetMapParameters(static_cast<uint32_t>(w), static_cast<uint32_t>(h), r);
  SetPointCloudThreshold(sonar_threshold, r);

  points2_sub_ =
      nh_->subscribe(point_cloud_topic, 100, &RawMap::PointCloudCallback, this);
  odom_sub_ = nh_->subscribe(odometry_topic, 100, &RawMap::OdomCallback, this);
  Start();
}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void RawMap::PointCloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr &msg_in) {
  last_pcl_ = msg_in;
  new_pcl_ready_ = true;
}

//------------------------------------------------------------------------------
//
void RawMap::OdomCallback(const nav_msgs::Odometry::ConstPtr &odo_in) {
  // - Generate 3x3 transformation matrix from Quaternions
  auto orientation = &odo_in.get()->pose.pose.orientation;
  Eigen::Quaterniond quaterniond(orientation->w, orientation->x, orientation->y,
                                 orientation->z);
  Eigen::Matrix3d rotation;
  rotation = quaterniond.toRotationMatrix();
  // - Set all odometry values that will be use in the cv::mat update thread
  Eigen::Vector3d euler_vec = rotation.eulerAngles(0, 1, 2);
  double roll = euler_vec.x();
  double pitch = euler_vec.y();
  double yaw = euler_vec.z();

  sub_.rotation = rotation;
  sub_.yaw = yaw;
  sub_.pitch = pitch;
  sub_.roll = roll;
  sub_.position.x = odo_in.get()->pose.pose.position.x;
  sub_.position.y = odo_in.get()->pose.pose.position.y;
}

//------------------------------------------------------------------------------
//
void RawMap::Run() {
  cv::Point2d initial_position_offset =
      sub_.initial_position * pixel_.m_to_pixel;

  while (IsRunning()) {
    if (new_pcl_ready_ && last_pcl_) {
      ProcessPointCloud(last_pcl_);
      new_pcl_ready_ = false;

      cv::Point2d sub_coord = CoordinateToPixel(sub_.position);
      sub_coord += (sub_.initial_position * pixel_.m_to_pixel);

      if (IsMapReadyForProcess()) {
        Notify(pixel_.map);
        is_map_ready_for_process_ = false;
      }
    }
  }
}

//------------------------------------------------------------------------------
//
void RawMap::SetPointCloudThreshold(double sonar_threshold, double resolution) {
  uint32_t numberOfPoints = static_cast<uint32_t>(sonar_threshold / resolution);
  point_cloud_threshold_ = numberOfPoints;
}

//------------------------------------------------------------------------------
//
void RawMap::SetMapParameters(const size_t &w, const size_t &h,
                              const double &r) {
  world_.width = w;
  world_.height = h;

  pixel_.width = static_cast<uint32_t>(w / r);
  pixel_.height = static_cast<uint32_t>(h / r);
  pixel_.pixel_to_m = r;
  pixel_.m_to_pixel = 1 / r;

  pixel_.map = cv::Mat(pixel_.width, pixel_.height, CV_8UC1);
  pixel_.map.setTo(cv::Scalar(0));
  pixel_.map_color = cv::Mat(pixel_.width, pixel_.height, CV_8UC3);
  pixel_.map_color.setTo(cv::Scalar(0));
  displayMap = cv::Mat(pixel_.width, pixel_.height, CV_8UC1);
  displayMap.setTo(cv::Scalar(0));
  std::cout << "Map meter info: Width=" << w << " Height=" << h << std::endl;
  std::cout << "Map pixel info: Width=" << pixel_.width
            << " Height= " << pixel_.height << std::endl;
  std::cout << "Resolution in meter: " << r << std::endl;

  // - Keeps the number of hits for each pixels
  pixel_.number_of_hits_.resize(pixel_.width * pixel_.height, 0);
}

//------------------------------------------------------------------------------
//
void RawMap::ProcessPointCloud(const sensor_msgs::PointCloud2::ConstPtr &msg) {
  float x = 0, y = 0, z = 0, intensity = 0;
  std::vector<uint8_t> intensity_map;
  std::vector<cv::Point2i> coordinate_map;

  uint32_t last_bin_index =
      static_cast<uint32_t>(msg->data.size() / msg->point_step) - 1;
  uint32_t i = 0,
           max_size = static_cast<uint32_t>(msg->data.size() / msg->point_step);
  cv::Point2i bin_coordinate;
  cv::Point2f offset_sub = sub_.position + sub_.initial_position;

  intensity_map.resize(max_size - point_cloud_threshold_);
  coordinate_map.resize(max_size - point_cloud_threshold_);

  uint8_t number_of_bin = 0;

  for (i = 0; i < max_size; i++) {
    int step = i * msg->point_step;
    memcpy(&x, &msg->data[step + msg->fields[0].offset], sizeof(float));
    memcpy(&y, &msg->data[step + msg->fields[1].offset], sizeof(float));
    memcpy(&z, &msg->data[step + msg->fields[2].offset], sizeof(float));
    memcpy(&intensity, &msg->data[i * msg->point_step + msg->fields[3].offset],
           sizeof(float));

    Eigen::Vector3d in(x, y, z), out;
    out = sub_.rotation * in;

    cv::Point2f coordinate_transformed =
        cv::Point2f(in.x(), in.y()) + offset_sub;

    bin_coordinate = CoordinateToPixel(coordinate_transformed);

    uint8_t threat_intensity = static_cast<uint8_t>(255.0f * intensity);
    if (threat_intensity > 5) {
      threat_intensity += 200;
      number_of_bin++;
    }

    if (i > point_cloud_threshold_) {
      if (number_of_bin < 15) {
        intensity_map[i - point_cloud_threshold_] = threat_intensity;
      } else {
        intensity_map[i - point_cloud_threshold_] = 0;
      }
      coordinate_map[i - point_cloud_threshold_] = bin_coordinate;
    }
  }

  for (int i = 0; i < intensity_map.size(); i++) {
    UpdateMat(coordinate_map[i], intensity_map[i]);
  }

  scanline_counter_++;
  if (scanline_counter_ >= scanlines_per_tile_) {
    is_map_ready_for_process_ = true;
    scanline_counter_ = 0;
  }
}

//------------------------------------------------------------------------------
//
void RawMap::UpdateMat(const cv::Point2d &p, const uint8_t &intensity) {
  if (static_cast<int>(p.x) < pixel_.width &&
      static_cast<int>(p.y) < pixel_.height) {
    // - Infinite mean
    pixel_.number_of_hits_.at((unsigned long)(p.x + p.y * pixel_.width))++;
    int n =
        pixel_.number_of_hits_.at((unsigned long)(p.x + p.y * pixel_.width)) +
        1;
    pixel_.map.at<uint8_t>((int)p.y, (int)p.x) = static_cast<uint8_t>(
        intensity / n +
        pixel_.map.at<uint8_t>((int)p.y, (int)p.x) * (n - 1) / n);
  }
}

//------------------------------------------------------------------------------
//
cv::Point2d RawMap::CoordinateToPixel(const cv::Point2d &p) {
  return p * pixel_.m_to_pixel;
}

//------------------------------------------------------------------------------
//
bool RawMap::IsMapReadyForProcess() { return is_map_ready_for_process_; }

}  // namespace proc_mapping
