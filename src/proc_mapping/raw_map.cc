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
      new_pcl_ready_(false),
      last_pcl_(nullptr),
      is_map_ready_for_process_(false),
      scanlines_for_process_(0),
      scanline_counter_(0),
      is_first_odom_(true) {
  std::string point_cloud_topic;
  std::string odometry_topic;

  nh->param<std::string>("/proc_mapping/topics/point_cloud2", point_cloud_topic,
                         "/provider_sonar/point_cloud2");
  nh->param<std::string>("/proc_mapping/topics/odometry", odometry_topic,
                         "/proc_navigation/odom");

  int n_bin = 0;
  int w = 0;
  int h = 0;
  double range = 0.;
  double sonar_threshold = 0.;
  nh->param<int>("/provider_sonar/sonar/n_bins_", n_bin, 400);
  nh->param<double>("/provider_sonar/sonar/range_", range, 10.0);

  nh->param<int>("/proc_mapping/map/width", w, 30);
  nh->param<int>("/proc_mapping/map/height", h, 30);
  nh->param<double>("/proc_mapping/map/sonar_threshold", sonar_threshold, 1.0);
  nh->param<int>("/proc_mapping/tile/number_of_scanlines",
                 scanlines_for_process_, 10);

  // Resolution is equal to the range of the sonar divide by the number of bin
  // of a scanline.
  double r = range / n_bin;

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
  if (is_first_odom_) {
    sub_.initial_position.x = odo_in.get()->pose.pose.position.x;
    sub_.initial_position.y = odo_in.get()->pose.pose.position.y;
    is_first_odom_ = false;
  }

  //  Generate 3x3 transformation matrix from Quaternions
  auto orientation = &odo_in.get()->pose.pose.orientation;
  Eigen::Quaterniond quaterniond(orientation->w, orientation->x, orientation->y,
                                 orientation->z);
  //  The quaternion is required to be normalized, otherwise the result is
  //  undefined.
  quaterniond.normalized();

  Eigen::Matrix3d rotation;
  rotation = quaterniond.toRotationMatrix();
  //  Set all odometry values that will be use in the cv::mat update thread
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
  while (IsRunning()) {
    if (is_first_odom_ == false) {
      if (new_pcl_ready_ && last_pcl_) {
        ProcessPointCloud(last_pcl_);
        new_pcl_ready_ = false;

        if (IsMapReadyForProcess()) {
          Notify(pixel_.map);
          is_map_ready_for_process_ = false;
        }
      }
    }
  }
}

//------------------------------------------------------------------------------
//
void RawMap::SetPointCloudThreshold(double sonar_threshold, double resolution) {
  point_cloud_threshold_ = static_cast<uint32_t>(sonar_threshold / resolution);
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
  std::cout << "Map meter info: Width=" << w << " Height=" << h << std::endl;
  std::cout << "Map pixel info: Width=" << pixel_.width
            << " Height= " << pixel_.height << std::endl;
  std::cout << "Resolution in meter: " << r << std::endl;

  //  Keeps the number of hits for each pixels
  pixel_.number_of_hits_.resize(pixel_.width * pixel_.height, 0);
}

//------------------------------------------------------------------------------
//
void RawMap::ProcessPointCloud(const sensor_msgs::PointCloud2::ConstPtr &msg) {
  bool debug = false;
  float x = 0, y = 0, z = 0, intensity = 0;
  std::vector<uint8_t> intensity_map;
  std::vector<cv::Point2i> coordinate_map;

  uint32_t i = 0,
           max_size = static_cast<uint32_t>(msg->data.size() / msg->point_step);
  cv::Point2i bin_coordinate;

  double scanline_length = 10;

  cv::Point2d heading(cos(sub_.yaw) * scanline_length, sin(sub_.yaw) * scanline_length);

  heading += GetPositionOffset();

  auto sub_position =
      sub_.position - sub_.initial_position + GetPositionOffset();

  heading = CoordinateToPixel(heading);

  cv::Point2d left_limit(cos(sub_.yaw - 0.785398) * scanline_length, sin(sub_.yaw - 0.785398) * scanline_length);
  cv::Point2d rigth_limit(cos(sub_.yaw + 0.785398) * scanline_length, sin(sub_.yaw + 0.785398) * scanline_length);

  left_limit += GetPositionOffset();
  rigth_limit += GetPositionOffset();

  left_limit = CoordinateToPixel(left_limit);
  rigth_limit = CoordinateToPixel(rigth_limit);

  intensity_map.resize(max_size);
  coordinate_map.resize(max_size);

  for (i = 0; i < max_size; i++) {
    int step = i * msg->point_step;
    memcpy(&x, &msg->data[step + msg->fields[0].offset], sizeof(float));
    memcpy(&y, &msg->data[step + msg->fields[1].offset], sizeof(float));
    memcpy(&z, &msg->data[step + msg->fields[2].offset], sizeof(float));
    memcpy(&intensity, &msg->data[step + msg->fields[3].offset], sizeof(float));

    Eigen::Vector3d in(x, -1 * y, z), out;

    out = sub_.rotation * in;
    cv::Point2d out_point(out.x(),out.y());
    cv::Point2d coordinate_transformed(out_point + sub_position);
    bin_coordinate = CoordinateToPixel(coordinate_transformed);

    bin_coordinate.y = (pixel_.width/2) - bin_coordinate.y + (pixel_.width/2);

    uint8_t threat_intensity = static_cast<uint8_t>(255.0f * intensity);

    if (i > point_cloud_threshold_) {
      intensity_map[i] = threat_intensity;
      coordinate_map[i] = bin_coordinate;
    }
  }

  for (size_t j = 0; j < intensity_map.size() - 1; j++) {
    UpdateMat(coordinate_map[j], intensity_map[j]);
  }

  if (debug) {
  cv::Mat sub_heading(pixel_.width, pixel_.height, CV_8UC1);
  cv::line(pixel_.map, cv::Point2d(pixel_.width/2, pixel_.height/2), cv::Point2d(pixel_.width/2, 0), cv::Scalar::all(255));
  cv::line(pixel_.map, cv::Point2d(pixel_.width/2, pixel_.height/2), cv::Point2d(pixel_.height, pixel_.height/2), cv::Scalar::all(255));

  cv::Point2d sub = CoordinateToPixel(sub_position);
  sub.y = (pixel_.width/2) - sub.y + (pixel_.width/2);

  cv::circle(sub_heading, sub, 2, cv::Scalar::all(255), -1);
  cv::circle(pixel_.map, sub, 2, cv::Scalar::all(255), -1);
  cv::line(sub_heading, sub, heading, cv::Scalar::all(255));
  cv::line(sub_heading, sub, left_limit, cv::Scalar::all(100));
  cv::line(sub_heading, sub, rigth_limit, cv::Scalar::all(100));
  cv::imshow("heading", sub_heading);
  }

  scanline_counter_++;
  if (scanline_counter_ >= scanlines_for_process_) {
    is_map_ready_for_process_ = true;
    scanline_counter_ = 0;
  }
}

//------------------------------------------------------------------------------
//
void RawMap::UpdateMat(const cv::Point2i &p, const uint8_t &intensity) {
  if (p.x < pixel_.width && p.y < pixel_.height) {
    // - Infinite mean
    int position = p.x + p.y * pixel_.width;
    pixel_.number_of_hits_.at(static_cast<unsigned long>(position))++;
    int n = pixel_.number_of_hits_.at(static_cast<unsigned long>(position)) + 1;
    pixel_.map.at<uint8_t>(p.y, p.x) = static_cast<uint8_t>(intensity
        / n + pixel_.map.at<uint8_t>(p.y, p.x) * (n - 1) / n);
  }
}

//------------------------------------------------------------------------------
//
cv::Point2i RawMap::CoordinateToPixel(const cv::Point2d &p) {
  return p * pixel_.m_to_pixel;
}

//------------------------------------------------------------------------------
//
bool RawMap::IsMapReadyForProcess() { return is_map_ready_for_process_; }

//------------------------------------------------------------------------------
//
cv::Point2d RawMap::GetPositionOffset() const {
  return {world_.width / 2, world_.height / 2};
}

}  // namespace proc_mapping
