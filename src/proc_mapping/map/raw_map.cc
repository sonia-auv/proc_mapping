/**
 * \file	raw_map_interpreter.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	07/02/2016
 *
 * \copyright Copyright (c) 2016 S.O.N.I.A. All rights reserved.
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

#include "raw_map.h"

namespace proc_mapping {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
RawMap::RawMap(const ros::NodeHandlePtr &nh, const CoordinateSystems::Ptr &cs)
    : atlas::Subject<cv::Mat>(),
      nh_(nh),
      points2_sub_(),
      number_of_hits_({}),
      cs_(cs),
      display_map_(),
      point_cloud_threshold_(0),
      new_pcl_ready_(false),
      last_pcl_(nullptr),
      is_map_ready_for_process_(false),
      scanlines_for_process_(0),
      scanline_counter_(0),
      is_first_scan_complete_(false) {
  std::string point_cloud_topic;
  nh->param<std::string>("/proc_mapping/topics/point_cloud2", point_cloud_topic,
                         "/provider_sonar/point_cloud2");

  double sonar_threshold = 0.;
  nh->param<double>("/proc_mapping/map/sonar_threshold", sonar_threshold, 3.0);
  nh->param<int>("/proc_mapping/tile/number_of_scanlines",
                 scanlines_for_process_, 10);

  SetPointCloudThreshold(sonar_threshold);

  display_map_ =
      cv::Mat(cs_->GetPixel().width, cs_->GetPixel().height, CV_8UC1);
  display_map_.setTo(cv::Scalar(0));

  //  Keeps the number of hits for each pixels
  number_of_hits_.resize(cs_->GetPixel().width * cs_->GetPixel().height, 0);

  points2_sub_ =
      nh_->subscribe(point_cloud_topic, 100, &RawMap::PointCloudCallback, this);
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
void RawMap::Run() {
  while (IsRunning()) {
    if (cs_->IsCoordinateSystemReady()) {
      if (new_pcl_ready_ && last_pcl_) {
        ProcessPointCloud(last_pcl_);
        new_pcl_ready_ = false;

        if (IsMapReadyForProcess()) {
          Notify(display_map_);
          is_map_ready_for_process_ = false;
        }
      }
    }
  }
}

//------------------------------------------------------------------------------
//
void RawMap::SetPointCloudThreshold(double sonar_threshold) {
  point_cloud_threshold_ =
      static_cast<uint32_t>(sonar_threshold * cs_->GetPixel().m_to_pixel);
}

//------------------------------------------------------------------------------
//
void RawMap::ProcessPointCloud(const sensor_msgs::PointCloud2::ConstPtr &msg) {
  float x = 0, y = 0, z = 0, intensity = 0;
  std::vector<uint8_t> intensity_map;
  std::vector<cv::Point2i> coordinate_map;

  uint32_t max_size = static_cast<uint32_t>(msg->data.size() / msg->point_step);
  cv::Point2i bin_coordinate;

  // Inverting the sub position value to transform the map in NED
  auto pose2d = cv::Point2d(cs_->GetSub().position.x, cs_->GetSub().position.y);
  cv::Point2d sub_position = pose2d + cs_->GetPositionOffset();

  intensity_map.resize(max_size);
  coordinate_map.resize(max_size);

  // Extract the point cloud data and store it in coordinate and intensity map
  for (uint32_t i = 0; i < max_size; i++) {
    int step = i * msg->point_step;
    memcpy(&x, &msg->data[step + msg->fields[0].offset], sizeof(float));
    memcpy(&y, &msg->data[step + msg->fields[1].offset], sizeof(float));
    memcpy(&z, &msg->data[step + msg->fields[2].offset], sizeof(float));
    memcpy(&intensity, &msg->data[step + msg->fields[3].offset], sizeof(float));

    // Create Vector3
    Eigen::Vector3d in(x, y, z), out;

    // Apply the rotation matrix to the input Vector3
    out = cs_->GetSub().rotation * in;

    // Adding the sub_position to position the point cloud in the world map.
    cv::Point2d coordinate_transformed(cv::Point2d(out.x(), out.y()) +
        sub_position);
    bin_coordinate = cs_->WorldToPixelCoordinates(coordinate_transformed);

    // Check if bin_coordinate are in the map boundary
    if ((bin_coordinate.x < cs_->GetPixel().width and bin_coordinate.x > 0) and
        (bin_coordinate.y < cs_->GetPixel().height and bin_coordinate.y > 0)) {

      uint8_t buffed_intensity = static_cast<uint8_t>(255.0f * intensity);
      if (buffed_intensity > 20) {
        buffed_intensity = 255;
      } else {
        buffed_intensity = 0;
      }

      // Filling the two maps without thresholded data
      if (i > point_cloud_threshold_) {
        intensity_map[i] = buffed_intensity;
        //        intensity_map[i] = static_cast<uint8_t>(255.0f * intensity);
        coordinate_map[i] = bin_coordinate;
      }
    }
  }

  // Update the world Mat
  for (size_t j = 0; j < intensity_map.size() - 1; j++) {
    UpdateMat(coordinate_map[j], intensity_map[j]);

    // Send a command when enough scanline is arrived
    scanline_counter_++;

    if (scanline_counter_ > 300) {
      is_first_scan_complete_ = true;
    }

    if (is_first_scan_complete_) {
      if (scanline_counter_ >= scanlines_for_process_) {
        is_map_ready_for_process_ = true;
        scanline_counter_ = 0;
      }
    }
  }
}

//------------------------------------------------------------------------------
//
void RawMap::UpdateMat(const cv::Point2i &p, const uint8_t &intensity) {
  if (p.x < cs_->GetPixel().width && p.y < cs_->GetPixel().height) {
    //  Infinite mean
    int position = p.x + p.y * cs_->GetPixel().width;
    number_of_hits_.at(static_cast<unsigned long>(position))++;
    int n = number_of_hits_.at(static_cast<unsigned long>(position)) + 1;
    display_map_.at<uint8_t>(p.y, p.x) = static_cast<uint8_t>(
        intensity / n + display_map_.at<uint8_t>(p.y, p.x) * (n - 1) / n);
//    // Simple mean
//    // Pointer to prevent multiple calls to at.
//    uint8_t *value = &(display_map_.at<uint8_t>(p.y, p.x));
//    // 16 bit to prevent overflow. Compiler will prbly optimize all that...
//    uint16_t total = *value + intensity;
//    *value = static_cast<uint8_t>(total)/2;
  }
}

//------------------------------------------------------------------------------
//
bool RawMap::IsMapReadyForProcess() { return is_map_ready_for_process_; }

//------------------------------------------------------------------------------
//
void RawMap::ResetRawMap() {
  display_map_.setTo(cv::Scalar(0));
  std::fill(number_of_hits_.begin(), number_of_hits_.end(), 0);
  scanline_counter_ = 0;
  is_first_scan_complete_ = false;
}

}  // namespace proc_mapping
