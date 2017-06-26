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
#include <proc_mapping/proc_mapping_node.h>
#include "proc_mapping/sonar/SonarMapper.h"

static const std::string node_prefix("proc_mapping");

class OdometryEmulator : public atlas::Runnable {
 public:
  explicit OdometryEmulator() {
    ros::NodeHandle n("proc_navigation");
    publisher_ = n.advertise<nav_msgs::Odometry>("odom", 100);
  }

  void Run() override
  {
    ros::Rate loop_rate(100);

    double yaw = 3*M_PI/2, pitch = M_PI/8, roll = M_PI/4;
    float x = 0.0f, y = 0.0f, z = 0.0f;

    while (!MustStop()) {
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
      nav_msgs::Odometry msg;
      msg.pose.pose.orientation = quaternion;
      msg.pose.pose.position.x = x;
      msg.pose.pose.position.y = y;
      msg.pose.pose.position.z = z;

      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = "NED";
      publisher_.publish(msg);
      ros::spinOnce();
      loop_rate.sleep();

      // Increasing the values to see the changes.
      yaw += 0.1;
      pitch += 0.1;
      roll += 0.1;
      x -= 0.01;
      y += 0.01;
      z += 0.01;
    }
  }

 private:
  ros::Publisher publisher_;
};

class SonarEmulator : public atlas::Runnable  {
 public:
  explicit SonarEmulator() {
    ros::NodeHandle n("provider_sonar");
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
  proc_mapping::ProcMappingNode node(nh);

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

/**
 * \file	raw_map_interpreter.h
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

#ifndef PROC_MAPPING_RAW_MAP_H_
#define PROC_MAPPING_RAW_MAP_H_

#include <lib_atlas/macros.h>
#include <lib_atlas/ros/image_publisher.h>
#include <opencv/cv.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <proc_mapping/general/async_image_publisher.h>
#include <sensor_msgs/PointCloud2.h>
#include <array>
#include <eigen3/Eigen/Geometry>
#include <memory>
#include <mutex>
#include <vector>
#include "coordinate_systems.h"

namespace proc_mapping {

class RawMap {
 public:
  const int NB_PIXEL_BY_METER = 10;
  //==========================================================================
  // P U B L I C   C / D T O R S
  explicit RawMap(const ros::NodeHandlePtr &nh,
                  const CoordinateSystems::Ptr &cs);

  virtual ~RawMap() = default;

  //==========================================================================
  // P U B L I C   M E T H O D S
  void ResetRawMap();

 private:
  //==========================================================================
  // P R I V A T E   M E T H O D S

  void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg_in);

  /**
   * Set the point cloud threshold to subtract the front sonar noise.
   *
   * \param sonar_threshold The sonar threshold in meters
   */
  void SetPointCloudThreshold(double sonar_threshold);

  /**
   * Received and treat point cloud data and update the world map
   *
   * \param msg The point cloud message from the provider_sonar
   */
  void ProcessPointCloud(const sensor_msgs::PointCloud2::ConstPtr &msg);

  /**
   * Update world map and apllying an infinite mean.
   *
   * \param p The position x,y in the map
   * \param intensity The intensity of the point
   */
  void UpdateMat(const cv::Point2i &p, const uint8_t &intensity);

  bool IsMapReadyForProcess();

  //==========================================================================
  // P R I V A T E   M E M B E R S

  ros::NodeHandlePtr nh_;
  ros::Subscriber points2_sub_;

  ///  Number of Hits per pixel
  std::vector<uint8_t> number_of_hits_;

  CoordinateSystems::Ptr cs_;
  cv::Mat display_map_;
  AsyncImagePublisher image_publisher_;
  std::mutex map_mutex_;
  /// The first data of the sonar may be scrap. Keeping a threshold and starting
  /// to process data after it
  uint32_t point_cloud_threshold_;

  bool new_pcl_ready_;

  sensor_msgs::PointCloud2::ConstPtr last_pcl_;

  /// As we don't want to process the map every scanline, we set a counter of
  /// scan line and increment it. When the scanline_counter hits the wanted
  /// value, we set the map to be ready to process and it is sent to the map
  /// interpreter for processing.
  bool is_map_ready_for_process_;
  int scanlines_for_process_;
  int scanline_counter_;

  /// Flag that states if the first odometry message has already been received.
  /// It is used to calculate the initial position of the submarine.
  bool is_first_scan_complete_;
};

}  // namespace proc_mapping

#endif  // PROC_MAPPING_RAW_MAP_H_


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

#include "SonarMapper.h"
#include <mutex>
#include "proc_mapping/config.h"

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
      image_publisher_(kRosNodeName + "raw_map"),
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
    ros::Time previous = ros::Time::now();
    if (cs_->IsCoordinateSystemReady()) {
      ros::Time now = ros::Time::now();
      ros::Duration desired(1, 0);
      if ((now - previous) < desired) {
        if (new_pcl_ready_ && last_pcl_) {
          ProcessPointCloud(last_pcl_);
          new_pcl_ready_ = false;
        }
        previous = ros::Time::now();
      } else {
        ROS_INFO("The new pcl take too much time. Connection seem lost.");
        previous = ros::Time::now();
      }
      if (IsMapReadyForProcess()) {
        Notify(display_map_);
        // To fit in OpenCv coordinate system, we have to made a rotation of
        // 90 degrees on the display map
        std::lock_guard<std::mutex> guard(map_mutex_);
        cv::Point2f src_center(display_map_.cols / 2.0f,
                               display_map_.rows / 2.0f);
        cv::Mat rot_mat = getRotationMatrix2D(src_center, 90, 1.0);
        cv::Mat dst;
        cv::warpAffine(display_map_, dst, rot_mat, display_map_.size());
        cvtColor(dst, dst, CV_GRAY2RGB);
        image_publisher_.Publish(dst);

        is_map_ready_for_process_ = false;
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
  }
  std::lock_guard<std::mutex> guard(map_mutex_);
  // Send a command when enough scanline is arrived
  scanline_counter_++;

  if (scanline_counter_ > 200) {
    is_first_scan_complete_ = true;
    ROS_INFO("First Scan Complete");
  }

  if (is_first_scan_complete_) {
    if (scanline_counter_ >= scanlines_for_process_) {
      is_map_ready_for_process_ = true;
      scanline_counter_ = 0;
    }
  }
}

//------------------------------------------------------------------------------
//
void RawMap::UpdateMat(const cv::Point2i &p, const uint8_t &intensity) {
  std::lock_guard<std::mutex> guard(map_mutex_);
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
    //    // 16 bit to prevent overflow. Compiler will prbly optimize all
    //    that...
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
  ROS_INFO("Resetting the raw map");
  std::lock_guard<std::mutex> guard(map_mutex_);
  display_map_.setTo(cv::Scalar(0));
  std::fill(number_of_hits_.begin(), number_of_hits_.end(), 0);
  scanline_counter_ = 0;
  is_first_scan_complete_ = false;
}

}  // namespace proc_mapping


/*