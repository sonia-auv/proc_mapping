/**
 * \file	SonarSimulator.cc
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

#include "ros/ros.h"
#include <thread>
#include <vector>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/PointCloud2.h>
#include "lib_atlas/maths.h"
#include <eigen3/Eigen/Geometry>


class SonarEmulator {
public:
  explicit SonarEmulator(const std::string &node_name = "provider_sonar",
                         const std::string &topic_name = "point_cloud2") :
      publishing_thread_(),
      number_bin_(400),
      step_size_(0.2f),
      object_distance_(2.5f),
      object_depth_(1.0f),
      min_scanning_angle_(135),
      max_scanning_angle_(225),
      loop_rate_(10),
      stop_publishing_(false)
  {
    ros::NodeHandle n(node_name);
    publisher_ = n.advertise<sensor_msgs::PointCloud2>(topic_name, 100);
  }

  ~SonarEmulator()
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
    publishing_thread_ = std::thread(std::bind(&SonarEmulator::PublishScanlineThreadFunction, this));
  }
  inline void StopPublishing()
  {
    stop_publishing_ = true;
    publishing_thread_.join();
  }

  inline void PublishScanlineThreadFunction()
  {
    ros::Rate loop_rate(loop_rate_);

    std::vector <std::pair<float, uint8_t>> intensity_bins;
    SetIntensityBins(intensity_bins);

    sensor_msgs::PointCloud2 point_cloud_msg_;
    SetPointCloudMessage(point_cloud_msg_);
    // 135 225
    bool ascending = true;
    float angle = min_scanning_angle_;
    while (!stop_publishing_) {
      point_cloud_msg_.header.stamp = ros::Time::now();

      // - Centered at 0 degree. 180 degree is the middle of the sonar scanline
      float delta_x = step_size_ * std::cos(atlas::DegToRad( angle - 180.0f));
      float delta_y = step_size_ * std::sin(atlas::DegToRad( angle - 180.0f));

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

      if( angle >= max_scanning_angle_)
      {
        ascending = false;
      }
      else if( angle <= min_scanning_angle_ )
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

  inline void SetPointCloudMessage(sensor_msgs::PointCloud2 &point_cloud_msg){
    point_cloud_msg.width = number_bin_;
    point_cloud_msg.height = 1;
    // - Fields: x, y, z, intensity
    // - Fields describe the binary blob in data
    point_cloud_msg.fields.resize(4);
    point_cloud_msg.fields[0].name = "x";
    point_cloud_msg.fields[1].name = "y";
    point_cloud_msg.fields[2].name = "z";
    point_cloud_msg.fields[3].name = "intensity";
    // - Offset from the beginning of the point struct in bytes
    uint offset = 0;
    for (size_t i = 0; i < point_cloud_msg.fields.size(); ++i, offset += 4) {
      point_cloud_msg.fields[i].offset = offset;
      point_cloud_msg.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
      point_cloud_msg.fields[i].count = 1;
    }
    // - Offset per point of data (x, y, z, intensity)
    point_cloud_msg.point_step = offset;
    // - length of the row TODO: is it ok?
    point_cloud_msg.row_step = point_cloud_msg.width;
    point_cloud_msg.data.resize(point_cloud_msg.point_step *
                                 point_cloud_msg.row_step);
    point_cloud_msg.is_bigendian = false;
    point_cloud_msg.is_dense = false;
    point_cloud_msg.header.frame_id = "SUB";
  }

  inline void SetIntensityBins(std::vector <std::pair<float, uint8_t>> &intensity_bins) const {
    intensity_bins.resize(number_bin_);
    for (size_t i = 0; i < intensity_bins.size(); ++i) {
      intensity_bins[i].first = step_size_ * (float)(i + 1);
      intensity_bins[i].second = 0;
      if( i > 100 && i <150){
        intensity_bins[i].second = 255;
      }
    }
  }

private:

  std::thread publishing_thread_;
  int number_bin_;
  float step_size_;
  float object_distance_, object_depth_;
  int min_scanning_angle_, max_scanning_angle_;
  int loop_rate_;

  bool stop_publishing_;

  ros::Publisher publisher_;
};