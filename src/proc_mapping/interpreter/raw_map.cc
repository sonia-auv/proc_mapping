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
      new_pcl_ready_(false),
      last_pcl_(nullptr){
  std::string points_topic;
  std::string odometry_topic;

  nh->param<std::string>("/proc_mapping/topics/points_topic", points_topic,
                         "/provider_sonar/point_cloud2");
  nh->param<std::string>("/proc_mapping/topics/odometry", odometry_topic,
                         "/proc_navigation/odom");

  int n_bin, w, h, scanlines_per_tile;
  float range;
  double r, sonar_threshold;
  nh->param<int>("/provider_sonar/sonar/n_bins_", n_bin, 400);
  nh->param<float>("/provider_sonar/sonar/range_", range, 8.0);

  nh->param<int>("/proc_mapping/map/width", w, 20);
  nh->param<int>("/proc_mapping/map/height", h, 20);
  nh->param<double>("/proc_mapping/map/sonar_threshold", sonar_threshold, 0.5);
  nh->param<float>("/proc_mapping/map/sub_initial_x",
                    world_.sub.initialPosition.x, 10);
  nh->param<float>("/proc_mapping/map/sub_initial_y",
                    world_.sub.initialPosition.y, 5);
  nh->param<int>("/proc_mapping/tile/number_of_scanlines", scanlines_per_tile,
                 100);

  // Resolution is equal to the range of the sonar divide by the number of bin
  // of a scanline.
  r = range / n_bin;


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
  ROS_INFO("PC cb");
}

//------------------------------------------------------------------------------
//
void RawMap::OdomCallback(const nav_msgs::Odometry::ConstPtr &odo_in)
ATLAS_NOEXCEPT {
  // - Generate 3x3 transformation matrix from Quaternions
  auto orientation = &odo_in.get()->pose.pose.orientation;
  Eigen::Quaterniond quaterniond(orientation->w, orientation->x, orientation->y, orientation->z);
  Eigen::Matrix3d rotation;
  rotation =  quaterniond.toRotationMatrix();
  // - Set all odometry values that will be use in the cv::mat update thread
  Eigen::Vector3d euler_vec = rotation.eulerAngles(0, 1, 2);
  float roll = euler_vec.x(), pitch = euler_vec.y(), yaw = euler_vec.z();

  world_.sub.rotation = rotation;
  world_.sub.yaw = yaw;
  world_.sub.pitch = pitch;
  world_.sub.roll = roll;
  world_.sub.position.x = odo_in.get()->pose.pose.position.x;
  world_.sub.position.y = odo_in.get()->pose.pose.position.y;
}

//------------------------------------------------------------------------------
//
void RawMap::Run() {

  cv::Point2i initial_position_offset = world_.sub.initialPosition * pixel_.m_to_pixel;
//  cv::Mat rotation_mat = cv::getRotationMatrix2D(cv::Point(pixel_.width/2,pixel_.height/2), -90, 1);
  while (IsRunning()) {
    if (new_pcl_ready_ and last_pcl_) {
      ProcessPointCloud(last_pcl_);
      new_pcl_ready_ = false;

      pixel_.map.copyTo(displayMap);

      cv::Point2f sub_coord = CoordinateToPixel(world_.sub.position);
      sub_coord += (world_.sub.initialPosition *pixel_.m_to_pixel);
      cv::circle(displayMap, sub_coord, 5, CV_RGB(255, 255, 255), -1);


      for( size_t i = 0; i < pixel_.width; i+=pixel_.m_to_pixel)
      {
        cv::line(displayMap, cv::Point2i(i, 0), cv::Point2i(i, pixel_.width), CV_RGB(100,100,100), 2);
      }
      for( size_t i = 0; i < pixel_.height; i += pixel_.m_to_pixel)
      {
        cv::line(displayMap, cv::Point2i(0, i), cv::Point2i(pixel_.width, i), CV_RGB(100,100,100), 2);
      }
      cv::circle(displayMap, initial_position_offset, 10, CV_RGB(255, 255, 255), -1);

//      cv::warpAffine(displayMap, displayMap, rotation_mat, displayMap.size());
//      cv::flip(displayMap, displayMap, 0);
      //cv::imshow(" ", displayMap);
      //cv::waitKey(1);
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
  pixel_.pixel_to_m = r;
  pixel_.m_to_pixel = 1/r;

  pixel_.map = cv::Mat(static_cast<int>(pixel_.width),
                       static_cast<int>(pixel_.height), CV_8UC1);
  pixel_.map.setTo(cv::Scalar(0));
  pixel_.map_color = cv::Mat(static_cast<int>(pixel_.width),
                       static_cast<int>(pixel_.height), CV_8UC3);
  pixel_.map_color.setTo(cv::Scalar(0));
  displayMap = cv::Mat(static_cast<int>(pixel_.width),
                       static_cast<int>(pixel_.height), CV_8UC1);
  displayMap.setTo(cv::Scalar(0));
  std::cout << "Map meter info: Width=" << w << " Height=" << h << std::endl;
  std::cout << "Map pixel info: Width=" << pixel_.width << " Height= " << pixel_.height << std::endl;
  std::cout << "Resolution in meter: " << r << std::endl;

  // - Keeps the number of hits for each pixels
  pixel_.number_of_hits_.resize(pixel_.width * pixel_.height, 0);
}

//------------------------------------------------------------------------------
//
void RawMap::ProcessPointCloud(const sensor_msgs::PointCloud2::ConstPtr &msg) {
  float x = 0, y = 0, z = 0, intensity = 0;

  uint32_t last_bin_index = static_cast<uint32_t>(msg->data.size() / msg->point_step) - 1;
  uint32_t i = 0, max_size = msg->data.size() / msg->point_step;
  cv::Point2i bin_coordinate;
  cv::Point2f offset_sub = world_.sub.position+world_.sub.initialPosition;
  for (i = 0; i < max_size; i ++ ) {
    int step = i * msg->point_step;
    memcpy(&x, &msg->data[step + msg->fields[0].offset], sizeof(float));
    memcpy(&y, &msg->data[step + msg->fields[1].offset], sizeof(float));
    memcpy(&z, &msg->data[step + msg->fields[2].offset], sizeof(float));
    memcpy(&intensity, &msg->data[i * msg->point_step + msg->fields[3].offset], sizeof(float));

    Eigen::Vector3d in(x,y,z), out;
    out = world_.sub.rotation * in;

    cv::Point2f coordinate_transformed = cv::Point2f(out.x(), out.y()) + offset_sub;

    bin_coordinate = CoordinateToPixel(coordinate_transformed);

    UpdateMat(bin_coordinate, (static_cast<uint8_t>(255.0f * intensity)));
  }


  // -- Tile Generator logic
  if (i == point_cloud_threshold_ || i == last_bin_index) {
    tile_generator_.UpdateTileBoundaries(bin_coordinate);
  }

  if (tile_generator_.IsTileReadyForProcess()) {
    cv::Mat ROI;
  }
}

}  // namespace proc_mapping
