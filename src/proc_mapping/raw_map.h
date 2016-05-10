/**
 * \file	raw_map_interpreter.h
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

#ifndef PROC_MAPPING_INTERPRETER_RAW_MAP_H_
#define PROC_MAPPING_INTERPRETER_RAW_MAP_H_

#include <lib_atlas/macros.h>
#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <array>
#include <eigen3/Eigen/Geometry>
#include <memory>
#include <vector>
#include "proc_mapping/interpreter/data_interpreter.h"

namespace proc_mapping {

class RawMap : public atlas::Subject<cv::Mat>, public atlas::Runnable {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<RawMap>;
  using ConstPtr = std::shared_ptr<const RawMap>;
  using PtrList = std::vector<RawMap::Ptr>;
  using ConstPtrList = std::vector<RawMap::ConstPtr>;

  // Pixel Coordinate System
  struct PixelCS {
    // Cannot use size_t because of the compatibility with opencv...
    int width;
    int height;
    double m_to_pixel;
    double pixel_to_m;
    cv::Mat map, map_color;
    // - Number of Hits per pixel
    std::vector<uint8_t> number_of_hits_;
  };

  // Sub Marine Coordinate System
  struct SubMarineCS {
    Eigen::Transform<double, 3, Eigen::Affine> affine;
    Eigen::Matrix3d rotation;
    double yaw;
    double pitch;
    double roll;
    cv::Point2d position;
    cv::Point2d initial_position;
  };

  struct WorldCS {
    double width;
    double height;
  };

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit RawMap(const ros::NodeHandlePtr &nh);

  virtual ~RawMap() = default;

  //==========================================================================
  // P U B L I C   M E T H O D S

  void Run() override;

 private:
  //==========================================================================
  // P R I V A T E   M E T H O D S

  void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg_in);

  void OdomCallback(const nav_msgs::Odometry::ConstPtr &odo_in);

  /**
   * Set the parameters of the coordinate systems. These parameters are going
   * to be used when converting from XY CCS to UV CCS.
   *
   * \param w The width of the map in world CCS (meters)
   * \param h The height of the map in world CCS (meters)
   * \param r The pixel_to_m of the pixel CCS (meter/pixel)
   */
  void SetMapParameters(const size_t &w, const size_t &h, const double &r);

  void SetPointCloudThreshold(double sonar_threshold, double resolution);

  void ProcessPointCloud(const sensor_msgs::PointCloud2::ConstPtr &msg);

  void UpdateMat(const cv::Point2d &p, const uint8_t &intensity);

  cv::Point2d CoordinateToPixel(const cv::Point2d &p);

  bool IsMapReadyForProcess();

  //==========================================================================
  // P R I V A T E   M E M B E R S

  ros::NodeHandlePtr nh_;
  ros::Subscriber points2_sub_;
  ros::Subscriber odom_sub_;

  // The first data of the sonar may be scrap. Keeping a threshold and starting
  // to process data after it. MUST BE A MULTIPLE OF 16 (sonar_threshold_ = 16 *
  // numberOfPointsToSkip)
  uint32_t point_cloud_threshold_;
  uint32_t hit_count_;

  std::atomic<bool> new_pcl_ready_;

  sensor_msgs::PointCloud2::ConstPtr last_pcl_;

  PixelCS pixel_;
  SubMarineCS sub_;
  WorldCS world_;
  cv::Mat displayMap;

  int scanlines_per_tile_;
  int scanline_counter_;

  bool is_map_ready_for_process_;
};

}  // namespace proc_mapping

#endif  // PROC_MAPPING_INTERPRETER_RAW_MAP_H_
