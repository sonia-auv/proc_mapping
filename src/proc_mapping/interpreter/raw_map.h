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

#include <memory>
#include <vector>
#include <array>
#include <opencv/cv.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <eigen3/Eigen/Eigen>

#include <lib_atlas/macros.h>
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

  struct PixelCCS {
    size_t width;
    size_t height;
    double resolution;
    cv::Mat map;
  };

  struct WorldCCS {
    size_t width;
    size_t height;
    double x_0;
    double y_0;
    pcl::PointCloud<pcl::PointXYZ> cloud;
  };

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit RawMap(const ros::NodeHandlePtr &nh) ATLAS_NOEXCEPT;

  virtual ~RawMap() ATLAS_NOEXCEPT;

  //==========================================================================
  // P U B L I C   M E T H O D S

  void Run() override;

 private:
  //==========================================================================
  // P R I V A T E   M E T H O D S

  void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg_in)
      ATLAS_NOEXCEPT;

  void OdomCallback(const nav_msgs::Odometry::ConstPtr &odo_in) ATLAS_NOEXCEPT;

  /**
   * Set the parameters of the coordinate systems. These parameters are going
   * to be used when converting from XY CCS to UV CCS.
   *
   * \param w The width of the map in world CCS (meters)
   * \param h The height of the map in world CCS (meters)
   * \param r The resolution of the pixel CCS (meter/pixel)
   */
  void SetMapParameters(const size_t &w, const size_t &h,
                        const double &r) ATLAS_NOEXCEPT;

  void ProcessPointCloud(const sensor_msgs::PointCloud2::ConstPtr &msg,
                         const Eigen::Affine3f &t);

  //==========================================================================
  // P R I V A T E   M E M B E R S

  ros::NodeHandlePtr nh_;
  ros::Subscriber points2_sub_;
  ros::Subscriber odom_sub_;

  sensor_msgs::PointCloud2::ConstPtr last_pcl_;
  nav_msgs::Odometry::ConstPtr last_odom_;

  // The first data of the sonar may be scrap. Keeping a threshold and starting
  // to process data after it.
  uint32_t sonar_threshold_;
  uint32_t hit_count_;

  std::atomic<bool> new_pcl_ready_;
  std::atomic<bool> first_odom_received_;

  PixelCCS pixel_;
  WorldCCS world_;
};

}  // namespace proc_mapping

#endif  // PROC_MAPPING_INTERPRETER_RAW_MAP_H_
