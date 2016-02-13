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

#ifndef PROC_MAPPING_INTERPRETER_RAW_MAP_INTERPRETER_H_
#define PROC_MAPPING_INTERPRETER_RAW_MAP_INTERPRETER_H_

#include <memory>
#include <vector>
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

class RawMapInterpreter : public DataInterpreter<cv::Mat> {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<RawMapInterpreter>;
  using ConstPtr = std::shared_ptr<const RawMapInterpreter>;
  using PtrList = std::vector<RawMapInterpreter::Ptr>;
  using ConstPtrList = std::vector<RawMapInterpreter::ConstPtr>;

  // Using message filter in order to sync the two topics without buffers
  using PointCloud2SubPtr =
      std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>>;
  using OdometrySubPtr =
      std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>>;

  // Our synchronizers types
  using PointsOdomSyncPolicies =
      message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
                                                      nav_msgs::Odometry>;
  using PointsOdomSyncPtr =
      std::shared_ptr<message_filters::Synchronizer<PointsOdomSyncPolicies>>;

  static const uint16_t FRAME_SYNC;

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit RawMapInterpreter(const ros::NodeHandlePtr &nh) ATLAS_NOEXCEPT;

  virtual ~RawMapInterpreter() ATLAS_NOEXCEPT;

  //==========================================================================
  // P U B L I C   M E T H O D S

  static pcl::PointCloud<pcl::PointXYZ> MatToPointXYZ(const cv::Mat &m)
      ATLAS_NOEXCEPT;

  static cv::Mat PointXYZToMat(
      const pcl::PointCloud<pcl::PointXYZ> &point_cloud) ATLAS_NOEXCEPT;

 private:
  //==========================================================================
  // P R I V A T E   M E T H O D S

  void PointCloud2OdomCallback(const sensor_msgs::PointCloud2::ConstPtr &msg_in,
                               const nav_msgs::Odometry::ConstPtr &odo_in)
      ATLAS_NOEXCEPT;

  /**
   * From an odometry message, this will compare the value of the last
   * odometry we stored and this will sort out the transformation matrix
   * between the both odometry.
   *
   * The transformation matrix will be usefull when we will register the
   * new PointCloud in the original PoinCloud message.
   */
  Eigen::Affine3d GetTransformationMatrix(
      const nav_msgs::Odometry::ConstPtr &odom) ATLAS_NOEXCEPT;

  //==========================================================================
  // P R I V A T E   M E M B E R S

  ros::NodeHandlePtr nh_;

  PointCloud2SubPtr points2_sub_;
  OdometrySubPtr odom_sub_;

  PointsOdomSyncPtr sync_po_;

  uint32_t nb_added_clouds_;

  /**
   * We are going to store the final point_cloud here.
   * When the first point cloud is received, it is stored here and laters pc
   * will be registered with the odometry delta (taking the translation and
   * rotation)
   */
  pcl::PointXYZ point_cloud_;

  Eigen::Affine3d last_odom_;
  Eigen::Affine3d current_odom_;
};

}  // namespace proc_mapping

#endif  // PROC_MAPPING_INTERPRETER_RAW_MAP_INTERPRETER_H_
