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
#include <lib_atlas/pattern/runnable.h>
#include <lib_atlas/pattern/subject.h>
#include <lib_atlas/ros/image_publisher.h>
#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <proc_mapping/async_image_publisher.h>
#include <sensor_msgs/PointCloud2.h>
#include <array>
#include <eigen3/Eigen/Geometry>
#include <memory>
#include <mutex>
#include <vector>
#include "proc_mapping/map/coordinate_systems.h"
#include "proc_mapping/pipeline/parameter.h"

namespace proc_mapping {

class RawMap : public atlas::Subject<cv::Mat>, public atlas::Runnable {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<RawMap>;
  using ConstPtr = std::shared_ptr<const RawMap>;
  using PtrList = std::vector<RawMap::Ptr>;
  using ConstPtrList = std::vector<RawMap::ConstPtr>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit RawMap(const ros::NodeHandlePtr &nh,
                  const CoordinateSystems::Ptr &cs);

  virtual ~RawMap() = default;

  //==========================================================================
  // P U B L I C   M E T H O D S

  /// We have a separate thread for it because we want to sync the odometry
  /// with the point cloud. We simply check the flags for new data and
  /// run the processing when needed.
  void Run() override;

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
