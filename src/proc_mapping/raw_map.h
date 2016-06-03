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

#ifndef PROC_MAPPING_RAW_MAP_H_
#define PROC_MAPPING_RAW_MAP_H_

#include <lib_atlas/macros.h>
#include <lib_atlas/pattern/runnable.h>
#include <lib_atlas/pattern/subject.h>
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
    // Cannot use size_t because of the compatibility with opencv
    int width;
    int height;
    double m_to_pixel;
    double pixel_to_m;
    cv::Mat map;
    std::vector<uint8_t> number_of_hits_;  //  Number of Hits per pixel
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
    cv::Point2d origin;
  };

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit RawMap(const ros::NodeHandlePtr &nh);

  virtual ~RawMap() = default;

  //==========================================================================
  // P U B L I C   M E T H O D S

  /// We have a separate thread for it because we want to sync the odometry
  /// with the point cloud. We simply check the flags for new data and
  /// run the processing when needed.
  void Run() override;

  /// Converting a point from the world Coordinate System to the pixel CS.
  /// In order to do this, we will use the m_to_pixel ratio from the PixelCS
  /// struct.
  cv::Point2i WorldToPixelCoordinates(const cv::Point2d &p) const noexcept;

  /// Converting a pixel point to the world Coordinate system.
  /// Apply the opposite convetion that WorldToPixelCoordinates does.
  cv::Point2d PixelToWorldCoordinates(const cv::Point2i &p) const noexcept;

  double GetSubMarineYaw() const noexcept;

  cv::Point2d GetSubMarinePosition() const noexcept;

  /// We want the submarine to be in the center of the raw map.
  /// Thus, we are going to offset it by the half of the map size.
  cv::Point2d GetPositionOffset() const;

 private:
  //==========================================================================
  // P R I V A T E   M E T H O D S

  void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg_in);

  void OdomCallback(const nav_msgs::Odometry::ConstPtr &odo_in);

  /**
   * Set the parameters of the coordinate systems. These parameters are going
   * to be used when converting from XY CS to UV CS.
   *
   * \param w The width of the map in world CS (meters)
   * \param h The height of the map in world CS (meters)
   * \param r The pixel_to_m of the pixel CS (meter/pixel)
   */
  void SetMapParameters(const size_t &w, const size_t &h, const double &r);

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
  ros::Subscriber odom_sub_;

  /// The first data of the sonar may be scrap. Keeping a threshold and starting
  /// to process data after it
  uint32_t point_cloud_threshold_;
  double sonar_range_;

  std::atomic<bool> new_pcl_ready_;

  sensor_msgs::PointCloud2::ConstPtr last_pcl_;

  /// There is a few transformation involved here. We are going to store the
  /// Coordinate system as well as the informations for changing the
  /// referential.
  PixelCS pixel_;
  SubMarineCS sub_;
  WorldCS world_;

  /// As we don't want to process the map every scanline, we set a counter of
  /// scan line and increment it. When the scanline_counter hits the wanted
  /// value, we set the map to be ready to process and it is sent to the map
  /// interpreter for processing.
  bool is_map_ready_for_process_;
  int scanlines_for_process_;
  int scanline_counter_;

  /// Flag that states if the first odometry message has already been received.
  /// It is used to calculate the initial position of the submarine.
  bool is_first_odom_;
};

}  // namespace proc_mapping

#endif  // PROC_MAPPING_RAW_MAP_H_
