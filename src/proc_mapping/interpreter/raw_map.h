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
#include <tf/transform_datatypes.h>


#include <lib_atlas/macros.h>
#include "proc_mapping/interpreter/data_interpreter.h"
#include "proc_mapping/interpreter/proc_mapping_types.h"
#include "proc_mapping/interpreter/tile_generator.h"

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
    // - Number of Hits per pixel
    std::vector<uint8_t> number_of_hits_;
  };

  struct SubMarineCS{
    double yaw, pitch, roll;
    PointXY<double> position;
    PointXY<double> initialPosition;
  };

  struct WorldCCS {
    size_t width;
    size_t height;
    SubMarineCS sub;
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
  void SetPointCloudThreshold(double sonar_threshold, double resolution);
  void ProcessPointCloud(const sensor_msgs::PointCloud2::ConstPtr &msg);
  inline void UpdateMat(PointXY<int> p, uchar intensity);
  inline PointXY<double> Transform(double x, double y, double cosRotFactor, double sinRotFactor);
  inline PointXY<int> CoordinateToPixel(const PointXY<double> &p);
  //==========================================================================
  // P R I V A T E   M E M B E R S

  ros::NodeHandlePtr nh_;
  ros::Subscriber points2_sub_;
  ros::Subscriber odom_sub_;

  sensor_msgs::PointCloud2::ConstPtr last_pcl_;
  //nav_msgs::Odometry::ConstPtr last_odom_;

  // The first data of the sonar may be scrap. Keeping a threshold and starting
  // to process data after it. MUST BE A MULTIPLE OF 16 (sonar_threshold_ = 16 * numberOfPointsToSkip)
  uint32_t point_cloud_threshold;
  uint32_t hit_count_;

  std::atomic<bool> new_pcl_ready_;

  TileGenerator tile_generator_;

  PixelCCS pixel_;
  WorldCCS world_;

};

inline PointXY<double> RawMap::Transform(double x, double y, double cosRotFactor, double sinRotFactor){
  PointXY<double> offset, result;
  // - Initial position is simply to center the submarine in the middle of the map.
  offset.x = world_.sub.position.x + world_.sub.initialPosition.x;
  offset.y = world_.sub.position.y + world_.sub.initialPosition.y;
  result.x = x * cosRotFactor - y * sinRotFactor + offset.x;
  result.y = x * sinRotFactor + y * cosRotFactor + offset.y;
  return result;
}
inline void RawMap::UpdateMat(PointXY<int> p, uchar intensity){
  if(p.x < pixel_.width && p.y < pixel_.height){
    // - Infinite mean
    pixel_.number_of_hits_.at(p.x + p.y * pixel_.width) ++;
    uint8_t n = pixel_.number_of_hits_.at(p.x + p.y * pixel_.width);
    pixel_.map.at<uchar>(p.x, p.y) = static_cast<uchar>(intensity/n + pixel_.map.at<uchar>(p.x, p.y) * (n - 1)/ n);
    // - Local mean
    //pixel_.map.at<uchar>(p.x, p.y) = static_cast<uchar>((intensity + pixel_.map.at<uchar>(p.x, p.y)) / 2);
    // - Replacement
    //pixel_.map.at<uchar>(p.x, p.y) = intensity;
  }
}
inline PointXY<int> RawMap::CoordinateToPixel(const PointXY<double> &p){
  PointXY<int> result;
  result.x = static_cast<int>(p.x / pixel_.resolution);
  result.y = static_cast<int>(p.y / pixel_.resolution);
  return result;
}

}

#endif  // PROC_MAPPING_INTERPRETER_RAW_MAP_H_
