/**
 * \file	vision_interpreter.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	09/06/2016
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

#ifndef PROC_MAPPING_COORDINATE_SYSTEM_CONVERTOR_H_
#define PROC_MAPPING_COORDINATE_SYSTEM_CONVERTOR_H_

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <atomic>
#include <eigen3/Eigen/Eigen>
#include <memory>
#include <vector>
#include "opencv/highgui.h"

namespace proc_mapping {

class CoordinateSystems {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<CoordinateSystems>;
  using ConstPtr = std::shared_ptr<const CoordinateSystems>;
  using PtrList = std::vector<CoordinateSystems::Ptr>;
  using ConstPtrList = std::vector<CoordinateSystems::ConstPtr>;

  // Pixel Coordinate System
  struct PixelCS {
    // Cannot use size_t because of the compatibility with opencv
    int width;
    int height;
    double m_to_pixel;
    double pixel_to_m;
  };

  // Sub Marine Coordinate System
  struct SubMarineCS {
    Eigen::Matrix3d rotation;
    double yaw;
    double pitch;
    double roll;
    Eigen::Quaterniond orientation;
    cv::Point3d position;
    cv::Point3d initial_position;
  };

  struct WorldCS {
    double width;
    double height;
    cv::Point2d origin;
    cv::Point2d offset;
  };

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit CoordinateSystems(const ros::NodeHandlePtr &nh);
  virtual ~CoordinateSystems();

  //==========================================================================
  // P U B L I C   M E T H O D S

  void OdomCallback(const nav_msgs::Odometry::ConstPtr &odo_in);

  /// Converting a point from the world Coordinate System to the pixel CS.
  /// In order to do this, we will use the m_to_pixel ratio from the PixelCS
  /// struct.
  cv::Point2i WorldToPixelCoordinates(const cv::Point2d &p) const noexcept;

  std::vector<cv::Point2i> WorldToPixelCoordinates(
      const std::vector<cv::Point2d> &p) const noexcept;

  cv::Rect WorldToPixelCoordinates(const cv::Rect &p) const noexcept;

  /// Converting a pixel point to the world Coordinate system.
  /// Apply the opposite convetion that WorldToPixelCoordinates does.
  cv::Point2d PixelToWorldCoordinates(const cv::Point2i &p) const noexcept;

  /**
   * Set the parameters of the coordinate systems. These parameters are going
   * to be used when converting from XY CS to UV CS.
   *
   * \param w The width of the map in world CS (meters)
   * \param h The height of the map in world CS (meters)
   * \param r The pixel_to_m of the pixel CS (meter/pixel)
   */
  void SetMapParameters(const size_t &w, const size_t &h, const double &r);

  // Function that return the conversion function of the RawMap object (need
  // the delegate to send it to the Draw method of the rois).
  template <class Tp_>
  auto GetConvertionFunction() const -> std::function<cv::Point2i(const Tp_ &)>;

  const PixelCS &GetPixel() const;

  const SubMarineCS &GetSub() const;

  const WorldCS &GetWorld() const;

  bool IsCoordinateSystemReady() const;

  /// We want the submarine to be in the center of the raw map.
  /// Thus, we are going to offset it by the half of the map size.
  cv::Point2d GetPositionOffset() const;
  void SetPositionOffset(cv::Point2d offset);

  void ResetPosition();

 private:
  //==========================================================================
  // P R I V A T E   M E M B E R S

  std::atomic<bool> is_first_odom_;
  ros::Subscriber odom_sub_;

  /// There is a few transformation involved here. We are going to store the
  /// Coordinate system as well as the informations for changing the
  /// referential.
  PixelCS pixel_;
  SubMarineCS sub_;
  WorldCS world_;

  mutable std::mutex data_mutex;
};

//------------------------------------------------------------------------------
//
inline const CoordinateSystems::PixelCS &CoordinateSystems::GetPixel() const {
  std::lock_guard<std::mutex> lock(data_mutex);
  return pixel_;
}

//------------------------------------------------------------------------------
//
inline const CoordinateSystems::SubMarineCS &CoordinateSystems::GetSub() const {
  std::lock_guard<std::mutex> lock(data_mutex);
  return sub_;
}

//------------------------------------------------------------------------------
//
inline const CoordinateSystems::WorldCS &CoordinateSystems::GetWorld() const {
  std::lock_guard<std::mutex> lock(data_mutex);
  return world_;
}

//------------------------------------------------------------------------------
//
inline bool CoordinateSystems::IsCoordinateSystemReady() const {
  return is_first_odom_ == false;
}

//------------------------------------------------------------------------------
//
template <class Tp_>
inline auto CoordinateSystems::GetConvertionFunction() const
    -> std::function<cv::Point2i(const Tp_ &)> {
  using std::placeholders::_1;
  // The function WorldToPixelCoordinates is overloaded, thus can't be
  // binded automatically, need to do this manually.
  // cf. http://www.boost.org/doc/libs/1_50_0/libs/bind/bind.html#err_overloaded
  return std::bind(
      static_cast<cv::Point2i (CoordinateSystems::*)(const Tp_ &) const>(
          &CoordinateSystems::WorldToPixelCoordinates),
      this, _1);
}

}  // namespace proc_mapping

#endif  // PROC_MAPPING_COORDINATE_SYSTEM_CONVERTOR_H_
