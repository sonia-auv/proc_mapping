/**
 * \file	coordinate_system_convertor.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	10/06/2016
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

#include "proc_mapping/map/coordinate_systems.h"
#include <lib_atlas/maths/matrix.h>

namespace proc_mapping {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
CoordinateSystems::CoordinateSystems(const ros::NodeHandlePtr &nh)
    : is_first_odom_(true) {
  int w = 0;
  int h = 0;
  nh->param<int>("/proc_mapping/map/width", w, 20);
  nh->param<int>("/proc_mapping/map/height", h, 20);
  nh->param<double>("/proc_mapping/map/origin_x", world_.origin.x, 10.0);
  nh->param<double>("/proc_mapping/map/origin_y", world_.origin.y, 10.0);

  int n_bin = 0;
  double range = 0.;
  nh->param<int>("/provider_sonar/sonar/n_bins_", n_bin, 400);
  nh->param<double>("/provider_sonar/sonar/range_", range, 10.0);
  // Resolution is equal to the range of the sonar divide by the number of bin
  // of a scanline.
  double r = range / n_bin;

  world_.offset = world_.origin;

  SetMapParameters(static_cast<uint32_t>(w), static_cast<uint32_t>(h), r);

  std::string odometry_topic;
  nh->param<std::string>("/proc_mapping/topics/odometry", odometry_topic,
                         "/proc_navigation/odom");
  odom_sub_ = nh->subscribe(odometry_topic, 100,
                            &CoordinateSystems::OdomCallback, this);
}

//------------------------------------------------------------------------------
//
CoordinateSystems::~CoordinateSystems() {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void CoordinateSystems::OdomCallback(
    const nav_msgs::Odometry::ConstPtr &odo_in) {
  std::lock_guard<std::mutex> lock(data_mutex);
  if (is_first_odom_) {
    sub_.initial_position.x = odo_in.get()->pose.pose.position.x;
    sub_.initial_position.y = odo_in.get()->pose.pose.position.y;
    sub_.initial_position.z = odo_in.get()->pose.pose.position.z;
    is_first_odom_ = false;
  }

  //  Generate 3x3 transformation matrix from Quaternions
  auto orientation = &odo_in.get()->pose.pose.orientation;
  Eigen::Quaterniond quaterniond(orientation->w, orientation->x, orientation->y,
                                 orientation->z);
  //  The quaternion is required to be normalized, otherwise the result is
  //  undefined.
  quaterniond.normalized();

  Eigen::Matrix3d rotation;
  rotation = quaterniond.toRotationMatrix();

  //  Set all odometry values that will be use in the cv::mat update thread
  Eigen::Vector3d euler_vec = rotation.eulerAngles(0, 1, 2);

  // The pitch axis as well as the yaw axes are inverted. We want to invert them
  // here. (the minus sign)
  double roll = euler_vec.x();
  double pitch = -euler_vec.y();
  double yaw = -euler_vec.z();

  Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

  sub_.orientation = yawAngle * pitchAngle * rollAngle;
  sub_.orientation.normalize();

  sub_.rotation = sub_.orientation.matrix();
  sub_.yaw = yaw;
  sub_.pitch = pitch;
  sub_.roll = roll;
  sub_.position.x = odo_in.get()->pose.pose.position.x;
  sub_.position.y = odo_in.get()->pose.pose.position.y;
  sub_.position.z = odo_in.get()->pose.pose.position.z;
}

//------------------------------------------------------------------------------
//
cv::Point2i CoordinateSystems::WorldToPixelCoordinates(
    const cv::Point2d &p) const noexcept {
  return p * pixel_.m_to_pixel;
}

//------------------------------------------------------------------------------
//
std::vector<cv::Point2i> CoordinateSystems::WorldToPixelCoordinates(
    const std::vector<cv::Point2d> &p) const noexcept {
  std::vector<cv::Point2i> v(p.size());
  for (size_t i = 0; i < p.size(); ++i) {
    v[i] = WorldToPixelCoordinates(p[i]);
  }
  return v;
}

//------------------------------------------------------------------------------
//
cv::Rect CoordinateSystems::WorldToPixelCoordinates(const cv::Rect &p) const
    noexcept {
  std::lock_guard<std::mutex> lock(data_mutex);
  cv::Rect world_rect(p.x * pixel_.m_to_pixel, p.y * pixel_.m_to_pixel,
                      p.height * pixel_.m_to_pixel,
                      p.width * pixel_.m_to_pixel);
  return world_rect;
}

//------------------------------------------------------------------------------
//
cv::Point2d CoordinateSystems::PixelToWorldCoordinates(
    const cv::Point2i &p) const noexcept {
  std::lock_guard<std::mutex> lock(data_mutex);
  // The operator/ does not exist for Point2i, we must assign members one by
  // one.
  cv::Point2d world_point;
  world_point.x = static_cast<double>(p.x) / pixel_.m_to_pixel;
  world_point.y = static_cast<double>(p.y) / pixel_.m_to_pixel;
  return world_point;
}

//------------------------------------------------------------------------------
//
double CoordinateSystems::PixelToWorldCoordinates(double p) const
noexcept {
  std::lock_guard<std::mutex> lock(data_mutex);
  return static_cast<double>(p) / pixel_.m_to_pixel;
}

//------------------------------------------------------------------------------
//
void CoordinateSystems::SetMapParameters(const size_t &w, const size_t &h,
                                         const double &r) {
  std::lock_guard<std::mutex> lock(data_mutex);
  world_.width = w;
  world_.height = h;

  pixel_.width = static_cast<uint32_t>(w / r);
  pixel_.height = static_cast<uint32_t>(h / r);
  pixel_.pixel_to_m = r;
  pixel_.m_to_pixel = 1 / r;
}

//------------------------------------------------------------------------------
//
void CoordinateSystems::SetPositionOffset(cv::Point2d offset) {
  world_.offset = offset;
}

//------------------------------------------------------------------------------
//
cv::Point2d CoordinateSystems::GetPositionOffset() const {
  std::lock_guard<std::mutex> lock(data_mutex);
  return world_.offset;
}

//------------------------------------------------------------------------------
//
void CoordinateSystems::ResetPosition() {
  std::lock_guard<std::mutex> lock(data_mutex);
  auto pose2d = cv::Point2d(sub_.position.x, sub_.position.y);
  cv::Point2d delta = world_.origin - pose2d;
  SetPositionOffset(delta);
}

}  // namespace proc_mapping
