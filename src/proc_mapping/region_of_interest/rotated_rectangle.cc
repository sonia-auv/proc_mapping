/**
 * \file	rotated_rectangle.cc
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

#include "proc_mapping/region_of_interest/rotated_rectangle.h"

namespace proc_mapping {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
RotatedRectangle::RotatedRectangle(const YAML::Node &node)
    : RegionOfInterest(node) {
  Deserialize(node);
}

//------------------------------------------------------------------------------
//
RotatedRectangle::RotatedRectangle(const std::string &name,
                                   const cv::Point2d &center,
                                   const cv::Size2d &size, double angle,
                                   const DetectionMode &mode)
    : RegionOfInterest(name, mode),
      center_(center),
      size_(size),
      angle_(angle) {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
bool RotatedRectangle::Deserialize(const YAML::Node &node) {
  // Asser that the node is formated correctly.
  assert(node["object_type"]);
  assert(node["name"]);
  assert(node["center"]);
  assert(node["size"]);
  assert(node["angle"]);

  SetName(node["name"].as<std::string>());

  auto object_type = node["object_type"].as<std::string>();
  DetectionModeFactory(object_type);

  center_ = cv::Point2d(node["center"][0].as<double>(),
                        node["center"][1].as<double>());
  size_ = cv::Size(node["size"][0].as<int>(), node["size"][1].as<int>());
  angle_ = node["angle"].as<double>();

  return true;
}

//------------------------------------------------------------------------------
//
bool RotatedRectangle::Serialize(const YAML::Node &node) {
  // Todo Thibaut: Implement the serializing for the RegionOfInterest
  return true;
}

//------------------------------------------------------------------------------
//
bool RotatedRectangle::IsInZone(const cv::Point2d &p) const {
  double rotation = (M_PI * angle_) / 180;
  double delta_x = p.x - center_.x * 40;
  double delta_y = p.y - center_.y * 40;
  double distance = sqrt((delta_x * delta_x) + (delta_y * delta_y));

  double current_angle = atan2(delta_y, delta_x);
  double new_angle = current_angle - rotation;

  double new_x = cos(new_angle) * distance;
  double new_y = sin(new_angle) * distance;

  if (new_x > -0.5 * size_.width * 40 and new_x < 0.5 * size_.width * 40 and
      new_y > -0.5 * size_.height * 40 and new_y < 0.5 * size_.height * 40) {
    return true;
  }

  return false;
}

//------------------------------------------------------------------------------
//
bool RotatedRectangle::IsInZone(const cv::Rect &p) const {
  // TODO: Implement this method
  return true;
}

//------------------------------------------------------------------------------
//
cv::RotatedRect RotatedRectangle::GetCvRotatedRect() const {
  return cv::RotatedRect(center_, size_, angle_);
}

//------------------------------------------------------------------------------
//
void RotatedRectangle::DrawRegion(
    cv::Mat mat,
    const std::function<cv::Point2i(const cv::Point2d &p)> &convert) const {
  cv::Point2f rect_points[4];
  auto rotated_rect = GetCvRotatedRect();
  rotated_rect.points(rect_points);
  for (int i = 0; i < 4; ++i) {
    rect_points[i] = convert(rect_points[i]);
  }
  for (int i = 0; i < 4; ++i) {
    cv::line(mat, rect_points[i], rect_points[(i + 1) % 4], cv::Scalar(255), 3);
  }
}

}  // namespace proc_mapping
