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
    : RegionOfInterest(node) {}

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
  assert(node["objecy_type"]);
  assert(node["name"]);
  assert(node["center"]);
  assert(node["size"]);
  assert(node["angle"]);

  SetName(node["name"].as<std::string>());

  auto objecy_type = node["objecy_type"].as<std::string>();
  if (objecy_type == "buoys") {
    SetObjectType(DetectionMode::BUOYS);
  } else if (objecy_type == "fence") {
    SetObjectType(DetectionMode::FENCE);
  } else {
    SetObjectType(DetectionMode::NONE);
  }

  auto center = node["center"];
  for (size_t i = 0; i < center.size(); ++i) {
    assert(center[i].Type() == YAML::NodeType::Sequence);
    assert(center[i].size() == 2);
    center_.x = center[i][0].as<int>();
    center_.y = center[i][1].as<int>();
  }

  auto size = node["size"];
  for (size_t i = 0; i < size.size(); ++i) {
    assert(size[i].Type() == YAML::NodeType::Sequence);
    assert(size[i].size() == 2);
    size_.width = size[i][0].as<int>();
    size_.height = size[i][1].as<int>();
  }

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
bool RotatedRectangle::IsInZone(const cv::Point2i &p) const {
  // TODO: Implement this method
  return true;
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
