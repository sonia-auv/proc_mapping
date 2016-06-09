/**
 * \file	semantic_map.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	31/05/2016
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

#include "proc_mapping/region_of_interest.h"

namespace proc_mapping {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
RegionOfInterest::RegionOfInterest(const YAML::Node &node)
    : object_type_(DetectionMode::NONE), contours_({}) {
  Deserialize(node);
}

//------------------------------------------------------------------------------
//
RegionOfInterest::RegionOfInterest(const std::string &name,
                                   const ContourType &contour,
                                   const DetectionMode &mode)
    : object_type_(mode), contours_(contour), name_(name) {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
bool RegionOfInterest::Deserialize(const YAML::Node &node) {
  // Asser that the node is formated correctly.
  assert(node["objecy_type"]);
  assert(node["name"]);
  assert(node["center"]);
  assert(node["size"]);
  assert(node["angle"]);

  name_ = node["name"].as<std::string>();

  auto objecy_type = node["objecy_type"].as<std::string>();
  if (objecy_type == "buoys") {
    object_type_ = DetectionMode::BUOYS;
  } else if (objecy_type == "fence") {
    object_type_ = DetectionMode::FENCE;
  } else {
    object_type_ = DetectionMode::NONE;
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
bool RegionOfInterest::Serialize(const YAML::Node &node) {
  // Todo Thibaut: Implement the serializing for the RegionOfInterest
  return true;
}

//------------------------------------------------------------------------------
//
const DetectionMode &RegionOfInterest::GetObjectType() const {
  return object_type_;
}

//------------------------------------------------------------------------------
//
RegionOfInterest::ContourType RegionOfInterest::GetContour() const {
  return contours_;
}

//------------------------------------------------------------------------------
//
RegionOfInterest::RotatedRectType RegionOfInterest::GetRotatedRect() const {
    return cv::RotatedRect(center_, size_, angle_);
}

//------------------------------------------------------------------------------
//
bool RegionOfInterest::IsInZone(const cv::Point2i &p) const {
  auto result = cv::pointPolygonTest(contours_, p, false);
  return result == 0 || result == 1;
}

//------------------------------------------------------------------------------
//
bool RegionOfInterest::IsInZone(const cv::Rect &p) const {
  auto result = cv::pointPolygonTest(contours_, cv::Point2d(p.x, p.y), false);
  if (!(result == 0 || result == 1)) {
    return false;
  }

  result =
      cv::pointPolygonTest(contours_, cv::Point2d(p.x + p.width, p.y), false);
  if (!(result == 0 || result == 1)) {
    return false;
  }

  result =
      cv::pointPolygonTest(contours_, cv::Point2d(p.x, p.y + p.height), false);
  if (!(result == 0 || result == 1)) {
    return false;
  }
  result = cv::pointPolygonTest(
      contours_, cv::Point2d(p.x + p.width, p.y + p.height), false);
  if (!(result == 0 || result == 1)) {
    return false;
  }

  return true;
}

}  // namespace proc_mapping
