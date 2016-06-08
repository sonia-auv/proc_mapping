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

#include "region_of_interest.h"

namespace proc_mapping {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
RegionOfInterest::RegionOfInterest(const YAML::Node &node)
    : object_type_(DetectionMode::NONE), contours_({}) {
  Deserialize(node);
}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
bool RegionOfInterest::Deserialize(const YAML::Node &node) {
  // Asser that the node is formated correctly.
  assert(node["objecy_type"]);
  assert(node["points"]);
  assert(node.Type() == YAML::NodeType::Sequence);

  auto objecy_type = node["objecy_type"].as<std::string>();
  if (objecy_type == "buoys") {
    object_type_ = DetectionMode::BUOYS;
  } else if (objecy_type == "fence") {
    object_type_ = DetectionMode::FENCE;
  } else {
    object_type_ = DetectionMode::NONE;
  }

  auto points_node = node["points"];
  for (int i = 0; i < points_node.size(); ++i) {
    assert(points_node[i].Type() == YAML::NodeType::Sequence);
    assert(points_node[i].size() == 2);
    contours_.push_back(
        {points_node[i][0].as<double>(), points_node[i][1].as<double>()});
  }
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
cv::Rect RegionOfInterest::GetCvBoundingRect() const {
  return cv::boundingRect(contours_);
}

//------------------------------------------------------------------------------
//
bool RegionOfInterest::IsInZone(const cv::Point2d &p) const {
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
