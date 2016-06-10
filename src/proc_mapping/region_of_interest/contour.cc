/**
 * \file	contour.cc
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

#include "proc_mapping/region_of_interest/contour.h"

namespace proc_mapping {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
Contour::Contour(const YAML::Node &node) : RegionOfInterest(node) {}

//------------------------------------------------------------------------------
//
Contour::Contour(const std::string &name, const ContourType &contour,
                 const DetectionMode &mode)
    : RegionOfInterest(name, mode), contours_(contour) {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
bool Contour::Deserialize(const YAML::Node &node) {
  // Asser that the node is formated correctly.
  assert(node["objecy_type"]);
  assert(node["points"]);
  assert(node["name"]);

  SetName(node["name"].as<std::string>());

  auto objecy_type = node["objecy_type"].as<std::string>();
  if (objecy_type == "buoys") {
    SetObjectType(DetectionMode::BUOYS);
  } else if (objecy_type == "fence") {
    SetObjectType(DetectionMode::FENCE);
  } else {
    SetObjectType(DetectionMode::NONE);
  }

  auto points_node = node["points"];
  for (int i = 0; i < points_node.size(); ++i) {
    assert(points_node[i].Type() == YAML::NodeType::Sequence);
    assert(points_node[i].size() == 2);
    contours_.push_back(
        {points_node[i][0].as<int>(), points_node[i][1].as<int>()});
  }
  return true;
}

//------------------------------------------------------------------------------
//
bool Contour::Serialize(const YAML::Node &node) {
  // Todo Thibaut: Implement the serializing for the RegionOfInterest
  return true;
}

//------------------------------------------------------------------------------
//
bool Contour::IsInZone(const cv::Point2i &p) const {
  auto result = cv::pointPolygonTest(contours_, p, false);
  return result == 0 || result == 1;
}

//------------------------------------------------------------------------------
//
bool Contour::IsInZone(const cv::Rect &p) const {
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

//------------------------------------------------------------------------------
//
void Contour::DrawRegion(cv::Mat mat,
                         const std::function<cv::Point2i(const cv::Point2d &p)
                                                 const> &convert) const {
  // Todo: Implement method
}

}  // namespace proc_mapping
