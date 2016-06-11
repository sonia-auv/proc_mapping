/**
 * \file	ellipse.cc
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

#include "proc_mapping/region_of_interest/ellipse.h"

namespace proc_mapping {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
Ellipse::Ellipse(const YAML::Node &node) : RegionOfInterest(node) {
  Deserialize(node);
}

//------------------------------------------------------------------------------
//
Ellipse::Ellipse(const std::string &name, const cv::Point2d &center,
                 const cv::Size &axes, double angle, double start_angle,
                 double end_angle, const DetectionMode &mode)
    : RegionOfInterest(name, mode),
      center_(center),
      axes_(axes),
      angle_(angle),
      start_angle_(start_angle),
      end_angle_(end_angle) {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
bool Ellipse::Deserialize(const YAML::Node &node) {
  // Asser that the node is formated correctly.
  assert(node["object_type"]);
  assert(node["name"]);
  assert(node["center"]);
  assert(node["axes"]);
  assert(node["angle"]);
  assert(node["start_angle"]);
  assert(node["end_angle"]);

  SetName(node["name"].as<std::string>());

  auto object_type = node["object_type"].as<std::string>();
  DetectionModeFactory(object_type);

  axes_ = cv::Size(node["axes"][0].as<int>(), node["axes"][1].as<int>());
  center_ = cv::Point2d(node["center"][0].as<double>(),
                        node["center"][1].as<double>());
  angle_ = node["angle"].as<double>();
  start_angle_ = node["start_angle"].as<double>();
  end_angle_ = node["end_angle"].as<double>();

  return true;
}

//------------------------------------------------------------------------------
//
bool Ellipse::Serialize(const YAML::Node &node) {
  // Todo Thibaut: Implement the serializing for the RegionOfInterest
  return true;
}

//------------------------------------------------------------------------------
//
bool Ellipse::IsInZone(const cv::Point2d &p) const { return true; }

//------------------------------------------------------------------------------
//
bool Ellipse::IsInZone(const cv::Rect &p) const { return true; }

//------------------------------------------------------------------------------
//
void Ellipse::DrawRegion(
    cv::Mat mat,
    const std::function<cv::Point2i(const cv::Point2d &p)> &convert) const {
  cv::Size pixel_axes{convert(
      {static_cast<double>(axes_.width), static_cast<double>(axes_.height)})};
  cv::ellipse(mat, convert(center_), pixel_axes, angle_, start_angle_,
              end_angle_, cv::Scalar(255), 3);
}

}  // namespace proc_mapping
