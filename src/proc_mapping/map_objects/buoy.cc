/**
 * \file	buoy.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	09/06/2016
 *
 * \copyright Copyright (c) 2016 S.O.N.I.A. All rights reserved.
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

#include "proc_mapping/map_objects/buoy.h"

namespace proc_mapping {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
Buoy::Buoy(const cv::KeyPoint &key_point) : MapObject() {
  SetCvKeyPoint(key_point);
  pose_.x = key_point.pt.x;
  pose_.y = key_point.pt.y;
  size_ = key_point.size;
}

//------------------------------------------------------------------------------
//
Buoy::~Buoy() = default;

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void Buoy::DrawToMap(cv::Mat map) const {
  cv::circle(map, GetCvKeyPoint().pt, 10, 255, -1);
}

//------------------------------------------------------------------------------
//
uint8_t Buoy::GetMessageObjectType() const {
  return sonia_msgs::MapObject::BUOYS;
}

//------------------------------------------------------------------------------
//
visualization_msgs::Marker Buoy::GenerateVisualizationMarker(int id) const {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "NED";
  marker.header.stamp = ros::Time();
  marker.ns = "proc_mapping";
  marker.id = id;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = GetPose().x;
  marker.pose.position.y = GetPose().y;
  marker.pose.position.z = GetPose().z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0;
  marker.color.b = 0;

  return marker;
}

}  // namespace proc_mapping
