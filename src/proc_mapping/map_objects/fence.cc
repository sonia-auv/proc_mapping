/**
 * \file	vision_interpreter.cc
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

#include "proc_mapping/map_objects/fence.h"

namespace proc_mapping {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
Fence::Fence(const cv::KeyPoint &key_point) : MapObject() {
  SetCvKeyPoint(key_point);

  pose_.x = key_point.pt.x;
  pose_.y = key_point.pt.y;
  size_ = key_point.size;
}

//------------------------------------------------------------------------------
//
Fence::~Fence() = default;

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void Fence::DrawToMap(cv::Mat map) const {
  cv::line(map,
           cv::Point2d(GetCvKeyPoint().pt.x - 10, GetCvKeyPoint().pt.y - 10),
           cv::Point2d(GetCvKeyPoint().pt.x + 10, GetCvKeyPoint().pt.y + 10),
           cv::Scalar(255), 4);
}

//------------------------------------------------------------------------------
//
uint8_t Fence::GetMessageObjectType() const {
  return sonia_msgs::MapObject::FENCE;
}

//------------------------------------------------------------------------------
//
visualization_msgs::Marker Fence::GenerateVisualizationMarker(int id) const {
  return visualization_msgs::Marker();
}

}  // namespace proc_mapping
