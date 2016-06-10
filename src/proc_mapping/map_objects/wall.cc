/**
 * \file	vision_interpreter.cc
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

#include "proc_mapping/map_objects/wall.h"

namespace proc_mapping {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
Wall::Wall() : MapObject() {}

//------------------------------------------------------------------------------
//
Wall::~Wall() = default;

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void Wall::DrawToMap(
    cv::Mat,
    const std::function<cv::Point2i(const cv::Point2d &p)> &convert) const {}

//------------------------------------------------------------------------------
//
uint8_t Wall::GetMessageObjectType() const {
  return sonia_msgs::MapObject::UNKNOWN;
}

}  // namespace proc_mapping
