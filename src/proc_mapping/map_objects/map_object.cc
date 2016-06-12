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

#include "proc_mapping/map_objects/map_object.h"

namespace proc_mapping {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
MapObject::MapObject() {}

//------------------------------------------------------------------------------
//
MapObject::~MapObject() = default;

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
const std::string &MapObject::GetName() const { return name_; }

//------------------------------------------------------------------------------
//
void MapObject::SetName(const std::string &name) { name_ = name; }

//------------------------------------------------------------------------------
//
const MapObject::Pose &MapObject::GetPose() const { return pose_; }

//------------------------------------------------------------------------------
//
void MapObject::SetPose(const Pose &pose) { pose_ = pose; }

//------------------------------------------------------------------------------
//
const MapObject::Orientation &MapObject::GetOrientation() const {
  return orientation_;
}

//------------------------------------------------------------------------------
//
void MapObject::SetOrientation(const Orientation &orientation) {
  orientation_ = orientation;
}

//------------------------------------------------------------------------------
//
const cv::KeyPoint &MapObject::GetCvKeyPoint() const {
  return trigged_keypoint_;
}

//------------------------------------------------------------------------------
//
void MapObject::SetCvKeyPoint(const cv::KeyPoint &key_point) {
  trigged_keypoint_ = key_point;
}

//------------------------------------------------------------------------------
//
const cv::Rect &MapObject::GetBoundingBox() const { return bounding_box_; }

//------------------------------------------------------------------------------
//
void MapObject::SetBoundingBox(const cv::Rect &box) { bounding_box_ = box; }

//------------------------------------------------------------------------------
//
void MapObject::AddWeight(uint8_t to_add) { weight_ += to_add; }

//------------------------------------------------------------------------------
//
void MapObject::RemoveWeight(uint8_t to_remove) {
  if (weight_ < to_remove) {
    weight_ = 0;
  } else {
    weight_ -= to_remove;
  }
}

//------------------------------------------------------------------------------
//
sonia_msgs::MapObject MapObject::GenerateToMapObjectMessge() const {
  sonia_msgs::MapObject msg{};
  msg.name = GetName();
  msg.pose.x = pose_.x;
  msg.pose.y = pose_.y;
  msg.type = GetMessageObjectType();
  return msg;
}

}  // namespace proc_mapping
