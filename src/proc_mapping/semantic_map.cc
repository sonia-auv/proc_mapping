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

#include "proc_mapping/semantic_map.h"
#include <proc_mapping/interpreter/object_registery.h>

namespace proc_mapping {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
SemanticMap::SemanticMap(const RawMap::Ptr &raw_map)
    : raw_map_(raw_map),
      map_objects_({}),
      mode_(DetectionMode::NO_MODE),
      new_objects_available_(false){};

//------------------------------------------------------------------------------
//
SemanticMap::~SemanticMap() = default;

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void SemanticMap::OnSubjectNotify(atlas::Subject<> &) {
  auto map_objects = ObjectRegistery::GetInstance().GetAllMapObject();
  ObjectRegistery::GetInstance().ClearRegistery();
  switch (mode_) {
    case DetectionMode::BUOYS:
      GetMetaDataForBuoys(std::move(map_objects));
      break;
    case DetectionMode::FENCE:
      // Todo : Impl the algo for the fence detection from keyPoints
      break;
    case DetectionMode::NO_MODE:
      ROS_WARN("There is no execution mode set for the semantic map.");
      // Todo : Impl the algo for the fence detection from keyPoints
      break;
  }
}

//------------------------------------------------------------------------------
//
void SemanticMap::GetMetaDataForBuoys(std::vector<cv::KeyPoint> &&map_objects) {
  static std::vector<Keypoint> trigged_keypoints;
  static bool is_first = true;

  if (map_objects.size() > 1) {
    for (size_t i = 0; i < map_objects.size(); i++) {
      float distance_x_minus =
          (map_objects[i].pt.x - map_objects[i + 1].pt.x) / 40;
      float distance_y_minus =
          (map_objects[i].pt.y - map_objects[i + 1].pt.y) / 40;
      double distance_keypoint_minus =
          sqrt((distance_x_minus * distance_x_minus) +
               (distance_y_minus * distance_y_minus));

      float distance_x_plus =
          (map_objects[i].pt.x - map_objects[i - 1].pt.x) / 40;
      float distance_y_plus =
          (map_objects[i].pt.y - map_objects[i - 1].pt.y) / 40;
      double distance_keypoint_plus = sqrt((distance_x_plus * distance_x_plus) +
                                           (distance_y_plus * distance_y_plus));

      if ((distance_keypoint_minus > 1.0f and distance_keypoint_minus < 2.6f) or
          (distance_keypoint_plus > 1.0f and distance_keypoint_plus < 2.6f)) {
        bool is_already_trigged = false;

        for (uint32_t k = 0; k < trigged_keypoints.size(); ++k) {
          if (map_objects[i].pt.inside(trigged_keypoints.at(k).bounding_box)) {
            is_already_trigged = true;
          }
        }

        if (!is_already_trigged) {
          Keypoint trigged_keypoint;
          trigged_keypoint.trigged_keypoint = map_objects[i];
          std::vector<cv::Point> rect;
          rect.push_back(
              cv::Point2d(map_objects[i].pt.x + 20, map_objects[i].pt.y + 20));
          rect.push_back(
              cv::Point2d(map_objects[i].pt.x + 20, map_objects[i].pt.y - 20));
          rect.push_back(
              cv::Point2d(map_objects[i].pt.x - 20, map_objects[i].pt.y + 20));
          rect.push_back(
              cv::Point2d(map_objects[i].pt.x - 20, map_objects[i].pt.y - 20));
          trigged_keypoint.bounding_box = cv::boundingRect(rect);
          trigged_keypoint.is_object_send = false;
          trigged_keypoints.push_back(trigged_keypoint);
        }

        for (uint32_t k = 0; k < trigged_keypoints.size(); ++k) {
          if (!trigged_keypoints.at(k).is_object_send) {
            sonia_msgs::MapObject obj;

            cv::Point2d offset = raw_map_->GetPositionOffset();
            auto world_point = raw_map_->PixelToWorldCoordinates(
                trigged_keypoints.at(k).trigged_keypoint.pt);
            obj.name = "Buoy [" + std::to_string(k) + "]";
            obj.size = trigged_keypoints.at(k).trigged_keypoint.size;
            obj.pose.x = world_point.x - offset.x;
            obj.pose.y = world_point.y - offset.y;
            trigged_keypoints.at(k).is_object_send = true;

            // Finally adding the object to the semantic map
            map_objects_.push_back(std::move(obj));
            new_objects_available_ = true;
          }
        }
      }
    }
  }
}

//------------------------------------------------------------------------------
//
const std::vector<SemanticMap::MapObjectsType> &SemanticMap::GetMapObjects() {
  new_objects_available_ = false;
  return map_objects_;
}

//------------------------------------------------------------------------------
//
bool SemanticMap::IsNewDataAvailable() const { return new_objects_available_; }

}  // namespace proc_mapping