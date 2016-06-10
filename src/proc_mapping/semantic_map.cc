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
#include "proc_mapping/interpreter/object_registery.h"
#include "proc_mapping/region_of_interest/contour.h"
#include "proc_mapping/region_of_interest/rotated_rectangle.h"

namespace proc_mapping {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
SemanticMap::SemanticMap(const RawMap::Ptr &raw_map)
    : raw_map_(raw_map),
      map_objects_({}),
      rois_{},
      new_objects_available_(false) {
  InsertRegionOfInterest("regions_of_interest.yaml");
}

//------------------------------------------------------------------------------
//
SemanticMap::~SemanticMap() = default;

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void SemanticMap::OnSubjectNotify(atlas::Subject<DetectionMode> &subject,
                                  DetectionMode mode) {
  auto map_objects = ObjectRegistery::GetInstance().GetAllMapObject();
  ObjectRegistery::GetInstance().ClearRegistery();
  switch (mode) {
    case DetectionMode::BUOYS:
      GetMetaDataForBuoys(std::move(map_objects));
      break;
    case DetectionMode::FENCE:
      // Todo : Impl the algo for the fence detection from keyPoints
      break;
    case DetectionMode ::WALL:
      // Todo
    case DetectionMode::NONE:
      ROS_WARN("There is no execution mode set for the semantic map.");
      break;
  }
}

//------------------------------------------------------------------------------
//
void SemanticMap::GetMetaDataForBuoys(std::vector<cv::KeyPoint> &&map_objects) {
  std::lock_guard<std::mutex> guard(object_mutex_);
  for (size_t i = 0; i < map_objects.size(); i++) {
    double distance_to_right_blob =
        GetDistanceBewteenKeypoint(map_objects[i].pt, map_objects[i + 1].pt);
    double distance_to_left_blob =
        GetDistanceBewteenKeypoint(map_objects[i].pt, map_objects[i - 1].pt);

    if ((distance_to_right_blob > 1.0f and distance_to_right_blob < 1.6f) or
        (distance_to_left_blob > 1.0f and distance_to_left_blob < 1.6f)) {
      bool is_already_trigged = IsAlreadyTrigged(map_objects[i]);

      if (!is_already_trigged) {
        Keypoint trigged_keypoint;
        trigged_keypoint.trigged_keypoint = map_objects[i];
        trigged_keypoint.bounding_box = SetBoundingBox(map_objects[i].pt, 20);
        trigged_keypoint.is_object_send = false;
        //        if (rois_.size() > 0) {
        //          if
        //          (map_objects[i].pt.inside(rois_.at(0).GetCvBoundingRect()))
        //          {
        //            trigged_keypoint.weight += 10;
        //          } else {
        //            trigged_keypoint.weight += 1;
        //          }
        //        }
        trigged_keypoints_.push_back(trigged_keypoint);
      }

      if (trigged_keypoints_.size() >= 3) {
        for (uint32_t k = 0; k < trigged_keypoints_.size(); ++k) {
          if (!trigged_keypoints_.at(k).is_object_send) {
            sonia_msgs::MapObject obj;

            cv::Point2d offset = raw_map_->GetPositionOffset();
            cv::Point2d world_point = raw_map_->PixelToWorldCoordinates(
                trigged_keypoints_.at(k).trigged_keypoint.pt);
            obj.name = "Buoy [" + std::to_string(k) + "]";
            obj.size = trigged_keypoints_.at(k).trigged_keypoint.size;
            obj.pose.x = world_point.x - offset.x;
            obj.pose.y = world_point.y - offset.y;
            obj.type = sonia_msgs::MapObject::BUOYS;
            trigged_keypoints_.at(k).is_object_send = true;

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
bool SemanticMap::IsAlreadyTrigged(cv::KeyPoint map_object) {
  for (uint32_t k = 0; k < trigged_keypoints_.size(); ++k) {
    if (map_object.pt.inside(trigged_keypoints_.at(k).bounding_box)) {
      trigged_keypoints_.at(k).weight++;
      return true;
    }
  }
  return false;
}

//------------------------------------------------------------------------------
//
double SemanticMap::GetDistanceBewteenKeypoint(cv::Point2d p1, cv::Point2d p2) {
  cv::Point2d world_p1 = raw_map_->PixelToWorldCoordinates(p1);
  cv::Point2d world_p2 = raw_map_->PixelToWorldCoordinates(p2);
  double delta_x = (world_p1.x - world_p2.x);
  double delta_y = (world_p1.y - world_p2.y);
  return sqrt((delta_x * delta_x) + (delta_y * delta_y));
}

//------------------------------------------------------------------------------
//
cv::Rect SemanticMap::SetBoundingBox(cv::Point2d keypoint, int box_size) {
  std::vector<cv::Point> rect;
  rect.push_back(cv::Point2d(keypoint.x + box_size, keypoint.y + box_size));
  rect.push_back(cv::Point2d(keypoint.x + box_size, keypoint.y - box_size));
  rect.push_back(cv::Point2d(keypoint.x - box_size, keypoint.y + box_size));
  rect.push_back(cv::Point2d(keypoint.x - box_size, keypoint.y - box_size));
  return cv::boundingRect(rect);
}

//------------------------------------------------------------------------------
//
const std::vector<SemanticMap::MapObjectsType> &SemanticMap::GetMapObjects() {
  std::lock_guard<std::mutex> guard(object_mutex_);
  new_objects_available_ = false;
  return map_objects_;
}

//------------------------------------------------------------------------------
//
const std::vector<SemanticMap::RegionOfInterestType>
    &SemanticMap::GetRegionOfInterest() const {
  std::lock_guard<std::mutex> guard(object_mutex_);
  return rois_;
}

//------------------------------------------------------------------------------
//
bool SemanticMap::IsNewDataAvailable() const {
  std::lock_guard<std::mutex> guard(object_mutex_);
  return new_objects_available_;
}

//------------------------------------------------------------------------------
//
void SemanticMap::ClearMapObjects() {
  std::lock_guard<std::mutex> guard(object_mutex_);
  map_objects_.clear();
  trigged_keypoints_.clear();
  new_objects_available_ = true;
}

//------------------------------------------------------------------------------
//
SemanticMap::RegionOfInterestType SemanticMap::RegionOfInterestFactory(
    const YAML::Node &node) const {
  assert(node["roi_type"]);
  auto roi_type = node["roi_type"].as<std::string>();

  RegionOfInterest *r = nullptr;
  if (roi_type == "rotated_rectangle") {
    r = new RotatedRectangle(node);
  } else if (roi_type == "contour") {
    r = new Contour(node);
  }

  if (r) {
    return RegionOfInterest::Ptr{r};
  }

  return nullptr;
}

//------------------------------------------------------------------------------
//
void SemanticMap::InsertRegionOfInterest(
    const std::string &proc_tree_file_name) {
  YAML::Node node = YAML::LoadFile(kConfigFilePath + proc_tree_file_name);

  assert(node["regions_of_interest"]);
  auto regions_of_interests = node["regions_of_interest"];
  assert(regions_of_interests.Type() == YAML::NodeType::Sequence);

  std::lock_guard<std::mutex> guard(object_mutex_);
  for (size_t i = 0; i < regions_of_interests.size(); ++i) {
    auto roi = RegionOfInterestFactory(regions_of_interests[i]);
    if (roi) {
      rois_.push_back(roi);
    }
  }
}

}  // namespace proc_mapping
