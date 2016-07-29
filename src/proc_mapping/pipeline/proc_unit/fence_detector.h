/**
 * \file	fence_detector.h
 * \author	Francis Masse <francis.masse05@gmail.com>
 * \date	27/06/2016
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

#ifndef PROC_MAPPING_PIPELINE_PROC_UNIT_FENCE_DETECTOR_H_
#define PROC_MAPPING_PIPELINE_PROC_UNIT_FENCE_DETECTOR_H_

#include <opencv/cv.h>
#include <proc_mapping/map/coordinate_systems.h>
#include <proc_mapping/region_of_interest/rotated_rectangle.h>
#include "proc_mapping/map_objects/fence.h"
#include "proc_mapping/pipeline/proc_unit.h"

namespace proc_mapping {

class FenceDetector : public ProcUnit {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<FenceDetector>;
  using ConstPtr = std::shared_ptr<const FenceDetector>;
  using PtrList = std::vector<FenceDetector::Ptr>;
  using ConstPtrList = std::vector<FenceDetector::ConstPtr>;

  struct TriggedKeypoint {
    cv::KeyPoint trigged_keypoint;
    cv::Rect bounding_box;
    uint8_t weight;
    bool is_object_send;
  };

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit FenceDetector(const std::string &topic_namespace,
                         const ObjectRegistery::Ptr &object_registery);

  virtual ~FenceDetector() = default;

  //==========================================================================
  // P U B L I C   M E T H O D S

  virtual void ConfigureFromYamlNode(const YAML::Node &node) override;

  virtual boost::any ProcessData(boost::any input) override;

  std::string GetName() const override;

 private:
  //==========================================================================
  // PRIVATE   M E T H O D S

  inline bool IsAlreadyTrigged(cv::KeyPoint keypoint);

  inline void AddToTriggeredList(cv::KeyPoint keypoint);

  inline void RemoveToTriggeredList(cv::KeyPoint keypoint);

  inline void AddWeightToCorrespondingTriggedKeypoint(
      cv::Point2d trigged_keypoint, int weight);

  inline void RemoveWeightToCorrespondingTriggedKeypoint(
      cv::Point2d trigged_keypoint, int weight);

  inline bool IsKeypointHasEnoughWeight(TriggedKeypoint keypoint);

  inline void SetWeigthGoal(int weight_goal);

  inline cv::Rect SetBoundingBox(cv::Point2d keypoint, int box_size);

  //==========================================================================
  // P R I V A T E   M E M B E R S

  Parameter<int> weight_goal_;
  Parameter<bool> roi_needed_;

  std::vector<TriggedKeypoint> trigged_keypoint_list_;
  ObjectRegistery::Ptr object_registery_;
};

//==============================================================================
// I N L I N E   M E T H O D S

//------------------------------------------------------------------------------
//
inline FenceDetector::FenceDetector(
    const std::string &topic_namespace,
    const ObjectRegistery::Ptr &object_registery)
    : ProcUnit(topic_namespace),
      weight_goal_("Weight Goal", 0, parameters_),
      roi_needed_("ROI Needed", false, parameters_),
      object_registery_(object_registery) {}

//------------------------------------------------------------------------------
//
inline void FenceDetector::ConfigureFromYamlNode(const YAML::Node &node) {
  weight_goal_ = node["weight_goal"].as<int>();
  roi_needed_ = node["roi_needed"].as<bool>();
}

//------------------------------------------------------------------------------
//
inline boost::any FenceDetector::ProcessData(boost::any input) {
  auto keypoint = boost::any_cast<std::vector<cv::KeyPoint>>(input);
  auto rois =
      object_registery_->GetRegionOfInterestOfType(DetectionMode::FENCE);

  if (object_registery_->IsRegisteryCleared()) {
    trigged_keypoint_list_.clear();
    object_registery_->ResetRegisteryClearedFlag();
    for (auto &roi : rois) {
      object_registery_->DeleteRegionOfInterest(roi);
    }
  }

  bool added_new_object = false;
  for (size_t i = 0; i < keypoint.size(); i++) {
    bool is_already_trigged = IsAlreadyTrigged(keypoint[i]);

    if (!is_already_trigged) {
      AddToTriggeredList(keypoint[i]);
    } else {
      for (size_t j = 0; j < trigged_keypoint_list_.size(); ++j) {
        if (roi_needed_.GetValue()) {
          for (auto &roi : rois) {
            if(roi->IsInZone(trigged_keypoint_list_[j].trigged_keypoint.pt)) {
              AddWeightToCorrespondingTriggedKeypoint(
                  trigged_keypoint_list_[j].trigged_keypoint.pt, 1);
            }
          }
        } else {
          AddWeightToCorrespondingTriggedKeypoint(
              trigged_keypoint_list_[j].trigged_keypoint.pt, 1);
        }
      }
    }
  }

  for (size_t j = 0; j < trigged_keypoint_list_.size(); ++j) {
    if (IsKeypointHasEnoughWeight(trigged_keypoint_list_[j])) {
      if (!trigged_keypoint_list_[j].is_object_send) {
        MapObject::Ptr map_object =
            std::make_shared<Fence>(trigged_keypoint_list_[j].trigged_keypoint);
        map_object->SetName("Fence [" + std::to_string(j) + "]");
        map_object->SetSize(trigged_keypoint_list_[j].trigged_keypoint.size);
        ROS_INFO_STREAM(
            "Detecting a FENCE object at the position ["
            << trigged_keypoint_list_[j].trigged_keypoint.pt.x << ";"
            << trigged_keypoint_list_[j].trigged_keypoint.pt.y << "]");
        object_registery_->AddMapObject(std::move(map_object));
        trigged_keypoint_list_[j].is_object_send = true;
        added_new_object = true;
      }
    }
  }

  // We are supposed to be the last PU in the pipeline, so let's return a
  // void boost any.
  return boost::any(added_new_object);
}

//------------------------------------------------------------------------------
//
inline std::string FenceDetector::GetName() const { return "fence_detector"; }

//------------------------------------------------------------------------------
//
inline bool FenceDetector::IsAlreadyTrigged(cv::KeyPoint keypoint) {
  for (size_t i = 0; i < trigged_keypoint_list_.size(); ++i) {
    if (keypoint.pt.inside(trigged_keypoint_list_.at(i).bounding_box)) {
      return true;
    }
  }
  return false;
}

//------------------------------------------------------------------------------
//
inline void FenceDetector::AddToTriggeredList(cv::KeyPoint keypoint) {
  TriggedKeypoint trigged_keypoint;
  trigged_keypoint.trigged_keypoint = keypoint;
  trigged_keypoint.bounding_box = SetBoundingBox(keypoint.pt, 20);
  trigged_keypoint.is_object_send = false;
  trigged_keypoint.weight = 0;
  trigged_keypoint_list_.push_back(trigged_keypoint);
}

//------------------------------------------------------------------------------
//
inline void FenceDetector::RemoveToTriggeredList(cv::KeyPoint keypoint) {
  for (size_t i = 0; i < trigged_keypoint_list_.size(); ++i) {
    if (keypoint.pt.inside(trigged_keypoint_list_[i].bounding_box)) {
      trigged_keypoint_list_.erase(trigged_keypoint_list_.begin() + i);
    }
  }
}

//------------------------------------------------------------------------------
//
inline void FenceDetector::AddWeightToCorrespondingTriggedKeypoint(
    cv::Point2d trigged_keypoint, int weight) {
  for (size_t i = 0; i < trigged_keypoint_list_.size(); ++i) {
    if (trigged_keypoint.inside(trigged_keypoint_list_[i].bounding_box)) {
      if (trigged_keypoint_list_[i].weight + weight <=
          weight_goal_.GetValue()) {
        trigged_keypoint_list_[i].weight += weight;
      } else {
        trigged_keypoint_list_[i].weight +=
            (trigged_keypoint_list_[i].weight + weight) -
            weight_goal_.GetValue();
      }
    }
  }
}

//------------------------------------------------------------------------------
//
inline void FenceDetector::RemoveWeightToCorrespondingTriggedKeypoint(
    cv::Point2d trigged_keypoint, int weight) {
  for (size_t i = 0; i < trigged_keypoint_list_.size(); ++i) {
    if (trigged_keypoint.inside(trigged_keypoint_list_[i].bounding_box)) {
      if (trigged_keypoint_list_[i].weight - weight > 0) {
        trigged_keypoint_list_[i].weight -= weight;
      } else {
        trigged_keypoint_list_[i].weight = 0;
      }
    }
  }
}

//------------------------------------------------------------------------------
//
inline bool FenceDetector::IsKeypointHasEnoughWeight(
    FenceDetector::TriggedKeypoint keypoint) {
  if (keypoint.weight > weight_goal_.GetValue()) {
    return true;
  }
  return false;
}

//------------------------------------------------------------------------------
//
inline void FenceDetector::SetWeigthGoal(int weight_goal) {
  weight_goal_.SetValue(weight_goal);
}

//------------------------------------------------------------------------------
//
inline cv::Rect FenceDetector::SetBoundingBox(cv::Point2d keypoint,
                                              int box_size) {
  std::vector<cv::Point> rect;
  rect.push_back(cv::Point2d(keypoint.x + box_size, keypoint.y + box_size));
  rect.push_back(cv::Point2d(keypoint.x + box_size, keypoint.y - box_size));
  rect.push_back(cv::Point2d(keypoint.x - box_size, keypoint.y + box_size));
  rect.push_back(cv::Point2d(keypoint.x - box_size, keypoint.y - box_size));
  return cv::boundingRect(rect);
}

}  // namespace proc_mapping

#endif  // PROC_MAPPING_FENCE_DETECTOR_H
