/**
 * \file	far_buoys_detector.h
 * \author	Francis Masse <francis.masse05@gmail.com>
 * \date	19/06/2016
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

#ifndef PROC_MAPPING_PIPELINE_PROC_UNIT_FAR_BUOYS_DETECTOR_H_
#define PROC_MAPPING_PIPELINE_PROC_UNIT_FAR_BUOYS_DETECTOR_H_

#include <opencv/cv.h>
#include <proc_mapping/map/coordinate_systems.h>
#include <proc_mapping/region_of_interest/rotated_rectangle.h>
#include "proc_mapping/pipeline/proc_unit.h"

namespace proc_mapping {

class FarBuoysDetector : public ProcUnit {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<FarBuoysDetector>;
  using ConstPtr = std::shared_ptr<const FarBuoysDetector>;
  using PtrList = std::vector<FarBuoysDetector::Ptr>;
  using ConstPtrList = std::vector<FarBuoysDetector::ConstPtr>;

  struct TriggedKeypoint {
    cv::KeyPoint trigged_keypoint;
    cv::Rect bounding_box;
    uint8_t weight;
    bool is_object_send;
  };

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit FarBuoysDetector(const std::string &topic_namespace, const
  ObjectRegistery::Ptr &object_registery);

  virtual ~FarBuoysDetector() = default;

  //==========================================================================
  // P U B L I C   M E T H O D S

  virtual void ConfigureFromYamlNode(const YAML::Node &node) override;

  virtual boost::any ProcessData(boost::any input) override;

  std::string GetName() const override;

 private:
  //==========================================================================
  // P R I V A T E   M E T H O D S

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

  std::vector<TriggedKeypoint> trigged_keypoint_list_;
  ObjectRegistery::Ptr object_registery_;
};

//==============================================================================
// I N L I N E   M E T H O D S

//------------------------------------------------------------------------------
//
inline FarBuoysDetector::FarBuoysDetector(const std::string &topic_namespace, const
ObjectRegistery::Ptr &object_registery)
    : ProcUnit(topic_namespace),
      weight_goal_("Weight Goal", 0, parameters_),
      object_registery_(object_registery) {}

//------------------------------------------------------------------------------
//
inline void FarBuoysDetector::ConfigureFromYamlNode(const YAML::Node &node) {
  weight_goal_ = node["weight_goal"].as<int>();
}

//------------------------------------------------------------------------------
//
inline boost::any FarBuoysDetector::ProcessData(boost::any input) {
  auto keypoint = boost::any_cast<std::vector<cv::KeyPoint>>(input);
  auto rois =
      object_registery_->GetRegionOfInterestOfType(DetectionMode::BUOYS);
  SetWeigthGoal(150);

  if (object_registery_->IsRegisteryCleared()) {
    trigged_keypoint_list_.clear();
    object_registery_->ResetRegisteryClearedFlag();
  }

  bool added_new_object = false;
  for (size_t i = 0; i < keypoint.size(); i++) {
    bool is_already_trigged = IsAlreadyTrigged(keypoint[i]);

    if (!is_already_trigged) {
      AddToTriggeredList(keypoint[i]);
    } else {
      for (size_t j = 0; j < trigged_keypoint_list_.size(); ++j) {
        AddWeightToCorrespondingTriggedKeypoint(
            trigged_keypoint_list_[j].trigged_keypoint.pt, 1);
      }
    }
  }

  for (size_t j = 0; j < trigged_keypoint_list_.size(); ++j) {
    if (IsKeypointHasEnoughWeight(trigged_keypoint_list_[j])) {
      if (!trigged_keypoint_list_[j].is_object_send) {
        MapObject::Ptr map_object = std::make_shared<Buoy>(
            trigged_keypoint_list_[j].trigged_keypoint);
        map_object->SetName("Buoy [" + std::to_string(j) + "]");
        map_object->SetSize(trigged_keypoint_list_[j].trigged_keypoint.size);
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
inline std::string FarBuoysDetector::GetName() const { return "far_buoys_detector"; }

//------------------------------------------------------------------------------
//
bool FarBuoysDetector::IsAlreadyTrigged(cv::KeyPoint keypoint) {
  for (size_t i = 0; i < trigged_keypoint_list_.size(); ++i) {
    if (keypoint.pt.inside(trigged_keypoint_list_.at(i).bounding_box)) {
      return true;
    }
  }
  return false;
}

//------------------------------------------------------------------------------
//
inline void FarBuoysDetector::AddToTriggeredList(cv::KeyPoint keypoint) {
  TriggedKeypoint trigged_keypoint;
  trigged_keypoint.trigged_keypoint = keypoint;
  trigged_keypoint.bounding_box = SetBoundingBox(keypoint.pt, 20);
  trigged_keypoint.is_object_send = false;
  trigged_keypoint.weight = 0;
  trigged_keypoint_list_.push_back(trigged_keypoint);
}

//------------------------------------------------------------------------------
//
inline void FarBuoysDetector::RemoveToTriggeredList(cv::KeyPoint keypoint) {
  for (size_t i = 0; i < trigged_keypoint_list_.size(); ++i) {
    if (keypoint.pt.inside(trigged_keypoint_list_[i].bounding_box)) {
      trigged_keypoint_list_.erase(trigged_keypoint_list_.begin() + i);
    }
  }
}

//------------------------------------------------------------------------------
//
inline void
FarBuoysDetector::AddWeightToCorrespondingTriggedKeypoint(cv::Point2d trigged_keypoint,
                                                          int weight) {
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
inline void
FarBuoysDetector::RemoveWeightToCorrespondingTriggedKeypoint(cv::Point2d trigged_keypoint,
                                                             int weight) {
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
inline bool
FarBuoysDetector::IsKeypointHasEnoughWeight(FarBuoysDetector::TriggedKeypoint keypoint) {
  if (keypoint.weight > weight_goal_.GetValue()) {
    return true;
  }
  return false;
}

//------------------------------------------------------------------------------
//
inline void FarBuoysDetector::SetWeigthGoal(int weight_goal) {
  weight_goal_.SetValue(weight_goal);
}

//------------------------------------------------------------------------------
//
inline cv::Rect FarBuoysDetector::SetBoundingBox(cv::Point2d keypoint, int box_size) {
  std::vector<cv::Point> rect;
  rect.push_back(cv::Point2d(keypoint.x + box_size, keypoint.y + box_size));
  rect.push_back(cv::Point2d(keypoint.x + box_size, keypoint.y - box_size));
  rect.push_back(cv::Point2d(keypoint.x - box_size, keypoint.y + box_size));
  rect.push_back(cv::Point2d(keypoint.x - box_size, keypoint.y - box_size));
  return cv::boundingRect(rect);
}

}  // namespace proc_mapping

#endif  // PROC_MAPPING_PIPELINE_PROC_UNIT_FAR_BUOYS_DETECTOR_H
