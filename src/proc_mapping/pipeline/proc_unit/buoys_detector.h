/**
 * \file	buoys_detector.h
 * \author	Francis Masse <francis.masse05@gmail.com>
 * \date	10/06/2016
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

#ifndef PROC_MAPPING_PIPELINE_PROC_UNIT_BUOYS_DETECTOR_H_
#define PROC_MAPPING_PIPELINE_PROC_UNIT_BUOYS_DETECTOR_H_

#include <opencv/cv.h>
#include <proc_mapping/map/coordinate_systems.h>
#include <proc_mapping/region_of_interest/rotated_rectangle.h>
#include "proc_mapping/pipeline/proc_unit.h"

namespace proc_mapping {

class BuoysDetector : public ProcUnit {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<BuoysDetector>;
  using ConstPtr = std::shared_ptr<const BuoysDetector>;
  using PtrList = std::vector<BuoysDetector::Ptr>;
  using ConstPtrList = std::vector<BuoysDetector::ConstPtr>;

  struct TriggedKeypoint {
    cv::KeyPoint trigged_keypoint;
    cv::Rect bounding_box;
    cv::Scalar mean;
    uint8_t weight;
  };

  struct Candidate {
    cv::KeyPoint trigged_keypoint;
    cv::Rect bounding_box;
    uint8_t weight;
    bool is_object_send;
  };

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit BuoysDetector(const std::string &topic_namespace,
                         const ObjectRegistery::Ptr &object_registery);

  virtual ~BuoysDetector() = default;

  //==========================================================================
  // P U B L I C   M E T H O D S

  virtual void ConfigureFromYamlNode(const YAML::Node &node) override;

  virtual boost::any ProcessData(boost::any input) override;

  std::string GetName() const override;

 private:
  //==========================================================================
  // PRIVATE   M E T H O D S

  inline bool IsAlreadyTrigged(cv::KeyPoint keypoint);

  inline bool IsAlreadyCandidate(TriggedKeypoint trigged_keypoint);

  inline bool IsCandidateHasEnoughWeight(Candidate candidate);

  inline void AddToTriggeredList(cv::KeyPoint keypoint);

  inline void RemoveToTriggeredList(cv::KeyPoint keypoint);

  inline void AddToCandidateList(TriggedKeypoint trigged_keypoint);

  inline void RemoveToCandidateList(cv::KeyPoint keypoint);

  inline double GetDistanceBewteenKeypoint(cv::Point2d p1, cv::Point2d p2);

  inline bool HasBlobInGoodRange(TriggedKeypoint trigged_keypoint,
                                 double low_range, double high_range);

  inline bool HasTwoBlobInGoodRange(TriggedKeypoint trigged_keypoint,
                                    double low_range, double high_range);

  inline bool BoxMean(TriggedKeypoint trigged_keypoint, double reached_mean);

  inline void AddWeightToCorrespondingTriggedKeypoint(
      cv::Point2d trigged_keypoint, int weight);

  inline void RemoveWeightToCorrespondingTriggedKeypoint(
      cv::Point2d trigged_keypoint, int weight);

  inline void AddWeightToCorrespondingCandidate(cv::Point2d candidate,
                                                int weight);

  inline void RemoveWeightToCorrespondingCandidate(cv::Point2d trigged_keypoint,
                                                   int weight);

  inline void SetWeigthGoal(int weight_goal);

  inline cv::Rect SetBoundingBox(cv::Point2d keypoint, int box_size);

  //==========================================================================
  // P R I V A T E   M E M B E R S

  Parameter<int> weight_goal_;
  Parameter<bool> roi_needed_;

  std::vector<TriggedKeypoint> trigged_keypoint_list_;
  std::vector<Candidate> candidate_list_;
  ObjectRegistery::Ptr object_registery_;
};

//==============================================================================
// I N L I N E   M E T H O D S

//------------------------------------------------------------------------------
//
inline void BuoysDetector::ConfigureFromYamlNode(const YAML::Node &node) {
  weight_goal_ = node["weight_goal"].as<int>();
  roi_needed_ = node["roi_needed"].as<bool>();
}

//------------------------------------------------------------------------------
//
inline BuoysDetector::BuoysDetector(
    const std::string &topic_namespace,
    const ObjectRegistery::Ptr &object_registery)
    : ProcUnit(topic_namespace),
      weight_goal_("Weight Goal", 0, parameters_),
      roi_needed_("ROI Needed", false, parameters_),
      object_registery_(object_registery) {}

//------------------------------------------------------------------------------
//
inline cv::Rect BuoysDetector::SetBoundingBox(cv::Point2d keypoint,
                                              int box_size) {
  std::vector<cv::Point> rect;
  rect.push_back(cv::Point2d(keypoint.x + box_size, keypoint.y + box_size));
  rect.push_back(cv::Point2d(keypoint.x + box_size, keypoint.y - box_size));
  rect.push_back(cv::Point2d(keypoint.x - box_size, keypoint.y + box_size));
  rect.push_back(cv::Point2d(keypoint.x - box_size, keypoint.y - box_size));
  return cv::boundingRect(rect);
}

//------------------------------------------------------------------------------
//
inline void BuoysDetector::SetWeigthGoal(int weight_goal) {
  weight_goal_.SetValue(weight_goal);
}

//------------------------------------------------------------------------------
//
inline void BuoysDetector::RemoveWeightToCorrespondingCandidate(
    cv::Point2d trigged_keypoint, int weight) {
  for (size_t i = 0; i < candidate_list_.size(); ++i) {
    if (trigged_keypoint.inside(candidate_list_[i].bounding_box)) {
      if (candidate_list_[i].weight - weight > 0) {
        candidate_list_[i].weight -= weight;
      } else {
        candidate_list_[i].weight = 0;
      }
    }
  }
}

//------------------------------------------------------------------------------
//
inline void BuoysDetector::AddWeightToCorrespondingCandidate(
    cv::Point2d candidate, int weight) {
  for (size_t i = 0; i < candidate_list_.size(); ++i) {
    if (candidate.inside(candidate_list_[i].bounding_box)) {
      if (candidate_list_[i].weight + weight <= weight_goal_.GetValue()) {
        candidate_list_[i].weight += weight;
      } else {
        candidate_list_[i].weight +=
            (candidate_list_[i].weight + weight) - weight_goal_.GetValue();
      }
    }
  }
}

//------------------------------------------------------------------------------
//
inline void BuoysDetector::RemoveWeightToCorrespondingTriggedKeypoint(
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
inline void BuoysDetector::AddWeightToCorrespondingTriggedKeypoint(
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
inline bool BuoysDetector::BoxMean(
    BuoysDetector::TriggedKeypoint trigged_keypoint, double reached_mean) {
  cv::Mat box(trigged_keypoint.bounding_box.height,
              trigged_keypoint.bounding_box.width, CV_8UC1);
  cv::Scalar actual_mean = cv::mean(box);

  if (actual_mean[0] <= reached_mean) {
    trigged_keypoint.mean = actual_mean;
    return true;
  }

  trigged_keypoint.mean = actual_mean;
  return false;
}

//------------------------------------------------------------------------------
//
inline bool BuoysDetector::HasTwoBlobInGoodRange(
    BuoysDetector::TriggedKeypoint trigged_keypoint, double low_range,
    double high_range) {
  if (candidate_list_.size() > 2) {
    for (size_t i = 0; i < candidate_list_.size(); ++i) {
      double distance =
          GetDistanceBewteenKeypoint(trigged_keypoint.trigged_keypoint.pt,
                                     candidate_list_[i].trigged_keypoint.pt);
      if (distance >= low_range * 40 and distance <= high_range * 40) {
        for (size_t j = 0; j < candidate_list_.size(); ++j) {
          if (j != i) {
            double distance2 = GetDistanceBewteenKeypoint(
                trigged_keypoint.trigged_keypoint.pt,
                candidate_list_[j].trigged_keypoint.pt);
            if (distance2 >= low_range * 40 and distance2 <= high_range * 40) {
              return true;
            }
          }
        }
      }
    }
  }
  return false;
}

//------------------------------------------------------------------------------
//
inline bool BuoysDetector::HasBlobInGoodRange(
    BuoysDetector::TriggedKeypoint trigged_keypoint, double low_range,
    double high_range) {
  for (size_t i = 0; i < trigged_keypoint_list_.size(); ++i) {
    double distance = GetDistanceBewteenKeypoint(
        trigged_keypoint.trigged_keypoint.pt,
        trigged_keypoint_list_[i].trigged_keypoint.pt);
    if (distance >= low_range * 40 and distance <= high_range * 40) {
      return true;
    }
  }
  return false;
}

//------------------------------------------------------------------------------
//
inline double BuoysDetector::GetDistanceBewteenKeypoint(cv::Point2d p1,
                                                        cv::Point2d p2) {
  double delta_x = (p1.x - p2.x);
  double delta_y = (p1.y - p2.y);
  return sqrt((delta_x * delta_x) + (delta_y * delta_y));
}

//------------------------------------------------------------------------------
//
inline void BuoysDetector::RemoveToCandidateList(cv::KeyPoint keypoint) {
  for (size_t i = 0; i < candidate_list_.size(); ++i) {
    if (keypoint.pt.inside(candidate_list_[i].bounding_box)) {
      candidate_list_.erase(candidate_list_.begin() + i);
    }
  }
}

//------------------------------------------------------------------------------
//
inline void BuoysDetector::AddToCandidateList(
    BuoysDetector::TriggedKeypoint trigged_keypoint) {
  Candidate candidate;
  candidate.trigged_keypoint = trigged_keypoint.trigged_keypoint;
  candidate.bounding_box =
      SetBoundingBox(trigged_keypoint.trigged_keypoint.pt, 20);
  candidate.weight = trigged_keypoint.weight;
  candidate.is_object_send = false;
  candidate_list_.push_back(candidate);
}

//------------------------------------------------------------------------------
//
inline void BuoysDetector::RemoveToTriggeredList(cv::KeyPoint keypoint) {
  for (size_t i = 0; i < trigged_keypoint_list_.size(); ++i) {
    if (keypoint.pt.inside(trigged_keypoint_list_[i].bounding_box)) {
      trigged_keypoint_list_.erase(trigged_keypoint_list_.begin() + i);
    }
  }
}

//------------------------------------------------------------------------------
//
inline void BuoysDetector::AddToTriggeredList(cv::KeyPoint keypoint) {
  TriggedKeypoint trigged_keypoint;
  trigged_keypoint.trigged_keypoint = keypoint;
  trigged_keypoint.bounding_box = SetBoundingBox(keypoint.pt, 20);
  cv::Mat box(trigged_keypoint.bounding_box.height,
              trigged_keypoint.bounding_box.width, CV_8UC1);
  trigged_keypoint.mean = cv::mean(box);
  trigged_keypoint.weight = 0;
  trigged_keypoint_list_.push_back(trigged_keypoint);
}

//------------------------------------------------------------------------------
//
inline bool BuoysDetector::IsCandidateHasEnoughWeight(
    BuoysDetector::Candidate candidate) {
  if (candidate.weight > weight_goal_.GetValue()) {
    return true;
  }
  return false;
}

//------------------------------------------------------------------------------
//
inline bool BuoysDetector::IsAlreadyCandidate(
    BuoysDetector::TriggedKeypoint trigged_keypoint) {
  for (size_t i = 0; i < candidate_list_.size(); ++i) {
    if (trigged_keypoint.trigged_keypoint.pt.inside(
            candidate_list_.at(i).bounding_box)) {
      return true;
    }
  }
  return false;
}

//------------------------------------------------------------------------------
//
inline bool BuoysDetector::IsAlreadyTrigged(cv::KeyPoint keypoint) {
  for (size_t i = 0; i < trigged_keypoint_list_.size(); ++i) {
    if (keypoint.pt.inside(trigged_keypoint_list_.at(i).bounding_box)) {
      return true;
    }
  }
  return false;
}

//------------------------------------------------------------------------------
//
inline std::string BuoysDetector::GetName() const { return "buoys_detector"; }

//------------------------------------------------------------------------------
//
inline boost::any BuoysDetector::ProcessData(boost::any input) {
  auto keypoint = boost::any_cast<std::vector<cv::KeyPoint>>(input);
  auto rois =
      object_registery_->GetRegionOfInterestOfType(DetectionMode::BUOYS);

  if (object_registery_->IsRegisteryCleared()) {
    trigged_keypoint_list_.clear();
    candidate_list_.clear();
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
        if (HasBlobInGoodRange(trigged_keypoint_list_[j], 0.8, 1.6)) {
          if (!IsAlreadyCandidate(trigged_keypoint_list_[j])) {
            for (const auto &roi : rois) {
              if (roi->IsInZone(
                      trigged_keypoint_list_[j].trigged_keypoint.pt)) {
                AddToCandidateList(trigged_keypoint_list_[j]);
                AddWeightToCorrespondingCandidate(
                    trigged_keypoint_list_[j].trigged_keypoint.pt, 5);
              } else {
                if (!roi_needed_.GetValue()) {
                  AddToCandidateList(trigged_keypoint_list_[j]);
                  AddWeightToCorrespondingCandidate(
                      trigged_keypoint_list_[j].trigged_keypoint.pt, 1);
                }
              }
            }
          } else {
            if (HasTwoBlobInGoodRange(trigged_keypoint_list_[j], 0.8, 2.6)) {
              AddWeightToCorrespondingCandidate(
                  trigged_keypoint_list_[j].trigged_keypoint.pt, 1);
            } else {
              RemoveWeightToCorrespondingCandidate(
                  trigged_keypoint_list_[j].trigged_keypoint.pt, 1);
              if (trigged_keypoint_list_[j].weight == 0) {
                RemoveToCandidateList(
                    trigged_keypoint_list_[j].trigged_keypoint);
              }
            }
          }
        } else {
          if (IsAlreadyCandidate(trigged_keypoint_list_[j])) {
            RemoveWeightToCorrespondingCandidate(
                trigged_keypoint_list_[j].trigged_keypoint.pt, 5);
            if (trigged_keypoint_list_[j].weight == 0) {
              RemoveToCandidateList(trigged_keypoint_list_[j].trigged_keypoint);
            }
          } else {
            RemoveWeightToCorrespondingTriggedKeypoint(
                trigged_keypoint_list_[j].trigged_keypoint.pt, 5);
            if (trigged_keypoint_list_[j].weight == 0) {
              RemoveToTriggeredList(trigged_keypoint_list_[j].trigged_keypoint);
            }
          }
        }
      }
    }

    for (size_t j = 0; j < candidate_list_.size(); ++j) {
      if (IsCandidateHasEnoughWeight(candidate_list_[j])) {
        if (!candidate_list_[j].is_object_send) {
          MapObject::Ptr map_object =
              std::make_shared<Buoy>(candidate_list_[j].trigged_keypoint);
          map_object->SetName("Buoy [" + std::to_string(j) + "]");
          map_object->SetSize(candidate_list_[j].trigged_keypoint.size);
          object_registery_->AddMapObject(std::move(map_object));
          candidate_list_[j].is_object_send = true;
          added_new_object = true;
        }
      }
    }
  }
  // We are supposed to be the last PU in the pipeline, so let's return a
  // void boost any.
  return boost::any(added_new_object);
}

}  // namespace proc_mapping

#endif  // PROC_MAPPING_BUOYS_DETECTOR_H_
