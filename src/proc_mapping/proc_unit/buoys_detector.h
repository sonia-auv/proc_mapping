/**
 * \file	buoys_detector.h
 * \author	Francis Masse <francis.masse05@gmail.com>
 * \date	10/06/2016
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

#ifndef PROC_MAPPING_BUOYS_DETECTOR_H_
#define PROC_MAPPING_BUOYS_DETECTOR_H_

#include <opencv/cv.h>
#include "proc_mapping/proc_unit/proc_unit.h"

namespace proc_mapping {

class BuoysDetector : public ProcUnit {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<BuoysDetector>;
  using ConstPtr = std::shared_ptr<const BuoysDetector>;
  using PtrList = std::vector<BuoysDetector::Ptr>;
  using ConstPtrList = std::vector<BuoysDetector::ConstPtr>;

  struct Keypoint {
    cv::KeyPoint trigged_keypoint;
    cv::Rect bounding_box;
    uint8_t weight;
    bool is_object_send;
  };

  //==========================================================================
  // P U B L I C   C / D T O R S

  BuoysDetector(){};

  virtual ~BuoysDetector() = default;

  //==========================================================================
  // P U B L I C   M E T H O D S

  virtual boost::any ProcessData(boost::any input) override {
    auto map_objects = boost::any_cast<std::vector<cv::KeyPoint>>(input);

    for (size_t i = 0; i < map_objects.size(); i++) {
      bool is_already_trigged = IsAlreadyTrigged(map_objects[i]);

      if (!is_already_trigged) {
        Keypoint trigged_keypoint;
        trigged_keypoint.trigged_keypoint = map_objects[i];
        trigged_keypoint.bounding_box = SetBoundingBox(map_objects[i].pt, 20);
        trigged_keypoint.is_object_send = false;
        trigged_keypoints_.push_back(trigged_keypoint);
      }

      if (trigged_keypoints_.size() >= 3) {
        for (uint32_t k = 0; k < trigged_keypoints_.size(); ++k) {
          if (!trigged_keypoints_.at(k).is_object_send) {
            MapObject::Ptr map_object =
                std::make_shared<Buoy>(trigged_keypoints_[i].trigged_keypoint);
            ObjectRegistery::GetInstance().AddObject(std::move(map_object));
          }
        }
      }
    }
    // We are supposed to be the last PU in the pipeline, so let's return a
    // void boost any.
    return boost::any();
  }

 private:
  //------------------------------------------------------------------------------
  //
  inline bool IsAlreadyTrigged(cv::KeyPoint map_object) {
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
  inline double GetDistanceBewteenKeypoint(cv::Point2d p1, cv::Point2d p2) {
    //    cv::Point2d world_p1 = raw_map_->PixelToWorldCoordinates(p1);
    //    cv::Point2d world_p2 = raw_map_->PixelToWorldCoordinates(p2);
    //    double delta_x = (world_p1.x - world_p2.x);
    //    double delta_y = (world_p1.y - world_p2.y);
    //    return sqrt((delta_x * delta_x) + (delta_y * delta_y));
    return 0;
  }

  //------------------------------------------------------------------------------
  //
  inline cv::Rect SetBoundingBox(cv::Point2d keypoint, int box_size) {
    std::vector<cv::Point> rect;
    rect.push_back(cv::Point2d(keypoint.x + box_size, keypoint.y + box_size));
    rect.push_back(cv::Point2d(keypoint.x + box_size, keypoint.y - box_size));
    rect.push_back(cv::Point2d(keypoint.x - box_size, keypoint.y + box_size));
    rect.push_back(cv::Point2d(keypoint.x - box_size, keypoint.y - box_size));
    return cv::boundingRect(rect);
  }

  std::vector<Keypoint> trigged_keypoints_;
};

}  // namespace proc_mapping

#endif  // PROC_MAPPING_BUOYS_DETECTOR_H_
