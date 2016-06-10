///**
// * \file	buoys_detector.h
// * \author	Francis Masse <francis.masse05@gmail.com>
// * \date	10/06/2016
// *
// * \copyright Copyright (c) 2015 S.O.N.I.A. All rights reserved.
// *
// * \section LICENSE
// *
// * This file is part of S.O.N.I.A. software.
// *
// * S.O.N.I.A. software is free software: you can redistribute it and/or modify
// * it under the terms of the GNU General Public License as published by
// * the Free Software Foundation, either version 3 of the License, or
// * (at your option) any later version.
// *
// * S.O.N.I.A. software is distributed in the hope that it will be useful,
// * but WITHOUT ANY WARRANTY; without even the implied warranty of
// * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// * GNU General Public License for more details.
// *
// * You should have received a copy of the GNU General Public License
// * along with S.O.N.I.A. software. If not, see <http://www.gnu.org/licenses/>.
// */
//
//#ifndef PROC_MAPPING_BUOYS_DETECTOR_H
//#define PROC_MAPPING_BUOYS_DETECTOR_H
//
//#include <opencv2/features2d/features2d.hpp>
//namespace proc_mapping {
//
//class BuoysDetector : public ProcUnit<cv::Mat> {
// public:
//  //==========================================================================
//  // T Y P E D E F   A N D   E N U M
//
//  using Ptr = std::shared_ptr<BuoysDetector>;
//  using ConstPtr = std::shared_ptr<const BuoysDetector>;
//  using PtrList = std::vector<BuoysDetector::Ptr>;
//  using ConstPtrList = std::vector<BuoysDetector::ConstPtr>;
//
//  struct Keypoint {
//    cv::KeyPoint trigged_keypoint;
//    cv::Rect bounding_box;
//    uint8_t weight;
//    bool is_object_send;
//  };
//
//  //==========================================================================
//  // P U B L I C   C / D T O R S
//
//  BuoysDetector() {};
//
//  virtual ~BuoysDetector() = default;
//
//  //==========================================================================
//  // P U B L I C   M E T H O D S
//
//  virtual void ProcessData(cv::KeyPoint &input) override {
//    for (size_t i = 0; i < input.size(); i++) {
//      double distance_to_right_blob =
//          GetDistanceBewteenKeypoint(map_objects[i]->GetCvKeyPoint().pt,
//                                     map_objects[i + 1]->GetCvKeyPoint().pt);
//      double distance_to_left_blob =
//          GetDistanceBewteenKeypoint(map_objects[i]->GetCvKeyPoint().pt,
//                                     map_objects[i - 1]->GetCvKeyPoint().pt);
//
//      if ((distance_to_right_blob > 1.0f and distance_to_right_blob < 1.6f) or
//          (distance_to_left_blob > 1.0f and distance_to_left_blob < 1.6f)) {
//        bool is_already_trigged =
//            IsAlreadyTrigged(map_objects[i]->GetCvKeyPoint());
//
//        if (!is_already_trigged) {
//          Keypoint trigged_keypoint;
//          trigged_keypoint.trigged_keypoint = map_objects[i]->GetCvKeyPoint();
//          trigged_keypoint.bounding_box =
//              SetBoundingBox(map_objects[i]->GetCvKeyPoint().pt, 20);
//          trigged_keypoint.is_object_send = false;
//          //        if (rois_.size() > 0) {
//          //          if
//          //          (map_objects[i].pt.inside(rois_.at(0).GetCvBoundingRect()))
//          //          {
//          //            trigged_keypoint.weight += 10;
//          //          } else {
//          //            trigged_keypoint.weight += 1;
//          //          }
//          //        }
//          trigged_keypoints_.push_back(trigged_keypoint);
//        }
//
//        if (trigged_keypoints_.size() >= 3) {
//          for (uint32_t k = 0; k < trigged_keypoints_.size(); ++k) {
//            if (!trigged_keypoints_.at(k).is_object_send) {
//              sonia_msgs::MapObject obj;
//
//              cv::Point2d offset = raw_map_->GetPositionOffset();
//              cv::Point2d world_point = raw_map_->PixelToWorldCoordinates(
//                  trigged_keypoints_.at(k).trigged_keypoint.pt);
//              obj.name = "Buoy [" + std::to_string(k) + "]";
//              obj.size = trigged_keypoints_.at(k).trigged_keypoint.size;
//              obj.pose.x = world_point.x - offset.x;
//              obj.pose.y = world_point.y - offset.y;
//              obj.type = sonia_msgs::MapObject::BUOYS;
//              trigged_keypoints_.at(k).is_object_send = true;
//
//              // Finally adding the object to the semantic map
//              map_objects_.push_back(std::move(obj));
//              new_objects_available_ = true;
//            }
//          }
//        }
//      }
//    }
//  }
//
// private:
////------------------------------------------------------------------------------
////
//  inline bool IsAlreadyTrigged(cv::KeyPoint map_object) {
//    for (uint32_t k = 0; k < trigged_keypoints_.size(); ++k) {
//      if (map_object.pt.inside(trigged_keypoints_.at(k).bounding_box)) {
//        trigged_keypoints_.at(k).weight++;
//        return true;
//      }
//    }
//    return false;
//  }
//
////------------------------------------------------------------------------------
////
//  inline double GetDistanceBewteenKeypoint(cv::Point2d p1, cv::Point2d p2) {
//    cv::Point2d world_p1 = raw_map_->PixelToWorldCoordinates(p1);
//    cv::Point2d world_p2 = raw_map_->PixelToWorldCoordinates(p2);
//    double delta_x = (world_p1.x - world_p2.x);
//    double delta_y = (world_p1.y - world_p2.y);
//    return sqrt((delta_x * delta_x) + (delta_y * delta_y));
//  }
//
////------------------------------------------------------------------------------
////
//  inline cv::Rect SetBoundingBox(cv::Point2d keypoint, int box_size) {
//    std::vector<cv::Point> rect;
//    rect.push_back(cv::Point2d(keypoint.x + box_size, keypoint.y + box_size));
//    rect.push_back(cv::Point2d(keypoint.x + box_size, keypoint.y - box_size));
//    rect.push_back(cv::Point2d(keypoint.x - box_size, keypoint.y + box_size));
//    rect.push_back(cv::Point2d(keypoint.x - box_size, keypoint.y - box_size));
//    return cv::boundingRect(rect);
//  }
//
//  std::vector<Keypoint> trigged_keypoints_;
//
//};
//
//}  // namespace proc_mapping
//
//#endif //PROC_MAPPING_BUOYS_DETECTOR_H
