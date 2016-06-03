/**
 * \file	pattern_detection.h
 * \author	Francis Masse <francis.masse05@gmail.com>
 * \date	18/05/2016
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

#ifndef PROC_MAPPING_PROC_UNIT_BLOB_DETECTOR_H_
#define PROC_MAPPING_PROC_UNIT_BLOB_DETECTOR_H_

#include <opencv/cv.h>
#include <ros/ros.h>
#include <sonia_msgs/ObstacleTemplate.h>
#include <memory>

#include <highgui.h>
#include "proc_mapping/interpreter/object_registery.h"
#include "proc_mapping/proc_unit/proc_unit.h"

namespace proc_mapping {

int filter_area_off = 1;
const int filter_area_on = 1;
int min_area = 0;
const int min_area_max = 3000;
int max_area = 0;
const int max_area_max = 3000;
int filter_circularity_off = 0;
const int filter_circularity_on = 1;
int min_circularity = 0;
const int min_circularity_max = 100;
int max_circularity = 0;
const int max_circularity_max = 100;
int filter_convexity_off = 0;
const int filter_convexity_on = 1;
int min_convexity = 0;
const int min_convexity_max = 10;
int max_convexity = 0;
const int max_convexity_max = 10;
int filter_inertial_off = 0;
const int filter_inertial_on = 1;
int min_inertia_ratio = 0;
const int min_inertia_ratio_max = 10;
int max_inertia_ratio = 0;
const int max_inertia_ratio_max = 10;

class BlobDetector : public ProcUnit<cv::Mat> {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<BlobDetector>;
  using ConstPtr = std::shared_ptr<const BlobDetector>;
  using PtrList = std::vector<BlobDetector::Ptr>;
  using ConstPtrList = std::vector<BlobDetector::ConstPtr>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  BlobDetector(){};

  BlobDetector(const ros::NodeHandlePtr &nh, bool debug)
      : nh_(nh), debug_(debug) {
    obstacle_server_ = nh_->advertiseService(
        "obstacle", &BlobDetector::ObstacleTemplate, this);
  }
  virtual ~BlobDetector() = default;

  //==========================================================================
  // P U B L I C   M E T H O D S

  // Method use to change the obstacle template
  bool ObstacleTemplate(sonia_msgs::ObstacleTemplate::Request &req,
                        sonia_msgs::ObstacleTemplate::Response &resp) {
    obstacle_ = req.obstacle_template;
    return true;
  }

  virtual void ProcessData(cv::Mat &input) override {
    cv::createTrackbar("area filter", "Blob Detector", &filter_area_off,
                       filter_area_on);
    cv::createTrackbar("min area", "Blob Detector", &min_area, min_area_max);
    cv::createTrackbar("max area", "Blob Detector", &max_area, max_area_max);
    //    cv::createTrackbar("circularity filter", "Blob Detector",
    //    &filter_circularity_off, filter_circularity_on);
    //    cv::createTrackbar("min circularity", "Blob Detector",
    //    &min_circularity, min_circularity_max);
    //    cv::createTrackbar("max circularity", "Blob Detector",
    //    &max_circularity, max_circularity_max);
    //    cv::createTrackbar("convexity filter", "Blob Detector",
    //    &filter_convexity_off, filter_convexity_on);
    //    cv::createTrackbar("min convexity", "Blob Detector", &min_convexity,
    //    min_convexity_max);
    //    cv::createTrackbar("max convexity", "Blob Detector", &max_convexity,
    //    max_convexity_max);
    //    cv::createTrackbar("inertia filter", "Blob Detector",
    //    &filter_inertial_off, filter_inertial_on);
    //    cv::createTrackbar("min inertia", "Blob Detector", &min_inertia_ratio,
    //    min_inertia_ratio_max);
    //    cv::createTrackbar("max inertia", "Blob Detector", &max_inertia_ratio,
    //    max_inertia_ratio_max);

    if (obstacle_.compare("buoy") == 0) {
      params_.filterByArea = true;
      params_.minArea = 378;
      params_.maxArea = 526;
    } else {
      params_.minThreshold = 0;
      params_.maxThreshold = 255;
      // Filter by Area.
      params_.filterByArea = filter_area_off;
      params_.minArea = min_area;
      params_.maxArea = max_area;
      //// Filter by Circularity
      params_.filterByCircularity = filter_circularity_off;
      params_.minCircularity = min_circularity / 100;
      params_.maxCircularity = max_circularity / 100;
      params_.filterByColor = false;
      // Filter by Convexity
      params_.filterByConvexity = filter_convexity_off;
      params_.minConvexity = min_convexity / 10;
      params_.maxConvexity = max_convexity / 10;
      // Filter by Inertia
      params_.filterByInertia = filter_inertial_off;
      params_.minInertiaRatio = min_inertia_ratio / 10;
      params_.maxInertiaRatio = max_inertia_ratio / 10;
    }
    cv::SimpleBlobDetector detector(params_);
    std::vector<cv::KeyPoint> keyPoints;
    detector.detect(input, keyPoints);

    for (auto &key_point : keyPoints) {
      ObjectRegistery::GetInstance().AddObject(key_point);
    }

    //    for (int i = 0; i < keyPoints.size(); ++i) {
    //      cv::putText(input, std::to_string(i), keyPoints[i].pt,
    //                  cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 1, cv::Scalar(255));
    //    }

    //    // The fact that we are pushing mutliple obj seems to kill the process
    //    if (!keyPoints.empty()) {
    //      for (size_t i = 0; i < keyPoints.size(); ++i) {
    //        sonia_msgs::MapObject obj;
    //        obj.name = "Buoy [" + std::to_string(i) + "]";
    //        obj.pose.x = keyPoints[i].pt.x;
    //        obj.pose.y = keyPoints[i].pt.y;
    //
    //        ObjectRegistery::GetInstance().AddObject(obj);
    //      }
    //    }

    //    // Simple for loop to print keypoint descriptor
    //    if (!keyPoints.empty()) {
    //      for(int i = 0; i < keyPoints.size(); ++i) {
    //        std::cout << "Keypoint[" << i << "]: " << std::endl <<
    //            "x= " << keyPoints[i].pt.x << " y= " << keyPoints[i].pt.y
    //            << std::endl <<
    //            "size= " << keyPoints[i].size << std::endl <<
    //            "angle= " << keyPoints[i].angle << std::endl <<
    //            "class_id= " << keyPoints[i].class_id << std::endl <<
    //            "response= " << keyPoints[i].response << std::endl <<
    //            "octave= " << keyPoints[i].octave << std::endl;
    //      }
    //    }

    if (debug_) {
      cv::Mat output;
      cv::drawKeypoints(input, keyPoints, output, cv::Scalar(0, 0, 255),
                        cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

      //      cv::imshow("Blob Detector", output);
      //      cv::waitKey(1);
    }
  }

 private:
  //==========================================================================
  // P R I V A T E   M E M B E R S

  ros::NodeHandlePtr nh_;
  ros::ServiceServer obstacle_server_;

  std::string obstacle_;
  cv::SimpleBlobDetector::Params params_;
  bool debug_ = false;
};

}  // namespace proc_mapping

#endif  // PROC_MAPPING_PROC_UNIT_BLOB_DETECTOR_H_
