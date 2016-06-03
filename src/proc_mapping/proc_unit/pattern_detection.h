/**
 * \file	pattern_detection.h
 * \author	Francis Masse <francis.masse05@gmail.com>
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	09/05/2016
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

#ifndef PROC_MAPPING_PATTERN_DETECTION_H
#define PROC_MAPPING_PATTERN_DETECTION_H

#include <lib_atlas/config.h>
#include <lib_atlas/macros.h>
#include <opencv/cv.h>
#include <ros/ros.h>
#include <sonia_msgs/MapObject.h>
#include <memory>
#include "proc_mapping/interpreter/object_registery.h"
#include "proc_mapping/proc_unit/proc_unit.h"

namespace proc_mapping {

class PatternDetection : public ProcUnit<cv::Mat> {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<PatternDetection>;
  using ConstPtr = std::shared_ptr<const PatternDetection>;
  using PtrList = std::vector<PatternDetection::Ptr>;
  using ConstPtrList = std::vector<PatternDetection::ConstPtr>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  PatternDetection(const ros::NodeHandlePtr &nh)
      : ProcUnit(), nh_(nh), obstacle_template_path_() {
    obstacle_template_path_ =
        atlas::kWorkspaceRoot + "/src/proc_mapping/template/buoy_template2.png";
  }

  virtual ~PatternDetection() = default;

  //==========================================================================
  // P U B L I C   M E T H O D S

  virtual void ProcessData(cv::Mat &input) override {
    cv::Mat result;
    cv::Mat buoy_template;

    // Load obtacle template
    buoy_template =
        cv::imread(obstacle_template_path_, CV_LOAD_IMAGE_GRAYSCALE);

    // Create the result matrix
    int result_cols = input.cols;
    int result_rows = input.rows;

    result.create(result_rows, result_cols, CV_32FC1);

    // Do the Matching
    matchTemplate(input, buoy_template, result, CV_TM_CCORR_NORMED);

    /// Localizing the best match with minMaxLoc
    double minVal;
    double maxVal;
    cv::Point minLoc;
    cv::Point maxLoc;
    cv::Point matchLoc;

    minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());

#ifndef OS_DARWIN
    cv::imshow("Pattern Detection Map", input);
    //      cv::imshow("buoy", buoy_template);
    cv::waitKey(1);
#endif
  }

 private:
  //==========================================================================
  // P R I V A T E   M E M B E R S

  ros::NodeHandlePtr nh_;

  std::string obstacle_template_path_;
};

}  // namespace proc_mapping

#endif  // PROC_MAPPING_PATTERN_DETECTION_H
