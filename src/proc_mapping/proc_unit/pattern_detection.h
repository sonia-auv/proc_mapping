/**
 * \file	pattern_detection.h
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

#include <opencv/cv.h>
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

  PatternDetection(){};

  virtual ~PatternDetection() = default;

  //==========================================================================
  // P U B L I C   M E T H O D S

  virtual void ProcessData(cv::Mat &input) override {
    cv::Mat result;

    cv::Mat buoy_template;
    buoy_template = cv::Mat(40, 40, CV_8UC1);
    buoy_template.setTo(cv::Scalar(0));
    cv::rectangle(buoy_template, cv::Point2i(19, 0), cv::Point2i(21, 40),
                  cv::Scalar(255), CV_FILLED);

    /// Create the result matrix
    int result_cols = input.cols;
    int result_rows = input.rows;

    result.create(result_rows, result_cols, CV_32FC1);

    /// Do the Matching and Normalize
    matchTemplate(input, buoy_template, result, CV_TM_SQDIFF);
    normalize(result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

    /// Localizing the best match with minMaxLoc
    double minVal;
    double maxVal;
    cv::Point minLoc;
    cv::Point maxLoc;
    cv::Point matchLoc;

    minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());

    matchLoc = minLoc;

    /// Show me what you got
    rectangle(input, matchLoc, cv::Point(matchLoc.x + buoy_template.cols,
                                         matchLoc.y + buoy_template.rows),
              cv::Scalar::all(255), 2, 8, 0);

    cv::imshow("display Map", input);
    cv::imshow("buoy", buoy_template);
    cv::waitKey(1);
  }
};

}  // namespace proc_mapping

#endif  // PROC_MAPPING_PATTERN_DETECTION_H
