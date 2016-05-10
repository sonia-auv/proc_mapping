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

#ifndef PROC_MAPPING_HARRIS_CORNER_H
#define PROC_MAPPING_HARRIS_CORNER_H

#include <opencv/cv.h>
#include "proc_mapping/proc_unit/proc_unit.h"

namespace proc_mapping {

class HarrisCorner : public ProcUnit<cv::Mat> {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<HarrisCorner>;
  using ConstPtr = std::shared_ptr<const HarrisCorner>;
  using PtrList = std::vector<HarrisCorner::Ptr>;
  using ConstPtrList = std::vector<HarrisCorner::ConstPtr>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  HarrisCorner(){};

  virtual ~HarrisCorner() = default;

  //==========================================================================
  // P U B L I C   M E T H O D S

  virtual void ProcessData(cv::Mat &input) override {
    cv::Mat dst, dst_norm, dst_norm_scaled;
    dst = cv::Mat::zeros(displayMap.size(), CV_32FC1);

    /// Detector parameters
    int blockSize = 2;
    int apertureSize = 3;
    double k = 0.04;

    /// Detecting corners
    cornerHarris(displayMap, dst, blockSize, apertureSize, k,
                 cv::BORDER_DEFAULT);

    /// Normalizing
    normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
    convertScaleAbs(dst_norm, dst_norm_scaled);

    /// Drawing a circle around corners
    for (int j = 0; j < dst_norm.rows; j++) {
      for (int i = 0; i < dst_norm.cols; i++) {
        pixel_.number_of_hits_POI_.at(j + i * pixel_.width)++;
        if ((int)dst_norm.at<float>(j, i) > 240) {
          circle(displayMap, cv::Point(i, j), 5, cv::Scalar(255), 2, 8, 0);
          int n = pixel_.number_of_hits_POI_.at(j + i * pixel_.width) + 1;
          pixel_.map_poi.at<uint8_t>(j, i) = static_cast<uint8_t>(
              255 / n + pixel_.map_poi.at<uint8_t>(j, i) * (n - 1) / n);
        }
      }
    }
  }
};

}  // namespace proc_mapping

#endif  // PROC_MAPPING_HARRIS_CORNER_H
