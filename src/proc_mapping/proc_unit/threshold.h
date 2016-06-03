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

#ifndef PROC_MAPPING_PROC_UNIT_THRESHOLD_H_
#define PROC_MAPPING_PROC_UNIT_THRESHOLD_H_

#include <opencv/cv.h>
#include "proc_mapping/proc_unit/proc_unit.h"

namespace proc_mapping {

const int thresh_value_max = 255;
int thresh_value = 0;

class Threshold : public ProcUnit<cv::Mat> {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<Threshold>;
  using ConstPtr = std::shared_ptr<const Threshold>;
  using PtrList = std::vector<Threshold::Ptr>;
  using ConstPtrList = std::vector<Threshold::ConstPtr>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  Threshold(int threshold_type = 8, bool debug = false)
      : threshold_type(threshold_type), debug(debug){};

  virtual ~Threshold() = default;

  //==========================================================================
  // P U B L I C   M E T H O D S

  virtual void ProcessData(cv::Mat &input) override {
    cv::createTrackbar("Thresh Value", "Threshold", &thresh_value,
                       thresh_value_max);
    if ((threshold_type == 0) | (threshold_type == 1) | (threshold_type == 2) |
        (threshold_type == 3) | (threshold_type == 4) | (threshold_type == 7) |
        (threshold_type == 8)) {
      cv::threshold(input, input, thresh_value, 255, threshold_type);
    } else {
      ROS_ERROR("Threshold type is undefined");
    }
    if (debug) {
      cv::imshow("Threshold", input);
      cv::waitKey(1);
    }
  }

 private:
  int threshold_type;
  bool debug;
  /*
   * 0: Binary
   * 1: Binary Inverted
   * 2: Threshold Truncated
   * 3: Threshold to Zero
   * 4: Threshold to Zero Inverted
   * 7: Threshold Mask
   * 8: Threshold OTSU
   */
};

}  // namespace proc_mapping

#endif  // PROC_MAPPING_PROC_UNIT_THRESHOLD_H_
