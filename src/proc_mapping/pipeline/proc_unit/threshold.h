/**
 * \file	threshold.h
 * \author	Francis Masse <francis.masse05@gmail.com>
 * \date	18/05/2016
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

#ifndef PROC_MAPPING_PIPELINE_PROC_UNIT_THRESHOLD_H_
#define PROC_MAPPING_PIPELINE_PROC_UNIT_THRESHOLD_H_

#include <opencv/cv.h>
#include "proc_mapping/pipeline/proc_unit.h"

namespace proc_mapping {

class Threshold : public ProcUnit {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<Threshold>;
  using ConstPtr = std::shared_ptr<const Threshold>;
  using PtrList = std::vector<Threshold::Ptr>;
  using ConstPtrList = std::vector<Threshold::ConstPtr>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit Threshold(std::string proc_tree_name = "", int threshold_type = 0,
                     int thresh_value = 0, bool debug = false)
      : threshold_type(threshold_type),
        thresh_value_(thresh_value),
        debug(debug),
        image_publisher_(kRosNodeName + "_threshold_" + proc_tree_name) {
    image_publisher_.Start();
  }

  virtual ~Threshold() = default;

  //==========================================================================
  // P U B L I C   M E T H O D S

  virtual boost::any ProcessData(boost::any input) override {
    cv::Mat map = boost::any_cast<cv::Mat>(input);
    if ((threshold_type == 0) | (threshold_type == 1) | (threshold_type == 2) |
        (threshold_type == 3) | (threshold_type == 4) | (threshold_type == 7) |
        (threshold_type == 8)) {
      cv::threshold(map, map, thresh_value_, 255, threshold_type);
    } else {
      ROS_ERROR("Threshold type is undefined");
    }
    if (debug) {
      // To fit in OpenCv coordinate system, we have to made a rotation of
      // 90 degrees on the display map
      cv::Point2f src_center(map.cols / 2.0f, map.rows / 2.0f);
      cv::Mat rot_mat = getRotationMatrix2D(src_center, 90, 1.0);
      cv::Mat dst;
      cv::warpAffine(map, dst, rot_mat, map.size());

      cvtColor(dst, dst, CV_GRAY2RGB);
      image_publisher_.Write(dst);
    }
    return boost::any(map);
  }

  std::string GetName() const override { return "threshold"; }

 private:
  /*
 * 0: Binary
 * 1: Binary Inverted
 * 2: Threshold Truncated
 * 3: Threshold to Zero
 * 4: Threshold to Zero Inverted
 * 7: Threshold Mask
 * 8: Threshold OTSU
 */
  int threshold_type;
  int thresh_value_;
  bool debug;

  atlas::ImagePublisher image_publisher_;
};

}  // namespace proc_mapping

#endif  // PROC_MAPPING_PIPELINE_PROC_UNIT_THRESHOLD_H_
