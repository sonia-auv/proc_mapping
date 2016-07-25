/**
 * \file	dilate.h
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

#ifndef PROC_MAPPING_PROC_UNIT_DILATE_H_
#define PROC_MAPPING_PROC_UNIT_DILATE_H_

#include <opencv/cv.h>
#include "proc_mapping/proc_unit/proc_unit.h"

namespace proc_mapping {

class Dilate : public ProcUnit {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<Dilate>;
  using ConstPtr = std::shared_ptr<const Dilate>;
  using PtrList = std::vector<Dilate::Ptr>;
  using ConstPtrList = std::vector<Dilate::ConstPtr>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  Dilate(std::string proc_tree_name = "", int kernel_size_x = 5,
         int kernel_size_y = 5, bool debug = false) :
      debug_(debug),
      kernel_size_x_(kernel_size_x),
      kernel_size_y_(kernel_size_y),
      image_publisher_(kRosNodeName + "_dilate_" + proc_tree_name) {
    image_publisher_.Start();
  };

  virtual ~Dilate() = default;

  //==========================================================================
  // P U B L I C   M E T H O D S

  virtual boost::any ProcessData(boost::any input) override {
    cv::Mat map = boost::any_cast<cv::Mat>(input);
    cv::Size size =
        cv::Size(kernel_size_x_, kernel_size_y_);
    cv::Mat kernel_ = cv::getStructuringElement(kernel_type_, size, anchor_);
    cv::dilate(map, map, kernel_, anchor_, iteration_);
    if (debug_) {
      // To fit in OpenCv coordinate system, we have to made a rotation of
      // 90 degrees on the display map
      cv::Point2f src_center(map.cols/2.0f, map.rows/2.0f);
      cv::Mat rot_mat = getRotationMatrix2D(src_center, 90, 1.0);
      cv::Mat dst;
      cv::warpAffine(map, dst, rot_mat, map.size());

      cvtColor(dst, dst, CV_GRAY2RGB);
      image_publisher_.Write(dst);
    }
    return boost::any(map);
  }

  const std::string GetName() const override { return "dilate"; }

 private:
  bool debug_;
  const cv::Point anchor_ = cv::Point(-1, -1);
  int iteration_ = 1;
  int kernel_type_ = cv::MORPH_RECT;

  int kernel_size_x_;
  int kernel_size_y_;

  atlas::ImagePublisher image_publisher_;
};

}  // namespace proc_mapping

#endif  // PROC_MAPPING_PROC_UNIT_DILATE_H_
