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

#ifndef PROC_MAPPING_WALL_REMOVER_H_
#define PROC_MAPPING_WALL_REMOVER_H_

#include <opencv/cv.h>
#include "proc_mapping/proc_unit/proc_unit.h"

namespace proc_mapping {

class WallRemover : public ProcUnit {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<WallRemover>;
  using ConstPtr = std::shared_ptr<const WallRemover>;
  using PtrList = std::vector<WallRemover::Ptr>;
  using ConstPtrList = std::vector<WallRemover::ConstPtr>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit WallRemover(std::string proc_tree_name = "", bool debug = false) :
      debug_(debug),
      image_publisher_(kRosNodeName + "_threshold_" + proc_tree_name) {
    image_publisher_.Start();
  }

  virtual ~WallRemover() = default;

  //==========================================================================
  // P U B L I C   M E T H O D S

  virtual boost::any ProcessData(boost::any input) override {
    cv::Mat map = boost::any_cast<cv::Mat>(input);

    std::vector<std::vector<cv::Point>> contour_list, final_contour_list;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(map.clone(), contour_list, hierarchy, CV_RETR_EXTERNAL,
                     CV_CHAIN_APPROX_SIMPLE);

    for(size_t i = 0; i < contour_list.size(); i++) {
      double area = cv::contourArea(contour_list[i]);
      // Is enough big
      if(area < 30) {
        continue;
      }

      cv::RotatedRect rotatedRect = cv::minAreaRect(contour_list[i]);
      // RotatedRect does not guarantee that the height is longer than
      // the width, so we decide that height is for the longer side.
      if( rotatedRect.size.width > rotatedRect.size.height) {
        std::swap(rotatedRect.size.width, rotatedRect.size.height);
      }

      // Is thin enough
      if(rotatedRect.size.width > 50 /*&& rotatedRect.size.width < 100*/) {
        continue;
      }
      // Keep it if it matches
      final_contour_list.push_back(contour_list[i]);
    }

    map.setTo(cv::Scalar(0));
    cv::drawContours(map, final_contour_list, -1, CV_RGB(255,255,255), -1);

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

 private:
  bool debug_;

  atlas::ImagePublisher image_publisher_;
};

}  // namespace proc_mapping

#endif //PROC_MAPPING_WALL_REMOVER_H_
