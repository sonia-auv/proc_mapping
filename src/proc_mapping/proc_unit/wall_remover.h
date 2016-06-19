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

  explicit WallRemover(bool debug = false) : debug(debug) {}

  virtual ~WallRemover() = default;

  //==========================================================================
  // P U B L I C   M E T H O D S

  virtual boost::any ProcessData(boost::any input) override {
    cv::Mat map = boost::any_cast<cv::Mat>(input);
    cv::Mat wall_map;
    map.copyTo(wall_map);
    cv::Size2i kernel(2 * 2 + 1, 2 * 2 + 1);
    cv::GaussianBlur(wall_map, wall_map, kernel, 0, 0);
    cv::Mat element = cv::getStructuringElement(
        0, cv::Size(2 * 2 + 1, 2 * 2 + 1), cv::Point(2, 2));
    cv::morphologyEx(wall_map, wall_map, cv::MORPH_CLOSE, element);
    cv::threshold(wall_map, wall_map, 40, 255, 0);

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    findContours(wall_map, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    Mat drawing = Mat::zeros(wall_map.size(), CV_8UC1);

    for (int i = 0; i < contours.size(); i++) {
      double arc_length = 10;//0.1 * cv::arcLength(contours[i], true);
      std::vector<cv::Point> output;
      cv::approxPolyDP(contours[i], output, arc_length, false);
      std::swap(contours[i], output);
      cv::Scalar area = contourArea(contours[i]);
      if (area[0] > 200) {
        cv::drawContours(drawing, contours, i, 255);
        cv::fillPoly(drawing, contours, 255);
      }
    }

    cv::subtract(map, drawing, map);

    if (debug) {
      cv::imshow("Wall Remover", drawing);
      cv::waitKey(1);
    }
    return boost::any(map);
  }

 private:
  bool debug;
};

}  // namespace proc_mapping

#endif //PROC_MAPPING_WALL_REMOVER_H_

