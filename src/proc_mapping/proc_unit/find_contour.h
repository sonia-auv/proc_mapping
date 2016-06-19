/**
 * \file	find_contour.h
 * \author	Francis Masse <francis.masse05@gmail.com>
 * \date	19/06/2016
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

#ifndef PROC_MAPPING_FIND_CONTOUR_H_
#define PROC_MAPPING_FIND_CONTOUR_H_

using namespace cv;
using namespace std;

namespace proc_mapping {

class FindContour : public ProcUnit {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<FindContour>;
  using ConstPtr = std::shared_ptr<const FindContour>;
  using PtrList = std::vector<FindContour::Ptr>;
  using ConstPtrList = std::vector<FindContour::ConstPtr>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  FindContour(bool debug = false) : debug_(debug){};

  virtual ~FindContour() = default;

  //==========================================================================
  // P U B L I C   M E T H O D S

  virtual boost::any ProcessData(boost::any input) override {
    cv::Mat map = boost::any_cast<cv::Mat>(input);

    Mat canny_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    int thresh = 150;
    int max_thresh = 255;
    RNG rng(12345);

    /// Detect edges using canny
//    Canny( map, canny_output, thresh, thresh*2, 3 );
    /// Find contours
    findContours(map, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

//    static vector<Point> wall_contour;
//    /// Draw contours
    Mat drawing = Mat::zeros( map.size(), CV_8UC3 );
//    for( int i = 0; i < contours.size(); i++ ) {
//      if (contours[i].size() > 100) {
//        for (int j = 0; j < contours[i].size(); j++) {
//          wall_contour.push_back(contours[i][j]);
//        }
//      }
//    }
//
    for (int i = 0; i < contours.size(); i++) {
      double arc_length = 10;//0.1 * cv::arcLength(contours[i], true);
      std::vector<cv::Point> output;
      cv::approxPolyDP(contours[i], output, arc_length, false);
      std::swap(contours[i], output);
//      convexHull(contours[i], contours[i]);
      cv::Scalar area = contourArea(contours[i]);
      if (area[0] > 50) {
        cv::drawContours(drawing, contours, i, CV_RGB(255,255,255));
      }
    }
//    if (wall_contour.size() > 200) {
//      for (int i = 0; i < wall_contour.size(); i += 2) {
//        line(drawing, wall_contour[i], wall_contour[i+1], 255);
//      }
//    }

    if (debug_) {
      cv::imshow("Find Contour", drawing);
      cv::waitKey(1);
    }
    return boost::any(map);
  }

 private:
  bool debug_;
};

}  // namespace proc_mapping

#endif //PROC_MAPPING_FIND_CONTOUR_H_


