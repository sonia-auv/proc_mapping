/**
 * \file	far_buoys_detector.h
 * \author	Francis Masse <francis.masse05@gmail.com>
 * \date	05/26/2016
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
#ifndef PROC_MAPPING_PROC_UNIT_HISTOGRAM_H_
#define PROC_MAPPING_PROC_UNIT_HISTOGRAM_H_

#include <opencv/cv.h>
#include "proc_mapping/config.h"
#include "proc_mapping/proc_unit/proc_unit.h"

namespace proc_mapping {

class Histogram : public ProcUnit<cv::Mat> {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<Histogram>;
  using ConstPtr = std::shared_ptr<const Histogram>;
  using PtrList = std::vector<Histogram::Ptr>;
  using ConstPtrList = std::vector<Histogram::ConstPtr>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  Histogram(){};

  virtual ~Histogram() = default;

  //==========================================================================
  // P U B L I C   M E T H O D S

  virtual boost::any ProcessData(boost::any input) override {
    cv::Mat map = boost::any_cast<cv::Mat>(input);
    int hist_size = 256;
    float range[] = {0, 255};
    const float *hist_range = {range};

    cv::Mat hist;

    cv::calcHist(&map, 1, 0, cv::Mat(), hist, 1, &hist_size, &hist_range, true,
                 false);

    int hist_w = 1024;
    int hist_h = 400;
    int bin_w = cvRound(static_cast<double>(hist_w / hist_size));

    cv::Mat hist_image(hist_h, hist_w, CV_8UC1, cv::Scalar(0));

    for (int i = 1; i < hist_size; i++) {
      cv::line(hist_image, cv::Point(bin_w * (i - 1),
                                     hist_h - cvRound(hist.at<float>(i - 1))),
               cv::Point(bin_w * (i), hist_h - cvRound(hist.at<float>(i))),
               cv::Scalar(255), 2, 8, 0);
    }

    //    float thresh = (static_cast<float>(thresh_value) /
    //                    static_cast<float>(thresh_value_max));
    //
    //    cv::line(hist_image, cv::Point(hist_w * thresh, 0),
    //             cv::Point(hist_w * thresh, hist_h), cv::Scalar(255));

    cv::imshow("Histogram", hist_image);
    cv::waitKey(1);
  }
};

}  // namespace proc_mapping

#endif  // PROC_MAPPING_PROC_UNIT_HISTOGRAM_H_
