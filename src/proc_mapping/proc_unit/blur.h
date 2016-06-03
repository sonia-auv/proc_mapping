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

#ifndef PROC_MAPPING_PROC_UNIT_BLUR_H_
#define PROC_MAPPING_PROC_UNIT_BLUR_H_

#include <opencv/cv.h>
#include "proc_mapping/proc_unit/proc_unit.h"

namespace proc_mapping {

class Blur : public ProcUnit<cv::Mat> {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<Blur>;
  using ConstPtr = std::shared_ptr<const Blur>;
  using PtrList = std::vector<Blur::Ptr>;
  using ConstPtrList = std::vector<Blur::ConstPtr>;

  struct Parameters {
    static const int kernel_size_max;
    static int kernel_size;
  };

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit Blur(int blur_type = 1, bool debug = false)
      : blur_type(blur_type), debug(debug){};

  virtual ~Blur() = default;

  //==========================================================================
  // P U B L I C   M E T H O D S

  virtual void ProcessData(cv::Mat &input) override {
    // To keep the kernel size odd, multiply by 2 and add 1
    cv::Size2i kernel(Parameters::kernel_size * 2 + 1,
                      Parameters::kernel_size * 2 + 1);
    // Bilateral Filter need another Mat to do algorithm
    cv::Mat dst = input.clone();
    if (blur_type == 0) {
      cv::blur(input, input, kernel);
    } else if (blur_type == 1) {
      cv::GaussianBlur(input, input, kernel, 0, 0);
    } else if (blur_type == 2) {
      cv::medianBlur(input, input, Parameters::kernel_size * 2 + 1);
    } else if (blur_type == 3) {
      cv::bilateralFilter(input, dst, Parameters::kernel_size * 2 + 1,
                          (Parameters::kernel_size * 2 + 1) * 2,
                          (Parameters::kernel_size * 2 + 1) / 2);
      input = dst;
    } else {
      ROS_ERROR("Blur Type is undefined");
    }
    if (debug) {
      cv::createTrackbar("Kernel Size", "Blur", &Parameters::kernel_size,
                         Parameters::kernel_size_max);
      if (blur_type == 3) {
        cv::imshow("Blur", dst);
        cv::waitKey(1);
      } else {
        cv::imshow("Blur", input);
        cv::waitKey(1);
      }
    }
  }

 private:
  /*
 * 0: Homogeneous Blur
 * 1: Gaussian Blur
 * 2: Median Blur
 * 3: Bilateral Blur
 */
  int blur_type;
  bool debug;
};

}  // namespace proc_mapping

#endif  // PROC_MAPPING_PROC_UNIT_BLUR_H_
