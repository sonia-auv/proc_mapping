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

#ifndef PROC_MAPPING_GAUSSIAN_BLUR_H
#define PROC_MAPPING_GAUSSIAN_BLUR_H

#include <opencv/cv.h>
#include "proc_mapping/proc_unit/proc_unit.h"

namespace proc_mapping {

using namespace std;
using namespace cv;

class GaussianBlur : public ProcUnit<cv::Mat> {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<GaussianBlur>;
  using ConstPtr = std::shared_ptr<const GaussianBlur>;
  using PtrList = std::vector<GaussianBlur::Ptr>;
  using ConstPtrList = std::vector<GaussianBlur::ConstPtr>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  GaussianBlur() { };

  virtual ~GaussianBlur() = default;

  //==========================================================================
  // P U B L I C   M E T H O D S

  virtual void ProcessData(cv::Mat &input) override {
    cv::Size2i kernel_(kernelSize*2+1,kernelSize*2+1) ;
    cv::GaussianBlur(input, input, kernel_, 0, 0);
  }
 private:
  int kernelSize = 3;
};

}  // namespace proc_mapping

#endif //PROC_MAPPING_GAUSSIAN_BLUR_H

