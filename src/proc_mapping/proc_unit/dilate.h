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

#ifndef PROC_MAPPING_DILATE_H
#define PROC_MAPPING_DILATE_H

namespace proc_mapping {

using namespace std;
using namespace cv;

class Dilate : public ProcUnit<cv::Mat> {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<Dilate>;
  using ConstPtr = std::shared_ptr<const Dilate>;
  using PtrList = std::vector<Dilate::Ptr>;
  using ConstPtrList = std::vector<Dilate::ConstPtr>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  Dilate() { };
  Dilate(int kernelSize_x, int kernerSize_y):kernelSize_x(kernelSize_x), kernerSize_y(kernerSize_y){};

  virtual ~Dilate() = default;

  //==========================================================================
  // P U B L I C   M E T H O D S

  virtual void ProcessData(cv::Mat &input) override {
    cv::Size size = cv::Size(kernelSize_x, kernerSize_y);
    cv::Mat kernel_ = cv::getStructuringElement(kernelType, size, anchor_);
    cv::dilate(input, input, kernel_, anchor_, iteration);
  }
 private:
  const cv::Point anchor_= cv::Point(-1, -1);
  int iteration = 1;
  int kernelSize_x = 0;
  int kernerSize_y = 0;
  int kernelType = cv::MORPH_RECT;

};

}  // namespace proc_mapping

#endif //PROC_MAPPING_DILATE_H
