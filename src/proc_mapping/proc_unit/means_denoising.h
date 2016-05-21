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

#ifndef PROC_MAPPING_MEANS_DENOISING_H
#define PROC_MAPPING_MEANS_DENOISING_H

#include <opencv/cv.h>
#include "proc_mapping/proc_unit/proc_unit.h"

namespace proc_mapping {

using namespace std;
using namespace cv;

class MeansDenoising : public ProcUnit<cv::Mat> {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<MeansDenoising>;
  using ConstPtr = std::shared_ptr<const MeansDenoising>;
  using PtrList = std::vector<MeansDenoising::Ptr>;
  using ConstPtrList = std::vector<MeansDenoising::ConstPtr>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  MeansDenoising(){};

  virtual ~MeansDenoising() = default;

  //==========================================================================
  // P U B L I C   M E T H O D S

  /// Global Variables
  int DELAY_CAPTION = 1500;
  int DELAY_BLUR = 100;
  int MAX_KERNEL_LENGTH = 10;
  int edgeThresh = 1;
  int lowThreshold = 40;
  int const max_lowThreshold = 100;
  int ratio = 2;
  int const max_ratio = 10;
  int kernel_size = 5;
  int const max_kernel_size = 10;

  Mat src;
  Mat dst;
  Mat detected_edges;

  virtual void ProcessData(cv::Mat &input) override {
    //    /// Load the source image
    //    src = input;
    //
    //    dst = src.clone();
    //    dst = Mat::zeros( src.size(), src.type() );
    //
    //    for ( int i = 1; i < MAX_KERNEL_LENGTH; i = i + 2 )
    //    { GaussianBlur( input, input, Size( i, i ), 10, 0 ); }
    //    createTrackbar( "Min Threshold:", "Edge Map", &lowThreshold,
    //    max_lowThreshold);
    //    createTrackbar( "Kernel size", "Edge Map", &kernel_size,
    //    max_kernel_size);
    //    createTrackbar( "ration", "Edge Map", &ratio, max_ratio);
    //
    //    Canny( input, input, lowThreshold, lowThreshold*ratio, kernel_size );

    //    imshow("Edge Map", input);

    //    imshow("TAMERE", dst);
  }
};

}  // namespace proc_mapping

#endif  // PROC_MAPPING_MEANS_DENOISING_H
