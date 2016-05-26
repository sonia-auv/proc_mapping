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

#ifndef PROC_MAPPING_BLOB_DETECTOR_H
#define PROC_MAPPING_BLOB_DETECTOR_H

namespace proc_mapping {

int minArea = 30;
const int maxArea = 300;
bool filterByConvexity = true;
int minConvexity = 1;
const int maxConvexity = 8;
bool filterByInertia = true;
int minInertiaRatio = 0;
const int maxInertiaRatio = 5;

class BlobDetector : public ProcUnit<cv::Mat> {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<BlobDetector>;
  using ConstPtr = std::shared_ptr<const BlobDetector>;
  using PtrList = std::vector<BlobDetector::Ptr>;
  using ConstPtrList = std::vector<BlobDetector::ConstPtr>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  BlobDetector() {
    params.minThreshold = 0;
    params.maxThreshold = 255;
    // Filter by Area.
    params.filterByArea = true;
    params.minArea = 30;
    params.maxArea = 300;
//// Filter by Circularity
//  params.filterByCircularity = true;
//  params.minCircularity = 0.00001;
//  params.maxCircularity = 0.5;
    params.filterByColor = false;
// Filter by Convexity
    params.filterByConvexity = false;
    params.minConvexity = 0.1;
    params.maxConvexity = 0.8;
// Filter by Inertia
    params.filterByInertia = false;
    params.minInertiaRatio = 0;
    params.maxInertiaRatio = 0.5;
  };

  BlobDetector(bool debug) : debug(debug) {  }
  virtual ~BlobDetector() = default;

  //==========================================================================
  // P U B L I C   M E T H O D S

  virtual void ProcessData(cv::Mat &input) override {
    cv::createTrackbar("Area", "Blob Detector", &minArea, maxArea);
    cv::createTrackbar("Convexity", "Blob Detector",
                       &minConvexity, maxConvexity);
    cv::createTrackbar("Inertia Ratio", "Blob Detector",
                       &minInertiaRatio, maxInertiaRatio);

    params.minThreshold = 0;
    params.maxThreshold = 255;
    // Filter by Area.
    params.filterByArea = true;
    params.minArea = minArea;
    params.maxArea = maxArea;
//// Filter by Circularity
//  params.filterByCircularity = true;
//  params.minCircularity = 0.00001;
//  params.maxCircularity = 0.5;
    params.filterByColor = false;
// Filter by Convexity
    params.filterByConvexity = filterByConvexity;
    params.minConvexity = minConvexity / 10;
    params.maxConvexity = maxConvexity / 10;
// Filter by Inertia
    params.filterByInertia = filterByInertia;
    params.minInertiaRatio = minInertiaRatio / 10;
    params.maxInertiaRatio = maxInertiaRatio / 10;

    cv::SimpleBlobDetector detector(params);
    std::vector<cv::KeyPoint> keyPoints;
    detector.detect(input, keyPoints);

    cv::Mat output;
    cv::drawKeypoints(input, keyPoints, output, cv::Scalar(0,0,255),
                      cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    if (debug) {
      cv::imshow("Blob Detector", output);
      cv::waitKey(1);
    }
  }
 private:
  cv::SimpleBlobDetector::Params params;
  bool debug = false;
};
}  // namespace proc_mapping

#endif //PROC_MAPPING_BLOB_DETECTOR_H
