/**
 * \file	pattern_detection.h
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

#ifndef PROC_MAPPING_PROC_UNIT_BLOB_DETECTOR_H_
#define PROC_MAPPING_PROC_UNIT_BLOB_DETECTOR_H_

#include <highgui.h>
#include <opencv/cv.h>
#include <ros/ros.h>
#include <memory>
#include "proc_mapping/map/object_registery.h"
#include "proc_mapping/map_objects/buoy.h"
#include "proc_mapping/proc_unit/proc_unit.h"

namespace proc_mapping {

class BlobDetector : public ProcUnit {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<BlobDetector>;
  using ConstPtr = std::shared_ptr<const BlobDetector>;
  using PtrList = std::vector<BlobDetector::Ptr>;
  using ConstPtrList = std::vector<BlobDetector::ConstPtr>;

  struct Parameters {
    static int filter_area_off;
    static const int filter_area_on;
    static int min_area;
    static const int min_area_max;
    static int max_area;
    static const int max_area_max;
    static int filter_circularity_off;
    static const int filter_circularity_on;
    static float min_circularity;
    static const float min_circularity_max;
    static float max_circularity;
    static const float max_circularity_max;
    static int filter_convexity_off;
    static const int filter_convexity_on;
    static float min_convexity;
    static const float min_convexity_max;
    static float max_convexity;
    static const float max_convexity_max;
    static int filter_inertial_off;
    static const int filter_inertial_on;
    static int min_inertia_ratio;
    static const float min_inertia_ratio_max;
    static int max_inertia_ratio;
    static const float max_inertia_ratio_max;
  };

  //==========================================================================
  // P U B L I C   C / D T O R S

  BlobDetector(uint8_t target = 0, bool debug = false)
      : target_(target), debug_(debug) {}
  virtual ~BlobDetector() = default;

  //==========================================================================
  // P U B L I C   M E T H O D S

  virtual boost::any ProcessData(boost::any input) override {
    cv::Mat map = boost::any_cast<cv::Mat>(input);
    if (target_ == 0) {
      params_.minThreshold = 30;
      params_.maxThreshold = 200;
      params_.filterByArea = true;
      params_.blobColor = 255;
      params_.minArea = 80;
      params_.maxArea = 400;
      params_.filterByCircularity = false;
      params_.filterByColor = true;
      params_.filterByConvexity = false;
      params_.filterByInertia = false;
      params_.minInertiaRatio = 0.1f;
      params_.maxInertiaRatio = 0.3f;
    } else if (target_ == 1) {
      params_.minThreshold = 200;
      params_.maxThreshold = 255;
      params_.filterByArea = true;
      params_.blobColor = 255;
      params_.minArea = 2000;
      params_.maxArea = 4000;
      params_.filterByCircularity = false;
      params_.filterByColor = false;
      params_.filterByConvexity = false;
      params_.filterByInertia = false;
      params_.minInertiaRatio = 0.1f;
      params_.maxInertiaRatio = 0.3f;
    } else {
      params_.minThreshold = 0;
      params_.maxThreshold = 255;
      // Filter by Area.
      params_.filterByArea = Parameters::filter_area_off;
      params_.minArea = Parameters::min_area;
      params_.maxArea = Parameters::max_area;
      //// Filter by Circularity
      params_.filterByCircularity = Parameters::filter_circularity_off;
      params_.minCircularity = Parameters::min_circularity / 100;
      params_.maxCircularity = Parameters::max_circularity / 100;
      params_.filterByColor = false;
      // Filter by Convexity
      params_.filterByConvexity = Parameters::filter_convexity_off;
      params_.minConvexity = Parameters::min_convexity / 10;
      params_.maxConvexity = Parameters::max_convexity / 10;
      // Filter by Inertia
      params_.filterByInertia = Parameters::filter_inertial_off;
      params_.minInertiaRatio = Parameters::min_inertia_ratio / 10;
      params_.maxInertiaRatio = Parameters::max_inertia_ratio / 10;
    }
    cv::SimpleBlobDetector detector(params_);
    std::vector<cv::KeyPoint> keyPoints;
    detector.detect(map, keyPoints);

    if (debug_) {
//      cv::createTrackbar("area filter", "Blob Detector",
//                         &Parameters::filter_area_off,
//                         Parameters::filter_area_on);
//      cv::createTrackbar("min area", "Blob Detector", &Parameters::min_area,
//                         Parameters::min_area_max);
//      cv::createTrackbar("max area", "Blob Detector", &Parameters::max_area,
//                         Parameters::max_area_max);
      //      cv::createTrackbar("circularity filter", "Blob Detector",
      //                         &Parameters::filter_circularity_off,
      //                         Parameters::filter_circularity_on);
      //      cv::createTrackbar("min circularity", "Blob Detector",
      //                         &Parameters::min_circularity,
      //                         Parameters::min_circularity_max);
      //      cv::createTrackbar("max circularity", "Blob Detector",
      //                         &Parameters::max_circularity,
      //                         Parameters::max_circularity_max);
      //      cv::createTrackbar("convexity filter", "Blob Detector",
      //                         &Parameters::filter_convexity_off,
      //                         Parameters::filter_convexity_on);
      //      cv::createTrackbar("min convexity", "Blob Detector",
      //                         &Parameters::min_convexity,
      //                         Parameters::min_convexity_max);
      //      cv::createTrackbar("max convexity", "Blob Detector",
      //                         &Parameters::max_convexity,
      //                         Parameters::max_convexity_max);
      //      cv::createTrackbar("inertia filter", "Blob Detector",
      //                         &Parameters::filter_inertial_off,
      //                         Parameters::filter_inertial_on);
//            cv::createTrackbar("min inertia", "Blob Detector",
//                               &Parameters::min_inertia_ratio,
//                               static_cast<int>(Parameters::min_inertia_ratio_max));
//            cv::createTrackbar("max inertia", "Blob Detector",
//                               &Parameters::max_inertia_ratio,
//                               static_cast<int>(Parameters::max_inertia_ratio_max));

      cv::Mat output;
      cv::drawKeypoints(map, keyPoints, output, cv::Scalar(0, 0, 255),
                        cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

      // To fit in OpenCv coordinate system, we have to made a rotation of
      // 90 degrees on the display map
      cv::Point2f src_center(output.cols/2.0f, output.rows/2.0f);
      cv::Mat rot_mat = getRotationMatrix2D(src_center, 90, 1.0);
      cv::Mat dst;
      cv::warpAffine(output, dst, rot_mat, output.size());

      cv::imshow("Blob Detector", dst);
      cv::waitKey(1);
    }
    return boost::any(keyPoints);
  }

 private:
  //==========================================================================
  // P R I V A T E   M E M B E R S

  uint8_t target_;
  bool debug_;
  cv::SimpleBlobDetector::Params params_;
};

}  // namespace proc_mapping

#endif  // PROC_MAPPING_PROC_UNIT_BLOB_DETECTOR_H_
