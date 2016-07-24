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
#include <lib_atlas/ros/image_publisher.h>
#include "proc_mapping/config.h"

namespace proc_mapping {

class BlobDetector : public ProcUnit {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<BlobDetector>;
  using ConstPtr = std::shared_ptr<const BlobDetector>;
  using PtrList = std::vector<BlobDetector::Ptr>;
  using ConstPtrList = std::vector<BlobDetector::ConstPtr>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  BlobDetector(std::string proc_tree_name = "", uint8_t target = 0, bool debug = false)
      : target_(target),
        debug_(debug),
        image_publisher_(kRosNodeName + "_blob_detector_" + proc_tree_name) {
    image_publisher_.Start();
  }

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
      params_.maxArea = 500;
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
      ROS_INFO("Wrong target.");
    }
    cv::SimpleBlobDetector detector(params_);
    std::vector<cv::KeyPoint> keyPoints;
    detector.detect(map, keyPoints);

    if (debug_) {
      cv::Mat output;
      cv::drawKeypoints(map, keyPoints, output, cv::Scalar(0, 0, 255),
                        cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

      // To fit in OpenCv coordinate system, we have to made a rotation of
      // 90 degrees on the display map
      cv::Point2f src_center(output.cols/2.0f, output.rows/2.0f);
      cv::Mat rot_mat = getRotationMatrix2D(src_center, 90, 1.0);
      cv::Mat dst;
      cv::warpAffine(output, dst, rot_mat, output.size());

      image_publisher_.Write(dst);
    }

    return boost::any(keyPoints);
  }

 private:
  //==========================================================================
  // P R I V A T E   M E M B E R S

  uint8_t target_;
  bool debug_;
  cv::SimpleBlobDetector::Params params_;
  atlas::ImagePublisher image_publisher_;
};

}  // namespace proc_mapping

#endif  // PROC_MAPPING_PROC_UNIT_BLOB_DETECTOR_H_
