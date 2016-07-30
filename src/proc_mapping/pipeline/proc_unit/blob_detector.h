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

#ifndef PROC_MAPPING_PIPELINE_PROC_UNIT_BLOB_DETECTOR_H_
#define PROC_MAPPING_PIPELINE_PROC_UNIT_BLOB_DETECTOR_H_

#include <highgui.h>
#include <lib_atlas/ros/image_publisher.h>
#include <opencv/cv.h>
#include <ros/ros.h>
#include <memory>
#include "proc_mapping/config.h"
#include "proc_mapping/map/object_registery.h"
#include "proc_mapping/map_objects/buoy.h"
#include "proc_mapping/pipeline/proc_unit.h"

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

  explicit BlobDetector(const std::string &topic_namespace);

  virtual ~BlobDetector() = default;

  //==========================================================================
  // P U B L I C   M E T H O D S

  virtual void ConfigureFromYamlNode(const YAML::Node &node) override;

  virtual boost::any ProcessData(boost::any input) override;

  void GenerateImageToPublish(const cv::Mat &map,
                              const std::vector<cv::KeyPoint> &keyPoints);

  std::string GetName() const override;

 private:
  //==========================================================================
  // P R I V A T E   M E M B E R S

  Parameter<int> min_threshold_;
  Parameter<int> max_threshold_;

  Parameter<bool> filter_by_area_;
  Parameter<int> min_area_;
  Parameter<int> max_area_;

  Parameter<bool> filter_by_color_;
  Parameter<int> blob_color_;

  Parameter<bool> filter_by_circularity_;
  Parameter<int> min_circularity_;
  Parameter<int> max_circularity_;

  Parameter<bool> filter_by_convexity_;
  Parameter<int> min_convexity_;
  Parameter<int> max_convexity_;

  Parameter<bool> filter_by_inertia_;
  Parameter<int> min_inertia_ratio_;
  Parameter<int> max_inertia_ratio_;

  cv::SimpleBlobDetector::Params params_;
};

//==============================================================================
// I N L I N E   M E T H O D S

//------------------------------------------------------------------------------
//
inline BlobDetector::BlobDetector(const std::string &topic_namespace)
    : ProcUnit(topic_namespace),
      min_threshold_("Min Threshold", 200, parameters_),
      max_threshold_("Max Threshold", 255, parameters_),
      filter_by_area_("Filter by area", false, parameters_),
      min_area_("Min area", 100, parameters_),
      max_area_("Max area", 350, parameters_),
      filter_by_color_("Filter by color", false, parameters_),
      blob_color_("Blob color", 255, parameters_),
      filter_by_circularity_("Filter by circularity", false, parameters_),
      min_circularity_("Min circularity", 1, parameters_),
      max_circularity_("Max circularity", 2, parameters_),
      filter_by_convexity_("Filter by convexity", false, parameters_),
      min_convexity_("Min convexity", 1, parameters_),
      max_convexity_("Max convexity", 2, parameters_),
      filter_by_inertia_("Filter by inertia", false, parameters_),
      min_inertia_ratio_("Min inertia", 1, parameters_),
      max_inertia_ratio_("Max inertia", 2, parameters_) {}

//------------------------------------------------------------------------------
//
inline std::string BlobDetector::GetName() const { return "blob_detection"; }

//------------------------------------------------------------------------------
//
inline void BlobDetector::GenerateImageToPublish(
    const cv::Mat &map, const std::vector<cv::KeyPoint> &keyPoints) {
  cv::Mat output;
  drawKeypoints(map, keyPoints, output, cv::Scalar(0, 0, 255),
                cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

  // To fit in OpenCv coordinate system, we have to made a rotation of
  // 90 degrees on the display map
  cv::Point2f src_center(output.cols / 2.0f, output.rows / 2.0f);
  cv::Mat rot_mat = getRotationMatrix2D(src_center, 90, 1.0);
  cv::Mat dst;
  warpAffine(output, dst, rot_mat, output.size());

  PublishImage(dst);
}

//------------------------------------------------------------------------------
//
inline boost::any BlobDetector::ProcessData(boost::any input) {
  cv::Mat map = boost::any_cast<cv::Mat>(input);

  params_.minThreshold = min_threshold_;
  params_.maxThreshold = max_threshold_;
  params_.filterByArea = filter_by_area_;
  params_.minArea = min_area_ * 100;
  params_.maxArea = max_area_ * 100;
  params_.filterByColor = filter_by_color_;
  params_.blobColor = blob_color_;
  params_.filterByCircularity = filter_by_circularity_;
  params_.minCircularity = min_circularity_ / 99;
  params_.maxCircularity = max_circularity_ / 99;
  params_.filterByConvexity = filter_by_convexity_;
  params_.minConvexity = min_convexity_ / 99;
  params_.maxConvexity = max_convexity_ / 99;
  params_.filterByInertia = filter_by_inertia_;
  params_.minInertiaRatio = min_inertia_ratio_ / 99;
  params_.maxInertiaRatio = max_inertia_ratio_ / 99;

  cv::SimpleBlobDetector detector(params_);
  std::vector<cv::KeyPoint> keyPoints;
  detector.detect(map, keyPoints);

  cv::Mat output;
  cv::drawKeypoints(map, keyPoints, output, cv::Scalar(0, 0, 255),
                    cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

  // To fit in OpenCv coordinate system, we have to made a rotation of
  // 90 degrees on the display map
  cv::Point2f src_center(output.cols / 2.0f, output.rows / 2.0f);
  cv::Mat rot_mat = getRotationMatrix2D(src_center, 90, 1.0);
  cv::Mat dst;
  cv::warpAffine(output, dst, rot_mat, output.size());

  PublishImage(dst);

  return boost::any(keyPoints);
}

//------------------------------------------------------------------------------
//
inline void BlobDetector::ConfigureFromYamlNode(const YAML::Node &node) {
  min_threshold_ = node["min_threshold"].as<int>();
  max_threshold_ = node["max_threshold"].as<int>();
  filter_by_area_ = node["filter_by_area"].as<bool>();
  min_area_ = node["min_area"].as<int>();
  max_area_ = node["max_area"].as<int>();
  filter_by_color_ = node["filter_by_color"].as<bool>();
  blob_color_ = node["blob_color"].as<int>();
  filter_by_circularity_ = node["filter_by_circularity"].as<bool>();
  min_circularity_ = node["min_circularity"].as<int>();
  max_circularity_ = node["max_circularity"].as<int>();
  filter_by_convexity_ = node["filter_by_convexity"].as<bool>();
  min_convexity_ = node["min_convexity"].as<int>();
  max_convexity_ = node["max_convexity"].as<int>();
  filter_by_inertia_ = node["filter_by_inertia"].as<bool>();
  min_inertia_ratio_ = node["min_inertia"].as<int>();
  max_inertia_ratio_ = node["max_inertia"].as<int>();
}

}  // namespace proc_mapping

#endif  // PROC_MAPPING_PIPELINE_PROC_UNIT_BLOB_DETECTOR_H_
