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

#ifndef PROC_MAPPING_PIPELINE_PROC_UNIT_BLUR_H_
#define PROC_MAPPING_PIPELINE_PROC_UNIT_BLUR_H_

#include <opencv/cv.h>
#include "proc_mapping/pipeline/proc_unit.h"

namespace proc_mapping {

class Blur : public ProcUnit {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<Blur>;
  using ConstPtr = std::shared_ptr<const Blur>;
  using PtrList = std::vector<Blur::Ptr>;
  using ConstPtrList = std::vector<Blur::ConstPtr>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit Blur(std::string proc_tree_name = "", int blur_type = 1,
                int kernel_size = 0, bool debug = false)
      : blur_type_(blur_type),
        kernel_size_(kernel_size),
        debug(debug),
        image_publisher_(kRosNodeName + "_blur_" + proc_tree_name) {
    image_publisher_.Start();
  };

  virtual ~Blur() = default;

  //==========================================================================
  // P U B L I C   M E T H O D S

  virtual boost::any ProcessData(boost::any input) override {
    cv::Mat map = boost::any_cast<cv::Mat>(input);
    // To keep the kernel size odd, multiply by 2 and add 1
    cv::Size2i kernel(kernel_size_ * 2 + 1, kernel_size_ * 2 + 1);

    // Bilateral Filter need another Mat to do algorithm
    cv::Mat dst = map.clone();
    if (blur_type_ == 0) {
      cv::blur(map, map, kernel);
    } else if (blur_type_ == 1) {
      cv::GaussianBlur(map, map, kernel, 0, 0);
    } else if (blur_type_ == 2) {
      cv::medianBlur(map, map, kernel_size_ * 2 + 1);
    } else if (blur_type_ == 3) {
      cv::bilateralFilter(map, dst, kernel_size_ * 2 + 1,
                          (kernel_size_ * 2 + 1) * 2,
                          (kernel_size_ * 2 + 1) / 2);
      input = dst;
    } else {
      ROS_ERROR("Blur Type is undefined");
    }
    if (debug) {
      if (blur_type_ == 3) {
        // To fit in OpenCv coordinate system, we have to made a rotation of
        // 90 degrees on the display map
        cv::Point2f src_center(dst.cols / 2.0f, dst.rows / 2.0f);
        cv::Mat rot_mat = getRotationMatrix2D(src_center, 90, 1.0);
        cv::Mat blur_dst;
        cv::warpAffine(map, blur_dst, rot_mat, map.size());

        cvtColor(blur_dst, blur_dst, CV_GRAY2RGB);
        image_publisher_.Write(blur_dst);
      } else {
        // To fit in OpenCv coordinate system, we have to made a rotation of
        // 90 degrees on the display map
        cv::Point2f src_center(map.cols / 2.0f, map.rows / 2.0f);
        cv::Mat rot_mat = getRotationMatrix2D(src_center, 90, 1.0);
        cv::Mat blur_dst;
        cv::warpAffine(map, blur_dst, rot_mat, map.size());

        cvtColor(blur_dst, blur_dst, CV_GRAY2RGB);
        image_publisher_.Write(blur_dst);
      }
    }
    return boost::any(map);
  }

  std::string GetName() const override { return "blur"; }

  inline int GetBlurType() { return blur_type_; }

  inline void SetBlurType(int blur_type) { blur_type_ = blur_type; }

  inline int GetKernelSize() { return kernel_size_; }

  inline void SetKernelSize(int kernel_size) { kernel_size_ = kernel_size; }

 private:
  //==========================================================================
  // P U B L I C   M E M B B E R S

  /*
 * 0: Homogeneous Blur
 * 1: Gaussian Blur
 * 2: Median Blur
 * 3: Bilateral Blur
 */
  int blur_type_;
  int kernel_size_;
  bool debug;

  atlas::ImagePublisher image_publisher_;
};

}  // namespace proc_mapping

#endif  // PROC_MAPPING_PIPELINE_PROC_UNIT_BLUR_H_