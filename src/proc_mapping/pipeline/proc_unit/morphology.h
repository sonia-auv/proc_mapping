/**
 * \file	morphology.h
 * \author	Francis Masse <francis.masse05@gmail.com>
 * \date	06/06/2016
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

#ifndef PROC_MAPPING_PIPELINE_PROC_UNIT_MORPHOLOGY_H
#define PROC_MAPPING_PIPELINE_PROC_UNIT_MORPHOLOGY_H

#include <opencv/cv.h>
#include "proc_mapping/pipeline/proc_unit.h"

namespace proc_mapping {

class Morphology : public ProcUnit {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<Morphology>;
  using ConstPtr = std::shared_ptr<const Morphology>;
  using PtrList = std::vector<Morphology::Ptr>;
  using ConstPtrList = std::vector<Morphology::ConstPtr>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit Morphology(const std::string &topic_namespace);

  virtual ~Morphology() = default;

  //==========================================================================
  // P U B L I C   M E T H O D S

  virtual void ConfigureFromYamlNode(const YAML::Node &node) override;

  virtual boost::any ProcessData(boost::any input) override {
    cv::Mat map = boost::any_cast<cv::Mat>(input);
    cv::Mat element = cv::getStructuringElement(
        0, cv::Size(kernel_size_x_.GetValue() * 2 + 1,
                    kernel_size_y_.GetValue() * 2 + 1));

    cv::morphologyEx(map, map, cv::MORPH_CLOSE, element);

    // To fit in OpenCv coordinate system, we have to made a rotation of
    // 90 degrees on the display map
    cv::Point2f src_center(map.cols / 2.0f, map.rows / 2.0f);
    cv::Mat rot_mat = getRotationMatrix2D(src_center, 90, 1.0);
    cv::Mat dst;
    cv::warpAffine(map, dst, rot_mat, map.size());

    cvtColor(dst, dst, CV_GRAY2RGB);
    PublishImage(dst);

    return boost::any(map);
  }

  std::string GetName() const override;

 private:
  Parameter<int> kernel_size_x_;
  Parameter<int> kernel_size_y_;
};

//==============================================================================
// I N L I N E   M E T H O D S

//------------------------------------------------------------------------------
//
inline Morphology::Morphology(const std::string &topic_namespace)
    : ProcUnit(topic_namespace),
      kernel_size_x_("Kernel Size X", 5, parameters_),
      kernel_size_y_("Kernel Size Y", 5, parameters_) {}

//------------------------------------------------------------------------------
//
inline void Morphology::ConfigureFromYamlNode(const YAML::Node &node) {
  kernel_size_x_ = node["kernel_size_x"].as<int>();
  kernel_size_y_ = node["kernel_size_y"].as<int>();
}

//------------------------------------------------------------------------------
//
inline std::string Morphology::GetName() const { return "morphology"; }

}  // namespace proc_mapping

#endif  // PROC_MAPPING_PIPELINE_PROC_UNIT_MORPHOLOGY_H
