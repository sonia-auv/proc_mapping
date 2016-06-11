/**
 * \file	ellipse.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	07/06/2016
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

#ifndef PROC_MAPPING_REGION_OF_INTEREST_ELLIPSE_H_
#define PROC_MAPPING_REGION_OF_INTEREST_ELLIPSE_H_

#include <yaml-cpp/yaml.h>
#include <memory>
#include "proc_mapping/config.h"
#include "proc_mapping/interpreter/map_interpreter.h"
#include "proc_mapping/region_of_interest/region_of_interest.h"

namespace proc_mapping {

class Ellipse : public RegionOfInterest {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<Ellipse>;
  using ConstPtr = std::shared_ptr<const Ellipse>;
  using PtrList = std::vector<Ellipse::Ptr>;
  using ConstPtrList = std::vector<Ellipse::ConstPtr>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit Ellipse(const YAML::Node &node);
  explicit Ellipse(const std::string &name, const cv::Point2d &center,
                   const cv::Size &axes, double angle, double start_angle,
                   double end_angle,
                   const DetectionMode &mode = DetectionMode::NONE);

  virtual ~Ellipse() = default;

  //==========================================================================
  // P U B L I C   M E T H O D S

  virtual bool IsInZone(const cv::Point2d &p) const override;
  virtual bool IsInZone(const cv::Rect &p) const override;

  virtual void DrawRegion(cv::Mat mat,
                          const std::function<cv::Point2i(const cv::Point2d &p)>
                              &convert) const override;

  bool Deserialize(const YAML::Node &node) override;
  bool Serialize(const YAML::Node &node) override;

 private:
  //==========================================================================
  // P R I V A T E   M E M B E R S

  /// Center of the ellipse
  cv::Point2d center_;

  /// Half of the size of the ellipse main axes.
  cv::Size axes_;

  /// Ellipse rotation angle in degrees.
  double angle_;

  /// Starting angle of the elliptic arc in degrees.
  double start_angle_;

  /// Ending angle of the elliptic arc in degrees.
  double end_angle_;
};

}  // namespace proc_mapping

#endif  //  PROC_MAPPING_REGION_OF_INTEREST_ELLIPSE_H_
