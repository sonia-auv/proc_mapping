/**
 * \file	region_of_interest.cc
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

#ifndef PROC_MAPPING_REGION_OF_INTEREST_H_
#define PROC_MAPPING_REGION_OF_INTEREST_H_

#include "proc_mapping/interpreter/map_interpreter.h"

namespace proc_mapping {

class RegionOfInterest {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<RegionOfInterest>;
  using ConstPtr = std::shared_ptr<const RegionOfInterest>;
  using PtrList = std::vector<RegionOfInterest::Ptr>;
  using ConstPtrList = std::vector<RegionOfInterest::ConstPtr>;

  // We don't want to use cv::Rect here because we want to be able to give an
  // orientation to the rectangle.
  using ContourType = std::vector<cv::Point2i>;
  using RotatedRectType = cv::RotatedRect;

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit RegionOfInterest(const YAML::Node &node);
  explicit RegionOfInterest(const std::string &name, const ContourType &contour,
                            const DetectionMode &mode = DetectionMode::NONE);

  ~RegionOfInterest() = default;

  //==========================================================================
  // P U B L I C   M E T H O D S

  const DetectionMode &GetObjectType() const;

  ContourType GetContour() const;
  RotatedRectType GetRotatedRect() const;


  bool IsInZone(const cv::Point2i &p) const;
  bool IsInZone(const cv::Rect &p) const;

  bool Deserialize(const YAML::Node &node);
  bool Serialize(const YAML::Node &node);

 private:
  //==========================================================================
  // P R I V A T E   M E M B E R S

  DetectionMode object_type_;

  std::string name_;

  ContourType contours_;

  cv::Point2d center_;
  cv::Size2d size_;
  double angle_;
};

}  // namespace proc_mapping

#endif  //  PROC_MAPPING_REGION_OF_INTEREST_H_
