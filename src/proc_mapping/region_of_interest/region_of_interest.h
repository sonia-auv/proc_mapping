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

#ifndef PROC_MAPPING_REGION_OF_INTEREST_REGION_OF_INTEREST_H_
#define PROC_MAPPING_REGION_OF_INTEREST_REGION_OF_INTEREST_H_

#include <highgui.h>
#include <yaml-cpp/yaml.h>
#include "proc_mapping/config.h"

namespace proc_mapping {

class RegionOfInterest {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<RegionOfInterest>;
  using ConstPtr = std::shared_ptr<const RegionOfInterest>;
  using PtrList = std::vector<RegionOfInterest::Ptr>;
  using ConstPtrList = std::vector<RegionOfInterest::ConstPtr>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit RegionOfInterest(const YAML::Node &node);
  explicit RegionOfInterest(const std::string &name,
                            const DetectionMode &mode = DetectionMode::NONE);

  virtual ~RegionOfInterest() = default;

  //==========================================================================
  // P U B L I C   M E T H O D S

  const DetectionMode &GetObjectType() const;
  void SetObjectType(const DetectionMode &object_type);

  const std::string &GetName() const;
  void SetName(const std::string &name);

  virtual bool IsInZone(const cv::Point2i &p) const = 0;
  virtual bool IsInZone(const cv::Rect &p) const = 0;

  virtual void DrawRegion(cv::Mat mat,
                          const std::function<cv::Point2i(const cv::Point2d &p)>
                              &convert) const = 0;

  virtual bool Deserialize(const YAML::Node &node) = 0;
  virtual bool Serialize(const YAML::Node &node) = 0;

 protected:
  //==========================================================================
  // P R O T E C T E D  M E T H O D S

  void DetectionModeFactory(const std::string &object_type);

 private:
  //==========================================================================
  // P R I V A T E   M E M B E R S

  DetectionMode object_type_;

  std::string name_;
};

}  // namespace proc_mapping

#endif  //  PROC_MAPPING_REGION_OF_INTEREST_REGION_OF_INTEREST_H_
