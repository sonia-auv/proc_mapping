/**
 * \file	buoy.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	10/06/2016
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

#ifndef PROC_MAPPING_MAP_OBJECT_BUOY_H_
#define PROC_MAPPING_MAP_OBJECT_BUOY_H_

#include <memory>
#include <string>
#include <vector>
#include "proc_mapping/map_objects/map_object.h"

namespace proc_mapping {

class Buoy : public MapObject {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<Buoy>;
  using ConstPtr = std::shared_ptr<const Buoy>;
  using PtrList = std::vector<Buoy::Ptr>;
  using ConstPtrList = std::vector<Buoy::ConstPtr>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit Buoy(const cv::KeyPoint &key_point);
  virtual ~Buoy();

  //==========================================================================
  // P U B L I C   M E T H O D S

  virtual void DrawToMap(cv::Mat) const override;

  virtual visualization_msgs::Marker GenerateVisualizationMarker(
      int id) const override;

 protected:
  //==========================================================================
  // P R O T E C T E D   M E T H O D S

  virtual uint8_t GetMessageObjectType() const override;
};

}  // namespace proc_mapping

#endif  // PROC_MAPPING_MAP_OBJECT_BUOY_H_
