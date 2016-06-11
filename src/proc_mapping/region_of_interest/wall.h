/**
 * \file	buoy.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	10/06/2016
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

#ifndef PROC_MAPPING_MAP_OBJECT_WALL_H_
#define PROC_MAPPING_MAP_OBJECT_WALL_H_

#include <memory>
#include <string>
#include <vector>
#include "proc_mapping/region_of_interest/contour.h"

namespace proc_mapping {

class Wall : public Contour {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<Wall>;
  using ConstPtr = std::shared_ptr<const Wall>;
  using PtrList = std::vector<Wall::Ptr>;
  using ConstPtrList = std::vector<Wall::ConstPtr>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit Wall();
  virtual ~Wall();

  //==========================================================================
  // P U B L I C   M E T H O D S

};

}  // namespace proc_mapping

#endif  // PROC_MAPPING_MAP_OBJECT_WALL_H_
