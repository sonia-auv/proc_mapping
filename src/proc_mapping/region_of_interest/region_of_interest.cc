/**
 * \file	semantic_map.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	31/05/2016
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

#include "region_of_interest.h"

namespace proc_mapping {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
RegionOfInterest::RegionOfInterest(const YAML::Node &node)
    : object_type_(DetectionMode::NONE) {
}

//------------------------------------------------------------------------------
//
RegionOfInterest::RegionOfInterest(const std::string &name,
                                   const DetectionMode &mode)
    : object_type_(mode), name_(name) {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
const DetectionMode &RegionOfInterest::GetObjectType() const {
  return object_type_;
}

//------------------------------------------------------------------------------
//
const std::string &RegionOfInterest::GetName() const { return name_; }

//------------------------------------------------------------------------------
//
void RegionOfInterest::SetObjectType(const DetectionMode &object_type) {
  object_type_ = object_type;
}

//------------------------------------------------------------------------------
//
void RegionOfInterest::SetName(const std::string &name) { name_ = name; }

}  // namespace proc_mapping
