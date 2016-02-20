/**
 * \file	object_mapper.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	07/02/2016
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

#include "proc_mapping/object_mapper.h"
#include "proc_mapping/interpreter/raw_map_interpreter.h"

namespace proc_mapping {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
ObjectMapper::ObjectMapper(const ros::NodeHandlePtr &nh) ATLAS_NOEXCEPT
    : nh_(nh),
      interpreters_() {
  RawMapInterpreter::Ptr raw_map = std::make_shared<RawMapInterpreter>(nh);
  interpreters_.push_back(raw_map);
}

//------------------------------------------------------------------------------
//
ObjectMapper::~ObjectMapper() ATLAS_NOEXCEPT {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void ObjectMapper::OnSubjectNotify(
    atlas::Subject<const WeightedObjectId::ConstPtrList &> &subject,
    const WeightedObjectId::ConstPtrList &obj) ATLAS_NOEXCEPT {}

}  // namespace proc_mapping
