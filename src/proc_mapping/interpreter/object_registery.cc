/**
 * \file	object_registery.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	09/05/2016
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

#include "object_registery.h"

namespace proc_mapping {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
ObjectRegistery::ObjectRegistery() : objects_({}), object_mutex_() {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void ObjectRegistery::AddObject(const MapObject::Ptr &obj) {
  std::lock_guard<std::mutex> guard(object_mutex_);
  objects_.push_back(obj);
}

//------------------------------------------------------------------------------
//
void ObjectRegistery::DeleteObject(const MapObject::Ptr &obj) {
  std::lock_guard<std::mutex> guard(object_mutex_);
  auto find_cond = [&obj](const MapObject::Ptr &item) {
    return item.get() == obj.get();
  };

  auto it = std::find_if(objects_.begin(), objects_.end(), find_cond);
  if (it != objects_.end()) {
    objects_.erase(it);
  }
}

//------------------------------------------------------------------------------
//
const ObjectRegistery::MapObjectList ObjectRegistery::GetAllMapObject() const {
  std::lock_guard<std::mutex> guard(object_mutex_);
  return objects_;
}

//------------------------------------------------------------------------------
//
void ObjectRegistery::ClearRegistery() {
  std::lock_guard<std::mutex> guard(object_mutex_);
  objects_.clear();
}

}  // namespace proc_mapping
