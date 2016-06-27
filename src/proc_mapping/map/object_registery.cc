/**
 * \file	object_registery.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	09/05/2016
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

#include "object_registery.h"

namespace proc_mapping {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
ObjectRegistery::ObjectRegistery() :
    objects_({}), is_registry_cleared_(false), object_mutex_() {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void ObjectRegistery::AddMapObject(const MapObject::Ptr &obj) {
  std::lock_guard<std::mutex> guard(object_mutex_);
  objects_.push_back(obj);
  is_registry_cleared_ = false;
}

//------------------------------------------------------------------------------
//
void ObjectRegistery::DeleteMapObject(const MapObject::Ptr &obj) {
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
const ObjectRegistery::MapObjectList &ObjectRegistery::GetAllMapObject() {
  std::lock_guard<std::mutex> guard(object_mutex_);
  return objects_;
}

//------------------------------------------------------------------------------
//
void ObjectRegistery::AddRegionOfInterest(const RegionOfInterest::Ptr &obj) {
  std::lock_guard<std::mutex> guard(object_mutex_);
  rois_.push_back(obj);
}

//------------------------------------------------------------------------------
//
void ObjectRegistery::DeleteRegionOfInterest(const RegionOfInterest::Ptr &obj) {
  std::lock_guard<std::mutex> guard(object_mutex_);
  auto find_cond = [&obj](const RegionOfInterest::Ptr &item) {
    return item.get() == obj.get();
  };

  auto it = std::find_if(rois_.begin(), rois_.end(), find_cond);
  if (it != rois_.end()) {
    rois_.erase(it);
  }
}

//------------------------------------------------------------------------------
//
const ObjectRegistery::RegionOfInterestList &
ObjectRegistery::GetAllRegionOfInterest() const {
  std::lock_guard<std::mutex> guard(object_mutex_);
  return rois_;
}

//------------------------------------------------------------------------------
//
const ObjectRegistery::RegionOfInterestList
ObjectRegistery::GetRegionOfInterestOfType(const DetectionMode type) const {
  RegionOfInterestList return_rois;
  for (const auto &roi : rois_) {
    if (roi->GetObjectType() == type) {
      return_rois.push_back(std::move(roi));
    }
  }
  return return_rois;
}

//------------------------------------------------------------------------------
//
template <class Tp_>
std::vector<std::shared_ptr<Tp_>> ObjectRegistery::GetObjectsOfType() const {
  std::vector<std::shared_ptr<Tp_>> return_objects;

  for (const auto &obj : objects_) {
    if (auto p = std::dynamic_pointer_cast<Tp_>(obj)) {
      return_objects.push_back(std::move(p));
    }
  }
  for (const auto &obj : rois_) {
    if (auto p = std::dynamic_pointer_cast<Tp_>(obj)) {
      return_objects.push_back(std::move(p));
    }
  }
  return return_objects;
}

//------------------------------------------------------------------------------
//
void ObjectRegistery::ClearRegistery() {
  std::lock_guard<std::mutex> guard(object_mutex_);
  is_registry_cleared_ = true;
  objects_.clear();
}

bool ObjectRegistery::IsRegisteryCleared() {
  return is_registry_cleared_;
}

void ObjectRegistery::ResetRegisteryClearedFlag() { is_registry_cleared_ = false; }

}  // namespace proc_mapping
