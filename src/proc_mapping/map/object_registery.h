/**
 * \file	object_registery.h
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

#ifndef PROC_MAPPING_INTERPRETER_OBJECT_REGISTERY_H_
#define PROC_MAPPING_INTERPRETER_OBJECT_REGISTERY_H_

#include <opencv/cv.h>
#include <mutex>
#include "proc_mapping/map_objects/map_object.h"
#include "proc_mapping/region_of_interest/region_of_interest.h"

namespace proc_mapping {

class ObjectRegistery {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<ObjectRegistery>;
  using ConstPtr = std::shared_ptr<const ObjectRegistery>;
  using PtrList = std::vector<ObjectRegistery::Ptr>;
  using ConstPtrList = std::vector<ObjectRegistery::ConstPtr>;

  using MapObjectList = std::vector<MapObject::Ptr>;
  using RegionOfInterestList = std::vector<RegionOfInterest::Ptr>;

  // As a Singleton, we want the object itself only to be able to create an
  // instance of itself.
  ObjectRegistery();

  //==========================================================================
  // P U B L I C   M E T H O D S

  void AddMapObject(const MapObject::Ptr &obj);
  void DeleteMapObject(const MapObject::Ptr &obj);
  const MapObjectList &GetAllMapObject();

  void AddRegionOfInterest(const RegionOfInterest::Ptr &obj);
  void DeleteRegionOfInterest(const RegionOfInterest::Ptr &obj);
  const RegionOfInterestList &GetAllRegionOfInterest() const;

  const RegionOfInterestList GetRegionOfInterestOfType(
      const DetectionMode type) const;

  /// Template method that will return a vector of all the object of the
  /// specified type to the user.
  /// e.g. calling GetObjectOfType<Buoy> will return all the buoys from the
  /// ObjectRegistery.
  template <class Tp_>
  std::vector<std::shared_ptr<Tp_>> GetObjectsOfType() const;

  void ClearRegistery();

  bool IsRegisteryCleared();

  void ResetRegisteryClearedFlag();

 private:
  //==========================================================================
  // P R I V A T E   M E M B E R S

  MapObjectList objects_;
  RegionOfInterestList rois_;
  bool is_registry_cleared_;

  /// We access the registry from the main thread as well as the processing
  /// thread (when we receive a scanline or an odometry). Thus, we must
  /// sync the access to the MapObjects.
  mutable std::mutex object_mutex_;
};

}  // namespace proc_mapping

#endif  // PROC_MAPPING_INTERPRETER_OBJECT_REGISTERY_H_
