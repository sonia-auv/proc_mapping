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

#ifndef PROC_MAPPING_OBJECT_REGISTERY_H
#define PROC_MAPPING_OBJECT_REGISTERY_H

#include <sonia_msgs/MapObject.h>
#include <opencv>

namespace proc_mapping {

class ObjectRegistery {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using MapObject = cv::Point2d;
  using MapObjectPtr = std::shared_ptr<MapObject>;
  using MapObjectPtrList = std::vector<MapObjectPtr>;

  // Deleting the copy ctor for the Singleton pattern compliance.
  // Deleting move ctor as well, we have a mutex here anyway.
  ObjectRegistery(ObjectRegistery const &) = delete;
  ObjectRegistery(ObjectRegistery &&) = delete;

  void operator=(ObjectRegistery const &) = delete;
  void operator=(ObjectRegistery &&) = delete;

  static ObjectRegistery &GetInstance() {
    static ObjectRegistery instance;
    return instance;
  }

  //==========================================================================
  // P U B L I C   M E T H O D S

  void AddObject(const MapObjectPtr &obj);
  void DeleteObject(const MapObjectPtr &obj);

  const MapObjectPtrList &GetAllMapObject() const;

  void ClearRegistery();

 private:
  // As a Singleton, we want the object itself only to be able to create an
  // instance of itself.
  ObjectRegistery();

  //==========================================================================
  // P R I V A T E   M E M B E R S

  MapObjectPtrList objects_;

  /// We access the registry from the main thread as well as the processing
  /// thread (when we receive a scanline or an odometry). Thus, we must
  /// sync the access to the MapObjects.
  mutable std::mutex object_mutex_;
};

}  // namespace proc_mapping

#endif  // PROC_MAPPING_OBJECT_REGISTERY_H
