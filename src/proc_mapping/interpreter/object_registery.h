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

namespace proc_mapping {

class ObjectRegistery {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using MapObject = sonia_msgs::MapObject::Ptr;

  ObjectRegistery(ObjectRegistery const&) = delete;
  void operator=(ObjectRegistery const&)  = delete;

  static ObjectRegistery& GetInstance()
  {
    static ObjectRegistery instance;
    return instance;
  }

  //==========================================================================
  // P U B L I C   M E T H O D S

  void AddObject(const MapObject &obj) noexcept;

  void DeleteObject(const MapObject &obj) noexcept;
  void DeleteObject(const MapObject *obj) noexcept;

  const std::vector<MapObject> &GetAllMapObject() const noexcept;

  void ClearRegistery() noexcept;

 private:
  // As a Singleton, we want the object itself only to be able to create an
  // instance of itself.
  ObjectRegistery() = default;

  //==========================================================================
  // P R I V A T E   M E M B E R S

  std::vector<MapObject> objects_;
};

} // namespace proc_mapping

#endif //PROC_MAPPING_OBJECT_REGISTERY_H
