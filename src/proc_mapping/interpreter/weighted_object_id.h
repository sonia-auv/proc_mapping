/**
 * \file	weighted_object_id.h
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

#ifndef PROC_MAPPING_INTERPRETER_WEIGHTED_OBJECT_ID_H_
#define PROC_MAPPING_INTERPRETER_WEIGHTED_OBJECT_ID_H_

#include <memory>
#include <vector>
#include <sonia_msgs/WeightedObjectId.h>
#include <lib_atlas/macros.h>

namespace proc_mapping {

/**
 * This class is a proxy for the sonia_msgs/WeightedObjectId class.
 * This provide simple method for serializing and deserializing the object
 * into/from the ROS message.
 */
class WeightedObjectId {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<WeightedObjectId>;
  using ConstPtr = std::shared_ptr<const WeightedObjectId>;
  using PtrList = std::vector<WeightedObjectId::Ptr>;
  using ConstPtrList = std::vector<WeightedObjectId::ConstPtr>;

  /**
   * This is the id of the object.
   * Caution with the values of the IDs. The same enum is being declared
   * in the sonia_msgs/WeightedObjectId file. For user simplicity, please,
   * use the same values.
   */
  enum class ObjectId {
    BUOY_ID = 0u,
    GREEN_BUOY_ID = 1u,
    RED_BUOY_ID = 2u,
    PIPE_ID = 3u,
    FENCE_ID = 4u,
    TRAIN_ID = 5u,
  };

  //==========================================================================
  // P U B L I C   C / D T O R S

  WeightedObjectId() ATLAS_NOEXCEPT;
  virtual ~WeightedObjectId() ATLAS_NOEXCEPT;

  //==========================================================================
  // P U B L I C   M E T H O D S

  sonia_msgs::WeightedObjectId::Ptr Serialize() const ATLAS_NOEXCEPT;

  void Deserialize(const sonia_msgs::WeightedObjectId::ConstPtr &obj)
      ATLAS_NOEXCEPT;

 private:
  //==========================================================================
  // P R I V A T E   M E M B E R S

  WeightedObjectId::ObjectId id_;

  double weight_;
};

}  // namespace proc_mapping

#endif  // PROC_MAPPING_INTERPRETER_WEIGHTED_OBJECT_ID_H_
