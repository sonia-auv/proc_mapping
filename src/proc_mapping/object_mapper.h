/**
 * \file	object_mapper.h
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

#ifndef PROC_MAPPING_OBJECT_MAPPER_H_
#define PROC_MAPPING_OBJECT_MAPPER_H_

#include <lib_atlas/macros.h>
#include <lib_atlas/pattern/observer.h>
#include <lib_atlas/pattern/subject.h>
#include <memory>
#include <vector>
#include "proc_mapping/interpreter/data_interpreter.h"
#include "proc_mapping/interpreter/weighted_object_id.h"

namespace proc_mapping {

class ObjectMapper
    : public atlas::Observer<const WeightedObjectId::ConstPtrList &> {
 public:
  //============================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<ObjectMapper>;
  using ConstPtr = std::shared_ptr<const ObjectMapper>;
  using PtrList = std::vector<ObjectMapper::Ptr>;
  using ConstPtrList = std::vector<ObjectMapper::ConstPtr>;

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit ObjectMapper(const ros::NodeHandlePtr &nh) ATLAS_NOEXCEPT;

  ~ObjectMapper() ATLAS_NOEXCEPT;

  //============================================================================
  // P U B L I C   M E T H O D S

 private:
  //============================================================================
  // P R I V A T E   M E T H O D S

  void OnSubjectNotify(
      atlas::Subject<const WeightedObjectId::ConstPtrList &> &subject,
      const WeightedObjectId::ConstPtrList &obj) ATLAS_NOEXCEPT override;

  //============================================================================
  // P R I V A T E   M E M B E R S

  ros::NodeHandlePtr nh_;

  DataInterpreterInterface::PtrList interpreters_;
};

}  // namespace proc_mapping

#endif  // PROC_MAPPING_OBJECT_MAPPER_H_
