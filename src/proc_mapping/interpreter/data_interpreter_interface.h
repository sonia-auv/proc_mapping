/**
 * \file	data_interpreter_interface.h
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

#ifndef PROC_MAPPING_INTERPRETER_DATA_INTERPRETER_INTERFACE_H_
#define PROC_MAPPING_INTERPRETER_DATA_INTERPRETER_INTERFACE_H_

#include <lib_atlas/macros.h>
#include <lib_atlas/pattern/runnable.h>
#include <lib_atlas/pattern/subject.h>
#include <memory>
#include <vector>
#include "proc_mapping/interpreter/weighted_object_id.h"

namespace proc_mapping {

/**
 * This is a simple interface for the DataInterpreter so we are able to store
 * pointers of it.
 * We especially need this in the ObjectMapper object that agregate this a
 * list of DataInterpreter.
 */
class DataInterpreterInterface
    : public atlas::Subject<const WeightedObjectId::ConstPtrList &>,
      public atlas::Runnable {
 public:
  //============================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<DataInterpreterInterface>;
  using ConstPtr = std::shared_ptr<const DataInterpreterInterface>;
  using PtrList = std::vector<DataInterpreterInterface::Ptr>;
  using ConstPtrList = std::vector<DataInterpreterInterface::ConstPtr>;

  //============================================================================
  // P U B L I C   C / D T O R S

  explicit DataInterpreterInterface(const ros::NodeHandlePtr &nh)
      ATLAS_NOEXCEPT {}

  virtual ~DataInterpreterInterface() ATLAS_NOEXCEPT {}
};

}  // namespace proc_mapping

#endif  // PROC_MAPPING_INTERPRETER_DATA_INTERPRETER_INTERFACE_H_
