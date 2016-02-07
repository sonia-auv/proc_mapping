/**
 * \file	data_interpreter.h
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

#ifndef PROC_MAPPING_INTERPRETER_VISION_INTERPRETER_H_
#define PROC_MAPPING_INTERPRETER_VISION_INTERPRETER_H_

#include <memory>
#include <vector>
#include <lib_atlas/macros.h>
#include <sonia_msgs/WeightedObjectId.h>  // to change for ObjectFullData
#include "proc_mapping/interpreter/data_interpreter.h"

namespace proc_mapping {

class VisionInterpreter : public DataInterpreter<sonia_msgs::WeightedObjectId> {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<VisionInterpreter>;
  using ConstPtr = std::shared_ptr<const VisionInterpreter>;
  using PtrList = std::vector<VisionInterpreter::Ptr>;
  using ConstPtrList = std::vector<VisionInterpreter::ConstPtr>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  VisionInterpreter() ATLAS_NOEXCEPT;

  virtual ~VisionInterpreter() ATLAS_NOEXCEPT;

  //==========================================================================
  // P U B L I C   M E T H O D S
};

}  // namespace proc_mapping

#endif  // PROC_MAPPING_INTERPRETER_VISION_INTERPRETER_H_
