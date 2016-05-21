/**
 * \file	proc_unit.h
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

#ifndef PROC_MAPPING_PROC_UNIT_H
#define PROC_MAPPING_PROC_UNIT_H

#include <memory.h>

namespace proc_mapping {

/**
 * Implementing the ProcUnit as a template because the DataInterpreter will
 * use a templated type for the data to process.
 * For now there is only cv::Mat, but later there will be the VisionTargets.
 * This is a workaround to stay compatible, but a good implementation would
 * allow us to connect an output to a different one depending on the type
 * (e.g. not the same input/output types)
 */
template <class Tp_>
class ProcUnit {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<ProcUnit>;
  using ConstPtr = std::shared_ptr<const ProcUnit>;
  using PtrList = std::vector<ProcUnit::Ptr>;
  using ConstPtrList = std::vector<ProcUnit::ConstPtr>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  ProcUnit(){};

  virtual ~ProcUnit() = default;

  //==========================================================================
  // P U B L I C   M E T H O D S

  virtual void ProcessData(Tp_ &input) = 0;
};

}  // namespace proc_mapping

#endif  // PROC_MAPPING_PROC_UNIT_H