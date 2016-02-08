/**
 * \file	data_interpreter.cc
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

#ifndef PROC_MAPPING_INTERPRETER_DATA_INTERPRETER_H_
#error This file may only be included data_interpreter.h
#endif  // PROC_MAPPING_INTERPRETER_DATA_INTERPRETER_H_

#include "proc_mapping/interpreter/data_interpreter.h"

namespace proc_mapping {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_ALWAYS_INLINE DataInterpreter<Tp_>::DataInterpreter() ATLAS_NOEXCEPT
    : new_data_ready_(false),
      last_data_() {}

//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_ALWAYS_INLINE DataInterpreter<Tp_>::~DataInterpreter() ATLAS_NOEXCEPT {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_ALWAYS_INLINE void DataInterpreter<Tp_>::Run() {
  while (IsRunning()) {
    if(new_data_ready_) {
      Notify(ProcessData());
    }
  }
}

//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_ALWAYS_INLINE Tp_ &DataInterpreter<Tp_>::GetLastData() const
    ATLAS_NOEXCEPT {
  return last_data_;
}
//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_ALWAYS_INLINE void DataInterpreter<Tp_>::SetNewData(const Tp_ &data)
    ATLAS_NOEXCEPT {
  std::lock_guard<std::mutex> guard(data_mutex_);
  last_data_ = data;
  new_data_ready_ = true;
}

//------------------------------------------------------------------------------
//
template <class Tp_>
ATLAS_ALWAYS_INLINE bool DataInterpreter<Tp_>::IsNewDataReady() const
    ATLAS_NOEXCEPT {
  return new_data_ready_;
}

}  // namespace proc_mapping
