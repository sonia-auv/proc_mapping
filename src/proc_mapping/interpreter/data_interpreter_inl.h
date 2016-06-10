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

#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include "proc_mapping/interpreter/object_registery.h"

namespace proc_mapping {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
template <class Tp_>
inline DataInterpreter<Tp_>::DataInterpreter(const ros::NodeHandlePtr &nh)
    : nh_(nh), new_data_ready_(false), last_data_() {}

//------------------------------------------------------------------------------
//
template <class Tp_>
inline DataInterpreter<Tp_>::~DataInterpreter() {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
template <class Tp_>
inline Tp_ DataInterpreter<Tp_>::GetLastData() {
  std::lock_guard<std::mutex> guard(data_mutex_);
  new_data_ready_ = false;
  return last_data_;
}

//------------------------------------------------------------------------------
//
template <class Tp_>
inline void DataInterpreter<Tp_>::SetNewData(const Tp_ &data) {
  std::lock_guard<std::mutex> guard(data_mutex_);
  last_data_ = data;
  new_data_ready_ = true;
}

//------------------------------------------------------------------------------
//
template <class Tp_>
inline bool DataInterpreter<Tp_>::IsNewDataReady() const {
  std::lock_guard<std::mutex> guard(data_mutex_);
  return new_data_ready_;
}

}  // namespace proc_mapping
