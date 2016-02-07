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

#ifndef PROC_MAPPING_INTERPRETER_DATA_INTERPRETER_H_
#define PROC_MAPPING_INTERPRETER_DATA_INTERPRETER_H_

#include <memory>
#include <vector>
#include <atomic>
#include <mutex>
#include <lib_atlas/macros.h>
#include "proc_mapping/interpreter/data_interpreter_interface.h"
#include "proc_mapping/interpreter/weighted_object_id.h"

namespace proc_mapping {

/**
 * A DataInterpreter is an object that subscribe to a source of information (
 * can be ROS, internal process or other source). The data that is being
 * received will be parsed in order to sort out a WeightedObjectId list.
 * The list that is being constructed every time the DataInterpreter receive
 * a new information is then send to the observers of this object.
 * See DataInterpreterInterface for public interface informations.
 */
template <class Tp_>
class DataInterpreter : public DataInterpreterInterface {
 public:
  //============================================================================
  // P U B L I C   C / D T O R S

  DataInterpreter() ATLAS_NOEXCEPT;

  virtual ~DataInterpreter() ATLAS_NOEXCEPT;

  //============================================================================
  // P U B L I C   M E T H O D S

 protected:
  //============================================================================
  // P R O T E C T E D   M E T H O D S

  /**
   * This is the most important method of the DataInterpreter as it is the one
   * that will return the WeightedObjects. When a new data is ready (should) be
   * set by the specific DataInterpreter, this method is called and a
   * notification will be sent to observers (particulary ObjectMapper)
   */
  virtual WeightedObjectId::ConstPtrList ProcessData() = 0;

  /**
   * The Run method is being override to allow the user to abstract the
   * managment of new data. The processing of the data will begin when a new
   * data is being set in the DataInterpreter.
   */
  void Run() override;

  Tp_ &GetLastData() const ATLAS_NOEXCEPT;

  void SetNewData(const Tp_ &data) ATLAS_NOEXCEPT;

  bool IsNewDataReady() const ATLAS_NOEXCEPT;

 private:
  //============================================================================
  // P R O T E C T E D   M E M B E R S

  std::atomic<bool> new_data_ready_;

  /**
   * Cannot use std::atomic here because it requieres to be nothrow constructor
   * type. Using mutext in the accessors of the variables instead.
   * (rf. http://cplusplus.github.io/LWG/lwg-defects.html#2165)
   */
  Tp_ last_data_;

  std::mutex data_mutex_;
};

}  // namespace proc_mapping

#include <proc_mapping/interpreter/data_interpreter_inl.h>

#endif  // PROC_MAPPING_INTERPRETER_DATA_INTERPRETER_H_
