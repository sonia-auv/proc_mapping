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

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include "proc_mapping/interpreter/data_interpreter.h"
#include "proc_mapping/interpreter/object_registery.h"

namespace proc_mapping {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
template <class Tp_>
inline DataInterpreter<Tp_>::DataInterpreter(
    const ros::NodeHandlePtr &nh, const std::string &proc_tree_file_name)
    : nh_(nh),
      all_proc_trees_(),
      current_proc_tree_(nullptr),
      new_data_ready_(false),
      last_data_() {
  InstanciateProcTrees(proc_tree_file_name + ".yaml");
}

//------------------------------------------------------------------------------
//
template <class Tp_>
inline DataInterpreter<Tp_>::~DataInterpreter() {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
template <class Tp_>
inline Tp_ &DataInterpreter<Tp_>::GetLastData() {
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

//------------------------------------------------------------------------------
//
template <class Tp_>
inline void DataInterpreter<Tp_>::ProcessData() {
  if (current_proc_tree_) {
    auto data = GetLastData();
    current_proc_tree_->ProcessData(data);
  }
}

//------------------------------------------------------------------------------
//
template <class Tp_>
bool DataInterpreter<Tp_>::SetCurrentProcTree(const std::string &name) {
  for (const auto &pt : all_proc_trees_) {
    if (pt->GetName().equals(name)) {
      current_proc_tree_ = pt;
      return true;
    }
  }
  ROS_ERROR("There is no proc tree with this name");
  return false;
}

//------------------------------------------------------------------------------
//
template <class Tp_>
bool DataInterpreter<Tp_>::SetCurrentProcTree(const ProcTreeType &proc_tree) {
  for (const auto &pt : all_proc_trees_) {
    if (&(pt.get()) == &(proc_tree.get())) {
      current_proc_tree_ = pt;
      return true;
    }
  }
  ROS_ERROR("There is no proc tree with this address");
  return false;
}

//------------------------------------------------------------------------------
//
template <class Tp_>
void DataInterpreter<Tp_>::InstanciateProcTrees(const std::string &proc_tree_file_name) {
  YAML::Node node = YAML::LoadFile(kProcTreesFilePath + proc_tree_file_name);

  std::string default_pt{""};
  if(node["default"]) {
    default_pt = node["default"].as<std::string>();
  }

  if (node["proc_trees"]) {
    auto proc_trees = node["proc_trees"];
    assert(proc_trees.Type() == YAML::NodeType::Sequence);

    for (std::size_t i = 0; i < proc_trees.size(); i++) {
      auto proc_tree = std::make_shared<ProcTree<Tp_>>(proc_trees[i],
                                                             nh_);
      if(!default_pt.empty() && default_pt == proc_trees[i]["name"]
          .as<std::string>()) {
        current_proc_tree_ = proc_tree;
      }
      all_proc_trees_.push_back(proc_tree);
    }
  }
}


//------------------------------------------------------------------------------
//
template <class Tp_>
void DataInterpreter<Tp_>::OnSubjectNotify(atlas::Subject<Tp_> &subject,
                                           Tp_ args) {
  ProcessData();
}

}  // namespace proc_mapping
