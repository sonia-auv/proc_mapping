/**
 * \file	proc_tree.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	03/06/2016
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

#ifndef PROC_MAPPING_PROC_UNIT_PROC_TREE_H_
#error This file may only be included proc_tree.h
#endif  // PROC_MAPPING_PROC_UNIT_PROC_TREE_H_

#include <fstream>
#include <string>
#include "proc_mapping/config.h"
#include "proc_mapping/proc_unit/blob_detector.h"
#include "proc_mapping/proc_unit/blur.h"
#include "proc_mapping/proc_unit/dilate.h"
#include "proc_mapping/proc_unit/threshold.h"

namespace proc_mapping {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
template <class Tp_>
ProcTree<Tp_>::ProcTree(const YAML::Node &node, const ros::NodeHandlePtr &nh)
    : nh_(nh), proc_units_({}) {
  Deserialize(node);
}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
template <class Tp_>
void ProcTree<Tp_>::ProcessData(Tp_ &input) const {
  for (const auto &pu : proc_units_) {
    pu->ProcessData(input);
  }
}

//------------------------------------------------------------------------------
//
template <class Tp_>
const std::string &ProcTree<Tp_>::GetName() const {
  return name_;
}

//------------------------------------------------------------------------------
//
template <class Tp_>
typename ProcUnit<Tp_>::Ptr ProcTree<Tp_>::ProcUnitFactory(
    const YAML::Node &node) const {
  if (node["name"]) {
    auto proc_unit_name = node["name"].as<std::string>();

    if (proc_unit_name == "blur") {
      auto debug = node["debug"].as<bool>();
      auto blur_type = node["blur_type"].as<int>();
      return std::make_shared<Blur>(blur_type, debug);
    } else if (proc_unit_name == "threshold") {
      auto debug = node["debug"].as<bool>();
      auto threshold_type = node["threshold_type"].as<int>();
      return std::make_shared<Threshold>(threshold_type, debug);
    } else if (proc_unit_name == "dilate") {
      auto debug = node["debug"].as<bool>();
      return std::make_shared<Dilate>(debug);
    } else if (proc_unit_name == "blob_detector") {
      auto debug = node["debug"].as<bool>();
      auto target = node["target"].as<int>();
      return std::make_shared<BlobDetector>(target, debug);
    }

  } else {
    ROS_ERROR("The proc tree file is not formatted correcly");
  }
  ROS_ERROR("There is no ProcUnit with such a name");
  return nullptr;
}

//------------------------------------------------------------------------------
//
template <class Tp_>
bool ProcTree<Tp_>::Deserialize(const YAML::Node &node) {
  name_ = node["name"].as<std::string>();

  auto proc_units = node["proc_units"];
  assert(proc_units.Type() == YAML::NodeType::Sequence);

  for (std::size_t j = 0; j < proc_units.size(); j++) {
    auto name = proc_units[j]["name"].as<std::string>();
    auto proc_unit = ProcUnitFactory(proc_units[j]);

    if (proc_unit) {
      proc_units_.push_back(std::move(proc_unit));
    }
  }
  return true;
}

}  // namespace proc_mapping
