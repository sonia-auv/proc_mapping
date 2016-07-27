/**
 * \file	proc_tree.cc
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	03/06/2016
 *
 * \copyright Copyright (c) 2016 S.O.N.I.A. All rights reserved.
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

#include "proc_mapping/pipeline/proc_tree.h"
#include "proc_mapping/pipeline/proc_unit/blob_detector.h"
#include "proc_mapping/pipeline/proc_unit/blur.h"
#include "proc_mapping/pipeline/proc_unit/buoys_detector.h"
#include "proc_mapping/pipeline/proc_unit/dilate.h"
#include "proc_mapping/pipeline/proc_unit/far_buoys_detector.h"
#include "proc_mapping/pipeline/proc_unit/fence_detector.h"
#include "proc_mapping/pipeline/proc_unit/histogram.h"
#include "proc_mapping/pipeline/proc_unit/morphology.h"
#include "proc_mapping/pipeline/proc_unit/threshold.h"
#include "proc_mapping/pipeline/proc_unit/wall_remover.h"

namespace proc_mapping {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
ProcTree::ProcTree(const YAML::Node &node, const ros::NodeHandlePtr &nh,
                   const ObjectRegistery::Ptr &object_registery)
    : nh_(nh),
      name_(""),
      proc_units_({}),
      object_registery_(object_registery) {
  Deserialize(node);
}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
bool ProcTree::ProcessData(boost::any input) const {
  for (const auto &pu : proc_units_) {
    input = pu->ProcessData(input);
  }

  bool success = false;
  try {
    success = boost::any_cast<bool>(input);
  } catch (const boost::bad_any_cast &e) {
    success = false;
  }
  return success;
}

//------------------------------------------------------------------------------
//
const std::string &ProcTree::GetName() const { return name_; }

//------------------------------------------------------------------------------
//
typename ProcUnit::Ptr ProcTree::ProcUnitFactory(const YAML::Node &node) const {
  ProcUnit::Ptr pu{nullptr};

  if (node["name"]) {
    auto proc_unit_name = node["name"].as<std::string>();
    auto publisher_namespace = kRosNodeName + name_ + "/";

    if (proc_unit_name == "blur") {
      pu = std::make_shared<Blur>(publisher_namespace);
    } else if (proc_unit_name == "threshold") {
      pu = std::make_shared<Threshold>(publisher_namespace);
    } else if (proc_unit_name == "wall_remover") {
      pu = std::make_shared<WallRemover>(publisher_namespace);
    } else if (proc_unit_name == "dilate") {
      pu = std::make_shared<Dilate>(publisher_namespace);
    } else if (proc_unit_name == "morphology") {
      pu = std::make_shared<Morphology>(publisher_namespace);
    } else if (proc_unit_name == "histogram") {
      pu = std::make_shared<Histogram>(publisher_namespace);
    } else if (proc_unit_name == "blob_detector") {
      pu = std::make_shared<BlobDetector>(publisher_namespace);
    } else if (proc_unit_name == "far_buoys_detector") {
      pu = std::make_shared<FarBuoysDetector>(publisher_namespace, object_registery_);
    } else if (proc_unit_name == "buoys_detector") {
      pu = std::make_shared<BuoysDetector>(publisher_namespace, object_registery_);
    } else if (proc_unit_name == "fence_detector") {
      pu = std::make_shared<FenceDetector>(publisher_namespace, object_registery_);
    } else {
      ROS_ERROR("There is no ProcUnit with such a name");
    }
  } else {
    ROS_ERROR("The proc tree file is not formatted correcly");
  }

  pu->Initialize(node);
  return pu;
}

//------------------------------------------------------------------------------
//
bool ProcTree::Deserialize(const YAML::Node &node) {
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

//------------------------------------------------------------------------------
//
ProcUnit::Ptr ProcTree::GetProcUnit(std::string &name) {
  for (const auto &pu : proc_units_) {
    if (pu->GetName() == name) {
      return pu;
    }
  }

  return nullptr;
}

//------------------------------------------------------------------------------
//
sonia_msgs::ProcTree ProcTree::BuildRosMessage() {
  sonia_msgs::ProcTree proc_tree_msg;
  proc_tree_msg.name = name_;

  std::vector<sonia_msgs::ProcUnit> proc_unit_list;
  std::vector<sonia_msgs::ProcUnitParameter> proc_unit_parameter_list;
  for (const auto &pu : proc_units_) {
    sonia_msgs::ProcUnit proc_unit;
    proc_unit.name = pu->GetName();
    for (const auto &pup : pu->GetParameters()) {
      sonia_msgs::ProcUnitParameter proc_unit_parameter;
      proc_unit_parameter = pup->BuildRosMessage();
      proc_unit_parameter_list.push_back(proc_unit_parameter);
    }
    proc_unit.proc_unit_parameter_list = proc_unit_parameter_list;
    proc_unit_list.push_back(proc_unit);
  }

  proc_tree_msg.proc_unit_list = proc_unit_list;

  return proc_tree_msg;
}

}  // namespace proc_mapping
