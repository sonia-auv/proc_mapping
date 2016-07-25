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
      blur_type_server_(),
      name_(""),
      proc_units_({}),
      object_registery_(object_registery) {
  Deserialize(node);

  blur_type_server_ =
      nh_->advertiseService("blur_type_configuration" + name_ + "_",
                            &ProcTree::BlurTypeConfiguration, this);
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
  if (node["name"]) {
    auto proc_unit_name = node["name"].as<std::string>();

    if (proc_unit_name == "blur") {
      auto debug = node["debug"].as<bool>();
      auto blur_type = node["blur_type"].as<int>();
      auto kernel_size = node["kernel_size"].as<int>();
      return std::make_shared<Blur>(name_, blur_type, kernel_size, debug);
    } else if (proc_unit_name == "threshold") {
      auto debug = node["debug"].as<bool>();
      auto threshold_type = node["threshold_type"].as<int>();
      auto thresh_value = node["thresh_value"].as<int>();
      return std::make_shared<Threshold>(name_, threshold_type, thresh_value,
                                         debug);
    } else if (proc_unit_name == "wall_remover") {
      auto debug = node["debug"].as<bool>();
      return std::make_shared<WallRemover>(name_, debug);
    } else if (proc_unit_name == "dilate") {
      auto kernel_size_x = node["kernel_size_x"].as<int>();
      auto kernel_size_y = node["kernel_size_y"].as<int>();
      auto debug = node["debug"].as<bool>();
      return std::make_shared<Dilate>(name_, kernel_size_x, kernel_size_y,
                                      debug);
    } else if (proc_unit_name == "morphology") {
      auto kernel_size_x = node["kernel_size_x"].as<int>();
      auto kernel_size_y = node["kernel_size_y"].as<int>();
      auto debug = node["debug"].as<bool>();
      return std::make_shared<Morphology>(name_, kernel_size_x, kernel_size_y,
                                          debug);
    } else if (proc_unit_name == "histogram") {
      return std::make_shared<Histogram>(name_);
    } else if (proc_unit_name == "blob_detector") {
      auto debug = node["debug"].as<bool>();
      auto target = node["target"].as<int>();
      return std::make_shared<BlobDetector>(name_, target, debug);
    } else if (proc_unit_name == "far_buoys_detector") {
      return std::make_shared<FarBuoysDetector>(object_registery_);
    } else if (proc_unit_name == "buoys_detector") {
      auto roi = node["roi"].as<bool>();
      return std::make_shared<BuoysDetector>(object_registery_, roi);
    } else if (proc_unit_name == "fence_detector") {
      return std::make_shared<FenceDetector>(object_registery_);
    }

  } else {
    ROS_ERROR("The proc tree file is not formatted correcly");
  }
  ROS_ERROR("There is no ProcUnit with such a name");
  return nullptr;
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

//------------------------------------------------------------------------------
//
bool ProcTree::BlurTypeConfiguration(
    sonia_msgs::BlurTypeConfiguration::Request &req,
    sonia_msgs::BlurTypeConfiguration::Response &resp) {
  for (const auto &pu : proc_units_) {
    if (pu->GetName() == "blur") {
      auto blur = dynamic_cast<Blur *>(pu.get());
      if (blur != nullptr) {
        if (req.blur_type != blur->GetBlurType()) {
          blur->SetBlurType(req.blur_type);
        }
      }
    }
  }

  return true;
}

}  // namespace proc_mapping
