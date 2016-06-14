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

#include "proc_mapping/proc_unit/proc_tree.h"
#include "proc_mapping/proc_unit/blob_detector.h"
#include "proc_mapping/proc_unit/blur.h"
#include "proc_mapping/proc_unit/buoys_detector.h"
#include "proc_mapping/proc_unit/dilate.h"
#include "proc_mapping/proc_unit/morphology.h"
#include "proc_mapping/proc_unit/threshold.h"

namespace proc_mapping {

int BlobDetector::Parameters::filter_area_off = 1;
const int BlobDetector::Parameters::filter_area_on = 1;
int BlobDetector::Parameters::min_area = 0;
const int BlobDetector::Parameters::min_area_max = 3000;
int BlobDetector::Parameters::max_area = 0;
const int BlobDetector::Parameters::max_area_max = 3000;
int BlobDetector::Parameters::filter_circularity_off = 0;
const int BlobDetector::Parameters::filter_circularity_on = 1;
int BlobDetector::Parameters::min_circularity = 0;
const int BlobDetector::Parameters::min_circularity_max = 100;
int BlobDetector::Parameters::max_circularity = 0;
const int BlobDetector::Parameters::max_circularity_max = 100;
int BlobDetector::Parameters::filter_convexity_off = 0;
const int BlobDetector::Parameters::filter_convexity_on = 1;
int BlobDetector::Parameters::min_convexity = 0;
const int BlobDetector::Parameters::min_convexity_max = 10;
int BlobDetector::Parameters::max_convexity = 0;
const int BlobDetector::Parameters::max_convexity_max = 10;
int BlobDetector::Parameters::filter_inertial_off = 0;
const int BlobDetector::Parameters::filter_inertial_on = 1;
int BlobDetector::Parameters::min_inertia_ratio = 0;
const int BlobDetector::Parameters::min_inertia_ratio_max = 10;
int BlobDetector::Parameters::max_inertia_ratio = 0;
const int BlobDetector::Parameters::max_inertia_ratio_max = 10;

const int Blur::Parameters::kernel_size_max = 15;
int Blur::Parameters::kernel_size = 5;

const int Dilate::Parameters::kernel_size_x_max = 32;
const int Dilate::Parameters::kernel_size_y_max = 32;
int Dilate::Parameters::kernel_size_x = 5;
int Dilate::Parameters::kernel_size_y = 5;

const int Threshold::Parameters::thresh_value_max = 255;
int Threshold::Parameters::thresh_value = 0;

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
ProcTree::ProcTree(const YAML::Node &node, const ros::NodeHandlePtr &nh,
                   const ObjectRegistery::Ptr &object_registery)
    : nh_(nh), proc_units_({}), object_registery_(object_registery) {
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
    } else if (proc_unit_name == "buoys_detector") {
      return std::make_shared<BuoysDetector>(object_registery_);
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

}  // namespace proc_mapping
