/**
 * \file	raw_map_interpreter.cc
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

#include "proc_mapping/interpreter/map_interpreter.h"
#include <tf/transform_datatypes.h>
#include <ros/console.h>

namespace proc_mapping {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
MapInterpreter::MapInterpreter(const ros::NodeHandlePtr &nh,
                               const std::string &proc_trees_file_name)
    : DataInterpreter<cv::Mat>(nh, proc_trees_file_name),
      mode_(DetectionMode::NONE) {
  SetDetectionMode(mode_);
  InstanciateProcTrees(proc_trees_file_name + ".yaml");
}

//------------------------------------------------------------------------------
//
MapInterpreter::~MapInterpreter() {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void MapInterpreter::OnSubjectNotify(atlas::Subject<cv::Mat> &subject,
                                     cv::Mat args) {
  /// We want to recreate a new map everytime so the processed map does not
  /// interfer with the real one.
  cv::Mat new_data;
  args.copyTo(new_data);
  SetNewData(new_data);
  ProcessData();

  // Notify potential observers that we just added new objects in the registery.
  Notify(mode_);
}

//------------------------------------------------------------------------------
//
void MapInterpreter::SetDetectionMode(const DetectionMode &mode) {
  if (mode == DetectionMode::BUOYS) {
    SetCurrentProcTree("buoys");
  } else if (mode == DetectionMode::FENCE) {
    SetCurrentProcTree("fence");
  } else {
    mode_ = DetectionMode::NONE;
    SetCurrentProcTree(ProcTree<cv::Mat>::Ptr(nullptr));
    return;
  }
  mode_ = mode;
}

//------------------------------------------------------------------------------
//
void MapInterpreter::InstanciateProcTrees(
    const std::string &proc_tree_file_name) {
  YAML::Node node = YAML::LoadFile(kProcTreesFilePath + proc_tree_file_name);

  std::string default_pt{""};
  if (node["default"]) {
    default_pt = node["default"].as<std::string>();
  }

  if (node["proc_trees"]) {
    auto proc_trees = node["proc_trees"];
    assert(proc_trees.Type() == YAML::NodeType::Sequence);

    for (std::size_t i = 0; i < proc_trees.size(); i++) {
      auto proc_tree = std::make_shared<ProcTree<cv::Mat>>(proc_trees[i], nh_);
      all_proc_trees_.push_back(proc_tree);
      if (!default_pt.empty() &&
          default_pt == proc_trees[i]["name"].as<std::string>()) {
        if(default_pt == "buoys") {
          SetDetectionMode(DetectionMode::BUOYS);
        } else if (default_pt == "fence"){
          SetDetectionMode(DetectionMode::FENCE);
        } else {
          ROS_ERROR("Trying to set default proc tree: There is no proc tree "
                        "with this name");
        }
      }
    }
  }
}

}  // namespace proc_mapping
