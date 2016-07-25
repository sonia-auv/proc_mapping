/**
 * \file	raw_map_interpreter.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	07/02/2016
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

#ifndef PROC_MAPPING_INTERPRETER_RAW_MAP_INTERPRETER_H_
#define PROC_MAPPING_INTERPRETER_RAW_MAP_INTERPRETER_H_

#include <lib_atlas/pattern/subject.h>
#include <ros/ros.h>
#include <memory>
#include <string>
#include <vector>
#include "proc_mapping/config.h"
#include "proc_mapping/interpreter/data_interpreter.h"
#include "proc_mapping/map/object_registery.h"
#include "proc_mapping/pipeline/proc_tree.h"

namespace proc_mapping {

/// The MapInterpreter is responsible for interpretting the informations of the
/// Raw map. It listen for new RawMap data to be available and execute a
/// proccessing tree for discovering MapObject.
/// It specify the DataInterpreter as a cv::Mat as it will deal with cv::Mat
/// as its data to interpret.
class MapInterpreter : public DataInterpreter<cv::Mat>,
                       public atlas::Observer<cv::Mat> {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<MapInterpreter>;
  using ConstPtr = std::shared_ptr<const MapInterpreter>;
  using PtrList = std::vector<MapInterpreter::Ptr>;
  using ConstPtrList = std::vector<MapInterpreter::ConstPtr>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit MapInterpreter(const ros::NodeHandlePtr &nh,
                          const std::string &proc_trees_file_name,
                          const ObjectRegistery::Ptr &object_registery);

  virtual ~MapInterpreter();

  void SetDetectionMode(const DetectionMode &mode);

  bool SetCurrentProcTree(const std::string &name);
  bool SetCurrentProcTree(const ProcTree::Ptr &proc_tree);

  const std::vector<ProcTree::Ptr> GetProcTreeList() const;

 private:
  //==========================================================================
  // P R I V A T E   M E T H O D S

  /// The method will update the latest data in the DataInterpreter.
  /// This will run the whole processing chain as the method SetNewData
  /// start it.
  void OnSubjectNotify(atlas::Subject<cv::Mat> &subject, cv::Mat args) override;

  void InstanciateProcTrees(const std::string &proc_tree_file_name);

  /// This will return true if the pipeline could detect new objects.
  virtual bool ProcessData() override;

  //==========================================================================
  // P R I V A T E   M E M B E R S

  DetectionMode mode_;

  std::vector<ProcTree::Ptr> all_proc_trees_;
  ProcTree::Ptr current_proc_tree_;

  mutable std::mutex proc_tree_mutex_;
};

}  // namespace proc_mapping

#endif  // PROC_MAPPING_INTERPRETER_RAW_MAP_INTERPRETER_H_
