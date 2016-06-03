/**
 * \file	proc_unit.h
 * \author	Thibaut Mattio <thibaut.mattio@gmail.com>
 * \date	09/05/2016
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
#define PROC_MAPPING_PROC_UNIT_PROC_TREE_H_

#include <proc_mapping/proc_unit/proc_unit.h>
#include <ros/forwards.h>
#include <yaml-cpp/yaml.h>
#include <memory>
#include <vector>

namespace proc_mapping {

template <class Tp_>
class ProcTree {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<ProcTree>;
  using ConstPtr = std::shared_ptr<const ProcTree>;
  using PtrList = std::vector<ProcTree::Ptr>;
  using ConstPtrList = std::vector<ProcTree::ConstPtr>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit ProcTree(const YAML::Node &node, const ros::NodeHandlePtr &nh);

  virtual ~ProcTree() = default;

  //==========================================================================
  // P U B L I C   M E T H O D S

  void ProcessData(Tp_ &input) const;

 private:
  //==========================================================================
  // P R I V A T E   M E T H O D S

  typename ProcUnit<Tp_>::Ptr ProcUnitFactory(const YAML::Node &node) const;

  bool Deserialize(const YAML::Node &node);

  //==========================================================================
  // P R I V A T E   M E M B E R S

  ros::NodeHandlePtr nh_;

  std::string name_;

  std::vector<typename ProcUnit<Tp_>::Ptr> proc_units_;
};

}  // namespace proc_mapping

#include "proc_mapping/proc_unit/proc_tree_inl.h"

#endif  // PROC_MAPPING_PROC_UNIT_PROC_TREE_H_
