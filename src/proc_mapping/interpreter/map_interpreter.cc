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
#include "proc_mapping/proc_unit/pattern_detection.h"

namespace proc_mapping {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
MapInterpreter::MapInterpreter(const ros::NodeHandlePtr &nh)
    : DataInterpreter<cv::Mat>(nh), nh_(nh) {
  ProcUnit<cv::Mat>::Ptr pu{new PatternDetection};
  AddProcUnit(std::move(pu));
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
  cv::Mat new_data;
  args.copyTo(new_data);
  SetNewData(new_data);
}

}  // namespace proc_mapping
