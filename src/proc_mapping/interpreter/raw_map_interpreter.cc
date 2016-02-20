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

#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/common/transforms.h>
#include <tf/transform_datatypes.h>
#include "proc_mapping/interpreter/raw_map_interpreter.h"

namespace proc_mapping {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
RawMapInterpreter::RawMapInterpreter(const ros::NodeHandlePtr &nh)
    ATLAS_NOEXCEPT : DataInterpreter<cv::Mat>(nh),
                     nh_(nh),
                     map_(nh_) {
  Observe(map_);
}

//------------------------------------------------------------------------------
//
RawMapInterpreter::~RawMapInterpreter() ATLAS_NOEXCEPT {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
WeightedObjectId::ConstPtrList RawMapInterpreter::ProcessData() { return {{}}; }

//------------------------------------------------------------------------------
//
void RawMapInterpreter::OnSubjectNotify(atlas::Subject<cv::Mat> &subject,
                                        cv::Mat args) ATLAS_NOEXCEPT {
  SetNewData(args);
  cv::imshow("", args);
  cv::waitKey(1);
}

}  // namespace proc_mapping
