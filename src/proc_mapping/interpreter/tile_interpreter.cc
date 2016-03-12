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

#include "proc_mapping/interpreter/tile_interpreter.h"
#include <opencv/cv.h>
#include <pcl/common/transforms.h>
#include <tf/transform_datatypes.h>
#include <opencv2/highgui/highgui.hpp>

namespace proc_mapping {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
TileInterpreter::TileInterpreter(const ros::NodeHandlePtr &nh) ATLAS_NOEXCEPT
    : DataInterpreter<cv::Mat>(nh),
      nh_(nh),
      map_(nh_) {
  Observe(map_);
}

//------------------------------------------------------------------------------
//
TileInterpreter::~TileInterpreter() ATLAS_NOEXCEPT {}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
WeightedObjectId::ConstPtrList TileInterpreter::ProcessData() { return {{}}; }

//------------------------------------------------------------------------------
//
void TileInterpreter::OnSubjectNotify(atlas::Subject<cv::Mat> &subject,
                                      cv::Mat args) ATLAS_NOEXCEPT {
  static int image_counter = 0, image_number = 0;
  SetNewData(args);
  cv::imshow("", args);
  cv::waitKey(1);
  image_counter++;

  if (image_counter == 100) {
    std::string filename =
        "/home/etienne/Documents/mapping_img/images/cv_mat_regular_mean_" +
        std::to_string(image_number) + ".png";
    cv::imwrite(filename, args);
    image_number++;
    image_counter = 0;
  }
}

}  // namespace proc_mapping
