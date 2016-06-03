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

}  // namespace proc_mapping
