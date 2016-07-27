/**
 * \file	config.h
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

#ifndef PROC_MAPPING_CONFIG_H_
#define PROC_MAPPING_CONFIG_H_

#include <lib_atlas/config.h>
#include <string>

#define DEBUG 1

const std::string kConfigFilePath =
    atlas::kWorkspaceRoot + "/src/proc_mapping/config/";
const std::string kRosNodeName = "/proc_mapping/";

enum class DetectionMode { NONE = 0, FAR_BUOYS, BUOYS, FENCE, WALL };

#endif  // PROC_MAPPING_CONFIG_H_
