/**
 * \file	raw_map_interpreter.h
 * \author	Etienne Boudreault-Pilon <etienne.b.pilon@gmail.com>
 * \date	12/03/2016
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

#include "proc_mapping/interpreter/tile_generator.h"

namespace proc_mapping {

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
TileGenerator::TileGenerator(int scanlines_per_tile)
    : scanlines_per_tile_(scanlines_per_tile),
      scanline_counter_(0),
      is_tile_ready_for_process_(false) {
  tile_min_boundary_.x = 10000;
  tile_min_boundary_.y = 10000;
  tile_max_boundary_.x = 0;
  tile_max_boundary_.y = 0;
}

//==============================================================================
// M E T H O D   S E C T I O N

//------------------------------------------------------------------------------
//
void TileGenerator::UpdateTileBoundaries(cv::Point2i bin_coordinate) {

  if (scanline_counter_ == 0) {
    tile_max_boundary_ = bin_coordinate;
  }

  tile_max_boundary_.x = std::max(bin_coordinate.x, tile_max_boundary_.x);
  tile_max_boundary_.y = std::max(bin_coordinate.y, tile_max_boundary_.y);
  tile_min_boundary_.x = std::min(bin_coordinate.x, tile_min_boundary_.x);
  tile_min_boundary_.y = std::min(bin_coordinate.y, tile_min_boundary_.y);

  scanline_counter_++;
  if (scanline_counter_ >= scanlines_per_tile_) {
    is_tile_ready_for_process_ = true;
    scanline_counter_ = 0;
  }
}

}  // namespace proc_mapping
