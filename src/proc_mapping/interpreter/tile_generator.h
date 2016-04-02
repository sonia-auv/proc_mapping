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

#ifndef PROC_MAPPING_INTERPRETER_TILE_GENERATOR_H_
#define PROC_MAPPING_INTERPRETER_TILE_GENERATOR_H_

#include <opencv/cv.h>
#include <memory>
#include "proc_mapping/types.h"

namespace proc_mapping {

/**
 * The tile generator provides a way to only process the updated region of the
 * raw_map.
 * This will allow easier treatment, more relevant information and faster
 * processing.
 */
class TileGenerator {
 public:
  //==========================================================================
  // T Y P E D E F   A N D   E N U M

  using Ptr = std::shared_ptr<TileGenerator>;
  using ConstPtr = std::shared_ptr<const TileGenerator>;
  using PtrList = std::vector<TileGenerator::Ptr>;
  using ConstPtrList = std::vector<TileGenerator::ConstPtr>;

  //==========================================================================
  // P U B L I C   C / D T O R S

  explicit TileGenerator(int scanlines_per_tile);

  ~TileGenerator() = default;

  //==========================================================================
  // P U B L I C   M E T H O D S

  inline bool IsTileReadyForProcess();

  // Tile GetTile(cv::Mat raw_map_);
  void UpdateTileBoundaries(cv::Point2i bin_coordinate);

  void SetScanlinePerTile(int scanlines_per_tile) {
    scanlines_per_tile_ = scanlines_per_tile;
  }

 private:
  //==========================================================================
  // P R I V A T E   M E M B E R S

  cv::Point2i tile_min_boundary_;
  cv::Point2i tile_max_boundary_;

  int scanlines_per_tile_;
  int scanline_counter_;

  bool is_tile_ready_for_process_;
};

//==============================================================================
// I N L I N E   F U N C T I O N S   D E F I N I T I O N S

//------------------------------------------------------------------------------
//
inline bool TileGenerator::IsTileReadyForProcess() {
  return is_tile_ready_for_process_;
}

}  // namespace proc_mapping

#endif  // PROC_MAPPING_INTERPRETER_TILE_GENERATOR_H_
