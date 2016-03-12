//
// Created by etienne on 12/03/16.
//
#include "proc_mapping/interpreter/tile_generator.h"

namespace proc_mapping{


void TileGenerator::UpdateTileBoundaries(PointXY<int> bin_coordinate) {
    if (scanline_counter_ == 0){
      tile_max_boundary_ = bin_coordinate;
      tile_min_boundary_ = bin_coordinate;
    }
    if (bin_coordinate.x > tile_max_boundary_.x){
      tile_max_boundary_.x = bin_coordinate.x;
    }
    else if (bin_coordinate.x < tile_min_boundary_.x){
      tile_min_boundary_.x = bin_coordinate.x;
    }
    if (bin_coordinate.y > tile_max_boundary_.y){
      tile_max_boundary_.y = bin_coordinate.y;
    }
    else if(bin_coordinate.y < tile_min_boundary_.y){
      tile_min_boundary_.y = bin_coordinate.y;
    }
    scanline_counter_ ++;
    if (scanline_counter_ >= scanlines_per_tile_){
      is_tile_ready_for_process_ = true;
      scanline_counter_ = 0;
    }
  }

}