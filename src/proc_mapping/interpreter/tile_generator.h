//
// Created by etienne on 12/03/16.
//
// --
// The tile generator provides a way to only process the updated region of the raw_map
// This will allow easier treatment, more relevant information and faster processing.
#ifndef PROJECT_TILE_GENERATOR_H
#define PROJECT_TILE_GENERATOR_H
#include "proc_mapping_types.h"
#include <opencv/cv.h>
namespace proc_mapping{

class TileGenerator{
 public:

  TileGenerator(int scanlines_per_tile):scanlines_per_tile_(scanlines_per_tile),
  scanline_counter_(0), is_tile_ready_for_process_(false){
    tile_min_boundary_.x = 0;
    tile_min_boundary_.y = 0;
    tile_max_boundary_.x = 0;
    tile_max_boundary_.y = 0;
  }
  ~TileGenerator(){}
  inline bool IsTileReadyForProcess();
  Tile GetTile(cv::Mat raw_map_);
  void UpdateTileBoundaries(PointXY<int> bin_coordinate);
  void SetScanlinePerTile(int scanlines_per_tile){
    scanlines_per_tile_ = scanlines_per_tile;
  }
 private:
  PointXY<int> tile_min_boundary_, tile_max_boundary_;
  int scanlines_per_tile_, scanline_counter_;
  bool is_tile_ready_for_process_;
};
inline bool TileGenerator::IsTileReadyForProcess(){
  return is_tile_ready_for_process_;
}

}

#endif //PROJECT_TILE_GENERATOR_H
