//
// Created by etienne on 12/03/16.
//

#ifndef PROJECT_PROC_MAPPING_TYPES_H
#define PROJECT_PROC_MAPPING_TYPES_H

template <typename T>
struct PointXY {
  T x, y;
};

struct Tile{
  PointXY<int> offset;
  cv::Mat data;
};

#endif //PROJECT_PROC_MAPPING_TYPES_H
