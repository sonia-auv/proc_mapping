//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: PointCloud2Types.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 31-Jul-2022 13:03:34
//

#ifndef POINTCLOUD2TYPES_H
#define POINTCLOUD2TYPES_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
namespace coder {
namespace ros {
namespace msg {
namespace sensor_msgs {
namespace internal {
class PointCloud2Types {
public:
  static void rosToMATLABType(unsigned char type,
                              ::coder::array<char, 2U> &mlType,
                              double *numBytes);
};

} // namespace internal
} // namespace sensor_msgs
} // namespace msg
} // namespace ros
} // namespace coder

#endif
//
// File trailer for PointCloud2Types.h
//
// [EOF]
//
