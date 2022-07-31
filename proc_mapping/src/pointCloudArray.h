//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: pointCloudArray.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 31-Jul-2022 13:03:34
//

#ifndef POINTCLOUDARRAY_H
#define POINTCLOUDARRAY_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
namespace coder {
namespace pointclouds {
namespace internal {
namespace codegen {
namespace pc {
class pointCloudArray {
};

class b_pointCloudArray {
public:
  array<float, 2U> Location;
  array<float, 2U> Normal;
  array<unsigned char, 2U> Color;
  array<float, 2U> Intensity;
};

} // namespace pc
} // namespace codegen
} // namespace internal
} // namespace pointclouds
} // namespace coder

#endif
//
// File trailer for pointCloudArray.h
//
// [EOF]
//
