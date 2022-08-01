//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: voxelGridFilter.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 01-Aug-2022 08:26:09
//

#ifndef VOXELGRIDFILTER_H
#define VOXELGRIDFILTER_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace vision {
namespace internal {
namespace codegen {
namespace pc {
void voxelGridFilter(const ::coder::array<double, 2U> &location,
                     const ::coder::array<unsigned char, 2U> &color,
                     const ::coder::array<double, 2U> &normal,
                     const ::coder::array<double, 1U> &intensity,
                     const ::coder::array<double, 2U> &rangeData,
                     double voxelSize,
                     ::coder::array<double, 2U> &filteredLocation,
                     ::coder::array<unsigned char, 2U> &filteredColor,
                     ::coder::array<double, 2U> &filteredNormal,
                     ::coder::array<double, 1U> &filteredIntensity,
                     ::coder::array<double, 2U> &filteredRangeData);

}
} // namespace codegen
} // namespace internal
} // namespace vision
} // namespace coder

#endif
//
// File trailer for voxelGridFilter.h
//
// [EOF]
//
