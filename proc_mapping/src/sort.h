//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: sort.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 01-Aug-2022 08:26:09
//

#ifndef SORT_H
#define SORT_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
void sort(::coder::array<unsigned int, 1U> &x);

void sort(::coder::array<double, 1U> &x, ::coder::array<int, 1U> &idx);

void sort(::coder::array<float, 1U> &x, ::coder::array<int, 1U> &idx);

} // namespace internal
} // namespace coder

#endif
//
// File trailer for sort.h
//
// [EOF]
//
