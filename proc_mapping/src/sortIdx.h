//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: sortIdx.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 01-Aug-2022 08:26:09
//

#ifndef SORTIDX_H
#define SORTIDX_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
void merge_block(::coder::array<int, 1U> &idx,
                 ::coder::array<unsigned int, 1U> &x, int offset, int n,
                 int preSortLevel, ::coder::array<int, 1U> &iwork,
                 ::coder::array<unsigned int, 1U> &xwork);

void merge_block(::coder::array<int, 1U> &idx, ::coder::array<double, 1U> &x,
                 int offset, int n, int preSortLevel,
                 ::coder::array<int, 1U> &iwork,
                 ::coder::array<double, 1U> &xwork);

void merge_block(::coder::array<int, 1U> &idx, ::coder::array<float, 1U> &x,
                 int offset, int n, int preSortLevel,
                 ::coder::array<int, 1U> &iwork,
                 ::coder::array<float, 1U> &xwork);

} // namespace internal
} // namespace coder

#endif
//
// File trailer for sortIdx.h
//
// [EOF]
//
