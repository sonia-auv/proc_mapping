//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: minOrMax.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 31-Jul-2022 13:03:34
//

#ifndef MINORMAX_H
#define MINORMAX_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
double maximum(const ::coder::array<double, 2U> &x);

double maximum(const double x[3]);

double maximum(const ::coder::array<double, 1U> &x);

void maximum(const ::coder::array<float, 2U> &x, float ex[3]);

void maximum(const float x[3], float *ex, int *idx);

double minimum(const ::coder::array<double, 2U> &x);

double minimum(const ::coder::array<double, 1U> &x);

double minimum(const double x[3]);

float minimum(const float x[3]);

void minimum(const ::coder::array<float, 2U> &x, float ex[3]);

} // namespace internal
} // namespace coder

#endif
//
// File trailer for minOrMax.h
//
// [EOF]
//
