//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xaxpy.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 01-Aug-2022 08:26:09
//

#ifndef XAXPY_H
#define XAXPY_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
namespace blas {
void xaxpy(float a, const float x[9], int ix0, float y[3]);

void xaxpy(float a, const float x[3], float y[9], int iy0);

void xaxpy(int n, float a, int ix0, float y[9], int iy0);

void xaxpy(double a, const double x[9], int ix0, double y[3]);

void xaxpy(double a, const double x[3], double y[9], int iy0);

void xaxpy(int n, double a, int ix0, double y[9], int iy0);

} // namespace blas
} // namespace internal
} // namespace coder

#endif
//
// File trailer for xaxpy.h
//
// [EOF]
//