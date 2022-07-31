//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xrot.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 31-Jul-2022 13:03:34
//

#ifndef XROT_H
#define XROT_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
namespace blas {
void b_xrot(int n, double x[16], int ix0, int iy0, double c, double s);

void xrot(float x[9], int ix0, int iy0, float c, float s);

void xrot(double x[9], int ix0, int iy0, double c, double s);

void xrot(int n, double x[16], int ix0, int iy0, double c, double s);

} // namespace blas
} // namespace internal
} // namespace coder

#endif
//
// File trailer for xrot.h
//
// [EOF]
//
