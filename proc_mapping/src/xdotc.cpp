//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xdotc.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 01-Aug-2022 08:26:09
//

// Include Files
#include "xdotc.h"
#include "rt_nonfinite.h"
#include <string.h>

// Function Definitions
//
// Arguments    : int n
//                const float x[9]
//                int ix0
//                const float y[9]
//                int iy0
// Return Type  : float
//
namespace coder {
namespace internal {
namespace blas {
float xdotc(int n, const float x[9], int ix0, const float y[9], int iy0)
{
  float d;
  d = 0.0F;
  if (n >= 1) {
    for (int k{0}; k < n; k++) {
      d += x[(ix0 + k) - 1] * y[(iy0 + k) - 1];
    }
  }
  return d;
}

//
// Arguments    : int n
//                const double x[9]
//                int ix0
//                const double y[9]
//                int iy0
// Return Type  : double
//
double xdotc(int n, const double x[9], int ix0, const double y[9], int iy0)
{
  double d;
  d = 0.0;
  if (n >= 1) {
    for (int k{0}; k < n; k++) {
      d += x[(ix0 + k) - 1] * y[(iy0 + k) - 1];
    }
  }
  return d;
}

} // namespace blas
} // namespace internal
} // namespace coder

//
// File trailer for xdotc.cpp
//
// [EOF]
//
