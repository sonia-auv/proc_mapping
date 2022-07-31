//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xaxpy.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 31-Jul-2022 13:03:34
//

// Include Files
#include "xaxpy.h"
#include "rt_nonfinite.h"
#include <string.h>

// Function Definitions
//
// Arguments    : float a
//                const float x[9]
//                int ix0
//                float y[3]
// Return Type  : void
//
namespace coder {
namespace internal {
namespace blas {
void xaxpy(float a, const float x[9], int ix0, float y[3])
{
  if (!(a == 0.0F)) {
    for (int k{0}; k < 2; k++) {
      y[k + 1] += a * x[(ix0 + k) - 1];
    }
  }
}

//
// Arguments    : float a
//                const float x[3]
//                float y[9]
//                int iy0
// Return Type  : void
//
void xaxpy(float a, const float x[3], float y[9], int iy0)
{
  if (!(a == 0.0F)) {
    for (int k{0}; k < 2; k++) {
      int i;
      i = (iy0 + k) - 1;
      y[i] += a * x[k + 1];
    }
  }
}

//
// Arguments    : int n
//                float a
//                int ix0
//                float y[9]
//                int iy0
// Return Type  : void
//
void xaxpy(int n, float a, int ix0, float y[9], int iy0)
{
  if ((n >= 1) && (!(a == 0.0F))) {
    int i;
    i = n - 1;
    for (int k{0}; k <= i; k++) {
      int i1;
      i1 = (iy0 + k) - 1;
      y[i1] += a * y[(ix0 + k) - 1];
    }
  }
}

//
// Arguments    : double a
//                const double x[9]
//                int ix0
//                double y[3]
// Return Type  : void
//
void xaxpy(double a, const double x[9], int ix0, double y[3])
{
  if (!(a == 0.0)) {
    for (int k{0}; k < 2; k++) {
      y[k + 1] += a * x[(ix0 + k) - 1];
    }
  }
}

//
// Arguments    : double a
//                const double x[3]
//                double y[9]
//                int iy0
// Return Type  : void
//
void xaxpy(double a, const double x[3], double y[9], int iy0)
{
  if (!(a == 0.0)) {
    for (int k{0}; k < 2; k++) {
      int i;
      i = (iy0 + k) - 1;
      y[i] += a * x[k + 1];
    }
  }
}

//
// Arguments    : int n
//                double a
//                int ix0
//                double y[9]
//                int iy0
// Return Type  : void
//
void xaxpy(int n, double a, int ix0, double y[9], int iy0)
{
  if ((n >= 1) && (!(a == 0.0))) {
    int i;
    i = n - 1;
    for (int k{0}; k <= i; k++) {
      int i1;
      i1 = (iy0 + k) - 1;
      y[i1] += a * y[(ix0 + k) - 1];
    }
  }
}

} // namespace blas
} // namespace internal
} // namespace coder

//
// File trailer for xaxpy.cpp
//
// [EOF]
//
