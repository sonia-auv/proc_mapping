//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xrot.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 01-Aug-2022 08:26:09
//

// Include Files
#include "xrot.h"
#include "rt_nonfinite.h"
#include <string.h>

// Function Definitions
//
// Arguments    : int n
//                double x[16]
//                int ix0
//                int iy0
//                double c
//                double s
// Return Type  : void
//
namespace coder {
namespace internal {
namespace blas {
void b_xrot(int n, double x[16], int ix0, int iy0, double c, double s)
{
  if (n >= 1) {
    for (int k{0}; k < n; k++) {
      double b_temp_tmp;
      double d_temp_tmp;
      int c_temp_tmp;
      int temp_tmp;
      temp_tmp = (iy0 + k) - 1;
      b_temp_tmp = x[temp_tmp];
      c_temp_tmp = (ix0 + k) - 1;
      d_temp_tmp = x[c_temp_tmp];
      x[temp_tmp] = c * b_temp_tmp - s * d_temp_tmp;
      x[c_temp_tmp] = c * d_temp_tmp + s * b_temp_tmp;
    }
  }
}

//
// Arguments    : float x[9]
//                int ix0
//                int iy0
//                float c
//                float s
// Return Type  : void
//
void xrot(float x[9], int ix0, int iy0, float c, float s)
{
  float temp;
  float temp_tmp;
  temp = x[iy0 - 1];
  temp_tmp = x[ix0 - 1];
  x[iy0 - 1] = c * temp - s * temp_tmp;
  x[ix0 - 1] = c * temp_tmp + s * temp;
  temp = c * x[ix0] + s * x[iy0];
  x[iy0] = c * x[iy0] - s * x[ix0];
  x[ix0] = temp;
  temp = x[iy0 + 1];
  temp_tmp = x[ix0 + 1];
  x[iy0 + 1] = c * temp - s * temp_tmp;
  x[ix0 + 1] = c * temp_tmp + s * temp;
}

//
// Arguments    : double x[9]
//                int ix0
//                int iy0
//                double c
//                double s
// Return Type  : void
//
void xrot(double x[9], int ix0, int iy0, double c, double s)
{
  double temp;
  double temp_tmp;
  temp = x[iy0 - 1];
  temp_tmp = x[ix0 - 1];
  x[iy0 - 1] = c * temp - s * temp_tmp;
  x[ix0 - 1] = c * temp_tmp + s * temp;
  temp = c * x[ix0] + s * x[iy0];
  x[iy0] = c * x[iy0] - s * x[ix0];
  x[ix0] = temp;
  temp = x[iy0 + 1];
  temp_tmp = x[ix0 + 1];
  x[iy0 + 1] = c * temp - s * temp_tmp;
  x[ix0 + 1] = c * temp_tmp + s * temp;
}

//
// Arguments    : int n
//                double x[16]
//                int ix0
//                int iy0
//                double c
//                double s
// Return Type  : void
//
void xrot(int n, double x[16], int ix0, int iy0, double c, double s)
{
  if (n >= 1) {
    for (int k{0}; k < n; k++) {
      double c_temp_tmp;
      double d_temp_tmp;
      int b_temp_tmp;
      int temp_tmp;
      temp_tmp = k << 2;
      b_temp_tmp = (iy0 + temp_tmp) - 1;
      c_temp_tmp = x[b_temp_tmp];
      temp_tmp = (ix0 + temp_tmp) - 1;
      d_temp_tmp = x[temp_tmp];
      x[b_temp_tmp] = c * c_temp_tmp - s * d_temp_tmp;
      x[temp_tmp] = c * d_temp_tmp + s * c_temp_tmp;
    }
  }
}

} // namespace blas
} // namespace internal
} // namespace coder

//
// File trailer for xrot.cpp
//
// [EOF]
//
