//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xrot.cpp
//
// Code generation for function 'xrot'
//

// Include files
#include "xrot.h"
#include "rt_nonfinite.h"
#include <string.h>

// Function Definitions
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

// End of code generation (xrot.cpp)
