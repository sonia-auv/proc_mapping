//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xnrm2.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 31-Jul-2022 13:03:34
//

// Include Files
#include "xnrm2.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <string.h>

// Function Definitions
//
// Arguments    : int n
//                const double x[16]
//                int ix0
// Return Type  : double
//
namespace coder {
namespace internal {
namespace blas {
double b_xnrm2(int n, const double x[16], int ix0)
{
  double y;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = std::abs(x[ix0 - 1]);
    } else {
      double scale;
      int kend;
      scale = 3.3121686421112381E-170;
      kend = (ix0 + n) - 1;
      for (int k{ix0}; k <= kend; k++) {
        double absxk;
        absxk = std::abs(x[k - 1]);
        if (absxk > scale) {
          double t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          double t;
          t = absxk / scale;
          y += t * t;
        }
      }
      y = scale * std::sqrt(y);
    }
  }
  return y;
}

//
// Arguments    : int n
//                const float x[9]
//                int ix0
// Return Type  : float
//
float xnrm2(int n, const float x[9], int ix0)
{
  float y;
  y = 0.0F;
  if (n >= 1) {
    if (n == 1) {
      y = std::abs(x[ix0 - 1]);
    } else {
      float scale;
      int kend;
      scale = 1.29246971E-26F;
      kend = (ix0 + n) - 1;
      for (int k{ix0}; k <= kend; k++) {
        float absxk;
        absxk = std::abs(x[k - 1]);
        if (absxk > scale) {
          float t;
          t = scale / absxk;
          y = y * t * t + 1.0F;
          scale = absxk;
        } else {
          float t;
          t = absxk / scale;
          y += t * t;
        }
      }
      y = scale * std::sqrt(y);
    }
  }
  return y;
}

//
// Arguments    : const float x[3]
// Return Type  : float
//
float xnrm2(const float x[3])
{
  float scale;
  float y;
  y = 0.0F;
  scale = 1.29246971E-26F;
  for (int k{2}; k < 4; k++) {
    float absxk;
    absxk = std::abs(x[k - 1]);
    if (absxk > scale) {
      float t;
      t = scale / absxk;
      y = y * t * t + 1.0F;
      scale = absxk;
    } else {
      float t;
      t = absxk / scale;
      y += t * t;
    }
  }
  return scale * std::sqrt(y);
}

//
// Arguments    : int n
//                const double x[3]
// Return Type  : double
//
double xnrm2(int n, const double x[3])
{
  double y;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = std::abs(x[1]);
    } else {
      double scale;
      int kend;
      scale = 3.3121686421112381E-170;
      kend = n + 1;
      for (int k{2}; k <= kend; k++) {
        double absxk;
        absxk = std::abs(x[k - 1]);
        if (absxk > scale) {
          double t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          double t;
          t = absxk / scale;
          y += t * t;
        }
      }
      y = scale * std::sqrt(y);
    }
  }
  return y;
}

//
// Arguments    : int n
//                const double x[9]
//                int ix0
// Return Type  : double
//
double xnrm2(int n, const double x[9], int ix0)
{
  double y;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = std::abs(x[ix0 - 1]);
    } else {
      double scale;
      int kend;
      scale = 3.3121686421112381E-170;
      kend = (ix0 + n) - 1;
      for (int k{ix0}; k <= kend; k++) {
        double absxk;
        absxk = std::abs(x[k - 1]);
        if (absxk > scale) {
          double t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          double t;
          t = absxk / scale;
          y += t * t;
        }
      }
      y = scale * std::sqrt(y);
    }
  }
  return y;
}

//
// Arguments    : const double x[3]
// Return Type  : double
//
double xnrm2(const double x[3])
{
  double scale;
  double y;
  y = 0.0;
  scale = 3.3121686421112381E-170;
  for (int k{2}; k < 4; k++) {
    double absxk;
    absxk = std::abs(x[k - 1]);
    if (absxk > scale) {
      double t;
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      double t;
      t = absxk / scale;
      y += t * t;
    }
  }
  return scale * std::sqrt(y);
}

} // namespace blas
} // namespace internal
} // namespace coder

//
// File trailer for xnrm2.cpp
//
// [EOF]
//
