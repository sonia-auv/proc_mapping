//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// norm.cpp
//
// Code generation for function 'norm'
//

// Include files
#include "norm.h"
#include "rt_nonfinite.h"
#include "svd.h"
#include "coder_array.h"
#include <cmath>
#include <string.h>

// Function Definitions
namespace coder {
double b_norm(const ::coder::array<double, 2U> &x)
{
  array<double, 1U> s;
  double y;
  bool MATRIX_INPUT_AND_P_IS_TWO;
  bool VECTOR_INPUT_AND_P_IS_NUMERIC;
  VECTOR_INPUT_AND_P_IS_NUMERIC = false;
  MATRIX_INPUT_AND_P_IS_TWO = false;
  if (x.size(0) == 1) {
    VECTOR_INPUT_AND_P_IS_NUMERIC = true;
  } else {
    MATRIX_INPUT_AND_P_IS_TWO = true;
  }
  if (x.size(0) == 0) {
    y = 0.0;
  } else if (MATRIX_INPUT_AND_P_IS_TWO) {
    int m;
    m = x.size(0);
    y = 0.0;
    for (int k{0}; k < 3; k++) {
      for (int i{0}; i < m; i++) {
        double scale;
        scale = std::abs(x[i + x.size(0) * k]);
        if (std::isnan(scale) || (scale > y)) {
          y = scale;
        }
      }
    }
    if ((!std::isinf(y)) && (!std::isnan(y))) {
      internal::svd(x, s);
      y = s[0];
    }
  } else if (VECTOR_INPUT_AND_P_IS_NUMERIC) {
    int m;
    m = x.size(0) * 3;
    y = 0.0;
    if (m >= 1) {
      if (m == 1) {
        y = std::abs(x[0]);
      } else {
        double scale;
        scale = 3.3121686421112381E-170;
        for (int k{0}; k < m; k++) {
          double absxk;
          absxk = std::abs(x[k]);
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
  } else {
    y = rtNaN;
  }
  return y;
}

} // namespace coder

// End of code generation (norm.cpp)
