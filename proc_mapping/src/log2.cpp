//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// log2.cpp
//
// Code generation for function 'log2'
//

// Include files
#include "log2.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <math.h>
#include <string.h>

// Function Definitions
namespace coder {
double b_log2(double x)
{
  double f;
  int eint;
  if (x == 0.0) {
    f = rtMinusInf;
  } else if (x < 0.0) {
    f = rtNaN;
  } else if ((!std::isinf(x)) && (!std::isnan(x))) {
    double t;
    t = frexp(x, &eint);
    if (t == 0.5) {
      f = static_cast<double>(eint) - 1.0;
    } else if ((eint == 1) && (t < 0.75)) {
      f = std::log(2.0 * t) / 0.69314718055994529;
    } else {
      f = std::log(t) / 0.69314718055994529 + static_cast<double>(eint);
    }
  } else {
    f = x;
  }
  return f;
}

} // namespace coder

// End of code generation (log2.cpp)
