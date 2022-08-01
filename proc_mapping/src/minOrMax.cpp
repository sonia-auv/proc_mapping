//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: minOrMax.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 01-Aug-2022 08:26:09
//

// Include Files
#include "minOrMax.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cmath>
#include <string.h>

// Function Definitions
//
// Arguments    : const ::coder::array<double, 2U> &x
// Return Type  : double
//
namespace coder {
namespace internal {
double maximum(const ::coder::array<double, 2U> &x)
{
  double ex;
  int last;
  last = x.size(1);
  if (x.size(1) <= 2) {
    if (x.size(1) == 1) {
      ex = x[0];
    } else if ((x[0] < x[x.size(1) - 1]) ||
               (std::isnan(x[0]) && (!std::isnan(x[x.size(1) - 1])))) {
      ex = x[x.size(1) - 1];
    } else {
      ex = x[0];
    }
  } else {
    int idx;
    int k;
    if (!std::isnan(x[0])) {
      idx = 1;
    } else {
      bool exitg1;
      idx = 0;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= last)) {
        if (!std::isnan(x[k - 1])) {
          idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }
    if (idx == 0) {
      ex = x[0];
    } else {
      ex = x[idx - 1];
      idx++;
      for (k = idx; k <= last; k++) {
        double d;
        d = x[k - 1];
        if (ex < d) {
          ex = d;
        }
      }
    }
  }
  return ex;
}

//
// Arguments    : const double x[3]
// Return Type  : double
//
double maximum(const double x[3])
{
  double ex;
  int idx;
  int k;
  if (!std::isnan(x[0])) {
    idx = 1;
  } else {
    bool exitg1;
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k <= 3)) {
      if (!std::isnan(x[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }
  if (idx == 0) {
    ex = x[0];
  } else {
    ex = x[idx - 1];
    idx++;
    for (k = idx; k < 4; k++) {
      double d;
      d = x[k - 1];
      if (ex < d) {
        ex = d;
      }
    }
  }
  return ex;
}

//
// Arguments    : const ::coder::array<double, 1U> &x
// Return Type  : double
//
double maximum(const ::coder::array<double, 1U> &x)
{
  double ex;
  int last;
  last = x.size(0);
  if (x.size(0) <= 2) {
    if (x.size(0) == 1) {
      ex = x[0];
    } else if ((x[0] < x[x.size(0) - 1]) ||
               (std::isnan(x[0]) && (!std::isnan(x[x.size(0) - 1])))) {
      ex = x[x.size(0) - 1];
    } else {
      ex = x[0];
    }
  } else {
    int idx;
    int k;
    if (!std::isnan(x[0])) {
      idx = 1;
    } else {
      bool exitg1;
      idx = 0;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= last)) {
        if (!std::isnan(x[k - 1])) {
          idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }
    if (idx == 0) {
      ex = x[0];
    } else {
      ex = x[idx - 1];
      idx++;
      for (k = idx; k <= last; k++) {
        double d;
        d = x[k - 1];
        if (ex < d) {
          ex = d;
        }
      }
    }
  }
  return ex;
}

//
// Arguments    : const ::coder::array<float, 2U> &x
//                float ex[3]
// Return Type  : void
//
void maximum(const ::coder::array<float, 2U> &x, float ex[3])
{
  int m;
  m = x.size(0);
  for (int j{0}; j < 3; j++) {
    ex[j] = x[x.size(0) * j];
    for (int i{2}; i <= m; i++) {
      float b;
      bool p;
      b = x[(i + x.size(0) * j) - 1];
      if (std::isnan(b)) {
        p = false;
      } else if (std::isnan(ex[j])) {
        p = true;
      } else {
        p = (ex[j] < b);
      }
      if (p) {
        ex[j] = b;
      }
    }
  }
}

//
// Arguments    : const float x[3]
//                float *ex
//                int *idx
// Return Type  : void
//
void maximum(const float x[3], float *ex, int *idx)
{
  int k;
  if (!std::isnan(x[0])) {
    *idx = 1;
  } else {
    bool exitg1;
    *idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 4)) {
      if (!std::isnan(x[k - 1])) {
        *idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }
  if (*idx == 0) {
    *ex = x[0];
    *idx = 1;
  } else {
    int i;
    *ex = x[*idx - 1];
    i = *idx + 1;
    for (k = i; k < 4; k++) {
      float f;
      f = x[k - 1];
      if (*ex < f) {
        *ex = f;
        *idx = k;
      }
    }
  }
}

//
// Arguments    : const ::coder::array<double, 2U> &x
// Return Type  : double
//
double minimum(const ::coder::array<double, 2U> &x)
{
  double ex;
  int last;
  last = x.size(1);
  if (x.size(1) <= 2) {
    if (x.size(1) == 1) {
      ex = x[0];
    } else if ((x[0] > x[x.size(1) - 1]) ||
               (std::isnan(x[0]) && (!std::isnan(x[x.size(1) - 1])))) {
      ex = x[x.size(1) - 1];
    } else {
      ex = x[0];
    }
  } else {
    int idx;
    int k;
    if (!std::isnan(x[0])) {
      idx = 1;
    } else {
      bool exitg1;
      idx = 0;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= last)) {
        if (!std::isnan(x[k - 1])) {
          idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }
    if (idx == 0) {
      ex = x[0];
    } else {
      ex = x[idx - 1];
      idx++;
      for (k = idx; k <= last; k++) {
        double d;
        d = x[k - 1];
        if (ex > d) {
          ex = d;
        }
      }
    }
  }
  return ex;
}

//
// Arguments    : const ::coder::array<double, 1U> &x
// Return Type  : double
//
double minimum(const ::coder::array<double, 1U> &x)
{
  double ex;
  int last;
  last = x.size(0);
  if (x.size(0) <= 2) {
    if (x.size(0) == 1) {
      ex = x[0];
    } else if ((x[0] > x[x.size(0) - 1]) ||
               (std::isnan(x[0]) && (!std::isnan(x[x.size(0) - 1])))) {
      ex = x[x.size(0) - 1];
    } else {
      ex = x[0];
    }
  } else {
    int idx;
    int k;
    if (!std::isnan(x[0])) {
      idx = 1;
    } else {
      bool exitg1;
      idx = 0;
      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= last)) {
        if (!std::isnan(x[k - 1])) {
          idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }
    if (idx == 0) {
      ex = x[0];
    } else {
      ex = x[idx - 1];
      idx++;
      for (k = idx; k <= last; k++) {
        double d;
        d = x[k - 1];
        if (ex > d) {
          ex = d;
        }
      }
    }
  }
  return ex;
}

//
// Arguments    : const double x[3]
// Return Type  : double
//
double minimum(const double x[3])
{
  double ex;
  int idx;
  int k;
  if (!std::isnan(x[0])) {
    idx = 1;
  } else {
    bool exitg1;
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k <= 3)) {
      if (!std::isnan(x[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }
  if (idx == 0) {
    ex = x[0];
  } else {
    ex = x[idx - 1];
    idx++;
    for (k = idx; k < 4; k++) {
      double d;
      d = x[k - 1];
      if (ex > d) {
        ex = d;
      }
    }
  }
  return ex;
}

//
// Arguments    : const float x[3]
// Return Type  : float
//
float minimum(const float x[3])
{
  float ex;
  int idx;
  int k;
  if (!std::isnan(x[0])) {
    idx = 1;
  } else {
    bool exitg1;
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k <= 3)) {
      if (!std::isnan(x[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }
  if (idx == 0) {
    ex = x[0];
  } else {
    ex = x[idx - 1];
    idx++;
    for (k = idx; k < 4; k++) {
      float f;
      f = x[k - 1];
      if (ex > f) {
        ex = f;
      }
    }
  }
  return ex;
}

//
// Arguments    : const ::coder::array<float, 2U> &x
//                float ex[3]
// Return Type  : void
//
void minimum(const ::coder::array<float, 2U> &x, float ex[3])
{
  int m;
  m = x.size(0);
  for (int j{0}; j < 3; j++) {
    ex[j] = x[x.size(0) * j];
    for (int i{2}; i <= m; i++) {
      float b;
      bool p;
      b = x[(i + x.size(0) * j) - 1];
      if (std::isnan(b)) {
        p = false;
      } else if (std::isnan(ex[j])) {
        p = true;
      } else {
        p = (ex[j] > b);
      }
      if (p) {
        ex[j] = b;
      }
    }
  }
}

} // namespace internal
} // namespace coder

//
// File trailer for minOrMax.cpp
//
// [EOF]
//
