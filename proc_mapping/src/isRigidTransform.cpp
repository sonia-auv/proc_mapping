//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: isRigidTransform.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 31-Jul-2022 13:03:34
//

// Include Files
#include "isRigidTransform.h"
#include "proc_mapping_rtwutil.h"
#include "rt_nonfinite.h"
#include "svd1.h"
#include <algorithm>
#include <cmath>
#include <string.h>

// Function Definitions
//
// Arguments    : const float T[16]
// Return Type  : bool
//
namespace coder {
namespace vision {
namespace internal {
bool isRigidTransform(const float T[16])
{
  float x[16];
  float s[3];
  float absx;
  float b_s;
  float ex;
  int exponent;
  int i;
  int idx;
  int k;
  bool exitg1;
  bool isodd;
  bool tf;
  isodd = true;
  for (k = 0; k < 9; k++) {
    if (isodd) {
      absx = T[k % 3 + (div_nzp_s32_floor(k, 3) << 2)];
      if (std::isinf(absx) || std::isnan(absx)) {
        isodd = false;
      }
    } else {
      isodd = false;
    }
  }
  if (isodd) {
    float b_T[9];
    for (i = 0; i < 3; i++) {
      idx = i << 2;
      b_T[3 * i] = T[idx];
      b_T[3 * i + 1] = T[idx + 1];
      b_T[3 * i + 2] = T[idx + 2];
    }
    ::coder::internal::b_svd(b_T, s);
  } else {
    s[0] = rtNaNF;
    s[1] = rtNaNF;
    s[2] = rtNaNF;
  }
  isodd = std::isnan(s[0]);
  if (!isodd) {
    idx = 1;
  } else {
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k <= 3)) {
      if (!std::isnan(s[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }
  if (idx == 0) {
    b_s = s[0];
  } else {
    b_s = s[idx - 1];
    i = idx + 1;
    for (k = i; k < 4; k++) {
      absx = s[k - 1];
      if (b_s < absx) {
        b_s = absx;
      }
    }
  }
  if (!isodd) {
    idx = 1;
  } else {
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k <= 3)) {
      if (!std::isnan(s[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }
  if (idx == 0) {
    ex = s[0];
  } else {
    ex = s[idx - 1];
    i = idx + 1;
    for (k = i; k < 4; k++) {
      absx = s[k - 1];
      if (ex > absx) {
        ex = absx;
      }
    }
  }
  absx = std::abs(b_s);
  if ((!std::isinf(absx)) && (!std::isnan(absx))) {
    if (absx <= 1.17549435E-38F) {
      absx = 1.4013E-45F;
    } else {
      std::frexp(absx, &exponent);
      absx = std::ldexp(1.0F, exponent - 24);
    }
  } else {
    absx = rtNaNF;
  }
  if (b_s - ex < 100.0F * absx) {
    signed char ipiv[4];
    std::copy(&T[0], &T[16], &x[0]);
    ipiv[0] = 1;
    ipiv[1] = 2;
    ipiv[2] = 3;
    for (int j{0}; j < 3; j++) {
      int b_tmp;
      int jp1j;
      int mmj_tmp;
      mmj_tmp = 2 - j;
      b_tmp = j * 5;
      jp1j = b_tmp + 2;
      idx = 4 - j;
      exponent = 0;
      absx = std::abs(x[b_tmp]);
      for (k = 2; k <= idx; k++) {
        b_s = std::abs(x[(b_tmp + k) - 1]);
        if (b_s > absx) {
          exponent = k - 1;
          absx = b_s;
        }
      }
      if (x[b_tmp + exponent] != 0.0F) {
        if (exponent != 0) {
          idx = j + exponent;
          ipiv[j] = static_cast<signed char>(idx + 1);
          absx = x[j];
          x[j] = x[idx];
          x[idx] = absx;
          absx = x[j + 4];
          x[j + 4] = x[idx + 4];
          x[idx + 4] = absx;
          absx = x[j + 8];
          x[j + 8] = x[idx + 8];
          x[idx + 8] = absx;
          absx = x[j + 12];
          x[j + 12] = x[idx + 12];
          x[idx + 12] = absx;
        }
        i = (b_tmp - j) + 4;
        for (idx = jp1j; idx <= i; idx++) {
          x[idx - 1] /= x[b_tmp];
        }
      }
      idx = b_tmp;
      for (exponent = 0; exponent <= mmj_tmp; exponent++) {
        absx = x[(b_tmp + (exponent << 2)) + 4];
        if (absx != 0.0F) {
          i = idx + 6;
          k = (idx - j) + 8;
          for (jp1j = i; jp1j <= k; jp1j++) {
            x[jp1j - 1] += x[((b_tmp + jp1j) - idx) - 5] * -absx;
          }
        }
        idx += 4;
      }
    }
    isodd = (ipiv[0] > 1);
    if (ipiv[1] > 2) {
      isodd = !isodd;
    }
    absx = x[0] * x[5] * x[10] * x[15];
    if (ipiv[2] > 3) {
      isodd = !isodd;
    }
    if (isodd) {
      absx = -absx;
    }
    if (std::abs(absx - 1.0F) < 1.1920929E-5F) {
      tf = true;
    } else {
      tf = false;
    }
  } else {
    tf = false;
  }
  return tf;
}

} // namespace internal
} // namespace vision
} // namespace coder

//
// File trailer for isRigidTransform.cpp
//
// [EOF]
//
