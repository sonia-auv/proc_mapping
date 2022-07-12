//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// rigid3d.cpp
//
// Code generation for function 'rigid3d'
//

// Include files
#include "rigid3d.h"
#include "minOrMax.h"
#include "rt_nonfinite.h"
#include "svd.h"
#include <cmath>
#include <math.h>
#include <string.h>

// Function Definitions
namespace coder {
bool rigid3d::isTransformationMatrixRigid(const double T[16])
{
  double b_T[9];
  double rot[9];
  double singularValues[3];
  double absx;
  double s;
  int exponent;
  int i;
  int jA;
  int jp1j;
  bool isRigid;
  for (i = 0; i < 3; i++) {
    jp1j = i << 2;
    absx = T[jp1j];
    rot[3 * i] = absx;
    b_T[3 * i] = absx;
    absx = T[jp1j + 1];
    jA = 3 * i + 1;
    rot[jA] = absx;
    b_T[jA] = absx;
    absx = T[jp1j + 2];
    jA = 3 * i + 2;
    rot[jA] = absx;
    b_T[jA] = absx;
  }
  svd(b_T, singularValues);
  s = internal::maximum(singularValues);
  absx = std::abs(s);
  if ((!std::isinf(absx)) && (!std::isnan(absx))) {
    if (absx <= 2.2250738585072014E-308) {
      absx = 4.94065645841247E-324;
    } else {
      frexp(absx, &exponent);
      absx = std::ldexp(1.0, exponent - 53);
    }
  } else {
    absx = rtNaN;
  }
  if (s - internal::minimum(singularValues) < 1000.0 * absx) {
    signed char ipiv[3];
    bool isodd;
    ipiv[0] = 1;
    ipiv[1] = 2;
    for (int j{0}; j < 2; j++) {
      int b_tmp;
      int mmj_tmp;
      mmj_tmp = 1 - j;
      b_tmp = j << 2;
      jp1j = b_tmp + 2;
      jA = 3 - j;
      exponent = 0;
      absx = std::abs(rot[b_tmp]);
      for (int k{2}; k <= jA; k++) {
        s = std::abs(rot[(b_tmp + k) - 1]);
        if (s > absx) {
          exponent = k - 1;
          absx = s;
        }
      }
      if (rot[b_tmp + exponent] != 0.0) {
        if (exponent != 0) {
          jA = j + exponent;
          ipiv[j] = static_cast<signed char>(jA + 1);
          absx = rot[j];
          rot[j] = rot[jA];
          rot[jA] = absx;
          absx = rot[j + 3];
          rot[j + 3] = rot[jA + 3];
          rot[jA + 3] = absx;
          absx = rot[j + 6];
          rot[j + 6] = rot[jA + 6];
          rot[jA + 6] = absx;
        }
        i = (b_tmp - j) + 3;
        for (jA = jp1j; jA <= i; jA++) {
          rot[jA - 1] /= rot[b_tmp];
        }
      }
      jA = b_tmp;
      for (exponent = 0; exponent <= mmj_tmp; exponent++) {
        absx = rot[(b_tmp + exponent * 3) + 3];
        if (absx != 0.0) {
          i = jA + 5;
          jp1j = (jA - j) + 6;
          for (int k{i}; k <= jp1j; k++) {
            rot[k - 1] += rot[((b_tmp + k) - jA) - 4] * -absx;
          }
        }
        jA += 3;
      }
    }
    isodd = (ipiv[0] > 1);
    absx = rot[0] * rot[4] * rot[8];
    if (ipiv[1] > 2) {
      isodd = !isodd;
    }
    if (isodd) {
      absx = -absx;
    }
    if (std::abs(absx - 1.0) < 2.2204460492503131E-13) {
      isRigid = true;
    } else {
      isRigid = false;
    }
  } else {
    isRigid = false;
  }
  return isRigid;
}

} // namespace coder

// End of code generation (rigid3d.cpp)