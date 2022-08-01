//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: proc_mapping_rtwutil.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 01-Aug-2022 08:26:09
//

// Include Files
#include "proc_mapping_rtwutil.h"
#include "rt_nonfinite.h"
#include "rt_defines.h"
#include <cfloat>
#include <cmath>
#include <string.h>

// Function Definitions
//
// Arguments    : int numerator
//                int denominator
// Return Type  : int
//
int div_nzp_s32(int numerator, int denominator)
{
  unsigned int b_denominator;
  unsigned int b_numerator;
  int quotient;
  if (numerator < 0) {
    b_numerator = ~static_cast<unsigned int>(numerator) + 1U;
  } else {
    b_numerator = static_cast<unsigned int>(numerator);
  }
  if (denominator < 0) {
    b_denominator = ~static_cast<unsigned int>(denominator) + 1U;
  } else {
    b_denominator = static_cast<unsigned int>(denominator);
  }
  b_numerator /= b_denominator;
  if ((numerator < 0) != (denominator < 0)) {
    quotient = -static_cast<int>(b_numerator);
  } else {
    quotient = static_cast<int>(b_numerator);
  }
  return quotient;
}

//
// Arguments    : int numerator
//                int denominator
// Return Type  : int
//
int div_nzp_s32_floor(int numerator, int denominator)
{
  unsigned int absDenominator;
  unsigned int absNumerator;
  int quotient;
  unsigned int tempAbsQuotient;
  bool quotientNeedsNegation;
  if (numerator < 0) {
    absNumerator = ~static_cast<unsigned int>(numerator) + 1U;
  } else {
    absNumerator = static_cast<unsigned int>(numerator);
  }
  if (denominator < 0) {
    absDenominator = ~static_cast<unsigned int>(denominator) + 1U;
  } else {
    absDenominator = static_cast<unsigned int>(denominator);
  }
  quotientNeedsNegation = ((numerator < 0) != (denominator < 0));
  tempAbsQuotient = absNumerator / absDenominator;
  if (quotientNeedsNegation) {
    absNumerator %= absDenominator;
    if (absNumerator > 0U) {
      tempAbsQuotient++;
    }
    quotient = -static_cast<int>(tempAbsQuotient);
  } else {
    quotient = static_cast<int>(tempAbsQuotient);
  }
  return quotient;
}

//
// Arguments    : unsigned int a
//                unsigned int b
// Return Type  : unsigned int
//
unsigned int mul_u32_sat(unsigned int a, unsigned int b)
{
  unsigned int result;
  unsigned int u32_chi;
  mul_wide_u32(a, b, &u32_chi, &result);
  if (u32_chi) {
    result = MAX_uint32_T;
  }
  return result;
}

//
// Arguments    : unsigned int in0
//                unsigned int in1
//                unsigned int *ptrOutBitsHi
//                unsigned int *ptrOutBitsLo
// Return Type  : void
//
void mul_wide_u32(unsigned int in0, unsigned int in1,
                  unsigned int *ptrOutBitsHi, unsigned int *ptrOutBitsLo)
{
  int in0Hi;
  int in0Lo;
  int in1Hi;
  int in1Lo;
  unsigned int outBitsLo;
  unsigned int productHiLo;
  unsigned int productLoHi;
  unsigned int productLoLo;
  in0Hi = static_cast<int>(in0 >> 16U);
  in0Lo = static_cast<int>(in0 & 65535U);
  in1Hi = static_cast<int>(in1 >> 16U);
  in1Lo = static_cast<int>(in1 & 65535U);
  productHiLo = static_cast<unsigned int>(in0Hi) * in1Lo;
  productLoHi = static_cast<unsigned int>(in0Lo) * in1Hi;
  productLoLo = static_cast<unsigned int>(in0Lo) * in1Lo;
  in0Lo = 0;
  outBitsLo = productLoLo + (productLoHi << 16U);
  if (outBitsLo < productLoLo) {
    in0Lo = 1;
  }
  productLoLo = outBitsLo;
  outBitsLo += productHiLo << 16U;
  if (outBitsLo < productLoLo) {
    in0Lo = static_cast<int>(in0Lo + 1U);
  }
  *ptrOutBitsHi = ((in0Lo + static_cast<unsigned int>(in0Hi) * in1Hi) +
                   (productLoHi >> 16U)) +
                  (productHiLo >> 16U);
  *ptrOutBitsLo = outBitsLo;
}

//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
double rt_atan2d_snf(double u0, double u1)
{
  double y;
  if (std::isnan(u0) || std::isnan(u1)) {
    y = rtNaN;
  } else if (std::isinf(u0) && std::isinf(u1)) {
    int b_u0;
    int b_u1;
    if (u0 > 0.0) {
      b_u0 = 1;
    } else {
      b_u0 = -1;
    }
    if (u1 > 0.0) {
      b_u1 = 1;
    } else {
      b_u1 = -1;
    }
    y = std::atan2(static_cast<double>(b_u0), static_cast<double>(b_u1));
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = std::atan2(u0, u1);
  }
  return y;
}

//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
double rt_hypotd_snf(double u0, double u1)
{
  double a;
  double y;
  a = std::abs(u0);
  y = std::abs(u1);
  if (a < y) {
    a /= y;
    y *= std::sqrt(a * a + 1.0);
  } else if (a > y) {
    y /= a;
    y = a * std::sqrt(y * y + 1.0);
  } else if (!std::isnan(y)) {
    y = a * 1.4142135623730951;
  }
  return y;
}

//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
double rt_powd_snf(double u0, double u1)
{
  double y;
  if (std::isnan(u0) || std::isnan(u1)) {
    y = rtNaN;
  } else {
    double d;
    double d1;
    d = std::abs(u0);
    d1 = std::abs(u1);
    if (std::isinf(u1)) {
      if (d == 1.0) {
        y = 1.0;
      } else if (d > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d1 == 0.0) {
      y = 1.0;
    } else if (d1 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = std::sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > std::floor(u1))) {
      y = rtNaN;
    } else {
      y = std::pow(u0, u1);
    }
  }
  return y;
}

//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
double rt_remd_snf(double u0, double u1)
{
  double y;
  if (std::isnan(u0) || std::isnan(u1) || std::isinf(u0)) {
    y = rtNaN;
  } else if (std::isinf(u1)) {
    y = u0;
  } else if ((u1 != 0.0) && (u1 != std::trunc(u1))) {
    double q;
    q = std::abs(u0 / u1);
    if (!(std::abs(q - std::floor(q + 0.5)) > DBL_EPSILON * q)) {
      y = 0.0 * u0;
    } else {
      y = std::fmod(u0, u1);
    }
  } else {
    y = std::fmod(u0, u1);
  }
  return y;
}

//
// File trailer for proc_mapping_rtwutil.cpp
//
// [EOF]
//
