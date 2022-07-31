//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: proc_mapping_rtwutil.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 31-Jul-2022 13:03:34
//

#ifndef PROC_MAPPING_RTWUTIL_H
#define PROC_MAPPING_RTWUTIL_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
extern int div_nzp_s32(int numerator, int denominator);

extern int div_nzp_s32_floor(int numerator, int denominator);

extern unsigned int mul_u32_sat(unsigned int a, unsigned int b);

extern void mul_wide_u32(unsigned int in0, unsigned int in1,
                         unsigned int *ptrOutBitsHi,
                         unsigned int *ptrOutBitsLo);

extern double rt_atan2d_snf(double u0, double u1);

extern double rt_hypotd_snf(double u0, double u1);

extern double rt_powd_snf(double u0, double u1);

extern double rt_remd_snf(double u0, double u1);

#endif
//
// File trailer for proc_mapping_rtwutil.h
//
// [EOF]
//
