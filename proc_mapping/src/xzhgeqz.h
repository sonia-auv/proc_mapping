//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xzhgeqz.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 31-Jul-2022 13:03:34
//

#ifndef XZHGEQZ_H
#define XZHGEQZ_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
namespace reflapack {
void xzhgeqz(creal_T A[16], int ilo, int ihi, creal_T Z[16], int *info,
             creal_T alpha1[4], creal_T beta1[4]);

}
} // namespace internal
} // namespace coder

#endif
//
// File trailer for xzhgeqz.h
//
// [EOF]
//
