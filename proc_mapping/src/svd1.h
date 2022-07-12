//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// svd1.h
//
// Code generation for function 'svd1'
//

#ifndef SVD1_H
#define SVD1_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
void b_svd(const float A[9], float U[3]);

void b_svd(const float A[9], float U[9], float s[3], float V[9]);

void b_svd(const double A[9], double U[9], double s[3], double V[9]);

} // namespace internal
} // namespace coder

#endif
// End of code generation (svd1.h)
