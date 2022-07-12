//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// svd.h
//
// Code generation for function 'svd'
//

#ifndef SVD_H
#define SVD_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
void svd(const double A[9], double U[9], double S[9], double V[9]);

void svd(const double A[9], double U[3]);

} // namespace coder

#endif
// End of code generation (svd.h)
