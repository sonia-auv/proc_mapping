//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// ref.cpp
//
// Code generation for function 'ref'
//

// Include files
#include "ref.h"
#include "rt_nonfinite.h"
#include <string.h>

// Function Definitions
namespace coder {
void f_captured_var::matlabCodegenDestructor()
{
  if (!matlabCodegenIsDeleted) {
    matlabCodegenIsDeleted = true;
  }
}

f_captured_var::f_captured_var()
{
  matlabCodegenIsDeleted = true;
}

f_captured_var::~f_captured_var()
{
  matlabCodegenDestructor();
}

} // namespace coder

// End of code generation (ref.cpp)
