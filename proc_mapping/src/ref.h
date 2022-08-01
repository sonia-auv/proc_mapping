//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ref.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 01-Aug-2022 08:26:09
//

#ifndef REF_H
#define REF_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace coder {
class b_pointCloud;

}

// Type Definitions
namespace coder {
class captured_var {
public:
  void matlabCodegenDestructor();
  ~captured_var();
  captured_var();
  bool matlabCodegenIsDeleted;
  b_pointCloud *contents;
};

class b_captured_var {
public:
  double contents[189];
};

class c_captured_var {
public:
  double contents[63];
};

class d_captured_var {
public:
  double contents[147];
};

class e_captured_var {
public:
  array<float, 2U> contents;
};

class f_captured_var {
public:
  double contents;
};

} // namespace coder

#endif
//
// File trailer for ref.h
//
// [EOF]
//
