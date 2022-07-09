//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// ref.h
//
// Code generation for function 'ref'
//

#ifndef REF_H
#define REF_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace coder {
class f_pointCloud;

}

// Type Definitions
namespace coder {
class captured_var {
public:
  double contents;
};

class b_captured_var {
public:
  double contents[147];
};

class c_captured_var {
public:
  double contents[63];
};

class d_captured_var {
public:
  double contents[189];
};

class e_captured_var {
public:
  array<float, 2U> contents;
};

class f_captured_var {
public:
  void matlabCodegenDestructor();
  ~f_captured_var();
  f_captured_var();
  bool matlabCodegenIsDeleted;
  f_pointCloud *contents;
};

} // namespace coder

#endif
// End of code generation (ref.h)
