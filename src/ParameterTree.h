//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// ParameterTree.h
//
// Code generation for function 'ParameterTree'
//

#ifndef PARAMETERTREE_H
#define PARAMETERTREE_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include "mlroscpp_param.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
namespace coder {
namespace ros {
class ParameterTree {
public:
  static void canonicalizeName(ParameterTree *obj,
                               ::coder::array<char, 2U> &validName);
  MATLABROSParameter ParameterHelper;
};

} // namespace ros
} // namespace coder

#endif
// End of code generation (ParameterTree.h)
