//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// ParameterTree.cpp
//
// Code generation for function 'ParameterTree'
//

// Include files
#include "ParameterTree.h"
#include "rt_nonfinite.h"
#include "startsWith.h"
#include "coder_array.h"
#include "mlroscpp_param.h"
#include <string.h>

// Function Definitions
namespace coder {
namespace ros {
void ParameterTree::canonicalizeName(ParameterTree *obj,
                                     ::coder::array<char, 2U> &validName)
{
  static const char b_name[41]{
      '/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p', 'p', 'i', 'n', 'g', '/',
      's', 'e', 'g', 'm', 'e', 'n', 't', 'a', 't', 'i', 'o', 'n', '/', 'b',
      'u', 'o', 'y', 's', '/', 'm', 'a', 'x', '_', 'a', 'r', 'e', 'a'};
  array<char, 2U> r;
  char name[41];
  for (int i{0}; i < 41; i++) {
    name[i] = b_name[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&obj->ParameterHelper,
                                                    &name[0]);
  validName.set_size(1, 41);
  for (int i{0}; i < 41; i++) {
    validName[i] = b_name[i];
  }
  if (startsWith(validName)) {
    r.set_size(1, 40);
    r[0] = '~';
    for (int i{0}; i < 39; i++) {
      r[i + 1] = validName[i + 2];
    }
    validName.set_size(1, 40);
    for (int i{0}; i < 40; i++) {
      validName[i] = r[i];
    }
  }
}

} // namespace ros
} // namespace coder

// End of code generation (ParameterTree.cpp)
