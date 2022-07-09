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
#include <functional>
#include <string.h>

// Variable Definitions
static const char cv[41]{'/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p', 'p', 'i',
                         'n', 'g', '/', 'p', 'r', 'e', 'p', 'r', 'o', 'c', 'e',
                         's', 's', 'i', 'n', 'g', '/', 'm', 'a', 'x', '_', 'i',
                         'n', 't', 'e', 'n', 's', 'i', 't', 'y'};

static const char cv1[37]{'/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p', 'p',
                          'i', 'n', 'g', '/', 'p', 'r', 'e', 'p', 'r', 'o',
                          'c', 'e', 's', 's', 'i', 'n', 'g', '/', 'm', 'i',
                          'n', '_', 'r', 'a', 'n', 'g', 'e'};

static const char cv2[37]{'/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p', 'p',
                          'i', 'n', 'g', '/', 'p', 'r', 'e', 'p', 'r', 'o',
                          'c', 'e', 's', 's', 'i', 'n', 'g', '/', 'm', 'a',
                          'x', '_', 'r', 'a', 'n', 'g', 'e'};

// Function Definitions
namespace coder {
namespace ros {
void ParameterTree::b_canonicalizeName(ParameterTree *obj,
                                       ::coder::array<char, 2U> &validName)
{
  array<char, 2U> r;
  char name[41];
  for (int i{0}; i < 41; i++) {
    name[i] = cv[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&obj->ParameterHelper,
                                                    &name[0]);
  validName.set_size(1, 41);
  for (int i{0}; i < 41; i++) {
    validName[i] = cv[i];
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

double ParameterTree::b_get()
{
  array<char, 2U> in;
  array<char, 2U> parameterName;
  double data;
  int loop_ub;
  char name[37];
  for (int i{0}; i < 37; i++) {
    name[i] = cv1[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&ParameterHelper, &name[0]);
  in.set_size(1, 37);
  for (int i{0}; i < 37; i++) {
    in[i] = cv1[i];
  }
  if (startsWith(in)) {
    parameterName.set_size(1, 36);
    parameterName[0] = '~';
    for (int i{0}; i < 35; i++) {
      parameterName[i + 1] = in[i + 2];
    }
    in.set_size(1, 36);
    for (int i{0}; i < 36; i++) {
      in[i] = parameterName[i];
    }
  }
  parameterName.set_size(1, in.size(1) + 1);
  loop_ub = in.size(1);
  for (int i{0}; i < loop_ub; i++) {
    parameterName[i] = in[i];
  }
  parameterName[in.size(1)] = '\x00';
  data = 0.0;
  std::mem_fn (&MATLABROSParameter::getParameter<double>)(
      &ParameterHelper, &parameterName[0], &data);
  return data;
}

void ParameterTree::c_canonicalizeName(ParameterTree *obj,
                                       ::coder::array<char, 2U> &validName)
{
  array<char, 2U> r;
  char name[37];
  for (int i{0}; i < 37; i++) {
    name[i] = cv1[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&obj->ParameterHelper,
                                                    &name[0]);
  validName.set_size(1, 37);
  for (int i{0}; i < 37; i++) {
    validName[i] = cv1[i];
  }
  if (startsWith(validName)) {
    r.set_size(1, 36);
    r[0] = '~';
    for (int i{0}; i < 35; i++) {
      r[i + 1] = validName[i + 2];
    }
    validName.set_size(1, 36);
    for (int i{0}; i < 36; i++) {
      validName[i] = r[i];
    }
  }
}

double ParameterTree::c_get()
{
  array<char, 2U> in;
  array<char, 2U> parameterName;
  double data;
  int loop_ub;
  char name[37];
  for (int i{0}; i < 37; i++) {
    name[i] = cv2[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&ParameterHelper, &name[0]);
  in.set_size(1, 37);
  for (int i{0}; i < 37; i++) {
    in[i] = cv2[i];
  }
  if (startsWith(in)) {
    parameterName.set_size(1, 36);
    parameterName[0] = '~';
    for (int i{0}; i < 35; i++) {
      parameterName[i + 1] = in[i + 2];
    }
    in.set_size(1, 36);
    for (int i{0}; i < 36; i++) {
      in[i] = parameterName[i];
    }
  }
  parameterName.set_size(1, in.size(1) + 1);
  loop_ub = in.size(1);
  for (int i{0}; i < loop_ub; i++) {
    parameterName[i] = in[i];
  }
  parameterName[in.size(1)] = '\x00';
  data = 0.0;
  std::mem_fn (&MATLABROSParameter::getParameter<double>)(
      &ParameterHelper, &parameterName[0], &data);
  return data;
}

void ParameterTree::canonicalizeName(ParameterTree *obj,
                                     ::coder::array<char, 2U> &validName)
{
  static const char b_name[41]{
      '/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p', 'p', 'i', 'n', 'g', '/',
      'p', 'r', 'e', 'p', 'r', 'o', 'c', 'e', 's', 's', 'i', 'n', 'g', '/',
      'm', 'i', 'n', '_', 'i', 'n', 't', 'e', 'n', 's', 'i', 't', 'y'};
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

void ParameterTree::d_canonicalizeName(ParameterTree *obj,
                                       ::coder::array<char, 2U> &validName)
{
  array<char, 2U> r;
  char name[37];
  for (int i{0}; i < 37; i++) {
    name[i] = cv2[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&obj->ParameterHelper,
                                                    &name[0]);
  validName.set_size(1, 37);
  for (int i{0}; i < 37; i++) {
    validName[i] = cv2[i];
  }
  if (startsWith(validName)) {
    r.set_size(1, 36);
    r[0] = '~';
    for (int i{0}; i < 35; i++) {
      r[i + 1] = validName[i + 2];
    }
    validName.set_size(1, 36);
    for (int i{0}; i < 36; i++) {
      validName[i] = r[i];
    }
  }
}

double ParameterTree::get()
{
  array<char, 2U> in;
  array<char, 2U> parameterName;
  double data;
  int loop_ub;
  char name[41];
  for (int i{0}; i < 41; i++) {
    name[i] = cv[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&ParameterHelper, &name[0]);
  in.set_size(1, 41);
  for (int i{0}; i < 41; i++) {
    in[i] = cv[i];
  }
  if (startsWith(in)) {
    parameterName.set_size(1, 40);
    parameterName[0] = '~';
    for (int i{0}; i < 39; i++) {
      parameterName[i + 1] = in[i + 2];
    }
    in.set_size(1, 40);
    for (int i{0}; i < 40; i++) {
      in[i] = parameterName[i];
    }
  }
  parameterName.set_size(1, in.size(1) + 1);
  loop_ub = in.size(1);
  for (int i{0}; i < loop_ub; i++) {
    parameterName[i] = in[i];
  }
  parameterName[in.size(1)] = '\x00';
  data = 0.0;
  std::mem_fn (&MATLABROSParameter::getParameter<double>)(
      &ParameterHelper, &parameterName[0], &data);
  return data;
}

} // namespace ros
} // namespace coder

// End of code generation (ParameterTree.cpp)
