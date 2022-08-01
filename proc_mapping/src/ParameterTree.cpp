//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ParameterTree.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 01-Aug-2022 08:26:09
//

// Include Files
#include "ParameterTree.h"
#include "rt_nonfinite.h"
#include "startsWith.h"
#include "coder_array.h"
#include "mlroscpp_param.h"
#include <functional>
#include <string.h>

// Variable Definitions
static const char cv[43]{'/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p', 'p', 'i',
                         'n', 'g', '/', 'f', 'i', 'l', 't', 'e', 'r', '/', 'h',
                         'y', 'd', 'r', 'o', '/', 'g', 'e', 'n', 'e', 'r', 'a',
                         'l', '/', 'b', 'o', 'x', '_', 's', 'i', 'z', 'e'};

static const char cv1[41]{'/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p', 'p', 'i',
                          'n', 'g', '/', 'f', 'i', 'l', 't', 'e', 'r', '/', 'h',
                          'y', 'd', 'r', 'o', '/', 'f', 'r', 'e', 'q', '_', 't',
                          'h', 'r', 'e', 's', 'h', 'o', 'l', 'd'};

static const char cv2[48]{'/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p', 'p',
                          'i', 'n', 'g', '/', 'f', 'i', 'l', 't', 'e', 'r',
                          '/', 's', 'o', 'n', 'a', 'r', '/', 'h', 'i', 's',
                          't', 'o', 'g', 'r', 'a', 'm', '_', 'f', 'i', 'l',
                          't', 'e', 'r', '/', 'n', 'B', 'i', 'n'};

static const char cv3[60]{
    '/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p', 'p', 'i', 'n', 'g', '/', 's',
    'o', 'n', 'a', 'r', '/', 'f', 'i', 'l', 't', 'e', 'r', '/', 'h', 'i', 's',
    't', 'o', 'g', 'r', 'a', 'm', '_', 'f', 'i', 'l', 't', 'e', 'r', '/', 'n',
    'B', 'i', 'n', 's', 'T', 'o', 'F', 'i', 'l', 't', 'e', 'r', 'O', 'u', 't'};

static const char cv4[45]{
    '/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p', 'p', 'i', 'n', 'g', '/', 's',
    'e', 'g', 'm', 'e', 'n', 't', 'a', 't', 'i', 'o', 'n', '/', 'b', 'u', 'o',
    'y', 's', '/', 'c', 'l', 'u', 's', 't', 'e', 'r', '_', 'd', 'i', 's', 't'};

static const char cv5[42]{'/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p', 'p', 'i',
                          'n', 'g', '/', 's', 'e', 'g', 'm', 'e', 'n', 't', 'a',
                          't', 'i', 'o', 'n', '/', 'b', 'u', 'o', 'y', 's', '/',
                          'p', 'l', 'a', 'n', 'e', '_', 't', 'o', 'l'};

static const char cv6[49]{'/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p', 'p',
                          'i', 'n', 'g', '/', 's', 'e', 'g', 'm', 'e', 'n',
                          't', 'a', 't', 'i', 'o', 'n', '/', 'b', 'u', 'o',
                          'y', 's', '/', 'i', 'c', 'p', '_', 'i', 'n', 'l',
                          'i', 'e', 'r', '_', 'r', 'a', 't', 'i', 'o'};

static const char cv7[47]{'/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p', 'p',
                          'i', 'n', 'g', '/', 's', 'e', 'g', 'm', 'e', 'n',
                          't', 'a', 't', 'i', 'o', 'n', '/', 'b', 'u', 'o',
                          'y', 's', '/', 'z', '_', 'n', 'o', 'r', 'm', 'a',
                          'l', '_', 't', 'h', 'r', 'e', 's'};

static const char cv8[47]{'/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p', 'p',
                          'i', 'n', 'g', '/', 's', 'e', 'g', 'm', 'e', 'n',
                          't', 'a', 't', 'i', 'o', 'n', '/', 'b', 'u', 'o',
                          'y', 's', '/', 'i', 'n', '_', 'p', 'l', 'a', 'n',
                          'e', '_', 't', 'h', 'r', 'e', 's'};

static const char cv9[41]{'/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p', 'p', 'i',
                          'n', 'g', '/', 's', 'e', 'g', 'm', 'e', 'n', 't', 'a',
                          't', 'i', 'o', 'n', '/', 'b', 'u', 'o', 'y', 's', '/',
                          'm', 'i', 'n', '_', 'a', 'r', 'e', 'a'};

static const char cv10[41]{
    '/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p', 'p', 'i', 'n', 'g', '/',
    's', 'e', 'g', 'm', 'e', 'n', 't', 'a', 't', 'i', 'o', 'n', '/', 'b',
    'u', 'o', 'y', 's', '/', 'm', 'a', 'x', '_', 'a', 'r', 'e', 'a'};

static const char cv11[36]{'/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p',
                           'p', 'i', 'n', 'g', '/', 's', 'e', 'g', 'm',
                           'e', 'n', 't', 'a', 't', 'i', 'o', 'n', '/',
                           'b', 'u', 'o', 'y', 's', '/', 'g', 'a', 'p'};

static const char cv12[46]{'/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p', 'p',
                           'i', 'n', 'g', '/', 's', 'e', 'g', 'm', 'e', 'n',
                           't', 'a', 't', 'i', 'o', 'n', '/', 't', 'a', 'b',
                           'l', 'e', 's', '/', 'c', 'l', 'u', 's', 't', 'e',
                           'r', '_', 'd', 'i', 's', 't'};

static const char cv13[46]{'/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p', 'p',
                           'i', 'n', 'g', '/', 's', 'e', 'g', 'm', 'e', 'n',
                           't', 'a', 't', 'i', 'o', 'n', '/', 't', 'a', 'b',
                           'l', 'e', 's', '/', 't', 'o', 'p', '_', 'a', 'r',
                           'e', 'a', '_', 'm', 'i', 'n'};

static const char cv14[46]{'/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p', 'p',
                           'i', 'n', 'g', '/', 's', 'e', 'g', 'm', 'e', 'n',
                           't', 'a', 't', 'i', 'o', 'n', '/', 't', 'a', 'b',
                           'l', 'e', 's', '/', 't', 'o', 'p', '_', 'a', 'r',
                           'e', 'a', '_', 'm', 'a', 'x'};

static const char cv15[44]{
    '/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p', 'p', 'i', 'n', 'g', '/', 's',
    'e', 'g', 'm', 'e', 'n', 't', 'a', 't', 'i', 'o', 'n', '/', 't', 'a', 'b',
    'l', 'e', 's', '/', 'p', 'o', 's', 'e', '_', 'd', 'e', 'p', 't', 'h'};

static const char cv16[50]{'/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p', 'p',
                           'i', 'n', 'g', '/', 's', 'e', 'g', 'm', 'e', 'n',
                           't', 'a', 't', 'i', 'o', 'n', '/', 't', 'a', 'b',
                           'l', 'e', 's', '/', 'm', 'a', 'x', '_', 'b', 'e',
                           't', 'w', 'e', 'e', 'n', '_', 'd', 'i', 's', 't'};

static const char cv17[51]{
    '/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p', 'p', 'i', 'n', 'g',
    '/', 's', 'e', 'g', 'm', 'e', 'n', 't', 'a', 't', 'i', 'o', 'n',
    '/', 't', 'a', 'b', 'l', 'e', 's', '/', 'm', 'a', 'x', '_', 'b',
    'e', 't', 'w', 'e', 'e', 'n', '_', 'a', 'n', 'g', 'l', 'e'};

static const char cv18[50]{'/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p', 'p',
                           'i', 'n', 'g', '/', 's', 'e', 'g', 'm', 'e', 'n',
                           't', 'a', 't', 'i', 'o', 'n', '/', 't', 'a', 'b',
                           'l', 'e', 's', '/', 's', 'q', 'u', 'a', 'r', 'e',
                           'n', 'e', 's', 's', '_', 'r', 'a', 't', 'i', 'o'};

static const char cv19[45]{
    '/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p', 'p', 'i', 'n', 'g', '/', 's',
    'e', 'g', 'm', 'e', 'n', 't', 'a', 't', 'i', 'o', 'n', '/', 'h', 'y', 'd',
    'r', 'o', '/', 'c', 'l', 'u', 's', 't', 'e', 'r', '_', 'd', 'i', 's', 't'};

static const char cv20[43]{
    '/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p', 'p', 'i', 'n', 'g', '/', 'p',
    'a', 'r', 'a', 'm', 'e', 't', 'e', 'r', 's', '/', 'h', 'y', 'd', 'r', 'o',
    '/', 'p', 'i', 'n', 'g', 'e', 'r', '_', 'd', 'e', 'p', 't', 'h'};

static const char cv21[44]{
    '/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p', 'p', 'i', 'n', 'g', '/', 'p',
    'a', 'r', 'a', 'm', 'e', 't', 'e', 'r', 's', '/', 'h', 'y', 'd', 'r', 'o',
    '/', 't', 'r', 'a', 'n', 's', 'l', 'a', 't', 'i', 'o', 'n', '/', 'x'};

static const char cv22[44]{
    '/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p', 'p', 'i', 'n', 'g', '/', 'p',
    'a', 'r', 'a', 'm', 'e', 't', 'e', 'r', 's', '/', 'h', 'y', 'd', 'r', 'o',
    '/', 't', 'r', 'a', 'n', 's', 'l', 'a', 't', 'i', 'o', 'n', '/', 'y'};

static const char cv23[44]{
    '/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p', 'p', 'i', 'n', 'g', '/', 'p',
    'a', 'r', 'a', 'm', 'e', 't', 'e', 'r', 's', '/', 'h', 'y', 'd', 'r', 'o',
    '/', 't', 'r', 'a', 'n', 's', 'l', 'a', 't', 'i', 'o', 'n', '/', 'z'};

static const char cv24[44]{
    '/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p', 'p', 'i', 'n', 'g', '/', 'p',
    'a', 'r', 'a', 'm', 'e', 't', 'e', 'r', 's', '/', 's', 'o', 'n', 'a', 'r',
    '/', 't', 'r', 'a', 'n', 's', 'l', 'a', 't', 'i', 'o', 'n', '/', 'x'};

static const char cv25[44]{
    '/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p', 'p', 'i', 'n', 'g', '/', 'p',
    'a', 'r', 'a', 'm', 'e', 't', 'e', 'r', 's', '/', 's', 'o', 'n', 'a', 'r',
    '/', 't', 'r', 'a', 'n', 's', 'l', 'a', 't', 'i', 'o', 'n', '/', 'y'};

static const char cv26[44]{
    '/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p', 'p', 'i', 'n', 'g', '/', 'p',
    'a', 'r', 'a', 'm', 'e', 't', 'e', 'r', 's', '/', 's', 'o', 'n', 'a', 'r',
    '/', 't', 'r', 'a', 'n', 's', 'l', 'a', 't', 'i', 'o', 'n', '/', 'z'};

// Function Definitions
//
// Arguments    : ParameterTree *obj
//                ::coder::array<char, 2U> &validName
// Return Type  : void
//
namespace coder {
namespace ros {
void ParameterTree::ab_canonicalizeName(ParameterTree *obj,
                                        ::coder::array<char, 2U> &validName)
{
  array<char, 2U> r;
  char name[43];
  for (int i{0}; i < 43; i++) {
    name[i] = cv20[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&obj->ParameterHelper,
                                                    &name[0]);
  validName.set_size(1, 43);
  for (int i{0}; i < 43; i++) {
    validName[i] = cv20[i];
  }
  if (startsWith(validName)) {
    r.set_size(1, 42);
    r[0] = '~';
    for (int i{0}; i < 41; i++) {
      r[i + 1] = validName[i + 2];
    }
    validName.set_size(1, 42);
    for (int i{0}; i < 42; i++) {
      validName[i] = r[i];
    }
  }
}

//
// Arguments    : void
// Return Type  : double
//
double ParameterTree::ab_get()
{
  array<char, 2U> in;
  array<char, 2U> parameterName;
  double data;
  int loop_ub;
  char name[44];
  for (int i{0}; i < 44; i++) {
    name[i] = cv25[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&ParameterHelper, &name[0]);
  in.set_size(1, 44);
  for (int i{0}; i < 44; i++) {
    in[i] = cv25[i];
  }
  if (startsWith(in)) {
    parameterName.set_size(1, 43);
    parameterName[0] = '~';
    for (int i{0}; i < 42; i++) {
      parameterName[i + 1] = in[i + 2];
    }
    in.set_size(1, 43);
    for (int i{0}; i < 43; i++) {
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

//
// Arguments    : ParameterTree *obj
//                ::coder::array<char, 2U> &validName
// Return Type  : void
//
void ParameterTree::b_canonicalizeName(ParameterTree *obj,
                                       ::coder::array<char, 2U> &validName)
{
  static const char b_name[41]{
      '/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p', 'p', 'i', 'n', 'g', '/',
      'p', 'r', 'e', 'p', 'r', 'o', 'c', 'e', 's', 's', 'i', 'n', 'g', '/',
      'm', 'a', 'x', '_', 'i', 'n', 't', 'e', 'n', 's', 'i', 't', 'y'};
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

//
// Arguments    : void
// Return Type  : double
//
double ParameterTree::b_get()
{
  array<char, 2U> in;
  array<char, 2U> parameterName;
  double data;
  int loop_ub;
  char name[41];
  for (int i{0}; i < 41; i++) {
    name[i] = cv1[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&ParameterHelper, &name[0]);
  in.set_size(1, 41);
  for (int i{0}; i < 41; i++) {
    in[i] = cv1[i];
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

//
// Arguments    : ParameterTree *obj
//                ::coder::array<char, 2U> &validName
// Return Type  : void
//
void ParameterTree::bb_canonicalizeName(ParameterTree *obj,
                                        ::coder::array<char, 2U> &validName)
{
  array<char, 2U> r;
  char name[44];
  for (int i{0}; i < 44; i++) {
    name[i] = cv21[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&obj->ParameterHelper,
                                                    &name[0]);
  validName.set_size(1, 44);
  for (int i{0}; i < 44; i++) {
    validName[i] = cv21[i];
  }
  if (startsWith(validName)) {
    r.set_size(1, 43);
    r[0] = '~';
    for (int i{0}; i < 42; i++) {
      r[i + 1] = validName[i + 2];
    }
    validName.set_size(1, 43);
    for (int i{0}; i < 43; i++) {
      validName[i] = r[i];
    }
  }
}

//
// Arguments    : void
// Return Type  : double
//
double ParameterTree::bb_get()
{
  array<char, 2U> in;
  array<char, 2U> parameterName;
  double data;
  int loop_ub;
  char name[44];
  for (int i{0}; i < 44; i++) {
    name[i] = cv26[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&ParameterHelper, &name[0]);
  in.set_size(1, 44);
  for (int i{0}; i < 44; i++) {
    in[i] = cv26[i];
  }
  if (startsWith(in)) {
    parameterName.set_size(1, 43);
    parameterName[0] = '~';
    for (int i{0}; i < 42; i++) {
      parameterName[i + 1] = in[i + 2];
    }
    in.set_size(1, 43);
    for (int i{0}; i < 43; i++) {
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

//
// Arguments    : ParameterTree *obj
//                ::coder::array<char, 2U> &validName
// Return Type  : void
//
void ParameterTree::c_canonicalizeName(ParameterTree *obj,
                                       ::coder::array<char, 2U> &validName)
{
  static const char b_name[37]{'/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p', 'p',
                               'i', 'n', 'g', '/', 'p', 'r', 'e', 'p', 'r', 'o',
                               'c', 'e', 's', 's', 'i', 'n', 'g', '/', 'm', 'i',
                               'n', '_', 'r', 'a', 'n', 'g', 'e'};
  array<char, 2U> r;
  char name[37];
  for (int i{0}; i < 37; i++) {
    name[i] = b_name[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&obj->ParameterHelper,
                                                    &name[0]);
  validName.set_size(1, 37);
  for (int i{0}; i < 37; i++) {
    validName[i] = b_name[i];
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

//
// Arguments    : void
// Return Type  : double
//
double ParameterTree::c_get()
{
  array<char, 2U> in;
  array<char, 2U> parameterName;
  double data;
  int loop_ub;
  char name[48];
  for (int i{0}; i < 48; i++) {
    name[i] = cv2[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&ParameterHelper, &name[0]);
  in.set_size(1, 48);
  for (int i{0}; i < 48; i++) {
    in[i] = cv2[i];
  }
  if (startsWith(in)) {
    parameterName.set_size(1, 47);
    parameterName[0] = '~';
    for (int i{0}; i < 46; i++) {
      parameterName[i + 1] = in[i + 2];
    }
    in.set_size(1, 47);
    for (int i{0}; i < 47; i++) {
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

//
// Arguments    : ParameterTree *obj
//                ::coder::array<char, 2U> &validName
// Return Type  : void
//
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

//
// Arguments    : ParameterTree *obj
//                ::coder::array<char, 2U> &validName
// Return Type  : void
//
void ParameterTree::cb_canonicalizeName(ParameterTree *obj,
                                        ::coder::array<char, 2U> &validName)
{
  array<char, 2U> r;
  char name[44];
  for (int i{0}; i < 44; i++) {
    name[i] = cv22[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&obj->ParameterHelper,
                                                    &name[0]);
  validName.set_size(1, 44);
  for (int i{0}; i < 44; i++) {
    validName[i] = cv22[i];
  }
  if (startsWith(validName)) {
    r.set_size(1, 43);
    r[0] = '~';
    for (int i{0}; i < 42; i++) {
      r[i + 1] = validName[i + 2];
    }
    validName.set_size(1, 43);
    for (int i{0}; i < 43; i++) {
      validName[i] = r[i];
    }
  }
}

//
// Arguments    : ParameterTree *obj
//                ::coder::array<char, 2U> &validName
// Return Type  : void
//
void ParameterTree::d_canonicalizeName(ParameterTree *obj,
                                       ::coder::array<char, 2U> &validName)
{
  static const char b_name[37]{'/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p', 'p',
                               'i', 'n', 'g', '/', 'p', 'r', 'e', 'p', 'r', 'o',
                               'c', 'e', 's', 's', 'i', 'n', 'g', '/', 'm', 'a',
                               'x', '_', 'r', 'a', 'n', 'g', 'e'};
  array<char, 2U> r;
  char name[37];
  for (int i{0}; i < 37; i++) {
    name[i] = b_name[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&obj->ParameterHelper,
                                                    &name[0]);
  validName.set_size(1, 37);
  for (int i{0}; i < 37; i++) {
    validName[i] = b_name[i];
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

//
// Arguments    : void
// Return Type  : double
//
double ParameterTree::d_get()
{
  array<char, 2U> in;
  array<char, 2U> parameterName;
  double data;
  int loop_ub;
  char name[60];
  for (int i{0}; i < 60; i++) {
    name[i] = cv3[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&ParameterHelper, &name[0]);
  in.set_size(1, 60);
  for (int i{0}; i < 60; i++) {
    in[i] = cv3[i];
  }
  if (startsWith(in)) {
    parameterName.set_size(1, 59);
    parameterName[0] = '~';
    for (int i{0}; i < 58; i++) {
      parameterName[i + 1] = in[i + 2];
    }
    in.set_size(1, 59);
    for (int i{0}; i < 59; i++) {
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

//
// Arguments    : ParameterTree *obj
//                ::coder::array<char, 2U> &validName
// Return Type  : void
//
void ParameterTree::db_canonicalizeName(ParameterTree *obj,
                                        ::coder::array<char, 2U> &validName)
{
  array<char, 2U> r;
  char name[44];
  for (int i{0}; i < 44; i++) {
    name[i] = cv23[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&obj->ParameterHelper,
                                                    &name[0]);
  validName.set_size(1, 44);
  for (int i{0}; i < 44; i++) {
    validName[i] = cv23[i];
  }
  if (startsWith(validName)) {
    r.set_size(1, 43);
    r[0] = '~';
    for (int i{0}; i < 42; i++) {
      r[i + 1] = validName[i + 2];
    }
    validName.set_size(1, 43);
    for (int i{0}; i < 43; i++) {
      validName[i] = r[i];
    }
  }
}

//
// Arguments    : ParameterTree *obj
//                ::coder::array<char, 2U> &validName
// Return Type  : void
//
void ParameterTree::e_canonicalizeName(ParameterTree *obj,
                                       ::coder::array<char, 2U> &validName)
{
  static const char b_name[43]{
      '/', 'p', 'r', 'o', 'c', '_', 'm', 'a', 'p', 'p', 'i', 'n', 'g', '/', 'f',
      'i', 'l', 't', 'e', 'r', '/', 's', 'o', 'n', 'a', 'r', '/', 'g', 'e', 'n',
      'e', 'r', 'a', 'l', '/', 'b', 'o', 'x', '_', 's', 'i', 'z', 'e'};
  array<char, 2U> r;
  char name[43];
  for (int i{0}; i < 43; i++) {
    name[i] = b_name[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&obj->ParameterHelper,
                                                    &name[0]);
  validName.set_size(1, 43);
  for (int i{0}; i < 43; i++) {
    validName[i] = b_name[i];
  }
  if (startsWith(validName)) {
    r.set_size(1, 42);
    r[0] = '~';
    for (int i{0}; i < 41; i++) {
      r[i + 1] = validName[i + 2];
    }
    validName.set_size(1, 42);
    for (int i{0}; i < 42; i++) {
      validName[i] = r[i];
    }
  }
}

//
// Arguments    : void
// Return Type  : double
//
double ParameterTree::e_get()
{
  array<char, 2U> in;
  array<char, 2U> parameterName;
  double data;
  int loop_ub;
  char name[45];
  for (int i{0}; i < 45; i++) {
    name[i] = cv4[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&ParameterHelper, &name[0]);
  in.set_size(1, 45);
  for (int i{0}; i < 45; i++) {
    in[i] = cv4[i];
  }
  if (startsWith(in)) {
    parameterName.set_size(1, 44);
    parameterName[0] = '~';
    for (int i{0}; i < 43; i++) {
      parameterName[i + 1] = in[i + 2];
    }
    in.set_size(1, 44);
    for (int i{0}; i < 44; i++) {
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

//
// Arguments    : ParameterTree *obj
//                ::coder::array<char, 2U> &validName
// Return Type  : void
//
void ParameterTree::eb_canonicalizeName(ParameterTree *obj,
                                        ::coder::array<char, 2U> &validName)
{
  array<char, 2U> r;
  char name[44];
  for (int i{0}; i < 44; i++) {
    name[i] = cv24[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&obj->ParameterHelper,
                                                    &name[0]);
  validName.set_size(1, 44);
  for (int i{0}; i < 44; i++) {
    validName[i] = cv24[i];
  }
  if (startsWith(validName)) {
    r.set_size(1, 43);
    r[0] = '~';
    for (int i{0}; i < 42; i++) {
      r[i + 1] = validName[i + 2];
    }
    validName.set_size(1, 43);
    for (int i{0}; i < 43; i++) {
      validName[i] = r[i];
    }
  }
}

//
// Arguments    : ParameterTree *obj
//                ::coder::array<char, 2U> &validName
// Return Type  : void
//
void ParameterTree::f_canonicalizeName(ParameterTree *obj,
                                       ::coder::array<char, 2U> &validName)
{
  array<char, 2U> r;
  char name[43];
  for (int i{0}; i < 43; i++) {
    name[i] = cv[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&obj->ParameterHelper,
                                                    &name[0]);
  validName.set_size(1, 43);
  for (int i{0}; i < 43; i++) {
    validName[i] = cv[i];
  }
  if (startsWith(validName)) {
    r.set_size(1, 42);
    r[0] = '~';
    for (int i{0}; i < 41; i++) {
      r[i + 1] = validName[i + 2];
    }
    validName.set_size(1, 42);
    for (int i{0}; i < 42; i++) {
      validName[i] = r[i];
    }
  }
}

//
// Arguments    : void
// Return Type  : double
//
double ParameterTree::f_get()
{
  array<char, 2U> in;
  array<char, 2U> parameterName;
  double data;
  int loop_ub;
  char name[42];
  for (int i{0}; i < 42; i++) {
    name[i] = cv5[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&ParameterHelper, &name[0]);
  in.set_size(1, 42);
  for (int i{0}; i < 42; i++) {
    in[i] = cv5[i];
  }
  if (startsWith(in)) {
    parameterName.set_size(1, 41);
    parameterName[0] = '~';
    for (int i{0}; i < 40; i++) {
      parameterName[i + 1] = in[i + 2];
    }
    in.set_size(1, 41);
    for (int i{0}; i < 41; i++) {
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

//
// Arguments    : ParameterTree *obj
//                ::coder::array<char, 2U> &validName
// Return Type  : void
//
void ParameterTree::fb_canonicalizeName(ParameterTree *obj,
                                        ::coder::array<char, 2U> &validName)
{
  array<char, 2U> r;
  char name[44];
  for (int i{0}; i < 44; i++) {
    name[i] = cv25[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&obj->ParameterHelper,
                                                    &name[0]);
  validName.set_size(1, 44);
  for (int i{0}; i < 44; i++) {
    validName[i] = cv25[i];
  }
  if (startsWith(validName)) {
    r.set_size(1, 43);
    r[0] = '~';
    for (int i{0}; i < 42; i++) {
      r[i + 1] = validName[i + 2];
    }
    validName.set_size(1, 43);
    for (int i{0}; i < 43; i++) {
      validName[i] = r[i];
    }
  }
}

//
// Arguments    : ParameterTree *obj
//                ::coder::array<char, 2U> &validName
// Return Type  : void
//
void ParameterTree::g_canonicalizeName(ParameterTree *obj,
                                       ::coder::array<char, 2U> &validName)
{
  array<char, 2U> r;
  char name[41];
  for (int i{0}; i < 41; i++) {
    name[i] = cv1[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&obj->ParameterHelper,
                                                    &name[0]);
  validName.set_size(1, 41);
  for (int i{0}; i < 41; i++) {
    validName[i] = cv1[i];
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

//
// Arguments    : void
// Return Type  : double
//
double ParameterTree::g_get()
{
  array<char, 2U> in;
  array<char, 2U> parameterName;
  double data;
  int loop_ub;
  char name[49];
  for (int i{0}; i < 49; i++) {
    name[i] = cv6[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&ParameterHelper, &name[0]);
  in.set_size(1, 49);
  for (int i{0}; i < 49; i++) {
    in[i] = cv6[i];
  }
  if (startsWith(in)) {
    parameterName.set_size(1, 48);
    parameterName[0] = '~';
    for (int i{0}; i < 47; i++) {
      parameterName[i + 1] = in[i + 2];
    }
    in.set_size(1, 48);
    for (int i{0}; i < 48; i++) {
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

//
// Arguments    : ParameterTree *obj
//                ::coder::array<char, 2U> &validName
// Return Type  : void
//
void ParameterTree::gb_canonicalizeName(ParameterTree *obj,
                                        ::coder::array<char, 2U> &validName)
{
  array<char, 2U> r;
  char name[44];
  for (int i{0}; i < 44; i++) {
    name[i] = cv26[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&obj->ParameterHelper,
                                                    &name[0]);
  validName.set_size(1, 44);
  for (int i{0}; i < 44; i++) {
    validName[i] = cv26[i];
  }
  if (startsWith(validName)) {
    r.set_size(1, 43);
    r[0] = '~';
    for (int i{0}; i < 42; i++) {
      r[i + 1] = validName[i + 2];
    }
    validName.set_size(1, 43);
    for (int i{0}; i < 43; i++) {
      validName[i] = r[i];
    }
  }
}

//
// Arguments    : void
// Return Type  : double
//
double ParameterTree::get()
{
  array<char, 2U> in;
  array<char, 2U> parameterName;
  double data;
  int loop_ub;
  char name[43];
  for (int i{0}; i < 43; i++) {
    name[i] = cv[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&ParameterHelper, &name[0]);
  in.set_size(1, 43);
  for (int i{0}; i < 43; i++) {
    in[i] = cv[i];
  }
  if (startsWith(in)) {
    parameterName.set_size(1, 42);
    parameterName[0] = '~';
    for (int i{0}; i < 41; i++) {
      parameterName[i + 1] = in[i + 2];
    }
    in.set_size(1, 42);
    for (int i{0}; i < 42; i++) {
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

//
// Arguments    : ParameterTree *obj
//                ::coder::array<char, 2U> &validName
// Return Type  : void
//
void ParameterTree::h_canonicalizeName(ParameterTree *obj,
                                       ::coder::array<char, 2U> &validName)
{
  array<char, 2U> r;
  char name[48];
  for (int i{0}; i < 48; i++) {
    name[i] = cv2[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&obj->ParameterHelper,
                                                    &name[0]);
  validName.set_size(1, 48);
  for (int i{0}; i < 48; i++) {
    validName[i] = cv2[i];
  }
  if (startsWith(validName)) {
    r.set_size(1, 47);
    r[0] = '~';
    for (int i{0}; i < 46; i++) {
      r[i + 1] = validName[i + 2];
    }
    validName.set_size(1, 47);
    for (int i{0}; i < 47; i++) {
      validName[i] = r[i];
    }
  }
}

//
// Arguments    : void
// Return Type  : double
//
double ParameterTree::h_get()
{
  array<char, 2U> in;
  array<char, 2U> parameterName;
  double data;
  int loop_ub;
  char name[47];
  for (int i{0}; i < 47; i++) {
    name[i] = cv7[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&ParameterHelper, &name[0]);
  in.set_size(1, 47);
  for (int i{0}; i < 47; i++) {
    in[i] = cv7[i];
  }
  if (startsWith(in)) {
    parameterName.set_size(1, 46);
    parameterName[0] = '~';
    for (int i{0}; i < 45; i++) {
      parameterName[i + 1] = in[i + 2];
    }
    in.set_size(1, 46);
    for (int i{0}; i < 46; i++) {
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

//
// Arguments    : ParameterTree *obj
//                ::coder::array<char, 2U> &validName
// Return Type  : void
//
void ParameterTree::i_canonicalizeName(ParameterTree *obj,
                                       ::coder::array<char, 2U> &validName)
{
  array<char, 2U> r;
  char name[60];
  for (int i{0}; i < 60; i++) {
    name[i] = cv3[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&obj->ParameterHelper,
                                                    &name[0]);
  validName.set_size(1, 60);
  for (int i{0}; i < 60; i++) {
    validName[i] = cv3[i];
  }
  if (startsWith(validName)) {
    r.set_size(1, 59);
    r[0] = '~';
    for (int i{0}; i < 58; i++) {
      r[i + 1] = validName[i + 2];
    }
    validName.set_size(1, 59);
    for (int i{0}; i < 59; i++) {
      validName[i] = r[i];
    }
  }
}

//
// Arguments    : void
// Return Type  : double
//
double ParameterTree::i_get()
{
  array<char, 2U> in;
  array<char, 2U> parameterName;
  double data;
  int loop_ub;
  char name[47];
  for (int i{0}; i < 47; i++) {
    name[i] = cv8[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&ParameterHelper, &name[0]);
  in.set_size(1, 47);
  for (int i{0}; i < 47; i++) {
    in[i] = cv8[i];
  }
  if (startsWith(in)) {
    parameterName.set_size(1, 46);
    parameterName[0] = '~';
    for (int i{0}; i < 45; i++) {
      parameterName[i + 1] = in[i + 2];
    }
    in.set_size(1, 46);
    for (int i{0}; i < 46; i++) {
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

//
// Arguments    : ParameterTree *obj
//                ::coder::array<char, 2U> &validName
// Return Type  : void
//
void ParameterTree::j_canonicalizeName(ParameterTree *obj,
                                       ::coder::array<char, 2U> &validName)
{
  array<char, 2U> r;
  char name[45];
  for (int i{0}; i < 45; i++) {
    name[i] = cv4[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&obj->ParameterHelper,
                                                    &name[0]);
  validName.set_size(1, 45);
  for (int i{0}; i < 45; i++) {
    validName[i] = cv4[i];
  }
  if (startsWith(validName)) {
    r.set_size(1, 44);
    r[0] = '~';
    for (int i{0}; i < 43; i++) {
      r[i + 1] = validName[i + 2];
    }
    validName.set_size(1, 44);
    for (int i{0}; i < 44; i++) {
      validName[i] = r[i];
    }
  }
}

//
// Arguments    : void
// Return Type  : double
//
double ParameterTree::j_get()
{
  array<char, 2U> in;
  array<char, 2U> parameterName;
  double data;
  int loop_ub;
  char name[41];
  for (int i{0}; i < 41; i++) {
    name[i] = cv9[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&ParameterHelper, &name[0]);
  in.set_size(1, 41);
  for (int i{0}; i < 41; i++) {
    in[i] = cv9[i];
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

//
// Arguments    : ParameterTree *obj
//                ::coder::array<char, 2U> &validName
// Return Type  : void
//
void ParameterTree::k_canonicalizeName(ParameterTree *obj,
                                       ::coder::array<char, 2U> &validName)
{
  array<char, 2U> r;
  char name[42];
  for (int i{0}; i < 42; i++) {
    name[i] = cv5[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&obj->ParameterHelper,
                                                    &name[0]);
  validName.set_size(1, 42);
  for (int i{0}; i < 42; i++) {
    validName[i] = cv5[i];
  }
  if (startsWith(validName)) {
    r.set_size(1, 41);
    r[0] = '~';
    for (int i{0}; i < 40; i++) {
      r[i + 1] = validName[i + 2];
    }
    validName.set_size(1, 41);
    for (int i{0}; i < 41; i++) {
      validName[i] = r[i];
    }
  }
}

//
// Arguments    : void
// Return Type  : double
//
double ParameterTree::k_get()
{
  array<char, 2U> in;
  array<char, 2U> parameterName;
  double data;
  int loop_ub;
  char name[41];
  for (int i{0}; i < 41; i++) {
    name[i] = cv10[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&ParameterHelper, &name[0]);
  in.set_size(1, 41);
  for (int i{0}; i < 41; i++) {
    in[i] = cv10[i];
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

//
// Arguments    : ParameterTree *obj
//                ::coder::array<char, 2U> &validName
// Return Type  : void
//
void ParameterTree::l_canonicalizeName(ParameterTree *obj,
                                       ::coder::array<char, 2U> &validName)
{
  array<char, 2U> r;
  char name[49];
  for (int i{0}; i < 49; i++) {
    name[i] = cv6[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&obj->ParameterHelper,
                                                    &name[0]);
  validName.set_size(1, 49);
  for (int i{0}; i < 49; i++) {
    validName[i] = cv6[i];
  }
  if (startsWith(validName)) {
    r.set_size(1, 48);
    r[0] = '~';
    for (int i{0}; i < 47; i++) {
      r[i + 1] = validName[i + 2];
    }
    validName.set_size(1, 48);
    for (int i{0}; i < 48; i++) {
      validName[i] = r[i];
    }
  }
}

//
// Arguments    : void
// Return Type  : double
//
double ParameterTree::l_get()
{
  array<char, 2U> in;
  array<char, 2U> parameterName;
  double data;
  int loop_ub;
  char name[36];
  for (int i{0}; i < 36; i++) {
    name[i] = cv11[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&ParameterHelper, &name[0]);
  in.set_size(1, 36);
  for (int i{0}; i < 36; i++) {
    in[i] = cv11[i];
  }
  if (startsWith(in)) {
    parameterName.set_size(1, 35);
    parameterName[0] = '~';
    for (int i{0}; i < 34; i++) {
      parameterName[i + 1] = in[i + 2];
    }
    in.set_size(1, 35);
    for (int i{0}; i < 35; i++) {
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

//
// Arguments    : ParameterTree *obj
//                ::coder::array<char, 2U> &validName
// Return Type  : void
//
void ParameterTree::m_canonicalizeName(ParameterTree *obj,
                                       ::coder::array<char, 2U> &validName)
{
  array<char, 2U> r;
  char name[47];
  for (int i{0}; i < 47; i++) {
    name[i] = cv7[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&obj->ParameterHelper,
                                                    &name[0]);
  validName.set_size(1, 47);
  for (int i{0}; i < 47; i++) {
    validName[i] = cv7[i];
  }
  if (startsWith(validName)) {
    r.set_size(1, 46);
    r[0] = '~';
    for (int i{0}; i < 45; i++) {
      r[i + 1] = validName[i + 2];
    }
    validName.set_size(1, 46);
    for (int i{0}; i < 46; i++) {
      validName[i] = r[i];
    }
  }
}

//
// Arguments    : void
// Return Type  : double
//
double ParameterTree::m_get()
{
  array<char, 2U> in;
  array<char, 2U> parameterName;
  double data;
  int loop_ub;
  char name[46];
  for (int i{0}; i < 46; i++) {
    name[i] = cv12[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&ParameterHelper, &name[0]);
  in.set_size(1, 46);
  for (int i{0}; i < 46; i++) {
    in[i] = cv12[i];
  }
  if (startsWith(in)) {
    parameterName.set_size(1, 45);
    parameterName[0] = '~';
    for (int i{0}; i < 44; i++) {
      parameterName[i + 1] = in[i + 2];
    }
    in.set_size(1, 45);
    for (int i{0}; i < 45; i++) {
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

//
// Arguments    : ParameterTree *obj
//                ::coder::array<char, 2U> &validName
// Return Type  : void
//
void ParameterTree::n_canonicalizeName(ParameterTree *obj,
                                       ::coder::array<char, 2U> &validName)
{
  array<char, 2U> r;
  char name[47];
  for (int i{0}; i < 47; i++) {
    name[i] = cv8[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&obj->ParameterHelper,
                                                    &name[0]);
  validName.set_size(1, 47);
  for (int i{0}; i < 47; i++) {
    validName[i] = cv8[i];
  }
  if (startsWith(validName)) {
    r.set_size(1, 46);
    r[0] = '~';
    for (int i{0}; i < 45; i++) {
      r[i + 1] = validName[i + 2];
    }
    validName.set_size(1, 46);
    for (int i{0}; i < 46; i++) {
      validName[i] = r[i];
    }
  }
}

//
// Arguments    : void
// Return Type  : double
//
double ParameterTree::n_get()
{
  array<char, 2U> in;
  array<char, 2U> parameterName;
  double data;
  int loop_ub;
  char name[46];
  for (int i{0}; i < 46; i++) {
    name[i] = cv13[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&ParameterHelper, &name[0]);
  in.set_size(1, 46);
  for (int i{0}; i < 46; i++) {
    in[i] = cv13[i];
  }
  if (startsWith(in)) {
    parameterName.set_size(1, 45);
    parameterName[0] = '~';
    for (int i{0}; i < 44; i++) {
      parameterName[i + 1] = in[i + 2];
    }
    in.set_size(1, 45);
    for (int i{0}; i < 45; i++) {
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

//
// Arguments    : ParameterTree *obj
//                ::coder::array<char, 2U> &validName
// Return Type  : void
//
void ParameterTree::o_canonicalizeName(ParameterTree *obj,
                                       ::coder::array<char, 2U> &validName)
{
  array<char, 2U> r;
  char name[41];
  for (int i{0}; i < 41; i++) {
    name[i] = cv9[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&obj->ParameterHelper,
                                                    &name[0]);
  validName.set_size(1, 41);
  for (int i{0}; i < 41; i++) {
    validName[i] = cv9[i];
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

//
// Arguments    : void
// Return Type  : double
//
double ParameterTree::o_get()
{
  array<char, 2U> in;
  array<char, 2U> parameterName;
  double data;
  int loop_ub;
  char name[46];
  for (int i{0}; i < 46; i++) {
    name[i] = cv14[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&ParameterHelper, &name[0]);
  in.set_size(1, 46);
  for (int i{0}; i < 46; i++) {
    in[i] = cv14[i];
  }
  if (startsWith(in)) {
    parameterName.set_size(1, 45);
    parameterName[0] = '~';
    for (int i{0}; i < 44; i++) {
      parameterName[i + 1] = in[i + 2];
    }
    in.set_size(1, 45);
    for (int i{0}; i < 45; i++) {
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

//
// Arguments    : ParameterTree *obj
//                ::coder::array<char, 2U> &validName
// Return Type  : void
//
void ParameterTree::p_canonicalizeName(ParameterTree *obj,
                                       ::coder::array<char, 2U> &validName)
{
  array<char, 2U> r;
  char name[41];
  for (int i{0}; i < 41; i++) {
    name[i] = cv10[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&obj->ParameterHelper,
                                                    &name[0]);
  validName.set_size(1, 41);
  for (int i{0}; i < 41; i++) {
    validName[i] = cv10[i];
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

//
// Arguments    : void
// Return Type  : double
//
double ParameterTree::p_get()
{
  array<char, 2U> in;
  array<char, 2U> parameterName;
  double data;
  int loop_ub;
  char name[44];
  for (int i{0}; i < 44; i++) {
    name[i] = cv15[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&ParameterHelper, &name[0]);
  in.set_size(1, 44);
  for (int i{0}; i < 44; i++) {
    in[i] = cv15[i];
  }
  if (startsWith(in)) {
    parameterName.set_size(1, 43);
    parameterName[0] = '~';
    for (int i{0}; i < 42; i++) {
      parameterName[i + 1] = in[i + 2];
    }
    in.set_size(1, 43);
    for (int i{0}; i < 43; i++) {
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

//
// Arguments    : ParameterTree *obj
//                ::coder::array<char, 2U> &validName
// Return Type  : void
//
void ParameterTree::q_canonicalizeName(ParameterTree *obj,
                                       ::coder::array<char, 2U> &validName)
{
  array<char, 2U> r;
  char name[36];
  for (int i{0}; i < 36; i++) {
    name[i] = cv11[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&obj->ParameterHelper,
                                                    &name[0]);
  validName.set_size(1, 36);
  for (int i{0}; i < 36; i++) {
    validName[i] = cv11[i];
  }
  if (startsWith(validName)) {
    r.set_size(1, 35);
    r[0] = '~';
    for (int i{0}; i < 34; i++) {
      r[i + 1] = validName[i + 2];
    }
    validName.set_size(1, 35);
    for (int i{0}; i < 35; i++) {
      validName[i] = r[i];
    }
  }
}

//
// Arguments    : void
// Return Type  : double
//
double ParameterTree::q_get()
{
  array<char, 2U> in;
  array<char, 2U> parameterName;
  double data;
  int loop_ub;
  char name[50];
  for (int i{0}; i < 50; i++) {
    name[i] = cv16[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&ParameterHelper, &name[0]);
  in.set_size(1, 50);
  for (int i{0}; i < 50; i++) {
    in[i] = cv16[i];
  }
  if (startsWith(in)) {
    parameterName.set_size(1, 49);
    parameterName[0] = '~';
    for (int i{0}; i < 48; i++) {
      parameterName[i + 1] = in[i + 2];
    }
    in.set_size(1, 49);
    for (int i{0}; i < 49; i++) {
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

//
// Arguments    : ParameterTree *obj
//                ::coder::array<char, 2U> &validName
// Return Type  : void
//
void ParameterTree::r_canonicalizeName(ParameterTree *obj,
                                       ::coder::array<char, 2U> &validName)
{
  array<char, 2U> r;
  char name[46];
  for (int i{0}; i < 46; i++) {
    name[i] = cv12[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&obj->ParameterHelper,
                                                    &name[0]);
  validName.set_size(1, 46);
  for (int i{0}; i < 46; i++) {
    validName[i] = cv12[i];
  }
  if (startsWith(validName)) {
    r.set_size(1, 45);
    r[0] = '~';
    for (int i{0}; i < 44; i++) {
      r[i + 1] = validName[i + 2];
    }
    validName.set_size(1, 45);
    for (int i{0}; i < 45; i++) {
      validName[i] = r[i];
    }
  }
}

//
// Arguments    : void
// Return Type  : double
//
double ParameterTree::r_get()
{
  array<char, 2U> in;
  array<char, 2U> parameterName;
  double data;
  int loop_ub;
  char name[51];
  for (int i{0}; i < 51; i++) {
    name[i] = cv17[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&ParameterHelper, &name[0]);
  in.set_size(1, 51);
  for (int i{0}; i < 51; i++) {
    in[i] = cv17[i];
  }
  if (startsWith(in)) {
    parameterName.set_size(1, 50);
    parameterName[0] = '~';
    for (int i{0}; i < 49; i++) {
      parameterName[i + 1] = in[i + 2];
    }
    in.set_size(1, 50);
    for (int i{0}; i < 50; i++) {
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

//
// Arguments    : ParameterTree *obj
//                ::coder::array<char, 2U> &validName
// Return Type  : void
//
void ParameterTree::s_canonicalizeName(ParameterTree *obj,
                                       ::coder::array<char, 2U> &validName)
{
  array<char, 2U> r;
  char name[46];
  for (int i{0}; i < 46; i++) {
    name[i] = cv13[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&obj->ParameterHelper,
                                                    &name[0]);
  validName.set_size(1, 46);
  for (int i{0}; i < 46; i++) {
    validName[i] = cv13[i];
  }
  if (startsWith(validName)) {
    r.set_size(1, 45);
    r[0] = '~';
    for (int i{0}; i < 44; i++) {
      r[i + 1] = validName[i + 2];
    }
    validName.set_size(1, 45);
    for (int i{0}; i < 45; i++) {
      validName[i] = r[i];
    }
  }
}

//
// Arguments    : void
// Return Type  : double
//
double ParameterTree::s_get()
{
  array<char, 2U> in;
  array<char, 2U> parameterName;
  double data;
  int loop_ub;
  char name[50];
  for (int i{0}; i < 50; i++) {
    name[i] = cv18[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&ParameterHelper, &name[0]);
  in.set_size(1, 50);
  for (int i{0}; i < 50; i++) {
    in[i] = cv18[i];
  }
  if (startsWith(in)) {
    parameterName.set_size(1, 49);
    parameterName[0] = '~';
    for (int i{0}; i < 48; i++) {
      parameterName[i + 1] = in[i + 2];
    }
    in.set_size(1, 49);
    for (int i{0}; i < 49; i++) {
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

//
// Arguments    : ParameterTree *obj
//                ::coder::array<char, 2U> &validName
// Return Type  : void
//
void ParameterTree::t_canonicalizeName(ParameterTree *obj,
                                       ::coder::array<char, 2U> &validName)
{
  array<char, 2U> r;
  char name[46];
  for (int i{0}; i < 46; i++) {
    name[i] = cv14[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&obj->ParameterHelper,
                                                    &name[0]);
  validName.set_size(1, 46);
  for (int i{0}; i < 46; i++) {
    validName[i] = cv14[i];
  }
  if (startsWith(validName)) {
    r.set_size(1, 45);
    r[0] = '~';
    for (int i{0}; i < 44; i++) {
      r[i + 1] = validName[i + 2];
    }
    validName.set_size(1, 45);
    for (int i{0}; i < 45; i++) {
      validName[i] = r[i];
    }
  }
}

//
// Arguments    : void
// Return Type  : double
//
double ParameterTree::t_get()
{
  array<char, 2U> in;
  array<char, 2U> parameterName;
  double data;
  int loop_ub;
  char name[45];
  for (int i{0}; i < 45; i++) {
    name[i] = cv19[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&ParameterHelper, &name[0]);
  in.set_size(1, 45);
  for (int i{0}; i < 45; i++) {
    in[i] = cv19[i];
  }
  if (startsWith(in)) {
    parameterName.set_size(1, 44);
    parameterName[0] = '~';
    for (int i{0}; i < 43; i++) {
      parameterName[i + 1] = in[i + 2];
    }
    in.set_size(1, 44);
    for (int i{0}; i < 44; i++) {
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

//
// Arguments    : ParameterTree *obj
//                ::coder::array<char, 2U> &validName
// Return Type  : void
//
void ParameterTree::u_canonicalizeName(ParameterTree *obj,
                                       ::coder::array<char, 2U> &validName)
{
  array<char, 2U> r;
  char name[44];
  for (int i{0}; i < 44; i++) {
    name[i] = cv15[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&obj->ParameterHelper,
                                                    &name[0]);
  validName.set_size(1, 44);
  for (int i{0}; i < 44; i++) {
    validName[i] = cv15[i];
  }
  if (startsWith(validName)) {
    r.set_size(1, 43);
    r[0] = '~';
    for (int i{0}; i < 42; i++) {
      r[i + 1] = validName[i + 2];
    }
    validName.set_size(1, 43);
    for (int i{0}; i < 43; i++) {
      validName[i] = r[i];
    }
  }
}

//
// Arguments    : void
// Return Type  : double
//
double ParameterTree::u_get()
{
  array<char, 2U> in;
  array<char, 2U> parameterName;
  double data;
  int loop_ub;
  char name[43];
  for (int i{0}; i < 43; i++) {
    name[i] = cv20[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&ParameterHelper, &name[0]);
  in.set_size(1, 43);
  for (int i{0}; i < 43; i++) {
    in[i] = cv20[i];
  }
  if (startsWith(in)) {
    parameterName.set_size(1, 42);
    parameterName[0] = '~';
    for (int i{0}; i < 41; i++) {
      parameterName[i + 1] = in[i + 2];
    }
    in.set_size(1, 42);
    for (int i{0}; i < 42; i++) {
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

//
// Arguments    : ParameterTree *obj
//                ::coder::array<char, 2U> &validName
// Return Type  : void
//
void ParameterTree::v_canonicalizeName(ParameterTree *obj,
                                       ::coder::array<char, 2U> &validName)
{
  array<char, 2U> r;
  char name[50];
  for (int i{0}; i < 50; i++) {
    name[i] = cv16[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&obj->ParameterHelper,
                                                    &name[0]);
  validName.set_size(1, 50);
  for (int i{0}; i < 50; i++) {
    validName[i] = cv16[i];
  }
  if (startsWith(validName)) {
    r.set_size(1, 49);
    r[0] = '~';
    for (int i{0}; i < 48; i++) {
      r[i + 1] = validName[i + 2];
    }
    validName.set_size(1, 49);
    for (int i{0}; i < 49; i++) {
      validName[i] = r[i];
    }
  }
}

//
// Arguments    : void
// Return Type  : double
//
double ParameterTree::v_get()
{
  array<char, 2U> in;
  array<char, 2U> parameterName;
  double data;
  int loop_ub;
  char name[44];
  for (int i{0}; i < 44; i++) {
    name[i] = cv21[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&ParameterHelper, &name[0]);
  in.set_size(1, 44);
  for (int i{0}; i < 44; i++) {
    in[i] = cv21[i];
  }
  if (startsWith(in)) {
    parameterName.set_size(1, 43);
    parameterName[0] = '~';
    for (int i{0}; i < 42; i++) {
      parameterName[i + 1] = in[i + 2];
    }
    in.set_size(1, 43);
    for (int i{0}; i < 43; i++) {
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

//
// Arguments    : ParameterTree *obj
//                ::coder::array<char, 2U> &validName
// Return Type  : void
//
void ParameterTree::w_canonicalizeName(ParameterTree *obj,
                                       ::coder::array<char, 2U> &validName)
{
  array<char, 2U> r;
  char name[51];
  for (int i{0}; i < 51; i++) {
    name[i] = cv17[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&obj->ParameterHelper,
                                                    &name[0]);
  validName.set_size(1, 51);
  for (int i{0}; i < 51; i++) {
    validName[i] = cv17[i];
  }
  if (startsWith(validName)) {
    r.set_size(1, 50);
    r[0] = '~';
    for (int i{0}; i < 49; i++) {
      r[i + 1] = validName[i + 2];
    }
    validName.set_size(1, 50);
    for (int i{0}; i < 50; i++) {
      validName[i] = r[i];
    }
  }
}

//
// Arguments    : void
// Return Type  : double
//
double ParameterTree::w_get()
{
  array<char, 2U> in;
  array<char, 2U> parameterName;
  double data;
  int loop_ub;
  char name[44];
  for (int i{0}; i < 44; i++) {
    name[i] = cv22[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&ParameterHelper, &name[0]);
  in.set_size(1, 44);
  for (int i{0}; i < 44; i++) {
    in[i] = cv22[i];
  }
  if (startsWith(in)) {
    parameterName.set_size(1, 43);
    parameterName[0] = '~';
    for (int i{0}; i < 42; i++) {
      parameterName[i + 1] = in[i + 2];
    }
    in.set_size(1, 43);
    for (int i{0}; i < 43; i++) {
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

//
// Arguments    : ParameterTree *obj
//                ::coder::array<char, 2U> &validName
// Return Type  : void
//
void ParameterTree::x_canonicalizeName(ParameterTree *obj,
                                       ::coder::array<char, 2U> &validName)
{
  array<char, 2U> r;
  char name[50];
  for (int i{0}; i < 50; i++) {
    name[i] = cv18[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&obj->ParameterHelper,
                                                    &name[0]);
  validName.set_size(1, 50);
  for (int i{0}; i < 50; i++) {
    validName[i] = cv18[i];
  }
  if (startsWith(validName)) {
    r.set_size(1, 49);
    r[0] = '~';
    for (int i{0}; i < 48; i++) {
      r[i + 1] = validName[i + 2];
    }
    validName.set_size(1, 49);
    for (int i{0}; i < 49; i++) {
      validName[i] = r[i];
    }
  }
}

//
// Arguments    : void
// Return Type  : double
//
double ParameterTree::x_get()
{
  array<char, 2U> in;
  array<char, 2U> parameterName;
  double data;
  int loop_ub;
  char name[44];
  for (int i{0}; i < 44; i++) {
    name[i] = cv23[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&ParameterHelper, &name[0]);
  in.set_size(1, 44);
  for (int i{0}; i < 44; i++) {
    in[i] = cv23[i];
  }
  if (startsWith(in)) {
    parameterName.set_size(1, 43);
    parameterName[0] = '~';
    for (int i{0}; i < 42; i++) {
      parameterName[i + 1] = in[i + 2];
    }
    in.set_size(1, 43);
    for (int i{0}; i < 43; i++) {
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

//
// Arguments    : ParameterTree *obj
//                ::coder::array<char, 2U> &validName
// Return Type  : void
//
void ParameterTree::y_canonicalizeName(ParameterTree *obj,
                                       ::coder::array<char, 2U> &validName)
{
  array<char, 2U> r;
  char name[45];
  for (int i{0}; i < 45; i++) {
    name[i] = cv19[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&obj->ParameterHelper,
                                                    &name[0]);
  validName.set_size(1, 45);
  for (int i{0}; i < 45; i++) {
    validName[i] = cv19[i];
  }
  if (startsWith(validName)) {
    r.set_size(1, 44);
    r[0] = '~';
    for (int i{0}; i < 43; i++) {
      r[i + 1] = validName[i + 2];
    }
    validName.set_size(1, 44);
    for (int i{0}; i < 44; i++) {
      validName[i] = r[i];
    }
  }
}

//
// Arguments    : void
// Return Type  : double
//
double ParameterTree::y_get()
{
  array<char, 2U> in;
  array<char, 2U> parameterName;
  double data;
  int loop_ub;
  char name[44];
  for (int i{0}; i < 44; i++) {
    name[i] = cv24[i];
  }
  std::mem_fn (&MATLABROSParameter::isValidPattern)(&ParameterHelper, &name[0]);
  in.set_size(1, 44);
  for (int i{0}; i < 44; i++) {
    in[i] = cv24[i];
  }
  if (startsWith(in)) {
    parameterName.set_size(1, 43);
    parameterName[0] = '~';
    for (int i{0}; i < 42; i++) {
      parameterName[i + 1] = in[i + 2];
    }
    in.set_size(1, 43);
    for (int i{0}; i < 43; i++) {
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

//
// File trailer for ParameterTree.cpp
//
// [EOF]
//
