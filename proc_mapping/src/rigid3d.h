//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// rigid3d.h
//
// Code generation for function 'rigid3d'
//

#ifndef RIGID3D_H
#define RIGID3D_H

// Include files
#include "affine3d.h"
#include "rigid3dImpl.h"
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
namespace coder {
class rigid3d {
public:
  static bool isTransformationMatrixRigid(const double T[16]);
  void init(const double varargin_1[9], const double varargin_2[3]);
  affine3d AffineTform;
  array<images::internal::rigid3dImpl, 2U> Data;
};

} // namespace coder

#endif
// End of code generation (rigid3d.h)
