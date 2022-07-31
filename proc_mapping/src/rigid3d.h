//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: rigid3d.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 31-Jul-2022 13:03:34
//

#ifndef RIGID3D_H
#define RIGID3D_H

// Include Files
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
  void init(const double varargin_2[3]);
  static bool isTransformationMatrixRigid(const double T[16]);
  affine3d AffineTform;
  array<images::internal::rigid3dImpl, 2U> Data;
};

} // namespace coder

#endif
//
// File trailer for rigid3d.h
//
// [EOF]
//
