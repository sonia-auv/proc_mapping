//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: pctransform.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 31-Jul-2022 13:03:34
//

#ifndef PCTRANSFORM_H
#define PCTRANSFORM_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace coder {
class g_pointCloud;

class rigid3d;

class f_pointCloud;

} // namespace coder

// Function Declarations
namespace coder {
f_pointCloud *pctransform(const g_pointCloud *ptCloudIn, const rigid3d *tform,
                          f_pointCloud *iobj_0);

}

#endif
//
// File trailer for pctransform.h
//
// [EOF]
//
