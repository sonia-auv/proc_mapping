//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: quatUtilities.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 31-Jul-2022 13:03:34
//

#ifndef QUATUTILITIES_H
#define QUATUTILITIES_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace coder {
class planeModel;

class pointCloud;

} // namespace coder

// Type Definitions
class quatUtilities {
public:
  static void getOrientedPointOnPlanarFace(const coder::planeModel *model,
                                           coder::pointCloud *plane,
                                           double p[3], double q[4]);
};

#endif
//
// File trailer for quatUtilities.h
//
// [EOF]
//
