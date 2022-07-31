//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: pcfitplane.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 31-Jul-2022 13:03:34
//

#ifndef PCFITPLANE_H
#define PCFITPLANE_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace coder {
class pointCloud;

class planeModel;

} // namespace coder

// Function Declarations
namespace coder {
void evalPlane(const ::coder::array<double, 2U> &model,
               const ::coder::array<double, 2U> &points,
               ::coder::array<double, 1U> &dis);

void pcfitplane(const pointCloud *varargin_1, double varargin_2,
                planeModel *iobj_0, planeModel **model,
                ::coder::array<double, 1U> &inlierIndices,
                ::coder::array<double, 1U> &outlierIndices,
                ::coder::array<double, 2U> &meanError);

} // namespace coder

#endif
//
// File trailer for pcfitplane.h
//
// [EOF]
//
