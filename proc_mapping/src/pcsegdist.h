//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: pcsegdist.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 31-Jul-2022 13:03:34
//

#ifndef PCSEGDIST_H
#define PCSEGDIST_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace coder {
class pointCloud;

}

// Function Declarations
namespace coder {
void pcsegdist(const pointCloud *ptCloud, double minDistance,
               ::coder::array<unsigned int, 1U> &labels, double *numClusters);

}

#endif
//
// File trailer for pcsegdist.h
//
// [EOF]
//
