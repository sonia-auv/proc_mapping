//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// pcsegdist.h
//
// Code generation for function 'pcsegdist'
//

#ifndef PCSEGDIST_H
#define PCSEGDIST_H

// Include files
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
// End of code generation (pcsegdist.h)
