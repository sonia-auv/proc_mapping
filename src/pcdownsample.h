//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// pcdownsample.h
//
// Code generation for function 'pcdownsample'
//

#ifndef PCDOWNSAMPLE_H
#define PCDOWNSAMPLE_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace coder {
class pointCloud;

class c_pointCloud;

} // namespace coder

// Function Declarations
namespace coder {
c_pointCloud *pcdownsample(const pointCloud *ptCloudIn, double varargin_2,
                           c_pointCloud *iobj_0);

}

#endif
// End of code generation (pcdownsample.h)
