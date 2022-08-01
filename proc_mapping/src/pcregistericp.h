//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: pcregistericp.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 01-Aug-2022 08:26:09
//

#ifndef PCREGISTERICP_H
#define PCREGISTERICP_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace coder {
class b_pointCloud;

class pointCloud;

class rigid3d;

} // namespace coder

// Function Declarations
namespace coder {
void pcregistericp(b_pointCloud *moving, pointCloud *fixed, double varargin_2,
                   rigid3d *tform);

}

#endif
//
// File trailer for pcregistericp.h
//
// [EOF]
//
