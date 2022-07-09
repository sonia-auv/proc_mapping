//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// pcregistericp.h
//
// Code generation for function 'pcregistericp'
//

#ifndef PCREGISTERICP_H
#define PCREGISTERICP_H

// Include files
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
void pcregistericp(const b_pointCloud *moving, pointCloud *fixed,
                   rigid3d *tform);

}

#endif
// End of code generation (pcregistericp.h)
