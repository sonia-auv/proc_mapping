//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: pccat.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 31-Jul-2022 13:03:34
//

#ifndef PCCAT_H
#define PCCAT_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace coder {
class f_pointCloud;

namespace vision {
namespace internal {
namespace codegen {
class Kdtree;

}
} // namespace internal
} // namespace vision
class b_pointCloud;

} // namespace coder

// Function Declarations
namespace coder {
namespace vision {
namespace internal {
namespace codegen {
namespace pc {
b_pointCloud *pccat(const f_pointCloud *b_pc, Kdtree *iobj_0,
                    b_pointCloud *iobj_1);

}
} // namespace codegen
} // namespace internal
} // namespace vision
} // namespace coder

#endif
//
// File trailer for pccat.h
//
// [EOF]
//
