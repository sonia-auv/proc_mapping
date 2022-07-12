//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// pctransform.h
//
// Code generation for function 'pctransform'
//

#ifndef PCTRANSFORM_H
#define PCTRANSFORM_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace coder {
class pointCloud;

class rigid3d;

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
pointCloud *pctransform(const pointCloud *ptCloudIn, const rigid3d *tform,
                        vision::internal::codegen::Kdtree *iobj_0,
                        pointCloud *iobj_1);

b_pointCloud *pctransform(const b_pointCloud *ptCloudIn, const rigid3d *tform,
                          vision::internal::codegen::Kdtree *iobj_0,
                          b_pointCloud *iobj_1);

} // namespace coder

#endif
// End of code generation (pctransform.h)
