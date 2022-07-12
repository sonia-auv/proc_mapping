//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// pcdownsample.cpp
//
// Code generation for function 'pcdownsample'
//

// Include files
#include "pcdownsample.h"
#include "Kdtree.h"
#include "pointCloud.h"
#include "pointCloudArray.h"
#include "rt_nonfinite.h"
#include "voxelGridFilter.h"
#include "coder_array.h"
#include <string.h>

// Function Definitions
namespace coder {
c_pointCloud *pcdownsample(const pointCloud *ptCloudIn, double varargin_2,
                           c_pointCloud *iobj_0)
{
  c_pointCloud *ptCloudOut;
  pointCloud pc;
  pointCloud *b_ptCloudOut;
  vision::internal::codegen::Kdtree lobj_1;
  array<double, 2U> b_rangeData;
  array<double, 2U> location;
  array<double, 2U> normals;
  array<double, 2U> rangeData;
  array<double, 2U> tempNV;
  array<double, 1U> tempI;
  array<unsigned char, 2U> C_;
  array<unsigned char, 2U> color;
  array<bool, 1U> indices;
  int loop_ub;
  pc.matlabCodegenIsDeleted = true;
  ptCloudIn->extractValidPoints(location, color, normals, tempI, rangeData,
                                indices);
  b_ptCloudOut = pc.init(location, color, normals, tempI, &lobj_1);
  b_ptCloudOut->RangeData.set_size(rangeData.size(0), rangeData.size(1));
  loop_ub = rangeData.size(0) * rangeData.size(1);
  for (int i{0}; i < loop_ub; i++) {
    b_ptCloudOut->RangeData[i] = rangeData[i];
  }
  vision::internal::codegen::pc::voxelGridFilter(
      pc.Location, pc.Color, pc.Normal, pc.Intensity, pc.RangeData, varargin_2,
      location, C_, tempNV, tempI, b_rangeData);
  pc.matlabCodegenDestructor();
  ptCloudOut = iobj_0;
  iobj_0->Location.set_size(location.size(0), 3);
  loop_ub = location.size(0) * 3;
  for (int i{0}; i < loop_ub; i++) {
    iobj_0->Location[i] = location[i];
  }
  iobj_0->Color.set_size(C_.size(0), 3);
  loop_ub = C_.size(0) * 3;
  for (int i{0}; i < loop_ub; i++) {
    iobj_0->Color[i] = C_[i];
  }
  iobj_0->Normal.set_size(tempNV.size(0), 3);
  loop_ub = tempNV.size(0) * 3;
  for (int i{0}; i < loop_ub; i++) {
    iobj_0->Normal[i] = tempNV[i];
  }
  iobj_0->Intensity.set_size(tempI.size(0));
  loop_ub = tempI.size(0);
  for (int i{0}; i < loop_ub; i++) {
    iobj_0->Intensity[i] = tempI[i];
  }
  pointclouds::internal::codegen::pc::pointCloudArray r;
  iobj_0->RangeData.set_size(0, 0);
  iobj_0->PointCloudArrayData.set_size(1, 1);
  iobj_0->PointCloudArrayData[0] = r;
  iobj_0->b_Kdtree = iobj_0->_pobj0.init();
  iobj_0->matlabCodegenIsDeleted = false;
  iobj_0->RangeData.set_size(b_rangeData.size(0), 3);
  loop_ub = b_rangeData.size(0) * 3;
  for (int i{0}; i < loop_ub; i++) {
    iobj_0->RangeData[i] = b_rangeData[i];
  }
  return ptCloudOut;
}

} // namespace coder

// End of code generation (pcdownsample.cpp)
