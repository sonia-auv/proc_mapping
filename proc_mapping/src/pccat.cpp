//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: pccat.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 31-Jul-2022 13:03:34
//

// Include Files
#include "pccat.h"
#include "Kdtree.h"
#include "pointCloud.h"
#include "pointCloudArray.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cmath>
#include <string.h>

// Function Definitions
//
// Arguments    : const f_pointCloud *b_pc
//                Kdtree *iobj_0
//                b_pointCloud *iobj_1
// Return Type  : b_pointCloud *
//
namespace coder {
namespace vision {
namespace internal {
namespace codegen {
namespace pc {
b_pointCloud *pccat(const f_pointCloud *b_pc, Kdtree *iobj_0,
                    b_pointCloud *iobj_1)
{
  Kdtree lobj_2[2];
  Kdtree lobj_0;
  b_pointCloud lobj_3[2];
  b_pointCloud lobj_1;
  b_pointCloud *ptCloudOut;
  b_pointCloud *this1;
  array<pointclouds::internal::codegen::pc::b_pointCloudArray, 2U> dataArray;
  array<pointclouds::internal::codegen::pc::b_pointCloudArray, 1U> r;
  array<float, 2U> b_locationArray;
  array<float, 2U> intensity;
  array<float, 2U> intensityArray;
  array<float, 2U> locationArray;
  array<float, 2U> normal;
  array<float, 2U> normalArray;
  array<float, 2U> rangeData;
  array<float, 2U> x;
  array<int, 1U> y;
  array<unsigned char, 2U> b_colorArray;
  array<unsigned char, 2U> color;
  array<unsigned char, 2U> colorArray;
  array<bool, 2U> b_x;
  array<bool, 2U> r1;
  array<bool, 1U> b_y;
  int loop_ub;
  int numClouds;
  bool hasColor;
  bool hasIntensity;
  bool hasNormal;
  lobj_3[0].matlabCodegenIsDeleted = true;
  lobj_3[1].matlabCodegenIsDeleted = true;
  lobj_1.matlabCodegenIsDeleted = true;
  numClouds =
      b_pc->PointCloudArrayData.size(0) * b_pc->PointCloudArrayData.size(1) - 1;
  hasColor = true;
  hasIntensity = true;
  hasNormal = true;
  b_pc->parenReference(&lobj_2[0], &lobj_3[0]);
  b_pc->parenReference(&lobj_2[1], &lobj_3[1]);
  locationArray.set_size(0, 3);
  colorArray.set_size(0, 0);
  normalArray.set_size(0, 0);
  intensityArray.set_size(0, 0);
  if (numClouds >= 0) {
    loop_ub =
        b_pc->PointCloudArrayData.size(0) * b_pc->PointCloudArrayData.size(1);
  }
  for (int i{0}; i <= numClouds; i++) {
    int sizes_idx_0;
    int vstride;
    int xoffset;
    bool empty_non_axis_sizes;
    this1 = pointCloud::makeEmptyPtCloud(&lobj_0, &lobj_1);
    dataArray.set_size(1, 1);
    r.set_size(b_pc->PointCloudArrayData.size(0) *
               b_pc->PointCloudArrayData.size(1));
    for (int k{0}; k < loop_ub; k++) {
      r[k] = b_pc->PointCloudArrayData[k];
    }
    dataArray[0] = r[i];
    this1->PointCloudArrayData.set_size(1, 1);
    this1->PointCloudArrayData[0] = dataArray[0];
    this1->Location.set_size(dataArray[0].Location.size(0), 3);
    vstride = dataArray[0].Location.size(0) * 3;
    for (int k{0}; k < vstride; k++) {
      this1->Location[k] = dataArray[0].Location[k];
    }
    this1->Color.set_size(dataArray[0].Color.size(0), this1->Color.size(1));
    this1->Color.set_size(this1->Color.size(0), dataArray[0].Color.size(1));
    vstride = dataArray[0].Color.size(0) * dataArray[0].Color.size(1);
    for (int k{0}; k < vstride; k++) {
      this1->Color[k] = dataArray[0].Color[k];
    }
    this1->Normal.set_size(dataArray[0].Normal.size(0), this1->Normal.size(1));
    this1->Normal.set_size(this1->Normal.size(0), dataArray[0].Normal.size(1));
    vstride = dataArray[0].Normal.size(0) * dataArray[0].Normal.size(1);
    for (int k{0}; k < vstride; k++) {
      this1->Normal[k] = dataArray[0].Normal[k];
    }
    this1->Intensity.set_size(dataArray[0].Intensity.size(0),
                              this1->Intensity.size(1));
    this1->Intensity.set_size(this1->Intensity.size(0),
                              dataArray[0].Intensity.size(1));
    vstride = dataArray[0].Intensity.size(0) * dataArray[0].Intensity.size(1);
    for (int k{0}; k < vstride; k++) {
      this1->Intensity[k] = dataArray[0].Intensity[k];
    }
    x.set_size(this1->Location.size(0), 3);
    vstride = this1->Location.size(0) * 3;
    for (int k{0}; k < vstride; k++) {
      x[k] = this1->Location[k];
    }
    b_x.set_size(x.size(0), 3);
    vstride = x.size(0) * 3;
    for (int k{0}; k < vstride; k++) {
      b_x[k] = std::isinf(x[k]);
    }
    r1.set_size(x.size(0), 3);
    vstride = x.size(0) * 3;
    for (int k{0}; k < vstride; k++) {
      r1[k] = std::isnan(x[k]);
    }
    vstride = b_x.size(0) * 3;
    b_x.set_size(b_x.size(0), 3);
    for (int k{0}; k < vstride; k++) {
      b_x[k] = ((!b_x[k]) && (!r1[k]));
    }
    if (b_x.size(0) == 0) {
      y.set_size(0);
    } else {
      vstride = b_x.size(0);
      y.set_size(b_x.size(0));
      for (sizes_idx_0 = 0; sizes_idx_0 < vstride; sizes_idx_0++) {
        y[sizes_idx_0] = b_x[sizes_idx_0];
      }
      for (int k{0}; k < 2; k++) {
        xoffset = (k + 1) * vstride;
        for (sizes_idx_0 = 0; sizes_idx_0 < vstride; sizes_idx_0++) {
          y[sizes_idx_0] = y[sizes_idx_0] + b_x[xoffset + sizes_idx_0];
        }
      }
    }
    b_y.set_size(y.size(0));
    vstride = y.size(0);
    for (int k{0}; k < vstride; k++) {
      b_y[k] = (y[k] == 3);
    }
    this1->subsetImpl(b_y, x, color, normal, intensity, rangeData);
    vstride = locationArray.size(0);
    b_locationArray.set_size(locationArray.size(0) + x.size(0), 3);
    for (int k{0}; k < 3; k++) {
      for (int b_i{0}; b_i < vstride; b_i++) {
        b_locationArray[b_i + b_locationArray.size(0) * k] =
            locationArray[b_i + locationArray.size(0) * k];
      }
    }
    vstride = x.size(0);
    for (int k{0}; k < 3; k++) {
      for (int b_i{0}; b_i < vstride; b_i++) {
        b_locationArray[(b_i + locationArray.size(0)) +
                        b_locationArray.size(0) * k] = x[b_i + x.size(0) * k];
      }
    }
    locationArray.set_size(b_locationArray.size(0), 3);
    vstride = b_locationArray.size(0) * 3;
    for (int k{0}; k < vstride; k++) {
      locationArray[k] = b_locationArray[k];
    }
    if ((colorArray.size(0) != 0) && (colorArray.size(1) != 0)) {
      xoffset = colorArray.size(1);
    } else if ((color.size(0) != 0) && (color.size(1) != 0)) {
      xoffset = color.size(1);
    } else {
      xoffset = colorArray.size(1);
      if (color.size(1) > colorArray.size(1)) {
        xoffset = color.size(1);
      }
    }
    empty_non_axis_sizes = (xoffset == 0);
    if (empty_non_axis_sizes ||
        ((colorArray.size(0) != 0) && (colorArray.size(1) != 0))) {
      vstride = colorArray.size(0);
    } else {
      vstride = 0;
    }
    if (empty_non_axis_sizes ||
        ((color.size(0) != 0) && (color.size(1) != 0))) {
      sizes_idx_0 = color.size(0);
    } else {
      sizes_idx_0 = 0;
    }
    b_colorArray.set_size(vstride + sizes_idx_0, xoffset);
    for (int k{0}; k < xoffset; k++) {
      for (int b_i{0}; b_i < vstride; b_i++) {
        b_colorArray[b_i + b_colorArray.size(0) * k] =
            colorArray[b_i + vstride * k];
      }
    }
    for (int k{0}; k < xoffset; k++) {
      for (int b_i{0}; b_i < sizes_idx_0; b_i++) {
        b_colorArray[(b_i + vstride) + b_colorArray.size(0) * k] =
            color[b_i + sizes_idx_0 * k];
      }
    }
    colorArray.set_size(b_colorArray.size(0), b_colorArray.size(1));
    vstride = b_colorArray.size(0) * b_colorArray.size(1);
    for (int k{0}; k < vstride; k++) {
      colorArray[k] = b_colorArray[k];
    }
    if ((normalArray.size(0) != 0) && (normalArray.size(1) != 0)) {
      xoffset = normalArray.size(1);
    } else if ((normal.size(0) != 0) && (normal.size(1) != 0)) {
      xoffset = normal.size(1);
    } else {
      xoffset = normalArray.size(1);
      if (normal.size(1) > normalArray.size(1)) {
        xoffset = normal.size(1);
      }
    }
    empty_non_axis_sizes = (xoffset == 0);
    if (empty_non_axis_sizes ||
        ((normalArray.size(0) != 0) && (normalArray.size(1) != 0))) {
      vstride = normalArray.size(0);
    } else {
      vstride = 0;
    }
    if (empty_non_axis_sizes ||
        ((normal.size(0) != 0) && (normal.size(1) != 0))) {
      sizes_idx_0 = normal.size(0);
    } else {
      sizes_idx_0 = 0;
    }
    rangeData.set_size(vstride + sizes_idx_0, xoffset);
    for (int k{0}; k < xoffset; k++) {
      for (int b_i{0}; b_i < vstride; b_i++) {
        rangeData[b_i + rangeData.size(0) * k] = normalArray[b_i + vstride * k];
      }
    }
    for (int k{0}; k < xoffset; k++) {
      for (int b_i{0}; b_i < sizes_idx_0; b_i++) {
        rangeData[(b_i + vstride) + rangeData.size(0) * k] =
            normal[b_i + sizes_idx_0 * k];
      }
    }
    normalArray.set_size(rangeData.size(0), rangeData.size(1));
    vstride = rangeData.size(0) * rangeData.size(1);
    for (int k{0}; k < vstride; k++) {
      normalArray[k] = rangeData[k];
    }
    if ((intensityArray.size(0) != 0) && (intensityArray.size(1) != 0)) {
      xoffset = intensityArray.size(1);
    } else if ((intensity.size(0) != 0) && (intensity.size(1) != 0)) {
      xoffset = intensity.size(1);
    } else {
      xoffset = intensityArray.size(1);
      if (intensity.size(1) > intensityArray.size(1)) {
        xoffset = intensity.size(1);
      }
    }
    empty_non_axis_sizes = (xoffset == 0);
    if (empty_non_axis_sizes ||
        ((intensityArray.size(0) != 0) && (intensityArray.size(1) != 0))) {
      vstride = intensityArray.size(0);
    } else {
      vstride = 0;
    }
    if (empty_non_axis_sizes ||
        ((intensity.size(0) != 0) && (intensity.size(1) != 0))) {
      sizes_idx_0 = intensity.size(0);
    } else {
      sizes_idx_0 = 0;
    }
    rangeData.set_size(vstride + sizes_idx_0, xoffset);
    for (int k{0}; k < xoffset; k++) {
      for (int b_i{0}; b_i < vstride; b_i++) {
        rangeData[b_i + rangeData.size(0) * k] =
            intensityArray[b_i + vstride * k];
      }
    }
    for (int k{0}; k < xoffset; k++) {
      for (int b_i{0}; b_i < sizes_idx_0; b_i++) {
        rangeData[(b_i + vstride) + rangeData.size(0) * k] =
            intensity[b_i + sizes_idx_0 * k];
      }
    }
    intensityArray.set_size(rangeData.size(0), rangeData.size(1));
    vstride = rangeData.size(0) * rangeData.size(1);
    for (int k{0}; k < vstride; k++) {
      intensityArray[k] = rangeData[k];
    }
    if (x.size(0) != 0) {
      if ((!hasColor) || ((color.size(0) == 0) || (color.size(1) == 0))) {
        hasColor = false;
      }
      if ((!hasIntensity) ||
          ((intensity.size(0) == 0) || (intensity.size(1) == 0))) {
        hasIntensity = false;
      }
      if ((!hasNormal) || ((normal.size(0) == 0) || (normal.size(1) == 0))) {
        hasNormal = false;
      }
    }
    lobj_1.matlabCodegenDestructor();
  }
  if ((!hasColor) ||
      (colorArray.size(0) * colorArray.size(1) != locationArray.size(0) * 3)) {
    colorArray.set_size(0, 0);
  }
  if ((!hasNormal) || (normalArray.size(0) * normalArray.size(1) !=
                       locationArray.size(0) * 3)) {
    normalArray.set_size(0, 0);
  }
  if ((!hasIntensity) || (intensityArray.size(0) != locationArray.size(0))) {
    intensityArray.set_size(0, 0);
  }
  ptCloudOut = iobj_1->init(locationArray, colorArray, normalArray,
                            intensityArray, iobj_0);
  lobj_3[0].matlabCodegenDestructor();
  lobj_3[1].matlabCodegenDestructor();
  return ptCloudOut;
}

} // namespace pc
} // namespace codegen
} // namespace internal
} // namespace vision
} // namespace coder

//
// File trailer for pccat.cpp
//
// [EOF]
//
