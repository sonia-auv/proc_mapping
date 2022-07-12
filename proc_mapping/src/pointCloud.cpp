//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// pointCloud.cpp
//
// Code generation for function 'pointCloud'
//

// Include files
#include "pointCloud.h"
#include "Kdtree.h"
#include "any1.h"
#include "find.h"
#include "log2.h"
#include "minOrMax.h"
#include "pointCloudArray.h"
#include "proc_mapping_internal_types.h"
#include "proc_mapping_rtwutil.h"
#include "rt_nonfinite.h"
#include "sort.h"
#include "PCANormalCore_api.hpp"
#include "coder_array.h"
#include <cmath>
#include <cstring>
#include <string.h>

// Function Definitions
namespace coder {
void e_pointCloud::buildKdtree()
{
  static float inputData[22317];
  vision::internal::codegen::c_Kdtree *b_this;
  array<cell_wrap_41, 1U> idxTemp1;
  array<double, 2U> lowerBoundsTemp;
  array<double, 2U> upperBoundsTemp;
  array<double, 1U> cutValTemp;
  array<double, 1U> leftChildTemp;
  array<double, 1U> rightChildTemp;
  array<float, 1U> sx;
  array<int, 1U> iidx;
  array<unsigned int, 1U> tempDim;
  array<short, 2U> notnan;
  array<signed char, 1U> cutDimTemp;
  array<bool, 1U> leafNodeTemp;
  short tempIdx[7439];
  if (!b_Kdtree->IsIndexed) {
    double cc;
    double temp_tmp;
    unsigned int currentNode;
    int i;
    int i1;
    int idx;
    int loop_ub;
    unsigned int nextUnusedNode;
    int nx;
    int unusedNodes;
    bool bv[22317];
    bool wasnan[7439];
    bool exitg1;
    bool y;
    b_this = b_Kdtree;
    for (i = 0; i < 22317; i++) {
      inputData[i] = Location[i];
    }
    for (i = 0; i < 22317; i++) {
      bv[i] = std::isnan(inputData[i]);
    }
    b_any(bv, wasnan);
    notnan.set_size(1, 7439);
    for (i = 0; i < 7439; i++) {
      notnan[i] = static_cast<short>(i + 1);
    }
    y = false;
    unusedNodes = 0;
    exitg1 = false;
    while ((!exitg1) && (unusedNodes <= 7438)) {
      if (wasnan[unusedNodes]) {
        y = true;
        exitg1 = true;
      } else {
        unusedNodes++;
      }
    }
    if (y) {
      idx = 0;
      notnan.set_size(1, 7439);
      unusedNodes = 0;
      exitg1 = false;
      while ((!exitg1) && (unusedNodes < 7439)) {
        if (!wasnan[unusedNodes]) {
          idx++;
          notnan[idx - 1] = static_cast<short>(unusedNodes + 1);
          if (idx >= 7439) {
            exitg1 = true;
          } else {
            unusedNodes++;
          }
        } else {
          unusedNodes++;
        }
      }
      if (idx < 1) {
        idx = 0;
      }
      notnan.set_size(notnan.size(0), idx);
      nx = notnan.size(1);
    } else {
      nx = 7439;
    }
    cc = b_log2(std::fmax(static_cast<double>(nx) / 50.0, 1.0));
    cc = std::ceil(cc);
    idx = static_cast<int>(rt_powd_snf(2.0, cc + 1.0) - 1.0);
    cutDimTemp.set_size(idx);
    cutValTemp.set_size(idx);
    for (i = 0; i < idx; i++) {
      cutDimTemp[i] = 0;
      cutValTemp[i] = 0.0;
    }
    lowerBoundsTemp.set_size(idx, 3);
    unusedNodes = static_cast<int>(rt_powd_snf(2.0, cc + 1.0) - 1.0) * 3;
    upperBoundsTemp.set_size(idx, 3);
    for (i = 0; i < unusedNodes; i++) {
      lowerBoundsTemp[i] = rtMinusInf;
      upperBoundsTemp[i] = rtInf;
    }
    leftChildTemp.set_size(idx);
    rightChildTemp.set_size(idx);
    leafNodeTemp.set_size(idx);
    idxTemp1.set_size(idx);
    for (i = 0; i < idx; i++) {
      leftChildTemp[i] = 0.0;
      rightChildTemp[i] = 0.0;
      leafNodeTemp[i] = false;
      idxTemp1[i].f1.set_size(1, 0);
    }
    idxTemp1[0].f1.set_size(1, notnan.size(1));
    loop_ub = notnan.size(1);
    for (i = 0; i < loop_ub; i++) {
      idxTemp1[0].f1[i] = notnan[i];
    }
    currentNode = 1U;
    nextUnusedNode = 2U;
    while (currentNode < nextUnusedNode) {
      if (idxTemp1[static_cast<int>(currentNode) - 1].f1.size(1) <= 50) {
        leafNodeTemp[static_cast<int>(currentNode) - 1] = true;
      } else {
        float b_ex[3];
        float ex[3];
        float b_tmp;
        float p;
        iidx.set_size(idxTemp1[static_cast<int>(currentNode) - 1].f1.size(1));
        loop_ub = idxTemp1[static_cast<int>(currentNode) - 1].f1.size(1);
        for (i = 0; i < loop_ub; i++) {
          iidx[i] = static_cast<int>(
              idxTemp1[static_cast<int>(currentNode) - 1].f1[i]);
        }
        unusedNodes = iidx.size(0);
        idx = iidx.size(0);
        for (loop_ub = 0; loop_ub < 3; loop_ub++) {
          p = inputData[(iidx[0] + 7439 * loop_ub) - 1];
          ex[loop_ub] = p;
          for (int c{2}; c <= unusedNodes; c++) {
            b_tmp = inputData[(iidx[c - 1] + 7439 * loop_ub) - 1];
            if (std::isnan(b_tmp)) {
              y = false;
            } else if (std::isnan(ex[loop_ub])) {
              y = true;
            } else {
              y = (ex[loop_ub] < b_tmp);
            }
            if (y) {
              ex[loop_ub] = b_tmp;
            }
          }
          b_ex[loop_ub] = p;
          for (int c{2}; c <= idx; c++) {
            b_tmp = inputData[(iidx[c - 1] + 7439 * loop_ub) - 1];
            if (std::isnan(b_tmp)) {
              y = false;
            } else if (std::isnan(b_ex[loop_ub])) {
              y = true;
            } else {
              y = (b_ex[loop_ub] > b_tmp);
            }
            if (y) {
              b_ex[loop_ub] = b_tmp;
            }
          }
          ex[loop_ub] -= b_ex[loop_ub];
        }
        if (!std::isnan(ex[0])) {
          idx = 1;
        } else {
          idx = 0;
          unusedNodes = 2;
          exitg1 = false;
          while ((!exitg1) && (unusedNodes < 4)) {
            if (!std::isnan(ex[unusedNodes - 1])) {
              idx = unusedNodes;
              exitg1 = true;
            } else {
              unusedNodes++;
            }
          }
        }
        if (idx == 0) {
          idx = 1;
        } else {
          p = ex[idx - 1];
          i = idx + 1;
          for (unusedNodes = i; unusedNodes < 4; unusedNodes++) {
            b_tmp = ex[unusedNodes - 1];
            if (p < b_tmp) {
              p = b_tmp;
              idx = unusedNodes;
            }
          }
        }
        sx.set_size(idxTemp1[static_cast<int>(currentNode) - 1].f1.size(1));
        loop_ub = idxTemp1[static_cast<int>(currentNode) - 1].f1.size(1);
        for (i = 0; i < loop_ub; i++) {
          sx[i] = inputData[(static_cast<int>(
                                 idxTemp1[static_cast<int>(currentNode) - 1]
                                     .f1[i]) +
                             7439 * (idx - 1)) -
                            1];
        }
        internal::sort(sx, iidx);
        notnan.set_size(1, iidx.size(0));
        loop_ub = iidx.size(0);
        for (i = 0; i < loop_ub; i++) {
          notnan[i] = static_cast<short>(
              idxTemp1[static_cast<int>(currentNode) - 1].f1[iidx[i] - 1]);
        }
        double temp[3];
        double b_temp_tmp;
        unusedNodes =
            static_cast<int>(std::ceil(static_cast<double>(sx.size(0)) / 2.0));
        p = (sx[unusedNodes - 1] + sx[unusedNodes]) / 2.0F;
        cutDimTemp[static_cast<int>(currentNode) - 1] =
            static_cast<signed char>(idx);
        cutValTemp[static_cast<int>(currentNode) - 1] = p;
        leftChildTemp[static_cast<int>(currentNode) - 1] = nextUnusedNode;
        rightChildTemp[static_cast<int>(currentNode) - 1] =
            static_cast<double>(nextUnusedNode) + 1.0;
        cc = upperBoundsTemp[static_cast<int>(currentNode) - 1];
        temp[0] = cc;
        temp_tmp = upperBoundsTemp[(static_cast<int>(currentNode) +
                                    upperBoundsTemp.size(0)) -
                                   1];
        temp[1] = temp_tmp;
        b_temp_tmp = upperBoundsTemp[(static_cast<int>(currentNode) +
                                      upperBoundsTemp.size(0) * 2) -
                                     1];
        temp[2] = b_temp_tmp;
        upperBoundsTemp[static_cast<int>(static_cast<double>(nextUnusedNode) +
                                         1.0) -
                        1] = cc;
        upperBoundsTemp[(static_cast<int>(static_cast<double>(nextUnusedNode) +
                                          1.0) +
                         upperBoundsTemp.size(0)) -
                        1] = temp_tmp;
        upperBoundsTemp[(static_cast<int>(static_cast<double>(nextUnusedNode) +
                                          1.0) +
                         upperBoundsTemp.size(0) * 2) -
                        1] = b_temp_tmp;
        temp[idx - 1] = p;
        upperBoundsTemp[static_cast<int>(nextUnusedNode) - 1] = temp[0];
        cc = lowerBoundsTemp[static_cast<int>(currentNode) - 1];
        temp[0] = cc;
        upperBoundsTemp[(static_cast<int>(nextUnusedNode) +
                         upperBoundsTemp.size(0)) -
                        1] = temp[1];
        temp_tmp = lowerBoundsTemp[(static_cast<int>(currentNode) +
                                    lowerBoundsTemp.size(0)) -
                                   1];
        temp[1] = temp_tmp;
        upperBoundsTemp[(static_cast<int>(nextUnusedNode) +
                         upperBoundsTemp.size(0) * 2) -
                        1] = temp[2];
        b_temp_tmp = lowerBoundsTemp[(static_cast<int>(currentNode) +
                                      lowerBoundsTemp.size(0) * 2) -
                                     1];
        temp[2] = b_temp_tmp;
        lowerBoundsTemp[static_cast<int>(nextUnusedNode) - 1] = cc;
        lowerBoundsTemp[(static_cast<int>(nextUnusedNode) +
                         lowerBoundsTemp.size(0)) -
                        1] = temp_tmp;
        lowerBoundsTemp[(static_cast<int>(nextUnusedNode) +
                         lowerBoundsTemp.size(0) * 2) -
                        1] = b_temp_tmp;
        temp[idx - 1] = p;
        lowerBoundsTemp[static_cast<int>(nextUnusedNode)] = temp[0];
        lowerBoundsTemp[static_cast<int>(nextUnusedNode) +
                        lowerBoundsTemp.size(0)] = temp[1];
        lowerBoundsTemp[static_cast<int>(nextUnusedNode) +
                        lowerBoundsTemp.size(0) * 2] = temp[2];
        idxTemp1[static_cast<int>(currentNode) - 1].f1.set_size(1, 0);
        idxTemp1[static_cast<int>(nextUnusedNode) - 1].f1.set_size(1,
                                                                   unusedNodes);
        for (i = 0; i < unusedNodes; i++) {
          idxTemp1[static_cast<int>(nextUnusedNode) - 1].f1[i] = notnan[i];
        }
        if (unusedNodes + 1 > iidx.size(0)) {
          unusedNodes = 0;
          i = 0;
        } else {
          i = notnan.size(1);
        }
        loop_ub = i - unusedNodes;
        idxTemp1[static_cast<int>(nextUnusedNode)].f1.set_size(1, loop_ub);
        for (i = 0; i < loop_ub; i++) {
          idxTemp1[static_cast<int>(nextUnusedNode)].f1[i] =
              notnan[unusedNodes + i];
        }
        nextUnusedNode += 2U;
      }
      currentNode++;
    }
    unusedNodes = static_cast<int>(nextUnusedNode);
    idx = static_cast<int>(nextUnusedNode) - 1;
    tempDim.set_size(idx);
    std::memset(&tempIdx[0], 0, 7439U * sizeof(short));
    cc = 1.0;
    for (int c{0}; c <= unusedNodes - 2; c++) {
      tempDim[c] = static_cast<unsigned int>(idxTemp1[c].f1.size(1));
      if (idxTemp1[c].f1.size(1) > 0) {
        temp_tmp = (cc + static_cast<double>(idxTemp1[c].f1.size(1))) - 1.0;
        if (cc > temp_tmp) {
          i = -1;
          i1 = 0;
        } else {
          i = static_cast<int>(cc) - 2;
          i1 = static_cast<int>(temp_tmp);
        }
        loop_ub = (i1 - i) - 1;
        for (i1 = 0; i1 < loop_ub; i1++) {
          tempIdx[(i + i1) + 1] = static_cast<short>(idxTemp1[c].f1[i1]);
        }
        cc += static_cast<double>(tempDim[c]);
      }
    }
    b_this->InputData.set_size(7439, 3);
    for (i = 0; i < 22317; i++) {
      b_this->InputData[i] = inputData[i];
    }
    b_this->CutDim.set_size(idx, 1);
    for (i = 0; i < idx; i++) {
      b_this->CutDim[i] = cutDimTemp[i];
    }
    b_this->CutVal.set_size(idx, 1);
    for (i = 0; i < idx; i++) {
      b_this->CutVal[i] = cutValTemp[i];
    }
    b_this->LowerBounds.set_size(idx, 3);
    for (i = 0; i < 3; i++) {
      for (i1 = 0; i1 < idx; i1++) {
        b_this->LowerBounds[i1 + b_this->LowerBounds.size(0) * i] =
            lowerBoundsTemp[i1 + lowerBoundsTemp.size(0) * i];
      }
    }
    b_this->UpperBounds.set_size(idx, 3);
    for (i = 0; i < 3; i++) {
      for (i1 = 0; i1 < idx; i1++) {
        b_this->UpperBounds[i1 + b_this->UpperBounds.size(0) * i] =
            upperBoundsTemp[i1 + upperBoundsTemp.size(0) * i];
      }
    }
    b_this->IdxAll.set_size(7439);
    for (i = 0; i < 7439; i++) {
      b_this->IdxAll[i] = static_cast<unsigned int>(tempIdx[i]);
    }
    b_this->IdxDim.set_size(tempDim.size(0));
    loop_ub = tempDim.size(0);
    for (i = 0; i < loop_ub; i++) {
      b_this->IdxDim[i] = tempDim[i];
    }
    b_this->LeftChild.set_size(idx, 1);
    for (i = 0; i < idx; i++) {
      b_this->LeftChild[i] = leftChildTemp[i];
    }
    b_this->RightChild.set_size(idx, 1);
    for (i = 0; i < idx; i++) {
      b_this->RightChild[i] = rightChildTemp[i];
    }
    b_this->LeafNode.set_size(idx, 1);
    for (i = 0; i < idx; i++) {
      b_this->LeafNode[i] = leafNodeTemp[i];
    }
    b_this->NxNoNaN = nx;
    b_this->IsIndexed = true;
  }
}

void f_pointCloud::matlabCodegenDestructor()
{
  if (!matlabCodegenIsDeleted) {
    matlabCodegenIsDeleted = true;
  }
}

void e_pointCloud::matlabCodegenDestructor()
{
  if (!matlabCodegenIsDeleted) {
    matlabCodegenIsDeleted = true;
  }
}

void d_pointCloud::matlabCodegenDestructor()
{
  if (!matlabCodegenIsDeleted) {
    matlabCodegenIsDeleted = true;
  }
}

pointCloud *
pointCloud::b_select(const ::coder::array<bool, 2U> &varargin_1,
                     ::coder::vision::internal::codegen::Kdtree *iobj_0,
                     pointCloud *iobj_1)
{
  pointCloud *ptCloudOut;
  array<double, 2U> loc;
  array<double, 2U> nv;
  array<double, 2U> r;
  array<double, 1U> c_ii;
  array<double, 1U> intensity;
  array<int, 1U> ii;
  array<unsigned char, 2U> c;
  int b_ii;
  int idx;
  int nx;
  bool exitg1;
  nx = varargin_1.size(0) * varargin_1.size(1);
  idx = 0;
  ii.set_size(nx);
  b_ii = 0;
  exitg1 = false;
  while ((!exitg1) && (b_ii <= nx - 1)) {
    if (varargin_1[b_ii]) {
      idx++;
      ii[idx - 1] = b_ii + 1;
      if (idx >= nx) {
        exitg1 = true;
      } else {
        b_ii++;
      }
    } else {
      b_ii++;
    }
  }
  if (nx == 1) {
    if (idx == 0) {
      ii.set_size(0);
    }
  } else {
    if (idx < 1) {
      idx = 0;
    }
    ii.set_size(idx);
  }
  c_ii.set_size(ii.size(0));
  nx = ii.size(0);
  for (idx = 0; idx < nx; idx++) {
    c_ii[idx] = ii[idx];
  }
  subsetImpl(c_ii, loc, c, nv, intensity, r);
  ptCloudOut = iobj_1->init(loc, c, nv, intensity, iobj_0);
  ptCloudOut->RangeData.set_size(r.size(0), r.size(1));
  nx = r.size(0) * r.size(1);
  for (idx = 0; idx < nx; idx++) {
    ptCloudOut->RangeData[idx] = r[idx];
  }
  return ptCloudOut;
}

f_pointCloud::f_pointCloud()
{
  matlabCodegenIsDeleted = true;
}

e_pointCloud::e_pointCloud()
{
  matlabCodegenIsDeleted = true;
}

d_pointCloud::d_pointCloud()
{
  matlabCodegenIsDeleted = true;
}

b_pointCloud::b_pointCloud()
{
  matlabCodegenIsDeleted = true;
}

pointCloud::pointCloud()
{
  matlabCodegenIsDeleted = true;
}

c_pointCloud::c_pointCloud()
{
  matlabCodegenIsDeleted = true;
}

d_pointCloud::~d_pointCloud()
{
  matlabCodegenDestructor();
}

b_pointCloud::~b_pointCloud()
{
  matlabCodegenDestructor();
}

c_pointCloud::~c_pointCloud()
{
  matlabCodegenDestructor();
}

pointCloud::~pointCloud()
{
  matlabCodegenDestructor();
}

e_pointCloud::~e_pointCloud()
{
  matlabCodegenDestructor();
}

f_pointCloud::~f_pointCloud()
{
  matlabCodegenDestructor();
}

void pointCloud::extractValidPoints(::coder::array<double, 2U> &location,
                                    ::coder::array<unsigned char, 2U> &color,
                                    ::coder::array<double, 2U> &normals,
                                    ::coder::array<double, 1U> &intensity,
                                    ::coder::array<double, 2U> &rangeData,
                                    ::coder::array<bool, 1U> &indices) const
{
  array<int, 1U> r1;
  array<int, 1U> r2;
  array<int, 1U> r3;
  array<int, 1U> r4;
  array<int, 1U> r5;
  array<int, 1U> y;
  array<bool, 2U> r;
  array<bool, 2U> x;
  int vstride;
  int xoffset;
  x.set_size(Location.size(0), 3);
  vstride = Location.size(0) * 3;
  for (int j{0}; j < vstride; j++) {
    x[j] = std::isinf(Location[j]);
  }
  r.set_size(Location.size(0), 3);
  vstride = Location.size(0) * 3;
  for (int j{0}; j < vstride; j++) {
    r[j] = std::isnan(Location[j]);
  }
  vstride = x.size(0) * 3;
  x.set_size(x.size(0), 3);
  for (int j{0}; j < vstride; j++) {
    x[j] = ((!x[j]) && (!r[j]));
  }
  if (x.size(0) == 0) {
    y.set_size(0);
  } else {
    vstride = x.size(0);
    y.set_size(x.size(0));
    for (int j{0}; j < vstride; j++) {
      y[j] = x[j];
    }
    for (int k{0}; k < 2; k++) {
      xoffset = (k + 1) * vstride;
      for (int j{0}; j < vstride; j++) {
        y[j] = y[j] + x[xoffset + j];
      }
    }
  }
  indices.set_size(y.size(0));
  vstride = y.size(0);
  for (int j{0}; j < vstride; j++) {
    indices[j] = (y[j] == 3);
  }
  if (Location.size(0) != 0) {
    xoffset = indices.size(0) - 1;
    vstride = 0;
    for (int j{0}; j <= xoffset; j++) {
      if (indices[j]) {
        vstride++;
      }
    }
    r1.set_size(vstride);
    vstride = 0;
    for (int j{0}; j <= xoffset; j++) {
      if (indices[j]) {
        r1[vstride] = j + 1;
        vstride++;
      }
    }
    location.set_size(r1.size(0), 3);
    vstride = r1.size(0);
    for (int j{0}; j < 3; j++) {
      for (int k{0}; k < vstride; k++) {
        location[k + location.size(0) * j] =
            Location[(r1[k] + Location.size(0) * j) - 1];
      }
    }
  } else {
    location.set_size(0, 3);
  }
  if ((Color.size(0) != 0) && (Color.size(1) != 0)) {
    xoffset = indices.size(0) - 1;
    vstride = 0;
    for (int j{0}; j <= xoffset; j++) {
      if (indices[j]) {
        vstride++;
      }
    }
    r2.set_size(vstride);
    vstride = 0;
    for (int j{0}; j <= xoffset; j++) {
      if (indices[j]) {
        r2[vstride] = j + 1;
        vstride++;
      }
    }
    vstride = Color.size(1);
    color.set_size(r2.size(0), vstride);
    for (int j{0}; j < vstride; j++) {
      xoffset = r2.size(0);
      for (int k{0}; k < xoffset; k++) {
        color[k + color.size(0) * j] = Color[(r2[k] + Color.size(0) * j) - 1];
      }
    }
  } else {
    color.set_size(0, 0);
  }
  if ((Normal.size(0) != 0) && (Normal.size(1) != 0)) {
    xoffset = indices.size(0) - 1;
    vstride = 0;
    for (int j{0}; j <= xoffset; j++) {
      if (indices[j]) {
        vstride++;
      }
    }
    r3.set_size(vstride);
    vstride = 0;
    for (int j{0}; j <= xoffset; j++) {
      if (indices[j]) {
        r3[vstride] = j + 1;
        vstride++;
      }
    }
    vstride = Normal.size(1);
    normals.set_size(r3.size(0), vstride);
    for (int j{0}; j < vstride; j++) {
      xoffset = r3.size(0);
      for (int k{0}; k < xoffset; k++) {
        normals[k + normals.size(0) * j] =
            Normal[(r3[k] + Normal.size(0) * j) - 1];
      }
    }
  } else {
    normals.set_size(0, 0);
  }
  if (Intensity.size(0) != 0) {
    xoffset = indices.size(0) - 1;
    vstride = 0;
    for (int j{0}; j <= xoffset; j++) {
      if (indices[j]) {
        vstride++;
      }
    }
    r4.set_size(vstride);
    vstride = 0;
    for (int j{0}; j <= xoffset; j++) {
      if (indices[j]) {
        r4[vstride] = j + 1;
        vstride++;
      }
    }
    intensity.set_size(r4.size(0));
    vstride = r4.size(0);
    for (int j{0}; j < vstride; j++) {
      intensity[j] = Intensity[r4[j] - 1];
    }
  } else {
    intensity.set_size(0);
  }
  if ((RangeData.size(0) != 0) && (RangeData.size(1) != 0)) {
    xoffset = indices.size(0) - 1;
    vstride = 0;
    for (int j{0}; j <= xoffset; j++) {
      if (indices[j]) {
        vstride++;
      }
    }
    r5.set_size(vstride);
    vstride = 0;
    for (int j{0}; j <= xoffset; j++) {
      if (indices[j]) {
        r5[vstride] = j + 1;
        vstride++;
      }
    }
    vstride = RangeData.size(1);
    rangeData.set_size(r5.size(0), vstride);
    for (int j{0}; j < vstride; j++) {
      xoffset = r5.size(0);
      for (int k{0}; k < xoffset; k++) {
        rangeData[k + rangeData.size(0) * j] =
            RangeData[(r5[k] + RangeData.size(0) * j) - 1];
      }
    }
  } else {
    rangeData.set_size(0, 0);
  }
}

void c_pointCloud::findNeighborsInRadius(
    const double varargin_1[3], double varargin_2,
    ::coder::array<unsigned int, 1U> &indices)
{
  ::coder::vision::internal::codegen::b_Kdtree *b_this;
  array<double, 2U> X;
  array<double, 2U> allDists;
  array<double, 2U> cutDim;
  array<double, 2U> cutVal;
  array<double, 2U> features;
  array<double, 2U> leftChild;
  array<double, 2U> rightChild;
  array<double, 2U> upBounds;
  array<double, 1U> c_this;
  array<double, 1U> nodeStack;
  array<int, 2U> r;
  array<int, 1U> iidx;
  array<unsigned int, 1U> nodeIdxThis;
  array<bool, 2U> leafNode;
  array<bool, 2U> x;
  array<bool, 1U> b_isFinite;
  array<bool, 1U> r1;
  h_struct_T pq;
  double b_pRadIn[3];
  double pRadIn[3];
  int nxin;
  nxin = Location.size(0);
  if (nxin < 500) {
    double powRadius;
    int k;
    int nxout;
    features.set_size(Location.size(0), 3);
    nxin = Location.size(0) * 3;
    for (k = 0; k < nxin; k++) {
      features[k] = Location[k];
    }
    nxin = Location.size(0);
    allDists.set_size(1, nxin);
    for (k = 0; k < nxin; k++) {
      allDists[k] = 0.0;
    }
    for (nxout = 0; nxout < nxin; nxout++) {
      powRadius = varargin_1[0] - features[nxout];
      pRadIn[0] = powRadius * powRadius;
      powRadius = varargin_1[1] - features[nxout + features.size(0)];
      pRadIn[1] = powRadius * powRadius;
      powRadius = varargin_1[2] - features[nxout + features.size(0) * 2];
      allDists[nxout] = (pRadIn[0] + pRadIn[1]) + powRadius * powRadius;
    }
    powRadius = varargin_2 * varargin_2;
    x.set_size(1, allDists.size(1));
    nxin = allDists.size(1);
    for (k = 0; k < nxin; k++) {
      x[k] = (allDists[k] <= powRadius);
    }
    eml_find(x, r);
    indices.set_size(r.size(1));
    nxin = r.size(1);
    for (k = 0; k < nxin; k++) {
      nxout = r[k];
      if (nxout < 0) {
        nxout = 0;
      }
      indices[k] = static_cast<unsigned int>(nxout);
    }
    if (indices.size(0) != 0) {
      nodeStack.set_size(indices.size(0));
      nxin = indices.size(0);
      for (k = 0; k < nxin; k++) {
        nodeStack[k] = allDists[static_cast<int>(indices[k]) - 1];
      }
      b_isFinite.set_size(nodeStack.size(0));
      nxin = nodeStack.size(0);
      for (k = 0; k < nxin; k++) {
        b_isFinite[k] = std::isinf(nodeStack[k]);
      }
      r1.set_size(nodeStack.size(0));
      nxin = nodeStack.size(0);
      for (k = 0; k < nxin; k++) {
        r1[k] = std::isnan(nodeStack[k]);
      }
      nxin = b_isFinite.size(0);
      for (k = 0; k < nxin; k++) {
        b_isFinite[k] = ((!b_isFinite[k]) && (!r1[k]));
      }
      nxin = b_isFinite.size(0) - 1;
      nxout = 0;
      k = 0;
      for (int i{0}; i <= nxin; i++) {
        if (b_isFinite[i]) {
          nxout++;
          indices[k] = indices[i];
          k++;
        }
      }
      indices.set_size(nxout);
    }
  } else {
    double powRadius;
    int k;
    bool exitg1;
    bool y;
    if (!b_Kdtree->IsIndexed) {
      b_this = b_Kdtree;
      features.set_size(Location.size(0), 3);
      nxin = Location.size(0) * 3;
      for (k = 0; k < nxin; k++) {
        features[k] = Location[k];
      }
      b_this->buildIndex(features);
      b_this->IsIndexed = true;
    }
    b_this = b_Kdtree;
    X.set_size(b_this->InputData.size(0), b_this->InputData.size(1));
    nxin = b_this->InputData.size(0) * b_this->InputData.size(1);
    for (k = 0; k < nxin; k++) {
      X[k] = b_this->InputData[k];
    }
    powRadius = varargin_2 * varargin_2;
    x.set_size(1, 3);
    x[0] = std::isnan(varargin_1[0]);
    x[1] = std::isnan(varargin_1[1]);
    x[2] = std::isnan(varargin_1[2]);
    y = false;
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k <= 2)) {
      if (x[k]) {
        y = true;
        exitg1 = true;
      } else {
        k++;
      }
    }
    if (y || (X.size(0) == 0)) {
      indices.set_size(0);
    } else {
      double startNode;
      bool guard1{false};
      cutDim.set_size(b_this->CutDim.size(0), b_this->CutDim.size(1));
      nxin = b_this->CutDim.size(0) * b_this->CutDim.size(1);
      for (k = 0; k < nxin; k++) {
        cutDim[k] = b_this->CutDim[k];
      }
      cutVal.set_size(b_this->CutVal.size(0), b_this->CutVal.size(1));
      nxin = b_this->CutVal.size(0) * b_this->CutVal.size(1);
      for (k = 0; k < nxin; k++) {
        cutVal[k] = b_this->CutVal[k];
      }
      leafNode.set_size(b_this->LeafNode.size(0), b_this->LeafNode.size(1));
      nxin = b_this->LeafNode.size(0) * b_this->LeafNode.size(1);
      for (k = 0; k < nxin; k++) {
        leafNode[k] = b_this->LeafNode[k];
      }
      leftChild.set_size(b_this->LeftChild.size(0), b_this->LeftChild.size(1));
      nxin = b_this->LeftChild.size(0) * b_this->LeftChild.size(1);
      for (k = 0; k < nxin; k++) {
        leftChild[k] = b_this->LeftChild[k];
      }
      rightChild.set_size(b_this->RightChild.size(0),
                          b_this->RightChild.size(1));
      nxin = b_this->RightChild.size(0) * b_this->RightChild.size(1);
      for (k = 0; k < nxin; k++) {
        rightChild[k] = b_this->RightChild[k];
      }
      startNode = 1.0;
      while (!leafNode[static_cast<int>(startNode) - 1]) {
        if (varargin_1[static_cast<int>(
                           cutDim[static_cast<int>(startNode) - 1]) -
                       1] <= cutVal[static_cast<int>(startNode) - 1]) {
          startNode = leftChild[static_cast<int>(startNode) - 1];
        } else {
          startNode = rightChild[static_cast<int>(startNode) - 1];
        }
      }
      pq.D.set_size(0);
      pq.b_I.set_size(0);
      vision::internal::codegen::Kdtree::getNodeFromArray(
          b_this->IdxAll, b_this->IdxDim, startNode, nodeIdxThis);
      vision::internal::codegen::Kdtree::searchNodeRadius(
          X, varargin_1, nodeIdxThis, powRadius, &pq);
      guard1 = false;
      if (pq.D.size(0) != 0) {
        nxin = b_this->LowerBounds.size(1);
        allDists.set_size(1, nxin);
        for (k = 0; k < nxin; k++) {
          allDists[k] = b_this->LowerBounds[(static_cast<int>(startNode) +
                                             b_this->LowerBounds.size(0) * k) -
                                            1];
        }
        nxin = b_this->UpperBounds.size(1);
        upBounds.set_size(1, nxin);
        for (k = 0; k < nxin; k++) {
          upBounds[k] = b_this->UpperBounds[(static_cast<int>(startNode) +
                                             b_this->UpperBounds.size(0) * k) -
                                            1];
        }
        if (allDists.size(1) == 3) {
          b_pRadIn[0] = varargin_1[0] - allDists[0];
          b_pRadIn[1] = varargin_1[1] - allDists[1];
          b_pRadIn[2] = varargin_1[2] - allDists[2];
        } else {
          minus(b_pRadIn, varargin_1, allDists);
        }
        if (upBounds.size(1) == 3) {
          pRadIn[0] = varargin_1[0] - upBounds[0];
          pRadIn[1] = varargin_1[1] - upBounds[1];
          pRadIn[2] = varargin_1[2] - upBounds[2];
        } else {
          minus(pRadIn, varargin_1, upBounds);
        }
        b_pRadIn[0] *= b_pRadIn[0];
        b_pRadIn[1] *= b_pRadIn[1];
        b_pRadIn[2] *= b_pRadIn[2];
        if (internal::minimum(b_pRadIn) <= powRadius) {
          guard1 = true;
        } else {
          pRadIn[0] *= pRadIn[0];
          pRadIn[1] *= pRadIn[1];
          pRadIn[2] *= pRadIn[2];
          if (internal::minimum(pRadIn) <= powRadius) {
            guard1 = true;
          }
        }
      } else {
        guard1 = true;
      }
      if (guard1) {
        nodeStack.set_size(1);
        nodeStack[0] = 1.0;
        while (nodeStack.size(0) != 0) {
          double currentNode;
          int nxout;
          currentNode = nodeStack[0];
          nxin = nodeStack.size(0);
          nxout = nodeStack.size(0) - 1;
          for (k = 0; k < nxout; k++) {
            nodeStack[k] = nodeStack[k + 1];
          }
          if (nxout < 1) {
            nxout = 0;
          } else {
            nxout = nxin - 1;
          }
          nodeStack.set_size(nxout);
          nxin = b_this->LowerBounds.size(1);
          allDists.set_size(1, nxin);
          for (k = 0; k < nxin; k++) {
            allDists[k] =
                b_this->LowerBounds[(static_cast<int>(currentNode) +
                                     b_this->LowerBounds.size(0) * k) -
                                    1];
          }
          nxin = b_this->UpperBounds.size(1);
          upBounds.set_size(1, nxin);
          for (k = 0; k < nxin; k++) {
            upBounds[k] =
                b_this->UpperBounds[(static_cast<int>(currentNode) +
                                     b_this->UpperBounds.size(0) * k) -
                                    1];
          }
          if (vision::internal::codegen::Kdtree::boundsOverlapBall(
                  varargin_1, allDists, upBounds, powRadius,
                  static_cast<double>(X.size(1)))) {
            if (!b_this->LeafNode[static_cast<int>(currentNode) - 1]) {
              if (varargin_1[static_cast<int>(
                                 b_this->CutDim[static_cast<int>(currentNode) -
                                                1]) -
                             1] <=
                  b_this->CutVal[static_cast<int>(currentNode) - 1]) {
                c_this.set_size(nodeStack.size(0) + 2);
                c_this[0] =
                    b_this->LeftChild[static_cast<int>(currentNode) - 1];
                c_this[1] =
                    b_this->RightChild[static_cast<int>(currentNode) - 1];
                nxin = nodeStack.size(0);
                for (k = 0; k < nxin; k++) {
                  c_this[k + 2] = nodeStack[k];
                }
                nodeStack.set_size(c_this.size(0));
                nxin = c_this.size(0);
                for (k = 0; k < nxin; k++) {
                  nodeStack[k] = c_this[k];
                }
              } else {
                c_this.set_size(nodeStack.size(0) + 2);
                c_this[0] =
                    b_this->RightChild[static_cast<int>(currentNode) - 1];
                c_this[1] =
                    b_this->LeftChild[static_cast<int>(currentNode) - 1];
                nxin = nodeStack.size(0);
                for (k = 0; k < nxin; k++) {
                  c_this[k + 2] = nodeStack[k];
                }
                nodeStack.set_size(c_this.size(0));
                nxin = c_this.size(0);
                for (k = 0; k < nxin; k++) {
                  nodeStack[k] = c_this[k];
                }
              }
            } else if (currentNode != startNode) {
              vision::internal::codegen::Kdtree::getNodeFromArray(
                  b_this->IdxAll, b_this->IdxDim, currentNode, nodeIdxThis);
              vision::internal::codegen::Kdtree::searchNodeRadius(
                  X, varargin_1, nodeIdxThis, powRadius, &pq);
            }
          }
        }
      }
      internal::sort(pq.D, iidx);
      nodeIdxThis.set_size(iidx.size(0));
      nxin = iidx.size(0);
      for (k = 0; k < nxin; k++) {
        nodeIdxThis[k] = pq.b_I[iidx[k] - 1];
      }
      pq.b_I.set_size(nodeIdxThis.size(0));
      nxin = nodeIdxThis.size(0);
      for (k = 0; k < nxin; k++) {
        pq.b_I[k] = nodeIdxThis[k];
      }
      indices.set_size(pq.b_I.size(0));
      nxin = pq.b_I.size(0);
      for (k = 0; k < nxin; k++) {
        indices[k] = pq.b_I[k];
      }
    }
  }
}

void pointCloud::get_XLimits(::coder::array<double, 2U> &xlim)
{
  array<double, 1U> b_this;
  array<double, 1U> c_this;
  array<int, 1U> r;
  array<int, 1U> y;
  array<bool, 2U> x;
  array<bool, 1U> tf;
  int vstride;
  int xoffset;
  if ((XLimitsInternal.size(0) == 0) || (XLimitsInternal.size(1) == 0)) {
    x.set_size(Location.size(0), 3);
    vstride = Location.size(0) * 3;
    for (xoffset = 0; xoffset < vstride; xoffset++) {
      x[xoffset] = std::isnan(Location[xoffset]);
    }
    vstride = x.size(0) * 3;
    x.set_size(x.size(0), 3);
    for (xoffset = 0; xoffset < vstride; xoffset++) {
      x[xoffset] = !x[xoffset];
    }
    if (x.size(0) == 0) {
      y.set_size(0);
    } else {
      vstride = x.size(0);
      y.set_size(x.size(0));
      for (int j{0}; j < vstride; j++) {
        y[j] = x[j];
      }
      for (int k{0}; k < 2; k++) {
        xoffset = (k + 1) * vstride;
        for (int j{0}; j < vstride; j++) {
          y[j] = y[j] + x[xoffset + j];
        }
      }
    }
    tf.set_size(y.size(0));
    vstride = y.size(0);
    for (xoffset = 0; xoffset < vstride; xoffset++) {
      tf[xoffset] = (y[xoffset] == 3);
    }
    vstride = 0;
    xoffset = tf.size(0);
    for (int k{0}; k < xoffset; k++) {
      if (tf[k]) {
        vstride++;
      }
    }
    if (vstride != 0) {
      xoffset = tf.size(0) - 1;
      vstride = 0;
      for (int j{0}; j <= xoffset; j++) {
        if (tf[j]) {
          vstride++;
        }
      }
      r.set_size(vstride);
      vstride = 0;
      for (int j{0}; j <= xoffset; j++) {
        if (tf[j]) {
          r[vstride] = j + 1;
          vstride++;
        }
      }
      b_this.set_size(r.size(0));
      vstride = r.size(0);
      for (xoffset = 0; xoffset < vstride; xoffset++) {
        b_this[xoffset] = Location[r[xoffset] - 1];
      }
      c_this.set_size(r.size(0));
      vstride = r.size(0);
      for (xoffset = 0; xoffset < vstride; xoffset++) {
        c_this[xoffset] = Location[r[xoffset] - 1];
      }
      XLimitsInternal.set_size(1, 2);
      XLimitsInternal[0] = internal::minimum(b_this);
      XLimitsInternal[1] = internal::maximum(c_this);
    } else {
      XLimitsInternal.set_size(0, 2);
    }
  }
  xlim.set_size(XLimitsInternal.size(0), XLimitsInternal.size(1));
  vstride = XLimitsInternal.size(0) * XLimitsInternal.size(1);
  for (xoffset = 0; xoffset < vstride; xoffset++) {
    xlim[xoffset] = XLimitsInternal[xoffset];
  }
}

void pointCloud::get_YLimits(::coder::array<double, 2U> &ylim)
{
  array<double, 1U> b_this;
  array<double, 1U> c_this;
  array<int, 1U> r;
  array<int, 1U> y;
  array<bool, 2U> x;
  array<bool, 1U> tf;
  int vstride;
  int xoffset;
  if ((YLimitsInternal.size(0) == 0) || (YLimitsInternal.size(1) == 0)) {
    x.set_size(Location.size(0), 3);
    vstride = Location.size(0) * 3;
    for (xoffset = 0; xoffset < vstride; xoffset++) {
      x[xoffset] = std::isnan(Location[xoffset]);
    }
    vstride = x.size(0) * 3;
    x.set_size(x.size(0), 3);
    for (xoffset = 0; xoffset < vstride; xoffset++) {
      x[xoffset] = !x[xoffset];
    }
    if (x.size(0) == 0) {
      y.set_size(0);
    } else {
      vstride = x.size(0);
      y.set_size(x.size(0));
      for (int j{0}; j < vstride; j++) {
        y[j] = x[j];
      }
      for (int k{0}; k < 2; k++) {
        xoffset = (k + 1) * vstride;
        for (int j{0}; j < vstride; j++) {
          y[j] = y[j] + x[xoffset + j];
        }
      }
    }
    tf.set_size(y.size(0));
    vstride = y.size(0);
    for (xoffset = 0; xoffset < vstride; xoffset++) {
      tf[xoffset] = (y[xoffset] == 3);
    }
    vstride = 0;
    xoffset = tf.size(0);
    for (int k{0}; k < xoffset; k++) {
      if (tf[k]) {
        vstride++;
      }
    }
    if (vstride != 0) {
      xoffset = tf.size(0) - 1;
      vstride = 0;
      for (int j{0}; j <= xoffset; j++) {
        if (tf[j]) {
          vstride++;
        }
      }
      r.set_size(vstride);
      vstride = 0;
      for (int j{0}; j <= xoffset; j++) {
        if (tf[j]) {
          r[vstride] = j + 1;
          vstride++;
        }
      }
      b_this.set_size(r.size(0));
      vstride = r.size(0);
      for (xoffset = 0; xoffset < vstride; xoffset++) {
        b_this[xoffset] = Location[(r[xoffset] + Location.size(0)) - 1];
      }
      c_this.set_size(r.size(0));
      vstride = r.size(0);
      for (xoffset = 0; xoffset < vstride; xoffset++) {
        c_this[xoffset] = Location[(r[xoffset] + Location.size(0)) - 1];
      }
      YLimitsInternal.set_size(1, 2);
      YLimitsInternal[0] = internal::minimum(b_this);
      YLimitsInternal[1] = internal::maximum(c_this);
    } else {
      YLimitsInternal.set_size(0, 2);
    }
  }
  ylim.set_size(YLimitsInternal.size(0), YLimitsInternal.size(1));
  vstride = YLimitsInternal.size(0) * YLimitsInternal.size(1);
  for (xoffset = 0; xoffset < vstride; xoffset++) {
    ylim[xoffset] = YLimitsInternal[xoffset];
  }
}

void pointCloud::get_ZLimits(::coder::array<double, 2U> &zlim)
{
  array<double, 1U> b_this;
  array<double, 1U> c_this;
  array<int, 1U> r;
  array<int, 1U> y;
  array<bool, 2U> x;
  array<bool, 1U> tf;
  int vstride;
  int xoffset;
  if ((ZLimitsInternal.size(0) == 0) || (ZLimitsInternal.size(1) == 0)) {
    x.set_size(Location.size(0), 3);
    vstride = Location.size(0) * 3;
    for (xoffset = 0; xoffset < vstride; xoffset++) {
      x[xoffset] = std::isnan(Location[xoffset]);
    }
    vstride = x.size(0) * 3;
    x.set_size(x.size(0), 3);
    for (xoffset = 0; xoffset < vstride; xoffset++) {
      x[xoffset] = !x[xoffset];
    }
    if (x.size(0) == 0) {
      y.set_size(0);
    } else {
      vstride = x.size(0);
      y.set_size(x.size(0));
      for (int j{0}; j < vstride; j++) {
        y[j] = x[j];
      }
      for (int k{0}; k < 2; k++) {
        xoffset = (k + 1) * vstride;
        for (int j{0}; j < vstride; j++) {
          y[j] = y[j] + x[xoffset + j];
        }
      }
    }
    tf.set_size(y.size(0));
    vstride = y.size(0);
    for (xoffset = 0; xoffset < vstride; xoffset++) {
      tf[xoffset] = (y[xoffset] == 3);
    }
    vstride = 0;
    xoffset = tf.size(0);
    for (int k{0}; k < xoffset; k++) {
      if (tf[k]) {
        vstride++;
      }
    }
    if (vstride != 0) {
      xoffset = tf.size(0) - 1;
      vstride = 0;
      for (int j{0}; j <= xoffset; j++) {
        if (tf[j]) {
          vstride++;
        }
      }
      r.set_size(vstride);
      vstride = 0;
      for (int j{0}; j <= xoffset; j++) {
        if (tf[j]) {
          r[vstride] = j + 1;
          vstride++;
        }
      }
      b_this.set_size(r.size(0));
      vstride = r.size(0);
      for (xoffset = 0; xoffset < vstride; xoffset++) {
        b_this[xoffset] = Location[(r[xoffset] + Location.size(0) * 2) - 1];
      }
      c_this.set_size(r.size(0));
      vstride = r.size(0);
      for (xoffset = 0; xoffset < vstride; xoffset++) {
        c_this[xoffset] = Location[(r[xoffset] + Location.size(0) * 2) - 1];
      }
      ZLimitsInternal.set_size(1, 2);
      ZLimitsInternal[0] = internal::minimum(b_this);
      ZLimitsInternal[1] = internal::maximum(c_this);
    } else {
      ZLimitsInternal.set_size(0, 2);
    }
  }
  zlim.set_size(ZLimitsInternal.size(0), ZLimitsInternal.size(1));
  vstride = ZLimitsInternal.size(0) * ZLimitsInternal.size(1);
  for (xoffset = 0; xoffset < vstride; xoffset++) {
    zlim[xoffset] = ZLimitsInternal[xoffset];
  }
}

pointCloud *pointCloud::init(const ::coder::array<double, 2U> &varargin_1,
                             const ::coder::array<double, 1U> &varargin_3,
                             ::coder::vision::internal::codegen::Kdtree *iobj_0)
{
  pointCloud *this_;
  int loop_ub;
  this_ = this;
  this_->Location.set_size(varargin_1.size(0), 3);
  loop_ub = varargin_1.size(0) * 3;
  for (int i{0}; i < loop_ub; i++) {
    this_->Location[i] = varargin_1[i];
  }
  this_->Color.set_size(0, 0);
  this_->Normal.set_size(0, 0);
  this_->Intensity.set_size(varargin_3.size(0));
  loop_ub = varargin_3.size(0);
  for (int i{0}; i < loop_ub; i++) {
    this_->Intensity[i] = varargin_3[i];
  }
  pointclouds::internal::codegen::pc::pointCloudArray r;
  this_->RangeData.set_size(0, 0);
  this_->PointCloudArrayData.set_size(1, 1);
  this_->PointCloudArrayData[0] = r;
  this_->XLimitsInternal.set_size(0, 0);
  this_->YLimitsInternal.set_size(0, 0);
  this_->ZLimitsInternal.set_size(0, 0);
  this_->Kdtree = iobj_0;
  this_->matlabCodegenIsDeleted = false;
  return this_;
}

pointCloud *
pointCloud::init(const ::coder::array<double, 2U> &varargin_1,
                 const ::coder::array<unsigned char, 2U> &varargin_3,
                 const ::coder::array<double, 2U> &varargin_5,
                 const ::coder::array<double, 1U> &varargin_7,
                 ::coder::vision::internal::codegen::Kdtree *iobj_0)
{
  pointCloud *this_;
  int loop_ub;
  this_ = this;
  this_->Location.set_size(varargin_1.size(0), 3);
  loop_ub = varargin_1.size(0) * 3;
  for (int i{0}; i < loop_ub; i++) {
    this_->Location[i] = varargin_1[i];
  }
  this_->Color.set_size(varargin_3.size(0), varargin_3.size(1));
  loop_ub = varargin_3.size(0) * varargin_3.size(1);
  for (int i{0}; i < loop_ub; i++) {
    this_->Color[i] = varargin_3[i];
  }
  this_->Normal.set_size(varargin_5.size(0), varargin_5.size(1));
  loop_ub = varargin_5.size(0) * varargin_5.size(1);
  for (int i{0}; i < loop_ub; i++) {
    this_->Normal[i] = varargin_5[i];
  }
  this_->Intensity.set_size(varargin_7.size(0));
  loop_ub = varargin_7.size(0);
  for (int i{0}; i < loop_ub; i++) {
    this_->Intensity[i] = varargin_7[i];
  }
  pointclouds::internal::codegen::pc::pointCloudArray r;
  this_->RangeData.set_size(0, 0);
  this_->PointCloudArrayData.set_size(1, 1);
  this_->PointCloudArrayData[0] = r;
  this_->XLimitsInternal.set_size(0, 0);
  this_->YLimitsInternal.set_size(0, 0);
  this_->ZLimitsInternal.set_size(0, 0);
  this_->Kdtree = iobj_0;
  this_->matlabCodegenIsDeleted = false;
  return this_;
}

void b_pointCloud::matlabCodegenDestructor()
{
  if (!matlabCodegenIsDeleted) {
    matlabCodegenIsDeleted = true;
  }
}

void c_pointCloud::matlabCodegenDestructor()
{
  if (!matlabCodegenIsDeleted) {
    matlabCodegenIsDeleted = true;
  }
}

void pointCloud::matlabCodegenDestructor()
{
  if (!matlabCodegenIsDeleted) {
    matlabCodegenIsDeleted = true;
  }
}

void pointCloud::removeInvalidPoints(
    c_pointCloud *iobj_0, c_pointCloud **ptCloudOut,
    ::coder::array<double, 1U> &indicesOut) const
{
  array<double, 2U> location;
  array<double, 2U> rangeData;
  array<double, 2U> tempNV;
  array<double, 1U> tempI;
  array<int, 1U> r1;
  array<unsigned char, 2U> C_;
  array<bool, 1U> indices;
  int loop_ub;
  extractValidPoints(location, C_, tempNV, tempI, rangeData, indices);
  *ptCloudOut = iobj_0;
  iobj_0->Location.set_size(location.size(0), 3);
  loop_ub = location.size(0) * 3;
  for (int i{0}; i < loop_ub; i++) {
    iobj_0->Location[i] = location[i];
  }
  iobj_0->Color.set_size(C_.size(0), C_.size(1));
  loop_ub = C_.size(0) * C_.size(1);
  for (int i{0}; i < loop_ub; i++) {
    iobj_0->Color[i] = C_[i];
  }
  iobj_0->Normal.set_size(tempNV.size(0), tempNV.size(1));
  loop_ub = tempNV.size(0) * tempNV.size(1);
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
  iobj_0->RangeData.set_size(rangeData.size(0), rangeData.size(1));
  loop_ub = rangeData.size(0) * rangeData.size(1);
  for (int i{0}; i < loop_ub; i++) {
    iobj_0->RangeData[i] = rangeData[i];
  }
  b_eml_find(indices, r1);
  indicesOut.set_size(r1.size(0));
  loop_ub = r1.size(0);
  for (int i{0}; i < loop_ub; i++) {
    indicesOut[i] = r1[i];
  }
}

void pointCloud::subsetImpl(const ::coder::array<double, 1U> &indices,
                            ::coder::array<double, 2U> &loc,
                            ::coder::array<unsigned char, 2U> &c,
                            ::coder::array<double, 2U> &nv,
                            ::coder::array<double, 1U> &intensity,
                            ::coder::array<double, 2U> &r) const
{
  int b_loop_ub;
  int loop_ub;
  if (Location.size(0) != 0) {
    loc.set_size(indices.size(0), 3);
    loop_ub = indices.size(0);
    for (int i{0}; i < 3; i++) {
      for (int i1{0}; i1 < loop_ub; i1++) {
        loc[i1 + loc.size(0) * i] =
            Location[(static_cast<int>(indices[i1]) + Location.size(0) * i) -
                     1];
      }
    }
  } else {
    loc.set_size(0, 3);
  }
  if ((Color.size(0) != 0) && (Color.size(1) != 0)) {
    loop_ub = Color.size(1);
    c.set_size(indices.size(0), loop_ub);
    for (int i{0}; i < loop_ub; i++) {
      b_loop_ub = indices.size(0);
      for (int i1{0}; i1 < b_loop_ub; i1++) {
        c[i1 + c.size(0) * i] =
            Color[(static_cast<int>(indices[i1]) + Color.size(0) * i) - 1];
      }
    }
  } else {
    c.set_size(0, 0);
  }
  if ((Normal.size(0) != 0) && (Normal.size(1) != 0)) {
    loop_ub = Normal.size(1);
    nv.set_size(indices.size(0), loop_ub);
    for (int i{0}; i < loop_ub; i++) {
      b_loop_ub = indices.size(0);
      for (int i1{0}; i1 < b_loop_ub; i1++) {
        nv[i1 + nv.size(0) * i] =
            Normal[(static_cast<int>(indices[i1]) + Normal.size(0) * i) - 1];
      }
    }
  } else {
    nv.set_size(0, 0);
  }
  if (Intensity.size(0) != 0) {
    intensity.set_size(indices.size(0));
    loop_ub = indices.size(0);
    for (int i{0}; i < loop_ub; i++) {
      intensity[i] = Intensity[static_cast<int>(indices[i]) - 1];
    }
  } else {
    intensity.set_size(0);
  }
  if ((RangeData.size(0) != 0) && (RangeData.size(1) != 0)) {
    loop_ub = RangeData.size(1);
    r.set_size(indices.size(0), loop_ub);
    for (int i{0}; i < loop_ub; i++) {
      b_loop_ub = indices.size(0);
      for (int i1{0}; i1 < b_loop_ub; i1++) {
        r[i1 + r.size(0) * i] =
            RangeData[(static_cast<int>(indices[i1]) + RangeData.size(0) * i) -
                      1];
      }
    }
  } else {
    r.set_size(0, 0);
  }
}

void e_pointCloud::surfaceNormalImpl(::coder::array<float, 2U> &normals)
{
  static float loc[22317];
  vision::internal::codegen::c_Kdtree *b_this;
  array<double, 2U> cutDim;
  array<double, 2U> cutVal;
  array<double, 2U> leftChild;
  array<double, 2U> lowBounds;
  array<double, 2U> rightChild;
  array<double, 2U> upBounds;
  array<double, 1U> c_this;
  array<double, 1U> nodeStack;
  array<float, 2U> X;
  array<unsigned int, 2U> indices;
  array<int, 2U> noNanCol;
  array<unsigned int, 1U> nodeIdxThis;
  array<int, 1U> r1;
  array<bool, 2U> leafNode;
  i_struct_T r;
  float b_pRadIn[3];
  float pRadIn[3];
  unsigned int valid[7439];
  int nxout;
  int yk;
  bool bv[22317];
  bool wasNaNY[7439];
  for (nxout = 0; nxout < 22317; nxout++) {
    loc[nxout] = Location[nxout];
  }
  buildKdtree();
  b_this = b_Kdtree;
  X.set_size(b_this->InputData.size(0), b_this->InputData.size(1));
  yk = b_this->InputData.size(0) * b_this->InputData.size(1);
  for (nxout = 0; nxout < yk; nxout++) {
    X[nxout] = b_this->InputData[nxout];
  }
  for (nxout = 0; nxout < 22317; nxout++) {
    bv[nxout] = std::isnan(loc[nxout]);
  }
  b_any(bv, wasNaNY);
  yk = static_cast<int>(std::fmin(6.0, static_cast<double>(X.size(0))));
  if (yk == 0) {
    indices.set_size(0, 7439);
    std::memset(&valid[0], 0, 7439U * sizeof(unsigned int));
  } else {
    double startNode;
    int numNN;
    startNode = b_this->NxNoNaN;
    if ((!std::isnan(startNode)) && (yk > startNode)) {
      numNN = static_cast<int>(startNode);
    } else {
      numNN = yk;
    }
    if (numNN > 0) {
      indices.set_size(yk, 7439);
      yk *= 7439;
      for (nxout = 0; nxout < yk; nxout++) {
        indices[nxout] = 0U;
      }
      std::memset(&valid[0], 0, 7439U * sizeof(unsigned int));
      noNanCol.set_size(1, numNN);
      noNanCol[0] = 1;
      yk = 1;
      for (int k{2}; k <= numNN; k++) {
        yk++;
        noNanCol[k - 1] = yk;
      }
      for (int j{0}; j < 7439; j++) {
        if (!wasNaNY[j]) {
          float b_loc[3];
          float b_loc_tmp;
          float loc_tmp;
          bool ballIsWithinBounds;
          cutDim.set_size(b_this->CutDim.size(0), b_this->CutDim.size(1));
          yk = b_this->CutDim.size(0) * b_this->CutDim.size(1);
          for (nxout = 0; nxout < yk; nxout++) {
            cutDim[nxout] = b_this->CutDim[nxout];
          }
          cutVal.set_size(b_this->CutVal.size(0), b_this->CutVal.size(1));
          yk = b_this->CutVal.size(0) * b_this->CutVal.size(1);
          for (nxout = 0; nxout < yk; nxout++) {
            cutVal[nxout] = b_this->CutVal[nxout];
          }
          leafNode.set_size(b_this->LeafNode.size(0), b_this->LeafNode.size(1));
          yk = b_this->LeafNode.size(0) * b_this->LeafNode.size(1);
          for (nxout = 0; nxout < yk; nxout++) {
            leafNode[nxout] = b_this->LeafNode[nxout];
          }
          leftChild.set_size(b_this->LeftChild.size(0),
                             b_this->LeftChild.size(1));
          yk = b_this->LeftChild.size(0) * b_this->LeftChild.size(1);
          for (nxout = 0; nxout < yk; nxout++) {
            leftChild[nxout] = b_this->LeftChild[nxout];
          }
          rightChild.set_size(b_this->RightChild.size(0),
                              b_this->RightChild.size(1));
          yk = b_this->RightChild.size(0) * b_this->RightChild.size(1);
          for (nxout = 0; nxout < yk; nxout++) {
            rightChild[nxout] = b_this->RightChild[nxout];
          }
          startNode = 1.0;
          while (!leafNode[static_cast<int>(startNode) - 1]) {
            if (loc[j + 7439 * (static_cast<int>(
                                    cutDim[static_cast<int>(startNode) - 1]) -
                                1)] <=
                cutVal[static_cast<int>(startNode) - 1]) {
              startNode = leftChild[static_cast<int>(startNode) - 1];
            } else {
              startNode = rightChild[static_cast<int>(startNode) - 1];
            }
          }
          vision::internal::codegen::Kdtree::getNodeFromArray(
              b_this->IdxAll, b_this->IdxDim, startNode, nodeIdxThis);
          r.D.set_size(0);
          r.b_I.set_size(0);
          b_loc[0] = loc[j];
          loc_tmp = loc[j + 7439];
          b_loc[1] = loc_tmp;
          b_loc_tmp = loc[j + 14878];
          b_loc[2] = b_loc_tmp;
          vision::internal::codegen::Kdtree::searchNode(X, b_loc, nodeIdxThis,
                                                        numNN, &r);
          if (r.D.size(0) != 0) {
            yk = b_this->LowerBounds.size(1);
            lowBounds.set_size(1, yk);
            for (nxout = 0; nxout < yk; nxout++) {
              lowBounds[nxout] =
                  b_this->LowerBounds[(static_cast<int>(startNode) +
                                       b_this->LowerBounds.size(0) * nxout) -
                                      1];
            }
            yk = b_this->UpperBounds.size(1);
            upBounds.set_size(1, yk);
            for (nxout = 0; nxout < yk; nxout++) {
              upBounds[nxout] =
                  b_this->UpperBounds[(static_cast<int>(startNode) +
                                       b_this->UpperBounds.size(0) * nxout) -
                                      1];
            }
            if (lowBounds.size(1) == 3) {
              pRadIn[0] = loc[j] - static_cast<float>(lowBounds[0]);
              pRadIn[1] = loc_tmp - static_cast<float>(lowBounds[1]);
              pRadIn[2] = b_loc_tmp - static_cast<float>(lowBounds[2]);
            } else {
              binary_expand_op(pRadIn, loc, j, lowBounds);
            }
            if (upBounds.size(1) == 3) {
              b_pRadIn[0] = loc[j] - static_cast<float>(upBounds[0]);
              b_pRadIn[1] = loc_tmp - static_cast<float>(upBounds[1]);
              b_pRadIn[2] = b_loc_tmp - static_cast<float>(upBounds[2]);
            } else {
              binary_expand_op(b_pRadIn, loc, j, upBounds);
            }
            b_loc[0] = pRadIn[0] * pRadIn[0];
            b_loc[1] = pRadIn[1] * pRadIn[1];
            b_loc[2] = pRadIn[2] * pRadIn[2];
            if (internal::minimum(b_loc) <= r.D[r.D.size(0) - 1]) {
              ballIsWithinBounds = false;
            } else {
              b_loc[0] = b_pRadIn[0] * b_pRadIn[0];
              b_loc[1] = b_pRadIn[1] * b_pRadIn[1];
              b_loc[2] = b_pRadIn[2] * b_pRadIn[2];
              if (internal::minimum(b_loc) <= r.D[r.D.size(0) - 1]) {
                ballIsWithinBounds = false;
              } else {
                ballIsWithinBounds = true;
              }
            }
          } else {
            ballIsWithinBounds = false;
          }
          if ((r.D.size(0) != numNN) || (!ballIsWithinBounds)) {
            nodeStack.set_size(1);
            nodeStack[0] = 1.0;
            while (nodeStack.size(0) != 0) {
              double currentNode;
              float sumDist;
              bool exitg1;
              currentNode = nodeStack[0];
              yk = nodeStack.size(0);
              nxout = nodeStack.size(0) - 1;
              for (int k{0}; k < nxout; k++) {
                nodeStack[k] = nodeStack[k + 1];
              }
              if (nxout < 1) {
                nxout = 0;
              } else {
                nxout = yk - 1;
              }
              nodeStack.set_size(nxout);
              yk = b_this->LowerBounds.size(1);
              lowBounds.set_size(1, yk);
              for (nxout = 0; nxout < yk; nxout++) {
                lowBounds[nxout] =
                    b_this->LowerBounds[(static_cast<int>(currentNode) +
                                         b_this->LowerBounds.size(0) * nxout) -
                                        1];
              }
              yk = b_this->UpperBounds.size(1);
              upBounds.set_size(1, yk);
              for (nxout = 0; nxout < yk; nxout++) {
                upBounds[nxout] =
                    b_this->UpperBounds[(static_cast<int>(currentNode) +
                                         b_this->UpperBounds.size(0) * nxout) -
                                        1];
              }
              ballIsWithinBounds = true;
              sumDist = 0.0F;
              yk = 0;
              exitg1 = false;
              while ((!exitg1) && (yk <= X.size(1) - 1)) {
                float c_pRadIn;
                c_pRadIn = loc[j + 7439 * yk];
                if (c_pRadIn < lowBounds[yk]) {
                  c_pRadIn -= static_cast<float>(lowBounds[yk]);
                  sumDist += c_pRadIn * c_pRadIn;
                } else if (c_pRadIn > upBounds[yk]) {
                  c_pRadIn -= static_cast<float>(upBounds[yk]);
                  sumDist += c_pRadIn * c_pRadIn;
                }
                if (sumDist > r.D[r.D.size(0) - 1]) {
                  ballIsWithinBounds = false;
                  exitg1 = true;
                } else {
                  yk++;
                }
              }
              if ((r.D.size(0) < numNN) || ballIsWithinBounds) {
                if (!b_this->LeafNode[static_cast<int>(currentNode) - 1]) {
                  nxout =
                      static_cast<int>(
                          b_this->CutDim[static_cast<int>(currentNode) - 1]) -
                      1;
                  if (loc[j + 7439 * nxout] <=
                      b_this->CutVal[static_cast<int>(currentNode) - 1]) {
                    c_this.set_size(nodeStack.size(0) + 2);
                    c_this[0] =
                        b_this->LeftChild[static_cast<int>(currentNode) - 1];
                    c_this[1] =
                        b_this->RightChild[static_cast<int>(currentNode) - 1];
                    yk = nodeStack.size(0);
                    for (nxout = 0; nxout < yk; nxout++) {
                      c_this[nxout + 2] = nodeStack[nxout];
                    }
                    nodeStack.set_size(c_this.size(0));
                    yk = c_this.size(0);
                    for (nxout = 0; nxout < yk; nxout++) {
                      nodeStack[nxout] = c_this[nxout];
                    }
                  } else {
                    c_this.set_size(nodeStack.size(0) + 2);
                    c_this[0] =
                        b_this->RightChild[static_cast<int>(currentNode) - 1];
                    c_this[1] =
                        b_this->LeftChild[static_cast<int>(currentNode) - 1];
                    yk = nodeStack.size(0);
                    for (nxout = 0; nxout < yk; nxout++) {
                      c_this[nxout + 2] = nodeStack[nxout];
                    }
                    nodeStack.set_size(c_this.size(0));
                    yk = c_this.size(0);
                    for (nxout = 0; nxout < yk; nxout++) {
                      nodeStack[nxout] = c_this[nxout];
                    }
                  }
                } else if (currentNode != startNode) {
                  vision::internal::codegen::Kdtree::getNodeFromArray(
                      b_this->IdxAll, b_this->IdxDim, currentNode, nodeIdxThis);
                  b_loc[0] = loc[j];
                  b_loc[1] = loc_tmp;
                  b_loc[2] = b_loc_tmp;
                  vision::internal::codegen::Kdtree::searchNode(
                      X, b_loc, nodeIdxThis, numNN, &r);
                }
              }
            }
          }
          yk = noNanCol.size(1);
          r1.set_size(noNanCol.size(1));
          for (nxout = 0; nxout < yk; nxout++) {
            r1[nxout] = noNanCol[nxout];
          }
          for (nxout = 0; nxout < yk; nxout++) {
            indices[(r1[nxout] + indices.size(0) * j) - 1] = r.b_I[nxout];
          }
          valid[j] = static_cast<unsigned int>(noNanCol.size(1));
        }
      }
    } else {
      indices.set_size(0, 7439);
      std::memset(&valid[0], 0, 7439U * sizeof(unsigned int));
    }
  }
  normals.set_size(7439, 3);
  PCANormalImpl_single(&loc[0], &indices[0], &valid[0], 7439U,
                       static_cast<unsigned int>(indices.size(0)), &normals[0]);
}

} // namespace coder

// End of code generation (pointCloud.cpp)
