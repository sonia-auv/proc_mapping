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
namespace coder
{
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
            iidx[i] = static_cast<int>(idxTemp1[static_cast<int>(currentNode) -
              1].f1[i]);
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
            sx[i] = inputData[(static_cast<int>(idxTemp1[static_cast<int>
              (currentNode) - 1].f1[i]) + 7439 * (idx - 1)) - 1];
          }

          internal::sort(sx, iidx);
          notnan.set_size(1, iidx.size(0));
          loop_ub = iidx.size(0);
          for (i = 0; i < loop_ub; i++) {
            notnan[i] = static_cast<short>(idxTemp1[static_cast<int>(currentNode)
              - 1].f1[iidx[i] - 1]);
          }

          double temp[3];
          double b_temp_tmp;
          unusedNodes = static_cast<int>(std::ceil(static_cast<double>(sx.size(0))
            / 2.0));
          p = (sx[unusedNodes - 1] + sx[unusedNodes]) / 2.0F;
          cutDimTemp[static_cast<int>(currentNode) - 1] = static_cast<signed
            char>(idx);
          cutValTemp[static_cast<int>(currentNode) - 1] = p;
          leftChildTemp[static_cast<int>(currentNode) - 1] = nextUnusedNode;
          rightChildTemp[static_cast<int>(currentNode) - 1] = static_cast<double>
            (nextUnusedNode) + 1.0;
          cc = upperBoundsTemp[static_cast<int>(currentNode) - 1];
          temp[0] = cc;
          temp_tmp = upperBoundsTemp[(static_cast<int>(currentNode) +
            upperBoundsTemp.size(0)) - 1];
          temp[1] = temp_tmp;
          b_temp_tmp = upperBoundsTemp[(static_cast<int>(currentNode) +
            upperBoundsTemp.size(0) * 2) - 1];
          temp[2] = b_temp_tmp;
          upperBoundsTemp[static_cast<int>(static_cast<double>(nextUnusedNode) +
            1.0) - 1] = cc;
          upperBoundsTemp[(static_cast<int>(static_cast<double>(nextUnusedNode)
            + 1.0) + upperBoundsTemp.size(0)) - 1] = temp_tmp;
          upperBoundsTemp[(static_cast<int>(static_cast<double>(nextUnusedNode)
            + 1.0) + upperBoundsTemp.size(0) * 2) - 1] = b_temp_tmp;
          temp[idx - 1] = p;
          upperBoundsTemp[static_cast<int>(nextUnusedNode) - 1] = temp[0];
          cc = lowerBoundsTemp[static_cast<int>(currentNode) - 1];
          temp[0] = cc;
          upperBoundsTemp[(static_cast<int>(nextUnusedNode) +
                           upperBoundsTemp.size(0)) - 1] = temp[1];
          temp_tmp = lowerBoundsTemp[(static_cast<int>(currentNode) +
            lowerBoundsTemp.size(0)) - 1];
          temp[1] = temp_tmp;
          upperBoundsTemp[(static_cast<int>(nextUnusedNode) +
                           upperBoundsTemp.size(0) * 2) - 1] = temp[2];
          b_temp_tmp = lowerBoundsTemp[(static_cast<int>(currentNode) +
            lowerBoundsTemp.size(0) * 2) - 1];
          temp[2] = b_temp_tmp;
          lowerBoundsTemp[static_cast<int>(nextUnusedNode) - 1] = cc;
          lowerBoundsTemp[(static_cast<int>(nextUnusedNode) +
                           lowerBoundsTemp.size(0)) - 1] = temp_tmp;
          lowerBoundsTemp[(static_cast<int>(nextUnusedNode) +
                           lowerBoundsTemp.size(0) * 2) - 1] = b_temp_tmp;
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

  pointCloud *pointCloud::b_select(const ::coder::array<bool, 2U> &varargin_1, ::
    coder::vision::internal::codegen::Kdtree *iobj_0, pointCloud *iobj_1)
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

  c_pointCloud::c_pointCloud()
  {
    matlabCodegenIsDeleted = true;
  }

  pointCloud::pointCloud()
  {
    matlabCodegenIsDeleted = true;
  }

  c_pointCloud::~c_pointCloud()
  {
    matlabCodegenDestructor();
  }

  f_pointCloud::~f_pointCloud()
  {
    matlabCodegenDestructor();
  }

  e_pointCloud::~e_pointCloud()
  {
    matlabCodegenDestructor();
  }

  d_pointCloud::~d_pointCloud()
  {
    matlabCodegenDestructor();
  }

  b_pointCloud::~b_pointCloud()
  {
    matlabCodegenDestructor();
  }

  pointCloud::~pointCloud()
  {
    matlabCodegenDestructor();
  }

  void pointCloud::extractValidPoints(::coder::array<double, 2U> &location, ::
    coder::array<unsigned char, 2U> &color, ::coder::array<double, 2U> &normals,
    ::coder::array<double, 1U> &intensity, ::coder::array<double, 2U> &rangeData,
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
          location[k + location.size(0) * j] = Location[(r1[k] + Location.size(0)
            * j) - 1];
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
          normals[k + normals.size(0) * j] = Normal[(r3[k] + Normal.size(0) * j)
            - 1];
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
          rangeData[k + rangeData.size(0) * j] = RangeData[(r5[k] +
            RangeData.size(0) * j) - 1];
        }
      }
    } else {
      rangeData.set_size(0, 0);
    }
  }

  void c_pointCloud::findNeighborsInRadius(const double varargin_1[3], ::coder::
    array<unsigned int, 1U> &indices)
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
    d_struct_T pq;
    double b_pRadIn[3];
    double pRadIn[3];
    int nxin;
    nxin = Location.size(0);
    if (nxin < 500) {
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
        double startNode;
        startNode = varargin_1[0] - features[nxout];
        pRadIn[0] = startNode * startNode;
        startNode = varargin_1[1] - features[nxout + features.size(0)];
        pRadIn[1] = startNode * startNode;
        startNode = varargin_1[2] - features[nxout + features.size(0) * 2];
        allDists[nxout] = (pRadIn[0] + pRadIn[1]) + startNode * startNode;
      }

      x.set_size(1, allDists.size(1));
      nxin = allDists.size(1);
      for (k = 0; k < nxin; k++) {
        x[k] = (allDists[k] <= 0.16000000000000003);
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
        bool guard1{ false };

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

        rightChild.set_size(b_this->RightChild.size(0), b_this->RightChild.size
                            (1));
        nxin = b_this->RightChild.size(0) * b_this->RightChild.size(1);
        for (k = 0; k < nxin; k++) {
          rightChild[k] = b_this->RightChild[k];
        }

        startNode = 1.0;
        while (!leafNode[static_cast<int>(startNode) - 1]) {
          if (varargin_1[static_cast<int>(cutDim[static_cast<int>(startNode) - 1])
              - 1] <= cutVal[static_cast<int>(startNode) - 1]) {
            startNode = leftChild[static_cast<int>(startNode) - 1];
          } else {
            startNode = rightChild[static_cast<int>(startNode) - 1];
          }
        }

        pq.D.set_size(0);
        pq.b_I.set_size(0);
        vision::internal::codegen::Kdtree::getNodeFromArray(b_this->IdxAll,
          b_this->IdxDim, startNode, nodeIdxThis);
        vision::internal::codegen::Kdtree::searchNodeRadius(X, varargin_1,
          nodeIdxThis, &pq);
        guard1 = false;
        if (pq.D.size(0) != 0) {
          nxin = b_this->LowerBounds.size(1);
          allDists.set_size(1, nxin);
          for (k = 0; k < nxin; k++) {
            allDists[k] = b_this->LowerBounds[(static_cast<int>(startNode) +
              b_this->LowerBounds.size(0) * k) - 1];
          }

          nxin = b_this->UpperBounds.size(1);
          upBounds.set_size(1, nxin);
          for (k = 0; k < nxin; k++) {
            upBounds[k] = b_this->UpperBounds[(static_cast<int>(startNode) +
              b_this->UpperBounds.size(0) * k) - 1];
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
          if (internal::minimum(b_pRadIn) <= 0.16000000000000003) {
            guard1 = true;
          } else {
            pRadIn[0] *= pRadIn[0];
            pRadIn[1] *= pRadIn[1];
            pRadIn[2] *= pRadIn[2];
            if (internal::minimum(pRadIn) <= 0.16000000000000003) {
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
              allDists[k] = b_this->LowerBounds[(static_cast<int>(currentNode) +
                b_this->LowerBounds.size(0) * k) - 1];
            }

            nxin = b_this->UpperBounds.size(1);
            upBounds.set_size(1, nxin);
            for (k = 0; k < nxin; k++) {
              upBounds[k] = b_this->UpperBounds[(static_cast<int>(currentNode) +
                b_this->UpperBounds.size(0) * k) - 1];
            }

            if (vision::internal::codegen::Kdtree::boundsOverlapBall(varargin_1,
                 allDists, upBounds, 0.16000000000000003, static_cast<double>
                 (X.size(1)))) {
              if (!b_this->LeafNode[static_cast<int>(currentNode) - 1]) {
                if (varargin_1[static_cast<int>(b_this->CutDim[static_cast<int>
                     (currentNode) - 1]) - 1] <= b_this->CutVal[static_cast<int>
                    (currentNode) - 1]) {
                  c_this.set_size(nodeStack.size(0) + 2);
                  c_this[0] = b_this->LeftChild[static_cast<int>(currentNode) -
                    1];
                  c_this[1] = b_this->RightChild[static_cast<int>(currentNode) -
                    1];
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
                  c_this[0] = b_this->RightChild[static_cast<int>(currentNode) -
                    1];
                  c_this[1] = b_this->LeftChild[static_cast<int>(currentNode) -
                    1];
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
                vision::internal::codegen::Kdtree::getNodeFromArray
                  (b_this->IdxAll, b_this->IdxDim, currentNode, nodeIdxThis);
                vision::internal::codegen::Kdtree::searchNodeRadius(X,
                  varargin_1, nodeIdxThis, &pq);
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
    const ::coder::array<unsigned char, 2U> &varargin_3, const ::coder::array<
    double, 2U> &varargin_5, const ::coder::array<double, 1U> &varargin_7, ::
    coder::vision::internal::codegen::Kdtree *iobj_0)
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

  b_pointCloud *b_pointCloud::init(::coder::vision::internal::codegen::Kdtree
    *iobj_0)
  {
    static const float xyzPoints[22317]{ 0.000308371F, 0.00356864F, -0.00592607F,
      0.0054914197F, -0.0141114F, -0.00743080024F, 0.000131774F, 0.0160783F,
      -0.00519657787F, 0.00626541348F, -0.0115893F, 0.00441592F, 0.0101762F,
      -0.00665552029F, 0.0044707F, -0.00530956313F, 0.00555077F, -0.00752147F,
      0.00503375F, 0.0108869F, -0.00693357F, 0.00594662502F, -0.0113048F,
      0.00431913976F, -0.0119420495F, -0.00378502486F, -0.000579219F,
      -0.0119552F, -0.00658242498F, 0.00619306974F, -0.00462360773F, 0.00225152F,
      -0.00466282F, -0.00907061F, 0.00206272304F, 0.0111174006F, -0.00660282F,
      0.00437581306F, 0.00994325057F, -0.0025385879F, 0.00517252F, 0.0124068F,
      -0.0117137F, -0.00755097F, 0.00718715042F, 0.00376006775F, -0.00775585F,
      0.00364194F, -0.00922726F, 0.00906141F, -0.00689537F, 0.0053776633F,
      -0.00570680527F, 0.0026155049F, -0.00515469F, 0.00531509332F,
      -0.00627995329F, 0.00302183651F, -0.0113458F, -0.00289588F, 0.0106482F,
      -0.00451936F, 0.0086451F, -0.00207398F, 0.00301043224F, -0.00212319987F,
      0.00224050251F, -0.00528776F, 0.00633978F, -0.00923484F, 0.00297089107F,
      -0.0123557F, -0.00505253F, -5.92988E-5F, 0.00542014511F, 0.0116495006F,
      -0.00355092F, 0.00868798F, -0.00594375F, 0.00451268535F, 0.00545442F,
      -0.00455172F, -0.00824965F, -4.50248E-6F, 0.0121721F, -0.00520191F,
      -0.00460163504F, -0.0113031F, -0.00442362F, -0.000465202F, -0.00675273966F,
      0.0022361F, -0.0120803F, 0.00431468F, -0.0112807006F, -0.00502654F,
      0.0120449F, -0.000739836F, 0.00331471F, -0.0067781F, 0.0111007F,
      -0.00707894F, 0.00299249496F, 0.0103384F, 0.00518953661F, -0.00626974506F,
      0.00100444F, -0.00394063303F, 0.0057291F, -0.0115648F, 0.00534473639F,
      0.0103386305F, -0.00359709491F, 0.00372298F, 0.0117579503F, -0.0111643F,
      -0.00156152F, 0.000746497F, 0.0119224F, -0.0121764F, -0.00341478F,
      0.00251148F, -0.000875331F, 0.00354126142F, 0.0106398F, -0.0119884F,
      -0.0060427119F, 0.00241564959F, 0.01220995F, -0.00547249336F,
      0.00289332098F, -0.0127703F, -0.00484625F, 0.0052954033F, 0.01451365F,
      -0.00548767F, 0.00201897F, 0.0123176F, -0.00173104F, 0.00300645F,
      0.0110757F, -0.00879238F, 0.00937755F, -0.0111977F, -0.0105361F,
      0.0114800446F, -0.0156662F, -0.0158778F, 0.0138074756F, -0.0115206502F,
      0.0144276F, -0.013083F, 0.0125209335F, 0.0125921499F, -0.0116675338F,
      -0.00974023F, 0.0128649669F, 0.0110966079F, -0.0130095F, 0.00897612F,
      0.0110328F, -0.0148531497F, 0.00894935F, 0.0164015F, 0.0108316F,
      -0.0129483F, -0.00928206F, 0.0141637F, -0.0148692F, -0.00916583464F,
      0.0118464F, -0.0163271F, 0.0120209493F, 0.00852625F, 0.014120725F,
      -0.0140114F, 0.0131088F, -0.0141432006F, -0.00938898F, 0.0144427009F,
      -0.0125571247F, -0.0126022995F, -0.00935222F, 0.00997881F, -0.0163727514F,
      -0.00800766F, 0.00879235F, 0.0131372306F, -0.0131674502F, -0.00967681F,
      0.0154333506F, -0.0148042F, 0.0159109F, -0.0140741F, 0.0152906F,
      -0.0143208F, 0.0148163F, -0.0139615F, -0.00874071F, 0.00853786F,
      0.0127683F, -0.0124227F, 0.0164601F, -0.0128757F, 0.0120597F,
      0.0153594995F, -0.0146814492F, 0.0120079331F, -0.0145688F, -0.00864097F,
      0.00886307F, -0.0141502507F, 0.01503166F, -0.0143887494F, 0.0133684F,
      0.0120983F, -0.0123358F, -0.00922885F, 0.01304702F, -0.0112683F,
      -0.0153101F, 0.0130752F, -0.0153136F, 0.0165515F, -0.0130008664F,
      -0.0145324506F, -0.00996723F, 0.0126851499F, -0.0130554F, 0.00876767468F,
      0.0116138F, 0.0149845F, -0.0138129F, -0.0105694F, 0.0152919497F,
      -0.0134411994F, -0.00912822F, 0.0131705F, -0.0130695F, 0.0141653F,
      -0.0160657503F, -0.0102586F, 0.0149582F, -0.00856512F, 0.00840568F,
      0.0127937F, -0.0107389F, -0.0103487F, 0.0162072F, 0.0162233F,
      -0.0146272499F, -0.00962297F, 0.0156904329F, -0.0147939F, 0.00966582F,
      -0.0130472F, 0.00917638F, 0.01097525F, -0.0140809994F, -0.0143058F,
      0.0124382F, 0.00844901F, 0.0142386667F, -0.0128662F, -0.0149079F,
      0.0162047F, -0.0143434498F, 0.0147121754F, -0.0159866F, 0.0154225F,
      -0.0110065F, 0.0203726F, -0.0192044F, 0.00476674503F, 0.00383538753F,
      -0.0159163F, -0.00607773F, 0.00354283F, -0.00234341F, 0.0027516F,
      -0.0131931F, -0.0146934334F, -0.0105835F, 0.0160499F, -0.0158110503F,
      -0.00970621F, 0.0151680335F, 0.0199551F, -0.0150138661F, 0.01166871F,
      -0.0143957501F, -0.0119705F, -0.0101297F, 0.0129208751F, -0.015327733F,
      0.0130761F, -0.014825F, 0.0140696F, -0.0143554751F, 0.0156145F, -0.013146F,
      0.0160758F, -0.0141921F, 0.0140697332F, -0.0150125F, 0.016602F,
      -0.0132838F, 0.0152875F, -0.0140562F, 0.0121816F, -0.0158981F,
      -0.0146730496F, -0.0100163F, 0.0134116F, -0.0109023F, 0.0147576F,
      -0.0155291F, 0.0138556007F, -0.0128386505F, 0.0110904267F, 0.0119135669F,
      -0.0100069F, 0.0129658F, 0.0128629329F, -0.012655233F, 0.0132244509F,
      -0.0157026F, -0.00964733F, 0.0152593339F, -0.0164218F, -0.0133417668F,
      -0.0138875F, 0.0160140321F, -0.0166703F, -0.0147303F, 0.01285375F,
      -0.0117619503F, 0.0130588F, -0.0126153F, 0.0137300501F, -0.01419478F,
      0.0149018F, -0.014182033F, 0.0121890306F, -0.00977129F, 0.0117789F,
      -0.0132222F, -0.0114152F, 0.0145282503F, -0.0149651F, 0.0135391504F,
      -0.0128318F, -0.0129509F, 0.0145847499F, -0.0166756F, 0.0130821504F,
      0.0144244F, -0.0140713509F, 0.0139852669F, -0.0145665F, -0.0105375F,
      0.01235855F, -0.0137010254F, -0.00947159F, 0.013776375F, -0.0152167F,
      0.0141197F, -0.0153742665F, 0.0164089F, -0.0123507F, 0.0155019F,
      0.0130509F, 0.0122379F, -0.0154284F, 0.0130635F, -0.0151380673F,
      0.0148677668F, -0.0163716F, 0.014235F, -0.0109723F, 0.016227F, -0.0131102F,
      0.015017F, -0.0113126F, 0.0142961498F, -0.0161819F, -0.0100371F, 0.014787F,
      -0.0147109F, -0.0103717F, 0.0144425F, -0.0205364F, 0.0146355666F,
      0.0185799F, 0.0195687F, -0.0175552F, -0.00321684F, -0.00366165F,
      0.00783592F, -0.0139334F, -0.000967931F, 0.00563141F, 0.00980697F,
      0.0146715F, 0.0197095498F, -0.0134000499F, 0.0136348F, -0.0136186F,
      0.000442538F, -0.00425579678F, 0.00529044F, 0.01508235F, -0.0126664F,
      0.000616885838F, -0.0141449F, -0.00599484518F, 0.00555351516F,
      0.0134426206F, -0.00422317F, -0.00857492F, 0.00408874732F, -0.0129294F,
      -0.00515836524F, -0.00713877F, 0.000597156F, 0.0100783F, 0.00423284667F,
      -0.00451813918F, 0.00399795F, -0.0118569F, -0.00492237F, 0.00968314F,
      -0.0110034F, -0.00759921502F, 0.00553372F, -0.0117225F, -0.00487638265F,
      0.00303983106F, -0.00690104486F, 0.00672912F, 0.0106008649F, 0.008961F,
      -0.00496040704F, 0.00689552538F, 0.00388149824F, -0.0120112F, -0.00756104F,
      0.00429642666F, 0.00958396F, -0.00338493986F, -0.00517031F, 0.00251586F,
      0.0133655F, -0.00725884F, 0.0114085F, -0.00555825F, 0.00686055515F,
      -0.0120382F, -0.00783623476F, 0.0046153F, 0.0122724334F, -0.0120477F,
      -0.00268672F, 0.00305046747F, -0.0044838544F, 0.00331925F, 0.0107642F,
      -0.0129519F, -0.00562541327F, 0.00609374512F, -0.00538097F, 0.00248228F,
      0.012869F, -0.00416848343F, 0.00100588F, 0.011937F, -0.00366482F,
      0.00309099F, 0.0100669F, -0.00513551338F, 0.00286023F, -0.0132281F,
      -0.00317125F, 0.00469961F, 0.0115671F, -0.0111016F, -0.00583249F,
      -0.0036551F, 0.00328729744F, -0.00344055F, 0.00531972619F, 0.0133915F,
      -0.00498249F, 0.00454682484F, 0.009471F, 0.00127679F, -0.00673122657F,
      -0.00594337471F, 0.0133362F, -0.00731172F, 0.00716118F, 0.00710371F,
      -0.0121931F, -0.00170887F, 0.00358331506F, 0.00482814666F, -0.00378489F,
      0.0100732F, -0.0113444F, -0.0091063F, 0.00275066821F, -0.0069211F,
      0.00408016238F, -0.00641421F, 0.0119420663F, -0.00520438794F,
      0.00269097695F, -0.0118509F, -0.00695878F, 0.000187906F, 0.0107403304F,
      -0.0120175F, 0.00672089495F, 0.0115331F, 0.00407759F, 0.0132669F,
      -0.00762381498F, -4.93204E-5F, -0.0030309F, 0.00396614289F, 0.00713988F,
      0.0135394F, -0.00329837F, 0.00278599397F, -0.00639045658F, 0.00537954504F,
      -0.00570973521F, 0.00494093122F, -0.0133051F, 0.00664343F, 0.012637F,
      -0.0109672993F, -0.00447059516F, 0.00347190304F, -0.0110879F,
      -0.00748386653F, 0.0065262F, 0.0144569F, -0.0104467F, 0.00466414F,
      -0.0125778F, -0.00886709F, 0.00653641485F, 0.0109746F, -0.0140088F,
      -0.00759999F, 0.00707476F, 0.0112832F, 0.0133182F, -0.00728544F,
      0.00340637844F, 0.0183099F, -0.00656507F, -0.014180067F, -0.00350765F,
      0.00317714037F, -0.0148658995F, -0.00874419F, 0.00995892473F, -0.0206003F,
      0.0114239352F, 0.00700074341F, 0.0135398F, 0.003175F, -0.003175F,
      0.00124622846F, -0.00018162F, -0.003175F, -0.003175F, -0.003175F,
      0.003175F, 0.003175F, -0.003175F, 0.000838942F, -0.0011781F, 0.00284173F,
      -0.003175F, 0.003175F, 0.00307468F, 0.003175F, 0.00108602F, -0.003175F,
      0.003175F, 0.003175F, -0.003175F, 0.00146379F, -0.003175F, 0.003175F,
      -0.00193892F, 0.003175F, 0.003175F, -0.003175F, -0.000607726F, -0.003175F,
      -0.003175F, 0.00301081F, -0.003175F, 0.003175F, 0.00297065F, -0.003175F,
      -0.003175F, 0.003175F, 0.003175F, 0.003175F, 0.00122123654F, 0.00599752F,
      0.00191455F, 0.00497874F, -0.0134105F, 0.0146702F, -0.00904826634F,
      0.00895706F, 0.011846F, -0.0130711F, -0.00071408F, 0.00431535486F,
      -0.0128902F, -0.0101006F, 0.00708134F, 0.0136829503F, 0.019693F,
      -0.0159896F, 0.0165095255F, -0.0134099F, -0.00617604656F, 0.0081535F,
      0.0139135F, -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F,
      0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.00244112F,
      0.00542628F, 0.0136033F, 0.0167052F, -0.0125236502F, -0.00614004F,
      0.0117915664F, -0.00530736335F, 0.010991F, -0.0153308F, 0.0109944F,
      0.0197901F, -0.0143716503F, -0.000685813F, 0.0128656495F, 0.003175F,
      -0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F,
      -0.003175F, -0.003175F, -0.00309159677F, 0.003789F, 0.00156456F,
      -0.0205991F, 0.00238669F, 0.0192949F, -0.00311796F, -0.00192788F,
      0.0124007F, -0.0159136988F, 0.0154450005F, -0.0158639F, -0.00411175F,
      0.00388172F, 0.00986893F, 0.003175F, 0.003175F, 0.003175F, 0.003175F,
      0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      0.003175F, -0.003175F, -0.00171022F, 0.00464744F, -0.0147073F,
      -0.00543208327F, 0.00997129F, -0.0125595F, 0.0141335F, -0.00399917F,
      -0.0123553F, -0.00730564F, 0.000221774F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      -0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.0016586F, -0.0137746F,
      0.00560036F, -0.0165602F, 0.00911165F, 0.00265678F, -0.0131024F,
      -0.00825379F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F,
      -0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, -0.0108333F, 0.0103375241F, -0.0132059F,
      0.0147913005F, -0.00194599F, 0.00717152469F, 0.0128674F, -0.00706104F,
      0.00688256F, -0.0131562F, 0.016303F, -0.0127365F, 0.0165559F, -0.00798035F,
      0.00238134F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, -0.003175F, -0.003175F, -0.003175F, -0.0115562F, -0.00987932F,
      0.00690943F, 0.00981825F, -0.0148467496F, 0.0154418666F, -0.00457627F,
      -0.0124901F, -0.00824595F, 0.0163273F, -0.0122342F, 0.0131132668F,
      0.00147118F, 0.0130888F, 0.003175F, 0.003175F, -0.003175F, -0.003175F,
      0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.000414008F, 0.00258281F,
      0.0111506F, 0.0146973506F, -0.0146761F, -0.00354914F, 0.0155176502F,
      -0.0131768F, 0.0165578F, 0.00733956F, 0.0101833F, -0.003175F, 0.003175F,
      0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, -6.44054E-5F, 0.00938621F,
      -0.0137443244F, 0.0142310504F, -0.0132034F, -0.0059493F, 0.0144139F,
      -0.0119806F, 0.00339982961F, -0.0106433F, 0.0127218F, -0.0132079F,
      0.0156721F, 0.00533761F, 0.0116474F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F,
      -0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.003175F, -0.00185920659F, 0.00177786F, -0.0114393F,
      0.00291612465F, 0.0156406F, -0.013207F, -0.0104193F, 0.00244034F,
      0.0134559497F, -0.0130733F, -0.000429249F, -0.003175F, 0.003175F,
      0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F,
      0.003175F, 0.003175F, 0.003175F, 0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.0120817F, 0.0107242F, 0.0165569F, -0.00940134F, 0.00407069F,
      0.0119405501F, 0.00258885254F, -0.0148584507F, 0.01264665F, -0.0165602F,
      0.00962989F, -0.00275697489F, 0.00654336F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F,
      0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F,
      0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.0140223F, -0.00686053233F, 0.00436038F,
      0.0118651F, 0.0161652F, -0.0115562F, -0.00634395F, 0.0089565F, -0.0165597F,
      0.0165576F, 0.010978F, -0.003175F, -0.003175F, -0.003175F, -0.003175F,
      0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F,
      0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, 0.011638F, 0.0163478F, -0.00214872F,
      0.00828963F, 0.0152205F, 0.00455536507F, -0.0146959F, 0.0141899F,
      -0.0159755F, -0.0109046F, -0.00657835975F, 0.000487093F, -0.003175F,
      -0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F, 0.003175F,
      0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.00125007F,
      0.00431438F, -0.0139544F, -0.00785714F, -0.0165567F, 0.0144887995F,
      -0.0127723F, -0.0101004F, 0.00658325F, -0.0148933F, 0.0113838F,
      -0.00603511F, -0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F,
      0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.00739314F,
      0.0145294499F, 0.007027F, 0.0116292434F, 0.00688362F, -0.00405481F,
      0.00359397661F, -0.0135374749F, -0.015969F, 0.015286F, -0.007513F,
      0.00189568498F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      5.85124E-5F, -0.015939F, 0.00640363F, 0.00978343F, 0.00715228F,
      -0.0146682F, 0.0124016497F, 0.0102397F, -0.003175F, 0.003175F, -0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F,
      -0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.0128185F, -0.0157893F, 0.00485514477F,
      -0.000771205F, -0.00908649527F, -0.0132072F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      0.003175F, -0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F,
      0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.000274313F,
      0.011333F, -0.0083694607F, 0.00874926F, 0.011527475F, 0.00206989F,
      -0.00507932F, 0.00246464F, -0.0148837492F, 0.0125202F, -0.00465655513F,
      0.00370123F, 0.00101627F, 0.003175F, 0.003175F, -0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, -0.003175F, -0.003175F, -0.003175F,
      -0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F,
      0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.00587994512F, 0.0123558994F, -0.0126197F, -0.0101997F,
      -0.00129738F, 0.004367F, -0.0134307F, 0.0124795F, -0.00344069F,
      0.00400996627F, 0.0121908F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.0146274F, 0.00305394642F,
      0.0124100503F, -0.0125406F, 0.012526F, -0.00522849523F, 0.0110578F,
      -0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F,
      0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.00330397F, -0.0150845F,
      -0.00985723F, -0.0144768506F, -0.0132216F, -0.00783911F, 0.015577F,
      -0.00504237F, 0.00787736F, -0.00980116F, 0.0149814496F, -0.00152645F,
      0.00436495338F, 0.0106955F, 0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F,
      -0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F,
      -0.000514261F, -0.00556278F, 0.00646944F, -0.0143013F, -0.0121611F,
      -0.00114474F, 0.00386787509F, 0.0149247F, -0.00453285F, 0.0013512572F,
      -0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.00347653497F, -0.0111594F,
      0.00854187F, -0.0163401F, -0.00930989534F, 0.00117006F, 0.0115301F,
      -0.0128474F, 0.011216F, -0.00891237F, 0.00861254521F, 0.00289175985F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F,
      0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F,
      0.003175F, 0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.00387484F, 0.00833315F, 0.00994191F, -0.0126079F, -0.0106679F,
      -0.00104868F, 0.0145647F, 0.00158902F, 0.00614539F, -0.0126942F,
      -0.00980048F, 0.013580149F, -0.0126208F, 0.01267F, -0.00500467F,
      0.0123985F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F,
      0.003175F, 0.00289765F, -0.00837281F, 0.00811743F, -0.01282595F,
      0.0108036F, -0.00233554F, 0.00198327494F, -0.0126877F, 0.0158272F,
      -0.0165204F, 0.0126727F, -0.0125424F, 0.003175F, -0.003175F, -0.003175F,
      0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      -0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.00665487F, 0.00177262F, 0.01199935F, -0.00251556F, 0.00275741145F,
      0.00717619F, -0.0162184F, 0.0149965F, -0.00589146F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, -0.003175F, -0.003175F, -0.003175F,
      0.003175F, -0.00272516F, 0.00600531F, -0.0125213F, 0.012683F, 0.0119585F,
      0.00322224F, 0.00196192204F, -0.0132067F, -0.00960332F, 0.00959175F,
      0.00204417F, 0.0103131F, -0.003175F, 0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, 0.003175F, 0.003175F, 0.003175F,
      0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.000870258F, -0.0136434F, 0.0151166F,
      -0.00811216F, 0.00560447F, 0.00775274F, 0.0129897F, -0.0115114F,
      -0.00267682F, -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F,
      0.003175F, -0.003175F, -0.003175F, -0.003175F, -0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F,
      0.003175F, 0.000519577F, 0.00522266F, -0.014967F, -0.0102572F, 0.00167388F,
      -0.00339141511F, 0.0069627F, -0.0080651F, 0.0070221154F, -0.00944527F,
      0.0131955F, -0.00213228F, -0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      -0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F, -0.003175F,
      -0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.00213447F,
      0.0102777F, -0.0158024F, 0.00377747F, -0.00217658F, -0.0115825F,
      -0.00268766F, 0.00989262946F, -0.0140005F, -0.0161118F, -0.0123337498F,
      0.00690106F, 0.0128456F, 0.003175F, -0.003175F, -0.003175F, 0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.00484838F, -0.0115983F, -0.0085458F,
      0.0118939F, 0.0136544F, -0.00102198F, -0.0141283665F, 0.014813F,
      0.00503416359F, 0.0132207F, 0.003175F, 0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.0062893F, 0.00352757F, -0.0108064F, -0.0106083F,
      -0.0159715F, 0.0147826504F, 0.000297694F, -0.00676207524F, 0.00670158F,
      -0.0141598F, -0.0103648F, 0.012568F, 0.0124198F, 0.00538996514F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      -0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F,
      0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.0077846F, 0.0129002F,
      0.0147618F, -0.00315017649F, -0.0128521F, -0.00956657F, -0.016559F,
      0.0132082F, -0.00440483F, 0.00335669F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.0134468F, 0.00457169488F, 0.0143119F, 0.00794755481F,
      -0.00072618F, -0.0105784F, 0.00692263F, -0.014023F, 0.0146387005F,
      -0.00635512499F, 0.0088841F, -0.003175F, 0.000177257345F, 0.003175F,
      0.003175F, 0.003175F, 0.003175F, 0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      -0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F,
      -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F,
      -0.003175F, 0.003175F, 0.0155581F, -0.0165587496F, 0.0158277F,
      -0.00248405F, -0.0104528F, 0.00845794F, -0.0129639497F, 0.0126479F,
      -0.00251617F, 0.00300706155F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F, -0.003175F,
      -0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F,
      -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.003191025F, -0.0157577F, -0.00794325F, 0.0146976F, -0.0162746F,
      0.0129278F, -0.0115496F, -0.00137789F, 0.00498187F, -0.0109459F,
      -0.00337431F, -0.0123719F, 0.0086641F, 0.0143321995F, -0.0135505665F,
      0.0134122F, -0.0118625F, -0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, -0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F, 0.00365938F,
      -0.0129938F, -0.00787144F, -0.0158441F, 0.0154389339F, 0.00993718F,
      -0.003367875F, -0.0161971506F, 0.013371F, 0.0122030992F, -0.00586166F,
      0.00386105292F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F,
      0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F,
      -0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.0121399F,
      -0.00118064F, 0.00725027F, 0.0105321F, 0.0132062F, 0.00553096F,
      -0.0126692504F, -0.0142709669F, 0.0148688499F, -0.00436513F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, -0.003175F, -0.003175F, -0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      0.003175F, 0.003175F, -0.0104995F, 0.00186047703F, 0.0122348F, -0.0157837F,
      0.0132104F, -0.0128472F, -0.00336312F, -0.00673983F, 0.0148682F,
      -0.0140100503F, 0.0132074F, -0.003175F, -0.003175F, 0.003175F, -0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.00562516F,
      -0.00857823F, 0.013153F, -0.0075923996F, -0.00244188F, 0.00616973F,
      0.0137015F, -0.00740511F, 0.003175F, 0.003175F, 0.003175F, -0.003175F,
      -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, 0.00471491F, 0.00747929F, 0.0154421F, -0.0160577F, 0.015977F,
      0.00669115F, 0.00517237F, -0.0145199504F, -0.0163424F, -0.00483724F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F,
      -0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.0041473F, 0.00411313F, -0.0165565F, 0.00105558F,
      -0.00317245489F, 0.00397865521F, 0.0124528F, -0.003175F, 0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F,
      0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F,
      0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F,
      0.003175F, -0.00299958F, 0.000414687F, -0.0160702F, 0.0165606F,
      0.0142108342F, -0.00114229F, 0.00566021F, -0.00659769494F, 0.00248464F,
      -0.0132003F, 0.00820013F, -0.016235F, -0.0135245F, -0.00142497F,
      -0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F,
      0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F, -0.003175F,
      0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      0.003175F, 0.0114733F, -0.0148830507F, -0.00963008F, 0.0103289355F,
      -0.00110359F, -0.00689099F, -0.010237F, 0.0131519F, -0.0141696F,
      0.00178788F, 0.0013306F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.0130995F,
      -0.0044824F, -0.0154751F, 0.0109938F, -0.006413355F, 0.00564316F,
      -0.0128648F, 0.0159033F, -0.0160407F, -0.0129656F, -0.00842827465F,
      0.00892286F, -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F,
      -0.003175F, 0.003175F, 0.003175F, 0.003175F, 0.003175F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F,
      -0.003175F, 0.003175F, 0.0113779502F, -0.0122886F, 0.00525257F,
      0.00263397209F, 0.00971415F, -0.0139308F, 0.0105278455F, -0.013208F,
      -0.00858535F, 0.00513105F, 0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F,
      0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F,
      0.003175F, 0.003175F, 0.003175F, -0.00892541558F, 0.00593131687F,
      0.0165569F, -0.0126347F, -0.00238177F, 0.000210783F, -0.0141307F,
      0.0144139994F, -0.0161197F, 0.0159511F, -0.0102634F, 0.00482129492F,
      0.013618F, 0.0011122F, 0.003175F, 0.003175F, -0.003175F, 0.003175F,
      0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F,
      -0.003175F, -0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F,
      0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      0.003175F, -0.003175F, 0.003175F, 0.00497896F, -0.0120527F, 0.00708187F,
      -0.0140711F, 0.0124861F, 0.00609749F, -0.0126079F, -0.00838686F,
      0.0148998341F, 0.0123600494F, -0.00382541493F, 0.00206474F, 0.0131575F,
      0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F,
      0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      -0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.003175F, -0.0151814F, -0.00420716219F, 0.00812564F,
      -0.0165575F, 0.0128556499F, -0.0109051F, -0.00930984F, -0.00487849675F,
      0.00267429F, 0.0119636F, -0.0131298332F, 0.015979F, -0.00808611F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.002550025F, -0.0141691F, 0.0157532F, 0.0164764F,
      -0.00551967F, 0.0136186F, 0.00129014F, -0.00587297F, -0.0152103007F,
      0.0147497F, 0.0142635F, -0.00472911F, 0.00789449F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F,
      -0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F,
      -0.003175F, -0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F,
      0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.013209F,
      -0.0141692F, -0.00773173F, 0.0116924F, 0.00499144F, 0.00171809F,
      -0.0126906F, -0.0129171F, 0.0100800395F, 0.00537249F, -0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F,
      0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F,
      0.003175F, -0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F,
      -0.003175F, 0.00128735F, 0.000291834F, 0.0108143F, -0.0163863F,
      -0.0119463F, -0.00569749F, 0.00218939967F, 0.0165605F, -0.0139701F,
      -0.00568380347F, 0.00338562F, -0.003175F, -0.003175F, -0.003175F,
      -0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.00328288344F, 0.0112589F, 0.0139904339F, 0.00249228F, -0.00981975F,
      0.0012159F, 0.0111676F, 0.015753F, -0.00898871F, 0.00291694049F, 0.003175F,
      0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      -0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F,
      0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.0110427F, -0.00831569F, -0.0160962F, 0.013208F, 0.00886207F,
      0.0147311F, -0.0110612F, -0.00862919F, 0.0141042F, -0.00296054F,
      0.0052977F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F,
      0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.000970006513F, -0.0132902F,
      -0.00443258F, 0.00969009F, -0.0132094F, 0.0151137663F, -0.0124072F,
      -0.00563574F, 0.00866115F, 0.0134847499F, -0.00505119469F, 0.00579073373F,
      0.00253099F, 0.003175F, -0.003175F, -0.003175F, -0.003175F, -0.003175F,
      0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.00536256F, 0.00315290387F, 0.0110094501F, 0.0120991F, 0.00580338F,
      -0.00514490483F, -0.011067F, 0.0147524F, -0.0132094F, 0.0115339F,
      0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      0.003175F, 0.003175F, 0.003175F, 0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F,
      0.00175882F, -0.0134998F, 0.00795569F, -0.0132069F, 0.0162350498F,
      -0.012181F, -0.00968976F, 0.0103523F, -0.00320821F, 0.0159799F,
      -0.0124021F, 0.0147146499F, -0.00243474F, -0.00176538F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F,
      -0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, -0.0057481F, -0.00682026F, 0.013319F, -0.0159778F, 0.0161824F,
      -0.00844784F, 0.0134277F, 0.00508399F, -0.0153112495F, 0.0159019F,
      -0.003175F, 0.002227F, 0.003175F, -0.003175F, -0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, -0.0120213F, -0.0100886F,
      0.00888196565F, -0.0132093F, -0.0110100508F, 0.00311964F, 0.00360642F,
      0.01265145F, -0.0128662F, 0.0160902F, 0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.000969048F,
      0.0121945F, -0.0129407495F, 0.00380421F, -0.00665068F, 0.00223300792F,
      -0.0107793F, 0.0161065F, -0.00531356F, 0.00665278F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.00508383F, -0.0165568F, -0.00756925F, 0.00373068F,
      -0.0157318F, -0.0145162F, 0.00752692F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, 0.003175F, 0.003175F, 0.003175F,
      -0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.000548307F,
      -0.0143408F, -0.00949124F, 0.00428265F, 0.0122968F, -0.00576092F,
      0.00917827F, 0.0141668F, 0.010568F, -0.00896247F, 0.00196343F,
      -0.00149647F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F,
      -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F,
      0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.0105177F, 0.00654136F, 0.00941307F,
      0.00413084F, 0.0148879F, -0.0129518F, -0.00458364F, 0.00526045F, 0.003175F,
      0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.00412554F, -0.0121009F, -0.0137142334F, -0.0139798F,
      -0.00782762468F, 0.00434474F, -0.0123224F, 0.0151114F, -0.003175F,
      0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, 0.0140847F, -0.0126365F, 0.00923707F,
      -0.00542444F, 0.00175605F, 0.011474167F, -0.0133865F, 0.0144145F,
      -0.00227483F, -0.00236428F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F,
      -0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, 0.003175F, 0.003175F, 0.003175F,
      -0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      -0.003175F, -0.003175F, 0.003175F, 0.00206572F, 0.00804492F, 0.0147867501F,
      -0.00960367F, 0.00848802F, -0.000775349F, 0.00602313F, 0.011119F,
      0.0125208F, -0.00370033504F, 0.000551293F, -0.003175F, 0.00173144F,
      -0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, -0.003175F, -0.003175F, -0.003175F,
      -0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.000304545F, -0.00574314035F,
      -0.0162387658F, -0.00297043496F, 0.0074233F, 0.0156522F, -0.00492891483F,
      0.00510257483F, 0.0126876F, -0.00823884F, 0.0143382F, -0.0110863F,
      -0.0108285F, 0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F,
      -0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, -0.0129078F, 0.00316469511F, 0.0109146F,
      -0.009510695F, 0.0119853F, -0.00483061F, -0.0124431495F, -0.00917525F,
      -0.0129136F, -0.00234358F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.000557398F, -0.0109436F, -0.00829677F,
      0.0116724F, 0.0125574F, -0.00338159F, -0.0037279F, 0.0116284F, -0.0109252F,
      -0.00823291F, 0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F,
      -0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F,
      0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.00216792F, -0.015484F,
      -0.00780228F, 0.0149742F, -0.0126382504F, 0.0165595F, -0.0157658F,
      -0.00812198F, -0.00539178355F, 0.00506668F, 0.0106801F, -0.0157874F,
      -0.00439647492F, 0.0135106F, -0.003175F, 0.000722685F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F,
      -0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F, -0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F,
      0.003175F, -0.0136086F, -0.00925029069F, -0.01321F, 0.0012179655F,
      0.0104313502F, -0.00539156F, -0.0123952497F, 0.0128957499F, 0.0143895F,
      -0.00663291F, 0.00731495F, 0.00974524F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F,
      -0.003175F, -0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F, 0.003175F,
      0.00694925F, -0.014880849F, -0.0119825F, -0.0056016F, -0.011427F,
      -0.00481179357F, 0.00280693732F, 0.0103439F, -0.00969357F, -0.0165606F,
      0.0159733F, 0.00652254F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      -0.003175F, 0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F,
      0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F,
      0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.00182524F,
      -0.011191F, -0.00558069F, 0.00972189F, -0.0130198F, -0.0106375F,
      -0.0104711F, 0.00135094F, 0.0120026702F, -0.00209084F, -0.0011923F,
      0.0139777251F, -0.0127218496F, 0.0152957663F, -0.0111671F, 0.00247434503F,
      -0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F, 0.003175F,
      -0.003175F, -0.003175F, -0.003175F, 0.00112426F, 0.013802F, 0.0129646F,
      -0.00988347F, 0.0089385F, -0.00285783F, 0.0038544F, 0.01299405F,
      -0.00310264F, 0.00608043F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      0.00485355F, 0.0148821F, -0.0126654F, 0.00551422F, 0.00653609F, 0.0110044F,
      0.016559F, -0.0147834495F, -0.00962675F, 0.0153496F, -0.00185769F,
      0.00290430663F, -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.00411346508F, -0.00379961F,
      0.000208544F, -0.0132078F, 0.0130494498F, 0.00785573F, 0.0127695F,
      -0.00529201F, 0.00598151F, -0.00702425F, 0.00493205F, 0.0136011F,
      0.0122416504F, -0.003175F, 0.00239589F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, -0.000143571F, -0.0135804499F, -0.0149867665F,
      0.0146924499F, -0.00743408967F, 0.011127F, 0.00681676F, 0.00665056F,
      -0.0165562F, -0.00270631F, 0.007421F, -0.00201713F, 0.003175F, -0.003175F,
      -0.003175F, -0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F,
      0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.002912F, 0.00969946F,
      -0.0163553F, 0.0153155F, 0.00347526F, -0.0092702F, -0.0135018F, 0.0155518F,
      0.00776784F, 0.0100806F, -0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      -0.003175F, 0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F,
      -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.0112487F, -0.0104226F, -0.0123752F, -0.00380476564F, 0.00561246509F,
      -0.00390309514F, 0.00729371F, -0.0136251505F, 0.010498F, -0.0151897F,
      0.0131611F, -0.00726514962F, 0.00614582F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F,
      0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, -0.00897706F, 0.00749035F, 0.00975162F,
      0.0148833F, -0.00622721F, 0.0102687F, 0.00623631F, -0.0163064F, 0.0141834F,
      -0.0123430993F, 0.011394F, 0.0108554F, 0.00194788F, 0.003175F, -0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F,
      0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.00262746F, 0.0106548F, -0.0160476F, 0.00320485514F,
      0.0139422F, -0.00654834F, 0.00137791F, 0.0140946507F, 0.0111735F,
      -0.00772817F, 0.005661F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F,
      -0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, -0.00573074F, 0.00668155F, -0.011757F, 0.00864717F, 0.0126267F,
      0.0164311F, -0.0126449503F, 0.00560956F, -0.01251F, 0.00918001F,
      -0.0161158F, 0.00919263F, -0.003175F, -0.003175F, 0.003175F, -0.003175F,
      -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      -0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F,
      0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F,
      0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      -0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F,
      0.003175F, -0.003175F, -0.0143378666F, -0.00782431F, 0.00831495F,
      0.0117832F, -0.0165285F, 0.0148380511F, -0.00186008F, -0.000875537F,
      -0.00226294F, 0.00740645F, 0.0145223495F, 0.014123F, -0.00300495F,
      0.00154566F, 0.003175F, -0.003175F, -0.003175F, -0.003175F, -0.003175F,
      0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F,
      0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F,
      0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F,
      0.003175F, -0.00246718F, 0.0047333F, 0.0165573F, -0.00818626F,
      -0.00436831F, -0.013601F, 0.00858963F, -0.003175F, 0.003175F, -0.003175F,
      -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F,
      0.003175F, 0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.00262964F, -0.0112161F,
      0.00148149F, -0.0127574F, -0.015218F, 0.00518538F, 0.0135408F,
      -0.00428516511F, 0.00557388505F, 0.00962801F, 0.00868612F, -0.0165569F,
      -0.00482765352F, -0.00129571F, 0.00153699F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F, -0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.0027601F, 0.00537027F, -0.0111978F,
      0.0136611F, -0.0112646F, -0.00654637F, -0.0132309496F, -0.0165587F,
      0.0109043F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F,
      -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F,
      -0.003175F, 0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.0046659F, 0.00903052F,
      0.0119875399F, -0.0125805F, 0.016491F, -0.0143912F, -0.00916648F,
      0.00327208F, -0.0123437F, -0.00119129F, 0.00370073F, -0.0141710499F,
      0.0135269F, -0.010582F, 0.0160035F, -0.00905600935F, 0.00210862F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F,
      0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F,
      0.003175F, 0.003175F, -0.003175F, 0.000127627F, -0.00360739F,
      0.0146131497F, -0.0120243F, 0.0114669F, -0.00395634F, 0.0033782F,
      0.0148252994F, -0.0145846335F, 0.0125253F, -0.003175F, 0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F, -0.003175F,
      -0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F,
      0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F, -0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      0.00158517F, 0.00312321F, 0.0153507F, 0.00252385F, 0.00504844F, 0.0068942F,
      -0.009317F, 0.0132077F, 0.00314068561F, 0.00206261035F, -0.003175F,
      0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F,
      0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F,
      0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.00483309524F,
      0.00647494F, -0.0147124501F, -0.0119021F, 0.00160025F, 0.013449F,
      -0.00652805483F, -0.0165562F, 0.0125008F, 0.0165569F, -0.00401794678F,
      0.00111266F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F,
      -0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      -0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F,
      0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.00462831F, 0.0118279F, -0.0165579F,
      0.0141963661F, 0.00510872F, 0.0157694F, 0.00146855F, -0.00210799F,
      0.0142803F, -0.0131666F, 0.00365892F, 0.0120854F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, 0.003175F, 0.003175F, 0.003175F,
      0.00405289F, -0.0157532F, -0.00624346035F, -0.0156671F, 0.000400996F,
      0.0122314F, 0.0162516F, -0.0119566992F, -0.00654192502F, 0.000613232F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F,
      -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, 0.003175F, -0.00479691F, 0.0041044F, -0.0111231F, 0.00806933F,
      0.0157307F, -0.00763546F, 0.00232303F, 0.0119496F, -0.0159853F,
      0.0162835009F, -0.00770819699F, 0.003175F, -0.003175F, 0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      0.00614638347F, 0.0130836F, -0.0135843F, -0.00252468F, 0.0116281F,
      -0.0117306F, 0.0161773F, -0.0118329F, 0.0146178F, -0.0113937F,
      -0.00886261F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.00280835503F,
      -0.0165558F, 0.0132104F, -0.00891708F, 0.00683712F, -0.000467848F,
      -0.0132075F, 0.0132092F, 0.00603216F, -0.003175F, 0.003175F, -0.003175F,
      -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, 0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      -0.003175F, -0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F,
      0.003175F, -0.0135898F, -0.00752381F, 0.00206582F, 0.012330967F,
      -0.00548089F, -0.00450192345F, -0.0150480494F, 0.0129826F, 0.003175F,
      0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F,
      0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.00170052F, -0.0141997F,
      -0.00806229F, -0.0129491F, 0.0165561F, 0.00386337F, -0.00591488F,
      -0.00732059F, 0.000410018F, 0.0112758F, -0.0164364F, 0.013560175F,
      -0.0127263F, -0.00965773F, 0.00680832F, 0.003175F, -0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F,
      0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F,
      0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F, -0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      -0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F, -0.003175F,
      0.00434247F, -0.0142792F, 0.0114507F, -0.00828075F, 0.00604068F,
      0.01080315F, 0.00407561F, -0.00242488F, 0.000283766F, -0.0119945F,
      -0.0103468F, 0.0121658F, -0.0156193F, 0.0118948F, -0.00147462F,
      0.00101612F, -0.003175F, -0.003175F, -0.003175F, -0.003175F, -0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F,
      0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F, -0.003175F,
      0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.0140596F, 0.005297F, 0.0103067F, -0.0093229F, 0.0090018F,
      0.0124511F, 0.0112179F, -0.0132059F, 0.0132133327F, 0.0122736F,
      -0.00248605F, 0.00254146336F, 0.0096178F, -0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.00315294F,
      -0.00785133F, -0.0165459F, 0.0164616F, -0.00413721F, 0.00730844F,
      0.00971056F, 0.00683123F, -0.00786625F, -0.0141821252F, 0.011947F,
      -0.0165497F, 0.0163848512F, -0.0124807F, -0.008352F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, -0.003175F, -0.003175F, -0.003175F, -0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.0129593F, -0.0143137F,
      -0.00733618F, 0.00828979537F, 0.0130897F, 0.00111398F, -0.0124615328F,
      0.012549F, -0.0137177501F, -0.00728174F, 0.003175F, -0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.00471137F, 0.00772922F,
      0.0128187F, -0.0132073F, 0.0147138F, -0.0112542994F, -0.00888029486F,
      0.0117787F, 0.00141925F, -0.0122498F, 0.0159285F, -0.016296F, -0.0104945F,
      0.0133577995F, -0.00664887F, 0.00579001475F, 0.000572252F, -0.003175F,
      -0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, -0.000177484F, 0.00934084F, 0.0141276F,
      -0.0144031495F, -0.00389864016F, -0.011117F, -0.00588578498F, -0.0165325F,
      -0.0165592F, -0.00916271F, -0.003175F, 0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F,
      0.0127682F, -0.00700996F, -0.0151843F, -0.0108853F, 0.00605461514F,
      -0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.0141244F, 0.00925582F,
      -0.00168985664F, 0.00248458F, -0.0162928F, 0.0126669F, -0.0128008F,
      0.0140089F, -0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F,
      0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F,
      0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F,
      -0.0120901F, -0.00114389F, 0.0119915F, -0.012613F, 0.0160961F, 0.015687F,
      -0.00545855F, -0.00552341F, -0.0108262F, -0.0114873F, -0.0115788F,
      -0.00657816511F, 0.00311216F, 0.0102292F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F,
      0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.00307076F, 0.000909629F, -0.008369F,
      0.0114856F, -0.0134884F, -0.00614362F, 0.00557698496F, 0.0127245F,
      -0.0137044F, 0.0128486501F, -0.0165569F, 0.0165554F, -0.00660038F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      -0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F,
      0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.00682015F, -0.0148881F, -0.00767043F, 0.00892873F,
      0.0129086F, -0.0114115F, 0.000615262F, 0.0118813F, 0.00350818F,
      -0.00369014F, 0.00321334F, 0.0099934F, -0.00968827F, 0.0165579F,
      0.00688764F, -0.00211998F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.00203946F, 0.00154314F, -0.0114234F,
      -0.00845290534F, -0.016270034F, 0.013208F, -0.0153757F, -0.00242652F,
      -0.0150915F, -0.016558F, 0.01296735F, -0.0130468F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.0108946F, -0.00559605472F, 0.0065841F,
      0.0113944234F, 0.0132105F, 0.00942814F, -3.21944972E-5F, -0.0157458F,
      0.0130974669F, 0.0116855F, -0.00812098F, 0.00580501463F, 0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      -0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.00240668491F, -0.00667692F,
      0.00261915F, 0.0119585F, -0.0165583F, 0.0130578F, 0.015363F, 0.000653464F,
      -0.0124371F, -0.00729671F, -0.0128785F, 0.0136172F, -0.0125193F,
      0.0150937494F, -0.0117177F, 0.00872717F, -0.003175F, 0.003175F, -0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      -0.003175F, 0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F,
      0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F,
      0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F, -0.00198449F,
      0.00266278F, -0.00602277F, 0.014494733F, -0.0124181F, 0.0137934331F,
      0.0154595F, -0.00483017F, -0.00723065F, -0.0127951F, 0.0158771F,
      0.00457517F, 0.0134179F, -0.003175F, 0.003175F, -0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.0110670496F, -0.0102572F,
      -0.0143551501F, -0.0152014F, -0.00452997F, -0.00766831F, -0.0126188F,
      -0.0163045F, 0.0104273F, -0.0126077F, -0.00627844F, -0.00174548F,
      -0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F,
      0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.00239659F, -0.0152477F, -0.000713861F, 0.00347589F,
      -0.0145826498F, 0.0165571F, -0.0113286F, -0.0084327748F, -0.00670632F,
      0.00332247F, 0.0104594F, -0.00225853F, 0.00266626F, 0.003175F, -0.003175F,
      0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F,
      -0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F,
      0.003175F, -0.000221875F, 0.0113437F, -0.0132082F, 0.0130082F, 0.00206225F,
      -0.00481574F, 0.006663F, 0.0116201F, -0.0122963F, 0.00881242F, 0.0161731F,
      -0.0165582F, -0.00895466F, -0.003175F, -0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F,
      -0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      -0.003175F, 0.003175F, 0.003175F, 0.003175F, 0.003175F, 0.003175F,
      0.003175F, 0.003175F, -0.00964996F, 0.0122393556F, 0.0165566F,
      -0.00949991F, 0.0114007629F, 0.00513333F, 0.0116673F, 0.0125866495F,
      -0.0139445495F, 0.0143593F, -0.00460074F, -0.003175F, -0.003175F,
      -0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F,
      0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F, 0.003175F,
      -0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, -0.0148077F, -0.0142013F, -0.0106166F, 0.00927085F,
      0.0135532F, -0.00075125F, 0.00693182461F, -0.00917821F, 0.014884349F,
      -0.0105693F, 0.0127394F, -0.00546377525F, 0.003175F, -0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, 0.003175F, -0.003175F, -0.00634478F, -0.0104428F, 0.00976438F,
      -0.00531679F, 0.0098642F, 0.00279034F, -0.0111267F, -0.00342494F,
      0.0055844048F, 0.0165594F, -0.0159897F, 0.0123747F, -0.00241485494F,
      0.0036106F, -0.003175F, 0.003175F, -0.003175F, -0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F,
      0.003175F, -0.003175F, 0.00210582F, -0.00367406F, -0.0164119489F,
      0.0146242501F, -0.012064F, -0.00753706F, 0.0134593F, -0.00705808F,
      -0.00160603F, 0.00113535F, 0.003175F, -0.003175F, -0.003175F, -0.003175F,
      0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F,
      -0.003175F, 0.003175F, -0.00742927333F, 0.00723471F, -0.0161466F,
      -0.00224253F, 0.00323610823F, 0.0125169903F, -0.0133296F, 0.0161719F,
      -0.0115316F, 0.00437906478F, 0.003175F, 0.003175F, -0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F,
      0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      -0.003175F, -0.003175F, 0.003175F, -0.003175F, 0.003175F, -0.003175F,
      -0.00253948F, 0.00124094F, -0.011137F, -0.00935437F, 0.0156905502F,
      -0.0143630672F, 0.018491F, -0.0119026F, -0.00597827509F, 0.00676147F,
      -0.0151541F, 0.01466865F, 0.0201463513F, 0.00463765F, 0.0118883336F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F,
      -0.003175F, 0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.003175F, 0.003175F, -0.003175F, -0.003175F,
      -0.003175F, -0.003175F, -0.003175F, 0.003175F, 0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.000360371F, -0.00611648662F, 0.0145515995F,
      -0.0160108F, 0.0165592F, 0.0205203F, -0.014490867F, 0.0135391504F,
      -0.0139432F, -0.00219770498F, 0.00144229F, -0.00776956F, 0.00103633245F,
      0.0148719F, -0.0126050506F, 0.0143035296F, -0.0170372669F, 0.0142884497F,
      -0.00366747985F, -0.00507477F, 0.0109749F, -0.00593739469F, -0.00290338F,
      0.00822929F, -0.000402868F, -0.003175F, 0.003175F, -0.002191F, 0.003175F,
      0.00166445F, 0.003175F, 0.00054118F, 0.00152816F, -0.003175F, -0.00106188F,
      -0.003175F, 0.00266336F, 0.003175F, -0.003175F, -0.003175F, -0.003175F,
      0.003175F, -0.003175F, -0.000312747F, -0.00212817499F, 0.003175F,
      -0.00287445F, 0.00160938F, -0.000670696F, 0.003175F, -0.003175F,
      -0.0019089F, -0.0034742651F, 0.00242819F, -0.0161428F, -0.00529912487F,
      0.00332859F, 0.013755F, -0.0177637339F, 0.0163400248F, 0.0057488F,
      0.00418042F, 0.0133593F, -0.000419619F, 0.0121395F, -0.0185516F,
      0.0201333F, -0.0131206F, -0.00296073F, 0.01372247F, -0.0142254F,
      -0.00654885F, 0.00291679497F, 0.00967553F, 0.00223039F, 0.0147399008F,
      -0.00989991F, 0.0014717F, 0.0118461177F, 0.00595025F, 0.00364150014F,
      0.0107759F, -0.0145965F, -0.00338167488F, 0.0101645F, -0.0146926F,
      -0.0049444912F, 0.00575673347F, 0.0125841F, 0.0075841F, -0.0131647F,
      0.00634005666F, 0.0123955F, -0.00593816023F, 0.00689783F, 0.0107385F,
      0.00603945F, 0.0122573F, -0.0125643F, -0.00268141F, 0.00356962672F,
      0.0119725F, -0.00466647F, 0.00426005526F, -0.00577181F, 0.00435337331F,
      -0.0139701F, -0.00626569381F, 0.0076118F, 0.013467F, -0.0122178F,
      -0.00195585494F, 0.00725731F, 0.0121609047F, -0.0127802501F, -0.00810346F,
      0.00380485F, 0.00504402F, 0.00990234F, -0.0147093F, -0.00731858F,
      0.00403211F, -0.00645882F, 0.00514304219F, 0.0118528008F, -0.00785721466F,
      0.0033941702F, 0.0117692F, -0.00443727523F, 0.00633497F, 0.0125636F,
      -0.00833126716F, 0.00403103232F, -0.00456049F, 0.00458985474F,
      -0.00657336041F, 0.00437772321F, -0.00566582242F, 0.00829554F,
      -0.00616861973F, 0.01016925F, -0.00910582F, 0.00752626F, 0.0147074F,
      -0.0143444F, -0.00184585F, 0.00603286F, 0.0101554F, -0.00873886F,
      0.00510736508F, 0.0101553705F, -0.00773793645F, 0.00529501F, -0.00249122F,
      0.00285143685F, -7.36793E-5F, -0.0123292F, -0.00971473F, 0.00521069F,
      0.0111133F, -0.00403556507F, -0.00842246F, 0.00194571F, 0.0127633F,
      -0.014791F, -0.0025223F, 0.00359443F, 0.00968282F, -0.00809172F,
      0.00619597F, 0.0146299F, -0.0141001F, -0.00622280687F, -0.0107455F,
      0.00565202767F, 0.0145706F, -0.0134821F, -0.00728637353F, 0.00291065825F,
      0.0115663F, -0.0136131F, -0.0068248203F, 0.00696721F, -0.0130329F,
      -0.00880634692F, -0.00541091245F, 0.00864405F, -0.0122473501F,
      -0.000628194F, 0.00689598F, 0.01124583F, -0.00388523F, 0.0093004F,
      0.0136819F, -0.0134589504F, -0.00928484F, -0.000204072508F, 0.00968371F,
      -0.00911008F, 0.0144896F, -0.0144329F, -0.00709519F, 0.00375720672F,
      -0.0117746F, -0.00461472F, 0.00387194194F, 0.0146014F, -0.00726761F,
      0.00722774491F, -0.0120912F, 0.00115593406F, 0.0116655864F, -0.0112822F,
      -0.0008598F, 0.00792986F, 0.0133342501F, -0.0129304F, -0.00290245982F,
      0.000305317F, -0.0146566F, -0.00272104F, 0.00360663F, 0.0122193F,
      -0.0118355509F, -0.00894597F, 0.00128703169F, -0.0116653F, -0.00693787215F,
      0.00108294177F, -0.00720316265F, 0.00565503F, 0.0117971F, -0.00778787F,
      0.0130499098F, -0.0128687499F, -0.00718719512F, 0.00400607521F, 0.0102219F,
      -0.0198326F, -0.0127722F, 0.00674054F, 0.0192356F, 0.00359317F,
      -0.0118909497F, -0.00191159F, -0.0149014F, -0.00900512F, 0.0120172F,
      0.014787849F, 0.0205095F, -0.0158533677F, 0.015976F, -0.0180023F,
      0.0152944F, 0.0202974F, -0.016684F, 0.0152049F, 0.0132234748F,
      -0.01632425F, 0.0133893667F, -0.0140299331F, 0.0123563F, -0.0146127492F,
      0.0130708497F, -0.0117892334F, 0.0161048F, -0.0160890333F, 0.0129486993F,
      -0.0145009F, -0.0137384F, 0.0145675503F, -0.0140808672F, 0.0114158F,
      -0.0131799F, 0.0145711247F, -0.0162923F, 0.0161512326F, -0.0162626F,
      0.0131896F, -0.0139686F, 0.0137328F, -0.0164273F, 0.0142409252F,
      -0.0124862F, 0.0165467F, 0.0119532F, -0.0143917501F, 0.0128018251F,
      -0.0150966F, 0.0156541504F, -0.0121755507F, 0.01571385F, -0.0159647F,
      -0.0161394F, 0.0165084F, -0.0140702995F, 0.013839649F, 0.0143775502F,
      -0.0138079245F, 0.0129828F, 0.0120289F, -0.0138048334F, 0.0156604499F,
      0.0139975669F, -0.0165935F, 0.0139986007F, 0.0161346F, 0.0137358F,
      0.0143395504F, -0.0158442501F, 0.0120294F, 0.0131507497F, -0.0127580501F,
      -0.0117978F, -0.0129894F, 0.0147184497F, -0.0140828F, -0.0147284F,
      0.0145818498F, -0.0147999339F, 0.0164014F, -0.0142722335F, 0.0165124F,
      -0.0150621F, -0.0130785F, -0.0130772498F, 0.0132847F, -0.0151499F,
      0.0147847496F, 0.0163808018F, -0.0135246497F, 0.016597F, -0.0125934333F,
      0.0161616F, -0.0138924671F, -0.0141772497F, 0.0157616511F, -0.0132367F,
      0.0126161501F, -0.0161573F, -0.0131402F, 0.0129517F, -0.0148268668F,
      0.0152901F, -0.0162348F, 0.0149867339F, -0.0166882593F, 0.0179772489F,
      -0.0165167F, 0.0139680505F, 0.0204913F, -0.00429004F, 0.00426675F,
      -0.0141582005F, -0.00999762F, 0.0194573F, -0.0165612753F, -0.0100634F,
      0.0142163327F, -0.0138856F, -0.00868993F, 0.0155107249F, -0.0164655F,
      0.0116369F, 0.0129191671F, -0.0122660995F, -0.0126608F, 0.0147615205F,
      -0.00717880484F, 0.00611737F, -0.0146222329F, 0.0118112F, -0.0138076833F,
      -0.00841415487F, -0.0162959F, -0.00589934F, 0.0116544F, 0.00872766F,
      0.0103938F, -0.0115924F, -0.00465311F, 0.0157776512F, -0.0129451994F,
      0.0115409F, -0.0122675F, -0.0103609F, 0.0163302F, -0.0153183F, 0.0111739F,
      -0.0146430507F, 0.014768F, -0.0138953496F, 0.0161221F, 0.00429536F,
      0.010401F, -0.013471F, 0.0129535506F, -0.0145346F, -0.00592030492F,
      0.0134811F, -0.00755427033F, 0.00779816F, 0.0113513703F, -0.0129148F,
      0.0126292501F, -0.0147181F, -0.0103527F, 0.00791955F, 0.0131904706F,
      -0.0127207665F, 0.00468559F, 0.0126806498F, -0.0155838F, 0.0136230337F,
      -0.0162226F, 0.00512322F, 0.0132348966F, -0.015449F, -0.0101828F,
      0.0159491F, -0.0141469501F, -0.0114531F, -0.00723119499F, 0.0154449F,
      -0.00879931F, 0.00687159F, 0.0112625F, -0.0145712F, 0.00741383526F,
      -0.014905F, -0.00849214F, 0.0127353F, -0.0080646F, 0.00877011F,
      0.0118232146F, -0.0120378F, 0.0131367F, -0.0137937F, 0.00575915F,
      0.00938283F, -0.0112356F, 0.0106691401F, -0.0123204328F, -0.00870107F,
      0.00739178F, 0.0115869F, -0.016337F, -0.00640513F, 0.0160099F, -0.0119352F,
      0.0142226F, -0.0126012F, -0.00833104458F, 0.00722455F, 0.0148255F,
      -0.0124426F, 0.00682752F, 0.0119761508F, 0.00818106532F, -0.0109936F,
      -0.00827279501F, 0.0101166F, -0.0127713F, 0.0140572246F, -0.0133821F,
      0.0109729F, -0.0123213F, 0.0153932F, -0.0134971F, 0.00557336F,
      0.0142086335F, -0.0116408505F, 0.00665512029F, 0.013348F, -0.0109755F,
      -0.00437736F, 0.00857853F, 0.0118086F, -0.0133442F, -0.00626416F,
      0.00685758F, 0.0138695333F, -0.0154007F, -0.00801262F, 0.00539535517F,
      -0.012907F, -0.00803667493F, -0.0140164F, -0.0142453499F, -0.00763071F,
      0.00455502F, 0.0134197F, -0.0135721499F, -0.00710807F, 0.00807582F,
      0.00487628F, -0.0128851496F, -0.00640605483F, 0.0131789297F, -0.0163447F,
      -0.00478811F, 0.00602315F, 0.0135122966F, -0.0148473F, 0.0135130491F,
      -0.0177398F, 0.00951322F, -0.0107237F, -0.00370394F, 0.00288099F,
      -0.00382011F, 0.00561144F, -0.0132404F, -0.00440663F, 0.0121618398F,
      0.00550826F, -0.00813787F, 0.00112473266F, 0.01052735F, 0.00607114471F,
      0.00460012F, -0.00884571F, 0.00559015479F, -0.00392981293F, 0.00392702967F,
      -0.00762204F, -4.98141E-5F, 0.00640439F, -0.00361876F, -0.000510749F,
      -0.00094649F, 0.00175152498F, 0.010333F, -0.00668331F, -0.0040438124F,
      0.00758227F, 0.00165761949F, -0.00961097F, 0.0040773F, -0.00488957F,
      0.00816695F, 0.000837898F, -0.00354829524F, 0.00271569029F, -0.00357364F,
      0.00520145F, -0.00372548751F, 0.00479719508F, -0.00492004678F, 0.00310388F,
      0.00461656647F, 0.00193386F, -0.00205062516F, 0.00261035F, -0.00906464F,
      0.00253297F, 0.000903972F, -0.00308268424F, 0.00190522F, -0.00406347F,
      0.00332644F, -0.00529853394F, 0.00843304F, -0.00739478F, 0.000444535F,
      -0.00388636F, 0.0021803F, -0.00245862F, 0.00121209F, 0.00506934F,
      0.00972413F, 0.00675948F, -0.0014584F, 0.00443061F, -0.00543192F,
      0.0106076F, 0.00935806F, -0.00997455F, 0.00456085F, -0.00760436524F,
      0.00152279F, -0.00826701F, 0.00748149F, -0.00186892506F, 0.000658621F,
      -0.00298342F, -0.00949676F, 0.00574221276F, -0.00663531665F, 0.000137932F,
      -0.0104277F, 0.00241866568F, -0.00542567484F, -0.00402154F, 0.00862035F,
      0.00531687494F, -0.00796994288F, 0.00150249F, 0.00943861F, 0.00439449F,
      -0.00150856F, 0.00411051F, -0.00502251508F, 0.00197654055F, -0.00922647F,
      0.00534F, -0.00547368F, 0.00714905513F, 0.0138796F, -0.00147746992F,
      0.00590941F, -0.0134774F, -0.00313746F, 0.0104164F, -0.0113005F,
      0.0090423F, 0.0110656F, -0.319868F, -0.3048F, -0.298964322F, -0.299592F,
      -0.288885F, -0.288792F, -0.286068499F, -0.291348F, -0.279865F,
      -0.27643466F, -0.26993F, -0.26521F, -0.272552F, -0.261423498F,
      -0.255879492F, -0.249147F, -0.248120338F, -0.237369F, -0.237915203F,
      -0.236257F, -0.232518494F, -0.227374494F, -0.220756F, -0.213453F,
      -0.208527F, -0.211921498F, -0.20385F, -0.199197F, -0.195197F,
      -0.194440991F, -0.186828256F, -0.18673F, -0.176519F, -0.163623F, -0.16408F,
      -0.169378504F, -0.153135F, -0.160725F, -0.157812F, -0.149451748F,
      -0.152052F, -0.147822F, -0.133422F, -0.137267F, -0.142214507F,
      -0.128215671F, -0.120653495F, -0.112457F, -0.086207F, -0.0915049F,
      -0.0750596F, -0.0773489F, -0.0679505244F, -0.0640590489F, -0.054871F,
      -0.057152234F, -0.0483379327F, -0.0433783978F, -0.039439F, -0.0335505F,
      -0.041381F, -0.0270089507F, -0.0243713F, -0.0140912F, -0.0144646671F,
      -0.00737127F, -0.00540152472F, -0.00090003F, 0.00130112F, 0.0113080852F,
      0.00925794058F, 0.0252777F, 0.0205098F, 0.0171882F, 0.0315914F,
      0.0313332491F, 0.0407079F, 0.0401783F, 0.0552445F, 0.0550541319F,
      0.0649422556F, 0.0710673332F, 0.0823649466F, 0.0856033F, 0.079355F,
      0.0929982F, 0.105842501F, 0.113861F, 0.112721F, 0.121071F, 0.129051F,
      0.13289699F, 0.138011F, 0.142467245F, 0.150342494F, 0.149043F, 0.149326F,
      0.166631F, 0.161751F, 0.176603F, 0.174733F, 0.181487501F, 0.180435508F,
      0.177331F, 0.194882661F, 0.200241506F, 0.200832F, 0.211379F, 0.215144F,
      0.223608F, 0.22183533F, 0.223666504F, 0.229183495F, 0.227634F, 0.234297F,
      0.244511F, 0.242407F, 0.242284F, 0.238607F, 0.252148F, 0.253757F,
      0.250173F, 0.276044F, 0.271819F, 0.27577F, 0.281210482F, 0.280750751F,
      0.283872F, 0.281399488F, 0.292623669F, 0.294359654F, 0.303022325F,
      0.300696343F, 0.30504F, 0.301364F, 0.310765F, 0.313605F, 0.310949F,
      0.325725F, 0.325762F, 0.32465452F, -0.329453F, -0.327041F, -0.3048F,
      -0.3048F, -0.306013495F, -0.301097F, -0.287907F, -0.285456F, -0.279221F,
      -0.278094F, -0.267304F, -0.266598314F, -0.259671F, -0.249601334F,
      -0.251473F, -0.251097322F, -0.236688495F, -0.225193F, -0.230894F,
      -0.226051F, -0.220853F, -0.217871F, -0.21606F, -0.208029F, -0.193602F,
      -0.196659F, -0.197946F, -0.188625246F, -0.187238008F, -0.188252F,
      -0.175182F, -0.178167F, -0.165688F, -0.157756746F, -0.14783451F,
      -0.152716F, -0.135531992F, -0.135118F, -0.118617F, -0.107834503F,
      -0.0988853F, -0.0977227F, -0.0997291F, -0.0899016F, -0.0898419F,
      -0.0876278F, -0.0908869505F, -0.0772740543F, -0.0742083F, -0.0794468522F,
      -0.0679025501F, -0.065874F, -0.0541465F, -0.0543612F, -0.0474131331F,
      -0.0485595651F, -0.0379045494F, -0.039254F, -0.0359544F, -0.0332867F,
      -0.0309399F, -0.0246427F, -0.0167769F, -0.0192022324F, -0.00657584F,
      0.00126707496F, 0.00295800669F, 0.00969517604F, 0.0116631F, 0.0143622F,
      0.0228254497F, 0.0238535F, 0.0330179F, 0.0317717F, 0.0439448F, 0.0545861F,
      0.0549085F, 0.0520008F, 0.0605233F, 0.0709472F, 0.0741948F, 0.0861197F,
      0.0808665F, 0.092312634F, 0.10232F, 0.0988109F, 0.105227F, 0.115311995F,
      0.1095405F, 0.114162F, 0.120601334F, 0.134039491F, 0.133343F, 0.133518502F,
      0.140532F, 0.141319F, 0.142353F, 0.154004F, 0.153143495F, 0.159821987F,
      0.163619F, 0.16021499F, 0.167658F, 0.173531F, 0.168461F, 0.185915F,
      0.183259F, 0.184077F, 0.19694F, 0.202842F, 0.201249F, 0.202143669F,
      0.224372F, 0.218913F, 0.232682F, 0.228601F, 0.2350135F, 0.242037F,
      0.254323F, 0.25658F, 0.261576F, 0.260804653F, 0.274759501F, 0.282901257F,
      0.28378F, 0.292945981F, 0.292458773F, 0.297569F, 0.301091F, 0.312551F,
      0.309507F, 0.32046F, 0.320585F, 0.333075494F, -0.333754F, -0.339987F,
      -0.340225F, -0.332621F, -0.331745F, -0.313818F, -0.29514733F, -0.294178F,
      -0.296129525F, -0.289529979F, -0.291873F, -0.290245324F, -0.284912F,
      -0.279790342F, -0.282059342F, -0.271056F, -0.254814F, -0.257334F,
      -0.258054018F, -0.248378F, -0.247944F, -0.239831239F, -0.236955494F,
      -0.227324754F, -0.228036F, -0.217541009F, -0.213715F, -0.206253F,
      -0.205158666F, -0.190202F, -0.186767F, -0.17845F, -0.17269F, -0.156118989F,
      -0.154391F, -0.150892F, -0.138264507F, -0.139307F, -0.134521499F,
      -0.12289F, -0.127738506F, -0.121369F, -0.116984501F, -0.111341499F,
      -0.108076334F, -0.099531F, -0.0841162F, -0.0892437F, -0.0802294314F,
      -0.0699133F, -0.0687275529F, -0.0563068502F, -0.0598557F, -0.0570799F,
      -0.0510648F, -0.040822465F, -0.024846F, -0.0279549323F, -0.0168929F,
      -0.0100634F, -0.00694760494F, 0.00374303525F, 0.00441971F, 0.0145256F,
      0.00950173F, 0.02055436F, 0.024217099F, 0.0324216634F, 0.0327107236F,
      0.0422644F, 0.046148F, 0.0544652F, 0.0614015F, 0.0595135503F,
      0.0684385672F, 0.0727185F, 0.0850491F, 0.0951953F, 0.0909324F, 0.103345F,
      0.0983206928F, 0.113876F, 0.124204502F, 0.122134671F, 0.129876F, 0.136543F,
      0.133934498F, 0.143129F, 0.13874F, 0.141906008F, 0.156253F, 0.151243597F,
      0.163642F, 0.165894F, 0.16748F, 0.175294489F, 0.184891F, 0.195705F,
      0.203109F, 0.203593F, 0.224662F, 0.22477366F, 0.231575504F, 0.229376F,
      0.241524F, 0.243486F, 0.260298F, 0.264796495F, 0.274364F, 0.270918489F,
      0.277727F, 0.278419F, 0.283328503F, 0.295258671F, 0.287763F, 0.290137F,
      0.306465F, 0.300623655F, 0.326608F, 0.325634F, 0.328074F, 0.33341F,
      0.339494F, 0.338855F, -0.337474F, -0.334956F, -0.341671497F, -0.339833F,
      -0.330069F, -0.328178525F, -0.315604985F, -0.321785F, -0.30438751F,
      -0.304015F, -0.295256644F, -0.300904F, -0.299532503F, -0.285079F,
      -0.28759F, -0.281278F, -0.277822018F, -0.277602732F, -0.281199664F,
      -0.266261F, -0.25673151F, -0.258139F, -0.252005F, -0.247289985F,
      -0.240941674F, -0.23982F, -0.240131F, -0.228576F, -0.216935009F,
      -0.213852F, -0.206726F, -0.205477F, -0.208218F, -0.198946F, -0.1992255F,
      -0.200412F, -0.191654F, -0.189549F, -0.188057333F, -0.179851F, -0.173061F,
      -0.179139495F, -0.167704F, -0.157501504F, -0.159233F, -0.147164196F,
      -0.134956F, -0.139103F, -0.138730675F, -0.136375F, -0.127625331F,
      -0.120734F, -0.105704F, -0.107385F, -0.0968078673F, -0.0987836F,
      -0.0881132F, -0.0875587F, -0.077742F, -0.0770847052F, -0.0818866F,
      -0.079407163F, -0.0725175F, -0.0719809F, -0.0688087046F, -0.0588230975F,
      -0.055008F, -0.0608161464F, -0.0504875F, -0.0477866642F, -0.047585398F,
      -0.0361912251F, -0.0383709F, -0.0389934F, -0.0277655F, -0.023275F,
      -0.0305117F, -0.0166059F, -0.0202148F, -0.0180455F, -0.00535044F,
      -0.00717746F, -0.00264329F, 0.00628787F, -0.000786424F, 0.00619927F,
      0.0163835F, 0.012076973F, 0.0209713F, 0.0239726249F, 0.0333278F,
      0.0337266326F, 0.0306903F, 0.0397408977F, 0.0410588533F, 0.0376759F,
      0.0555223F, 0.0585465021F, 0.0711593479F, 0.0770579F, 0.077159F,
      0.0871114F, 0.0908753F, 0.104308F, 0.106525F, 0.0994015485F, 0.113694333F,
      0.121079F, 0.124872F, 0.129428F, 0.130989F, 0.133926198F, 0.138346F,
      0.141942F, 0.155686F, 0.152461335F, 0.160735F, 0.162798494F, 0.175963491F,
      0.176295F, 0.170134F, 0.172774494F, 0.184597F, 0.182871491F, 0.17966F,
      0.190632F, 0.192005F, 0.201022506F, 0.198815F, 0.213067F, 0.212215498F,
      0.22203F, 0.226556F, 0.230503F, 0.23203966F, 0.243610337F, 0.2406995F,
      0.249380499F, 0.255226672F, 0.266935F, 0.264954F, 0.257619F, 0.272721F,
      0.272535026F, 0.271622F, 0.284455F, 0.284344673F, 0.283340663F, 0.281451F,
      0.296208F, 0.288804F, 0.299176F, 0.299943F, 0.298381984F, 0.297406495F,
      0.313377F, 0.30753F, 0.310729F, 0.310819F, 0.322593F, 0.33279F,
      0.334514499F, 0.331734F, 0.341807F, -0.335321337F, -0.342572F,
      -0.336545497F, -0.329355985F, -0.332196F, -0.330981016F, -0.321275F,
      -0.316607475F, -0.308799982F, -0.312634F, -0.292416513F, -0.281655F,
      -0.279542506F, -0.253152F, -0.248331502F, -0.235464F, -0.226521F,
      -0.229519501F, -0.21884051F, -0.193597F, -0.201164F, -0.189837F,
      -0.172215F, -0.16027F, -0.153589F, -0.14491F, -0.130782F, -0.119854F,
      -0.107163F, -0.108281F, -0.0887083F, -0.0651705F, -0.0716226F, -0.0314142F,
      -0.0106991F, 0.00583139F, 0.0011501F, 0.0187902F, 0.032272853F, 0.0561107F,
      0.0804647F, 0.0894813F, 0.0998567492F, 0.160421F, 0.163145F, 0.184913F,
      0.191274F, 0.203212F, 0.199705511F, 0.226253F, 0.236609F, 0.261708498F,
      0.282885F, 0.296496F, 0.306465983F, 0.312829018F, 0.313899517F,
      0.336181343F, 0.336188F, 0.333768F, 0.338199F, 0.342865F, 0.338250518F,
      -0.338319F, -0.335679F, -0.337401F, -0.334862F, -0.328411F, -0.317989F,
      -0.32009F, -0.312314F, -0.307171673F, -0.303357F, -0.307086F,
      -0.287003338F, -0.286712F, -0.275643F, -0.265639F, -0.263998F, -0.262732F,
      -0.245958F, -0.232695F, -0.175309F, -0.168364331F, -0.149259F,
      -0.13884899F, -0.123469F, -0.0835041F, -0.053318F, -0.0451134F,
      -0.0466779F, -0.0394385F, -0.0264338F, 0.0263466F, 0.0465144F, 0.0426444F,
      0.0513621494F, 0.0747664F, 0.0956947F, 0.121238F, 0.145221F, 0.152078509F,
      0.183661F, 0.199217F, 0.207672F, 0.2836245F, 0.290621F, 0.296983F,
      0.305772F, 0.304529F, 0.312474F, 0.323297F, 0.330417F, 0.333968F,
      0.332071F, -0.338398F, -0.334686F, -0.327950984F, -0.329639F, -0.323967F,
      -0.311164F, -0.305707F, -0.311460972F, -0.298125F, -0.285968F, -0.267917F,
      -0.246406F, -0.23963F, -0.237659F, -0.227995F, -0.216176F, -0.20736F,
      -0.198326F, -0.165698F, -0.168555F, -0.162664F, -0.153091F, -0.148712F,
      -0.12071F, -0.0896156F, -0.0673188F, -0.040852F, -0.0206985F, 0.000762398F,
      0.00102097F, 0.00761345F, 0.0264798F, 0.0248020496F, 0.0706178F,
      0.0712635517F, 0.101334F, 0.103462F, 0.125135F, 0.140155F, 0.152543008F,
      0.174476F, 0.216994F, 0.255812F, 0.254242F, 0.258082F, 0.282671F,
      0.292714F, 0.300728321F, 0.305942F, 0.309052F, 0.321766F, 0.335299F,
      0.329499F, 0.338565F, -0.335418F, -0.333272F, -0.326844F, -0.328374505F,
      -0.317491F, -0.307959497F, -0.306179494F, -0.308846F, -0.289567F,
      -0.258706F, -0.239764F, -0.23160851F, -0.21525F, -0.206742F, -0.187851F,
      -0.174149F, -0.175723F, -0.171666F, -0.165857F, -0.0964568F, -0.0680351F,
      -0.0497928485F, -0.032774F, -0.0176956F, -0.00392577F, -0.00100118597F,
      0.0146733504F, 0.0236241F, 0.0478065F, 0.0586286F, 0.0813614652F,
      0.0917571F, 0.0916365F, 0.11468F, 0.111899F, 0.128793F, 0.149116F,
      0.158578F, 0.169724F, 0.181446F, 0.192508F, 0.197116F, 0.199448F,
      0.202909F, 0.214566F, 0.236514F, 0.238065F, 0.240074992F, 0.271603F,
      0.286358F, 0.305707F, 0.302452505F, 0.314623F, 0.310261F, 0.308957F,
      0.326419F, 0.323250324F, 0.33833F, -0.311204F, -0.311244F, -0.305707F,
      -0.283347F, -0.277702F, -0.245459F, -0.248585F, -0.240536F, -0.224023F,
      -0.221646F, -0.215449F, -0.192733F, -0.150737F, -0.140006F, -0.137255F,
      -0.127339F, -0.0558155F, -0.0461354F, -0.0408279F, -0.0243271F,
      -0.0170101F, -0.000469636F, 0.00237387F, 0.0149533F, 0.054455F, 0.0671435F,
      0.105185501F, 0.114609F, 0.129183F, 0.154484F, 0.148337F, 0.17206499F,
      0.178958F, 0.184619F, 0.190548F, 0.201819509F, 0.214349F, 0.247354F,
      0.273613F, 0.288154F, 0.29845F, 0.313002F, 0.310246F, 0.320474F, 0.336099F,
      -0.338686F, -0.328124672F, -0.307909F, -0.29689F, -0.274671F, -0.27059F,
      -0.20848F, -0.201796F, -0.190607F, -0.177426F, -0.174963F, -0.172161F,
      -0.171984F, -0.151524F, -0.129849F, -0.106195F, -0.100757F, -0.0758467F,
      -0.0746204F, -0.0414463F, 0.0157314F, 0.0226821F, 0.0353258F,
      0.0440769494F, 0.0531701F, 0.0664501F, 0.0886307F, 0.116296F, 0.118468F,
      0.168255F, 0.179089F, 0.19419F, 0.198049F, 0.22905F, 0.232746F, 0.239075F,
      0.256155F, 0.262013F, 0.260067F, 0.274409F, 0.292743F, 0.309677F,
      0.309327F, 0.323979F, 0.320929F, 0.335413F, 0.333372504F, 0.33279F,
      -0.337303F, -0.33733F, -0.332526F, -0.325386F, -0.318674F, -0.321009F,
      -0.30775F, -0.309228F, -0.296159506F, -0.288552F, -0.271219F,
      -0.261882484F, -0.240854F, -0.229986504F, -0.219023F, -0.176152F,
      -0.167235F, -0.16329F, -0.161266F, -0.144234F, -0.1073635F, -0.0913012F,
      -0.081836F, -0.0827422F, -0.0626552F, -0.0403227508F, -0.0378407F,
      0.00897685F, 0.0228058F, 0.0587431F, 0.0699319F, 0.0716576F, 0.0796849504F,
      0.1022055F, 0.102707F, 0.111349F, 0.11243F, 0.123046495F, 0.137498F,
      0.197115F, 0.201276F, 0.19836F, 0.213965505F, 0.229779F, 0.241182F,
      0.283908F, 0.298309F, 0.315862F, 0.313464F, 0.307146F, 0.3134F, 0.3238585F,
      0.322738677F, 0.334712F, -0.333191F, -0.332693F, -0.325295F, -0.317113F,
      -0.316359341F, -0.305706F, -0.311931F, -0.279146F, -0.256909F, -0.210341F,
      -0.195832F, -0.199071F, -0.190417F, -0.150612F, -0.134576F, -0.130289F,
      -0.132313F, -0.0854209F, -0.0907376F, -0.0713941455F, -0.0469680503F,
      -0.00769657F, 0.00721803F, 0.0244425F, 0.0182122F, 0.0347452F, 0.0712983F,
      0.101894F, 0.115599F, 0.127074F, 0.129753F, 0.128201F, 0.146564F,
      0.143695F, 0.230045F, 0.242175F, 0.251274F, 0.264302F, 0.286171F, 0.29845F,
      0.309282F, 0.315161F, 0.323813498F, 0.32996F, 0.338424F, -0.327747524F,
      -0.320408F, -0.32238F, -0.31127F, -0.309042F, -0.296861F, -0.256649F,
      -0.248734F, -0.227575F, -0.207867F, -0.1932F, -0.168856F, -0.147201F,
      -0.136944F, -0.117148F, -0.107619F, -0.0870744F, -0.069126F, -0.0362848F,
      0.00141307F, 0.0128981005F, 0.029254F, 0.0495547F, 0.0527414F,
      0.0955624431F, 0.100004F, 0.125386F, 0.131073F, 0.139693F, 0.142548F,
      0.162664F, 0.179082F, 0.190473F, 0.208229F, 0.217429F, 0.219973505F,
      0.228489F, 0.244358F, 0.243261F, 0.271809F, 0.293329F, 0.29845F, 0.312969F,
      0.319814F, 0.317873F, 0.332479F, 0.334078F, 0.330412F, -0.333699F,
      -0.335807F, -0.33024F, -0.325818F, -0.322592F, -0.316785F, -0.310098F,
      -0.310494F, -0.289241F, -0.274572F, -0.259097F, -0.250762016F, -0.250303F,
      -0.219557F, -0.202933F, -0.20836F, -0.17983F, -0.179745F, -0.156585F,
      -0.160431F, -0.143825501F, -0.137637F, -0.12765801F, -0.131075F,
      -0.112631F, -0.102954F, -0.096775651F, -0.0682707503F, -0.0599467F,
      -0.0454887971F, -0.00950634F, 0.0125877298F, 0.0183221F, 0.041999F,
      0.0605672F, 0.0706829F, 0.0862875F, 0.0871451F, 0.0936798F, 0.103571996F,
      0.106714F, 0.118956F, 0.13133F, 0.150971F, 0.174413F, 0.185985F, 0.190211F,
      0.214953F, 0.218971F, 0.232273F, 0.227963F, 0.243622512F, 0.256923F,
      0.261838F, 0.294451F, 0.303288F, 0.305702F, 0.310285F, 0.309438F,
      0.316668F, 0.323178F, 0.335256F, 0.33868F, -0.327435017F, -0.315128505F,
      -0.309056F, -0.288003F, -0.274912F, -0.266377F, -0.258096F, -0.251233F,
      -0.227529496F, -0.164248F, -0.161416F, -0.146198F, -0.111846F, -0.0696341F,
      -0.0713971F, -0.0500758F, -0.0413094F, -0.0044646F, -0.0111487F,
      0.00155863F, 0.0116355F, 0.0308526494F, 0.0393288F, 0.06070875F,
      0.0838167F, 0.107001F, 0.114293F, 0.123492F, 0.128323F, 0.143059F,
      0.179414988F, 0.202816F, 0.202715F, 0.211311F, 0.218162F, 0.230503F,
      0.239646F, 0.247847F, 0.264533F, 0.277869F, 0.28857F, 0.297678F, 0.310929F,
      0.314425F, 0.321755F, 0.335956F, 0.334845F, 0.330832481F, -0.336589515F,
      -0.329554021F, -0.329958F, -0.320457F, -0.313227F, -0.3059825F, -0.310804F,
      -0.276292F, -0.274247F, -0.259223F, -0.259543F, -0.230032504F, -0.218408F,
      -0.199312F, -0.201477F, -0.180397F, -0.164087F, -0.159084F, -0.152396F,
      -0.135849F, -0.11329F, -0.114111F, -0.111315F, -0.0947719F, -0.0805298F,
      -0.0525189F, -0.0413364023F, -0.0166743F, -0.00520956F, 0.00390935F,
      0.0300439F, 0.0391735F, 0.0488128F, 0.0547579F, 0.0571911F, 0.0719076F,
      0.0964438F, 0.110609F, 0.109779F, 0.131428F, 0.127179F, 0.145419F,
      0.149936F, 0.162308F, 0.182248F, 0.193527F, 0.198705F, 0.230067F,
      0.251818F, 0.257327F, 0.281794F, 0.286831F, 0.295169F, 0.313433F, 0.31026F,
      0.309755F, 0.312741667F, 0.325903F, 0.334122F, 0.333851F, 0.331986F,
      -0.320788F, -0.322268F, -0.309827F, -0.261362F, -0.250813F, -0.235018F,
      -0.229724F, -0.231771F, -0.222135F, -0.193617F, -0.202099F, -0.189149F,
      -0.178009F, -0.132142F, -0.10571F, -0.0931584F, -0.0847579F,
      -0.0915573537F, -0.0783406496F, -0.0634483F, -0.0638376F, -0.0628221F,
      -0.062526F, -0.0476280525F, -0.0349599F, -0.0288611F, -0.0134357F,
      -0.00862893F, 0.00271296F, 0.00277461F, 0.0178545F, 0.0292614F,
      0.0330657512F, 0.0604607F, 0.0945691F, 0.105103F, 0.111697F, 0.122177F,
      0.128937F, 0.147511F, 0.158565F, 0.157889F, 0.168444F, 0.1716F, 0.185451F,
      0.181726F, 0.204759F, 0.20848F, 0.214842F, 0.231985F, 0.250898F, 0.294613F,
      0.310539F, 0.319292F, 0.335359F, 0.332510501F, 0.32901F, -0.336366504F,
      -0.324911505F, -0.330801F, -0.31791F, -0.309749F, -0.309059739F,
      -0.309054F, -0.284535F, -0.282206F, -0.259411F, -0.245804F, -0.238248F,
      -0.21892F, -0.212512F, -0.18358F, -0.176792F, -0.172472F, -0.172359F,
      -0.143229F, -0.130099F, -0.118131F, -0.057586F, -0.0576035F, -0.0232913F,
      0.0123541F, 0.0134450793F, 0.0211846F, 0.0486345F, 0.0594055F,
      0.0804828554F, 0.0937368F, 0.0962016F, 0.10417F, 0.114978F, 0.121033F,
      0.161866F, 0.160986F, 0.179176F, 0.190243F, 0.233937F, 0.272794485F,
      0.272381F, 0.279646F, 0.29845F, 0.306277F, 0.313315F, 0.311548F, 0.322963F,
      0.319062F, 0.332909F, 0.335554F, -0.337502F, -0.314946F, -0.315525F,
      -0.306741F, -0.296256F, -0.284281F, -0.274356F, -0.270866F, -0.26051F,
      -0.24515F, -0.217384F, -0.216589659F, -0.206878F, -0.198571F, -0.165174F,
      -0.155489F, -0.137833F, -0.113081F, -0.111206F, -0.0698469F, -0.0308868F,
      -0.0195952F, 0.00964011F, 0.0425191F, 0.0704781F, 0.136128F, 0.152330011F,
      0.150891F, 0.159492F, 0.18175F, 0.216951F, 0.234925F, 0.236967F, 0.246985F,
      0.261893F, 0.266533F, 0.268584F, 0.290539F, 0.311278F, 0.32283625F,
      0.333456F, 0.332231343F, 0.33741F, -0.336584508F, -0.336115F,
      -0.330161512F, -0.317885F, -0.315618F, -0.311367F, -0.307420492F,
      -0.298135F, -0.276629508F, -0.26438F, -0.213184F, -0.210081F, -0.184719F,
      -0.183797F, -0.177233502F, -0.171152F, -0.146596F, -0.129466F, -0.107377F,
      -0.0965042F, -0.0992261469F, -0.090805F, -0.0843545496F, -0.0750666F,
      -0.0704411F, -0.0655509F, -0.0608184F, -0.0492507964F, -0.0190536F,
      -0.00597911F, 0.0332997F, 0.08662F, 0.0862726F, 0.0875178F, 0.102593F,
      0.112229F, 0.11935F, 0.136717F, 0.14258F, 0.151429F, 0.162513331F,
      0.176694F, 0.18626F, 0.191723496F, 0.207967F, 0.226335F, 0.221687496F,
      0.237953F, 0.24017033F, 0.248238F, 0.265094489F, 0.26016F, 0.270175F,
      0.294617F, 0.309055F, 0.317773F, 0.333817F, 0.331157506F, -0.337188F,
      -0.32503F, -0.326149523F, -0.313858F, -0.296117F, -0.293412F, -0.285999F,
      -0.254633F, -0.256914F, -0.238627F, -0.227271F, -0.218796F, -0.217502F,
      -0.206847F, -0.202883F, -0.200219F, -0.177992F, -0.158282503F, -0.15513F,
      -0.13537401F, -0.108387F, -0.0994006F, -0.0558069F, -0.0627603F,
      -0.0496897474F, -0.0350964F, -0.0393819F, -0.0324423F, -0.0280383F,
      -0.00309924F, 0.0134705F, 0.0185971F, 0.0568113F, 0.0645497F, 0.0691355F,
      0.0732529F, 0.0814096F, 0.0962623F, 0.122751F, 0.144693F, 0.156902F,
      0.158625F, 0.1686F, 0.238994F, 0.254156F, 0.260984F, 0.325457F, 0.327305F,
      0.336201489F, 0.338811F, -0.336137F, -0.32306F, -0.272708F, -0.264903F,
      -0.260346F, -0.24872F, -0.234975F, -0.226823F, -0.214798F, -0.199346F,
      -0.182435F, -0.17118F, -0.156737F, -0.129546F, -0.118616499F, -0.105629F,
      -0.108858F, -0.0777634531F, -0.0693983F, -0.0582304F, -0.0434623F,
      -0.0235358F, -0.0246033F, -0.0203562F, 0.0109168906F, 0.0118876994F,
      0.0245758F, 0.0233229F, 0.0274393F, 0.0390266F, 0.0587822F, 0.0658677F,
      0.0718488F, 0.074169F, 0.110918F, 0.148505F, 0.161154F, 0.169411F,
      0.194867F, 0.191456333F, 0.197805F, 0.266126F, 0.271716505F, 0.277226F,
      0.294F, 0.309055F, 0.312192321F, 0.334600985F, 0.332194F, 0.334069F,
      0.338773F, -0.334578F, -0.335278F, -0.321736F, -0.31796F, -0.310135484F,
      -0.306124F, -0.29845F, -0.285644501F, -0.267341495F, -0.250506F,
      -0.240658F, -0.221931F, -0.209393F, -0.191768F, -0.176943F, -0.167512F,
      -0.139635F, -0.10812F, -0.097328268F, -0.0880994F, -0.0797361F,
      -0.0692482F, -0.0696928F, -0.024322F, -0.0212181F, -0.022836F,
      0.0241567492F, 0.0324235F, 0.0394592F, 0.0624437F, 0.0744849F,
      0.0835844427F, 0.0968418F, 0.0978775F, 0.116014F, 0.124796F, 0.12326F,
      0.146669F, 0.182388F, 0.204397F, 0.206574F, 0.211393F, 0.231157F,
      0.236346F, 0.246909F, 0.25069052F, 0.294532F, 0.297903F, 0.310554F,
      0.314068496F, 0.318324F, 0.335475F, 0.33882F, -0.336457F, -0.327871F,
      -0.317809F, -0.306041F, -0.310105503F, -0.311036F, -0.290731F, -0.265875F,
      -0.271991F, -0.253681F, -0.233852F, -0.229522F, -0.218985F, -0.221021F,
      -0.205338F, -0.183454F, -0.171607F, -0.1695645F, -0.15489F, -0.12747F,
      -0.100596897F, -0.07547535F, -0.0681486F, -0.0568994F, -0.0606729F,
      -0.0498689F, -0.0383095F, 0.0187819F, 0.0267257F, 0.0314403F, 0.0417099F,
      0.0662883F, 0.0606127F, 0.0968526F, 0.112036496F, 0.107505F, 0.135208F,
      0.149584F, 0.173672F, 0.195075F, 0.200169F, 0.212469F, 0.218853F,
      0.234684F, 0.243463F, 0.25813F, 0.283019F, 0.289691F, 0.292061508F,
      0.330044F, 0.334916F, 0.332677484F, -0.317902F, -0.317982F, -0.308586508F,
      -0.309914F, -0.288328F, -0.281091F, -0.267282F, -0.266800493F, -0.257585F,
      -0.255244F, -0.229638F, -0.217813F, -0.213777F, -0.19608F, -0.184007F,
      -0.179603F, -0.163284F, -0.153154F, -0.146739F, -0.138181F, -0.133382F,
      -0.12155F, -0.10614F, -0.0653014F, -0.0185406506F, -0.00428277F,
      0.0143553F, 0.00888625F, 0.0239414F, 0.0328953F, 0.0581352F, 0.0707005F,
      0.103071F, 0.113233F, 0.144872F, 0.145344F, 0.151831F, 0.175758F,
      0.215887F, 0.234321F, 0.25385952F, 0.28146F, 0.28267F, 0.291046F,
      0.303473F, 0.315279F, 0.313442F, 0.322826505F, 0.332461F, 0.336855F,
      0.328111F, -0.334588F, -0.332964F, -0.313404F, -0.31587851F, -0.309059F,
      -0.307681322F, -0.309545F, -0.29385F, -0.285406F, -0.278467506F,
      -0.279592F, -0.272482F, -0.272405F, -0.250105F, -0.237032F, -0.223840505F,
      -0.219094008F, -0.20757249F, -0.193465F, -0.185791F, -0.188188F,
      -0.175307F, -0.166246F, -0.14667F, -0.134992F, -0.109362F, -0.108026505F,
      -0.0746479F, -0.0759562F, -0.0651623F, -0.0716295F, -0.0572356F,
      -0.0617267F, -0.0437038F, -0.041058F, -0.00541314F, -0.00103285F,
      0.0723552F, 0.0799687F, 0.0936314F, 0.110973F, 0.115902F, 0.12876F,
      0.160789F, 0.169765F, 0.187952F, 0.212437F, 0.219141F, 0.24405F, 0.250074F,
      0.249700502F, 0.269928F, 0.280169F, 0.29845F, 0.310297F, 0.310746F,
      0.32199651F, 0.3309035F, -0.335472F, -0.336322486F, -0.329524F, -0.309814F,
      -0.307507515F, -0.278034F, -0.258974F, -0.250584F, -0.240665495F,
      -0.224391505F, -0.215028F, -0.21129F, -0.180832F, -0.175599F, -0.164659F,
      -0.168945F, -0.15857F, -0.159614F, -0.146984F, -0.137416F, -0.132813F,
      -0.107082F, -0.0993812F, -0.0765333F, -0.0211381F, -0.0185221F,
      -0.0121238F, 0.0151231F, 0.0253251F, 0.0219256505F, 0.0320285F, 0.0411917F,
      0.043601498F, 0.0678854F, 0.0726963F, 0.113359F, 0.114931F, 0.130574F,
      0.168077F, 0.172936663F, 0.177595F, 0.184249F, 0.197731F, 0.237687F,
      0.306053519F, 0.310004F, 0.308114F, 0.319278F, 0.335947F, -0.338818F,
      -0.33412F, -0.332834F, -0.315228F, -0.312485F, -0.308158517F, -0.296332F,
      -0.292172F, -0.272596F, -0.256208F, -0.257126F, -0.223625F, -0.22283F,
      -0.215557F, -0.152929F, -0.117668502F, -0.118498F, -0.1041435F,
      -0.0777303F, -0.0453562F, -0.0181502514F, -0.0143597F, -0.00641748F,
      0.00908203F, 0.0176455F, 0.0359547F, 0.04399845F, 0.0606402978F, 0.102837F,
      0.156696F, 0.173108F, 0.180987F, 0.207015F, 0.19966951F, 0.236445F,
      0.272826F, 0.289963F, 0.306158F, 0.307993F, 0.313523F, 0.326239F,
      0.330198F, 0.335472F, 0.33015F, 0.338782F, -0.337722F, -0.325919F,
      -0.331143F, -0.327553F, -0.318218F, -0.319774F, -0.308623016F, -0.311243F,
      -0.283727F, -0.279479F, -0.275449F, -0.256232F, -0.25766F, -0.233085F,
      -0.209705F, -0.187686F, -0.16374F, -0.17205F, -0.155976F, -0.127205F,
      -0.113759F, -0.104578F, -0.0846588F, -0.063282F, -0.0672159F, -0.0586362F,
      -0.0487238F, -0.0282921F, -0.00692186F, 0.00899104F, 0.0222310014F,
      0.0379175F, 0.0603114F, 0.115119F, 0.11903F, 0.130405009F, 0.140473F,
      0.149775F, 0.175284F, 0.196195F, 0.23601F, 0.251221F, 0.274473F, 0.279171F,
      0.301699519F, 0.307983F, 0.307867F, 0.319097519F, 0.334766F, -0.338775F,
      -0.337092519F, -0.325943F, -0.327172F, -0.321091F, -0.318526F, -0.311391F,
      -0.294329F, -0.285322F, -0.275344F, -0.273029F, -0.230174F, -0.218659F,
      -0.198464498F, -0.198779F, -0.180852F, -0.158111989F, -0.152475F,
      -0.146862F, -0.135245F, -0.125832F, -0.115377F, -0.100561053F, -0.0831262F,
      -0.06804F, -0.0564425F, -0.0511735491F, -0.0173105F, -0.00409841F,
      -0.00911024F, 0.0147736F, 0.0242281F, 0.0251784F, 0.0345087F, 0.0303452F,
      0.0553292F, 0.098394F, 0.123852F, 0.117785F, 0.129784F, 0.13772F,
      0.137864F, 0.151421F, 0.160785F, 0.186924F, 0.199841F, 0.233502507F,
      0.230387F, 0.242553F, 0.268771F, 0.276756F, 0.290683985F, 0.307012F,
      0.335464F, 0.330786F, 0.338713F, 0.338284492F, -0.337183F, -0.318821F,
      -0.315118F, -0.307492495F, -0.295555F, -0.264433F, -0.241187F, -0.226581F,
      -0.211562F, -0.204898F, -0.197741F, -0.197428F, -0.154173F, -0.0679689273F,
      -0.0573927F, -0.0609288F, -0.0463527F, -0.0146593F, 0.00621338F,
      0.0402509F, 0.0627657F, 0.0721329F, 0.0827095F, 0.083425F, 0.103796F,
      0.102291F, 0.114022F, 0.156299F, 0.166771F, 0.172836512F, 0.170532F,
      0.20533F, 0.2405435F, 0.248448F, 0.278581F, 0.289123F, 0.295699F, 0.29845F,
      0.306734F, 0.31137F, 0.318565F, 0.327964F, 0.338478F, -0.338648498F,
      -0.323438F, -0.3132F, -0.313188F, -0.305715F, -0.309186F, -0.263477F,
      -0.257436F, -0.237441F, -0.208941F, -0.195212F, -0.113328F, -0.122058F,
      -0.100311F, -0.101079F, -0.0786547512F, -0.0759409F, -0.0677991F,
      -0.0547991F, -0.052325F, -0.0429413F, -0.02753135F, -0.0067845434F,
      -0.00219342F, 0.00992912F, 0.0267783F, 0.0301051F, 0.0577014F, 0.068413F,
      0.0675755F, 0.0829784F, 0.0933356F, 0.109639F, 0.175212F, 0.190242F,
      0.207224F, 0.220837F, 0.241731F, 0.255597F, 0.250622F, 0.264816F,
      0.273162F, 0.282412F, 0.29845F, 0.312871516F, 0.32375735F, 0.336698F,
      0.334240317F, -0.333036F, -0.329529F, -0.310357F, -0.305836F, -0.292703F,
      -0.28439F, -0.254067F, -0.232766F, -0.227742F, -0.204821F, -0.199382F,
      -0.17710349F, -0.169785F, -0.158519F, -0.141438F, -0.135786F, -0.123471F,
      -0.113953F, -0.104107F, -0.0968814492F, -0.0886751562F, -0.0771848F,
      -0.0568053F, -0.0466694981F, -0.0295996F, -0.00140512F, 0.00310948F,
      0.0135355F, 0.0326929502F, 0.03529F, 0.0423805F, 0.0425840504F, 0.0576779F,
      0.0599035658F, 0.0827616F, 0.0895774F, 0.0993942469F, 0.121916F, 0.149422F,
      0.172338992F, 0.186595F, 0.181432F, 0.201278508F, 0.211069494F,
      0.24149701F, 0.241246F, 0.305704F, 0.310029F, 0.329458F, 0.330687F,
      0.335468F, 0.338356018F, 0.337355F, -0.334792495F, -0.335361F, -0.331497F,
      -0.32709F, -0.305709F, -0.29211998F, -0.237662F, -0.225912F, -0.223544F,
      -0.216152F, -0.214457F, -0.205174F, -0.211235F, -0.195457F, -0.176158F,
      -0.166511F, -0.160022F, -0.136411F, -0.127922F, -0.107821F, -0.0955476F,
      -0.0882143F, -0.0611328F, -0.0569639F, -0.0282153F, -0.0272098F,
      -0.0158373F, -0.00779244F, 0.00972518F, 0.0208372F, 0.0174623F, 0.042549F,
      0.0419087484F, 0.0730726F, 0.113062F, 0.11989F, 0.144954F, 0.155565F,
      0.160656F, 0.170234F, 0.184293F, 0.19091F, 0.205825F, 0.212762F, 0.246641F,
      0.248205F, 0.253758F, 0.259338F, 0.269763F, 0.295479F, 0.309162F,
      0.309222F, 0.31726F, 0.334924F, 0.338801F, -0.334096F, -0.335216F,
      -0.335662F, -0.331126F, -0.318421F, -0.311182022F, -0.311013F, -0.311689F,
      -0.254815F, -0.238975F, -0.227813F, -0.212499F, -0.195327F, -0.17618F,
      -0.17697F, -0.171619F, -0.166634336F, -0.156145334F, -0.139752F,
      -0.126497F, -0.11509F, -0.0759184F, -0.0273424014F, 0.000588119F,
      0.044257F, 0.0596821979F, 0.0643642F, 0.069843635F, 0.0691626F, 0.0878275F,
      0.102544F, 0.102144F, 0.143624F, 0.153127F, 0.174995F, 0.187044F,
      0.185101F, 0.189106F, 0.192968011F, 0.203403F, 0.19751F, 0.221523F,
      0.252599508F, 0.27642F, 0.295764327F, 0.306419F, 0.315936F, 0.308085F,
      0.310960501F, 0.329493016F, 0.338821F, -0.320056349F, -0.320629F,
      -0.306591F, -0.312063F, -0.278213501F, -0.265185F, -0.259225F,
      -0.237869501F, -0.225147F, -0.205376F, -0.205427F, -0.196763501F,
      -0.199099F, -0.187662497F, -0.174408F, -0.168512F, -0.153397F, -0.148706F,
      -0.139593F, -0.128651F, -0.119281F, -0.0938669F, -0.0741788F, -0.0679336F,
      -0.0575825F, -0.0400813F, -0.0173048489F, -0.0222894512F, 0.00438331F,
      0.0131059F, 0.0261115F, 0.0391949F, 0.0612129495F, 0.0845872F,
      0.103324994F, 0.111846F, 0.116851F, 0.13408F, 0.131557F, 0.144352F,
      0.147505F, 0.168246F, 0.206923F, 0.210667F, 0.271776F, 0.291989F,
      0.306809F, 0.303626329F, 0.314559F, 0.314215F, 0.326629F, 0.322429508F,
      0.33547F, -0.333610475F, -0.33745F, -0.330851F, -0.33058F, -0.326395F,
      -0.317588F, -0.310449481F, -0.286225975F, -0.277052F, -0.27472F,
      -0.249395F, -0.22347F, -0.211589F, -0.180529F, -0.168673F, -0.160707F,
      -0.148097992F, -0.144026F, -0.129849F, -0.121185F, -0.0987497F,
      -0.0481225F, -0.0269791F, -0.0281966664F, -0.0109898F, 0.00815752F,
      0.0195813F, 0.0292973F, 0.0422445F, 0.0480577F, 0.0784556F, 0.115808F,
      0.132600486F, 0.174992F, 0.189883F, 0.195129F, 0.202021F, 0.207116F,
      0.210501328F, 0.226504F, 0.262341F, 0.297317F, 0.311503F, 0.325148F,
      0.329807F, -0.337338F, -0.325333F, -0.331376F, -0.321292F, -0.322186F,
      -0.309779F, -0.309488F, -0.291564F, -0.26374352F, -0.259542674F, -0.22722F,
      -0.215387F, -0.209106F, -0.175191F, -0.172F, -0.135441497F, -0.122642F,
      -0.100007F, -0.0794256F, -0.0508629F, -0.0132497F, -0.00462705F,
      0.0110769F, 0.013797F, 0.0336109474F, 0.0452643F, 0.0497098491F,
      0.0510583F, 0.0850214F, 0.101992F, 0.112849F, 0.107221F, 0.12154F,
      0.142512F, 0.159022F, 0.172766F, 0.175912F, 0.185056F, 0.182435F,
      0.211087F, 0.231926F, 0.236679F, 0.241317511F, 0.2447505F, 0.274782F,
      0.28834F, 0.297672F, 0.312433F, 0.310269475F, 0.313936F, 0.332923F,
      0.338818F, -0.335097F, -0.337329F, -0.313434F, -0.320139F, -0.307235F,
      -0.312456F, -0.296996F, -0.29845F, -0.285977F, -0.279978F, -0.213408F,
      -0.203368F, -0.199155F, -0.188903F, -0.177997F, -0.170656F, -0.167148F,
      -0.162856F, -0.160168F, -0.137741F, -0.12911F, -0.116656F, -0.118262F,
      -0.109184504F, -0.104782F, -0.0986722559F, -0.0883825F, -0.0766174F,
      -0.0690049529F, -0.044288F, -0.0394011512F, -0.0256257504F,
      -0.00439381506F, 0.00265508494F, 0.00703466F, 0.0260634F, 0.0449238F,
      0.0430525F, 0.0487816483F, 0.0648601F, 0.0660787F, 0.087042F, 0.114676F,
      0.117989F, 0.123633F, 0.142489329F, 0.14041F, 0.150785F, 0.160889F,
      0.244087F, 0.255325496F, 0.266246F, 0.275358F, 0.316361F, 0.321478F,
      0.317362F, 0.33527F, -0.330492F, -0.332486F, -0.317670763F, -0.318434F,
      -0.305792F, -0.308559656F, -0.285731F, -0.281022F, -0.213974F, -0.208486F,
      -0.191766F, -0.191328F, -0.158826F, -0.105571F, -0.0910722F, -0.0759885F,
      -0.0654904F, -0.0659350306F, -0.060157F, -0.0582022F, -0.0433283F,
      -0.0210119F, -0.00479001F, 0.0194771F, 0.050758F, 0.0741478F, 0.0882485F,
      0.11437F, 0.128934F, 0.142888F, 0.157184F, 0.193515F, 0.197784F, 0.21348F,
      0.218387F, 0.221865F, 0.242882F, 0.260101F, 0.266938F, 0.283608F,
      0.305992484F, 0.317092F, 0.307737F, 0.314708F, 0.325505F, 0.319473F,
      0.328675F, 0.335473F, 0.3346F, -0.33473F, -0.335033F, -0.327115F,
      -0.33228F, -0.326278508F, -0.314877F, -0.315736F, -0.310709F, -0.297679F,
      -0.292814F, -0.261684F, -0.258054F, -0.251629F, -0.247748F, -0.20067F,
      -0.195361F, -0.186412334F, -0.177923F, -0.163063F, -0.159997F, -0.146099F,
      -0.136422F, -0.140463F, -0.121827F, -0.0908109F, -0.0832234F, -0.0724125F,
      -0.0565642F, -0.0482416F, -0.0340528F, -0.0268626F, -0.0162262F,
      0.0112094348F, 0.0180364F, 0.0352395512F, 0.0502236F, 0.0867324F,
      0.0929951295F, 0.105153F, 0.127062F, 0.129115492F, 0.138877988F, 0.153271F,
      0.199288F, 0.220258668F, 0.232641503F, 0.229225F, 0.238358F, 0.249518F,
      0.273312F, 0.275712F, 0.285845F, 0.291836F, 0.306102F, 0.311844F,
      0.307696F, 0.317417F, 0.320660651F, 0.335622F, -0.334957F, -0.325792F,
      -0.3321F, -0.31703198F, -0.306694F, -0.306449F, -0.288036F, -0.268578F,
      -0.254321F, -0.249401F, -0.227118F, -0.205083F, -0.179554F, -0.158353F,
      -0.13709F, -0.0986561F, -0.08965F, -0.0683879F, -0.0536373F, -0.0477265F,
      -0.0352975F, -0.0237732F, -0.0138224F, 0.00587115F, 0.0339493F, 0.0524499F,
      0.0867417F, 0.0936231F, 0.0966129F, 0.118102F, 0.132204F, 0.158456F,
      0.186778F, 0.192256F, 0.197329F, 0.219418F, 0.25832F, 0.274437F, 0.288421F,
      0.291513F, 0.316877F, 0.309058F, 0.311196F, 0.309457F, 0.32077F, -0.33788F,
      -0.330083489F, -0.317384F, -0.320813F, -0.309768F, -0.280946493F,
      -0.26817F, -0.238209F, -0.198947F, -0.187669F, -0.156174F, -0.147565F,
      -0.136038989F, -0.128017F, -0.122559F, -0.0753509F, -0.0507637F,
      -0.051687F, -0.0350235328F, -0.00782536F, 0.000661485F, -0.00237092F,
      0.0105766747F, 0.0121921F, 0.0228578504F, 0.0269591F, 0.0353981F,
      0.0419355482F, 0.0565118F, 0.0662286F, 0.0614346F, 0.0892296F, 0.0898871F,
      0.103866F, 0.10156F, 0.112574F, 0.118174F, 0.130403F, 0.137438F, 0.143537F,
      0.164294F, 0.168842F, 0.182908F, 0.207246F, 0.221636F, 0.238398F,
      0.249819F, 0.271991F, 0.284712F, 0.296776F, 0.309372F, 0.309343F,
      0.317033F, 0.31719F, 0.32372F, 0.33253F, 0.335036F, 0.337487F, -0.329622F,
      -0.317538F, -0.321568F, -0.286872F, -0.282424F, -0.260109F, -0.229226F,
      -0.216037512F, -0.220115F, -0.207869336F, -0.205666F, -0.185455F,
      -0.188542992F, -0.175007492F, -0.180503F, -0.168873F, -0.156952F,
      -0.148634F, -0.128883F, -0.126276493F, -0.121799F, -0.107693F, -0.101948F,
      -0.0872484F, -0.0887824446F, -0.0675889F, -0.065922F, -0.0595159F,
      -0.0491509F, -0.0385401F, 0.00526258F, 0.0151535F, 0.0218689F, 0.028005F,
      0.04401F, 0.0479216F, 0.0475022F, 0.0724277F, 0.073355F, 0.0848525F,
      0.0887206F, 0.10393F, 0.102488F, 0.140691F, 0.139191508F, 0.160853F,
      0.186424F, 0.210369661F, 0.222093F, 0.237244F, 0.251637F, 0.262979656F,
      0.267719F, 0.285132F, 0.306633F, 0.312159F, 0.320309F, 0.334302F,
      -0.336928F, -0.33772F, -0.327903509F, -0.307417F, -0.283141F, -0.256461F,
      -0.246084511F, -0.231632F, -0.221393F, -0.213042F, -0.208361F, -0.193579F,
      -0.19678F, -0.192012F, -0.18582F, -0.18002F, -0.169723511F, -0.166077F,
      -0.149503F, -0.113852F, -0.109740004F, -0.0868437F, -0.0745179F,
      -0.0604124F, -0.0355649F, -0.029808F, -0.0188992F, -0.0224664F, 0.016263F,
      0.0415654F, 0.0532788F, 0.0818325058F, 0.0880354F, 0.100758F, 0.15502F,
      0.154706F, 0.157466F, 0.162389F, 0.201143F, 0.205905497F, 0.238033F,
      0.255188F, 0.266752F, 0.279343F, 0.289619F, 0.306383F, 0.311327F,
      0.315931F, 0.32631F, 0.32661F, 0.337496F, -0.336621F, -0.325579524F,
      -0.319287F, -0.309898F, -0.283564F, -0.266099F, -0.265208F, -0.249292F,
      -0.231536F, -0.216832F, -0.168004F, -0.162303F, -0.150619F, -0.152471F,
      -0.128555F, -0.0919108F, -0.0753298F, -0.0643956512F, -0.0378257F,
      -0.0248052F, -0.0176288F, -0.0150459F, -0.00669973F, -0.0076029F,
      -0.00143337F, 0.00360569F, 0.0178104F, 0.0328271F, 0.0415602F, 0.0409273F,
      0.0480162F, 0.089032F, 0.106616F, 0.108957F, 0.111371F, 0.122036F,
      0.127503F, 0.173301011F, 0.177534F, 0.210664F, 0.218F, 0.220643F,
      0.2438135F, 0.255776F, 0.257283F, 0.282254517F, 0.280441523F, 0.294156F,
      0.309709F, 0.309689F, 0.323038F, 0.335469F, 0.338529F, -0.336306512F,
      -0.333187F, -0.255444F, -0.239651F, -0.211083F, -0.193477F, -0.187394F,
      -0.176026F, -0.163193F, -0.166655F, -0.158887F, -0.145934F, -0.148577F,
      -0.138409F, -0.120307F, -0.102382F, -0.0853282511F, -0.076791F,
      -0.0230426F, -0.0183068F, -0.00383657F, 0.00273046F, 0.0265241F,
      0.0328904F, 0.0522487F, 0.0597042F, 0.0641291F, 0.0685755F, 0.0953391F,
      0.114583F, 0.115376502F, 0.13164F, 0.14647F, 0.140233502F, 0.165487F,
      0.162286F, 0.181457F, 0.18831F, 0.206285F, 0.219856F, 0.225858F, 0.233533F,
      0.246754497F, 0.259347F, 0.27645F, 0.286544F, 0.293097F, 0.305923F,
      0.309054F, 0.318265F, 0.324419F, 0.32810533F, 0.338815F, 0.33786F,
      -0.337255F, -0.335272F, -0.324041F, -0.332745F, -0.318884F, -0.312568F,
      -0.29845F, -0.285883F, -0.277492F, -0.254376F, -0.243853F, -0.234986991F,
      -0.232865F, -0.229323F, -0.217109501F, -0.211436F, -0.198558F, -0.1813F,
      -0.170074F, -0.144562F, -0.0661365F, 0.011026F, 0.0223611F, 0.0453067F,
      0.0659457F, 0.0808638F, 0.0778424F, 0.0953299F, 0.100434F, 0.115891F,
      0.112639F, 0.120215F, 0.144316F, 0.154838F, 0.172263F, 0.196485F, 0.20417F,
      0.199862F, 0.211186F, 0.22305F, 0.220402F, 0.25047F, 0.276863F, 0.277855F,
      0.315718F, 0.32225F, 0.331313F, 0.335138977F, 0.338815F, -0.3374F,
      -0.330707F, -0.332517F, -0.313689F, -0.305724F, -0.29845F, -0.288312F,
      -0.274469F, -0.260835F, -0.248084486F, -0.239326F, -0.238329F, -0.225486F,
      -0.209254F, -0.189695F, -0.143698F, -0.151968F, -0.131361F, -0.117143F,
      -0.0862218F, -0.0526957F, -0.0331392F, -0.0206276F, -0.00400323F,
      -0.000410467F, 0.0231611F, 0.0581829F, 0.0931463F, 0.164264F, 0.169869F,
      0.181337F, 0.211396F, 0.217192F, 0.240713507F, 0.254557F, 0.258147F,
      0.261474F, 0.276812F, 0.287506F, 0.31195F, 0.309801F, 0.328471F, 0.33459F,
      -0.337311506F, -0.337854F, -0.332817F, -0.326886F, -0.318154F, -0.311816F,
      -0.309955508F, -0.308334F, -0.298241F, -0.279692F, -0.25099F, -0.238844F,
      -0.215746F, -0.20643F, -0.193006F, -0.191537F, -0.166337F, -0.13900134F,
      -0.134602F, -0.127062F, -0.120805F, -0.0872352F, -0.0874839F, -0.0781141F,
      -0.080496F, 0.000579635F, 0.0237003F, 0.0336587F, 0.044406F, 0.0641371F,
      0.0704886F, 0.0806172043F, 0.0812546F, 0.0926615F, 0.0893089F,
      0.0984047055F, 0.111367501F, 0.123025F, 0.130985F, 0.144628F, 0.15758F,
      0.168668F, 0.196768F, 0.199564F, 0.208035F, 0.214975F, 0.251922488F,
      0.276804F, 0.2795F, 0.287451F, 0.29587F, 0.312830508F, 0.317135F,
      0.337961F, -0.338481337F, -0.335763F, -0.331247F, -0.330185503F,
      -0.322488F, -0.312147F, -0.30651F, -0.295224F, -0.291793F, -0.28244F,
      -0.275854F, -0.250667F, -0.236512F, -0.227843F, -0.210384F, -0.187223F,
      -0.133577496F, -0.130231F, -0.104937F, -0.0840372F, -0.0763781F,
      -0.054579F, -0.0237963F, -0.0299746506F, -0.00846964F, 0.0205856F,
      0.0457184F, 0.0477817506F, 0.110875F, 0.111916497F, 0.141677F, 0.149171F,
      0.148118F, 0.158919F, 0.181854F, 0.188584F, 0.204862F, 0.216946F,
      0.221879989F, 0.227245F, 0.244112F, 0.26469F, 0.286571F, 0.294438F,
      0.310592F, 0.309364021F, 0.321732F, 0.329013348F, 0.335297F, 0.335471F,
      -0.330902F, -0.325971484F, -0.318451F, -0.31782F, -0.309106F,
      -0.310060501F, -0.312738F, -0.29845F, -0.286906F, -0.256753F, -0.231051F,
      -0.216621011F, -0.137409F, -0.128961F, -0.12376F, -0.120129F, -0.0902349F,
      -0.0895400494F, -0.0527155F, -0.0347083F, 0.00283758505F, 0.0391963F,
      0.084657F, 0.135321498F, 0.143407494F, 0.175865F, 0.18784F, 0.204172254F,
      0.217018F, 0.219422F, 0.233197F, 0.242317F, 0.254804F, 0.247614F, 0.25977F,
      0.260553F, 0.279016F, 0.29353F, 0.291459024F, 0.306457F, 0.3109F,
      0.311099F, 0.323885173F, 0.317834F, 0.333774507F, -0.333073F, -0.33654F,
      -0.325602F, -0.318162501F, -0.308209F, -0.309144F, -0.312F, -0.29401F,
      -0.286899F, -0.279946F, -0.265742F, -0.257508F, -0.2445F, -0.238117F,
      -0.205442503F, -0.198892F, -0.177313F, -0.157404F, -0.134878F, -0.129501F,
      -0.112381F, -0.083198F, -0.0848432F, -0.0773431F, -0.0575634502F,
      -0.044844F, -0.0305902F, 0.0240215F, 0.0312676504F, 0.0467173F,
      0.040402852F, 0.0612181F, 0.080627F, 0.121849492F, 0.130978F, 0.156051F,
      0.160109F, 0.177711F, 0.206717F, 0.211513F, 0.219102F, 0.224733F,
      0.236802F, 0.273114F, 0.294858F, 0.315447F, 0.308323F, 0.307891F,
      0.322361F, 0.321082503F, 0.329788F, 0.331632018F, -0.336663663F,
      -0.338655F, -0.333656F, -0.330311656F, -0.317923F, -0.307812F, -0.282656F,
      -0.267744F, -0.257499F, -0.234338F, -0.235432327F, -0.222176F, -0.214317F,
      -0.20995F, -0.197782F, -0.183595F, -0.176701F, -0.154681F, -0.147097498F,
      -0.129038F, -0.107968F, -0.0874845F, -0.0447738F, -0.0280987F, -0.0144563F,
      0.00334765F, 0.0167959F, 0.0485515F, 0.0477535F, 0.0604213F, 0.0733564496F,
      0.0846447F, 0.0906484F, 0.106962F, 0.108305F, 0.107909F, 0.122794F,
      0.136019F, 0.142599F, 0.165435F, 0.188134F, 0.192121F, 0.231446F, 0.24395F,
      0.243536F, 0.249538F, 0.25824F, 0.273184478F, 0.291358F, 0.301541507F,
      0.313688F, 0.317085F, 0.324735F, 0.335582F, 0.331794F, 0.338797F,
      -0.334122F, -0.328891516F, -0.329828F, -0.313853F, -0.306389F, -0.311595F,
      -0.287367F, -0.28598702F, -0.274799F, -0.2465F, -0.237221F, -0.22859F,
      -0.212449F, -0.186911F, -0.140752F, -0.130852F, -0.128131F, -0.105552F,
      -0.0912317F, -0.0793801F, -0.0672497F, -0.0427409F, -0.0181745F,
      -0.00772765512F, 0.0123315006F, 0.057258F, 0.0906234F, 0.105115F,
      0.139894F, 0.151994F, 0.174616F, 0.1792F, 0.192384F, 0.224426F, 0.239477F,
      0.251898497F, 0.289925F, 0.322724F, 0.330819488F, 0.336917F, 0.328426F,
      0.33795467F, -0.335468F, -0.325932F, -0.319385F, -0.31369251F, -0.306575F,
      -0.290725F, -0.280005F, -0.276675F, -0.266685F, -0.26994F, -0.21403F,
      -0.211697F, -0.18947351F, -0.171355F, -0.155805F, -0.152729F, -0.116226F,
      -0.0917546F, -0.0855377F, -0.070717F, -0.0633266F, -0.0548008F,
      -0.0443315F, -0.039708F, 0.0237887F, 0.0312358F, 0.0391441F, 0.0501783F,
      0.0631669F, 0.0587414F, 0.0692139F, 0.0850949F, 0.0937243F, 0.13229F,
      0.153009504F, 0.173994F, 0.179238F, 0.196933F, 0.214897F, 0.224515F,
      0.268887F, 0.286395F, 0.288498F, 0.29845F, 0.309055F, 0.31458F, 0.32509F,
      0.327991F, 0.333883F, -0.335149F, -0.324318F, -0.319424F, -0.309484661F,
      -0.309495F, -0.28408F, -0.263979F, -0.2398725F, -0.222567F, -0.204772F,
      -0.157653F, -0.154843F, -0.136239F, -0.111368F, -0.090416F, -0.0874892F,
      -0.0778157F, -0.0590971F, -0.0352648F, -0.0334297F, -0.0177718F,
      -0.0168443F, 0.00370595F, 0.00901806F, 0.0245094988F, 0.0341368467F,
      0.0662543F, 0.0808943957F, 0.120009F, 0.154461F, 0.187063F, 0.199451F,
      0.213117495F, 0.219838F, 0.241250008F, 0.275235F, 0.269858479F, 0.297117F,
      0.301083F, 0.315348F, 0.319434F, 0.33527F, -0.335715F, -0.335469F,
      -0.334446F, -0.32745F, -0.308334F, -0.309786022F, -0.270008F, -0.248683F,
      -0.241337F, -0.211113F, -0.212309F, -0.197814F, -0.176638499F, -0.15586F,
      -0.14788F, -0.135127F, -0.12618F, -0.119714F, -0.104999F, -0.0753462F,
      -0.0607897F, -0.0497655F, -0.0350339F, -0.0142613F, 0.00166136F,
      0.0166823F, 0.0694375F, 0.0784016F, 0.092087F, 0.122402F, 0.121507F,
      0.141991F, 0.150503F, 0.157604F, 0.211992F, 0.213186F, 0.221187F, 0.24363F,
      0.256903F, 0.257397F, 0.286684F, 0.284644F, 0.29192F, 0.290075F, 0.30991F,
      0.309884489F, 0.321583F, 0.322035F, 0.336257F, 0.329861F, -0.334616F,
      -0.332311F, -0.313578F, -0.309384F, -0.306553F, -0.291706F, -0.267352F,
      -0.267013F, -0.253698F, -0.223957F, -0.215377F, -0.176056504F, -0.176562F,
      -0.168031499F, -0.15458F, -0.146738F, -0.140308F, -0.124789F, -0.115794F,
      -0.109324F, -0.106463F, -0.0958683F, -0.0725319F, -0.071461F, -0.0424224F,
      -0.0243763F, -0.0249505F, 0.00300915982F, 0.000881882F, 0.0113073F,
      0.0298997F, 0.0405394323F, 0.0554075F, 0.0723421F, 0.0854633F,
      0.0918686539F, 0.0973574F, 0.117781F, 0.134129494F, 0.137472F, 0.162673F,
      0.173994F, 0.181962F, 0.190541F, 0.201349F, 0.243352F, 0.243412F,
      0.270557F, 0.296775F, 0.290211F, 0.302077F, 0.315833509F, 0.309787F,
      0.313272F, 0.321535F, 0.319299F, 0.3332735F, -0.334259F, -0.332283F,
      -0.315936506F, -0.308215976F, -0.309410334F, -0.29845F, -0.290558F,
      -0.233429F, -0.215069F, -0.174016F, -0.172564F, -0.169113F, -0.151915F,
      -0.137631506F, -0.128435F, -0.114711F, -0.0853352547F, -0.0786744F,
      -0.0634477F, -0.0545887F, -0.0361738F, -0.0232583F, 0.0093621F, 0.0302667F,
      0.0420130491F, 0.0589726F, 0.0715313F, 0.0810957F, 0.0885877F, 0.109093F,
      0.121131F, 0.134881F, 0.141818F, 0.149412F, 0.166556F, 0.169591F,
      0.200567F, 0.256883F, 0.263118F, 0.27414602F, 0.272598F, 0.306559F,
      0.302733481F, 0.312223017F, 0.333526F, 0.33782F, -0.335914016F, -0.329508F,
      -0.329824F, -0.321488F, -0.310382F, -0.286751F, -0.274837F, -0.263815F,
      -0.228117F, -0.223463F, -0.214361F, -0.207101F, -0.166339F, -0.151517F,
      -0.125032F, -0.122726F, -0.0955175F, -0.0748116F, -0.081041F, -0.0706585F,
      -0.0693388F, -0.0250575F, -0.0086886F, -0.00423932F, 0.0295558F,
      0.0438525F, 0.0755600333F, 0.0916271F, 0.112007F, 0.12037F, 0.131918F,
      0.14804F, 0.166352F, 0.172036F, 0.183414F, 0.191727F, 0.200857F, 0.212684F,
      0.207827F, 0.223268F, 0.261983514F, 0.29845F, 0.313007F, 0.307798F,
      0.323299F, 0.322761506F, 0.327585F, 0.33579F, 0.335247F, 0.338511F,
      -0.326599F, -0.31742F, -0.320626497F, -0.30924F, -0.29845F, -0.238537F,
      -0.229737F, -0.221333F, -0.211786F, -0.202F, -0.183652F, -0.169439F,
      -0.156498F, -0.146303F, -0.150756F, -0.135759F, -0.132675F, -0.113094F,
      -0.09531F, -0.0836527F, -0.0814985F, -0.0694071501F, -0.0595174022F,
      -0.0453763F, -0.0371806F, -0.0280424F, -0.00650537F, -0.00606242F,
      0.00855374F, 0.0413879F, 0.0667816F, 0.0624195486F, 0.0814708F, 0.1004695F,
      0.109091F, 0.1441F, 0.151025F, 0.175274F, 0.187091F, 0.186372F, 0.194318F,
      0.193039F, 0.204824F, 0.235642F, 0.246938F, 0.250395F, 0.252452F,
      0.274472F, 0.295669F, 0.306663F, 0.310943F, 0.312222F, 0.317918F,
      0.325839F, 0.332492F, 0.329863667F, -0.334572F, -0.316936493F, -0.317637F,
      -0.298004F, -0.29845F, -0.277357F, -0.272048F, -0.250955F, -0.238373503F,
      -0.240345F, -0.223243F, -0.223893F, -0.218399F, -0.199552F, -0.197555F,
      -0.190376F, -0.139131F, -0.114123F, -0.094644F, -0.0847743F,
      -0.0788912326F, -0.0723734F, -0.0459671F, -0.00167125498F, 0.0135289F,
      0.0334001F, 0.0306301F, 0.0409021F, 0.0421795473F, 0.102507852F, 0.116614F,
      0.128041F, 0.143689F, 0.146005F, 0.147932F, 0.161275F, 0.176396F, 0.18155F,
      0.181303F, 0.193837F, 0.207118F, 0.209861F, 0.228412F, 0.243616F,
      0.243309F, 0.273137F, 0.272114F, 0.284418F, 0.310869F, 0.313674F,
      0.310441494F, 0.321593F, 0.332045019F, 0.338521F, -0.338387F, -0.330117F,
      -0.314781487F, -0.318336F, -0.291561F, -0.278495F, -0.260294F, -0.20601F,
      -0.197060496F, -0.191211671F, -0.178947F, -0.182434F, -0.16471F,
      -0.153768F, -0.135204F, -0.140483F, -0.108826F, -0.106496F, -0.0951386F,
      -0.0894745365F, -0.0568992F, -0.0378791F, -0.0267708F, -0.0262723491F,
      -0.0203611F, -0.0117318F, 0.0147263F, 0.00875872F, 0.0224811509F,
      0.0281059F, 0.0423437F, 0.054114F, 0.057555F, 0.0741237F, 0.117697F,
      0.138997F, 0.144307F, 0.151750505F, 0.175959F, 0.170115501F, 0.178875F,
      0.193991676F, 0.223594F, 0.240129F, 0.252946F, 0.263945F, 0.272862494F,
      0.270816982F, 0.305703F, 0.31104F, 0.319533497F, 0.334917F, -0.337539F,
      -0.336873502F, -0.314458F, -0.318397F, -0.306546F, -0.308954F, -0.28487F,
      -0.270959F, -0.256808F, -0.254554F, -0.240151495F, -0.226998F, -0.200869F,
      -0.18327F, -0.16592F, -0.165213F, -0.149092495F, -0.108864F, -0.0928845F,
      -0.0871179F, -0.0645979F, -0.0593916F, -0.0526373F, -0.0096837F, 0.022615F,
      0.0193645F, 0.0441559F, 0.0698265F, 0.0719106F, 0.104481F, 0.116318F,
      0.122835F, 0.128659F, 0.149297F, 0.15828F, 0.167898F, 0.183263F, 0.199871F,
      0.225623F, 0.232416F, 0.244333F, 0.253823F, 0.2548F, 0.272442F, 0.295799F,
      0.309949F, 0.322857F, 0.33701F, 0.338351F, -0.327519F, -0.319534F,
      -0.307551F, -0.298338F, -0.29087F, -0.283143F, -0.274175F, -0.260216F,
      -0.244438F, -0.249755F, -0.225526F, -0.175064F, -0.155535F, -0.156908989F,
      -0.14474F, -0.125258F, -0.121613F, -0.11152F, -0.0958852F, -0.086706F,
      -0.0660164F, -0.0594267F, -0.049785F, -0.0296277F, -0.0165088F,
      -0.00697163027F, 0.00089365046F, 0.0100881F, 0.0247971F, 0.0297824F,
      0.0381476F, 0.0395709F, 0.0528784022F, 0.0636546F, 0.0823555F, 0.0888442F,
      0.109942F, 0.127579F, 0.183565F, 0.196011F, 0.201842F, 0.221127F,
      0.240479F, 0.263285F, 0.27616F, 0.285165F, 0.295288F, 0.305706F, 0.313986F,
      0.313075F, 0.309946507F, 0.311142F, 0.334187F, 0.336074F, -0.330841F,
      -0.314197F, -0.312537F, -0.309117F, -0.29845F, -0.269903F, -0.269006F,
      -0.243536F, -0.242425F, -0.23842F, -0.220599F, -0.206832F, -0.206331F,
      -0.1993545F, -0.191744F, -0.1543F, -0.145686F, -0.143897F, -0.136477F,
      -0.129413F, -0.128319502F, -0.116977F, -0.0953207F, -0.09067F,
      -0.0819063038F, -0.0763148F, -0.0454444F, -0.0260273F, 0.0331132F,
      0.0515370518F, 0.0594167F, 0.0671329F, 0.0931271F, 0.0971175F, 0.103323F,
      0.113294F, 0.117232F, 0.142227F, 0.176909F, 0.171358496F, 0.189379F,
      0.213635F, 0.23374F, 0.247269F, 0.262648523F, 0.257616F, 0.273795485F,
      0.267149F, 0.293971F, 0.314097F, 0.333738F, 0.331529F, 0.338056684F,
      -0.314931F, -0.311802F, -0.308267F, -0.308642F, -0.269746F, -0.234128505F,
      -0.15878F, -0.114114F, -0.119471F, -0.106125668F, -0.0894794F, -0.0750546F,
      -0.0691318F, -0.0557351F, -0.0428993F, -0.0281441F, -0.0290039F,
      -0.0175389014F, -0.0150015F, -0.00182605F, 0.027921F, 0.0670264F,
      0.0787415F, 0.0839249F, 0.093734F, 0.0983917F, 0.102526F, 0.111582F,
      0.125755F, 0.132321F, 0.146868F, 0.160762F, 0.179095F, 0.2054F, 0.218001F,
      0.22519F, 0.235956F, 0.2513F, 0.26356566F, 0.282849F, 0.292767F, 0.296841F,
      0.306226F, 0.316809F, 0.318095326F, 0.331162F, 0.332798511F, -0.336987674F,
      -0.317274F, -0.315317F, -0.297312F, -0.294924F, -0.288503F, -0.265993F,
      -0.260594F, -0.207835F, -0.195746F, -0.136696F, -0.0961387F, -0.0675953F,
      -0.0604518F, -0.041862F, -0.0417332F, -0.0315356F, -0.0167242F,
      -0.0114272498F, 0.0241561F, 0.0412169F, 0.0589225F, 0.079F, 0.105739F,
      0.104032F, 0.121348F, 0.126127F, 0.129434F, 0.153615F, 0.155523F,
      0.171885F, 0.200668499F, 0.248828F, 0.263082F, 0.260746479F, 0.276102F,
      0.281706F, 0.313544F, 0.318387F, 0.331705F, -0.337952F, -0.338795F,
      -0.328355342F, -0.315711021F, -0.314113F, -0.309196F, -0.29845F,
      -0.276014F, -0.27429F, -0.270625F, -0.270051F, -0.234424F, -0.213133F,
      -0.217394501F, -0.205384F, -0.153381F, -0.14143F, -0.132499F, -0.128787F,
      -0.11903F, -0.0969264F, -0.0868683F, -0.0897534F, -0.0670332F, -0.059999F,
      -0.0471637F, -0.0157388F, -0.00536796F, 0.00831985F, 0.0281531F,
      0.0503782F, 0.093076F, 0.0964688F, 0.104463F, 0.107503F, 0.127088F,
      0.151939F, 0.1888365F, 0.206042F, 0.225959F, 0.246711F, 0.264444F,
      0.294067F, 0.291215479F, 0.29845F, 0.311745524F, 0.321982F, 0.331339F,
      0.332456F, -0.338821F, -0.334038F, -0.334479F, -0.326572F, -0.306255519F,
      -0.305736F, -0.29637F, -0.29845F, -0.282709F, -0.276011527F, -0.266208F,
      -0.260102F, -0.248494F, -0.247407F, -0.230284F, -0.232299F, -0.213026F,
      -0.205967F, -0.204870492F, -0.195569F, -0.187424F, -0.174355F, -0.17335F,
      -0.161685F, -0.151098505F, -0.136946F, -0.116287F, -0.0873121F,
      -0.0733738F, -0.0679998F, -0.0407728478F, -0.00545678F, -0.00813962519F,
      0.00412491F, -0.0018864F, 0.0134452F, 0.01006F, 0.0312359F, 0.0312043F,
      0.0384986F, 0.0483619F, 0.0589932F, 0.0941631F, 0.1022975F, 0.141465F,
      0.175938F, 0.174921F, 0.183069333F, 0.19344F, 0.201104F, 0.216785F,
      0.225743F, 0.22007066F, 0.23762F, 0.24216F, 0.262720525F, 0.271708F,
      0.296059489F, 0.298414F, 0.29845F, 0.310322F, 0.321959347F, 0.33514F,
      0.333227F, 0.327829F, -0.336351514F, -0.336021483F, -0.332962F, -0.3327F,
      -0.330543F, -0.315019F, -0.309672F, -0.28666F, -0.254979F, -0.211796F,
      -0.181657F, -0.17136F, -0.153884F, -0.146861F, -0.0932746F, -0.0648522F,
      -0.033244F, -0.0275831F, -0.00830904F, -0.0127108F, 0.00382861F,
      -0.000555277F, 0.0132087506F, 0.0631583F, 0.10877F, 0.113324F, 0.149205F,
      0.161968F, 0.178536F, 0.189001F, 0.205609F, 0.221452F, 0.238847F,
      0.245292F, 0.250382F, 0.263775F, 0.270292F, 0.269843F, 0.28616F,
      0.312116981F, 0.309437F, 0.314753F, 0.333561F, 0.327917F, 0.338085F,
      -0.326565F, -0.331766F, -0.319371F, -0.305746F, -0.298191F, -0.280227F,
      -0.206814F, -0.185163F, -0.17172F, -0.156013F, -0.154263F, -0.151408F,
      -0.146366F, -0.131444F, -0.0759119F, -0.0751476F, -0.0647949F, -0.0699674F,
      -0.0531758F, -0.045817F, -0.0252009F, -0.0308795F, -0.0189372F, 0.0171982F,
      0.0363498F, 0.0609727502F, 0.0737757F, 0.112058F, 0.126744F, 0.128927F,
      0.137049F, 0.161329F, 0.17548F, 0.185827F, 0.195257F, 0.212684989F,
      0.215642989F, 0.221335F, 0.246728F, 0.258482F, 0.272525F, 0.305706F,
      0.314798F, 0.311877F, 0.310519F, 0.326435F, 0.335031F, -0.338405F,
      -0.333977F, -0.329753F, -0.332706F, -0.296041F, -0.28114F, -0.267644F,
      -0.254969F, -0.245719F, -0.237442F, -0.201892F, -0.183973F, -0.188251F,
      -0.176798F, -0.165895F, -0.160875F, -0.148331493F, -0.139951F,
      -0.129455507F, -0.123984F, -0.118607F, -0.118266F, -0.104122F, -0.0899023F,
      -0.0677978F, -0.0584795F, -0.047774F, -0.0352357F, -0.0357339F,
      -0.0186938F, -0.0138763F, 0.0155346F, 0.0107676F, 0.0184412F, 0.027661249F,
      0.0437741F, 0.0599947F, 0.106744F, 0.137234F, 0.146345F, 0.151251F,
      0.159453F, 0.166512F, 0.17044F, 0.179192F, 0.191252F, 0.200736F, 0.210706F,
      0.236046F, 0.29845F, 0.316065F, 0.307658F, 0.31508F, 0.322871F, 0.323587F,
      0.327393F, 0.336692F, -0.336427331F, -0.334577F, -0.334914F, -0.327313F,
      -0.309981525F, -0.312553F, -0.294807F, -0.29845F, -0.286858F,
      -0.276868492F, -0.263776F, -0.239305F, -0.231841F, -0.202039F, -0.191906F,
      -0.127998F, -0.11002F, -0.109695F, -0.0993917435F, -0.078224F, -0.0677243F,
      -0.0657693F, -0.0348464F, 0.0408702F, 0.0587313F, 0.0781386F, 0.104141F,
      0.119756505F, 0.120963F, 0.140866F, 0.150616F, 0.165048F, 0.169243F,
      0.1779F, 0.217787F, 0.239562F, 0.253462493F, 0.258234F, 0.281779F,
      0.286383F, 0.312714F, 0.308502018F, 0.321054F, 0.335404F, 0.330482F,
      -0.33622849F, -0.3264575F, -0.329649985F, -0.31407F, -0.311086F,
      -0.307419F, -0.308793F, -0.279139519F, -0.242271F, -0.225385F, -0.212379F,
      -0.19707F, -0.202367F, -0.178468496F, -0.159653F, -0.162836F, -0.135519F,
      -0.134584F, -0.123519F, -0.117006F, -0.0857134F, -0.056266F,
      -0.0491369516F, -0.0390304F, -0.0209309F, -0.00597756F, 0.011704F,
      0.0299545F, 0.0451441F, 0.0521582F, 0.0715561F, 0.08027675F, 0.0940123F,
      0.0908565F, 0.10664F, 0.132257F, 0.154633F, 0.163831F, 0.179315F, 0.18961F,
      0.19682F, 0.202314F, 0.212489F, 0.212116F, 0.221530497F, 0.22133249F,
      0.233932F, 0.250843F, 0.249848008F, 0.261405F, 0.271405F, 0.29292351F,
      0.311022F, 0.32378F, 0.333697F, 0.334278F, -0.334251F, -0.335632324F,
      -0.335054F, -0.335249F, -0.331249F, -0.32015F, -0.317902F, -0.306954F,
      -0.290863F, -0.274442F, -0.278002322F, -0.272381F, -0.217071F, -0.210432F,
      -0.205008F, -0.180483F, -0.167224F, -0.144848F, -0.130874F, -0.127387494F,
      -0.122549F, -0.110233F, -0.109952F, -0.0979503542F, -0.0830111F,
      -0.0809379F, -0.0814379F, -0.0378233F, -0.0419101F, -0.0315781F,
      -0.00742271496F, -0.011014F, 0.0168331F, 0.0400887F, 0.0706447512F,
      0.0806404F, 0.0928207487F, 0.104593F, 0.113069F, 0.144507498F, 0.179286F,
      0.181601F, 0.210349F, 0.224555F, 0.242834F, 0.260144502F, 0.29597F,
      0.305707F, 0.310073F, 0.310219F, 0.308776F, 0.32471F, 0.335038F, 0.330474F,
      0.335469F, 0.329972F, 0.338789F, -0.338822F, -0.326533258F, -0.31451F,
      -0.316676F, -0.310012F, -0.307531506F, -0.295780331F, -0.28643F,
      -0.284594F, -0.281921F, -0.268941F, -0.251124F, -0.226918F, -0.224513F,
      -0.222741F, -0.205273F, -0.201899F, -0.198433F, -0.186501F, -0.17927F,
      -0.15932F, -0.141216F, -0.106598F, -0.0938591F, -0.100834F, -0.0894211F,
      -0.0514331F, -0.0379247F, 0.00525536F, 0.0525218F, 0.0668659F, 0.0920179F,
      0.110114F, 0.132764F, 0.139354F, 0.150016F, 0.175975F, 0.183299F,
      0.188355F, 0.207238F, 0.234477F, 0.252355F, 0.264594018F, 0.27352F,
      0.288019F, 0.309053F, 0.313056F, 0.319609F, 0.33106F, 0.332005F, 0.338594F,
      0.338325F, -0.329521477F, -0.309423F, -0.309654683F, -0.2937F, -0.290109F,
      -0.290602F, -0.260593F, -0.233922F, -0.220642507F, -0.208073F, -0.195045F,
      -0.191276F, -0.164445F, -0.158363F, -0.15633601F, -0.146241F, -0.138898F,
      -0.125841498F, -0.119498F, -0.116685F, -0.0984625F, -0.0933932F,
      -0.0901033F, -0.0747368F, -0.0633865F, -0.0576185F, -0.0459841F,
      -0.0495262F, -0.0349335F, -0.0232702F, -0.0239102F, -0.021174F,
      0.000180662F, 0.0230243F, 0.037083F, 0.0517548F, 0.0535189F, 0.0928387F,
      0.102544F, 0.102827F, 0.109515503F, 0.122392F, 0.135465F, 0.165388F,
      0.173956F, 0.180245504F, 0.183868F, 0.195687F, 0.226936F, 0.228077F,
      0.247529F, 0.261721F, 0.261684F, 0.270888F, 0.292455F, 0.309887F,
      0.314921F, 0.332984656F, 0.337887F, -0.333741F, -0.334582F, -0.323192F,
      -0.320369512F, -0.313224F, -0.315728F, -0.305712F, -0.308310658F,
      -0.296119F, -0.296131492F, -0.286849F, -0.279784F, -0.275866F, -0.255591F,
      -0.232822F, -0.200145498F, -0.197478F, -0.146918F, -0.141193F, -0.113201F,
      -0.0863214F, -0.0608461F, -0.0298402F, -0.0205781F, 0.000877455F,
      0.103369F, 0.1105965F, 0.1089665F, 0.121608496F, 0.119647F, 0.154803F,
      0.148213F, 0.16986F, 0.176174F, 0.229554F, 0.238394F, 0.25362F, 0.261684F,
      0.273554F, 0.286655F, 0.306223F, 0.309613F, 0.309055F, 0.322620511F,
      0.32439F, 0.336824F, 0.332884F, 0.33797F, 0.337769F, -0.33346F, -0.338042F,
      -0.331824F, -0.311095F, -0.298084F, -0.29845F, -0.252884F, -0.257723F,
      -0.247046F, -0.1985275F, -0.19237F, -0.188840672F, -0.180208504F,
      -0.181217F, -0.166823506F, -0.142563F, -0.121213F, -0.099649F, -0.0369551F,
      -0.0163202F, -0.00804597F, 0.00390738F, -0.000755325F, 0.010548315F,
      0.020448599F, 0.0299045F, 0.0656845F, 0.0754573F, 0.0861734F, 0.0903755F,
      0.101842F, 0.108691F, 0.113454F, 0.131161988F, 0.151882336F, 0.154292F,
      0.176309F, 0.181512F, 0.204288F, 0.214605F, 0.245628F, 0.249516F,
      0.250174F, 0.265037F, 0.264084F, 0.286088F, 0.287692F, 0.29845F,
      0.316021979F, 0.324152648F, 0.32317251F, 0.333088517F, 0.334496F,
      0.337398F, -0.333674F, -0.323257F, -0.305844F, -0.307476F, -0.29845F,
      -0.297021F, -0.276247F, -0.251511F, -0.220474F, -0.210291F, -0.198306F,
      -0.187611F, -0.164585F, -0.152408F, -0.135366F, -0.131821F, -0.114103F,
      -0.115188F, -0.0763677F, -0.079813F, -0.0504694507F, -0.0309898F,
      -0.0200832F, 0.0326653F, 0.0345639512F, 0.0429786518F, 0.0517652F,
      0.070331F, 0.0692385F, 0.122783F, 0.130617F, 0.129874F, 0.14432F,
      0.173648F, 0.188293F, 0.227801F, 0.236122F, 0.253369F, 0.27644F, 0.270085F,
      0.295718F, 0.305899F, 0.308762F, 0.321080327F, 0.328494F, 0.338433F,
      -0.336031F, -0.317554F, -0.316338F, -0.311521F, -0.309018F, -0.293354F,
      -0.283206F, -0.262055F, -0.258921325F, -0.250798F, -0.242844F, -0.219179F,
      -0.20555F, -0.205024F, -0.18036F, -0.172117F, -0.170825F, -0.141059F,
      -0.12293F, -0.119166F, -0.108878F, -0.108365F, -0.09634725F, -0.0962783F,
      -0.0652663F, -0.0518321F, -0.0301514F, -0.0166525F, -0.00360596F,
      0.0185593F, 0.0439358503F, 0.0569589F, 0.0858849F, 0.114153F, 0.11885F,
      0.128568F, 0.149711F, 0.163227F, 0.172563F, 0.182742F, 0.191792488F,
      0.21868F, 0.226466F, 0.251347F, 0.288598F, 0.315328F, 0.314009F, 0.317406F,
      0.334704667F, 0.33581F, 0.338326514F, -0.333302F, -0.327973F, -0.330441F,
      -0.315478504F, -0.320339F, -0.307512F, -0.307160497F, -0.295102F,
      -0.27989F, -0.247972F, -0.247545987F, -0.193981F, -0.192864F, -0.18925F,
      -0.174213F, -0.161445F, -0.147103503F, -0.129244F, -0.120372F, -0.105454F,
      -0.111645F, -0.0758345F, -0.0724927F, -0.0487922F, -0.047833249F,
      -0.0424395F, -0.0245077F, -0.0159461F, -0.00795451F, 0.0128748F,
      0.0123463F, 0.0503064F, 0.0633844F, 0.0858358F, 0.099765F, 0.109099F,
      0.11489F, 0.117434F, 0.135763F, 0.142771F, 0.167289F, 0.1843F, 0.213043F,
      0.236964F, 0.237773F, 0.239124F, 0.281622F, 0.297908F, 0.312559F,
      0.307544F, 0.313334F, 0.32248F, 0.333918F, 0.33536F, -0.337715F,
      -0.325387F, -0.330812F, -0.319186509F, -0.315543F, -0.30976F, -0.29845F,
      -0.290892F, -0.261378F, -0.251928F, -0.22761F, -0.198439F, -0.187274F,
      -0.187696F, -0.1780615F, -0.164868504F, -0.158102F, -0.142923F,
      -0.139252499F, -0.130986F, -0.122661F, -0.122117F, -0.105292F, -0.0870719F,
      -0.0730655F, -0.059411F, -0.0146372F, -0.0109808F, -0.00110872F,
      0.0121343F, 0.0329149514F, 0.0421461F, 0.10604F, 0.122327F, 0.149605F,
      0.161995F, 0.185731F, 0.178772F, 0.215240508F, 0.225402F, 0.25452F,
      0.257356F, 0.271304F, 0.272519F, 0.284305F, 0.295176F, 0.29845F, 0.309528F,
      0.326347F, 0.33483F, 0.331231F, -0.337598F, -0.335469F, -0.326077491F,
      -0.315153F, -0.311494F, -0.306643F, -0.293389F, -0.288191F, -0.27013F,
      -0.260968F, -0.233618F, -0.222288F, -0.181604F, -0.165211F, -0.172748F,
      -0.139988F, -0.0945088F, -0.101567F, -0.0878205F, -0.0866928F, -0.0746053F,
      -0.0210105F, -0.00669831F, -0.00790334F, 0.00742741F, 0.0202869F,
      0.0345875F, 0.0272696F, 0.0416793F, 0.063971445F, 0.0585855F, 0.0678044F,
      0.080656F, 0.100233F, 0.119763F, 0.137061F, 0.142506F, 0.146183F,
      0.156639F, 0.171653509F, 0.181214F, 0.19043F, 0.203768F, 0.219038F,
      0.235132F, 0.254055F, 0.271918F, 0.306659F, 0.307072F, 0.316211F,
      0.312232F, 0.311545F, 0.319601F, 0.332551479F, -0.334277F, -0.326602F,
      -0.331762F, -0.318436F, -0.308491F, -0.288786F, -0.255143493F, -0.238312F,
      -0.216398507F, -0.206974F, -0.204232F, -0.198524F, -0.189369F, -0.185102F,
      -0.158574F, -0.135696F, -0.129048F, -0.0760422F, -0.0632188F, -0.0607011F,
      -0.0501688F, -0.0101160947F, 0.0268358F, 0.0294567F, 0.037864F, 0.0618684F,
      0.0636181F, 0.0690545F, 0.0955183F, 0.102853F, 0.110413F, 0.120043F,
      0.133800507F, 0.140176F, 0.153024F, 0.163762F, 0.169682F, 0.183705F,
      0.216728F, 0.226185501F, 0.222681F, 0.235247F, 0.249433F, 0.274014F,
      0.273394F, 0.289263F, 0.297441F, 0.316247314F, 0.311924517F, 0.311901F,
      0.316253F, 0.324562F, 0.32012552F, 0.335436F, 0.338799F, -0.338795F,
      -0.337101F, -0.325565F, -0.31361F, -0.309396F, -0.29845F, -0.289934F,
      -0.267900527F, -0.255011499F, -0.225326F, -0.220864F, -0.216618F,
      -0.204794F, -0.19235F, -0.174769F, -0.172664F, -0.151902F, -0.140247F,
      -0.126865F, -0.128759F, -0.11697F, -0.117139F, -0.111169F, -0.0984283F,
      -0.0977699F, -0.0747167F, -0.057408F, -0.0434232F, -0.0395074F, -0.015061F,
      -0.00223073F, 0.012784F, 0.0423078351F, 0.0616366F, 0.0862272F, 0.0801469F,
      0.0951681F, 0.10499F, 0.108276F, 0.112944499F, 0.118123F, 0.122343F,
      0.137058F, 0.138302F, 0.16234F, 0.173833F, 0.177192F, 0.196534F, 0.229777F,
      0.243108988F, 0.254238F, 0.263748F, 0.274207F, 0.282197F, 0.29845F,
      0.306402F, 0.321999F, 0.332752F, 0.338201F, -0.331816F, -0.312152F,
      -0.289476F, -0.286844671F, -0.272716F, -0.234397F, -0.22528F, -0.199502F,
      -0.174736F, -0.173725F, -0.172484F, -0.153101F, -0.146063F, -0.148288F,
      -0.137659F, -0.122896F, -0.0870843F, -0.0913551F, -0.0455954F, -0.0353058F,
      -0.0243607F, 0.00782622F, 0.0209073F, 0.0381424F, 0.0554142F, 0.0521967F,
      0.0737572F, 0.0694175F, 0.081545F, 0.104779F, 0.109619F, 0.109139F,
      0.120614F, 0.132475F, 0.146968F, 0.166122F, 0.167307F, 0.17892F, 0.192906F,
      0.206547F, 0.202401F, 0.208895F, 0.212809324F, 0.225258F, 0.292504F,
      0.305823F, 0.310111F, 0.309052F, 0.325684F, 0.329024F, 0.334523F,
      0.331929F, -0.338256F, -0.335875511F, -0.335804F, -0.332258F, -0.32282F,
      -0.30879432F, -0.29845F, -0.29845F, -0.275086F, -0.267627507F, -0.271865F,
      -0.252542F, -0.223398F, -0.222466F, -0.215714991F, -0.19491F, -0.166147F,
      -0.164403F, -0.16069F, -0.159489334F, -0.149641F, -0.145095F, -0.133689F,
      -0.0910913F, -0.0729275F, -0.0602776F, -0.0496604F, -0.0301975F,
      -0.00707514677F, -0.00854596496F, -0.00137442F, -0.00155669F, 0.0254291F,
      0.0896796F, 0.109719F, 0.115958F, 0.120809F, 0.133691F, 0.132495F,
      0.138145F, 0.165848F, 0.231593F, 0.232124329F, 0.241963F, 0.252328F,
      0.248031F, 0.261654F, 0.275147F, 0.286827F, 0.29845F, 0.30658F, 0.31524F,
      0.312813F, 0.329167F, 0.337599F, -0.332365513F, -0.321509F, -0.314676F,
      -0.294987F, -0.280271F, -0.28256F, -0.271321F, -0.250013F, -0.214357F,
      -0.209542F, -0.16874F, -0.156265F, -0.153807F, -0.150487497F,
      -0.135427505F, -0.132256F, -0.0758447945F, -0.0679023F, -0.0618601F,
      -0.0624465F, -0.0456532F, -0.0118687F, -0.000191063F, 0.0183825493F,
      0.0296355F, 0.0566681F, 0.0608684F, 0.0770495F, 0.0715684518F, 0.0900836F,
      0.100897F, 0.127939F, 0.139891505F, 0.154877F, 0.164018F, 0.168701F,
      0.184055F, 0.187177F, 0.208631F, 0.214107F, 0.232474F, 0.26473F, 0.306372F,
      0.312614F, 0.311276495F, 0.318177F, 0.32468F, 0.330452F, 0.336091F,
      0.338495F, -0.333337F, -0.335473F, -0.336651027F, -0.326875F, -0.328529F,
      -0.314212F, -0.318014503F, -0.308373511F, -0.307537496F, -0.297708F,
      -0.277241F, -0.265984F, -0.220256F, -0.205054F, -0.193931F, -0.112144F,
      -0.0999033F, -0.0938292F, -0.0577210486F, -0.00866683F, 0.0323687345F,
      0.029291451F, 0.0633172F, 0.0680345F, 0.0868513F, 0.0913003F, 0.103444F,
      0.112716F, 0.122534F, 0.14624F, 0.145014F, 0.154155F, 0.158034F, 0.17412F,
      0.195915F, 0.203929F, 0.222718507F, 0.233268F, 0.245184F, 0.254733F,
      0.267513F, 0.297714F, 0.29845F, 0.309561F, 0.319596499F, 0.333655F,
      0.328818F, 0.338304F, -0.335032F, -0.324283481F, -0.317245662F, -0.317979F,
      -0.267657F, -0.263975F, -0.259862F, -0.235155F, -0.21746999F, -0.168907F,
      -0.164218992F, -0.157378F, -0.149748F, -0.14935F, -0.103408F, -0.0798538F,
      -0.081817F, -0.0699874F, -0.0621611F, -0.0577613F, -0.0331561F,
      -0.0313199F, -0.00425512F, 0.0174718F, 0.039878F, 0.0913976F, 0.0966808F,
      0.0989353F, 0.107417F, 0.121171F, 0.119456F, 0.128961F, 0.143088F,
      0.162799F, 0.172758F, 0.178828F, 0.1911F, 0.205947F, 0.22249F, 0.218272F,
      0.269401F, 0.275044F, 0.305703F, 0.309426F, 0.31573F, 0.335262F, 0.338001F,
      -0.33739F, -0.331625F, -0.321807981F, -0.309732348F, -0.297948F,
      -0.2640315F, -0.264214F, -0.235276F, -0.223394F, -0.204842F, -0.195080012F,
      -0.190713F, -0.182109F, -0.178998F, -0.166582F, -0.170399F, -0.14038F,
      -0.116675F, -0.105681F, -0.0937079F, -0.0984187F, -0.0782004F, -0.071787F,
      -0.0581518F, -0.0531724F, -0.0370593F, -0.0129836F, 0.00709495F,
      0.00296196F, 0.0326671F, 0.0455775484F, 0.0488583F, 0.0705931485F,
      0.0902387F, 0.1047635F, 0.102034F, 0.141257F, 0.17463851F, 0.187495F,
      0.216619F, 0.207923F, 0.222553F, 0.221257F, 0.234393F, 0.254251F,
      0.283516496F, 0.287765F, 0.310758501F, 0.310749F, 0.320653021F, 0.328067F,
      0.335469F, 0.332089F, -0.335333496F, -0.323292F, -0.326647F, -0.321708F,
      -0.307905674F, -0.29845F, -0.275779486F, -0.275607497F, -0.272349F,
      -0.258812487F, -0.250114F, -0.239279F, -0.203534F, -0.193522F, -0.18941F,
      -0.15525F, -0.138199F, -0.124004F, -0.128616F, -0.112908F, -0.100632F,
      -0.0904198F, -0.0767662F, -0.0538746F, -0.0295829F, -0.00320703F,
      -0.00411576F, 0.00561465F, 0.0126063F, 0.0266966F, 0.0468328F, 0.0416902F,
      0.0577403F, 0.0893995F, 0.10495F, 0.100725152F, 0.124233F, 0.132792F,
      0.139629F, 0.177861F, 0.196304F, 0.240888F, 0.272051513F, 0.283757508F,
      0.309827F, 0.31633F, 0.322087F, 0.320883662F, 0.334565F, 0.327389F,
      0.338813F, -0.33537F, -0.330644F, -0.320368F, -0.306129F, -0.310946F,
      -0.298122F, -0.286443F, -0.274444F, -0.26385F, -0.268121F, -0.250102F,
      -0.246730506F, -0.234307F, -0.224316F, -0.222557F, -0.205008F, -0.188833F,
      -0.161958F, -0.147022009F, -0.138105F, -0.140502F, -0.110915005F,
      -0.0946947F, -0.0755779F, -0.0455516F, -0.0358732F, -0.0237212F,
      -0.022911F, -0.00560239F, 0.0168948F, 0.0538236F, 0.0615352F, 0.0595772F,
      0.076802F, 0.0815563F, 0.110351503F, 0.11976F, 0.150241F, 0.149487F,
      0.162941009F, 0.167857F, 0.185745F, 0.196456F, 0.198855F, 0.218486F,
      0.243102F, 0.262410522F, 0.273013F, 0.284153F, 0.306216F, 0.317075F,
      0.309171F, 0.327759F, 0.33547F, 0.327489F, -0.325579F, -0.318499506F,
      -0.310791492F, -0.29845F, -0.282312F, -0.270941F, -0.25742F, -0.196615F,
      -0.184498F, -0.178953F, -0.167524F, -0.163327F, -0.161906F, -0.145627335F,
      -0.152096F, -0.12831F, -0.117043F, -0.118408F, -0.108261F, -0.0974907F,
      -0.0872974F, -0.0800018F, -0.0706629455F, -0.0598582F, -0.0503958F,
      -0.0378315F, 0.00390232F, 0.00569385F, 0.0440024F, 0.0527153499F,
      0.0641801F, 0.0676231F, 0.0980594F, 0.132965F, 0.155243F, 0.155183F,
      0.181814F, 0.198427F, 0.211237F, 0.223349988F, 0.229821F, 0.28873F,
      0.306407F, 0.306231F, 0.31003F, 0.30786F, 0.327534F, -0.333108F,
      -0.338725F, -0.333669F, -0.326578F, -0.320418656F, -0.309005678F,
      -0.290271F, -0.279163F, -0.278563F, -0.266914487F, -0.255588F, -0.229632F,
      -0.228558496F, -0.211421F, -0.19115001F, -0.184252F, -0.168307F,
      -0.171322F, -0.14042F, -0.134835F, -0.108149F, -0.0985333F, -0.086308F,
      -0.0849923F, -0.0559714F, -0.0472076F, -0.032465F, -0.0142753F,
      0.00679559F, 0.0144181F, 0.0351478F, 0.0672724F, 0.101931F, 0.124383F,
      0.127219F, 0.142001F, 0.155542F, 0.158228F, 0.187821F, 0.202626F,
      0.236811F, 0.249539F, 0.27348F, 0.285049F, 0.292408F, 0.309591681F,
      0.32446F, 0.331845F, 0.335259F, 0.328538F, -0.333948F, -0.325858F,
      -0.316381F, -0.32031F, -0.310239F, -0.312434F, -0.274222F, -0.258676F,
      -0.248207F, -0.222839F, -0.22163F, -0.208458F, -0.199431F, -0.135262F,
      -0.134267F, -0.100796F, -0.0966903F, -0.090243F, -0.0702937F, -0.0613774F,
      -0.0452139F, -0.0409597F, -0.00429532F, 0.0118142702F, 0.00962152F,
      0.0179513F, 0.0341793F, 0.0488049F, 0.0597236F, 0.0572196F, 0.0671974F,
      0.0778544F, 0.145616F, 0.153167501F, 0.199353F, 0.214042F, 0.254306F,
      0.296957F, 0.295642F, 0.302240491F, 0.323543F, 0.323721F, 0.332024F,
      0.333566F, 0.338799F, -0.322872F, -0.322836F, -0.306741F, -0.295046F,
      -0.258118F, -0.225207F, -0.216995507F, -0.216234F, -0.207078F, -0.185028F,
      -0.155453F, -0.136169F, -0.12412F, -0.0959461F, -0.0945281F, -0.0670323F,
      -0.0542484F, -0.0551784486F, -0.00668038474F, -0.0103189F, 0.017025F,
      0.0226451F, 0.0409191F, 0.0757900476F, 0.0794659F, 0.0810057F,
      0.121579498F, 0.153794F, 0.150713F, 0.160041F, 0.170151F, 0.199743F,
      0.216259F, 0.222081F, 0.227428F, 0.242793F, 0.276767015F, 0.331835F,
      0.333172F, 0.335385F, 0.332537651F, 0.337923F, -0.33576268F, -0.328688502F,
      -0.319676F, -0.296342F, -0.286261F, -0.26952F, -0.262811F, -0.243729F,
      -0.222943F, -0.218588F, -0.204867F, -0.196840495F, -0.191361F, -0.175517F,
      -0.159893F, -0.143285F, -0.137514F, -0.128677F, -0.105002F, -0.0930808F,
      -0.0545957F, -0.048255302F, -0.0522188F, -0.0331822F, 0.0128916F,
      0.0315407F, 0.0519902F, 0.0628872F, 0.0901344F, 0.0990247F, 0.108543F,
      0.118855F, 0.14931F, 0.17482F, 0.203155F, 0.202375F, 0.212256F, 0.224533F,
      0.232774F, 0.240056F, 0.285541F, 0.27869F, 0.292211F, 0.297984F, 0.305702F,
      0.313741F, 0.307811F, 0.324973F, 0.32117F, 0.334901F, 0.337715507F,
      -0.337153018F, -0.33547F, -0.334314F, -0.31964F, -0.316439748F, -0.311576F,
      -0.308722F, -0.310958982F, -0.296134F, -0.27956F, -0.2491965F, -0.233031F,
      -0.219446F, -0.219776F, -0.207422F, -0.176097F, -0.152602F, -0.142127F,
      -0.118269503F, -0.110147499F, -0.105381F, -0.0947309F, -0.0858486F,
      -0.0804612F, -0.0522463F, -0.0362171F, -0.0256265F, 0.0350531F, 0.0373864F,
      0.0512361F, 0.0640036F, 0.076201F, 0.0903244F, 0.105326F, 0.126895F,
      0.129177F, 0.146539F, 0.152152F, 0.171741F, 0.171283F, 0.201396F,
      0.235983F, 0.229315F, 0.246841F, 0.256081F, 0.274337F, 0.290051F,
      0.306299F, 0.313879F, 0.310298F, 0.334668517F, 0.334028F, 0.332338512F,
      0.338278F, -0.338752F, -0.33547F, -0.327405F, -0.330598F, -0.327603F,
      -0.316573F, -0.316428F, -0.305717F, -0.305747F, -0.295638025F, -0.267636F,
      -0.253923F, -0.246012F, -0.234447F, -0.227171496F, -0.206898F, -0.208415F,
      -0.169690013F, -0.159193F, -0.144498F, -0.14147F, -0.124724F, -0.108287F,
      -0.0833409F, -0.0781041F, -0.0485305488F, -0.0330119F, 0.0122648F,
      0.0250444F, 0.0184707F, 0.0296632F, 0.0280496F, 0.0404288024F, 0.0545529F,
      0.0708037317F, 0.0772189F, 0.0982331F, 0.107532F, 0.112071F, 0.166392F,
      0.173814F, 0.183983F, 0.194073334F, 0.233788F, 0.238837F, 0.274191F,
      0.28237F, 0.28141F, 0.28991F, 0.291477F, 0.297898F, 0.313498F, 0.310071F,
      0.313887F, 0.331619F, 0.331941F, 0.333195F, -0.33437F, -0.324011F,
      -0.327884F, -0.317098F, -0.305784F, -0.306116521F, -0.308729F,
      -0.276437521F, -0.266164333F, -0.268982321F, -0.260077F, -0.240932F,
      -0.241079F, -0.225939F, -0.22138F, -0.193413F, -0.197307F, -0.176126F,
      -0.16651F, -0.150524F, -0.140066F, -0.139462F, -0.12473F, -0.126937F,
      -0.11367F, -0.0634829F, -0.0712905F, -0.0365806F, -0.0245679F, -0.0257137F,
      -0.017999F, -0.00968358666F, 0.000548452F, 0.0326103F, 0.0426634F,
      0.0512673F, 0.0718137F, 0.0850057453F, 0.112215F, 0.146037F, 0.164166F,
      0.168219F, 0.21343F, 0.222479F, 0.231125F, 0.240408F, 0.271244F, 0.272757F,
      0.279446483F, 0.293806F, 0.305964F, 0.311545F, 0.320429504F, 0.324791F,
      0.334829F, 0.333293F, 0.335771F, 0.337372F, -0.332975F, -0.32877925F,
      -0.327983F, -0.320065F, -0.320381492F, -0.311329F, -0.311906F, -0.295167F,
      -0.285167F, -0.255476F, -0.212772F, -0.199886F, -0.152621F, -0.145305F,
      -0.135402F, -0.135966F, -0.120935F, -0.120876F, -0.11235F, -0.107916F,
      -0.0873665F, -0.0794675F, -0.0715952F, -0.0643917F, -0.0619545F,
      -0.0354283F, -0.0174208507F, -0.0108342F, 0.00527921F, 0.0103438F,
      0.0201722F, 0.0540193F, 0.0616661F, 0.0684181F, 0.0881846F, 0.106562F,
      0.102383F, 0.124484F, 0.119233F, 0.162857503F, 0.158409F, 0.19937F,
      0.215276F, 0.225772F, 0.24271F, 0.251817F, 0.266724F, 0.278909F,
      0.291405022F, 0.31959F, 0.33058548F, 0.33328F, 0.336564F, 0.329328507F,
      -0.338788F, -0.329328328F, -0.326467F, -0.318151F, -0.309284F, -0.290524F,
      -0.267909F, -0.243372F, -0.224375F, -0.196222F, -0.174812F, -0.170611F,
      -0.159599F, -0.13263F, -0.121759F, -0.108035F, -0.0956559F, -0.0857045F,
      -0.0820269F, -0.0601349F, -0.0126446F, -0.00958019F, 0.00886029F,
      0.0114231F, 0.0471787F, 0.048229F, 0.104641F, 0.112278F, 0.119315F,
      0.124697F, 0.146571F, 0.197942F, 0.2036F, 0.226821F, 0.229556F, 0.235379F,
      0.241258F, 0.245901F, 0.252227F, 0.264991F, 0.268421F, 0.271029F,
      0.309863F, 0.307671F, 0.314699501F, 0.322973F, 0.323464F, 0.329185F,
      0.334323F, 0.328276F, 0.338818F, -0.327466F, -0.326792F, -0.319113F,
      -0.314121F, -0.317707F, -0.306979F, -0.308673024F, -0.29845F, -0.291592F,
      -0.257473F, -0.243199F, -0.242248F, -0.228026F, -0.196859F, -0.188147F,
      -0.186017F, -0.148699F, -0.130614504F, -0.096115F, -0.089902F,
      -0.0864426643F, -0.0768757F, -0.0640847F, -0.0638708F, -0.0482821F,
      -0.0284038F, -0.00674541341F, 0.026022F, 0.0350316F, 0.0592041F,
      0.0725013F, 0.0946485F, 0.122571498F, 0.164214F, 0.171961F, 0.185854F,
      0.206646F, 0.216866F, 0.223376F, 0.272081017F, 0.277444F, 0.294242F,
      0.305705F, 0.312924F, 0.313618F, 0.318512499F, 0.334893F, -0.33456F,
      -0.337598503F, -0.324547F, -0.321128F, -0.312744F, -0.290822983F,
      -0.276204497F, -0.271206F, -0.246574F, -0.214335F, -0.194113F, -0.1859245F,
      -0.176293F, -0.13845F, -0.142299F, -0.0963188F, -0.0959276F, -0.0760089F,
      -0.0481483489F, -0.0154799502F, 0.0154942F, 0.0627421513F, 0.0608673505F,
      0.0744098F, 0.0774132F, 0.0775004F, 0.097881F, 0.109062F, 0.125481F,
      0.125062F, 0.135718F, 0.179689F, 0.211309F, 0.215621F, 0.220273F,
      0.240527F, 0.252302F, 0.258014F, 0.270606667F, 0.318881F, 0.337332F,
      -0.329083F, -0.309729F, -0.307180494F, -0.279335499F, -0.260673344F,
      -0.227824509F, -0.214929F, -0.19868F, -0.174325F, -0.157011F, -0.133781F,
      -0.122704F, -0.0971601F, -0.0745876F, -0.0503395F, -0.0233604F,
      -0.0226805F, 0.00322109F, -0.000385068F, 0.0234061F, 0.0518052F, 0.106881F,
      0.120575F, 0.143987F, 0.149826497F, 0.159269F, 0.175027F, 0.184952F,
      0.192312F, 0.203196F, 0.216847F, 0.222120494F, 0.246343F, 0.251803F,
      0.251792F, 0.262913F, 0.291785F, 0.330552489F, 0.336026F, -0.337664664F,
      -0.338672F, -0.325437F, -0.326024F, -0.318927F, -0.313413F, -0.289223492F,
      -0.282132F, -0.268082F, -0.260694F, -0.245727F, -0.219011F, -0.207515F,
      -0.179378F, -0.168888F, -0.150913F, -0.065525949F, -0.0527001F,
      -0.0348231F, -0.0208236F, -0.0164327F, 0.0275206F, 0.0413122F,
      0.0532203503F, 0.08475F, 0.0844552F, 0.10931F, 0.110241503F, 0.118071F,
      0.136222F, 0.130455F, 0.138408F, 0.147818F, 0.172129F, 0.178373F,
      0.196854F, 0.219171494F, 0.230773F, 0.236273F, 0.239476F, 0.283789F,
      0.31097F, 0.309058F, 0.310837F, 0.32622F, 0.318358F, 0.327698F, 0.337928F,
      -0.334323F, -0.329924F, -0.31575F, -0.310425F, -0.31082952F, -0.307705F,
      -0.30919F, -0.260976F, -0.25091F, -0.241029F, -0.221547F, -0.203126F,
      -0.199063F, -0.17946F, -0.16815F, -0.146587F, -0.138786F, -0.123026F,
      -0.129403502F, -0.122445F, -0.0933363F, -0.0782773F, -0.0639388F,
      -0.0551663F, -0.0502668F, -0.0179102F, -0.00953377F, 0.030688433F,
      0.0374912F, 0.0558516495F, 0.0579279F, 0.0676168F, 0.098848F, 0.105332F,
      0.11665F, 0.1214F, 0.125404F, 0.130434F, 0.161938F, 0.174511F, 0.191868F,
      0.201413512F, 0.198191F, 0.210493F, 0.223519F, 0.232565F, 0.246588F,
      0.260505F, 0.25746F, 0.302418F, 0.305704F, 0.307991505F, 0.312859505F,
      0.33201F, 0.337757F, -0.337522F, -0.332927F, -0.331638F, -0.324980497F,
      -0.322804F, -0.320669F, -0.310826F, -0.277779F, -0.261036F, -0.2451F,
      -0.2441F, -0.236897F, -0.236611F, -0.215793F, -0.216721F, -0.199347F,
      -0.187869489F, -0.17491F, -0.173814F, -0.133487493F, -0.125106F,
      -0.120658F, -0.0886427F, -0.0653315037F, -0.0591102F, -0.0545216F,
      -0.0509376F, 0.0488232F, 0.0509313978F, 0.0607559F, 0.0679819F, 0.0895811F,
      0.121968F, 0.122723F, 0.1334555F, 0.143876F, 0.156297F, 0.15295F,
      0.202684F, 0.214838F, 0.210326F, 0.22148F, 0.221104F, 0.236594F, 0.245095F,
      0.239541009F, 0.251819F, 0.265409F, 0.266304F, 0.294699F, 0.307082F,
      0.314937F, 0.311439F, 0.312513F, 0.325117F, 0.328914F, 0.335470021F,
      0.333778F, 0.338443F, -0.334948F, -0.335076F, -0.335556F, -0.331255F,
      -0.322414F, -0.307139F, -0.29845F, -0.295375F, -0.282308F, -0.278421F,
      -0.266346F, -0.262806F, -0.224955F, -0.208354F, -0.201770991F, -0.176685F,
      -0.16885F, -0.148397F, -0.090736F, -0.0811248F, -0.0725658F, -0.0474917F,
      -0.0453143F, -0.0382129F, -0.0187572F, -0.0207586F, -0.0111153F,
      -0.00842784F, 0.0300909F, 0.0811286F, 0.0913921F, 0.110628F, 0.128859013F,
      0.174839F, 0.184998F, 0.187341F, 0.202214F, 0.219979F, 0.259287F,
      0.264972985F, 0.270226F, 0.286895F, 0.288926F, 0.291721F, 0.29845F,
      0.29845F, 0.310336F, 0.30804F, 0.32214433F, 0.322037F, 0.328752F,
      -0.338751F, -0.329243F, -0.321988F, -0.320479F, -0.311897F, -0.277420491F,
      -0.26978F, -0.222359F, -0.205573F, -0.186921F, -0.177782F, -0.133702F,
      -0.105148F, -0.107058F, -0.0998595953F, -0.0853838F, -0.0768307F,
      -0.0714178F, -0.0643222F, -0.0443069339F, -0.0340044F, -0.0365709F,
      -0.00846535F, 0.00879867F, 0.019162F, 0.0531512F, 0.0488422F, 0.068509F,
      0.083886F, 0.0808813F, 0.0892491F, 0.108748496F, 0.134928F, 0.137226F,
      0.150463F, 0.164559F, 0.171905F, 0.182548493F, 0.185896F, 0.205343F,
      0.241116494F, 0.2622F, 0.284592F, 0.277406F, 0.291928679F, 0.295244F,
      0.314713F, 0.31092F, 0.310812F, 0.311818331F, 0.323802F, 0.335927F,
      -0.337136984F, -0.327467F, -0.32749632F, -0.316058F, -0.311727F,
      -0.310370982F, -0.279852F, -0.272184F, -0.255924F, -0.249569F, -0.242197F,
      -0.204888F, -0.18468301F, -0.172895F, -0.170509F, -0.158102F, -0.138772F,
      -0.115942501F, -0.119745F, -0.103731F, -0.0938762F, -0.0881026536F,
      -0.0594831F, -0.0400144495F, -0.0246128F, -0.0233548F, -0.00998439F,
      -0.0011085F, 0.0152884F, 0.031773F, 0.0391696F, 0.0506109F, 0.061298F,
      0.0758109F, 0.0919231F, 0.0978183F, 0.097135F, 0.108255F, 0.119716503F,
      0.145445F, 0.144499987F, 0.156333F, 0.172251F, 0.18042F, 0.193777F,
      0.202048F, 0.217265F, 0.229462F, 0.239229F, 0.24387899F, 0.264002F,
      0.262936F, 0.28769F, 0.302232981F, 0.310859F, 0.309291F, 0.316556F,
      0.321787F, 0.324557F, 0.328763F, 0.338804483F, -0.333244F, -0.337167F,
      -0.325235F, -0.329140663F, -0.314162493F, -0.31627351F, -0.310564F,
      -0.312294F, -0.279219F, -0.258797F, -0.235887498F, -0.231113493F,
      -0.222715F, -0.216641F, -0.208214F, -0.193516F, -0.166555F, -0.145769F,
      -0.141538F, -0.128016F, -0.093734F, -0.0959305F, -0.084313F, -0.0657357F,
      -0.0688156039F, -0.0617357F, -0.040501F, 0.0058702F, 0.0139770508F,
      0.011495F, 0.0179469492F, 0.0344007F, 0.0404158495F, 0.0549875684F,
      0.051175F, 0.0596007F, 0.0611254F, 0.0923774F, 0.103084F, 0.115564F,
      0.14674F, 0.151124F, 0.190838501F, 0.201273501F, 0.21569F, 0.236776F,
      0.239239499F, 0.251917F, 0.264192F, 0.270797342F, 0.281864F, 0.287616F,
      0.305708F, 0.29845F, 0.309537321F, 0.314252F, 0.326943F, 0.323260337F,
      0.328552F, 0.338086F, -0.337205F, -0.325544F, -0.317545F, -0.310782492F,
      -0.312393F, -0.276018F, -0.266548F, -0.238179F, -0.223131F, -0.224433F,
      -0.212925F, -0.20046F, -0.176399F, -0.166438F, -0.16882F, -0.159522F,
      -0.0994059F, -0.088684F, -0.056724865F, -0.0423747F, -0.0234297F,
      -0.00489312F, 0.0226294F, 0.0233465F, 0.042137F, 0.0565143F, 0.0670268F,
      0.0776596F, 0.101631F, 0.131717F, 0.202665F, 0.20976F, 0.22367F,
      0.253148496F, 0.283199F, 0.28284F, 0.312348485F, 0.313843F, 0.323006511F,
      0.329053F, 0.334725F, -0.336953F, -0.326199F, -0.319145F, -0.314052F,
      -0.311457F, -0.310636F, -0.29845F, -0.263665F, -0.233915F, -0.222393F,
      -0.206442F, -0.170013F, -0.137690991F, -0.128100991F, -0.110599F,
      -0.0774349F, -0.0818699524F, -0.0697554F, -0.0601028F, -0.0161322504F,
      0.00667768F, 0.026354F, 0.0272093F, 0.0532808F, 0.0859595F, 0.0779680461F,
      0.087767F, 0.101047F, 0.113296494F, 0.130358F, 0.150175F, 0.149155F,
      0.159691F, 0.215534F, 0.2321F, 0.237688F, 0.278635F, 0.29371F, 0.305766F,
      0.315562F, 0.309057F, 0.309519F, 0.325347F, 0.321884F, 0.334349F,
      0.334581017F, -0.333642F, -0.338458F, -0.330479F, -0.305723F, -0.29845F,
      -0.249604F, -0.238472F, -0.24108F, -0.231915F, -0.215412F, -0.207889F,
      -0.208174F, -0.198235F, -0.188603F, -0.176028F, -0.139390498F, -0.131225F,
      -0.113312F, -0.0949848F, -0.077826F, -0.0750119686F, -0.0687305F,
      -0.0566283F, -0.0130884F, 0.000102799F, 0.0106090307F, 0.0145043F,
      0.0240701F, 0.03697F, 0.0329638347F, 0.0651367F, 0.0573318F, 0.076792F,
      0.0897522F, 0.090692F, 0.0993646F, 0.138846F, 0.146026F, 0.149287F,
      0.162091F, 0.18043F, 0.206392F, 0.213374F, 0.222543F, 0.227211F, 0.232635F,
      0.253739F, 0.268241F, 0.285357F, 0.284933F, 0.305705F, 0.315494F, 0.32234F,
      0.319772F, 0.335386F, -0.334517509F, -0.337538F, -0.333985F, -0.327386F,
      -0.332131F, -0.325874F, -0.321861F, -0.312529F, -0.297915F, -0.290625F,
      -0.286167F, -0.276355F, -0.269095F, -0.247486F, -0.246833F, -0.237974F,
      -0.206003487F, -0.200901F, -0.189762F, -0.168926F, -0.143374F,
      -0.135396495F, -0.136744F, -0.118201F, -0.106961F, -0.0897761509F,
      -0.0773951486F, -0.0769367F, -0.066017F, -0.041766F, -0.0175375F,
      -0.0104558F, 0.00783229F, 0.0258894F, 0.0365783F, 0.0280331F, 0.0449212F,
      0.0676738F, 0.10338F, 0.111004F, 0.124354F, 0.138713986F, 0.174228F,
      0.1745F, 0.191959F, 0.213991F, 0.223197F, 0.26755F, 0.280223F, 0.289061F,
      0.308744F, 0.311776519F, 0.321529F, 0.331442F, 0.332532346F, -0.335755497F,
      -0.333939F, -0.326325F, -0.317548513F, -0.314018F, -0.306354F, -0.279384F,
      -0.266904F, -0.240796F, -0.223249F, -0.224489F, -0.218411F, -0.201661F,
      -0.187561497F, -0.167697F, -0.165354F, -0.158781F, -0.149884F, -0.139699F,
      -0.110219F, -0.0931018F, -0.0369043F, -0.0323426F, -0.00437322492F,
      0.00515903F, 0.0171035F, 0.036252F, 0.0530992F, 0.0620464F, 0.0621393F,
      0.0742721F, 0.114101F, 0.136355F, 0.212946F, 0.226166F, 0.244157F,
      0.251975328F, 0.264291F, 0.273723F, 0.282083F, 0.314798F, 0.33078F,
      0.330286F, 0.331671F, 0.331907F, -0.338806F, -0.335442F, -0.331763F,
      -0.323230505F, -0.314199F, -0.318774F, -0.306792F, -0.295282F, -0.289959F,
      -0.26854F, -0.27056F, -0.196881F, -0.191570491F, -0.184614F, -0.180288F,
      -0.160327F, -0.1387095F, -0.116997F, -0.103483F, -0.104999F, -0.0333726F,
      -0.0240594F, -0.01841F, -0.0071199853F, 0.00179922464F, 0.0229493F,
      0.0324259F, 0.0522296F, 0.0627381F, 0.0840802938F, 0.109343F, 0.1213F,
      0.152331F, 0.157891F, 0.167753F, 0.174757F, 0.191493F, 0.201671F,
      0.210763F, 0.211274F, 0.258643F, 0.281405F, 0.306824F, 0.314029F,
      0.308789F, 0.334082484F, 0.331076F, 0.33865F, -0.33455F, -0.338485F,
      -0.337585F, -0.323537F, -0.319005F, -0.31742F, -0.309265018F,
      -0.309835494F, -0.26259F, -0.258736F, -0.246245F, -0.2273155F, -0.221369F,
      -0.199766F, -0.184589F, -0.184291F, -0.166078F, -0.15305F, -0.148987F,
      -0.135127F, -0.136349F, -0.13153F, -0.117284F, -0.116044F, -0.105644F,
      -0.0956744F, -0.0879253F, -0.0632316F, -0.0471762F, -0.0360486F,
      -0.0107250493F, 0.00351867F, -0.000372458F, 0.0411791F, 0.0570788F,
      0.0483127F, 0.0755026937F, 0.11294F, 0.128601F, 0.138664F, 0.149249F,
      0.153914F, 0.162966F, 0.17362F, 0.192908F, 0.208F, 0.221484F, 0.239242F,
      0.258621514F, 0.276222F, 0.287299F, 0.305702F, 0.309579F, 0.320719481F,
      0.320771486F, 0.327787F, 0.333164F, 0.332071F, 0.337304F, -0.305715F,
      -0.29845F, -0.270097F, -0.251285F, -0.226198F, -0.219922F, -0.196358F,
      -0.183492F, -0.174702F, -0.167861F, -0.138784F, -0.117755F, -0.105162F,
      -0.0706983F, -0.057857F, -0.022604F, 0.000954434F, 0.031992F, 0.0432081F,
      0.0522469F, 0.0622265F, 0.0602872F, 0.0864947F, 0.0910269F, 0.0891271F,
      0.100188F, 0.109018F, 0.163896F, 0.182255F, 0.197045F, 0.201928F,
      0.215822F, 0.222462F, 0.233302F, 0.256599F, 0.259195F, 0.26968F, 0.291982F,
      0.310014337F, 0.333336F, -0.335041F, -0.339818F, -0.339819342F,
      -0.335998505F, -0.315618515F, -0.318639F, -0.310378F, -0.308379F,
      -0.285746F, -0.275694F, -0.271446F, -0.267383665F, -0.25F, -0.241586F,
      -0.232654F, -0.218428F, -0.222127F, -0.197194F, -0.185414F, -0.168967992F,
      -0.150043F, -0.122001F, -0.104535F, -0.0924379F, -0.0902912F, -0.0600426F,
      -0.0490297F, -0.0367723F, -0.0393468F, -0.00517614F, -0.00043602346F,
      0.0131671503F, 0.0562203F, 0.0933125F, 0.0880898F, 0.121379F, 0.124994F,
      0.134248F, 0.152863F, 0.171094F, 0.18252F, 0.196636F, 0.204194F, 0.221297F,
      0.228008F, 0.242856F, 0.269505F, 0.284947F, 0.305799F, 0.301682F,
      0.309749F, 0.312937F, 0.312731F, 0.323515326F, 0.331423F, 0.339077F,
      -0.337907761F, -0.337566F, -0.316964656F, -0.320044488F, -0.318336487F,
      -0.304440022F, -0.310700983F, -0.282693F, -0.278224498F, -0.256608486F,
      -0.252173F, -0.215912F, -0.210239F, -0.173417F, -0.16652F, -0.142667F,
      -0.0956499F, -0.0507749F, -0.0367398486F, 0.00832185F, 0.0230729F,
      0.0421018F, 0.039751F, 0.0491949F, 0.0684484F, 0.06833F, 0.086271F,
      0.0916029F, 0.14586F, 0.150691F, 0.170368F, 0.185785F, 0.214231F,
      0.250911F, 0.261825F, 0.272085F, 0.277926F, 0.282675505F, 0.297133F,
      0.305705F, 0.310630322F, 0.31166935F, 0.317392F, 0.323354483F, 0.324222F,
      0.330608F, 0.332465F, 0.337472F, 0.34275949F, 0.338808F, -0.336589515F,
      -0.340765506F, -0.336517F, -0.325528502F, -0.330388248F, -0.318023F,
      -0.318894F, -0.306193F, -0.302271F, -0.296991F, -0.286525F, -0.275338F,
      -0.281319F, -0.237454F, -0.220562011F, -0.217732F, -0.210551F, -0.14528F,
      -0.134966F, -0.114199F, -0.0658569F, -0.0470761F, -0.040211767F,
      -0.0189625F, 0.00925463F, 0.0336098F, 0.0453629F, 0.0529202F, 0.0767487F,
      0.0796684F, 0.0979015F, 0.119046F, 0.125196F, 0.142988F, 0.141087F,
      0.154539F, 0.198167F, 0.225723F, 0.219159F, 0.251765F, 0.291367F,
      0.301466495F, 0.300773501F, 0.314369F, 0.30954951F, 0.30948F, 0.315183F,
      0.322974682F, 0.3209925F, 0.334549F, 0.340374F, 0.337955F, -0.342772F,
      -0.338923F, -0.331161F, -0.315546F, -0.303372502F, -0.30661F,
      -0.308025509F, -0.298412F, -0.300062984F, -0.295175493F, -0.298266F,
      -0.287195504F, -0.288257F, -0.277172F, -0.277611F, -0.277870238F,
      -0.263971984F, -0.259963512F, -0.256014F, -0.250656F, -0.246140495F,
      -0.245625F, -0.235995F, -0.237633258F, -0.240795F, -0.233448F,
      -0.231772497F, -0.216672F, -0.217024F, -0.222342491F, -0.209271327F,
      -0.20992F, -0.207503F, -0.199525505F, -0.196497F, -0.190378F, -0.184055F,
      -0.188247666F, -0.184847504F, -0.1812675F, -0.179922F, -0.170215F,
      -0.171253F, -0.159691F, -0.156830609F, -0.153039F, -0.162074F, -0.149333F,
      -0.14617601F, -0.15126F, -0.147239506F, -0.139123F, -0.138741F, -0.138412F,
      -0.128733F, -0.127429F, -0.121668F, -0.117860004F, -0.117761F,
      -0.106901996F, -0.108224005F, -0.108380005F, -0.0990147F, -0.0983806551F,
      -0.100897F, -0.0917324498F, -0.0916089F, -0.0890237F, -0.0770570636F,
      -0.0785901F, -0.0675793514F, -0.0639458448F, -0.0592008F, -0.0573635F,
      -0.0481015518F, -0.0520531F, -0.03526235F, -0.0374168F, -0.023428F,
      -0.0256002F, -0.025888F, -0.0131664F, -0.016031F, -0.0135754F, -0.0131461F,
      -0.00298622F, -0.00492312F, -0.00880566F, 0.00213174894F, 0.00687563F,
      0.00880367F, 0.00963115785F, 0.0174507F, 0.0344929F, 0.027417F, 0.0322155F,
      0.0308915321F, 0.0451395512F, 0.0536538512F, 0.0536804F, 0.054286F,
      0.0584658F, 0.0634998F, 0.0631438F, 0.0578757F, 0.0761007F, 0.0730785728F,
      0.0699562F, 0.0801296F, 0.0797103643F, 0.0950839F, 0.0923177302F,
      0.0915942F, 0.100886F, 0.101868346F, 0.106190331F, 0.101515F, 0.108205F,
      0.115530334F, 0.113169F, 0.124145F, 0.122873336F, 0.13182275F, 0.128976F,
      0.137891501F, 0.145283F, 0.151674F, 0.153575331F, 0.159842F, 0.157825F,
      0.165971503F, 0.174785495F, 0.173599496F, 0.173029F, 0.168824F, 0.180832F,
      0.185237F, 0.189941F, 0.191103F, 0.193548992F, 0.205786F, 0.202133507F,
      0.201451257F, 0.205027F, 0.215007F, 0.213473499F, 0.220952F, 0.222441494F,
      0.224659666F, 0.23451F, 0.227417F, 0.232146502F, 0.23486051F, 0.245299F,
      0.239599F, 0.246728F, 0.249932F, 0.253153F, 0.25571F, 0.249481F,
      0.264325023F, 0.258074F, 0.260390341F, 0.267289F, 0.273182601F,
      0.270097762F, 0.283280492F, 0.280194F, 0.283295333F, 0.293038F,
      0.292029679F, 0.300054491F, 0.302092493F, 0.301549494F, 0.298168F,
      0.312921F, 0.325989F, 0.332293F, 0.328319F, 0.342145F, -0.334371507F,
      -0.340228F, -0.33014F, -0.32768F, -0.323471F, -0.304373026F, -0.312229F,
      -0.297560334F, -0.298194F, -0.289036F, -0.290422F, -0.287961F, -0.280713F,
      -0.281898F, -0.26914224F, -0.258786F, -0.259262651F, -0.24658066F,
      -0.248046F, -0.240065008F, -0.239437F, -0.227243334F, -0.226309F,
      -0.216753F, -0.219974F, -0.204839498F, -0.195711F, -0.197732F, -0.18832F,
      -0.181836501F, -0.171546F, -0.168796F, -0.156821F, -0.158229008F,
      -0.148517F, -0.149873F, -0.13922675F, -0.138467F, -0.126341F,
      -0.128962755F, -0.113543F, -0.113898501F, -0.11051F, -0.0965343F,
      -0.0985051468F, -0.0862629F, -0.0886022747F, -0.0774509F, -0.0799809545F,
      -0.066085F, -0.0533286F, -0.062495F, -0.0509106517F, -0.0499579497F,
      -0.03968345F, -0.0285351239F, -0.030882F, -0.0217212F, -0.00566148665F,
      -0.00747944F, 0.00311376713F, 0.0156057F, 0.013806F, 0.0180779F,
      0.0314164758F, 0.0438763499F, 0.0524179488F, 0.0510468483F, 0.0603551492F,
      0.0731244F, 0.0778142F, 0.096277F, 0.0929688513F, 0.100921452F,
      0.112371504F, 0.111911F, 0.123695336F, 0.122334F, 0.13164334F, 0.128036F,
      0.145374F, 0.148119F, 0.161209F, 0.161093503F, 0.170859337F, 0.169267505F,
      0.185233504F, 0.192963988F, 0.189228F, 0.202768F, 0.201382F, 0.212344F,
      0.225717992F, 0.230458498F, 0.238467F, 0.242163509F, 0.251742F, 0.259483F,
      0.25908F, 0.273607016F, 0.273034334F, 0.282124F, 0.281267345F,
      0.291651815F, 0.289729F, 0.300353F, 0.3037F, 0.297178F, 0.34054F, 0.34092F,
      -0.3048F, -0.308854F, -0.311731F, -0.299939245F, -0.300002F, -0.298228F,
      -0.287168F, -0.292578F, -0.2889705F, -0.282379F, -0.278397F, -0.268752337F,
      -0.255795509F, -0.249105F, -0.248384F, -0.227999F, -0.231828F, -0.218694F,
      -0.219144F, -0.207003F, -0.210542F, -0.194321F, -0.193334F, -0.202454F,
      -0.183094F, -0.185811F, -0.175216F, -0.180093F, -0.177572012F, -0.168713F,
      -0.168779F, -0.157343F, -0.161356F, -0.162358F, -0.147755F, -0.150776F,
      -0.13567999F, -0.134409F, -0.128914F, -0.132248F, -0.120763F, -0.120324F,
      -0.110709667F, -0.108496502F, -0.0952048F, -0.0981457531F, -0.0983006507F,
      -0.0901975F, -0.0879416F, -0.0881034F, -0.0762985F, -0.0782669485F,
      -0.0635019F, -0.0728332F, -0.0704658F, -0.0651581362F, -0.0556135327F,
      -0.0550388F, -0.0573940501F, -0.0514003F, -0.0482506342F, -0.0413195F,
      -0.0360999F, -0.0366229676F, -0.0320966F, -0.024515F, -0.0321353F,
      -0.0165783986F, -0.0104792F, -0.0089361649F, -0.00624719F, 0.00182512577F,
      0.00137505F, -0.00252631F, 0.0121680535F, 0.0126651507F, 0.0243205F,
      0.0216757506F, 0.0186175F, 0.0321818F, 0.0363228F, 0.0304339F, 0.0397819F,
      0.0428683981F, 0.0526989028F, 0.0540016F, 0.0562855F, 0.063414F,
      0.0590956509F, 0.0735385045F, 0.0736331F, 0.0741302F, 0.0734035224F,
      0.0799846F, 0.0798932F, 0.0860666F, 0.0908776F, 0.0914402F, 0.101561673F,
      0.0993600041F, 0.100797266F, 0.103386F, 0.107486F, 0.114205F, 0.11057F,
      0.125373F, 0.135655F, 0.129537255F, 0.134402F, 0.139129505F, 0.143114F,
      0.156913F, 0.15409F, 0.161168F, 0.165104F, 0.171063513F, 0.175102F,
      0.172705F, 0.183602989F, 0.179976493F, 0.182617992F, 0.188872F, 0.194781F,
      0.195319F, 0.191379011F, 0.204379246F, 0.201623F, 0.202998F, 0.202278674F,
      0.208636F, 0.210794508F, 0.213206F, 0.222449F, 0.219290495F, 0.227936F,
      0.242349F, 0.242488F, 0.243006F, 0.252285F, 0.260045499F, 0.259493F,
      0.258358F, 0.270673F, 0.283887506F, 0.286379516F, 0.283840507F, 0.29066F,
      0.29616F, 0.291136F, 0.294406F, 0.305161744F, 0.303575F, 0.308838F,
      0.317395F, -0.316894F, -0.317658F, -0.321139F, -0.306803495F, -0.305553F,
      -0.29769F, -0.298923016F, -0.295776F, -0.286003F, -0.281547F,
      -0.278800339F, -0.276391506F, -0.266875982F, -0.255667F, -0.25202F,
      -0.246185496F, -0.236456499F, -0.236326F, -0.229828F, -0.223651F,
      -0.215575F, -0.21213F, -0.211088F, -0.202667F, -0.199295506F, -0.19731F,
      -0.191502512F, -0.177799746F, -0.176686496F, -0.165849F, -0.158991992F,
      -0.154636F, -0.147454F, -0.152379F, -0.142817F, -0.128F, -0.124744333F,
      -0.122275F, -0.118148F, -0.104452F, -0.107725501F, -0.0960051641F,
      -0.0946512F, -0.0856907368F, -0.0768089F, -0.0663128048F, -0.0588176474F,
      -0.0460375F, -0.0514892F, -0.036437951F, -0.0281901676F, -0.0323826F,
      -0.0207761F, -0.0174819678F, -0.00739175966F, -0.0127975F, 0.00660515F,
      0.0077073F, 0.0230793748F, 0.0245744F, 0.0330911F, 0.0312614329F,
      0.0380287F, 0.0395893F, 0.0529234F, 0.068712F, 0.0762341F, 0.0832947F,
      0.085229F, 0.0950652F, 0.106933F, 0.0989058F, 0.113132499F, 0.122154F,
      0.127535F, 0.134055F, 0.143705F, 0.143162F, 0.153395F, 0.157909F,
      0.161918F, 0.172608674F, 0.168756F, 0.18616F, 0.181441665F, 0.193453491F,
      0.201391F, 0.202325F, 0.212195009F, 0.219138667F, 0.237103F, 0.240948F,
      0.250385493F, 0.259809F, 0.260398507F, 0.271302491F, 0.272571683F,
      0.280742F, 0.280194F, 0.290533334F, 0.289357483F, 0.291214F, 0.300629973F,
      0.299852669F, 0.308461F, 0.309227F, 0.316977F, 0.317716F, 0.324041F,
      0.319409F, -0.650907F, -0.649509F, -0.649154663F, -0.64909631F, -0.648425F,
      -0.646304488F, -0.648335516F, -0.646326F, -0.64709574F, -0.647531F,
      -0.645391F, -0.64588F, -0.646475F, -0.646765471F, -0.64585F, -0.646499634F,
      -0.647587359F, -0.64429F, -0.647437394F, -0.64593F, -0.646649957F,
      -0.647017956F, -0.645609F, -0.647451F, -0.64496F, -0.646023035F,
      -0.650037F, -0.645032F, -0.646700501F, -0.648769498F, -0.647531F,
      -0.647976518F, -0.649403F, -0.647323F, -0.648153F, -0.64575249F,
      -0.644975F, -0.646874309F, -0.646653473F, -0.648763776F, -0.645567F,
      -0.644443F, -0.645295F, -0.648207F, -0.646378517F, -0.649395347F,
      -0.646091521F, -0.649634F, -0.647203F, -0.64733F, -0.644771F, -0.646631F,
      -0.647158504F, -0.64808F, -0.6492F, -0.645465F, -0.648655355F, -0.649506F,
      -0.645577F, -0.646354F, -0.646113F, -0.649313F, -0.64765F, -0.64984F,
      -0.648534358F, -0.648144484F, -0.646179497F, -0.649145F, -0.647500634F,
      -0.647197F, -0.646127F, -0.64451F, -0.645617F, -0.646728F, -0.647229493F,
      -0.645135522F, -0.649646F, -0.647617F, -0.648873F, -0.648952365F,
      -0.64726305F, -0.646768034F, -0.64773953F, -0.650113F, -0.644749F,
      -0.647367477F, -0.645803511F, -0.64561F, -0.649503F, -0.650052F,
      -0.646412F, -0.64813F, -0.644868F, -0.648423F, -0.645627499F, -0.645627F,
      -0.644915F, -0.646638F, -0.649677F, -0.644861F, -0.645766F, -0.648159504F,
      -0.647927523F, -0.646351F, -0.646534324F, -0.648537517F, -0.649981F,
      -0.648240685F, -0.648962F, -0.64541F, -0.646704F, -0.646350503F,
      -0.647799969F, -0.649623F, -0.645050526F, -0.645717F, -0.649908F,
      -0.650015F, -0.645074F, -0.644743F, -0.647974491F, -0.646405F, -0.64662F,
      -0.649366796F, -0.646119F, -0.647629499F, -0.648798704F, -0.650136471F,
      -0.64617F, -0.646548331F, -0.651156306F, -0.649265647F, -0.648916364F,
      -0.650429308F, -0.645003F, -0.652792F, -0.645232F, -0.649381F, -0.647644F,
      -0.647433F, -0.644557F, -0.642314F, -0.643987F, -0.640992F, -0.643023F,
      -0.638785958F, -0.639027F, -0.638694F, -0.638079286F, -0.639838F,
      -0.641809F, -0.639663458F, -0.642460346F, -0.637005508F, -0.639318347F,
      -0.642416F, -0.638772666F, -0.640069723F, -0.643658F, -0.643174F,
      -0.64048934F, -0.634614468F, -0.643195F, -0.635506F, -0.640784F,
      -0.643737F, -0.642939F, -0.638877F, -0.63770026F, -0.642951965F,
      -0.639328F, -0.636249F, -0.642195F, -0.643519F, -0.640327752F, -0.642239F,
      -0.635035F, -0.637259F, -0.642857F, -0.637382507F, -0.641286969F,
      -0.637503505F, -0.642885F, -0.642105F, -0.635902524F, -0.643917F,
      -0.643315F, -0.639247F, -0.64051497F, -0.642499F, -0.638269484F,
      -0.640939474F, -0.638187F, -0.64227F, -0.639681F, -0.638979495F,
      -0.637661338F, -0.640627503F, -0.643355F, -0.64351F, -0.637102F,
      -0.637937F, -0.635175F, -0.636805F, -0.64104F, -0.638470173F,
      -0.639327526F, -0.63847965F, -0.637717F, -0.643431F, -0.643261F,
      -0.637075F, -0.638053596F, -0.639524758F, -0.639948F, -0.63872F,
      -0.638147F, -0.64298F, -0.639740467F, -0.640425F, -0.635421F, -0.63529F,
      -0.63971F, -0.634487F, -0.638813F, -0.638620734F, -0.64212F, -0.641005039F,
      -0.640423536F, -0.643334031F, -0.63989F, -0.636761F, -0.64261049F,
      -0.641336F, -0.639661551F, -0.639523506F, -0.642979F, -0.634566F,
      -0.643579F, -0.637679F, -0.636895F, -0.641741F, -0.638689518F, -0.643489F,
      -0.643612F, -0.643939F, -0.641115F, -0.641623F, -0.636927F, -0.636807F,
      -0.635855F, -0.642569F, -0.638584316F, -0.640964F, -0.642513F, -0.635502F,
      -0.64302F, -0.640792F, -0.63794F, -0.637704492F, -0.637899F, -0.643579F,
      -0.637925327F, -0.640950739F, -0.635508776F, -0.636583507F, -0.63959527F,
      -0.638961256F, -0.638025F, -0.639326F, -0.639625F, -0.636054F, -0.635388F,
      -0.640615F, -0.639861465F, -0.625718F, -0.627863F, -0.628953F, -0.628565F,
      -0.630161F, -0.632789F, -0.629460335F, -0.625518F, -0.629000485F,
      -0.628206968F, -0.624328F, -0.627197325F, -0.628248F, -0.62604636F,
      -0.628133F, -0.630519032F, -0.627727F, -0.624902F, -0.62686F, -0.627282F,
      -0.631543F, -0.628368497F, -0.627340198F, -0.629882F, -0.62786F,
      -0.632074F, -0.628981F, -0.624735F, -0.627052F, -0.630331635F, -0.633818F,
      -0.633705F, -0.62708F, -0.629168F, -0.628307F, -0.628527F, -0.628325522F,
      -0.624748F, -0.627706F, -0.625951F, -0.631648F, -0.627636F, -0.627975F,
      -0.63073051F, -0.627276F, -0.627796352F, -0.624736F, -0.630705F,
      -0.62793231F, -0.62958467F, -0.626589F, -0.629585505F, -0.624248F,
      -0.627402F, -0.631296F, -0.628773689F, -0.628011465F, -0.629886329F,
      -0.633642F, -0.625737F, -0.628257275F, -0.62738651F, -0.631411F,
      -0.629334F, -0.628038526F, -0.628666818F, -0.63324F, -0.62774235F,
      -0.624868512F, -0.624416F, -0.627335F, -0.632652F, -0.626647F,
      -0.630674481F, -0.630245F, -0.628169477F, -0.629874F, -0.630592F,
      -0.630351F, -0.633224F, -0.627088547F, -0.631038487F, -0.629038513F,
      -0.631155312F, -0.625372F, -0.625456F, -0.629957497F, -0.628589749F,
      -0.624069F, -0.629603744F, -0.62691F, -0.628142178F, -0.627896667F,
      -0.631478F, -0.628675F, -0.628779531F, -0.631352F, -0.628443F, -0.627417F,
      -0.631447F, -0.626906693F, -0.62977463F, -0.63091445F, -0.629066F,
      -0.626046F, -0.630151F, -0.631802F, -0.626456F, -0.626508F, -0.62911F,
      -0.629474F, -0.624777F, -0.630988955F, -0.628455698F, -0.625231F,
      -0.625044F, -0.631585F, -0.627245307F, -0.630754F, -0.627689F, -0.631938F,
      -0.62609F, -0.630781F, -0.629086F, -0.614421F, -0.621839F, -0.619460464F,
      -0.62211F, -0.614968F, -0.617140532F, -0.616227031F, -0.61595F, -0.620733F,
      -0.616828F, -0.621286F, -0.619525492F, -0.622664511F, -0.622726F,
      -0.616704F, -0.61847949F, -0.619819045F, -0.620069F, -0.622017324F,
      -0.61727F, -0.619167507F, -0.621589F, -0.623078F, -0.618394F,
      -0.619668663F, -0.620169F, -0.620276F, -0.621126652F, -0.619193F,
      -0.620776F, -0.621669F, -0.61753F, -0.619973F, -0.620986F, -0.618661F,
      -0.617783F, -0.621538F, -0.619847F, -0.619897F, -0.620632052F, -0.618278F,
      -0.620677F, -0.619419F, -0.619556487F, -0.620321F, -0.620494F, -0.621879F,
      -0.618623F, -0.621014357F, -0.619897F, -0.619733F, -0.621264F, -0.620422F,
      -0.623722F, -0.620324F, -0.621297F, -0.62004F, -0.620242953F, -0.621916F,
      -0.619084F, -0.617403F, -0.62235F, -0.621929F, -0.61709F, -0.618602F,
      -0.618905306F, -0.617164F, -0.620802522F, -0.623159F, -0.618141F,
      -0.618386507F, -0.619746506F, -0.617066F, -0.623046F, -0.618638F,
      -0.620223F, -0.621778F, -0.617413F, -0.617138F, -0.620267F, -0.619306326F,
      -0.618961513F, -0.623531F, -0.617147F, -0.617438F, -0.621418F, -0.621061F,
      -0.621640682F, -0.620622F, -0.618918F, -0.620553F, -0.619492352F,
      -0.623737F, -0.621184468F, -0.61919F, -0.61981F, -0.620259F, -0.620091F,
      -0.621714F, -0.623653F, -0.61852F, -0.622244F, -0.618434F, -0.622127F,
      -0.620321F, -0.620772958F, -0.618880689F, -0.619840682F, -0.620272F,
      -0.621247F, -0.62378F, -0.618653F, -0.618358F, -0.619921327F, -0.618148F,
      -0.622061312F, -0.619722426F, -0.618383765F, -0.621949F, -0.618374F,
      -0.616798F, -0.622997522F, -0.621888F, -0.618475497F, -0.621392F,
      -0.61986F, -0.623533F, -0.62261F, -0.616782F, -0.617130518F, -0.619627237F,
      -0.618449F, -0.623879F, -0.620534F, -0.618463337F, -0.620653689F,
      -0.617975F, -0.619664F, -0.621717334F, -0.62364F, -0.621889F, -0.622695F,
      -0.620958F, -0.619767547F, -0.619234502F, -0.621051F, -0.621317327F,
      -0.62075F, -0.621920466F, -0.620559F, -0.621056F, -0.62265F, -0.619306F,
      -0.620312452F, -0.620933F, -0.615312F, -0.615631F, -0.617732F, -0.61595F,
      -0.61595F, -0.621691F, -0.619567513F, -0.616297F, -0.616433F,
      -0.608794332F, -0.610876F, -0.606652498F, -0.60772723F, -0.610218F,
      -0.607679486F, -0.606559F, -0.609444F, -0.609762669F, -0.611757F,
      -0.605486035F, -0.608779F, -0.609601F, -0.609601F, -0.609014034F,
      -0.606544F, -0.605561F, -0.606285512F, -0.606241524F, -0.607642F,
      -0.609601F, -0.609601F, -0.609601F, -0.606827F, -0.604103F, -0.609601F,
      -0.605893F, -0.609601F, -0.605273F, -0.60932F, -0.606948F, -0.604781F,
      -0.609601F, -0.604041F, -0.605935F, -0.609601F, -0.605633F, -0.606014F,
      -0.60682F, -0.609601F, -0.607935F, -0.608198F, -0.608048499F, -0.607726F,
      -0.607175F, -0.609601F, -0.607629F, -0.607834F, -0.606933951F, -0.608759F,
      -0.606162F, -0.609601F, -0.613714F, -0.612877F, -0.609386504F,
      -0.61224854F, -0.609679F, -0.60839F, -0.610612F, -0.604054F, -0.609076F,
      -0.613004F, -0.609290957F, -0.594335F, -0.596493F, -0.599333F,
      -0.60077703F, -0.602074F, -0.598003328F, -0.598616719F, -0.602212F,
      -0.598648965F, -0.59498F, -0.600348F, -0.600233316F, -0.597103F,
      -0.603633F, -0.601094F, -0.599217F, -0.603998F, -0.600533F, -0.603031F,
      -0.595342F, -0.601931036F, -0.599903F, -0.597283959F, -0.600909F,
      -0.600838F, -0.60032F, -0.59424F, -0.595368F, -0.603299F, -0.600643F,
      -0.599423F, -0.594979F, -0.595579F, -0.600752473F, -0.59791851F,
      -0.602376F, -0.59720552F, -0.601967F, -0.598912477F, -0.594402F,
      -0.594532F, -0.603682F, -0.597319F, -0.597064F, -0.598706F, -0.601593F,
      -0.603585482F, -0.595702F, -0.599338F, -0.599551499F, -0.598314F,
      -0.598756671F, -0.591715336F, -0.58963F, -0.590911686F, -0.585987F,
      -0.591344F, -0.592105031F, -0.591534F, -0.590178F, -0.590849F, -0.591905F,
      -0.584658F, -0.592366F, -0.5921F, -0.589126F, -0.590831F, -0.588081F,
      -0.590081F, -0.590071F, -0.586304F, -0.585223F, -0.58641F, -0.588737F,
      -0.592115F, -0.584162F, -0.590342045F, -0.585534F, -0.584998F, -0.590579F,
      -0.592661F, -0.591507F, -0.588787F, -0.587385F, -0.58921051F, -0.593297F,
      -0.589453459F, -0.586789F, -0.585011F, -0.58436F, -0.590671F,
      -0.589714527F, -0.588906F, -0.585735F, -0.589794F, -0.588841F, -0.591755F,
      -0.585499F, -0.584761F, -0.591001034F, -0.593177F, -0.59163F, -0.593001F,
      -0.593826F, -0.592345F, -0.589225F, -0.581383F, -0.581809F, -0.583364964F,
      -0.577317476F, -0.582603F, -0.576992F, -0.57802546F, -0.57523F, -0.576679F,
      -0.578944F, -0.58078F, -0.582406521F, -0.574531F, -0.580744F, -0.583182F,
      -0.57556F, -0.580527F, -0.580908F, -0.582291F, -0.576361F, -0.581676F,
      -0.582821F, -0.580193F, -0.582938F, -0.579724F, -0.577792F, -0.579088F,
      -0.576473F, -0.579518F, -0.579428F, -0.578273356F, -0.580744F, -0.575934F,
      -0.581788F, -0.579101503F, -0.574358F, -0.582591F, -0.582619F, -0.580942F,
      -0.578859508F, -0.576281F, -0.580818F, -0.577523F, -0.578097F, -0.578149F,
      -0.582702F, -0.582854F, -0.57739F, -0.5807F, -0.57546F, -0.581438F,
      -0.580762506F, -0.574246F, -0.579049F, -0.58043F, -0.57554F, -0.579678F,
      -0.580612F, -0.568415F, -0.567924F, -0.565109F, -0.573865F, -0.568981F,
      -0.567505F, -0.564033F, -0.567605F, -0.571235F, -0.573559F, -0.56738F,
      -0.565872F, -0.566424F, -0.567053F, -0.57105F, -0.564737F, -0.566920042F,
      -0.56896F, -0.566486F, -0.571597F, -0.565734F, -0.573875F, -0.567818F,
      -0.569577F, -0.566596F, -0.569552F, -0.568138F, -0.571771F, -0.565177F,
      -0.564858F, -0.570524F, -0.568912F, -0.565644F, -0.568504F, -0.565257F,
      -0.567285F, -0.569239F, -0.567062F, -0.568726F, -0.565758F, -0.570767F,
      -0.566065F, -0.567062F, -0.570477F, -0.567835F, -0.554426F, -0.558904648F,
      -0.556742F, -0.559278F, -0.55575F, -0.557626F, -0.561178F, -0.562989F,
      -0.5566F, -0.554794F, -0.555523515F, -0.559176F, -0.555707F, -0.55929F,
      -0.558948F, -0.554119F, -0.558729F, -0.55427F, -0.555088F, -0.56115F,
      -0.554206F, -0.55549F, -0.556979F, -0.559769511F, -0.56109F, -0.563299F,
      -0.560556F, -0.561509F, -0.559307F, -0.557791F, -0.556215F, -0.556781F,
      -0.557678F, -0.554125F, -0.557909966F, -0.561264F, -0.559439F, -0.563227F,
      -0.561644F, -0.560926F, -0.55642F, -0.559078F, -0.555887F, -0.560348F,
      -0.561859F, -0.558039F, -0.558061F, -0.555508F, -0.546148F, -0.547826F,
      -0.544558F, -0.552585F, -0.548829F, -0.551465F, -0.549843F, -0.549786F,
      -0.550734043F, -0.546597481F, -0.552214F, -0.547873497F, -0.550086F,
      -0.552203476F, -0.547907F, -0.5493415F, -0.544931F, -0.546436F, -0.546858F,
      -0.545288F, -0.550286531F, -0.545146F, -0.544222F, -0.553244F, -0.544409F,
      -0.547791F, -0.552839F, -0.547312F, -0.552474F, -0.551F, -0.55336F,
      -0.548327F, -0.551768541F, -0.549480498F, -0.546089F, -0.54507F,
      -0.553863F, -0.54976052F, -0.552703F, -0.552333F, -0.550786F, -0.548991F,
      -0.551417947F, -0.549865F, -0.551728F, -0.550151F, -0.546055F, -0.54496F,
      -0.546463F, -0.547253F, -0.551399F, -0.551921F, -0.545932651F, -0.550147F,
      -0.535635F, -0.538848F, -0.542075F, -0.536756F, -0.538075F, -0.541247F,
      -0.537048F, -0.541846F, -0.538091F, -0.543411F, -0.539286494F, -0.535145F,
      -0.541236F, -0.543375F, -0.535404F, -0.537954F, -0.54335F, -0.539232969F,
      -0.538863F, -0.539068F, -0.542101502F, -0.542432F, -0.542571F, -0.541443F,
      -0.544F, -0.543893F, -0.535309F, -0.53635F, -0.542585F, -0.543666F,
      -0.542454F, -0.54158F, -0.539853F, -0.537395F, -0.541428F, -0.539641F,
      -0.540707F, -0.540047F, -0.537362F, -0.536392F, -0.534325F, -0.539005F,
      -0.542557F, -0.536796F, -0.536234498F, -0.530476511F, -0.527868F,
      -0.528318F, -0.527754F, -0.530368F, -0.532421F, -0.530798F, -0.526723F,
      -0.526239F, -0.531402F, -0.527639F, -0.524551F, -0.533037F, -0.524322F,
      -0.528803468F, -0.532441F, -0.530854F, -0.532396F, -0.525721F, -0.527006F,
      -0.531176F, -0.524446F, -0.532601F, -0.527274966F, -0.527772486F,
      -0.533995F, -0.530166F, -0.527565479F, -0.529353F, -0.527162F,
      -0.529821038F, -0.527304F, -0.526728F, -0.532931F, -0.526629F,
      -0.529462457F, -0.52489F, -0.533209F, -0.531093F, -0.532539F, -0.527456F,
      -0.53186F, -0.526546F, -0.530305F, -0.524705F, -0.526207F, -0.528271F,
      -0.530806F, -0.515293F, -0.518177688F, -0.522889F, -0.519553F, -0.514143F,
      -0.516483F, -0.522477F, -0.522905F, -0.523572F, -0.517451F, -0.521997F,
      -0.520207524F, -0.520537F, -0.523233F, -0.518594F, -0.515331F, -0.521563F,
      -0.514268F, -0.518177F, -0.519531F, -0.519446F, -0.519657F, -0.52098453F,
      -0.516361F, -0.521120548F, -0.517472F, -0.520184517F, -0.518022478F,
      -0.517422F, -0.520058036F, -0.514854F, -0.51883F, -0.517342F, -0.523872F,
      -0.516365528F, -0.515629F, -0.523377F, -0.521141F, -0.518995F,
      -0.520826459F, -0.522153F, -0.522009F, -0.517697F, -0.519250035F,
      -0.519828F, -0.517988F, -0.51911F, -0.523979F, -0.523132F, -0.523355F,
      -0.51456F, -0.520618F, -0.515444F, -0.518284F, -0.51415F, -0.516369F,
      -0.519957F, -0.521858F, -0.521460652F, -0.518203F, -0.519392F, -0.515219F,
      -0.515724F, -0.510518491F, -0.506907F, -0.508951F, -0.508591533F,
      -0.508677F, -0.511203F, -0.504562497F, -0.512798F, -0.508276463F,
      -0.510381F, -0.507091F, -0.509759F, -0.510802F, -0.504124F, -0.504621F,
      -0.508423F, -0.512733F, -0.513413F, -0.509658F, -0.511045F, -0.507499F,
      -0.506317496F, -0.509126F, -0.507401F, -0.505172F, -0.504528F, -0.506243F,
      -0.507808F, -0.510022F, -0.506836F, -0.512314498F, -0.508843F, -0.505866F,
      -0.506285F, -0.506399F, -0.506879F, -0.506152F, -0.505303F, -0.513235F,
      -0.512539F, -0.506435F, -0.505454F, -0.507195F, -0.506898F, -0.510143F,
      -0.50477F, -0.512735F, -0.509069502F, -0.499671F, -0.497075021F,
      -0.497788489F, -0.503344F, -0.503466F, -0.495562F, -0.498722F,
      -0.498822033F, -0.497077F, -0.500175F, -0.50245F, -0.499638736F,
      -0.495634F, -0.503236F, -0.497312F, -0.503542F, -0.498104F, -0.496393F,
      -0.498183F, -0.497272F, -0.501132F, -0.497774F, -0.496625F, -0.497151F,
      -0.502888F, -0.494873F, -0.496974498F, -0.498257F, -0.496232F, -0.494575F,
      -0.497502F, -0.497446507F, -0.500373F, -0.499558F, -0.502867F, -0.498052F,
      -0.499117F, -0.502557F, -0.501284F, -0.501096F, -0.503322F, -0.49869F,
      -0.498941F, -0.49519F, -0.498281F, -0.503486F, -0.49506551F, -0.503618F,
      -0.501666F, -0.503329F, -0.50227F, -0.499698F, -0.498424F, -0.49627F,
      -0.499041F, -0.499282F, -0.500681639F, -0.496906F, -0.496207F, -0.500821F,
      -0.496363F, -0.491996F, -0.485293F, -0.485645F, -0.484597F, -0.490559F,
      -0.487013F, -0.489607F, -0.488471F, -0.490554F, -0.488114F, -0.488241F,
      -0.488038F, -0.486531F, -0.487294F, -0.492183F, -0.487403F, -0.484939F,
      -0.492638499F, -0.489268482F, -0.486648F, -0.493922F, -0.484185F,
      -0.491678F, -0.492450505F, -0.485646F, -0.493865F, -0.484051F, -0.484257F,
      -0.490516F, -0.492260486F, -0.493566F, -0.486091F, -0.489938021F,
      -0.493633F, -0.49083F, -0.484521F, -0.491767F, -0.490403F, -0.484702F,
      -0.492994F, -0.485186F, -0.490535F, -0.491841F, -0.490531F, -0.488206F,
      -0.492865F, -0.489542F, -0.486185F, -0.488853F, -0.49113F, -0.485938F,
      -0.488621F, -0.485318F, -0.490277F, -0.49305F, -0.488948494F, -0.485783F,
      -0.479367018F, -0.476444F, -0.475508F, -0.475127F, -0.483867F,
      -0.481788516F, -0.477547F, -0.483007F, -0.478563F, -0.478342F, -0.478903F,
      -0.480793F, -0.475399F, -0.481836F, -0.48133F, -0.478277F, -0.476208F,
      -0.475508F, -0.480266F, -0.476495F, -0.48284F, -0.475874F, -0.477364F,
      -0.477954F, -0.474305F, -0.480902493F, -0.477262F, -0.481479F, -0.48024F,
      -0.476832509F, -0.479869F, -0.482917F, -0.475535F, -0.481373F, -0.48381F,
      -0.475768F, -0.477046F, -0.475981F, -0.481363F, -0.475186F, -0.479893506F,
      -0.476379F, -0.480729F, -0.478454F, -0.477357F, -0.475081F, -0.475556F,
      -0.477915F, -0.477510512F, -0.478591F, -0.478261F, -0.466729F, -0.470895F,
      -0.472709F, -0.470643F, -0.467357F, -0.473104F, -0.469666F, -0.464185F,
      -0.473117F, -0.464981F, -0.473315F, -0.468336016F, -0.46458F, -0.466569F,
      -0.468477F, -0.472656F, -0.465297F, -0.470022F, -0.472231F, -0.469908F,
      -0.471489F, -0.466248F, -0.46571F, -0.473402F, -0.471029F, -0.46772F,
      -0.465940475F, -0.465269F, -0.467563F, -0.469446F, -0.464373F, -0.465151F,
      -0.465285F, -0.465782F, -0.466598F, -0.467345F, -0.471472F, -0.474003F,
      -0.467371F, -0.468181759F, -0.464616F, -0.471480668F, -0.472955F,
      -0.458156F, -0.458682F, -0.458017498F, -0.462526F, -0.456042F, -0.463741F,
      -0.463734F, -0.454335F, -0.4580625F, -0.460099F, -0.459551F, -0.458586F,
      -0.463438F, -0.455881F, -0.457050502F, -0.458428F, -0.460451F, -0.463162F,
      -0.457947F, -0.459583F, -0.462057F, -0.46069F, -0.455922F, -0.458568F,
      -0.454223F, -0.458804488F, -0.45938F, -0.4560785F, -0.45616F, -0.461869F,
      -0.462199F, -0.462303F, -0.460237F, -0.463902F, -0.46174F, -0.458754F,
      -0.463896F, -0.454692F, -0.457593F, -0.458055F, -0.456967324F, -0.463269F,
      -0.460266F, -0.455837F, -0.455746F, -0.454598F, -0.460292488F, -0.454889F,
      -0.456018329F, -0.463322F, -0.456711F, -0.460991F, -0.459809F, -0.462253F,
      -0.463015F, -0.455108F, -0.456542F, -0.457602F, -0.453208F, -0.450721502F,
      -0.449944496F, -0.444353F, -0.445464F, -0.447865F, -0.446984F, -0.444033F,
      -0.450395F, -0.452405F, -0.445113F, -0.445772F, -0.447743F, -0.451487F,
      -0.447061F, -0.449207F, -0.453094F, -0.448407501F, -0.450257F, -0.450823F,
      -0.449709326F, -0.452199F, -0.44956F, -0.448361F, -0.447476506F,
      -0.445046F, -0.451949F, -0.445005F, -0.449819505F, -0.444347F, -0.444111F,
      -0.450338F, -0.453094F, -0.449487F, -0.448432F, -0.445774F, -0.453055F,
      -0.452385F, -0.452225F, -0.446431F, -0.444227F, -0.44645F, -0.453097F,
      -0.446736F, -0.448785F, -0.450649F, -0.452023F, -0.448259F, -0.446020484F,
      -0.452808F, -0.441063F, -0.434074F, -0.441765F, -0.436375F, -0.442835F,
      -0.43731F, -0.44203F, -0.439903498F, -0.436556F, -0.44342F, -0.436889F,
      -0.440423F, -0.442576F, -0.434951F, -0.437592506F, -0.435371F, -0.442161F,
      -0.441085517F, -0.443147F, -0.443897F, -0.441802F, -0.442744F, -0.435625F,
      -0.440343F, -0.439926F, -0.43875F, -0.443883F, -0.440342F, -0.443945F,
      -0.434738F, -0.440982F, -0.440061F, -0.444008F, -0.442916F, -0.440598F,
      -0.434071F, -0.435876F, -0.438595F, -0.44225F, -0.439915985F, -0.444008F,
      -0.437159F, -0.437918484F, -0.434729F, -0.435101F, -0.434679F,
      -0.441205323F, -0.43808198F, -0.443015F, -0.434628487F, -0.437646F,
      -0.433486F, -0.429137F, -0.428709984F, -0.426387F, -0.430904F, -0.429174F,
      -0.425908F, -0.429088503F, -0.431914F, -0.428286F, -0.428487F, -0.427499F,
      -0.426189F, -0.42968F, -0.432082F, -0.431526F, -0.426105F, -0.428875F,
      -0.431099683F, -0.430243F, -0.4267F, -0.425689F, -0.430717F, -0.431931F,
      -0.424742F, -0.432459F, -0.42682F, -0.433445F, -0.426842F, -0.427736F,
      -0.425772F, -0.432243F, -0.425295F, -0.432404F, -0.433352F, -0.431163F,
      -0.430241F, -0.430318F, -0.430845F, -0.425481F, -0.42741F, -0.427944F,
      -0.425731F, -0.424759F, -0.428375F, -0.42815F, -0.425155F, -0.430585F,
      -0.426703513F, -0.42684F, -0.426766F, -0.429221F, -0.430629F, -0.41709F,
      -0.419568F, -0.417899F, -0.423343F, -0.420737505F, -0.422151F, -0.423971F,
      -0.414393F, -0.421583F, -0.416506F, -0.417226F, -0.423735F, -0.423548F,
      -0.419946F, -0.421794F, -0.423987F, -0.415229F, -0.419062495F,
      -0.419049978F, -0.419884F, -0.420132518F, -0.421736479F, -0.417743F,
      -0.418329F, -0.414228F, -0.418069F, -0.420505F, -0.420257F, -0.418378F,
      -0.417946F, -0.415462494F, -0.423153F, -0.416321F, -0.415477F, -0.418693F,
      -0.421944F, -0.419964F, -0.415466F, -0.417790473F, -0.416723F, -0.421668F,
      -0.415421F, -0.423712F, -0.42147F, -0.423395F, -0.423276F, -0.423233F,
      -0.418619F, -0.415708F, -0.415732F, -0.41909802F, -0.417018503F,
      -0.408623F, -0.409244F, -0.413551509F, -0.407988F, -0.404154F, -0.409413F,
      -0.408031F, -0.408704519F, -0.408456F, -0.411163F, -0.411227F, -0.4133F,
      -0.411142F, -0.406939F, -0.412553F, -0.406803F, -0.406427F, -0.405865F,
      -0.413856F, -0.40807F, -0.407176F, -0.413535F, -0.412133F, -0.405909F,
      -0.406540513F, -0.413237F, -0.410155505F, -0.404858F, -0.406613F,
      -0.408643F, -0.407302F, -0.406106F, -0.409968F, -0.407141F, -0.408827F,
      -0.411394F, -0.412804F, -0.409749F, -0.40416F, -0.40456F, -0.412591517F,
      -0.409568F, -0.410642F, -0.407899F, -0.410793662F, -0.405107F, -0.413692F,
      -0.407367498F, -0.407887F, -0.413264F, -0.407618F, -0.395339F, -0.395436F,
      -0.398243F, -0.398178518F, -0.399111F, -0.39767167F, -0.394411F,
      -0.398747F, -0.395033F, -0.399761F, -0.397552F, -0.39907F, -0.39692F,
      -0.401081F, -0.399636F, -0.39668F, -0.397043F, -0.399864F, -0.400286F,
      -0.401354F, -0.400392F, -0.403509F, -0.394332F, -0.394044F, -0.400368512F,
      -0.402414F, -0.401405513F, -0.401262F, -0.395297F, -0.396892F, -0.402848F,
      -0.395943F, -0.402684F, -0.399625F, -0.397378F, -0.397002F, -0.402334F,
      -0.402128F, -0.40334F, -0.397925F, -0.395178F, -0.403115F, -0.403329F,
      -0.401579F, -0.394942F, -0.400003F, -0.395651F, -0.403367F, -0.394985F,
      -0.400314F, -0.402072489F, -0.395035F, -0.401627F, -0.403757F,
      -0.394552499F, -0.401058F, -0.398453F, -0.401528478F, -0.389278F,
      -0.391668499F, -0.390002F, -0.386315F, -0.390673518F, -0.392458F,
      -0.386551F, -0.386994F, -0.392116487F, -0.387871504F, -0.391F, -0.385042F,
      -0.39249F, -0.386576F, -0.386955F, -0.38469F, -0.385101F, -0.384263F,
      -0.392024F, -0.385265F, -0.386986F, -0.387514F, -0.385123F, -0.389786F,
      -0.387369F, -0.389609516F, -0.384116F, -0.386465F, -0.389033F,
      -0.38964951F, -0.386708498F, -0.389782F, -0.387513518F, -0.388169F,
      -0.391873F, -0.392516F, -0.391357F, -0.387486F, -0.393194F, -0.389585674F,
      -0.39187F, -0.384781F, -0.386902F, -0.385765F, -0.388530016F, -0.389355F,
      -0.384842F, -0.390796F, -0.389732F, -0.377153F, -0.381972F, -0.38152F,
      -0.381598F, -0.374691F, -0.379726F, -0.377721488F, -0.379257F, -0.383422F,
      -0.377511F, -0.382798F, -0.381205F, -0.377993F, -0.379659F, -0.382713F,
      -0.3775765F, -0.383771F, -0.380501F, -0.374647F, -0.376376F, -0.380663514F,
      -0.381846F, -0.383524F, -0.377344F, -0.380482F, -0.377305F, -0.379198F,
      -0.37898466F, -0.378735F, -0.375653F, -0.376975F, -0.380674F, -0.378382F,
      -0.380166471F, -0.383457F, -0.378125F, -0.376838F, -0.381445F, -0.376346F,
      -0.383678F, -0.380494F, -0.37516F, -0.374602F, -0.382546F, -0.375077F,
      -0.364665F, -0.366402F, -0.366992F, -0.370802F, -0.364737F, -0.369910479F,
      -0.364346504F, -0.364261F, -0.37233F, -0.372142F, -0.368044F, -0.373625F,
      -0.364037F, -0.369702F, -0.36544F, -0.364321F, -0.371251F, -0.366963F,
      -0.36889F, -0.370586F, -0.369677F, -0.373052F, -0.366418F, -0.364264F,
      -0.364963F, -0.372457981F, -0.373589F, -0.368588F, -0.371056F, -0.37111F,
      -0.36648351F, -0.364084F, -0.368251F, -0.367991F, -0.366633F,
      -0.366698503F, -0.368681F, -0.364257F, -0.368021F, -0.367687F, -0.372264F,
      -0.369418502F, -0.368495F, -0.370289F, -0.372950017F, -0.367822F,
      -0.367202F, -0.369477F, -0.37367F, -0.357022F, -0.360900521F, -0.358098F,
      -0.358213F, -0.360891521F, -0.362578F, -0.354286F, -0.35875F, -0.362236F,
      -0.359825F, -0.362453F, -0.35412F, -0.361142F, -0.361081F, -0.363045F,
      -0.356904F, -0.360449016F, -0.361873F, -0.362585F, -0.356907F, -0.358245F,
      -0.359919F, -0.362686515F, -0.360956F, -0.357612F, -0.354678F,
      -0.358980507F, -0.362322658F, -0.359985F, -0.356978F, -0.357138F,
      -0.358287F, -0.359776F, -0.36366F, -0.357088F, -0.359565F, -0.359069F,
      -0.358801F, -0.355643F, -0.357264F, -0.358545F, -0.356039F, -0.35877F,
      -0.356481F, -0.358758F, -0.357728F, -0.356230497F, -0.362293F, -0.361973F,
      -0.363602F, -0.358612F, -0.359019518F, -0.361484F, -0.35711F,
      -0.360106498F, -0.358882F, -0.360722F, -0.349592F, -0.352998F, -0.353633F,
      -0.350539505F, -0.350988F, -0.348723F, -0.346182F, -0.344335F, -0.353713F,
      -0.346743F, -0.35024F, -0.348475F, -0.345913F, -0.348917246F,
      -0.347384512F, -0.344565F, -0.351472F, -0.349244F, -0.350882F, -0.345339F,
      -0.350078F, -0.346936F, -0.348908663F, -0.35322F, -0.349144F, -0.353575F,
      -0.344698F, -0.35396F, -0.35116F, -0.351741016F, -0.353302F, -0.351622F,
      -0.350676507F, -0.353857F, -0.346236F, -0.347127F, -0.346611F, -0.34767F,
      -0.34495F, -0.350784F, -0.351811F, -0.351905F, -0.35101F, -0.340519488F,
      -0.341959F, -0.336731F, -0.337356F, -0.335646F, -0.342622F, -0.342281F,
      -0.343312F, -0.342764F, -0.339702F, -0.339395523F, -0.339309F, -0.340238F,
      -0.334463F, -0.335291F, -0.337469518F, -0.339364F, -0.337173F, -0.343555F,
      -0.341985F, -0.341845F, -0.338168979F, -0.338174671F, -0.336956F,
      -0.337631F, -0.335696F, -0.339879F, -0.343534F, -0.34018F, -0.341343F,
      -0.341351509F, -0.34015F, -0.34133F, -0.34157F, -0.343199F, -0.339756F,
      -0.337579F, -0.335184F, -0.334641F, -0.334236F, -0.343547F, -0.341574F,
      -0.339030027F, -0.342096F, -0.33751449F, -0.337081671F, -0.340812F,
      -0.339391351F, -0.333858F, -0.32930851F, -0.324259F, -0.331524F,
      -0.331949F, -0.325049F, -0.325076F, -0.3313F, -0.332444F, -0.327015F,
      -0.328116506F, -0.329912484F, -0.330639F, -0.32955F, -0.332101F,
      -0.324564F, -0.325474F, -0.327309F, -0.332042F, -0.329613F, -0.327499F,
      -0.328071505F, -0.325697F, -0.330822F, -0.331387F, -0.324199F, -0.332932F,
      -0.325945F, -0.3275415F, -0.330825F, -0.328139F, -0.329351F, -0.329236F,
      -0.327892F, -0.330854F, -0.333082F, -0.329453498F, -0.325902F, -0.327532F,
      -0.329867F, -0.32406F, -0.324159F, -0.331808746F, -0.331950486F,
      -0.33148998F, -0.32820648F, -0.329582F, -0.332921F, -0.324427F, -0.326824F,
      -0.331421F, -0.329065979F, -0.326947F, -0.323558F, -0.319997489F,
      -0.322506F, -0.318995F, -0.315145F, -0.315901F, -0.323144F, -0.317262F,
      -0.317266F, -0.314542F, -0.323111F, -0.319528F, -0.316087F, -0.320275F,
      -0.314931F, -0.315288F, -0.319038F, -0.319525F, -0.322197F, -0.315896F,
      -0.316459F, -0.316862F, -0.32361F, -0.314935F, -0.322981F, -0.319253F,
      -0.316659F, -0.320861F, -0.315955F, -0.315746F, -0.319066F, -0.317461F,
      -0.314848512F, -0.315435F, -0.316627979F, -0.317943F, -0.323258F,
      -0.323454F, -0.319257F, -0.317104F, -0.320116F, -0.315544F, -0.319094F,
      -0.318489F, -0.315115F, -0.320077F, -0.315416F, -0.316017F, -0.315462F,
      -0.318321F, -0.31847F, -0.31545F, -0.314182F, -0.314038F, -0.32108F,
      -0.310679F, -0.307689F, -0.305963516F, -0.313622F, -0.311352F,
      -0.309797525F, -0.310688F, -0.309418F, -0.304731F, -0.30538F, -0.304132F,
      -0.312129F, -0.307969F, -0.310388F, -0.306149F, -0.311521F, -0.307264F,
      -0.311441F, -0.308582F, -0.309304F, -0.306484F, -0.309671F, -0.308893F,
      -0.312728F, -0.305158F, -0.309948981F, -0.305417F, -0.309284329F,
      -0.309777F, -0.307318F, -0.312377F, -0.313844F, -0.305031F, -0.306954F,
      -0.307832479F, -0.308585F, -0.308581F, -0.305329F, -0.310348511F,
      -0.309382021F, -0.31039F, -0.307594F, -0.307263017F, -0.31133F,
      -0.309117675F, -0.307094F, -0.305553F, -0.307768F, -0.308774F, -0.310552F,
      -0.313939F, -0.301428676F, -0.299146F, -0.299007F, -0.30222F,
      -0.298838496F, -0.295282F, -0.294964F, -0.296970487F, -0.29764F,
      -0.301713F, -0.295696F, -0.296922F, -0.300044F, -0.299012F, -0.302493F,
      -0.297312498F, -0.296684F, -0.295153F, -0.300146F, -0.300114F, -0.295175F,
      -0.302929F, -0.294551F, -0.298149F, -0.296290487F, -0.298761F, -0.298167F,
      -0.295183F, -0.296622F, -0.300276F, -0.296049F, -0.297607F, -0.296111494F,
      -0.302163F, -0.30112952F, -0.300762F, -0.302293F, -0.297720492F,
      -0.294518F, -0.297398F, -0.30313F, -0.296547F, -0.296656F, -0.294874F,
      -0.301472F, -0.2964F, -0.294279F, -0.299452335F, -0.300479F, -0.301767F,
      -0.294049F, -0.298879027F, -0.299851F, -0.286127985F, -0.290282F,
      -0.284954F, -0.286057F, -0.285402F, -0.287132F, -0.289129F, -0.28845948F,
      -0.286171F, -0.286897F, -0.284716F, -0.293248F, -0.285154F, -0.291302F,
      -0.292249F, -0.28897F, -0.289146F, -0.292533F, -0.29288F, -0.284971F,
      -0.292842F, -0.291804F, -0.288961F, -0.28664735F, -0.285445F, -0.288016F,
      -0.292707F, -0.292248F, -0.284146F, -0.289228F, -0.292396F, -0.291123F,
      -0.289049F, -0.286816F, -0.287058F, -0.290739F, -0.288371F, -0.28806F,
      -0.292075F, -0.291215F, -0.292076F, -0.293023F, -0.286627F, -0.285679F,
      -0.288659F, -0.279502F, -0.281518F, -0.274081F, -0.278831F, -0.274689F,
      -0.279048F, -0.282411F, -0.275675F, -0.278277516F, -0.27977F, -0.281979F,
      -0.283584F, -0.274855F, -0.277613F, -0.276986F, -0.277596503F, -0.280111F,
      -0.279102F, -0.278794F, -0.27809F, -0.280099F, -0.27918F, -0.280747F,
      -0.280088F, -0.274792492F, -0.282855F, -0.281997502F, -0.275503F,
      -0.276143F, -0.277408F, -0.277498F, -0.278641F, -0.275278F, -0.281869F,
      -0.279993F, -0.281445F, -0.279835F, -0.283215F, -0.275468F, -0.277539F,
      -0.2823385F, -0.279327F, -0.279587507F, -0.277170479F, -0.281059F,
      -0.283863F, -0.274293F, -0.281251F, -0.278887F, -0.28129F, -0.279753506F,
      -0.274157F, -0.269525F, -0.273781F, -0.264671F, -0.268747F, -0.269057F,
      -0.270649F, -0.272364F, -0.270015985F, -0.270144F, -0.272694F, -0.268054F,
      -0.267367F, -0.270715F, -0.27032F, -0.266222F, -0.268825F, -0.268404F,
      -0.273331F, -0.266659F, -0.269589514F, -0.264631F, -0.272148F, -0.268328F,
      -0.272381F, -0.264558F, -0.266899526F, -0.270548F, -0.267086F,
      -0.267967492F, -0.272435F, -0.266214F, -0.265267491F, -0.271954F,
      -0.269782F, -0.273872F, -0.268027F, -0.268921F, -0.26437F, -0.266960025F,
      -0.27009F, -0.272058F, -0.265899F, -0.271175F, -0.273083F, -0.271128476F,
      -0.26753667F, -0.265428F, -0.272424F, -0.272595F, -0.266971F,
      -0.270601511F, -0.271196F, -0.270057F, -0.26659F, -0.271805F, -0.266421F,
      -0.269277F, -0.262668F, -0.255857F, -0.260650247F, -0.26384F, -0.263247F,
      -0.258158982F, -0.25818F, -0.255401F, -0.258754F, -0.258046F, -0.255528F,
      -0.259961F, -0.260198F, -0.256174F, -0.263855F, -0.261741F, -0.258933F,
      -0.260211676F, -0.261111F, -0.25906F, -0.256038F, -0.257249F, -0.260877F,
      -0.254024F, -0.263884F, -0.256573F, -0.254646F, -0.25407F, -0.261453F,
      -0.259456F, -0.263629F, -0.256195F, -0.259203F, -0.263051F, -0.255635F,
      -0.262313F, -0.263488F, -0.258536F, -0.255701F, -0.263819F, -0.260169983F,
      -0.254445F, -0.26235F, -0.258423507F, -0.260641F, -0.257138F, -0.255944F,
      -0.255579F, -0.260267F, -0.252062F, -0.251122F, -0.244736F, -0.249507F,
      -0.245698988F, -0.248893663F, -0.247313499F, -0.250647F, -0.246727F,
      -0.245764F, -0.248042F, -0.252324F, -0.244716F, -0.253534F, -0.251707F,
      -0.247055F, -0.248048663F, -0.253807F, -0.245444F, -0.248466F, -0.252849F,
      -0.253746F, -0.251141F, -0.25065F, -0.252999F, -0.252494F, -0.248259F,
      -0.24635F, -0.247941F, -0.249183F, -0.253788F, -0.24604F, -0.248604F,
      -0.250629F, -0.246377498F, -0.246523F, -0.245244F, -0.249936327F,
      -0.248819F, -0.247502F, -0.250702024F, -0.244517505F, -0.253069F,
      -0.244873F, -0.250733346F, -0.246599495F, -0.253615F, -0.253371F,
      -0.244733F, -0.250396F, -0.252797F, -0.250071F, -0.251442492F, -0.252588F,
      -0.253826F, -0.249222F, -0.247289F, -0.250591666F, -0.250668F,
      -0.236298501F, -0.240055501F, -0.2421415F, -0.238997F, -0.242356F,
      -0.241487503F, -0.238944F, -0.23616F, -0.239191F, -0.242959F, -0.243403F,
      -0.235042F, -0.237759501F, -0.234481F, -0.237886F, -0.238653F, -0.243027F,
      -0.238838F, -0.243071F, -0.238126F, -0.240857F, -0.243917F, -0.242204F,
      -0.235999F, -0.243732F, -0.243728F, -0.234029F, -0.239152F, -0.237863F,
      -0.242891F, -0.234959F, -0.237036F, -0.235678F, -0.237531F, -0.242087F,
      -0.243443F, -0.239144F, -0.239801F, -0.243617F, -0.242621F, -0.235246F,
      -0.234814F, -0.240043F, -0.239309F, -0.235318F, -0.230781F, -0.228587F,
      -0.228042334F, -0.226904511F, -0.226587F, -0.229072511F, -0.225691F,
      -0.22784F, -0.228675F, -0.226528F, -0.2319F, -0.233368F, -0.227608502F,
      -0.233581F, -0.22685F, -0.231993F, -0.225455F, -0.224618F, -0.225717008F,
      -0.23395F, -0.228105F, -0.227382F, -0.228229F, -0.22876F, -0.231433496F,
      -0.228919F, -0.232368F, -0.231613F, -0.225197F, -0.2303F, -0.22797F,
      -0.22566F, -0.225775331F, -0.233703F, -0.228821F, -0.224964F, -0.231306F,
      -0.231817663F, -0.224698F, -0.2297F, -0.229089499F, -0.228098F,
      -0.229472503F, -0.226474F, -0.230283F, -0.22638F, -0.227935F, -0.227956F,
      -0.225658F, -0.23121F, -0.229514F, -0.227212F, -0.224964F, -0.232994F,
      -0.224708F, -0.228868663F, -0.22874F, -0.232877F, -0.215902F,
      -0.219894499F, -0.220663F, -0.22319F, -0.222777F, -0.216843F, -0.21962F,
      -0.216427F, -0.218076497F, -0.219245672F, -0.214152F, -0.21638F,
      -0.2229525F, -0.221502498F, -0.218149F, -0.223851F, -0.221064F, -0.21475F,
      -0.214743F, -0.219797492F, -0.215938F, -0.221927F, -0.214715F, -0.216982F,
      -0.218366F, -0.223012F, -0.220721F, -0.222487F, -0.215555F, -0.220829F,
      -0.222218F, -0.219690502F, -0.217317F, -0.216233F, -0.216347F, -0.215377F,
      -0.215148F, -0.215922F, -0.219603F, -0.22393F, -0.219313F, -0.217448F,
      -0.219364F, -0.216324F, -0.217391F, -0.218789503F, -0.221857F,
      -0.217821673F, -0.223565F, -0.222129F, -0.214293F, -0.217956F, -0.21462F,
      -0.220897F, -0.22386F, -0.220922F, -0.216697F, -0.220266F, -0.2052515F,
      -0.20483F, -0.211762F, -0.20506F, -0.209114F, -0.211275F, -0.210807502F,
      -0.209708F, -0.20965F, -0.213677F, -0.20924F, -0.207482F, -0.206440508F,
      -0.204782F, -0.20628F, -0.204915F, -0.209580988F, -0.2096F, -0.213619F,
      -0.206891F, -0.212132499F, -0.210165F, -0.211823F, -0.212381F, -0.21218F,
      -0.208341F, -0.206193F, -0.206084F, -0.210238F, -0.212338F, -0.210507F,
      -0.207011327F, -0.210868F, -0.204877F, -0.213101F, -0.211085F, -0.210764F,
      -0.211927F, -0.211468F, -0.210742F, -0.213838F, -0.211112F, -0.211107F,
      -0.211945F, -0.205276F, -0.207187F, -0.210291F, -0.206547F, -0.205608F,
      -0.209443F, -0.211421F, -0.201503336F, -0.197808504F, -0.194822F,
      -0.195119F, -0.196847F, -0.197645F, -0.199256F, -0.195597F, -0.19503F,
      -0.201972F, -0.194842F, -0.199469F, -0.194595F, -0.197207F, -0.194038F,
      -0.194785F, -0.197069F, -0.196345F, -0.197336F, -0.19456F, -0.196026F,
      -0.198478F, -0.20223F, -0.202787F, -0.196272F, -0.202245F, -0.196519F,
      -0.201883F, -0.197119F, -0.198099F, -0.20319F, -0.203757F, -0.196691F,
      -0.203137F, -0.202558F, -0.200489F, -0.202175F, -0.20223251F, -0.20242F,
      -0.196933F, -0.197483F, -0.198491F, -0.19631049F, -0.1987845F, -0.195597F,
      -0.203098506F, -0.199074F, -0.198182F, -0.194855F, -0.199276F, -0.20101F,
      -0.199471F, -0.198274493F, -0.191238493F, -0.1892F, -0.188949F, -0.191174F,
      -0.19192F, -0.187275F, -0.189583F, -0.186488F, -0.190388F, -0.191045F,
      -0.189077F, -0.188471F, -0.19012F, -0.187135F, -0.188813F, -0.193773F,
      -0.188959F, -0.185551F, -0.191456F, -0.187274F, -0.189534F, -0.184937F,
      -0.193626F, -0.192418F, -0.19209F, -0.191907F, -0.187056F, -0.1893F,
      -0.190135F, -0.188404F, -0.189332008F, -0.187019F, -0.188935F,
      -0.191075504F, -0.184518F, -0.184351F, -0.192123F, -0.186397F, -0.193318F,
      -0.187939F, -0.185512F, -0.191503F, -0.188752F, -0.1859615F, -0.192888F,
      -0.193907F, -0.18791F, -0.186438F, -0.18757F, -0.187567F, -0.193677F,
      -0.186755329F, -0.191188F, -0.187162F, -0.179766F, -0.176143F, -0.182402F,
      -0.181041F, -0.179072F, -0.183887F, -0.180594F, -0.183707F, -0.174515F,
      -0.175891F, -0.183035F, -0.178595F, -0.179814F, -0.174839F, -0.1804865F,
      -0.176735F, -0.176556F, -0.177087F, -0.181812F, -0.183077F, -0.180181F,
      -0.17461F, -0.178881496F, -0.1762F, -0.17417F, -0.175844F, -0.174262F,
      -0.17432F, -0.177964F, -0.181125F, -0.179711F, -0.174897F, -0.182875F,
      -0.181115F, -0.17937F, -0.175839F, -0.182716F, -0.176821F, -0.180855F,
      -0.177509F, -0.179113F, -0.182298F, -0.183009F, -0.18232F, -0.180591F,
      -0.18383351F, -0.17772F, -0.179650486F, -0.177538F, -0.164232F, -0.171906F,
      -0.164552F, -0.165193F, -0.168345F, -0.168467F, -0.164685F, -0.173915F,
      -0.166268F, -0.168783009F, -0.166511F, -0.172611F, -0.172264F, -0.166726F,
      -0.164487F, -0.166441F, -0.168919F, -0.168068F, -0.169003F, -0.166454F,
      -0.164841F, -0.166294F, -0.170153F, -0.168866F, -0.164721F, -0.16762F,
      -0.168433F, -0.17349F, -0.171013F, -0.173414F, -0.168207F, -0.167431F,
      -0.164955F, -0.167249501F, -0.168312F, -0.165528F, -0.172756F, -0.169275F,
      -0.169894F, -0.165752F, -0.164064F, -0.166274F, -0.169526F, -0.15749F,
      -0.161225F, -0.159416F, -0.160337F, -0.158101F, -0.156268F, -0.159576F,
      -0.158479F, -0.161688F, -0.158009F, -0.157242F, -0.154442F, -0.159242F,
      -0.160202F, -0.156545F, -0.161621F, -0.154715F, -0.155884F, -0.158429F,
      -0.158471F, -0.158944F, -0.160562F, -0.161844F, -0.162024F, -0.155739F,
      -0.158904672F, -0.158535F, -0.156485F, -0.160816491F, -0.160813F,
      -0.15485F, -0.159883499F, -0.159926F, -0.162489F, -0.158115F, -0.158957F,
      -0.156901F, -0.163064F, -0.155204F, -0.159056F, -0.157424F, -0.160413F,
      -0.161356F, -0.157158F, -0.160769F, -0.157114F, -0.161117F, -0.156818F,
      -0.161404F, -0.159461F, -0.159887F, -0.157548502F, -0.159909F, -0.160044F,
      -0.148069337F, -0.151983F, -0.145564F, -0.148148507F, -0.153668F,
      -0.151637F, -0.152679F, -0.1504675F, -0.14835F, -0.14932F, -0.152302F,
      -0.146883F, -0.145635501F, -0.145645F, -0.144663F, -0.148295F, -0.149527F,
      -0.153109F, -0.150928F, -0.150541F, -0.147792F, -0.145732F, -0.146339F,
      -0.144405991F, -0.152301F, -0.149745F, -0.151888F, -0.150891F, -0.147853F,
      -0.147022486F, -0.145853F, -0.146067F, -0.146713F, -0.146781F,
      -0.151507333F, -0.15366F, -0.151139F, -0.14524F, -0.14755F, -0.145862F,
      -0.151626F, -0.151483F, -0.147839F, -0.146543F, -0.147926494F,
      -0.149935663F, -0.144066F, -0.147588F, -0.153125F, -0.150459F, -0.143331F,
      -0.139228F, -0.141372F, -0.140615F, -0.143789F, -0.138327509F, -0.138815F,
      -0.134756F, -0.134806F, -0.134889F, -0.138882F, -0.137688F, -0.141156F,
      -0.140661F, -0.13981F, -0.136959F, -0.142104F, -0.142196F, -0.141888F,
      -0.135445F, -0.137984499F, -0.137452F, -0.14088F, -0.136255F,
      -0.141975492F, -0.141201F, -0.138604F, -0.13751F, -0.137931F, -0.135076F,
      -0.143405F, -0.142487F, -0.134706F, -0.134278F, -0.139128F, -0.139098F,
      -0.134749F, -0.141439F, -0.137369F, -0.138512F, -0.140428F, -0.143775F,
      -0.1374F, -0.135542F, -0.140897F, -0.132665F, -0.12963F, -0.129694328F,
      -0.126420498F, -0.12738651F, -0.126136F, -0.130582F, -0.129643F,
      -0.131617F, -0.124069F, -0.12658599F, -0.125055F, -0.12855F, -0.124076F,
      -0.127794504F, -0.131142F, -0.132062F, -0.12412F, -0.125776F, -0.125393F,
      -0.128565F, -0.12799F, -0.125872F, -0.129167F, -0.131096244F, -0.133488F,
      -0.126662F, -0.127345F, -0.125523493F, -0.129866F, -0.129073F, -0.133152F,
      -0.124097F, -0.127856F, -0.124817F, -0.131471F, -0.129149F, -0.131051F,
      -0.126252F, -0.126662F, -0.127936F, -0.128130496F, -0.132619F, -0.128612F,
      -0.126381F, -0.133923F, -0.132061F, -0.131605F, -0.132749F, -0.128850505F,
      -0.133109F, -0.132106513F, -0.120525666F, -0.123981F, -0.1185F, -0.117967F,
      -0.11865F, -0.116488F, -0.116018F, -0.120894F, -0.12166F, -0.123336F,
      -0.120083F, -0.116878F, -0.114468F, -0.11876F, -0.120595F, -0.118599F,
      -0.122525F, -0.119533F, -0.118310004F, -0.11675F, -0.115987F,
      -0.119748503F, -0.118716F, -0.117148F, -0.116693F, -0.123688F, -0.117004F,
      -0.114568F, -0.118614F, -0.116408F, -0.122087501F, -0.117194F, -0.121136F,
      -0.121999F, -0.116523F, -0.121517F, -0.117711F, -0.119765F, -0.117334F,
      -0.119546F, -0.118046F, -0.116441F, -0.118839F, -0.115544F, -0.123204F,
      -0.114084F, -0.120434F, -0.116291F, -0.118468F, -0.121137F, -0.116693F,
      -0.122797F, -0.119823F, -0.119817495F, -0.11856F, -0.122196F, -0.106414F,
      -0.108246505F, -0.106232F, -0.112313F, -0.108782F, -0.112877F, -0.110533F,
      -0.108317502F, -0.112514F, -0.113104F, -0.105514F, -0.110987F, -0.106704F,
      -0.104604F, -0.10752F, -0.113792F, -0.111555F, -0.113785F, -0.104118F,
      -0.107202F, -0.111618F, -0.111048F, -0.108997F, -0.109268501F,
      -0.110410005F, -0.110045F, -0.106445F, -0.107656F, -0.111374F, -0.111303F,
      -0.108064F, -0.1048F, -0.111778F, -0.106734F, -0.108645F, -0.107586503F,
      -0.105198F, -0.110419F, -0.107257F, -0.112397F, -0.11232F, -0.106276669F,
      -0.100184F, -0.101616F, -0.100226F, -0.0978137478F, -0.0969729F,
      -0.103961F, -0.101543F, -0.0973976F, -0.101437F, -0.0992427F, -0.0953891F,
      -0.102414F, -0.100436598F, -0.0987586F, -0.0947704F, -0.0981283F,
      -0.101163F, -0.0941469F, -0.0952793F, -0.103048F, -0.103135F, -0.0994048F,
      -0.098596F, -0.102179F, -0.0985073F, -0.101472005F, -0.100738F, -0.103761F,
      -0.101395398F, -0.0962079F, -0.097558F, -0.099726F, -0.0979839F,
      -0.100851F, -0.0999899F, -0.0958433F, -0.102925F, -0.102003F, -0.0956213F,
      -0.097651F, -0.0943068F, -0.0963627F, -0.0940959F, -0.0978695F,
      -0.0992661F, -0.103007F, -0.101595F, -0.101774F, -0.0994048F,
      -0.0891072527F, -0.0910684F, -0.093252264F, -0.0903246328F, -0.089494F,
      -0.0899401F, -0.090041F, -0.0861800462F, -0.0915459F, -0.0906946F,
      -0.0935503F, -0.0869105F, -0.0848811F, -0.0872676F, -0.0880546495F,
      -0.0902775F, -0.0877323F, -0.0925009F, -0.0929925F, -0.0906018F,
      -0.0874913F, -0.0921555F, -0.0885407F, -0.0897679F, -0.0887151286F,
      -0.0890841484F, -0.0905459F, -0.0882482529F, -0.0913827F, -0.0884751F,
      -0.0840423F, -0.0880304F, -0.0891247F, -0.0855448F, -0.0907686502F,
      -0.0876372F, -0.085912548F, -0.0884051F, -0.090835996F, -0.0920715F,
      -0.0882935F, -0.0910635F, -0.0791383F, -0.0819288F, -0.081678F,
      -0.0822299F, -0.0807186F, -0.0791071951F, -0.0789382F, -0.0756869F,
      -0.0791458F, -0.0753629F, -0.0820515F, -0.0789227039F, -0.0775769502F,
      -0.0800517F, -0.0786163F, -0.0817039F, -0.0834154F, -0.0798286F,
      -0.0825986F, -0.0768779F, -0.0798201F, -0.0745742F, -0.0816734F,
      -0.0814057F, -0.0795724F, -0.0740973F, -0.0823822F, -0.0788676F,
      -0.0744322F, -0.0746337F, -0.0746789F, -0.0751111F, -0.0789044F, -0.08374F,
      -0.0746199F, -0.07822F, -0.0786090046F, -0.0800371F, -0.082198F,
      -0.0745496F, -0.0817072F, -0.075609F, -0.0774472F, -0.0835556F,
      -0.0741336F, -0.0792119503F, -0.07708F, -0.0786825F, -0.0836374F,
      -0.0803904F, -0.0718313F, -0.0681268F, -0.0661191F, -0.0706347F,
      -0.0683581F, -0.0698541F, -0.069881F, -0.0663F, -0.0672262F, -0.0671688F,
      -0.0672723055F, -0.0723879486F, -0.0663445517F, -0.0702628046F, -0.071876F,
      -0.0715018064F, -0.0702925F, -0.0730716F, -0.0666155F, -0.0677296F,
      -0.0642697F, -0.0654547F, -0.0697855F, -0.0671719F, -0.0704927519F,
      -0.069411F, -0.0730049F, -0.0689284652F, -0.0689727068F, -0.0661482F,
      -0.0688578F, -0.0704374F, -0.0689719F, -0.069896F, -0.0675495F,
      -0.0669727F, -0.0711455F, -0.0690028F, -0.0692120492F, -0.0723891F,
      -0.0706347F, -0.0699695F, -0.0687728F, -0.0650385F, -0.0685946F,
      -0.0724102F, -0.0690985471F, -0.0676141F, -0.0700949F, -0.0709837F,
      -0.0679623038F, -0.0685167F, -0.0663033F, -0.0723664F, -0.0647275F,
      -0.0701665357F, -0.0694398507F, -0.0572514F, -0.0602628F, -0.0569257F,
      -0.0580173F, -0.0608944F, -0.063471F, -0.0595827F, -0.0572346F,
      -0.0588197F, -0.062099F, -0.0554541F, -0.0600791F, -0.0581167F,
      -0.0583193F, -0.0548311F, -0.0634226F, -0.059866704F, -0.0571039F,
      -0.0634354F, -0.0571893F, -0.0572897F, -0.0586726F, -0.0568651F,
      -0.0621849F, -0.0626702458F, -0.0591749F, -0.0580226F, -0.0606886F,
      -0.0565414F, -0.0579905F, -0.0612398F, -0.0637456F, -0.0585723F,
      -0.0568672F, -0.05623F, -0.0616532F, -0.0585744F, -0.0560607F, -0.0584543F,
      -0.0597118512F, -0.0637911F, -0.05782F, -0.0560460463F, -0.0589635968F,
      -0.0562527F, -0.0610631F, -0.0478112474F, -0.0450405F, -0.0527514F,
      -0.0449592F, -0.0479546F, -0.0472337976F, -0.0473139F, -0.0470204F,
      -0.0463105F, -0.0483974F, -0.048534F, -0.0456196517F, -0.0451638F,
      -0.0530837F, -0.0464078F, -0.045045F, -0.0522374F, -0.0457332F,
      -0.0494741F, -0.0497369F, -0.0503132F, -0.0527194F, -0.0493225F,
      -0.0476438F, -0.0483595F, -0.0534309F, -0.0448796339F, -0.0472858F,
      -0.0510807F, -0.050799F, -0.0515249F, -0.0453436F, -0.0457582F,
      -0.0488018F, -0.0474630333F, -0.0457256F, -0.0462158F, -0.0509855F,
      -0.0458592F, -0.0469736F, -0.0519753024F, -0.0452455F, -0.0488061346F,
      -0.0512363F, -0.04851F, -0.047745198F, -0.0469695F, -0.0489224F,
      -0.0530496F, -0.0478878F, -0.0421258F, -0.0375432F, -0.0409739F,
      -0.0363694F, -0.0436959F, -0.0342457F, -0.0369304F, -0.0395302F,
      -0.0411651F, -0.0355119F, -0.040089F, -0.0366004F, -0.042926F, -0.0397912F,
      -0.0388502F, -0.0363032F, -0.0367391F, -0.0395232F, -0.0411975F,
      -0.0344526F, -0.0375773F, -0.0379787497F, -0.0381508F, -0.0405071F,
      -0.0377119F, -0.0381856F, -0.0434363F, -0.041621F, -0.0403612F, -0.034228F,
      -0.0378484F, -0.0385441482F, -0.0397014469F, -0.0377031F, -0.0364637F,
      -0.0405132473F, -0.038261F, -0.0348611F, -0.0374235F, -0.0351475F,
      -0.0392666F, -0.0422629F, -0.0350609F, -0.0413707F, -0.036232F,
      -0.0400787F, -0.043119F, -0.0436566F, -0.0396878F, -0.0373344F,
      -0.0348116F, -0.0373689F, -0.0419048F, -0.0346429F, -0.0356069F,
      -0.0405128673F, -0.0248292F, -0.0317602F, -0.0288343F, -0.0318764F,
      -0.0245277F, -0.0320949F, -0.0281076F, -0.0272404F, -0.0322724F,
      -0.0334685F, -0.0268216F, -0.0284379F, -0.0333027F, -0.0304852F,
      -0.028982F, -0.0255291F, -0.0312454F, -0.025951F, -0.0242491F, -0.030545F,
      -0.0277465675F, -0.0326294F, -0.0284482F, -0.0271520987F, -0.0302826F,
      -0.0293686F, -0.0296019F, -0.0245061F, -0.0278986F, -0.0305568986F,
      -0.029084F, -0.0278419F, -0.0312542F, -0.0264622F, -0.0270082F,
      -0.0334924F, -0.0240822F, -0.0250976F, -0.0278832F, -0.0286845F,
      -0.0258419F, -0.0271864F, -0.031109F, -0.0325021F, -0.028142F, -0.0287728F,
      -0.024119F, -0.0331559F, -0.0308657F, -0.0333328F, -0.0250043012F,
      -0.0314811F, -0.0257164501F, -0.028707F, -0.017683F, -0.0186222009F,
      -0.0157128498F, -0.0170708F, -0.0205176F, -0.0218416F, -0.0229929F,
      -0.0149853F, -0.0172018502F, -0.0217032656F, -0.0157419F, -0.0190098F,
      -0.0156108F, -0.0216668F, -0.0141537F, -0.0150488F, -0.0220562F,
      -0.0237978F, -0.0223069F, -0.0161171F, -0.0216117F, -0.0238917F,
      -0.0200546F, -0.0185686015F, -0.0183385F, -0.0169807F, -0.0172424F,
      -0.01603F, -0.0164893493F, -0.0231523F, -0.0209082F, -0.016038F,
      -0.0229428F, -0.0221204F, -0.016595F, -0.0231842F, -0.0237644F,
      -0.0177938F, -0.0194544F, -0.0178962F, -0.0217806F, -0.0184319336F,
      -0.0216619F, -0.0161008F, -0.0175749F, -0.016192F, -0.0193731F,
      -0.0169031F, -0.0188443F, -0.0177522F, -0.0205748491F, -0.0194207F,
      -0.00626975F, -0.00874035F, -0.0139802F, -0.0105918F, -0.0130178F,
      -0.00699797F, -0.00966020487F, -0.00679356F, -0.00992811F, -0.00476652F,
      -0.0120642502F, -0.00917375F, -0.00552362F, -0.00794029F, -0.0108547F,
      -0.00620663F, -0.0102127399F, -0.0090652F, -0.0064187F, -0.0132659F,
      -0.00808924F, -0.00420326F, -0.00612915F, -0.0123218F, -0.00968444F,
      -0.0117431F, -0.00751656F, -0.0116001F, -0.0138889F, -0.01183F,
      -0.00936884F, -0.0104473F, -0.00573826F, -0.0129576F, -0.0126406F,
      -0.012969F, -0.00922292F, -0.0119882F, -0.00957406F, -0.0136313F,
      -0.00748622F, -0.0122466F, -0.00810581F, -0.00907755F, -0.00620061F,
      -0.00895321F, -0.00438708F, -0.00554192F, -0.0122437F, 0.000712216F,
      0.00022584002F, -0.000581801F, 0.00390595F, 0.00225335F, 0.000110567F,
      0.00282568F, 0.00509661F, 0.00357175F, 0.00281543F, 0.00116551F,
      -0.00346822F, 0.000319779F, -0.00187358458F, 0.00345284F, 0.00394034F,
      -0.00201356F, 0.00122219F, 0.00212490791F, 0.00364786F, -0.00364852F,
      -0.00380486F, -0.000448585F, 0.00402796F, 0.00513124F, 0.00308564F,
      0.00174132013F, 0.00311953F, -0.0025714F, -0.00245434F, 0.00200409F,
      -0.00276548F, 0.00255287F, 0.00448751F, -0.000525951F, -0.00094074F,
      0.00507712F, 0.000344396F, 0.00417048F, -0.00369537F, -0.00119698F,
      0.00121379F, 0.00272983F, 0.00248116F, 0.00234437F, 0.00464952F,
      -0.00112873F, 0.00050199F, -0.0011403F, -0.00400001F, 0.00350383F,
      -0.000649393F, 0.00110263F, 0.0053584F, 0.0156154F, 0.006396F, 0.00626117F,
      0.0131694F, 0.0133207F, 0.0134809F, 0.0094502F, 0.00737417F, 0.0108397F,
      0.0115576F, 0.00775033F, 0.0155221F, 0.0124825F, 0.0110155549F,
      0.00628972F, 0.0123675F, 0.0150526F, 0.0102028F, 0.00863874F, 0.0133454F,
      0.00723341F, 0.0148217F, 0.0124097F, 0.0141609F, 0.0138385501F,
      0.0102676731F, 0.0125003F, 0.00993007F, 0.0113881957F, 0.00914134458F,
      0.0133736F, 0.0145776F, 0.00833535F, 0.00683558F, 0.0116442F, 0.0125185F,
      0.0089677F, 0.0157648F, 0.0118466F, 0.00959476F, 0.0110219F, 0.00876945F,
      0.00719398F, 0.00731063F, 0.0152804498F, 0.00662863F, 0.0151775498F,
      0.0156796F, 0.00819135F, 0.0112657F, 0.0100684F, 0.00881392F,
      0.0106047895F, 0.0201452F, 0.0207259F, 0.0229395498F, 0.0224816985F,
      0.0186685F, 0.0192171F, 0.0176905F, 0.0235985F, 0.0256039F, 0.0233611669F,
      0.0251952F, 0.019471F, 0.020303F, 0.0200092F, 0.021143F, 0.017428F,
      0.0204247F, 0.0178058501F, 0.0251284F, 0.0231231F, 0.017674F, 0.0173442F,
      0.0242243F, 0.023899F, 0.022148F, 0.0194737F, 0.0257927F, 0.0204519F,
      0.0198898F, 0.0217542F, 0.0173537F, 0.0168381F, 0.0173608F, 0.0256737F,
      0.0177883F, 0.0210351F, 0.02183F, 0.0187315661F, 0.0212062988F, 0.0213409F,
      0.0231175F, 0.0253539F, 0.0257611F, 0.0229717F, 0.0227094665F, 0.0237105F,
      0.021639F, 0.0305945668F, 0.0325239F, 0.033241F, 0.0330942F, 0.0273998F,
      0.0337201F, 0.0327122F, 0.0316246F, 0.0348088F, 0.0358346F, 0.0320231654F,
      0.035332F, 0.0264481F, 0.0305299F, 0.0296132F, 0.0281296F, 0.0294269F,
      0.0278399F, 0.0302448496F, 0.0316759F, 0.0330902F, 0.0288696F, 0.0292741F,
      0.0279337F, 0.0279412F, 0.0323269F, 0.0350152F, 0.02789F, 0.026414F,
      0.0359044F, 0.0310094F, 0.0297859497F, 0.0351716504F, 0.0302036516F,
      0.0296636503F, 0.0297012F, 0.0271656513F, 0.0359454F, 0.0300902F,
      0.0350974F, 0.0383878F, 0.0425592F, 0.0419507F, 0.0427134484F, 0.0398017F,
      0.0452748F, 0.0413488F, 0.0432172F, 0.0362638F, 0.0388491F, 0.0436096F,
      0.0423394F, 0.0441567F, 0.0445524529F, 0.0417482F, 0.0384341F, 0.0377074F,
      0.0445925F, 0.0436688F, 0.041158F, 0.0402659F, 0.0454975F, 0.0415565F,
      0.043187F, 0.0368342F, 0.0382181F, 0.0418451F, 0.0394363F, 0.0431386F,
      0.044798F, 0.0437137485F, 0.044245F, 0.0379931F, 0.0443383F, 0.0425089F,
      0.0456303F, 0.0377592F, 0.045452252F, 0.0369013F, 0.0401936F, 0.0418195F,
      0.0385224F, 0.0444691F, 0.0415736F, 0.041988F, 0.0394444F, 0.041479F,
      0.0396702F, 0.0392001F, 0.0490534492F, 0.0494661F, 0.0530524F, 0.0493979F,
      0.0508672521F, 0.0521564F, 0.0487943F, 0.0479826F, 0.055575F,
      0.0488322489F, 0.0516841F, 0.0488836F, 0.053833F, 0.0472694F, 0.0554109F,
      0.0467966F, 0.0502363F, 0.0490721F, 0.0499696508F, 0.054018F, 0.0473059F,
      0.052588F, 0.0471259F, 0.0465176F, 0.0502915494F, 0.0498297F, 0.0541865F,
      0.0549993F, 0.0483218F, 0.0464877F, 0.0536613502F, 0.0547701F,
      0.0495619476F, 0.0528662F, 0.0551746F, 0.0488814488F, 0.0528134F,
      0.0542023F, 0.0496219F, 0.0532508F, 0.0490775F, 0.0502828F, 0.0555692F,
      0.0479527488F, 0.046227F, 0.0461434F, 0.054356F, 0.0508467332F, 0.0511229F,
      0.0501152F, 0.0552211F, 0.0463029F, 0.0523621328F, 0.0528049F, 0.0530869F,
      0.0527172498F, 0.0463685F, 0.049913F, 0.0471693F, 0.046056F, 0.0469312519F,
      0.0510172322F, 0.0498575494F, 0.0482551F, 0.0479509F, 0.0617522F,
      0.0609805F, 0.0637879F, 0.0603541F, 0.0603942F, 0.0635078F, 0.0604502F,
      0.0594953F, 0.0650927F, 0.0607023F, 0.0597844F, 0.0572636F, 0.0609767F,
      0.0645497F, 0.0574504F, 0.0600162521F, 0.0634611F, 0.0562879F, 0.0619612F,
      0.0581505F, 0.0621755F, 0.0585248F, 0.0621571466F, 0.064375F,
      0.0611114502F, 0.0627894F, 0.063025F, 0.0598319F, 0.06322F, 0.0653968F,
      0.060388F, 0.0600233972F, 0.064214F, 0.0615743F, 0.0632222F, 0.0621197F,
      0.0592926F, 0.0578092F, 0.0586128F, 0.05676F, 0.0631145537F, 0.0593633F,
      0.0615580529F, 0.0564786F, 0.0615445F, 0.0684304535F, 0.0660513F,
      0.0722236F, 0.07151F, 0.0698489F, 0.0724258F, 0.0682887F, 0.069703F,
      0.0681735F, 0.0740575F, 0.0745488F, 0.0679049F, 0.0661584F, 0.075658F,
      0.0715436F, 0.0716883F, 0.0692139F, 0.0720937F, 0.0706264F, 0.0690296F,
      0.0665698F, 0.0739024F, 0.067841F, 0.075104F, 0.0758665F, 0.0697012544F,
      0.0693028F, 0.0703456F, 0.067628F, 0.0723637F, 0.0737194F, 0.0748551041F,
      0.0742912F, 0.0721012F, 0.0684529F, 0.0735683F, 0.0698924959F, 0.0753369F,
      0.0749498F, 0.0692101F, 0.0687111F, 0.072134F, 0.0716693F, 0.0747694F,
      0.0694626F, 0.0703813F, 0.0660053F, 0.0827106F, 0.076819F, 0.0835997F,
      0.0835075F, 0.0845349F, 0.0787638F, 0.0834127F, 0.0855574F, 0.0796434F,
      0.0777979F, 0.0781116F, 0.0833883F, 0.0847052F, 0.0812418F, 0.0807785541F,
      0.080561F, 0.0847360045F, 0.0782566F, 0.0781424046F, 0.0798464F,
      0.0792319477F, 0.0826492F, 0.0785509F, 0.0858488F, 0.0782399F, 0.077376F,
      0.0771729F, 0.0772929F, 0.0820793957F, 0.0797696337F, 0.0820212F,
      0.0854245F, 0.080003947F, 0.0774958F, 0.0790845454F, 0.0827532F,
      0.0776975F, 0.0824417F, 0.0835345F, 0.0778422F, 0.0809937F, 0.0765697F,
      0.0772861F, 0.0801848F, 0.0833549F, 0.083534047F, 0.0802217F, 0.0772932F,
      0.0792247F, 0.0765405F, 0.081856F, 0.0845461F, 0.0823575F, 0.0817093477F,
      0.0798993F, 0.0824746F, 0.083483F, 0.0920339301F, 0.0907853F, 0.0952579F,
      0.0951692F, 0.0928183049F, 0.0860003F, 0.0919787F, 0.0909794F, 0.0949616F,
      0.0930226F, 0.0942389F, 0.0922476F, 0.0923049F, 0.0893078F, 0.0897397F,
      0.0911003053F, 0.090042F, 0.0948027F, 0.0914156437F, 0.0915301F,
      0.0915622F, 0.0879121F, 0.091633F, 0.0862896F, 0.0875711F, 0.092662F,
      0.0958542F, 0.0908218F, 0.0861078F, 0.0927154F, 0.0884276F, 0.0919634F,
      0.0954856F, 0.0867412F, 0.0918089F, 0.0913135F, 0.0890273F, 0.0932793F,
      0.088124F, 0.0900674F, 0.0876607F, 0.0888718069F, 0.0896819F,
      0.0920960456F, 0.089457646F, 0.0993447527F, 0.0966812521F, 0.1016206F,
      0.100443F, 0.103985F, 0.0978219F, 0.104501F, 0.0995353F, 0.103815F,
      0.100366F, 0.100949F, 0.098838F, 0.104588F, 0.100082703F, 0.0997364F,
      0.104317F, 0.102366F, 0.105405F, 0.096642F, 0.0988213F, 0.0974975F,
      0.10399F, 0.101101503F, 0.0968531519F, 0.104439F, 0.104207F, 0.104453F,
      0.100588F, 0.0962244F, 0.101119697F, 0.104238F, 0.102599F, 0.0991153F,
      0.101318F, 0.0966137F, 0.101038F, 0.104325995F, 0.098766F, 0.105836F,
      0.101496F, 0.105953F, 0.0982825F, 0.102117F, 0.101765633F, 0.0971264467F,
      0.0988862F, 0.10098F, 0.103065F, 0.101161502F, 0.100816F, 0.104031F,
      0.104642004F, 0.102355F, 0.101746F, 0.100672F, 0.103087F, 0.115053F,
      0.110421F, 0.110286333F, 0.11297F, 0.108882F, 0.11368F, 0.114558F,
      0.108836F, 0.109691F, 0.107264F, 0.109965667F, 0.111957F, 0.107875F,
      0.109908F, 0.109095F, 0.108079F, 0.113955F, 0.1135F, 0.112837F, 0.109756F,
      0.114581F, 0.114289F, 0.110918F, 0.106557995F, 0.108753F, 0.109891F,
      0.115965F, 0.106995F, 0.113455F, 0.106278F, 0.113211006F, 0.108978F,
      0.107351F, 0.108198F, 0.111567497F, 0.108954F, 0.108683497F, 0.109937F,
      0.111899F, 0.109779499F, 0.109542F, 0.109315F, 0.112615F, 0.111695F,
      0.108964F, 0.113674F, 0.110859F, 0.115715F, 0.111752F, 0.108627F,
      0.107659F, 0.114813F, 0.107013F, 0.108908F, 0.106412F, 0.11264433F,
      0.111344F, 0.122396F, 0.123143502F, 0.121972501F, 0.120245337F, 0.11827F,
      0.1214F, 0.122727334F, 0.121473F, 0.119422F, 0.117504F, 0.123336494F,
      0.122477F, 0.125258F, 0.117889F, 0.124954F, 0.122519F, 0.125066F,
      0.120392F, 0.120368F, 0.123852F, 0.121194F, 0.123362F, 0.119887747F,
      0.120582F, 0.125566F, 0.119669F, 0.12362F, 0.121793F, 0.116613F, 0.117114F,
      0.117326F, 0.120607F, 0.119320005F, 0.117631F, 0.11915F, 0.11895F,
      0.123507F, 0.123448F, 0.124398F, 0.123639F, 0.12304F, 0.125335F,
      0.121144503F, 0.116159F, 0.125015F, 0.125214F, 0.12123F, 0.121753F,
      0.120838F, 0.12268F, 0.125765F, 0.121658F, 0.13530001F, 0.126518F,
      0.129605F, 0.135445F, 0.132863F, 0.132113F, 0.126972F, 0.132042F,
      0.131014496F, 0.127895F, 0.132928F, 0.135408F, 0.127619F, 0.133893F,
      0.130587503F, 0.132745F, 0.130039F, 0.129473F, 0.13489F, 0.133576F,
      0.128957F, 0.130362F, 0.130865008F, 0.132063F, 0.128576F, 0.132293493F,
      0.126224F, 0.128856F, 0.132082F, 0.135632F, 0.132083F, 0.126933F,
      0.133348F, 0.129801F, 0.133668F, 0.126804F, 0.126396F, 0.130698F,
      0.126859F, 0.131222F, 0.131440505F, 0.132414F, 0.129683F, 0.130311F,
      0.13114F, 0.129121F, 0.133522F, 0.132461F, 0.130844F, 0.131958F, 0.126212F,
      0.127506F, 0.128503F, 0.133401F, 0.135924F, 0.126909F, 0.12971F,
      0.132584333F, 0.132292F, 0.139825F, 0.145682F, 0.138025F, 0.1396465F,
      0.138454F, 0.13916F, 0.140663F, 0.142531F, 0.13653F, 0.13915351F,
      0.142536F, 0.138617992F, 0.140876F, 0.1396F, 0.138418F, 0.139276505F,
      0.142807F, 0.14292F, 0.143487F, 0.142868F, 0.14015F, 0.139467F, 0.142133F,
      0.137909F, 0.143325F, 0.13967F, 0.138886511F, 0.138428509F, 0.143244F,
      0.140853F, 0.1361F, 0.137971F, 0.144757F, 0.145756F, 0.144629F, 0.144072F,
      0.137938F, 0.136735F, 0.141989F, 0.140558F, 0.142053008F, 0.144908F,
      0.138878F, 0.141706496F, 0.142213F, 0.137502F, 0.140037F, 0.138705F,
      0.136905F, 0.149136F, 0.153312F, 0.15373F, 0.151445F, 0.146768F, 0.152736F,
      0.1553F, 0.155605F, 0.151122F, 0.153386F, 0.155355F, 0.150774658F,
      0.149338499F, 0.148511F, 0.149291992F, 0.149287F, 0.148368F, 0.148282F,
      0.154969F, 0.149611F, 0.151999F, 0.150061F, 0.154419F, 0.153727502F,
      0.146856502F, 0.146449F, 0.152963F, 0.147021F, 0.150549F, 0.148764F,
      0.150114F, 0.14988F, 0.152478F, 0.152429491F, 0.152855337F, 0.152779F,
      0.146643F, 0.147327676F, 0.148916F, 0.155051F, 0.149277F, 0.148961F,
      0.151F, 0.153991F, 0.154783F, 0.151365F, 0.154727F, 0.153321F, 0.149035F,
      0.148436323F, 0.148095012F, 0.151519F, 0.154898F, 0.151594F, 0.159572F,
      0.161722F, 0.161859F, 0.164027F, 0.163198F, 0.162715F, 0.162945F,
      0.160628F, 0.161924F, 0.165216F, 0.162601F, 0.159368F, 0.162955F, 0.16563F,
      0.157475F, 0.164367F, 0.164963F, 0.164951F, 0.156919F, 0.16154F,
      0.161259502F, 0.159618F, 0.161287F, 0.157683F, 0.159881502F, 0.159962F,
      0.163655F, 0.15906F, 0.159835488F, 0.163148F, 0.156311F, 0.164897F,
      0.161337F, 0.162722F, 0.156965F, 0.164127F, 0.164382F, 0.160437F,
      0.159786F, 0.164232F, 0.162408F, 0.160405F, 0.164604F, 0.160752F,
      0.165268F, 0.158952F, 0.167477F, 0.169863F, 0.170371F, 0.169163F,
      0.168075F, 0.172512F, 0.16731F, 0.16877F, 0.171889335F, 0.16989F,
      0.169439F, 0.169094F, 0.172088504F, 0.166078F, 0.175687F, 0.168362F,
      0.167246F, 0.172829F, 0.166097F, 0.170407F, 0.171664F, 0.172553F,
      0.171670496F, 0.166711F, 0.168068F, 0.175115F, 0.1703F, 0.175882F,
      0.167755F, 0.166261F, 0.170955F, 0.172159F, 0.169871F, 0.167833F,
      0.171867F, 0.169535F, 0.170113F, 0.170303F, 0.170723F, 0.172424F,
      0.169352502F, 0.16945F, 0.175589F, 0.175334F, 0.16751F, 0.175146F,
      0.166022F, 0.171524F, 0.168939337F, 0.170938F, 0.172125F, 0.183285F,
      0.180969507F, 0.182719F, 0.176744491F, 0.17778F, 0.183115497F, 0.180902F,
      0.181885F, 0.176326F, 0.182200491F, 0.178619F, 0.183805F, 0.180118F,
      0.183008F, 0.183128F, 0.17605F, 0.183621496F, 0.182019F, 0.184222F,
      0.184931502F, 0.185469F, 0.182386F, 0.184805F, 0.179508F, 0.180569F,
      0.184511F, 0.177065F, 0.182822F, 0.181081F, 0.181696F, 0.179574504F,
      0.18046F, 0.178484F, 0.183998F, 0.176328F, 0.183594F, 0.184448F, 0.179982F,
      0.17756F, 0.184083F, 0.179633F, 0.179F, 0.18589F, 0.181253F, 0.181621F,
      0.185376F, 0.183125F, 0.176234F, 0.177708F, 0.180454F, 0.180688F,
      0.179995507F, 0.184503F, 0.177205F, 0.189096F, 0.191408F, 0.189561F,
      0.189380497F, 0.190422F, 0.190612F, 0.189769F, 0.195531F, 0.191859F,
      0.186199F, 0.189687327F, 0.189041F, 0.187692F, 0.187212F, 0.190444499F,
      0.189402F, 0.189181F, 0.186701F, 0.190196007F, 0.188749F, 0.193348F,
      0.188392F, 0.188175F, 0.189451F, 0.188188F, 0.189712F, 0.188348F,
      0.192466F, 0.19201F, 0.191017F, 0.189640492F, 0.194313F, 0.186165F,
      0.189058512F, 0.192614F, 0.1936775F, 0.191187F, 0.189566F, 0.189199F,
      0.187547F, 0.188976F, 0.192799F, 0.193291F, 0.191483F, 0.191947F,
      0.192423F, 0.186013F, 0.191115F, 0.188154F, 0.189195991F, 0.188622F,
      0.20269F, 0.201978F, 0.203301F, 0.204578F, 0.20394F, 0.204777F, 0.203218F,
      0.202998F, 0.199035F, 0.199539F, 0.199901F, 0.196031F, 0.1995195F,
      0.205465F, 0.196533F, 0.196967F, 0.204305F, 0.198197F, 0.198019F,
      0.203682497F, 0.196343F, 0.20361F, 0.197113F, 0.197072F, 0.204756F,
      0.200771F, 0.198073F, 0.197667F, 0.19625F, 0.199364F, 0.197392F, 0.197176F,
      0.20056F, 0.204128F, 0.204897F, 0.198246F, 0.198176F, 0.204031F, 0.201841F,
      0.204134494F, 0.202373505F, 0.198946F, 0.19879F, 0.20531F, 0.199361F,
      0.205064F, 0.196864F, 0.197741F, 0.205482F, 0.198954F, 0.200642F,
      0.200026F, 0.202087F, 0.202298492F, 0.21073F, 0.211018F, 0.213046F,
      0.209177F, 0.206642F, 0.215054F, 0.209917501F, 0.213197F, 0.210663497F,
      0.20934F, 0.21245F, 0.213978F, 0.208219F, 0.208623F, 0.207715F, 0.215436F,
      0.215917F, 0.207824F, 0.210831F, 0.209288F, 0.213949F, 0.209832504F,
      0.209218F, 0.208037F, 0.215096F, 0.212416F, 0.215234F, 0.211694F,
      0.206881F, 0.208471F, 0.211901F, 0.20665F, 0.212984502F, 0.215229F,
      0.210118F, 0.20644F, 0.212022F, 0.213802F, 0.210468F, 0.209924F, 0.213986F,
      0.206732F, 0.212748F, 0.206467F, 0.211058661F, 0.210814F, 0.215397F,
      0.213747665F, 0.209341F, 0.213253F, 0.215684F, 0.210833F, 0.211087495F,
      0.21295F, 0.214686F, 0.224446F, 0.218454F, 0.216929495F, 0.225896F,
      0.225858F, 0.216108F, 0.219408F, 0.222096F, 0.2229155F, 0.219456F,
      0.22197F, 0.222634F, 0.218653F, 0.223619F, 0.219849F, 0.217042F, 0.219951F,
      0.220387F, 0.219057F, 0.223635F, 0.221284F, 0.217275F, 0.223889F,
      0.218519F, 0.222184F, 0.222797245F, 0.220263F, 0.224571F, 0.222308F,
      0.218364F, 0.225952F, 0.220583F, 0.222624F, 0.218669F, 0.219663F,
      0.216799F, 0.217419F, 0.219339F, 0.216742F, 0.218656F, 0.225042F,
      0.217898F, 0.223267F, 0.220768F, 0.22582F, 0.220537F, 0.224596F, 0.217033F,
      0.222367F, 0.219905496F, 0.225848F, 0.225616F, 0.220989F, 0.21855F,
      0.220868F, 0.217875F, 0.217402F, 0.219777F, 0.221438F, 0.23168F, 0.231212F,
      0.23259601F, 0.230495661F, 0.232635F, 0.230903F, 0.231823F, 0.227324665F,
      0.228221F, 0.227989F, 0.230554491F, 0.22662F, 0.231342F, 0.231878F,
      0.233709499F, 0.227609F, 0.23438F, 0.231188F, 0.232571F, 0.232507F,
      0.235763F, 0.227003F, 0.232549F, 0.233552F, 0.226791F, 0.227416F,
      0.232531F, 0.232424F, 0.234269F, 0.227735F, 0.226191F, 0.227298F,
      0.235135F, 0.234406F, 0.2337F, 0.22629F, 0.229474F, 0.228558F, 0.228715F,
      0.231741F, 0.232794493F, 0.2338F, 0.230105668F, 0.235157F, 0.231173009F,
      0.226912F, 0.233755F, 0.231041F, 0.235554F, 0.235479F, 0.234327F,
      0.228172F, 0.244720504F, 0.238543F, 0.238786F, 0.236828F, 0.24417F,
      0.23983933F, 0.243119F, 0.236873F, 0.24199F, 0.239044F, 0.241228F,
      0.240696F, 0.242729F, 0.241689F, 0.238294512F, 0.236686F, 0.239057F,
      0.243799F, 0.237229F, 0.240024328F, 0.240787F, 0.236477F, 0.243143F,
      0.240443F, 0.23802F, 0.243406F, 0.242428F, 0.240347F, 0.241849F,
      0.238099992F, 0.243735F, 0.236004F, 0.244889F, 0.241343F, 0.243945509F,
      0.237664F, 0.240517F, 0.236647F, 0.236719F, 0.243111F, 0.241328F,
      0.242543F, 0.241080657F, 0.242808F, 0.243808508F, 0.243805F, 0.241274506F,
      0.242479F, 0.240267F, 0.239174F, 0.240239F, 0.241661F, 0.244169F, 0.2451F,
      0.241114F, 0.251339495F, 0.25414F, 0.252161F, 0.2471F, 0.246241F,
      0.246773F, 0.25099F, 0.246648F, 0.255556F, 0.250397503F, 0.246136F,
      0.249085F, 0.246465F, 0.252094507F, 0.252259493F, 0.25404F, 0.250288F,
      0.251108F, 0.255065F, 0.255425F, 0.24608F, 0.24743F, 0.250099F, 0.25368F,
      0.253967494F, 0.251465F, 0.246149F, 0.250471F, 0.24990949F, 0.251349F,
      0.24643F, 0.246435F, 0.251158F, 0.247308F, 0.249149F, 0.255229F, 0.249615F,
      0.247208F, 0.254037F, 0.253716F, 0.253978F, 0.254792F, 0.247953F,
      0.246942F, 0.252171516F, 0.255117F, 0.251516F, 0.254657F, 0.254654F,
      0.249963F, 0.264884F, 0.262672F, 0.265198499F, 0.264914513F, 0.259198F,
      0.259467F, 0.264737487F, 0.262782F, 0.261714518F, 0.264176F, 0.257125F,
      0.263421F, 0.258164F, 0.262522519F, 0.256968F, 0.256876F, 0.26189F,
      0.25823F, 0.257311F, 0.26244849F, 0.260103673F, 0.262441F, 0.259673F,
      0.265762F, 0.262821F, 0.264234F, 0.261937F, 0.25847F, 0.26517F, 0.264328F,
      0.262629F, 0.265218F, 0.256181F, 0.263111F, 0.259158F, 0.260843F,
      0.259376496F, 0.257699F, 0.255999F, 0.265734F, 0.26588F, 0.258626F,
      0.25817F, 0.258971F, 0.262776017F, 0.264155F, 0.264318F, 0.264152F,
      0.270781F, 0.270273983F, 0.27161333F, 0.268029F, 0.267797F, 0.27581F,
      0.271267F, 0.269952F, 0.271513F, 0.270921F, 0.270731F, 0.268199F,
      0.267898F, 0.272097F, 0.271622F, 0.271421015F, 0.271872F, 0.271703F,
      0.268251F, 0.274998F, 0.272844F, 0.267599F, 0.267398F, 0.273031F,
      0.275713F, 0.27212F, 0.271031F, 0.274402F, 0.268237F, 0.270632F, 0.273064F,
      0.266161F, 0.270396F, 0.271698F, 0.269762F, 0.267254F, 0.269418F,
      0.269492F, 0.267293F, 0.274090528F, 0.271596F, 0.275285F, 0.271701F,
      0.269932F, 0.275222F, 0.275053F, 0.266223F, 0.285707F, 0.279322F,
      0.283682F, 0.28098467F, 0.279393494F, 0.278520495F, 0.281336F, 0.276695F,
      0.28277F, 0.280072F, 0.283811F, 0.283499F, 0.2818F, 0.284462F, 0.277042F,
      0.280881F, 0.278887F, 0.280432F, 0.276225F, 0.282991F, 0.283282F,
      0.281804F, 0.276477F, 0.281716F, 0.284504F, 0.276038F, 0.279853F,
      0.281826F, 0.278628F, 0.281726F, 0.279899F, 0.285959F, 0.282402F,
      0.279845F, 0.279223502F, 0.277401F, 0.276442F, 0.283831477F, 0.280792F,
      0.277968F, 0.276684F, 0.283905F, 0.276423F, 0.277128F, 0.282363F,
      0.279467523F, 0.285864F, 0.280801475F, 0.280103F, 0.282818496F, 0.277087F,
      0.283781F, 0.277951F, 0.287459016F, 0.294709F, 0.291854F, 0.2941F,
      0.291368F, 0.29289F, 0.289812982F, 0.289945F, 0.29305F, 0.287457F,
      0.295323F, 0.289121509F, 0.291702F, 0.288361F, 0.286675F, 0.291217F,
      0.293529F, 0.287675F, 0.288432F, 0.288714F, 0.295228F, 0.294476F,
      0.293013F, 0.295053F, 0.287755F, 0.291895F, 0.286463F, 0.287397F,
      0.295919F, 0.288458F, 0.294523F, 0.287479F, 0.293249F, 0.292433F,
      0.288324F, 0.289874F, 0.294259F, 0.286096F, 0.289414F, 0.287974F,
      0.292741F, 0.292950511F, 0.287242502F, 0.292704523F, 0.294337F, 0.286092F,
      0.294568F, 0.29121834F, 0.294346F, 0.289933F, 0.294606F, 0.305696F,
      0.300242F, 0.305107F, 0.297662F, 0.304548F, 0.303717F, 0.30453F, 0.302364F,
      0.30179F, 0.301363F, 0.299074F, 0.301789F, 0.302407F, 0.297534F, 0.300967F,
      0.305907F, 0.296472F, 0.30104F, 0.300968498F, 0.298985F, 0.300606F,
      0.302753F, 0.303474F, 0.30397F, 0.29729F, 0.300114F, 0.303726F, 0.300109F,
      0.302052F, 0.304637F, 0.299746F, 0.300864F, 0.298366F, 0.305201F,
      0.301582F, 0.301423F, 0.297374F, 0.296481F, 0.303235F, 0.297463F,
      0.298519F, 0.301863492F, 0.301172F, 0.30402F, 0.29872F, 0.303165495F,
      0.301169F, 0.30304F, 0.305575F, 0.302384F, 0.304445F, 0.297480494F,
      0.298534F, 0.302775F, 0.303254F, 0.312488F, 0.312905F, 0.311196F, 0.30784F,
      0.315224F, 0.307208F, 0.314409F, 0.307152F, 0.310399503F, 0.309065F,
      0.310859F, 0.307444F, 0.309417F, 0.31293F, 0.308324F, 0.313919F, 0.31576F,
      0.312305F, 0.313451F, 0.31201F, 0.315456F, 0.306093F, 0.309523493F,
      0.311141F, 0.306152F, 0.308572F, 0.312508F, 0.313599F, 0.313955F,
      0.311628491F, 0.309886F, 0.3136F, 0.310465F, 0.309424F, 0.309851676F,
      0.309762F, 0.313235F, 0.30899F, 0.315558F, 0.308237493F, 0.312506F,
      0.313522F, 0.314129F, 0.309576F, 0.312521F, 0.31412F, 0.315647F, 0.319943F,
      0.316154F, 0.321031F, 0.322253F, 0.321520984F, 0.319285F, 0.321092F,
      0.322959F, 0.325441F, 0.319988F, 0.317499F, 0.321813F, 0.323345F,
      0.322542F, 0.323683F, 0.323611F, 0.321398F, 0.32109F, 0.317827F, 0.321899F,
      0.316421F, 0.320409F, 0.319796F, 0.320937F, 0.323227F, 0.321479023F,
      0.319996F, 0.321899F, 0.318408F, 0.318978F, 0.322918F, 0.322397F,
      0.322037F, 0.323272F, 0.322068F, 0.323077F, 0.32022202F, 0.321308F,
      0.319961F, 0.321836F, 0.318967F, 0.320939F, 0.324393F, 0.316143F,
      0.321158F, 0.319460332F, 0.325813F, 0.321067F, 0.321417F, 0.32346F,
      0.331192F, 0.334377F, 0.331727F, 0.329752028F, 0.32604F, 0.332457F,
      0.334057F, 0.331516F, 0.332161F, 0.328394F, 0.327306F, 0.332149F,
      0.327787F, 0.327414F, 0.329574F, 0.326137F, 0.334299F, 0.332745F,
      0.327249F, 0.328944F, 0.328574479F, 0.326819F, 0.334201F, 0.333696485F,
      0.326529F, 0.326408F, 0.328027F, 0.330403F, 0.331236F, 0.326069F,
      0.330516F, 0.326203F, 0.329057F, 0.328709F, 0.333473F, 0.335389F,
      0.328993F, 0.335766F, 0.334619F, 0.328973F, 0.326143F, 0.329392F,
      0.333977F, 0.328801F, 0.326961F, 0.336674F, 0.338535F, 0.336786F,
      0.344682F, 0.3439F, 0.341924F, 0.338643491F, 0.341221F, 0.341381F,
      0.339323F, 0.34259F, 0.340332F, 0.336081F, 0.338581F, 0.340381F, 0.343691F,
      0.336697F, 0.345433503F, 0.341146F, 0.341715F, 0.336799F, 0.341252F,
      0.345112F, 0.344370484F, 0.338639F, 0.336939F, 0.341701508F, 0.344267F,
      0.345216F, 0.338391F, 0.338561F, 0.338284F, 0.344485F, 0.341005504F,
      0.339338F, 0.343146F, 0.338931024F, 0.339185F, 0.344324F, 0.338681F,
      0.341275F, 0.337121F, 0.352196664F, 0.352195978F, 0.354645F, 0.348914F,
      0.352909F, 0.353407F, 0.352585F, 0.35098F, 0.355117F, 0.353635F, 0.348354F,
      0.354466498F, 0.355003F, 0.350711F, 0.352154732F, 0.348725F, 0.350732F,
      0.348113F, 0.352706F, 0.355816F, 0.353549F, 0.353046F, 0.35042F, 0.351505F,
      0.346976F, 0.349907F, 0.353765F, 0.35239F, 0.347047F, 0.349515F, 0.349002F,
      0.348766F, 0.355596F, 0.346972F, 0.352236F, 0.348884F, 0.349143F, 0.34783F,
      0.355153F, 0.354266F, 0.346349F, 0.348595F, 0.350524485F, 0.347015F,
      0.352325F, 0.347389F, 0.354878F, 0.348037F, 0.350851F, 0.350482F,
      0.349523485F, 0.359654486F, 0.362834F, 0.36572F, 0.362618F, 0.359567761F,
      0.35889F, 0.361496F, 0.358673513F, 0.362933F, 0.357424F, 0.357765F,
      0.364008F, 0.36361F, 0.365267F, 0.359785F, 0.361815F, 0.363203F, 0.360022F,
      0.361617F, 0.359238267F, 0.361121F, 0.357939F, 0.356763F, 0.359449F,
      0.361413F, 0.357518F, 0.357495F, 0.362711F, 0.357062F, 0.359443486F,
      0.360107F, 0.358632F, 0.359272F, 0.365561F, 0.358065F, 0.360968F,
      0.363028F, 0.362632F, 0.356227F, 0.364273F, 0.357313F, 0.365733F,
      0.356084F, 0.364205F, 0.356659F, 0.359485984F, 0.358244F, 0.361403F,
      0.361494F, 0.360658F, 0.362440526F, 0.364636F, 0.36491F, 0.364144F,
      0.367199F, 0.36771F, 0.371588498F, 0.372908F, 0.370619F, 0.369519F,
      0.375586F, 0.368771F, 0.375436F, 0.372068495F, 0.367607F, 0.368722F,
      0.3727F, 0.369759F, 0.370245F, 0.375917F, 0.371318489F, 0.371863514F,
      0.374187F, 0.36845F, 0.373146F, 0.367048F, 0.3745F, 0.373092F, 0.369814F,
      0.368727982F, 0.366767F, 0.373835F, 0.367299F, 0.370036F, 0.371515F,
      0.368666F, 0.368942499F, 0.372893F, 0.37168768F, 0.373956F, 0.374929F,
      0.369656F, 0.374579F, 0.375443F, 0.375795F, 0.372625F, 0.372004F,
      0.373245F, 0.370688F, 0.375032F, 0.367361486F, 0.369475F, 0.367236F,
      0.368535F, 0.373587F, 0.36729F, 0.368609F, 0.372541F, 0.36806F, 0.367481F,
      0.37309F, 0.377734F, 0.382708F, 0.379266F, 0.378912F, 0.379662F, 0.383301F,
      0.380117F, 0.379798502F, 0.380553335F, 0.38058266F, 0.37668F, 0.381944F,
      0.379701F, 0.382226497F, 0.384112F, 0.378045F, 0.381927F, 0.382986F,
      0.380081F, 0.377648F, 0.383178F, 0.380408F, 0.38529F, 0.380977F, 0.376391F,
      0.382392F, 0.379008F, 0.377527F, 0.383131F, 0.377874F, 0.376138F,
      0.379756F, 0.38537F, 0.378574491F, 0.379493F, 0.379655F, 0.381308F,
      0.377179503F, 0.383902F, 0.376441F, 0.385844F, 0.379772F, 0.381144F,
      0.38009F, 0.380158F, 0.381198496F, 0.379088F, 0.380255F, 0.378264F,
      0.379231F, 0.377977F, 0.382042F, 0.384365022F, 0.378155F, 0.381579F,
      0.380688F, 0.384884F, 0.377716F, 0.38622F, 0.388641F, 0.389671F, 0.390022F,
      0.394559F, 0.394021F, 0.391174F, 0.395959F, 0.395347F, 0.387959F,
      0.388811F, 0.392045F, 0.391622F, 0.393838F, 0.393831F, 0.389257F,
      0.389584F, 0.39594F, 0.393791F, 0.395956F, 0.386104F, 0.391111016F,
      0.393549F, 0.390234F, 0.388297F, 0.390719F, 0.392041504F, 0.39357F,
      0.386596F, 0.38762F, 0.391157F, 0.389335F, 0.393637F, 0.389724F, 0.395093F,
      0.387017F, 0.387255F, 0.391695F, 0.38847F, 0.393629491F, 0.388097F,
      0.390895F, 0.393441F, 0.392571F, 0.388258F, 0.395873F, 0.388624F,
      0.386997F, 0.391031F, 0.392643F, 0.391589F, 0.393228F, 0.390766025F,
      0.390002489F, 0.402292F, 0.402873665F, 0.401994F, 0.403595507F,
      0.398373485F, 0.401992F, 0.400031F, 0.402075F, 0.400342F, 0.396118F,
      0.403695F, 0.402241F, 0.40146F, 0.40113F, 0.400782F, 0.399353653F,
      0.397227F, 0.403399F, 0.403917F, 0.396496F, 0.396317F, 0.404769F,
      0.401941F, 0.403022F, 0.397248F, 0.403036505F, 0.40391F, 0.397766F,
      0.403693F, 0.401857F, 0.397749F, 0.404118F, 0.402052F, 0.396659F,
      0.404035F, 0.397535F, 0.405302F, 0.404927F, 0.400301F, 0.39963F, 0.398849F,
      0.402326F, 0.401389F, 0.396795F, 0.401601017F, 0.404045F, 0.402187F,
      0.396806F, 0.398909F, 0.400765F, 0.397627F, 0.407266F, 0.415372F,
      0.414033F, 0.415541F, 0.40745F, 0.408302F, 0.411909491F, 0.41029F,
      0.408456F, 0.414555F, 0.413199F, 0.41191F, 0.410361499F, 0.412952F,
      0.412977F, 0.410764F, 0.409862F, 0.411039F, 0.408699F, 0.412463486F,
      0.409076333F, 0.414789F, 0.407991F, 0.409512F, 0.411578F, 0.413284F,
      0.412426651F, 0.410421F, 0.411324F, 0.408548F, 0.411556F, 0.415388F,
      0.414986F, 0.406343F, 0.415745F, 0.414932F, 0.4126F, 0.406771F, 0.413038F,
      0.41177702F, 0.406488F, 0.410934F, 0.414357F, 0.410765F, 0.406811F,
      0.4107F, 0.410407F, 0.42591F, 0.422245026F, 0.419114F, 0.419001F,
      0.424269F, 0.419635475F, 0.419642508F, 0.418884F, 0.417574F, 0.425906F,
      0.418823F, 0.416696489F, 0.416108F, 0.416435F, 0.423065F, 0.425644F,
      0.420508F, 0.424793512F, 0.420964F, 0.425388F, 0.419166F, 0.420142F,
      0.419053495F, 0.420989F, 0.4221F, 0.422318F, 0.416384F, 0.416809F,
      0.417135F, 0.418714494F, 0.425048F, 0.425751F, 0.425584F, 0.416094F,
      0.417722F, 0.417332F, 0.41706F, 0.424185F, 0.421932F, 0.416688F, 0.418691F,
      0.432285F, 0.426938F, 0.430407494F, 0.431086481F, 0.43040067F, 0.432559F,
      0.432505F, 0.430354F, 0.43239F, 0.433657F, 0.428096F, 0.432003F, 0.430014F,
      0.427299F, 0.435132F, 0.428199F, 0.428003F, 0.428533F, 0.432018F, 0.43001F,
      0.431965F, 0.434218496F, 0.433759F, 0.434274F, 0.432524025F, 0.43356F,
      0.427517F, 0.432109F, 0.431111F, 0.430823F, 0.433412F, 0.428866F,
      0.428208F, 0.429308504F, 0.426557F, 0.430091F, 0.43464F, 0.426884F,
      0.432987F, 0.437541336F, 0.437305F, 0.442859F, 0.44345F, 0.437897F,
      0.440844F, 0.445656478F, 0.436615F, 0.445801F, 0.441142F, 0.444158F,
      0.442025F, 0.438288F, 0.438337F, 0.443285F, 0.44286F, 0.441874981F,
      0.437508F, 0.436211F, 0.442725F, 0.439373732F, 0.441179F, 0.438089F,
      0.442984492F, 0.441266F, 0.438356F, 0.445516F, 0.44106698F, 0.441787F,
      0.445071F, 0.436691F, 0.443803F, 0.442599F, 0.437544F, 0.436545F,
      0.442699F, 0.440916508F, 0.444349F, 0.440247F, 0.442707479F, 0.441504F,
      0.436866F, 0.440925F, 0.437819F, 0.439308F, 0.437612F, 0.440222F,
      0.438946F, 0.455479F, 0.455286F, 0.447282F, 0.446232F, 0.4512375F,
      0.449690491F, 0.451466F, 0.446422F, 0.447003F, 0.455763F, 0.446968F,
      0.450705F, 0.450507F, 0.452816F, 0.450982F, 0.450024F, 0.447534F,
      0.447236F, 0.451616526F, 0.451864F, 0.451421F, 0.450053F, 0.449478F,
      0.448217F, 0.446342F, 0.455424F, 0.452184F, 0.449490339F, 0.449717F,
      0.452337F, 0.45522F, 0.452544F, 0.449257F, 0.451474F, 0.455238F, 0.452636F,
      0.453846F, 0.450537F, 0.451664F, 0.447117984F, 0.451815F, 0.45137F,
      0.450829F, 0.451018F, 0.449728F, 0.453971F, 0.449064F, 0.448284F,
      0.455107F, 0.4544245F, 0.45506F, 0.447726488F, 0.449976981F, 0.4512F,
      0.452263F, 0.459766984F, 0.464637F, 0.465074F, 0.4586505F, 0.463127F,
      0.463438F, 0.458212F, 0.456496F, 0.457928F, 0.456157F, 0.464085F,
      0.460308F, 0.46302F, 0.460532F, 0.463449F, 0.461891F, 0.458848F, 0.464052F,
      0.456897F, 0.461988509F, 0.459218F, 0.461322F, 0.465413F, 0.461395F,
      0.458795F, 0.463375F, 0.464672F, 0.46519F, 0.461735F, 0.459745F, 0.460835F,
      0.45691F, 0.461194515F, 0.461423337F, 0.457318F, 0.462462F, 0.457891F,
      0.458102F, 0.457558F, 0.464663F, 0.464091F, 0.462112F, 0.465749F,
      0.463657F, 0.458082F, 0.460751F, 0.463218F, 0.458372F, 0.460708F,
      0.458635F, 0.463486F, 0.463635F, 0.456233F, 0.460402F, 0.465557F,
      0.463705F, 0.460642517F, 0.464592F, 0.460812509F, 0.475468F, 0.470325F,
      0.471541F, 0.472185F, 0.471028F, 0.469378F, 0.473554F, 0.466568F, 0.4664F,
      0.470922F, 0.471F, 0.473354F, 0.474132F, 0.466917F, 0.467760026F,
      0.472368F, 0.468777F, 0.473169982F, 0.471195F, 0.475829F, 0.475963F,
      0.474083F, 0.47245F, 0.469686F, 0.472787F, 0.475404F, 0.468179F, 0.468836F,
      0.472691F, 0.475167F, 0.472849F, 0.467229F, 0.469581485F, 0.467729F,
      0.474006F, 0.471662F, 0.469078F, 0.471482F, 0.465983F, 0.467894495F,
      0.470326F, 0.468843F, 0.470212F, 0.470834F, 0.470935F, 0.473642F,
      0.474435F, 0.468820512F, 0.471185327F, 0.469464F, 0.472409F, 0.48293F,
      0.479587F, 0.481857F, 0.479064F, 0.481173F, 0.476721495F, 0.485284F,
      0.476029F, 0.477319F, 0.482636F, 0.485598F, 0.482058F, 0.483653F,
      0.480279F, 0.481580496F, 0.483454F, 0.479756F, 0.481877F, 0.479267F,
      0.480294317F, 0.485951F, 0.483268F, 0.478864F, 0.483644F, 0.481744F,
      0.479102F, 0.481914F, 0.478622F, 0.479844F, 0.482622504F, 0.477723F,
      0.48258F, 0.485445F, 0.478429F, 0.485949F, 0.477777F, 0.48532F, 0.48122F,
      0.478043F, 0.485642F, 0.479385018F, 0.481447F, 0.485377F, 0.480278F,
      0.48386234F, 0.485109F, 0.479951F, 0.481820494F, 0.483849F, 0.481360674F,
      0.482723F, 0.479857F, 0.488722503F, 0.487004F, 0.489174336F, 0.492343F,
      0.49129F, 0.490089476F, 0.490732F, 0.488049F, 0.492889F, 0.490259F,
      0.490437F, 0.491417F, 0.488312662F, 0.48657F, 0.493845F, 0.488355F,
      0.49405551F, 0.489930511F, 0.49566F, 0.487488F, 0.490332F, 0.489078522F,
      0.490127F, 0.491774F, 0.488246F, 0.492803F, 0.489651F, 0.495412F,
      0.486888F, 0.486189F, 0.490698F, 0.489695F, 0.487399F, 0.49529F, 0.491959F,
      0.487645F, 0.493322F, 0.495518F, 0.488058507F, 0.490746F, 0.4927755F,
      0.492898F, 0.490128F, 0.491315F, 0.492155F, 0.489655F, 0.490382F,
      0.495597F, 0.490538F, 0.490932524F, 0.486986F, 0.494710505F, 0.488758F,
      0.491486F, 0.495896F, 0.490937F, 0.488597F, 0.495161F, 0.495334F,
      0.487278F, 0.488323F, 0.496878F, 0.5017F, 0.505148F, 0.498978F, 0.496968F,
      0.501271963F, 0.50406F, 0.50516F, 0.499515F, 0.501153F, 0.500758529F,
      0.503132F, 0.50166F, 0.499347F, 0.501887F, 0.501676F, 0.505648F, 0.502899F,
      0.499435F, 0.500696F, 0.49607F, 0.502044F, 0.496183F, 0.497167F, 0.502005F,
      0.500401F, 0.505332F, 0.504578F, 0.500998F, 0.498889F, 0.502544F,
      0.505594F, 0.503958464F, 0.499352664F, 0.505952F, 0.502843F, 0.505842F,
      0.501356F, 0.504953F, 0.496472F, 0.501819F, 0.498254F, 0.505155504F,
      0.501148F, 0.496154F, 0.496656F, 0.499937505F, 0.5016675F, 0.500818F,
      0.501639307F, 0.503162503F, 0.504157F, 0.496343F, 0.504254F, 0.500373662F,
      0.500747F, 0.496296F, 0.50155431F, 0.498813F, 0.503681F, 0.515673F,
      0.507109F, 0.509592F, 0.513931036F, 0.515474F, 0.513041F, 0.508083F,
      0.510436535F, 0.509862F, 0.510388F, 0.515581F, 0.511206F, 0.513554F,
      0.515097F, 0.514581F, 0.510854F, 0.511311F, 0.510933F, 0.510764658F,
      0.515031F, 0.510696F, 0.507496F, 0.509016F, 0.515748F, 0.515326F,
      0.506952F, 0.510265F, 0.510009F, 0.512161F, 0.510944F, 0.5061F, 0.506359F,
      0.510775F, 0.512418509F, 0.506557F, 0.506844F, 0.510095F, 0.513706F,
      0.510485291F, 0.508056F, 0.508355F, 0.525898F, 0.521544F, 0.523324F,
      0.519725F, 0.525118F, 0.524886F, 0.517873F, 0.517892F, 0.521896F,
      0.518373F, 0.519423962F, 0.523529F, 0.520207524F, 0.519258499F, 0.523074F,
      0.519628F, 0.524238F, 0.519416F, 0.517353F, 0.519694F, 0.517745F,
      0.520564F, 0.518592F, 0.523702F, 0.51628F, 0.520930529F, 0.52125F,
      0.517208F, 0.521145463F, 0.522821F, 0.521042F, 0.521676F, 0.518086F,
      0.525316F, 0.516923F, 0.524679F, 0.52079F, 0.525435F, 0.525018F, 0.516646F,
      0.521289F, 0.52521F, 0.520040512F, 0.525396F, 0.52277F, 0.522468F,
      0.527012F, 0.53382F, 0.533239F, 0.532004F, 0.534473F, 0.529233F,
      0.531100512F, 0.53187F, 0.531661F, 0.531927F, 0.531827F, 0.532329F,
      0.531864F, 0.534783F, 0.528252F, 0.530157506F, 0.532432F, 0.52789F,
      0.532509F, 0.534561F, 0.529424F, 0.526584F, 0.529778F, 0.534197F,
      0.529564F, 0.529488504F, 0.533567F, 0.535495F, 0.532298F, 0.532619F,
      0.532434F, 0.533656F, 0.530724F, 0.530672F, 0.526577F, 0.53299F, 0.529537F,
      0.526555F, 0.52952F, 0.528014F, 0.532315F, 0.529296F, 0.5349F, 0.534293F,
      0.533475F, 0.528819F, 0.529502F, 0.528634F, 0.529052F, 0.527285F,
      0.532718F, 0.532213F, 0.529632F, 0.53258F, 0.531366F, 0.540241F, 0.54051F,
      0.53898F, 0.536468F, 0.541638F, 0.544184F, 0.544124F, 0.540644F, 0.539333F,
      0.539629F, 0.545245F, 0.537308F, 0.537122F, 0.545792F, 0.542369F,
      0.543808F, 0.54029F, 0.540194F, 0.538184F, 0.54047F, 0.542979F,
      0.542176962F, 0.538523F, 0.543722F, 0.542896032F, 0.543805957F,
      0.538956523F, 0.539421F, 0.538382F, 0.539622F, 0.541796F, 0.541723F,
      0.54244F, 0.543799F, 0.540628F, 0.540621F, 0.53869F, 0.538551F, 0.541288F,
      0.54001F, 0.541636F, 0.540607452F, 0.539832F, 0.539994F, 0.543539F,
      0.537004948F, 0.541628F, 0.543807F, 0.543841F, 0.542383492F, 0.538478F,
      0.536448F, 0.537838F, 0.536925F, 0.538150311F, 0.546823502F, 0.549602F,
      0.553315043F, 0.54924655F, 0.548696F, 0.54606F, 0.55462F, 0.554951F,
      0.549827F, 0.547756F, 0.554129481F, 0.549699F, 0.55208F, 0.550196052F,
      0.546504F, 0.553339F, 0.555311F, 0.550106F, 0.550081F, 0.550138F,
      0.554157F, 0.550826F, 0.54852F, 0.555197477F, 0.551203F, 0.550772F,
      0.550823F, 0.552524F, 0.546381F, 0.549203F, 0.548483F, 0.555837F,
      0.552403F, 0.548307F, 0.547883F, 0.546572F, 0.55106765F, 0.547133F,
      0.553415F, 0.548713F, 0.554481F, 0.551032F, 0.553605F, 0.550042F, 0.55179F,
      0.558599F, 0.562145F, 0.560575F, 0.561388969F, 0.559843F, 0.557943F,
      0.559219956F, 0.559594F, 0.556498F, 0.556208F, 0.557171F, 0.557238F,
      0.558262F, 0.55603F, 0.564325F, 0.561149F, 0.558929503F, 0.563127F,
      0.56365F, 0.556175F, 0.557707F, 0.564312F, 0.559211671F, 0.563760519F,
      0.562429F, 0.563627958F, 0.564328F, 0.564188F, 0.560567F, 0.563277F,
      0.556635F, 0.557709F, 0.56386F, 0.562938F, 0.560093F, 0.56341F, 0.564587F,
      0.557855F, 0.56455F, 0.558203F, 0.560776F, 0.560449F, 0.562705F, 0.558215F,
      0.560538F, 0.557642519F, 0.558306F, 0.56484F, 0.569855F, 0.572004F,
      0.573736548F, 0.567605F, 0.572324514F, 0.571076F, 0.57014F, 0.568408F,
      0.567035F, 0.574479F, 0.575608F, 0.572450519F, 0.566037F, 0.575211F,
      0.568258F, 0.575945F, 0.571963F, 0.573519F, 0.5697F, 0.571666F, 0.572018F,
      0.573857F, 0.572042F, 0.574118F, 0.569088F, 0.570198F, 0.568662F,
      0.572103F, 0.575138F, 0.568953F, 0.572096F, 0.566613F, 0.569137F,
      0.573084F, 0.566397F, 0.571141F, 0.57559F, 0.573092F, 0.575015F, 0.57379F,
      0.572433F, 0.569955528F, 0.568951F, 0.570086F, 0.573995F, 0.575943F,
      0.569249F, 0.572447F, 0.567690969F, 0.574788F, 0.568954F, 0.566279F,
      0.575486F, 0.571660519F, 0.571509F, 0.573096F, 0.569131F, 0.568484F,
      0.575708F, 0.581722F, 0.580378F, 0.576599F, 0.58169F, 0.576124F, 0.583121F,
      0.583017F, 0.585073F, 0.584558F, 0.57852304F, 0.579204F, 0.579896F,
      0.577924F, 0.579725F, 0.582243F, 0.584496F, 0.584525F, 0.580242F,
      0.583218F, 0.576518F, 0.576151F, 0.585198F, 0.581331F, 0.578527F,
      0.582865F, 0.578468F, 0.577082F, 0.585494F, 0.582796F, 0.579975F,
      0.576937F, 0.583561F, 0.580188F, 0.582967F, 0.584428F, 0.579703F,
      0.583375F, 0.578033F, 0.58169663F, 0.578638F, 0.59573F, 0.591343F,
      0.592328966F, 0.594519496F, 0.590214F, 0.588506F, 0.586917F, 0.592734F,
      0.589232F, 0.587641F, 0.589762F, 0.592682F, 0.586203F, 0.589F, 0.587529F,
      0.594256F, 0.586065F, 0.588024F, 0.589137F, 0.595650494F, 0.586362F,
      0.588133F, 0.595364F, 0.591385F, 0.594016F, 0.595289F, 0.588443F, 0.59233F,
      0.592574F, 0.591676F, 0.592257F, 0.595104F, 0.594966F, 0.593263F,
      0.592339F, 0.594938F, 0.594353F, 0.594826F, 0.58613F, 0.592677F, 0.589681F,
      0.589775F, 0.590388F, 0.586773F, 0.59203F, 0.592219F, 0.59435F, 0.586535F,
      0.594633F, 0.595094F, 0.593331F, 0.587686F, 0.594133F, 0.591175318F,
      0.592909F, 0.595805F, 0.603258967F, 0.597348F, 0.598962307F, 0.603667F,
      0.601084471F, 0.597226F, 0.601471364F, 0.598266959F, 0.59958446F,
      0.598519504F, 0.600304F, 0.604755F, 0.59836F, 0.601917F, 0.60155797F,
      0.600335F, 0.603839F, 0.596406F, 0.601268053F, 0.605465F, 0.597424F,
      0.602037F, 0.597592F, 0.599196F, 0.596757F, 0.601044F, 0.59807F,
      0.604936481F, 0.598129F, 0.603675F, 0.600299F, 0.598381F, 0.600157F,
      0.601809F, 0.600794F, 0.599792F, 0.602897F, 0.60520947F, 0.602327F,
      0.601651F, 0.600634F, 0.603482366F, 0.597566F, 0.602812529F, 0.605096F,
      0.601251304F, 0.600280523F, 0.603817F, 0.604698479F, 0.605606F,
      0.608629465F, 0.613472F, 0.609533F, 0.611629963F, 0.613853455F,
      0.611942351F, 0.615723968F, 0.613697F, 0.61287F, 0.615864F, 0.611001F,
      0.6096F, 0.614548F, 0.6096F, 0.607514501F, 0.60885F, 0.6096F, 0.608221F,
      0.6096F, 0.608511F, 0.6096F, 0.6096F, 0.6084F, 0.6096F, 0.607806F, 0.6096F,
      0.606942F, 0.606485F, 0.608707F, 0.607225F, 0.606214F, 0.607548F, 0.6096F,
      0.609066963F, 0.606451F, 0.6096F, 0.6096F, 0.6096F, 0.608048F, 0.609463F,
      0.6096F, 0.610994F, 0.61065352F, 0.611084F, 0.610260248F, 0.615775F,
      0.615949F, 0.609487653F, 0.612786233F, 0.61580503F, 0.610402F, 0.613532F,
      0.618636F, 0.616219F, 0.617901F, 0.623354F, 0.623273492F, 0.620066F,
      0.620619535F, 0.62466F, 0.618364692F, 0.617021501F, 0.619966F, 0.618694F,
      0.622368F, 0.620138F, 0.616905F, 0.623017788F, 0.619260669F, 0.619171F,
      0.625758F, 0.625384F, 0.619042516F, 0.620341F, 0.625616F, 0.619303286F,
      0.619416F, 0.622613F, 0.620634496F, 0.62337F, 0.620719F, 0.622367F,
      0.619477689F, 0.622056F, 0.62571F, 0.619903743F, 0.622187F, 0.622587F,
      0.62045F, 0.619799674F, 0.621987462F, 0.619297504F, 0.619861F, 0.617881F,
      0.61869967F, 0.624419F, 0.620518804F, 0.618643F, 0.623764F, 0.622136F,
      0.620352507F, 0.622332F, 0.624823F, 0.622868538F, 0.622981F, 0.617212F,
      0.617579F, 0.62462F, 0.625656F, 0.620570481F, 0.620798945F, 0.622078776F,
      0.617670238F, 0.622101F, 0.618948F, 0.620610476F, 0.621572F, 0.620946F,
      0.621739F, 0.622586F, 0.620503F, 0.618905F, 0.619331479F, 0.619204044F,
      0.619538248F, 0.619959F, 0.620845258F, 0.618926F, 0.619855046F,
      0.622652531F, 0.619529F, 0.622538F, 0.625652F, 0.624907F, 0.620338F,
      0.617989F, 0.62495F, 0.623469F, 0.618186474F, 0.622771F, 0.621330678F,
      0.621309F, 0.61704F, 0.619803F, 0.616721F, 0.622281F, 0.619996F, 0.621274F,
      0.62279135F, 0.621104956F, 0.623535514F, 0.616968F, 0.622847F, 0.625853F,
      0.617044F, 0.620604F, 0.624334F, 0.618842F, 0.621123F, 0.625465F,
      0.624589F, 0.620492339F, 0.620787F, 0.620561481F, 0.625321F, 0.623783F,
      0.622099876F, 0.620777309F, 0.621417F, 0.623954F, 0.619965F, 0.622109F,
      0.623198F, 0.621004701F, 0.621007264F, 0.623396F, 0.622174F, 0.620174F,
      0.620193481F, 0.621296644F, 0.620716F, 0.619678F, 0.624043465F,
      0.623753488F, 0.624027491F, 0.616738498F, 0.624335F, 0.62164247F,
      0.625126F, 0.625022F, 0.622208F, 0.618573368F, 0.621577F, 0.619365F,
      0.618418F, 0.625429034F, 0.62234F, 0.620327F, 0.621971F, 0.618552506F,
      0.62325F, 0.621199F, 0.620205F, 0.620739F, 0.623660445F, 0.623064518F,
      0.617093503F, 0.62013F, 0.625529F, 0.61707F, 0.620189667F, 0.622138F,
      0.621901512F, 0.623628F, 0.619198F, 0.621493F, 0.618474F, 0.619404256F,
      0.620815516F, 0.616456F, 0.621927321F, 0.621792F, 0.623282969F,
      0.617385507F, 0.620733F, 0.61744523F, 0.62029F, 0.62453F, 0.616394F,
      0.624833F, 0.624051F, 0.621407F, 0.633029461F, 0.629614F, 0.634966F,
      0.630745F, 0.629679F, 0.631960511F, 0.629487F, 0.629868686F, 0.628715F,
      0.63185966F, 0.627168F, 0.629791F, 0.633287F, 0.626277F, 0.62914151F,
      0.630682468F, 0.62948966F, 0.63266933F, 0.628728F, 0.630679965F,
      0.63262105F, 0.627424657F, 0.629209F, 0.632087F, 0.631112516F,
      0.631260514F, 0.631162345F, 0.630562F, 0.630986691F, 0.626592517F,
      0.63233F, 0.633417F, 0.630311F, 0.630977333F, 0.630086F, 0.63442F,
      0.632918477F, 0.631608725F, 0.631337F, 0.63120997F, 0.629011F, 0.633389F,
      0.627755F, 0.631233F, 0.632552743F, 0.626591F, 0.632779241F, 0.628826499F,
      0.628161F, 0.628687F, 0.62915F, 0.634839F, 0.629910946F, 0.628247499F,
      0.630274F, 0.632108212F, 0.630833F, 0.627937F, 0.630032F, 0.628669F,
      0.631811678F, 0.632599F, 0.631513F, 0.629304F, 0.632793486F, 0.631504536F,
      0.630545497F, 0.628163517F, 0.634715557F, 0.630458474F, 0.627296F,
      0.635941F, 0.631433F, 0.62929821F, 0.632138F, 0.631951272F, 0.63181F,
      0.635589F, 0.627567649F, 0.634635F, 0.626508F, 0.63156F, 0.631551F,
      0.633126497F, 0.63057369F, 0.63190949F, 0.632869F, 0.627169F, 0.633049F,
      0.630199313F, 0.629415F, 0.63155669F, 0.629173517F, 0.628552496F,
      0.632762F, 0.631844F, 0.629286F, 0.634795F, 0.630597F, 0.629383F,
      0.632094F, 0.629874F, 0.631046355F, 0.632054389F, 0.630304F, 0.634808F,
      0.632285476F, 0.635643F, 0.627747F, 0.626466F, 0.640726447F, 0.641863F,
      0.639173F, 0.638792515F, 0.641994F, 0.642163F, 0.641541362F, 0.643393F,
      0.639628232F, 0.640419F, 0.639833F, 0.639451F, 0.638314F, 0.644022524F,
      0.640750825F, 0.644552F, 0.645175F, 0.640352309F, 0.639412F, 0.64094466F,
      0.643425F, 0.636485F, 0.645265F, 0.64534F, 0.643364F, 0.641564F, 0.63994F,
      0.645781F, 0.638054967F, 0.640750051F, 0.640065F, 0.638311F, 0.641606F,
      0.636079F, 0.639464676F, 0.640547F, 0.63621F, 0.640965F, 0.638739467F,
      0.63755F, 0.645929F, 0.641554F, 0.642709315F, 0.640245497F, 0.64101249F,
      0.64521F, 0.641972F, 0.643882036F, 0.644077F, 0.643066525F, 0.636508F,
      0.640715837F, 0.640987515F, 0.641617F, 0.643984F, 0.641027153F,
      0.639368653F, 0.645768F, 0.64408505F, 0.639026F, 0.639950335F, 0.637042F,
      0.645586F, 0.641120672F, 0.639344F, 0.641839F, 0.638052F, 0.638148F,
      0.640183F, 0.644097F, 0.63925F, 0.643286526F, 0.644788F, 0.640431F,
      0.638534665F, 0.644224F, 0.640655F, 0.643544555F, 0.637181F, 0.643873F,
      0.643331F, 0.640319765F, 0.638865F, 0.642904043F, 0.638983965F, 0.645323F,
      0.642861F, 0.645661F, 0.643854F, 0.642714F, 0.643384F, 0.644389F,
      0.641040504F, 0.636173F, 0.645055F, 0.637948F, 0.639113F, 0.641728342F,
      0.64084F, 0.643361449F, 0.644419F, 0.640788F, 0.637888F, 0.644822F,
      0.642231F, 0.643783F, 0.640782F, 0.643551F, 0.641925F, 0.641170502F,
      0.640035748F, 0.643171F, 0.645863F, 0.638181F, 0.639429F, 0.642964482F,
      0.6454F, 0.639135301F, 0.642205F, 0.644927F, 0.643157482F, 0.645861F,
      0.645895F, 0.643478F, 0.641757488F, 0.641307235F, 0.645114F, 0.644799F,
      0.639350355F, 0.639485F, 0.643912F, 0.64547348F, 0.636566F, 0.643893957F,
      0.642344F, 0.640936494F, 0.644206F, 0.645822F, 0.643042F, 0.638734F,
      0.644607F, 0.643864F, 0.645689F, 0.643818498F, 0.644737482F, 0.643981F,
      0.636114F, 0.645725F, 0.645214F, 0.643212676F, 0.63911128F, 0.640336514F,
      0.643599F, 0.639518F, 0.648854F, 0.6514F, 0.650088F, 0.649618506F,
      0.653212F, 0.649211F, 0.649327636F, 0.647385478F, 0.65118F, 0.648075F,
      0.649180651F, 0.646204472F, 0.648424F, 0.649428F, 0.647495F, 0.648869F,
      0.64922F, 0.647447F, 0.648177F, 0.646728F, 0.648681F, 0.649636F, 0.646667F,
      0.64661F, 0.646504045F, 0.646354F, 0.648565531F, 0.648666263F,
      0.648068547F, 0.648105F, 0.646908045F, 0.64602F, 0.649309F, 0.647951F,
      0.650002F, 0.647097647F, 0.64949435F, 0.649642F, 0.649179F, 0.649138331F,
      0.649222F, 0.649106F, 0.646326F, 0.649316669F, 0.649858F, 0.649842501F,
      0.648079515F, 0.647327F, 0.649779F, 0.646615505F, 0.648397326F, 0.649862F,
      0.646025F, 0.649674654F, 0.648694396F, 0.647812F, 0.648271F, 0.646676F,
      0.648560524F, 0.649825F, 0.646411F, 0.647679329F, 0.649234F, 0.646821F,
      0.648338F, 0.64992F, 0.649499F, 0.649084F, 0.646143F, 0.647102F, 0.646629F,
      0.649445F, 0.647861481F, 0.646534F, 0.64791F, 0.648235F, 0.649866F,
      0.650026F, 0.646342F, 0.646995F, 0.648934245F, 0.647175F, 0.646716F,
      0.646281F, 0.646359324F, 0.647203445F, 0.647523F, 0.647668F, 0.647318959F,
      0.64779532F, 0.649915F, 0.64704F, 0.647306502F, 0.646536F, 0.647496F,
      0.648893F, 0.646474302F, 0.647203F, 0.651993F, 0.647858322F, 0.650604486F,
      0.648382545F, 0.648229F, 0.648890674F, 0.648776F, 0.653491F, 0.64902F,
      0.648139F, 0.646345F, 0.647565F };

    b_pointCloud *this_;
    this_ = this;
    for (int i{0}; i < 22317; i++) {
      this_->Location[i] = xyzPoints[i];
    }

    pointclouds::internal::codegen::pc::pointCloudArray r;
    this_->Color.set_size(0, 0);
    this_->Normal.set_size(0, 0);
    this_->Intensity.set_size(0, 0);
    this_->RangeData.set_size(0, 0);
    this_->PointCloudArrayData.set_size(1, 1);
    this_->PointCloudArrayData[0] = r;
    this_->Kdtree = iobj_0;
    this_->matlabCodegenIsDeleted = false;
    return this_;
  }

  pointCloud *pointCloud::init(const ::coder::array<double, 2U> &varargin_1,
    const ::coder::array<double, 1U> &varargin_3, ::coder::vision::internal::
    codegen::Kdtree *iobj_0)
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

  void c_pointCloud::matlabCodegenDestructor()
  {
    if (!matlabCodegenIsDeleted) {
      matlabCodegenIsDeleted = true;
    }
  }

  void b_pointCloud::matlabCodegenDestructor()
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

  void pointCloud::removeInvalidPoints(c_pointCloud *iobj_0, c_pointCloud
    **ptCloudOut, ::coder::array<double, 1U> &indicesOut) const
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

  void pointCloud::subsetImpl(const ::coder::array<double, 1U> &indices, ::coder::
    array<double, 2U> &loc, ::coder::array<unsigned char, 2U> &c, ::coder::array<
    double, 2U> &nv, ::coder::array<double, 1U> &intensity, ::coder::array<
    double, 2U> &r) const
  {
    int b_loop_ub;
    int loop_ub;
    if (Location.size(0) != 0) {
      loc.set_size(indices.size(0), 3);
      loop_ub = indices.size(0);
      for (int i{0}; i < 3; i++) {
        for (int i1{0}; i1 < loop_ub; i1++) {
          loc[i1 + loc.size(0) * i] = Location[(static_cast<int>(indices[i1]) +
            Location.size(0) * i) - 1];
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
          c[i1 + c.size(0) * i] = Color[(static_cast<int>(indices[i1]) +
            Color.size(0) * i) - 1];
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
          nv[i1 + nv.size(0) * i] = Normal[(static_cast<int>(indices[i1]) +
            Normal.size(0) * i) - 1];
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
          r[i1 + r.size(0) * i] = RangeData[(static_cast<int>(indices[i1]) +
            RangeData.size(0) * i) - 1];
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
    e_struct_T r;
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

            leftChild.set_size(b_this->LeftChild.size(0), b_this->LeftChild.size
                               (1));
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
              if (loc[j + 7439 * (static_cast<int>(cutDim[static_cast<int>
                    (startNode) - 1]) - 1)] <= cutVal[static_cast<int>(startNode)
                  - 1]) {
                startNode = leftChild[static_cast<int>(startNode) - 1];
              } else {
                startNode = rightChild[static_cast<int>(startNode) - 1];
              }
            }

            vision::internal::codegen::Kdtree::getNodeFromArray(b_this->IdxAll,
              b_this->IdxDim, startNode, nodeIdxThis);
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
                lowBounds[nxout] = b_this->LowerBounds[(static_cast<int>
                  (startNode) + b_this->LowerBounds.size(0) * nxout) - 1];
              }

              yk = b_this->UpperBounds.size(1);
              upBounds.set_size(1, yk);
              for (nxout = 0; nxout < yk; nxout++) {
                upBounds[nxout] = b_this->UpperBounds[(static_cast<int>
                  (startNode) + b_this->UpperBounds.size(0) * nxout) - 1];
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
                  lowBounds[nxout] = b_this->LowerBounds[(static_cast<int>
                    (currentNode) + b_this->LowerBounds.size(0) * nxout) - 1];
                }

                yk = b_this->UpperBounds.size(1);
                upBounds.set_size(1, yk);
                for (nxout = 0; nxout < yk; nxout++) {
                  upBounds[nxout] = b_this->UpperBounds[(static_cast<int>
                    (currentNode) + b_this->UpperBounds.size(0) * nxout) - 1];
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
                    nxout = static_cast<int>(b_this->CutDim[static_cast<int>
                      (currentNode) - 1]) - 1;
                    if (loc[j + 7439 * nxout] <= b_this->CutVal[static_cast<int>
                        (currentNode) - 1]) {
                      c_this.set_size(nodeStack.size(0) + 2);
                      c_this[0] = b_this->LeftChild[static_cast<int>(currentNode)
                        - 1];
                      c_this[1] = b_this->RightChild[static_cast<int>
                        (currentNode) - 1];
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
                      c_this[0] = b_this->RightChild[static_cast<int>
                        (currentNode) - 1];
                      c_this[1] = b_this->LeftChild[static_cast<int>(currentNode)
                        - 1];
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
                    vision::internal::codegen::Kdtree::getNodeFromArray
                      (b_this->IdxAll, b_this->IdxDim, currentNode, nodeIdxThis);
                    b_loc[0] = loc[j];
                    b_loc[1] = loc_tmp;
                    b_loc[2] = b_loc_tmp;
                    vision::internal::codegen::Kdtree::searchNode(X, b_loc,
                      nodeIdxThis, numNN, &r);
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
    PCANormalImpl_single(&loc[0], &indices[0], &valid[0], 7439U, static_cast<
                         unsigned int>(indices.size(0)), &normals[0]);
  }
}

// End of code generation (pointCloud.cpp)
