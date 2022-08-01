//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Kdtree.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 01-Aug-2022 08:26:09
//

// Include Files
#include "Kdtree.h"
#include "any1.h"
#include "blockedSummation.h"
#include "find.h"
#include "log2.h"
#include "minOrMax.h"
#include "proc_mapping_internal_types.h"
#include "proc_mapping_rtwutil.h"
#include "rt_nonfinite.h"
#include "sort.h"
#include "coder_array.h"
#include <cmath>
#include <string.h>

// Function Declarations
static void
b_binary_expand_op(double in1[3], const coder::array<double, 2U> &in2, int in3,
                   const coder::vision::internal::codegen::b_Kdtree *in4,
                   double in5);

static void
c_binary_expand_op(double in1[3], const coder::array<double, 2U> &in2, int in3,
                   const coder::vision::internal::codegen::b_Kdtree *in4,
                   double in5);

// Function Definitions
//
// Arguments    : const ::coder::array<double, 2U> &X
//                const double queryPt[3]
//                const ::coder::array<unsigned int, 1U> &nodeIdxStart
//                int numNN
//                r_struct_T *pq
// Return Type  : void
//
namespace coder {
namespace vision {
namespace internal {
namespace codegen {
void Kdtree::searchNode(const ::coder::array<double, 2U> &X,
                        const double queryPt[3],
                        const ::coder::array<unsigned int, 1U> &nodeIdxStart,
                        int numNN, r_struct_T *pq)
{
  array<double, 2U> b_diffAllDim;
  array<double, 2U> diffAllDim;
  array<double, 1U> aDistOut;
  array<double, 1U> distInP;
  array<double, 1U> r;
  array<int, 1U> iidx;
  array<unsigned int, 1U> r1;
  int acoef;
  int b_acoef;
  int i;
  int varargin_2;
  diffAllDim.set_size(nodeIdxStart.size(0), 3);
  if (nodeIdxStart.size(0) != 0) {
    acoef = (X.size(1) != 1);
    b_acoef = (nodeIdxStart.size(0) != 1);
    for (int k{0}; k < 3; k++) {
      varargin_2 = acoef * k;
      i = diffAllDim.size(0) - 1;
      for (int b_k{0}; b_k <= i; b_k++) {
        diffAllDim[b_k + diffAllDim.size(0) * k] =
            X[(static_cast<int>(nodeIdxStart[b_acoef * b_k]) +
               X.size(0) * varargin_2) -
              1] -
            queryPt[k];
      }
    }
  }
  b_diffAllDim.set_size(diffAllDim.size(0), 3);
  acoef = diffAllDim.size(0) * 3;
  for (i = 0; i < acoef; i++) {
    b_diffAllDim[i] = diffAllDim[i] * diffAllDim[i];
  }
  if (b_diffAllDim.size(0) == 0) {
    aDistOut.set_size(0);
  } else {
    acoef = b_diffAllDim.size(0);
    aDistOut.set_size(b_diffAllDim.size(0));
    for (varargin_2 = 0; varargin_2 < acoef; varargin_2++) {
      aDistOut[varargin_2] = b_diffAllDim[varargin_2];
    }
    for (int k{0}; k < 2; k++) {
      b_acoef = (k + 1) * acoef;
      for (varargin_2 = 0; varargin_2 < acoef; varargin_2++) {
        aDistOut[varargin_2] =
            aDistOut[varargin_2] + b_diffAllDim[b_acoef + varargin_2];
      }
    }
  }
  acoef = aDistOut.size(0);
  distInP.set_size(aDistOut.size(0));
  for (i = 0; i < acoef; i++) {
    distInP[i] = aDistOut[i];
  }
  ::coder::internal::sort(distInP, iidx);
  if (pq->D.size(0) == 0) {
    if (distInP.size(0) <= numNN) {
      pq->D.set_size(distInP.size(0));
      acoef = distInP.size(0);
      for (i = 0; i < acoef; i++) {
        pq->D[i] = distInP[i];
      }
      pq->b_I.set_size(iidx.size(0));
      acoef = iidx.size(0);
      for (i = 0; i < acoef; i++) {
        pq->b_I[i] = nodeIdxStart[iidx[i] - 1];
      }
    } else {
      if (numNN < 1) {
        acoef = 0;
      } else {
        acoef = numNN;
      }
      pq->D.set_size(acoef);
      for (i = 0; i < acoef; i++) {
        pq->D[i] = distInP[i];
      }
      if (numNN < 1) {
        acoef = 0;
      } else {
        acoef = numNN;
      }
      pq->b_I.set_size(acoef);
      for (i = 0; i < acoef; i++) {
        pq->b_I[i] = nodeIdxStart[iidx[i] - 1];
      }
    }
  } else {
    unsigned int cD1;
    unsigned int cD2;
    unsigned int x;
    bool exitg1;
    cD1 = 1U;
    cD2 = 1U;
    x = static_cast<unsigned int>(pq->D.size(0)) + distInP.size(0);
    if (static_cast<double>(x) > numNN) {
      acoef = numNN;
    } else {
      acoef = static_cast<int>(x);
    }
    r.set_size(acoef);
    r1.set_size(acoef);
    for (i = 0; i < acoef; i++) {
      r[i] = 0.0;
      r1[i] = 0U;
    }
    b_acoef = 0;
    exitg1 = false;
    while ((!exitg1) && (b_acoef <= acoef - 1)) {
      double d;
      double d1;
      d = distInP[static_cast<int>(cD2) - 1];
      d1 = pq->D[static_cast<int>(cD1) - 1];
      if (d1 <= d) {
        r[b_acoef] = d1;
        r1[b_acoef] = pq->b_I[static_cast<int>(cD1) - 1];
        cD1++;
        if (cD1 > static_cast<unsigned int>(pq->D.size(0))) {
          i = b_acoef + 2;
          for (int b_k{i}; b_k <= acoef; b_k++) {
            varargin_2 = static_cast<int>((cD2 + b_k) - b_acoef) - 3;
            r[b_k - 1] = distInP[varargin_2];
            r1[b_k - 1] = nodeIdxStart[iidx[varargin_2] - 1];
          }
          exitg1 = true;
        } else {
          b_acoef++;
        }
      } else {
        r[b_acoef] = d;
        r1[b_acoef] = nodeIdxStart[iidx[static_cast<int>(cD2) - 1] - 1];
        cD2++;
        if (cD2 > static_cast<unsigned int>(distInP.size(0))) {
          i = b_acoef + 2;
          for (int b_k{i}; b_k <= acoef; b_k++) {
            varargin_2 = static_cast<int>((cD1 + b_k) - b_acoef) - 3;
            r[b_k - 1] = pq->D[varargin_2];
            r1[b_k - 1] = pq->b_I[varargin_2];
          }
          exitg1 = true;
        } else {
          b_acoef++;
        }
      }
    }
    pq->D.set_size(r.size(0));
    acoef = r.size(0);
    for (i = 0; i < acoef; i++) {
      pq->D[i] = r[i];
    }
    pq->b_I.set_size(r1.size(0));
    acoef = r1.size(0);
    for (i = 0; i < acoef; i++) {
      pq->b_I[i] = r1[i];
    }
  }
}

//
// Arguments    : double in1[3]
//                const coder::array<double, 2U> &in2
//                int in3
//                const coder::vision::internal::codegen::b_Kdtree *in4
//                double in5
// Return Type  : void
//
} // namespace codegen
} // namespace internal
} // namespace vision
} // namespace coder
static void
b_binary_expand_op(double in1[3], const coder::array<double, 2U> &in2, int in3,
                   const coder::vision::internal::codegen::b_Kdtree *in4,
                   double in5)
{
  int stride_0_1;
  stride_0_1 = (in4->UpperBounds.size(1) != 1);
  in1[0] = in2[in3] - in4->UpperBounds[static_cast<int>(in5) - 1];
  in1[1] = in2[in3 + in2.size(0)] -
           in4->UpperBounds[(static_cast<int>(in5) +
                             in4->UpperBounds.size(0) * stride_0_1) -
                            1];
  in1[2] = in2[in3 + in2.size(0) * 2] -
           in4->UpperBounds[(static_cast<int>(in5) +
                             in4->UpperBounds.size(0) * (stride_0_1 << 1)) -
                            1];
}

//
// Arguments    : double in1[3]
//                const coder::array<double, 2U> &in2
//                int in3
//                const coder::vision::internal::codegen::b_Kdtree *in4
//                double in5
// Return Type  : void
//
static void
c_binary_expand_op(double in1[3], const coder::array<double, 2U> &in2, int in3,
                   const coder::vision::internal::codegen::b_Kdtree *in4,
                   double in5)
{
  int stride_0_1;
  stride_0_1 = (in4->LowerBounds.size(1) != 1);
  in1[0] = in2[in3] - in4->LowerBounds[static_cast<int>(in5) - 1];
  in1[1] = in2[in3 + in2.size(0)] -
           in4->LowerBounds[(static_cast<int>(in5) +
                             in4->LowerBounds.size(0) * stride_0_1) -
                            1];
  in1[2] = in2[in3 + in2.size(0) * 2] -
           in4->LowerBounds[(static_cast<int>(in5) +
                             in4->LowerBounds.size(0) * (stride_0_1 << 1)) -
                            1];
}

//
// Arguments    : const double queryPt[3]
//                const ::coder::array<double, 2U> &lowBounds
//                const ::coder::array<double, 2U> &upBounds
//                double radius
//                double nDims
// Return Type  : bool
//
namespace coder {
namespace vision {
namespace internal {
namespace codegen {
bool Kdtree::boundsOverlapBall(const double queryPt[3],
                               const ::coder::array<double, 2U> &lowBounds,
                               const ::coder::array<double, 2U> &upBounds,
                               double radius, double nDims)
{
  double sumDist;
  int c;
  bool exitg1;
  bool isBoundsOverlapBall;
  isBoundsOverlapBall = true;
  sumDist = 0.0;
  c = 0;
  exitg1 = false;
  while ((!exitg1) && (c <= static_cast<int>(nDims) - 1)) {
    if (queryPt[c] < lowBounds[c]) {
      double pRadIn;
      pRadIn = queryPt[c] - lowBounds[c];
      sumDist += pRadIn * pRadIn;
    } else if (queryPt[c] > upBounds[c]) {
      double pRadIn;
      pRadIn = queryPt[c] - upBounds[c];
      sumDist += pRadIn * pRadIn;
    }
    if (sumDist > radius) {
      isBoundsOverlapBall = false;
      exitg1 = true;
    } else {
      c++;
    }
  }
  return isBoundsOverlapBall;
}

//
// Arguments    : const float queryPt[3]
//                const ::coder::array<double, 2U> &lowBounds
//                const ::coder::array<double, 2U> &upBounds
//                float radius
//                double nDims
// Return Type  : bool
//
bool Kdtree::boundsOverlapBall(const float queryPt[3],
                               const ::coder::array<double, 2U> &lowBounds,
                               const ::coder::array<double, 2U> &upBounds,
                               float radius, double nDims)
{
  float sumDist;
  int c;
  bool exitg1;
  bool isBoundsOverlapBall;
  isBoundsOverlapBall = true;
  sumDist = 0.0F;
  c = 0;
  exitg1 = false;
  while ((!exitg1) && (c <= static_cast<int>(nDims) - 1)) {
    if (queryPt[c] < lowBounds[c]) {
      float pRadIn;
      pRadIn = queryPt[c] - static_cast<float>(lowBounds[c]);
      sumDist += pRadIn * pRadIn;
    } else if (queryPt[c] > upBounds[c]) {
      float pRadIn;
      pRadIn = queryPt[c] - static_cast<float>(upBounds[c]);
      sumDist += pRadIn * pRadIn;
    }
    if (sumDist > radius) {
      isBoundsOverlapBall = false;
      exitg1 = true;
    } else {
      c++;
    }
  }
  return isBoundsOverlapBall;
}

//
// Arguments    : const ::coder::array<double, 2U> &inputData
// Return Type  : void
//
void b_Kdtree::buildIndex(const ::coder::array<double, 2U> &inputData)
{
  array<cell_wrap_46, 1U> idxTemp1;
  array<double, 2U> lowerBoundsTemp;
  array<double, 2U> upperBoundsTemp;
  array<double, 1U> cutValTemp;
  array<double, 1U> leftChildTemp;
  array<double, 1U> rightChildTemp;
  array<double, 1U> sx;
  array<int, 2U> notnan;
  array<int, 1U> iidx;
  array<unsigned int, 1U> tempDim;
  array<signed char, 1U> cutDimTemp;
  array<bool, 2U> b_wasnan;
  array<bool, 2U> r;
  array<bool, 2U> wasnan;
  array<bool, 1U> leafNodeTemp;
  double b_temp_tmp;
  double cc;
  unsigned int currentNode;
  int i;
  int i1;
  int loop_ub;
  int m;
  unsigned int nextUnusedNode;
  int nx;
  int unusedNodes;
  bool exitg1;
  bool y;
  r.set_size(inputData.size(0), 3);
  loop_ub = inputData.size(0) * 3;
  for (i = 0; i < loop_ub; i++) {
    r[i] = std::isnan(inputData[i]);
  }
  any(r, leafNodeTemp);
  wasnan.set_size(1, leafNodeTemp.size(0));
  loop_ub = leafNodeTemp.size(0);
  for (i = 0; i < loop_ub; i++) {
    wasnan[i] = leafNodeTemp[i];
  }
  if (inputData.size(0) < 1) {
    notnan.set_size(1, 0);
  } else {
    loop_ub = inputData.size(0);
    notnan.set_size(1, inputData.size(0));
    for (i = 0; i < loop_ub; i++) {
      notnan[i] = i + 1;
    }
  }
  y = false;
  unusedNodes = 1;
  exitg1 = false;
  while ((!exitg1) && (unusedNodes <= wasnan.size(1))) {
    if (wasnan[unusedNodes - 1]) {
      y = true;
      exitg1 = true;
    } else {
      unusedNodes++;
    }
  }
  if (y) {
    b_wasnan.set_size(1, wasnan.size(1));
    loop_ub = wasnan.size(1);
    for (i = 0; i < loop_ub; i++) {
      b_wasnan[i] = !wasnan[i];
    }
    eml_find(b_wasnan, notnan);
    nx = notnan.size(1);
  } else {
    nx = inputData.size(0);
  }
  cc = b_log2(std::fmax(static_cast<double>(nx) / 50.0, 1.0));
  cc = std::ceil(cc);
  m = static_cast<int>(rt_powd_snf(2.0, cc + 1.0) - 1.0);
  cutDimTemp.set_size(m);
  cutValTemp.set_size(m);
  for (i = 0; i < m; i++) {
    cutDimTemp[i] = 0;
    cutValTemp[i] = 0.0;
  }
  lowerBoundsTemp.set_size(m, 3);
  unusedNodes = static_cast<int>(rt_powd_snf(2.0, cc + 1.0) - 1.0) * 3;
  upperBoundsTemp.set_size(m, 3);
  for (i = 0; i < unusedNodes; i++) {
    lowerBoundsTemp[i] = rtMinusInf;
    upperBoundsTemp[i] = rtInf;
  }
  leftChildTemp.set_size(m);
  rightChildTemp.set_size(m);
  leafNodeTemp.set_size(m);
  idxTemp1.set_size(m);
  for (i = 0; i < m; i++) {
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
      double ex[3];
      double temp[3];
      double temp_tmp;
      iidx.set_size(idxTemp1[static_cast<int>(currentNode) - 1].f1.size(1));
      loop_ub = idxTemp1[static_cast<int>(currentNode) - 1].f1.size(1);
      for (i = 0; i < loop_ub; i++) {
        iidx[i] =
            static_cast<int>(idxTemp1[static_cast<int>(currentNode) - 1].f1[i]);
      }
      unusedNodes = iidx.size(0);
      m = iidx.size(0);
      for (loop_ub = 0; loop_ub < 3; loop_ub++) {
        temp_tmp = inputData[(iidx[0] + inputData.size(0) * loop_ub) - 1];
        temp[loop_ub] = temp_tmp;
        for (int c{2}; c <= unusedNodes; c++) {
          cc = inputData[(iidx[c - 1] + inputData.size(0) * loop_ub) - 1];
          if (std::isnan(cc)) {
            y = false;
          } else if (std::isnan(temp[loop_ub])) {
            y = true;
          } else {
            y = (temp[loop_ub] < cc);
          }
          if (y) {
            temp[loop_ub] = cc;
          }
        }
        ex[loop_ub] = temp_tmp;
        for (int c{2}; c <= m; c++) {
          cc = inputData[(iidx[c - 1] + inputData.size(0) * loop_ub) - 1];
          if (std::isnan(cc)) {
            y = false;
          } else if (std::isnan(ex[loop_ub])) {
            y = true;
          } else {
            y = (ex[loop_ub] > cc);
          }
          if (y) {
            ex[loop_ub] = cc;
          }
        }
        temp[loop_ub] -= ex[loop_ub];
      }
      if (!std::isnan(temp[0])) {
        m = 1;
      } else {
        m = 0;
        unusedNodes = 2;
        exitg1 = false;
        while ((!exitg1) && (unusedNodes < 4)) {
          if (!std::isnan(temp[unusedNodes - 1])) {
            m = unusedNodes;
            exitg1 = true;
          } else {
            unusedNodes++;
          }
        }
      }
      if (m == 0) {
        m = 1;
      } else {
        cc = temp[m - 1];
        i = m + 1;
        for (unusedNodes = i; unusedNodes < 4; unusedNodes++) {
          b_temp_tmp = temp[unusedNodes - 1];
          if (cc < b_temp_tmp) {
            cc = b_temp_tmp;
            m = unusedNodes;
          }
        }
      }
      sx.set_size(iidx.size(0));
      loop_ub = iidx.size(0);
      for (i = 0; i < loop_ub; i++) {
        sx[i] = inputData[(iidx[i] + inputData.size(0) * (m - 1)) - 1];
      }
      ::coder::internal::sort(sx, iidx);
      notnan.set_size(1, iidx.size(0));
      loop_ub = iidx.size(0);
      for (i = 0; i < loop_ub; i++) {
        notnan[i] = static_cast<int>(
            idxTemp1[static_cast<int>(currentNode) - 1].f1[iidx[i] - 1]);
      }
      double c_temp_tmp;
      unusedNodes =
          static_cast<int>(std::ceil(static_cast<double>(sx.size(0)) / 2.0));
      cc = (sx[unusedNodes - 1] + sx[unusedNodes]) / 2.0;
      cutDimTemp[static_cast<int>(currentNode) - 1] =
          static_cast<signed char>(m);
      cutValTemp[static_cast<int>(currentNode) - 1] = cc;
      leftChildTemp[static_cast<int>(currentNode) - 1] = nextUnusedNode;
      rightChildTemp[static_cast<int>(currentNode) - 1] =
          static_cast<double>(nextUnusedNode) + 1.0;
      temp_tmp = upperBoundsTemp[static_cast<int>(currentNode) - 1];
      temp[0] = temp_tmp;
      b_temp_tmp = upperBoundsTemp[(static_cast<int>(currentNode) +
                                    upperBoundsTemp.size(0)) -
                                   1];
      temp[1] = b_temp_tmp;
      c_temp_tmp = upperBoundsTemp[(static_cast<int>(currentNode) +
                                    upperBoundsTemp.size(0) * 2) -
                                   1];
      temp[2] = c_temp_tmp;
      upperBoundsTemp[static_cast<int>(static_cast<double>(nextUnusedNode) +
                                       1.0) -
                      1] = temp_tmp;
      upperBoundsTemp[(static_cast<int>(static_cast<double>(nextUnusedNode) +
                                        1.0) +
                       upperBoundsTemp.size(0)) -
                      1] = b_temp_tmp;
      upperBoundsTemp[(static_cast<int>(static_cast<double>(nextUnusedNode) +
                                        1.0) +
                       upperBoundsTemp.size(0) * 2) -
                      1] = c_temp_tmp;
      temp[m - 1] = cc;
      upperBoundsTemp[static_cast<int>(nextUnusedNode) - 1] = temp[0];
      temp_tmp = lowerBoundsTemp[static_cast<int>(currentNode) - 1];
      temp[0] = temp_tmp;
      upperBoundsTemp[(static_cast<int>(nextUnusedNode) +
                       upperBoundsTemp.size(0)) -
                      1] = temp[1];
      b_temp_tmp = lowerBoundsTemp[(static_cast<int>(currentNode) +
                                    lowerBoundsTemp.size(0)) -
                                   1];
      temp[1] = b_temp_tmp;
      upperBoundsTemp[(static_cast<int>(nextUnusedNode) +
                       upperBoundsTemp.size(0) * 2) -
                      1] = temp[2];
      c_temp_tmp = lowerBoundsTemp[(static_cast<int>(currentNode) +
                                    lowerBoundsTemp.size(0) * 2) -
                                   1];
      temp[2] = c_temp_tmp;
      lowerBoundsTemp[static_cast<int>(nextUnusedNode) - 1] = temp_tmp;
      lowerBoundsTemp[(static_cast<int>(nextUnusedNode) +
                       lowerBoundsTemp.size(0)) -
                      1] = b_temp_tmp;
      lowerBoundsTemp[(static_cast<int>(nextUnusedNode) +
                       lowerBoundsTemp.size(0) * 2) -
                      1] = c_temp_tmp;
      temp[m - 1] = cc;
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
  m = static_cast<int>(nextUnusedNode) - 1;
  tempDim.set_size(m);
  IdxAll.set_size(inputData.size(0));
  loop_ub = inputData.size(0);
  for (i = 0; i < loop_ub; i++) {
    IdxAll[i] = 0U;
  }
  cc = 1.0;
  for (int c{0}; c <= unusedNodes - 2; c++) {
    tempDim[c] = static_cast<unsigned int>(idxTemp1[c].f1.size(1));
    if (idxTemp1[c].f1.size(1) > 0) {
      b_temp_tmp = (cc + static_cast<double>(idxTemp1[c].f1.size(1))) - 1.0;
      if (cc > b_temp_tmp) {
        i = -1;
        i1 = 0;
      } else {
        i = static_cast<int>(cc) - 2;
        i1 = static_cast<int>(b_temp_tmp);
      }
      loop_ub = (i1 - i) - 1;
      for (i1 = 0; i1 < loop_ub; i1++) {
        b_temp_tmp = idxTemp1[c].f1[i1];
        if (b_temp_tmp >= 0.0) {
          currentNode = static_cast<unsigned int>(b_temp_tmp);
        } else {
          currentNode = 0U;
        }
        IdxAll[(i + i1) + 1] = currentNode;
      }
      cc += static_cast<double>(tempDim[c]);
    }
  }
  InputData.set_size(inputData.size(0), 3);
  loop_ub = inputData.size(0) * 3;
  for (i = 0; i < loop_ub; i++) {
    InputData[i] = inputData[i];
  }
  CutDim.set_size(m, 1);
  CutVal.set_size(m, 1);
  for (i = 0; i < m; i++) {
    CutDim[i] = cutDimTemp[i];
    CutVal[i] = cutValTemp[i];
  }
  LowerBounds.set_size(m, 3);
  UpperBounds.set_size(m, 3);
  for (i = 0; i < 3; i++) {
    for (i1 = 0; i1 < m; i1++) {
      LowerBounds[i1 + LowerBounds.size(0) * i] =
          lowerBoundsTemp[i1 + lowerBoundsTemp.size(0) * i];
      UpperBounds[i1 + UpperBounds.size(0) * i] =
          upperBoundsTemp[i1 + upperBoundsTemp.size(0) * i];
    }
  }
  IdxDim.set_size(tempDim.size(0));
  loop_ub = tempDim.size(0);
  for (i = 0; i < loop_ub; i++) {
    IdxDim[i] = tempDim[i];
  }
  LeftChild.set_size(m, 1);
  RightChild.set_size(m, 1);
  LeafNode.set_size(m, 1);
  for (i = 0; i < m; i++) {
    LeftChild[i] = leftChildTemp[i];
    RightChild[i] = rightChildTemp[i];
    LeafNode[i] = leafNodeTemp[i];
  }
  NxNoNaN = nx;
}

//
// Arguments    : const ::coder::array<float, 2U> &inputData
// Return Type  : void
//
void c_Kdtree::buildIndex(const ::coder::array<float, 2U> &inputData)
{
  array<cell_wrap_46, 1U> idxTemp1;
  array<double, 2U> lowerBoundsTemp;
  array<double, 2U> upperBoundsTemp;
  array<double, 1U> cutValTemp;
  array<double, 1U> leftChildTemp;
  array<double, 1U> rightChildTemp;
  array<float, 2U> b_inputData;
  array<float, 1U> sx;
  array<int, 2U> notnan;
  array<int, 1U> cutDimTemp;
  array<int, 1U> iidx;
  array<unsigned int, 1U> tempDim;
  array<bool, 2U> b_wasnan;
  array<bool, 2U> r;
  array<bool, 2U> wasnan;
  array<bool, 1U> leafNodeTemp;
  double cc;
  double temp_tmp;
  float p;
  unsigned int currentNode;
  int half;
  int i;
  int i1;
  int loop_ub;
  unsigned int nextUnusedNode;
  int nx;
  int unusedNodes;
  bool exitg1;
  bool y;
  r.set_size(inputData.size(0), 3);
  loop_ub = inputData.size(0) * 3;
  for (i = 0; i < loop_ub; i++) {
    r[i] = std::isnan(inputData[i]);
  }
  any(r, leafNodeTemp);
  wasnan.set_size(1, leafNodeTemp.size(0));
  loop_ub = leafNodeTemp.size(0);
  for (i = 0; i < loop_ub; i++) {
    wasnan[i] = leafNodeTemp[i];
  }
  if (inputData.size(0) < 1) {
    notnan.set_size(1, 0);
  } else {
    loop_ub = inputData.size(0);
    notnan.set_size(1, inputData.size(0));
    for (i = 0; i < loop_ub; i++) {
      notnan[i] = i + 1;
    }
  }
  y = false;
  unusedNodes = 1;
  exitg1 = false;
  while ((!exitg1) && (unusedNodes <= wasnan.size(1))) {
    if (wasnan[unusedNodes - 1]) {
      y = true;
      exitg1 = true;
    } else {
      unusedNodes++;
    }
  }
  if (y) {
    b_wasnan.set_size(1, wasnan.size(1));
    loop_ub = wasnan.size(1);
    for (i = 0; i < loop_ub; i++) {
      b_wasnan[i] = !wasnan[i];
    }
    eml_find(b_wasnan, notnan);
    nx = notnan.size(1);
  } else {
    nx = inputData.size(0);
  }
  cc = b_log2(std::fmax(static_cast<double>(nx) / 50.0, 1.0));
  cc = std::ceil(cc);
  half = static_cast<int>(rt_powd_snf(2.0, cc + 1.0) - 1.0);
  cutDimTemp.set_size(half);
  cutValTemp.set_size(half);
  for (i = 0; i < half; i++) {
    cutDimTemp[i] = 0;
    cutValTemp[i] = 0.0;
  }
  lowerBoundsTemp.set_size(half, 3);
  unusedNodes = static_cast<int>(rt_powd_snf(2.0, cc + 1.0) - 1.0) * 3;
  upperBoundsTemp.set_size(half, 3);
  for (i = 0; i < unusedNodes; i++) {
    lowerBoundsTemp[i] = rtMinusInf;
    upperBoundsTemp[i] = rtInf;
  }
  leftChildTemp.set_size(half);
  rightChildTemp.set_size(half);
  leafNodeTemp.set_size(half);
  idxTemp1.set_size(half);
  for (i = 0; i < half; i++) {
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
      float fv[3];
      iidx.set_size(idxTemp1[static_cast<int>(currentNode) - 1].f1.size(1));
      loop_ub = idxTemp1[static_cast<int>(currentNode) - 1].f1.size(1);
      for (i = 0; i < loop_ub; i++) {
        iidx[i] =
            static_cast<int>(idxTemp1[static_cast<int>(currentNode) - 1].f1[i]);
      }
      b_inputData.set_size(iidx.size(0), 3);
      loop_ub = iidx.size(0);
      for (i = 0; i < 3; i++) {
        for (i1 = 0; i1 < loop_ub; i1++) {
          b_inputData[i1 + b_inputData.size(0) * i] =
              inputData[(iidx[i1] + inputData.size(0) * i) - 1];
        }
      }
      ::coder::internal::maximum(b_inputData, fv);
      b_inputData.set_size(iidx.size(0), 3);
      loop_ub = iidx.size(0);
      for (i = 0; i < 3; i++) {
        for (i1 = 0; i1 < loop_ub; i1++) {
          b_inputData[i1 + b_inputData.size(0) * i] =
              inputData[(iidx[i1] + inputData.size(0) * i) - 1];
        }
      }
      float fv1[3];
      ::coder::internal::minimum(b_inputData, fv1);
      fv[0] -= fv1[0];
      fv[1] -= fv1[1];
      fv[2] -= fv1[2];
      ::coder::internal::maximum(fv, &p, &unusedNodes);
      sx.set_size(iidx.size(0));
      loop_ub = iidx.size(0);
      for (i = 0; i < loop_ub; i++) {
        sx[i] =
            inputData[(iidx[i] + inputData.size(0) * (unusedNodes - 1)) - 1];
      }
      ::coder::internal::sort(sx, iidx);
      notnan.set_size(1, iidx.size(0));
      loop_ub = iidx.size(0);
      for (i = 0; i < loop_ub; i++) {
        notnan[i] = static_cast<int>(
            idxTemp1[static_cast<int>(currentNode) - 1].f1[iidx[i] - 1]);
      }
      double temp[3];
      double b_temp_tmp;
      half = static_cast<int>(std::ceil(static_cast<double>(sx.size(0)) / 2.0));
      p = (sx[half - 1] + sx[half]) / 2.0F;
      cutDimTemp[static_cast<int>(currentNode) - 1] = unusedNodes;
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
      temp[unusedNodes - 1] = p;
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
      temp[unusedNodes - 1] = p;
      lowerBoundsTemp[static_cast<int>(nextUnusedNode)] = temp[0];
      lowerBoundsTemp[static_cast<int>(nextUnusedNode) +
                      lowerBoundsTemp.size(0)] = temp[1];
      lowerBoundsTemp[static_cast<int>(nextUnusedNode) +
                      lowerBoundsTemp.size(0) * 2] = temp[2];
      idxTemp1[static_cast<int>(currentNode) - 1].f1.set_size(1, 0);
      idxTemp1[static_cast<int>(nextUnusedNode) - 1].f1.set_size(1, half);
      for (i = 0; i < half; i++) {
        idxTemp1[static_cast<int>(nextUnusedNode) - 1].f1[i] = notnan[i];
      }
      if (half + 1 > iidx.size(0)) {
        half = 0;
        i = 0;
      } else {
        i = notnan.size(1);
      }
      loop_ub = i - half;
      idxTemp1[static_cast<int>(nextUnusedNode)].f1.set_size(1, loop_ub);
      for (i = 0; i < loop_ub; i++) {
        idxTemp1[static_cast<int>(nextUnusedNode)].f1[i] = notnan[half + i];
      }
      nextUnusedNode += 2U;
    }
    currentNode++;
  }
  unusedNodes = static_cast<int>(nextUnusedNode);
  half = static_cast<int>(nextUnusedNode) - 1;
  tempDim.set_size(half);
  IdxAll.set_size(inputData.size(0));
  loop_ub = inputData.size(0);
  for (i = 0; i < loop_ub; i++) {
    IdxAll[i] = 0U;
  }
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
        temp_tmp = idxTemp1[c].f1[i1];
        if (temp_tmp >= 0.0) {
          currentNode = static_cast<unsigned int>(temp_tmp);
        } else {
          currentNode = 0U;
        }
        IdxAll[(i + i1) + 1] = currentNode;
      }
      cc += static_cast<double>(tempDim[c]);
    }
  }
  InputData.set_size(inputData.size(0), 3);
  loop_ub = inputData.size(0) * 3;
  for (i = 0; i < loop_ub; i++) {
    InputData[i] = inputData[i];
  }
  CutDim.set_size(half, 1);
  CutVal.set_size(half, 1);
  for (i = 0; i < half; i++) {
    CutDim[i] = cutDimTemp[i];
    CutVal[i] = cutValTemp[i];
  }
  LowerBounds.set_size(half, 3);
  UpperBounds.set_size(half, 3);
  for (i = 0; i < 3; i++) {
    for (i1 = 0; i1 < half; i1++) {
      LowerBounds[i1 + LowerBounds.size(0) * i] =
          lowerBoundsTemp[i1 + lowerBoundsTemp.size(0) * i];
      UpperBounds[i1 + UpperBounds.size(0) * i] =
          upperBoundsTemp[i1 + upperBoundsTemp.size(0) * i];
    }
  }
  IdxDim.set_size(tempDim.size(0));
  loop_ub = tempDim.size(0);
  for (i = 0; i < loop_ub; i++) {
    IdxDim[i] = tempDim[i];
  }
  LeftChild.set_size(half, 1);
  RightChild.set_size(half, 1);
  LeafNode.set_size(half, 1);
  for (i = 0; i < half; i++) {
    LeftChild[i] = leftChildTemp[i];
    RightChild[i] = rightChildTemp[i];
    LeafNode[i] = leafNodeTemp[i];
  }
  NxNoNaN = nx;
}

//
// Arguments    : const ::coder::array<unsigned int, 1U> &idxAll
//                const ::coder::array<double, 1U> &idxDim
//                double this_node
//                ::coder::array<unsigned int, 1U> &nodeIdxThis
// Return Type  : void
//
void Kdtree::getNodeFromArray(const ::coder::array<unsigned int, 1U> &idxAll,
                              const ::coder::array<double, 1U> &idxDim,
                              double this_node,
                              ::coder::array<unsigned int, 1U> &nodeIdxThis)
{
  array<double, 1U> b_idxDim;
  array<unsigned int, 2U> r;
  double d;
  int loop_ub_tmp;
  loop_ub_tmp = static_cast<int>(this_node) - 1;
  d = idxDim[static_cast<int>(this_node) - 1];
  if (d == 0.0) {
    nodeIdxThis.set_size(0);
  } else if (this_node == 1.0) {
    if (idxDim[0] < 1.0) {
      loop_ub_tmp = 0;
    } else {
      loop_ub_tmp = static_cast<int>(idxDim[0]);
    }
    nodeIdxThis.set_size(loop_ub_tmp);
    for (int i{0}; i < loop_ub_tmp; i++) {
      nodeIdxThis[i] = idxAll[i];
    }
  } else {
    double nIdx;
    b_idxDim.set_size(loop_ub_tmp);
    for (int i{0}; i < loop_ub_tmp; i++) {
      b_idxDim[i] = idxDim[i];
    }
    nIdx = blockedSummation(b_idxDim, static_cast<int>(this_node - 1.0));
    r.set_size(1, static_cast<int>(d - 1.0) + 1);
    loop_ub_tmp =
        static_cast<int>(idxDim[static_cast<int>(this_node) - 1] - 1.0);
    for (int i{0}; i <= loop_ub_tmp; i++) {
      r[i] = static_cast<unsigned int>(nIdx + (static_cast<double>(i) + 1.0));
    }
    nodeIdxThis.set_size(r.size(1));
    loop_ub_tmp = r.size(1);
    for (int i{0}; i < loop_ub_tmp; i++) {
      nodeIdxThis[i] = idxAll[static_cast<int>(r[i]) - 1];
    }
  }
}

//
// Arguments    : void
// Return Type  : b_Kdtree *
//
b_Kdtree *b_Kdtree::init()
{
  b_Kdtree *this_;
  this_ = this;
  this_->InputData.set_size(0, 0);
  this_->NxNoNaN = 0.0;
  this_->CutDim.set_size(0, 0);
  this_->CutVal.set_size(0, 0);
  this_->LowerBounds.set_size(0, 0);
  this_->UpperBounds.set_size(0, 0);
  this_->LeftChild.set_size(0, 0);
  this_->RightChild.set_size(0, 0);
  this_->LeafNode.set_size(0, 0);
  this_->IdxAll.set_size(0);
  this_->IdxDim.set_size(0);
  this_->IsIndexed = false;
  return this_;
}

//
// Arguments    : void
// Return Type  : c_Kdtree *
//
c_Kdtree *c_Kdtree::init()
{
  c_Kdtree *this_;
  this_ = this;
  this_->InputData.set_size(0, 0);
  this_->NxNoNaN = 0.0;
  this_->CutDim.set_size(0, 0);
  this_->CutVal.set_size(0, 0);
  this_->LowerBounds.set_size(0, 0);
  this_->UpperBounds.set_size(0, 0);
  this_->LeftChild.set_size(0, 0);
  this_->RightChild.set_size(0, 0);
  this_->LeafNode.set_size(0, 0);
  this_->IdxAll.set_size(0);
  this_->IdxDim.set_size(0);
  this_->IsIndexed = false;
  return this_;
}

//
// Arguments    : const ::coder::array<double, 2U> &queryPoints
//                double K
//                ::coder::array<unsigned int, 2U> &indices
//                ::coder::array<double, 2U> &dists
//                ::coder::array<unsigned int, 1U> &valid
// Return Type  : void
//
void b_Kdtree::knnSearch(const ::coder::array<double, 2U> &queryPoints,
                         double K, ::coder::array<unsigned int, 2U> &indices,
                         ::coder::array<double, 2U> &dists,
                         ::coder::array<unsigned int, 1U> &valid) const
{
  array<double, 2U> b_this;
  array<double, 2U> c_this;
  array<double, 1U> d_this;
  array<double, 1U> nodeStack;
  array<int, 2U> noNanCol;
  array<unsigned int, 1U> r1;
  array<int, 1U> r2;
  array<bool, 2U> r;
  array<bool, 1U> wasNaNY;
  r_struct_T pq;
  double b_pRadIn[3];
  double pRadIn[3];
  double startNode;
  int i;
  int yk;
  r.set_size(queryPoints.size(0), 3);
  yk = queryPoints.size(0) * 3;
  for (i = 0; i < yk; i++) {
    r[i] = std::isnan(queryPoints[i]);
  }
  any(r, wasNaNY);
  startNode = std::fmin(K, static_cast<double>(InputData.size(0)));
  if ((queryPoints.size(0) == 0) || (static_cast<int>(startNode) == 0)) {
    indices.set_size(static_cast<int>(startNode), queryPoints.size(0));
    yk = static_cast<int>(startNode) * queryPoints.size(0);
    for (i = 0; i < yk; i++) {
      indices[i] = 0U;
    }
    dists.set_size(static_cast<int>(startNode), queryPoints.size(0));
    yk = static_cast<int>(startNode) * queryPoints.size(0);
    for (i = 0; i < yk; i++) {
      dists[i] = 0.0;
    }
    valid.set_size(queryPoints.size(0));
    yk = queryPoints.size(0);
    for (i = 0; i < yk; i++) {
      valid[i] = 0U;
    }
  } else {
    double currentNode;
    int numNN;
    currentNode = NxNoNaN;
    if ((!std::isnan(currentNode)) &&
        (static_cast<int>(startNode) > currentNode)) {
      numNN = static_cast<int>(currentNode);
    } else {
      numNN = static_cast<int>(startNode);
    }
    if (numNN > 0) {
      indices.set_size(static_cast<int>(startNode), queryPoints.size(0));
      yk = static_cast<int>(startNode) * queryPoints.size(0);
      for (i = 0; i < yk; i++) {
        indices[i] = 0U;
      }
      dists.set_size(static_cast<int>(startNode), queryPoints.size(0));
      yk = static_cast<int>(startNode) * queryPoints.size(0);
      for (i = 0; i < yk; i++) {
        dists[i] = 0.0;
      }
      valid.set_size(queryPoints.size(0));
      yk = queryPoints.size(0);
      for (i = 0; i < yk; i++) {
        valid[i] = 0U;
      }
      noNanCol.set_size(1, numNN);
      noNanCol[0] = 1;
      yk = 1;
      for (int k{2}; k <= numNN; k++) {
        yk++;
        noNanCol[k - 1] = yk;
      }
      i = queryPoints.size(0);
      for (int j{0}; j < i; j++) {
        if (!wasNaNY[j]) {
          double b_queryPoints[3];
          int nxout;
          bool ballIsWithinBounds;
          startNode = 1.0;
          while (!LeafNode[static_cast<int>(startNode) - 1]) {
            if (queryPoints[j +
                            queryPoints.size(0) *
                                (static_cast<int>(
                                     CutDim[static_cast<int>(startNode) - 1]) -
                                 1)] <=
                CutVal[static_cast<int>(startNode) - 1]) {
              startNode = LeftChild[static_cast<int>(startNode) - 1];
            } else {
              startNode = RightChild[static_cast<int>(startNode) - 1];
            }
          }
          pq.D.set_size(0);
          pq.b_I.set_size(0);
          b_queryPoints[0] = queryPoints[j];
          b_queryPoints[1] = queryPoints[j + queryPoints.size(0)];
          b_queryPoints[2] = queryPoints[j + queryPoints.size(0) * 2];
          Kdtree::getNodeFromArray(IdxAll, IdxDim, startNode, r1);
          Kdtree::searchNode(InputData, b_queryPoints, r1, numNN, &pq);
          if (pq.D.size(0) != 0) {
            if (LowerBounds.size(1) == 3) {
              pRadIn[0] =
                  queryPoints[j] - LowerBounds[static_cast<int>(startNode) - 1];
              pRadIn[1] = queryPoints[j + queryPoints.size(0)] -
                          LowerBounds[(static_cast<int>(startNode) +
                                       LowerBounds.size(0)) -
                                      1];
              pRadIn[2] = queryPoints[j + queryPoints.size(0) * 2] -
                          LowerBounds[(static_cast<int>(startNode) +
                                       LowerBounds.size(0) * 2) -
                                      1];
            } else {
              c_binary_expand_op(pRadIn, queryPoints, j, this, startNode);
            }
            if (UpperBounds.size(1) == 3) {
              b_pRadIn[0] =
                  queryPoints[j] - UpperBounds[static_cast<int>(startNode) - 1];
              b_pRadIn[1] = queryPoints[j + queryPoints.size(0)] -
                            UpperBounds[(static_cast<int>(startNode) +
                                         UpperBounds.size(0)) -
                                        1];
              b_pRadIn[2] = queryPoints[j + queryPoints.size(0) * 2] -
                            UpperBounds[(static_cast<int>(startNode) +
                                         UpperBounds.size(0) * 2) -
                                        1];
            } else {
              b_binary_expand_op(b_pRadIn, queryPoints, j, this, startNode);
            }
            b_queryPoints[0] = pRadIn[0] * pRadIn[0];
            b_queryPoints[1] = pRadIn[1] * pRadIn[1];
            b_queryPoints[2] = pRadIn[2] * pRadIn[2];
            if (::coder::internal::minimum(b_queryPoints) <=
                pq.D[pq.D.size(0) - 1]) {
              ballIsWithinBounds = false;
            } else {
              b_queryPoints[0] = b_pRadIn[0] * b_pRadIn[0];
              b_queryPoints[1] = b_pRadIn[1] * b_pRadIn[1];
              b_queryPoints[2] = b_pRadIn[2] * b_pRadIn[2];
              if (::coder::internal::minimum(b_queryPoints) <=
                  pq.D[pq.D.size(0) - 1]) {
                ballIsWithinBounds = false;
              } else {
                ballIsWithinBounds = true;
              }
            }
          } else {
            ballIsWithinBounds = false;
          }
          if ((pq.D.size(0) != numNN) || (!ballIsWithinBounds)) {
            nodeStack.set_size(1);
            nodeStack[0] = 1.0;
            while (nodeStack.size(0) != 0) {
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
              b_queryPoints[0] = queryPoints[j];
              b_queryPoints[1] = queryPoints[j + queryPoints.size(0)];
              b_queryPoints[2] = queryPoints[j + queryPoints.size(0) * 2];
              yk = LowerBounds.size(1);
              b_this.set_size(1, yk);
              for (nxout = 0; nxout < yk; nxout++) {
                b_this[nxout] = LowerBounds[(static_cast<int>(currentNode) +
                                             LowerBounds.size(0) * nxout) -
                                            1];
              }
              yk = UpperBounds.size(1);
              c_this.set_size(1, yk);
              for (nxout = 0; nxout < yk; nxout++) {
                c_this[nxout] = UpperBounds[(static_cast<int>(currentNode) +
                                             UpperBounds.size(0) * nxout) -
                                            1];
              }
              if ((pq.D.size(0) < numNN) ||
                  Kdtree::boundsOverlapBall(
                      b_queryPoints, b_this, c_this, pq.D[pq.D.size(0) - 1],
                      static_cast<double>(InputData.size(1)))) {
                if (!LeafNode[static_cast<int>(currentNode) - 1]) {
                  if (queryPoints
                          [j +
                           queryPoints.size(0) *
                               (static_cast<int>(
                                    CutDim[static_cast<int>(currentNode) - 1]) -
                                1)] <=
                      CutVal[static_cast<int>(currentNode) - 1]) {
                    d_this.set_size(nodeStack.size(0) + 2);
                    d_this[0] = LeftChild[static_cast<int>(currentNode) - 1];
                    d_this[1] = RightChild[static_cast<int>(currentNode) - 1];
                    yk = nodeStack.size(0);
                    for (nxout = 0; nxout < yk; nxout++) {
                      d_this[nxout + 2] = nodeStack[nxout];
                    }
                    nodeStack.set_size(d_this.size(0));
                    yk = d_this.size(0);
                    for (nxout = 0; nxout < yk; nxout++) {
                      nodeStack[nxout] = d_this[nxout];
                    }
                  } else {
                    d_this.set_size(nodeStack.size(0) + 2);
                    d_this[0] = RightChild[static_cast<int>(currentNode) - 1];
                    d_this[1] = LeftChild[static_cast<int>(currentNode) - 1];
                    yk = nodeStack.size(0);
                    for (nxout = 0; nxout < yk; nxout++) {
                      d_this[nxout + 2] = nodeStack[nxout];
                    }
                    nodeStack.set_size(d_this.size(0));
                    yk = d_this.size(0);
                    for (nxout = 0; nxout < yk; nxout++) {
                      nodeStack[nxout] = d_this[nxout];
                    }
                  }
                } else if (currentNode != startNode) {
                  b_queryPoints[0] = queryPoints[j];
                  b_queryPoints[1] = queryPoints[j + queryPoints.size(0)];
                  b_queryPoints[2] = queryPoints[j + queryPoints.size(0) * 2];
                  Kdtree::getNodeFromArray(IdxAll, IdxDim, currentNode, r1);
                  Kdtree::searchNode(InputData, b_queryPoints, r1, numNN, &pq);
                }
              }
            }
          }
          yk = noNanCol.size(1);
          r2.set_size(noNanCol.size(1));
          for (nxout = 0; nxout < yk; nxout++) {
            r2[nxout] = noNanCol[nxout];
          }
          for (nxout = 0; nxout < yk; nxout++) {
            indices[(r2[nxout] + indices.size(0) * j) - 1] = pq.b_I[nxout];
            dists[(r2[nxout] + dists.size(0) * j) - 1] = pq.D[nxout];
          }
          valid[j] = static_cast<unsigned int>(noNanCol.size(1));
        }
      }
    } else {
      indices.set_size(0, queryPoints.size(0));
      dists.set_size(0, queryPoints.size(0));
      valid.set_size(queryPoints.size(0));
      yk = queryPoints.size(0);
      for (i = 0; i < yk; i++) {
        valid[i] = 0U;
      }
    }
  }
}

//
// Arguments    : const ::coder::array<float, 2U> &X
//                const float queryPt[3]
//                const ::coder::array<unsigned int, 1U> &nodeIdxStart
//                int numNN
//                q_struct_T *pq
// Return Type  : void
//
void Kdtree::searchNode(const ::coder::array<float, 2U> &X,
                        const float queryPt[3],
                        const ::coder::array<unsigned int, 1U> &nodeIdxStart,
                        int numNN, q_struct_T *pq)
{
  array<float, 2U> diffAllDim;
  array<float, 1U> aDistOut;
  array<float, 1U> r;
  array<int, 1U> iidx;
  array<unsigned int, 1U> r1;
  int acoef;
  int b_acoef;
  int i;
  int varargin_2;
  diffAllDim.set_size(nodeIdxStart.size(0), 3);
  if (nodeIdxStart.size(0) != 0) {
    acoef = (X.size(1) != 1);
    b_acoef = (nodeIdxStart.size(0) != 1);
    for (int k{0}; k < 3; k++) {
      varargin_2 = acoef * k;
      i = diffAllDim.size(0) - 1;
      for (int b_k{0}; b_k <= i; b_k++) {
        diffAllDim[b_k + diffAllDim.size(0) * k] =
            X[(static_cast<int>(nodeIdxStart[b_acoef * b_k]) +
               X.size(0) * varargin_2) -
              1] -
            queryPt[k];
      }
    }
  }
  acoef = diffAllDim.size(0) * 3;
  diffAllDim.set_size(diffAllDim.size(0), 3);
  for (i = 0; i < acoef; i++) {
    diffAllDim[i] = diffAllDim[i] * diffAllDim[i];
  }
  if (diffAllDim.size(0) == 0) {
    aDistOut.set_size(0);
  } else {
    acoef = diffAllDim.size(0);
    aDistOut.set_size(diffAllDim.size(0));
    for (varargin_2 = 0; varargin_2 < acoef; varargin_2++) {
      aDistOut[varargin_2] = diffAllDim[varargin_2];
    }
    for (int k{0}; k < 2; k++) {
      b_acoef = (k + 1) * acoef;
      for (varargin_2 = 0; varargin_2 < acoef; varargin_2++) {
        aDistOut[varargin_2] =
            aDistOut[varargin_2] + diffAllDim[b_acoef + varargin_2];
      }
    }
  }
  ::coder::internal::sort(aDistOut, iidx);
  if (pq->D.size(0) == 0) {
    if (aDistOut.size(0) <= numNN) {
      pq->D.set_size(aDistOut.size(0));
      acoef = aDistOut.size(0);
      for (i = 0; i < acoef; i++) {
        pq->D[i] = aDistOut[i];
      }
      pq->b_I.set_size(iidx.size(0));
      acoef = iidx.size(0);
      for (i = 0; i < acoef; i++) {
        pq->b_I[i] = nodeIdxStart[iidx[i] - 1];
      }
    } else {
      if (numNN < 1) {
        acoef = 0;
      } else {
        acoef = numNN;
      }
      pq->D.set_size(acoef);
      for (i = 0; i < acoef; i++) {
        pq->D[i] = aDistOut[i];
      }
      if (numNN < 1) {
        acoef = 0;
      } else {
        acoef = numNN;
      }
      pq->b_I.set_size(acoef);
      for (i = 0; i < acoef; i++) {
        pq->b_I[i] = nodeIdxStart[iidx[i] - 1];
      }
    }
  } else {
    unsigned int cD1;
    unsigned int cD2;
    unsigned int x;
    bool exitg1;
    cD1 = 1U;
    cD2 = 1U;
    x = static_cast<unsigned int>(pq->D.size(0)) + aDistOut.size(0);
    if (static_cast<double>(x) > numNN) {
      acoef = numNN;
    } else {
      acoef = static_cast<int>(x);
    }
    r.set_size(acoef);
    r1.set_size(acoef);
    for (i = 0; i < acoef; i++) {
      r[i] = 0.0F;
      r1[i] = 0U;
    }
    b_acoef = 0;
    exitg1 = false;
    while ((!exitg1) && (b_acoef <= acoef - 1)) {
      float f;
      float f1;
      f = aDistOut[static_cast<int>(cD2) - 1];
      f1 = pq->D[static_cast<int>(cD1) - 1];
      if (f1 <= f) {
        r[b_acoef] = f1;
        r1[b_acoef] = pq->b_I[static_cast<int>(cD1) - 1];
        cD1++;
        if (cD1 > static_cast<unsigned int>(pq->D.size(0))) {
          i = b_acoef + 2;
          for (int b_k{i}; b_k <= acoef; b_k++) {
            varargin_2 = static_cast<int>((cD2 + b_k) - b_acoef) - 3;
            r[b_k - 1] = aDistOut[varargin_2];
            r1[b_k - 1] = nodeIdxStart[iidx[varargin_2] - 1];
          }
          exitg1 = true;
        } else {
          b_acoef++;
        }
      } else {
        r[b_acoef] = f;
        r1[b_acoef] = nodeIdxStart[iidx[static_cast<int>(cD2) - 1] - 1];
        cD2++;
        if (cD2 > static_cast<unsigned int>(aDistOut.size(0))) {
          i = b_acoef + 2;
          for (int b_k{i}; b_k <= acoef; b_k++) {
            varargin_2 = static_cast<int>((cD1 + b_k) - b_acoef) - 3;
            r[b_k - 1] = pq->D[varargin_2];
            r1[b_k - 1] = pq->b_I[varargin_2];
          }
          exitg1 = true;
        } else {
          b_acoef++;
        }
      }
    }
    pq->D.set_size(r.size(0));
    acoef = r.size(0);
    for (i = 0; i < acoef; i++) {
      pq->D[i] = r[i];
    }
    pq->b_I.set_size(r1.size(0));
    acoef = r1.size(0);
    for (i = 0; i < acoef; i++) {
      pq->b_I[i] = r1[i];
    }
  }
}

//
// Arguments    : const ::coder::array<double, 2U> &X
//                const double queryPt[3]
//                const ::coder::array<unsigned int, 1U> &nodeIdxThis
//                double powRadius
//                r_struct_T *pq
// Return Type  : void
//
void Kdtree::searchNodeRadius(
    const ::coder::array<double, 2U> &X, const double queryPt[3],
    const ::coder::array<unsigned int, 1U> &nodeIdxThis, double powRadius,
    r_struct_T *pq)
{
  array<double, 2U> b_diffAllDim;
  array<double, 2U> diffAllDim;
  array<double, 1U> aDistOut;
  array<double, 1U> distInP;
  array<double, 1U> pqTempD;
  array<int, 2U> r1;
  array<unsigned int, 1U> pqTempI;
  array<unsigned int, 1U> r;
  array<int, 1U> r2;
  array<int, 1U> r3;
  int acoef;
  int b_acoef;
  int b_k;
  unsigned int b_pq;
  int i;
  int nD1;
  diffAllDim.set_size(nodeIdxThis.size(0), 3);
  if (nodeIdxThis.size(0) != 0) {
    acoef = (X.size(1) != 1);
    b_acoef = (nodeIdxThis.size(0) != 1);
    for (int k{0}; k < 3; k++) {
      nD1 = acoef * k;
      i = diffAllDim.size(0) - 1;
      for (b_k = 0; b_k <= i; b_k++) {
        diffAllDim[b_k + diffAllDim.size(0) * k] =
            X[(static_cast<int>(nodeIdxThis[b_acoef * b_k]) + X.size(0) * nD1) -
              1] -
            queryPt[k];
      }
    }
  }
  b_diffAllDim.set_size(diffAllDim.size(0), 3);
  acoef = diffAllDim.size(0) * 3;
  for (i = 0; i < acoef; i++) {
    b_diffAllDim[i] = diffAllDim[i] * diffAllDim[i];
  }
  if (b_diffAllDim.size(0) == 0) {
    aDistOut.set_size(0);
  } else {
    acoef = b_diffAllDim.size(0);
    aDistOut.set_size(b_diffAllDim.size(0));
    for (nD1 = 0; nD1 < acoef; nD1++) {
      aDistOut[nD1] = b_diffAllDim[nD1];
    }
    for (int k{0}; k < 2; k++) {
      b_acoef = (k + 1) * acoef;
      for (nD1 = 0; nD1 < acoef; nD1++) {
        aDistOut[nD1] = aDistOut[nD1] + b_diffAllDim[b_acoef + nD1];
      }
    }
  }
  acoef = aDistOut.size(0);
  distInP.set_size(aDistOut.size(0));
  for (i = 0; i < acoef; i++) {
    distInP[i] = aDistOut[i];
  }
  nD1 = pq->D.size(0);
  b_acoef = distInP.size(0);
  b_k = 0;
  for (int k{0}; k < b_acoef; k++) {
    if (distInP[k] <= powRadius) {
      b_k++;
    }
  }
  pqTempD.set_size(X.size(0));
  acoef = X.size(0);
  for (i = 0; i < acoef; i++) {
    pqTempD[i] = 0.0;
  }
  if (pq->D.size(0) < 1) {
    acoef = 0;
  } else {
    acoef = pq->D.size(0);
  }
  for (i = 0; i < acoef; i++) {
    pqTempD[i] = pq->D[i];
  }
  pqTempI.set_size(X.size(0));
  acoef = X.size(0);
  for (i = 0; i < acoef; i++) {
    pqTempI[i] = 0U;
  }
  if (pq->D.size(0) < 1) {
    acoef = 0;
  } else {
    acoef = pq->D.size(0);
  }
  for (i = 0; i < acoef; i++) {
    pqTempI[i] = pq->b_I[i];
  }
  if (b_k > 0) {
    b_pq = pq->D.size(0) + 1U;
    r.set_size(b_k);
    acoef = b_k - 1;
    for (i = 0; i <= acoef; i++) {
      r[i] = i + b_pq;
    }
    r1.set_size(1, r.size(0));
    acoef = r.size(0);
    for (i = 0; i < acoef; i++) {
      r1[i] = static_cast<int>(r[i]);
    }
    b_acoef = distInP.size(0) - 1;
    acoef = 0;
    for (int k{0}; k <= b_acoef; k++) {
      if (distInP[k] <= powRadius) {
        acoef++;
      }
    }
    r2.set_size(acoef);
    acoef = 0;
    for (int k{0}; k <= b_acoef; k++) {
      if (distInP[k] <= powRadius) {
        r2[acoef] = k + 1;
        acoef++;
      }
    }
    acoef = r1.size(1);
    for (i = 0; i < acoef; i++) {
      pqTempD[r1[i] - 1] = distInP[r2[i] - 1];
    }
    r1.set_size(1, r.size(0));
    acoef = r.size(0);
    for (i = 0; i < acoef; i++) {
      r1[i] = static_cast<int>(r[i]);
    }
    b_acoef = distInP.size(0) - 1;
    acoef = 0;
    for (int k{0}; k <= b_acoef; k++) {
      if (distInP[k] <= powRadius) {
        acoef++;
      }
    }
    r3.set_size(acoef);
    acoef = 0;
    for (int k{0}; k <= b_acoef; k++) {
      if (distInP[k] <= powRadius) {
        r3[acoef] = k + 1;
        acoef++;
      }
    }
    acoef = r1.size(1);
    for (i = 0; i < acoef; i++) {
      pqTempI[r1[i] - 1] = nodeIdxThis[r3[i] - 1];
    }
  }
  b_pq = static_cast<unsigned int>(pq->D.size(0)) + b_k;
  if (b_pq < 1U) {
    acoef = 0;
  } else {
    acoef = static_cast<int>(b_pq);
  }
  pq->D.set_size(acoef);
  for (i = 0; i < acoef; i++) {
    pq->D[i] = pqTempD[i];
  }
  b_pq = static_cast<unsigned int>(nD1) + b_k;
  if (b_pq < 1U) {
    acoef = 0;
  } else {
    acoef = static_cast<int>(b_pq);
  }
  pq->b_I.set_size(acoef);
  for (i = 0; i < acoef; i++) {
    pq->b_I[i] = pqTempI[i];
  }
}

//
// Arguments    : float in1[3]
//                const float in2[22317]
//                int in3
//                const coder::array<double, 2U> &in4
// Return Type  : void
//
} // namespace codegen
} // namespace internal
} // namespace vision
} // namespace coder
void b_binary_expand_op(float in1[3], const float in2[22317], int in3,
                        const coder::array<double, 2U> &in4)
{
  int stride_0_1;
  stride_0_1 = (in4.size(1) != 1);
  in1[0] = in2[in3] - static_cast<float>(in4[0]);
  in1[1] = in2[in3 + 7439] - static_cast<float>(in4[stride_0_1]);
  in1[2] = in2[in3 + 14878] - static_cast<float>(in4[stride_0_1 << 1]);
}

//
// Arguments    : float in1[3]
//                const coder::array<float, 2U> &in2
//                int in3
//                const coder::array<double, 2U> &in4
// Return Type  : void
//
void b_binary_expand_op(float in1[3], const coder::array<float, 2U> &in2,
                        int in3, const coder::array<double, 2U> &in4)
{
  int stride_0_1;
  stride_0_1 = (in4.size(1) != 1);
  in1[0] = in2[in3] - static_cast<float>(in4[0]);
  in1[1] = in2[in3 + in2.size(0)] - static_cast<float>(in4[stride_0_1]);
  in1[2] =
      in2[in3 + in2.size(0) * 2] - static_cast<float>(in4[stride_0_1 << 1]);
}

//
// Arguments    : double in1[3]
//                const double in2[3]
//                const coder::array<double, 2U> &in3
// Return Type  : void
//
void minus(double in1[3], const double in2[3],
           const coder::array<double, 2U> &in3)
{
  int stride_0_1;
  stride_0_1 = (in3.size(1) != 1);
  in1[0] = in2[0] - in3[0];
  in1[1] = in2[1] - in3[stride_0_1];
  in1[2] = in2[2] - in3[stride_0_1 << 1];
}

//
// File trailer for Kdtree.cpp
//
// [EOF]
//
