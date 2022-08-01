//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: pctransform.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 01-Aug-2022 08:26:09
//

// Include Files
#include "pctransform.h"
#include "Kdtree.h"
#include "affine3d.h"
#include "any1.h"
#include "isRigidTransform.h"
#include "minOrMax.h"
#include "mtimes.h"
#include "pointCloud.h"
#include "pointCloudArray.h"
#include "proc_mapping_internal_types.h"
#include "rigid3d.h"
#include "rt_nonfinite.h"
#include "PCANormalCore_api.hpp"
#include "coder_array.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <string.h>

// Function Definitions
//
// Arguments    : const g_pointCloud *ptCloudIn
//                const rigid3d *tform
//                f_pointCloud *iobj_0
// Return Type  : f_pointCloud *
//
namespace coder {
f_pointCloud *pctransform(const g_pointCloud *ptCloudIn, const rigid3d *tform,
                          f_pointCloud *iobj_0)
{
  static e_pointCloud lobj_0;
  static float b_loc[22317];
  static float loc[22317];
  f_pointCloud *ptCloudOut;
  pointclouds::internal::codegen::pc::b_pointCloudArray r;
  vision::internal::codegen::c_Kdtree *b_this;
  array<double, 2U> cutDim;
  array<double, 2U> cutVal;
  array<double, 2U> leftChild;
  array<double, 2U> lowBounds;
  array<double, 2U> rightChild;
  array<double, 2U> upBounds;
  array<double, 1U> c_this;
  array<double, 1U> nodeStack;
  array<float, 2U> nv;
  array<unsigned int, 2U> indices;
  array<int, 2U> noNanCol;
  array<unsigned int, 1U> nodeIdxThis;
  array<int, 1U> r2;
  array<bool, 2U> leafNode;
  q_struct_T r1;
  float T[16];
  float b_pRadIn[3];
  float pRadIn[3];
  unsigned int valid[7439];
  int k;
  int nxout;
  int yk;
  for (k = 0; k < 16; k++) {
    T[k] = static_cast<float>(tform->AffineTform.T[k]);
  }
  lobj_0.matlabCodegenIsDeleted = true;
  for (k = 0; k < 7439; k++) {
    for (nxout = 0; nxout < 3; nxout++) {
      yk = nxout << 2;
      loc[k + 7439 * nxout] = (ptCloudIn->Location[k] * T[yk] +
                               ptCloudIn->Location[k + 7439] * T[yk + 1]) +
                              ptCloudIn->Location[k + 14878] * T[yk + 2];
    }
    loc[k] += T[3];
    loc[k + 7439] += T[7];
    loc[k + 14878] += T[11];
  }
  nv.set_size(0, 0);
  if ((ptCloudIn->Normal.size(0) != 0) && (ptCloudIn->Normal.size(1) != 0)) {
    if (vision::internal::isRigidTransform(T)) {
      float b_T[9];
      for (k = 0; k < 3; k++) {
        yk = k << 2;
        b_T[3 * k] = T[yk];
        b_T[3 * k + 1] = T[yk + 1];
        b_T[3 * k + 2] = T[yk + 2];
      }
      internal::blas::mtimes(ptCloudIn->Normal, b_T, nv);
    } else {
      bool bv[22317];
      bool wasNaNY[7439];
      std::copy(&loc[0], &loc[22317], &lobj_0.Location[0]);
      lobj_0.Color.set_size(0, 0);
      lobj_0.Normal.set_size(0, 0);
      lobj_0.Intensity.set_size(0, 0);
      lobj_0.b_Kdtree = lobj_0._pobj0.init();
      lobj_0.matlabCodegenIsDeleted = false;
      std::copy(&lobj_0.Location[0], &lobj_0.Location[22317], &b_loc[0]);
      lobj_0.buildKdtree();
      b_this = lobj_0.b_Kdtree;
      nv.set_size(b_this->InputData.size(0), b_this->InputData.size(1));
      yk = b_this->InputData.size(0) * b_this->InputData.size(1);
      for (k = 0; k < yk; k++) {
        nv[k] = b_this->InputData[k];
      }
      for (k = 0; k < 22317; k++) {
        bv[k] = std::isnan(b_loc[k]);
      }
      b_any(bv, wasNaNY);
      yk = static_cast<int>(std::fmin(6.0, static_cast<double>(nv.size(0))));
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
          for (k = 0; k < yk; k++) {
            indices[k] = 0U;
          }
          std::memset(&valid[0], 0, 7439U * sizeof(unsigned int));
          noNanCol.set_size(1, numNN);
          noNanCol[0] = 1;
          yk = 1;
          for (k = 2; k <= numNN; k++) {
            yk++;
            noNanCol[k - 1] = yk;
          }
          for (int j{0}; j < 7439; j++) {
            if (!wasNaNY[j]) {
              float c_loc[3];
              float b_loc_tmp;
              float loc_tmp;
              bool ballIsWithinBounds;
              cutDim.set_size(b_this->CutDim.size(0), b_this->CutDim.size(1));
              yk = b_this->CutDim.size(0) * b_this->CutDim.size(1);
              for (k = 0; k < yk; k++) {
                cutDim[k] = b_this->CutDim[k];
              }
              cutVal.set_size(b_this->CutVal.size(0), b_this->CutVal.size(1));
              yk = b_this->CutVal.size(0) * b_this->CutVal.size(1);
              for (k = 0; k < yk; k++) {
                cutVal[k] = b_this->CutVal[k];
              }
              leafNode.set_size(b_this->LeafNode.size(0),
                                b_this->LeafNode.size(1));
              yk = b_this->LeafNode.size(0) * b_this->LeafNode.size(1);
              for (k = 0; k < yk; k++) {
                leafNode[k] = b_this->LeafNode[k];
              }
              leftChild.set_size(b_this->LeftChild.size(0),
                                 b_this->LeftChild.size(1));
              yk = b_this->LeftChild.size(0) * b_this->LeftChild.size(1);
              for (k = 0; k < yk; k++) {
                leftChild[k] = b_this->LeftChild[k];
              }
              rightChild.set_size(b_this->RightChild.size(0),
                                  b_this->RightChild.size(1));
              yk = b_this->RightChild.size(0) * b_this->RightChild.size(1);
              for (k = 0; k < yk; k++) {
                rightChild[k] = b_this->RightChild[k];
              }
              startNode = 1.0;
              while (!leafNode[static_cast<int>(startNode) - 1]) {
                if (b_loc[j +
                          7439 * (static_cast<int>(
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
              r1.D.set_size(0);
              r1.b_I.set_size(0);
              c_loc[0] = b_loc[j];
              loc_tmp = b_loc[j + 7439];
              c_loc[1] = loc_tmp;
              b_loc_tmp = b_loc[j + 14878];
              c_loc[2] = b_loc_tmp;
              vision::internal::codegen::Kdtree::searchNode(
                  nv, c_loc, nodeIdxThis, numNN, &r1);
              if (r1.D.size(0) != 0) {
                yk = b_this->LowerBounds.size(1);
                lowBounds.set_size(1, yk);
                for (k = 0; k < yk; k++) {
                  lowBounds[k] =
                      b_this->LowerBounds[(static_cast<int>(startNode) +
                                           b_this->LowerBounds.size(0) * k) -
                                          1];
                }
                yk = b_this->UpperBounds.size(1);
                upBounds.set_size(1, yk);
                for (k = 0; k < yk; k++) {
                  upBounds[k] =
                      b_this->UpperBounds[(static_cast<int>(startNode) +
                                           b_this->UpperBounds.size(0) * k) -
                                          1];
                }
                if (lowBounds.size(1) == 3) {
                  pRadIn[0] = b_loc[j] - static_cast<float>(lowBounds[0]);
                  pRadIn[1] = loc_tmp - static_cast<float>(lowBounds[1]);
                  pRadIn[2] = b_loc_tmp - static_cast<float>(lowBounds[2]);
                } else {
                  b_binary_expand_op(pRadIn, b_loc, j, lowBounds);
                }
                if (upBounds.size(1) == 3) {
                  b_pRadIn[0] = b_loc[j] - static_cast<float>(upBounds[0]);
                  b_pRadIn[1] = loc_tmp - static_cast<float>(upBounds[1]);
                  b_pRadIn[2] = b_loc_tmp - static_cast<float>(upBounds[2]);
                } else {
                  b_binary_expand_op(b_pRadIn, b_loc, j, upBounds);
                }
                c_loc[0] = pRadIn[0] * pRadIn[0];
                c_loc[1] = pRadIn[1] * pRadIn[1];
                c_loc[2] = pRadIn[2] * pRadIn[2];
                if (internal::minimum(c_loc) <= r1.D[r1.D.size(0) - 1]) {
                  ballIsWithinBounds = false;
                } else {
                  c_loc[0] = b_pRadIn[0] * b_pRadIn[0];
                  c_loc[1] = b_pRadIn[1] * b_pRadIn[1];
                  c_loc[2] = b_pRadIn[2] * b_pRadIn[2];
                  if (internal::minimum(c_loc) <= r1.D[r1.D.size(0) - 1]) {
                    ballIsWithinBounds = false;
                  } else {
                    ballIsWithinBounds = true;
                  }
                }
              } else {
                ballIsWithinBounds = false;
              }
              if ((r1.D.size(0) != numNN) || (!ballIsWithinBounds)) {
                nodeStack.set_size(1);
                nodeStack[0] = 1.0;
                while (nodeStack.size(0) != 0) {
                  double currentNode;
                  currentNode = nodeStack[0];
                  yk = nodeStack.size(0);
                  nxout = nodeStack.size(0) - 1;
                  for (k = 0; k < nxout; k++) {
                    nodeStack[k] = nodeStack[k + 1];
                  }
                  if (nxout < 1) {
                    nxout = 0;
                  } else {
                    nxout = yk - 1;
                  }
                  nodeStack.set_size(nxout);
                  c_loc[0] = b_loc[j];
                  c_loc[1] = loc_tmp;
                  c_loc[2] = b_loc_tmp;
                  yk = b_this->LowerBounds.size(1);
                  lowBounds.set_size(1, yk);
                  for (k = 0; k < yk; k++) {
                    lowBounds[k] =
                        b_this->LowerBounds[(static_cast<int>(currentNode) +
                                             b_this->LowerBounds.size(0) * k) -
                                            1];
                  }
                  yk = b_this->UpperBounds.size(1);
                  upBounds.set_size(1, yk);
                  for (k = 0; k < yk; k++) {
                    upBounds[k] =
                        b_this->UpperBounds[(static_cast<int>(currentNode) +
                                             b_this->UpperBounds.size(0) * k) -
                                            1];
                  }
                  if ((r1.D.size(0) < numNN) ||
                      vision::internal::codegen::Kdtree::boundsOverlapBall(
                          c_loc, lowBounds, upBounds, r1.D[r1.D.size(0) - 1],
                          static_cast<double>(nv.size(1)))) {
                    if (!b_this->LeafNode[static_cast<int>(currentNode) - 1]) {
                      k = static_cast<int>(
                              b_this
                                  ->CutDim[static_cast<int>(currentNode) - 1]) -
                          1;
                      if (b_loc[j + 7439 * k] <=
                          b_this->CutVal[static_cast<int>(currentNode) - 1]) {
                        c_this.set_size(nodeStack.size(0) + 2);
                        c_this[0] =
                            b_this
                                ->LeftChild[static_cast<int>(currentNode) - 1];
                        c_this[1] =
                            b_this
                                ->RightChild[static_cast<int>(currentNode) - 1];
                        yk = nodeStack.size(0);
                        for (k = 0; k < yk; k++) {
                          c_this[k + 2] = nodeStack[k];
                        }
                        nodeStack.set_size(c_this.size(0));
                        yk = c_this.size(0);
                        for (k = 0; k < yk; k++) {
                          nodeStack[k] = c_this[k];
                        }
                      } else {
                        c_this.set_size(nodeStack.size(0) + 2);
                        c_this[0] =
                            b_this
                                ->RightChild[static_cast<int>(currentNode) - 1];
                        c_this[1] =
                            b_this
                                ->LeftChild[static_cast<int>(currentNode) - 1];
                        yk = nodeStack.size(0);
                        for (k = 0; k < yk; k++) {
                          c_this[k + 2] = nodeStack[k];
                        }
                        nodeStack.set_size(c_this.size(0));
                        yk = c_this.size(0);
                        for (k = 0; k < yk; k++) {
                          nodeStack[k] = c_this[k];
                        }
                      }
                    } else if (currentNode != startNode) {
                      vision::internal::codegen::Kdtree::getNodeFromArray(
                          b_this->IdxAll, b_this->IdxDim, currentNode,
                          nodeIdxThis);
                      c_loc[0] = b_loc[j];
                      c_loc[1] = loc_tmp;
                      c_loc[2] = b_loc_tmp;
                      vision::internal::codegen::Kdtree::searchNode(
                          nv, c_loc, nodeIdxThis, numNN, &r1);
                    }
                  }
                }
              }
              yk = noNanCol.size(1);
              r2.set_size(noNanCol.size(1));
              for (k = 0; k < yk; k++) {
                r2[k] = noNanCol[k];
              }
              for (k = 0; k < yk; k++) {
                indices[(r2[k] + indices.size(0) * j) - 1] = r1.b_I[k];
              }
              valid[j] = static_cast<unsigned int>(noNanCol.size(1));
            }
          }
        } else {
          indices.set_size(0, 7439);
          std::memset(&valid[0], 0, 7439U * sizeof(unsigned int));
        }
      }
      nv.set_size(7439, 3);
      PCANormalImpl_single(&b_loc[0], &indices[0], &valid[0], 7439U,
                           static_cast<unsigned int>(indices.size(0)), &nv[0]);
    }
  }
  ptCloudOut = iobj_0;
  for (k = 0; k < 22317; k++) {
    iobj_0->Location[k] = loc[k];
  }
  iobj_0->Color.set_size(ptCloudIn->Color.size(0), ptCloudIn->Color.size(1));
  yk = ptCloudIn->Color.size(0) * ptCloudIn->Color.size(1);
  for (k = 0; k < yk; k++) {
    iobj_0->Color[k] = ptCloudIn->Color[k];
  }
  iobj_0->Normal.set_size(nv.size(0), nv.size(1));
  yk = nv.size(0) * nv.size(1);
  for (k = 0; k < yk; k++) {
    iobj_0->Normal[k] = nv[k];
  }
  iobj_0->Intensity.set_size(ptCloudIn->Intensity.size(0),
                             ptCloudIn->Intensity.size(1));
  yk = ptCloudIn->Intensity.size(0) * ptCloudIn->Intensity.size(1);
  for (k = 0; k < yk; k++) {
    iobj_0->Intensity[k] = ptCloudIn->Intensity[k];
  }
  r.Location.set_size(7439, 3);
  for (k = 0; k < 22317; k++) {
    r.Location[k] = iobj_0->Location[k];
  }
  r.Normal.set_size(iobj_0->Normal.size(0), iobj_0->Normal.size(1));
  yk = iobj_0->Normal.size(0) * iobj_0->Normal.size(1);
  for (k = 0; k < yk; k++) {
    r.Normal[k] = iobj_0->Normal[k];
  }
  r.Color.set_size(iobj_0->Color.size(0), iobj_0->Color.size(1));
  yk = iobj_0->Color.size(0) * iobj_0->Color.size(1);
  for (k = 0; k < yk; k++) {
    r.Color[k] = iobj_0->Color[k];
  }
  r.Intensity.set_size(iobj_0->Intensity.size(0), iobj_0->Intensity.size(1));
  yk = iobj_0->Intensity.size(0) * iobj_0->Intensity.size(1);
  for (k = 0; k < yk; k++) {
    r.Intensity[k] = iobj_0->Intensity[k];
  }
  iobj_0->PointCloudArrayData.set_size(1, 1);
  iobj_0->PointCloudArrayData[0] = r;
  iobj_0->b_Kdtree = &iobj_0->_pobj0;
  iobj_0->matlabCodegenIsDeleted = false;
  return ptCloudOut;
}

} // namespace coder

//
// File trailer for pctransform.cpp
//
// [EOF]
//
