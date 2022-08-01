//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Kdtree.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 01-Aug-2022 08:26:09
//

#ifndef KDTREE_H
#define KDTREE_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct r_struct_T;

struct q_struct_T;

// Type Definitions
namespace coder {
namespace vision {
namespace internal {
namespace codegen {
class Kdtree {
public:
  static void getNodeFromArray(const ::coder::array<unsigned int, 1U> &idxAll,
                               const ::coder::array<double, 1U> &idxDim,
                               double this_node,
                               ::coder::array<unsigned int, 1U> &nodeIdxThis);
  static bool boundsOverlapBall(const double queryPt[3],
                                const ::coder::array<double, 2U> &lowBounds,
                                const ::coder::array<double, 2U> &upBounds,
                                double radius, double nDims);
  static bool boundsOverlapBall(const float queryPt[3],
                                const ::coder::array<double, 2U> &lowBounds,
                                const ::coder::array<double, 2U> &upBounds,
                                float radius, double nDims);
  static void searchNode(const ::coder::array<double, 2U> &X,
                         const double queryPt[3],
                         const ::coder::array<unsigned int, 1U> &nodeIdxStart,
                         int numNN, r_struct_T *pq);
  static void searchNode(const ::coder::array<float, 2U> &X,
                         const float queryPt[3],
                         const ::coder::array<unsigned int, 1U> &nodeIdxStart,
                         int numNN, q_struct_T *pq);
  static void
  searchNodeRadius(const ::coder::array<double, 2U> &X, const double queryPt[3],
                   const ::coder::array<unsigned int, 1U> &nodeIdxThis,
                   double powRadius, r_struct_T *pq);
};

class b_Kdtree {
public:
  void buildIndex(const ::coder::array<double, 2U> &inputData);
  void knnSearch(const ::coder::array<double, 2U> &queryPoints, double K,
                 ::coder::array<unsigned int, 2U> &indices,
                 ::coder::array<double, 2U> &dists,
                 ::coder::array<unsigned int, 1U> &valid) const;
  b_Kdtree *init();
  array<double, 2U> InputData;
  array<double, 2U> CutDim;
  array<double, 2U> CutVal;
  array<double, 2U> LowerBounds;
  array<double, 2U> UpperBounds;
  array<double, 2U> LeftChild;
  array<double, 2U> RightChild;
  array<bool, 2U> LeafNode;
  array<unsigned int, 1U> IdxAll;
  array<double, 1U> IdxDim;
  bool IsIndexed;

private:
  double NxNoNaN;
};

class c_Kdtree {
public:
  void buildIndex(const ::coder::array<float, 2U> &inputData);
  c_Kdtree *init();
  array<float, 2U> InputData;
  double NxNoNaN;
  array<double, 2U> CutDim;
  array<double, 2U> CutVal;
  array<double, 2U> LowerBounds;
  array<double, 2U> UpperBounds;
  array<double, 2U> LeftChild;
  array<double, 2U> RightChild;
  array<bool, 2U> LeafNode;
  array<unsigned int, 1U> IdxAll;
  array<double, 1U> IdxDim;
  bool IsIndexed;
};

} // namespace codegen
} // namespace internal
} // namespace vision
} // namespace coder

// Function Declarations
void b_binary_expand_op(float in1[3], const float in2[22317], int in3,
                        const coder::array<double, 2U> &in4);

void b_binary_expand_op(float in1[3], const coder::array<float, 2U> &in2,
                        int in3, const coder::array<double, 2U> &in4);

void minus(double in1[3], const double in2[3],
           const coder::array<double, 2U> &in3);

#endif
//
// File trailer for Kdtree.h
//
// [EOF]
//
