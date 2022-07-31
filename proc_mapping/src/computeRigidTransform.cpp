//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: computeRigidTransform.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 31-Jul-2022 13:03:34
//

// Include Files
#include "computeRigidTransform.h"
#include "combineVectorElements.h"
#include "rt_nonfinite.h"
#include "svd1.h"
#include "coder_array.h"
#include <cmath>
#include <string.h>

// Function Definitions
//
// Arguments    : const ::coder::array<float, 2U> &p
//                const ::coder::array<double, 2U> &q
//                float R[9]
//                float t[3]
// Return Type  : void
//
namespace coder {
namespace vision {
namespace internal {
namespace calibration {
void computeRigidTransform(const ::coder::array<float, 2U> &p,
                           const ::coder::array<double, 2U> &q, float R[9],
                           float t[3])
{
  array<double, 2U> normPoints2;
  array<float, 2U> normPoints1;
  double centroid2[3];
  float C[9];
  float U[9];
  float V[9];
  float b_V[9];
  float centroid1[3];
  float b_s;
  float bsum;
  float f;
  int firstBlockLength;
  int hi;
  int ib;
  int lastBlockLength;
  int nblocks;
  int xblockoffset;
  int xpageoffset;
  signed char ipiv[3];
  bool isodd;
  if (p.size(0) == 0) {
    centroid1[0] = 0.0F;
    centroid1[1] = 0.0F;
    centroid1[2] = 0.0F;
  } else {
    if (p.size(0) <= 1024) {
      firstBlockLength = p.size(0);
      lastBlockLength = 0;
      nblocks = 1;
    } else {
      firstBlockLength = 1024;
      nblocks = p.size(0) >> 10;
      lastBlockLength = p.size(0) - (nblocks << 10);
      if (lastBlockLength > 0) {
        nblocks++;
      } else {
        lastBlockLength = 1024;
      }
    }
    for (int xi{0}; xi < 3; xi++) {
      xpageoffset = xi * p.size(0);
      centroid1[xi] = p[xpageoffset];
      for (int k{2}; k <= firstBlockLength; k++) {
        centroid1[xi] += p[(xpageoffset + k) - 1];
      }
      for (ib = 2; ib <= nblocks; ib++) {
        xblockoffset = xpageoffset + ((ib - 1) << 10);
        bsum = p[xblockoffset];
        if (ib == nblocks) {
          hi = lastBlockLength;
        } else {
          hi = 1024;
        }
        for (int k{2}; k <= hi; k++) {
          bsum += p[(xblockoffset + k) - 1];
        }
        centroid1[xi] += bsum;
      }
    }
  }
  combineVectorElements(q, centroid2);
  centroid1[0] /= static_cast<float>(p.size(0));
  centroid2[0] /= static_cast<double>(q.size(0));
  centroid1[1] /= static_cast<float>(p.size(0));
  centroid2[1] /= static_cast<double>(q.size(0));
  centroid1[2] /= static_cast<float>(p.size(0));
  centroid2[2] /= static_cast<double>(q.size(0));
  normPoints1.set_size(p.size(0), 3);
  if (p.size(0) != 0) {
    firstBlockLength = (p.size(0) != 1);
    for (int k{0}; k < 3; k++) {
      ib = normPoints1.size(0) - 1;
      for (nblocks = 0; nblocks <= ib; nblocks++) {
        normPoints1[nblocks + normPoints1.size(0) * k] =
            p[firstBlockLength * nblocks + p.size(0) * k] - centroid1[k];
      }
    }
  }
  normPoints2.set_size(q.size(0), 3);
  if (q.size(0) != 0) {
    firstBlockLength = (q.size(0) != 1);
    for (int k{0}; k < 3; k++) {
      ib = normPoints2.size(0) - 1;
      for (nblocks = 0; nblocks <= ib; nblocks++) {
        normPoints2[nblocks + normPoints2.size(0) * k] =
            q[firstBlockLength * nblocks + q.size(0) * k] - centroid2[k];
      }
    }
  }
  firstBlockLength = normPoints1.size(0);
  for (ib = 0; ib < 3; ib++) {
    for (xpageoffset = 0; xpageoffset < 3; xpageoffset++) {
      nblocks = ib + 3 * xpageoffset;
      C[nblocks] = 0.0F;
      for (lastBlockLength = 0; lastBlockLength < firstBlockLength;
           lastBlockLength++) {
        C[nblocks] +=
            normPoints1[lastBlockLength + normPoints1.size(0) * ib] *
            static_cast<float>(normPoints2[lastBlockLength +
                                           normPoints2.size(0) * xpageoffset]);
      }
    }
  }
  isodd = true;
  for (int k{0}; k < 9; k++) {
    if ((!isodd) || (std::isinf(C[k]) || std::isnan(C[k]))) {
      isodd = false;
    }
  }
  if (isodd) {
    float s[3];
    ::coder::internal::b_svd(C, U, s, V);
  } else {
    for (ib = 0; ib < 9; ib++) {
      U[ib] = rtNaNF;
      V[ib] = rtNaNF;
    }
  }
  for (ib = 0; ib < 3; ib++) {
    bsum = U[ib];
    b_s = U[ib + 3];
    f = U[ib + 6];
    for (xpageoffset = 0; xpageoffset < 3; xpageoffset++) {
      C[ib + 3 * xpageoffset] =
          (bsum * V[xpageoffset] + b_s * V[xpageoffset + 3]) +
          f * V[xpageoffset + 6];
    }
    ipiv[ib] = static_cast<signed char>(ib + 1);
  }
  for (int xi{0}; xi < 2; xi++) {
    xblockoffset = 1 - xi;
    hi = xi << 2;
    lastBlockLength = hi + 2;
    firstBlockLength = 3 - xi;
    nblocks = 0;
    bsum = std::abs(C[hi]);
    for (int k{2}; k <= firstBlockLength; k++) {
      b_s = std::abs(C[(hi + k) - 1]);
      if (b_s > bsum) {
        nblocks = k - 1;
        bsum = b_s;
      }
    }
    if (C[hi + nblocks] != 0.0F) {
      if (nblocks != 0) {
        firstBlockLength = xi + nblocks;
        ipiv[xi] = static_cast<signed char>(firstBlockLength + 1);
        bsum = C[xi];
        C[xi] = C[firstBlockLength];
        C[firstBlockLength] = bsum;
        bsum = C[xi + 3];
        C[xi + 3] = C[firstBlockLength + 3];
        C[firstBlockLength + 3] = bsum;
        bsum = C[xi + 6];
        C[xi + 6] = C[firstBlockLength + 6];
        C[firstBlockLength + 6] = bsum;
      }
      ib = (hi - xi) + 3;
      for (firstBlockLength = lastBlockLength; firstBlockLength <= ib;
           firstBlockLength++) {
        C[firstBlockLength - 1] /= C[hi];
      }
    }
    firstBlockLength = hi;
    for (nblocks = 0; nblocks <= xblockoffset; nblocks++) {
      bsum = C[(hi + nblocks * 3) + 3];
      if (bsum != 0.0F) {
        ib = firstBlockLength + 5;
        xpageoffset = (firstBlockLength - xi) + 6;
        for (lastBlockLength = ib; lastBlockLength <= xpageoffset;
             lastBlockLength++) {
          C[lastBlockLength - 1] +=
              C[((hi + lastBlockLength) - firstBlockLength) - 4] * -bsum;
        }
      }
      firstBlockLength += 3;
    }
  }
  isodd = (ipiv[0] > 1);
  bsum = C[0] * C[4] * C[8];
  if (ipiv[1] > 2) {
    isodd = !isodd;
  }
  if (isodd) {
    bsum = -bsum;
  }
  if (!std::isnan(bsum)) {
    if (bsum < 0.0F) {
      bsum = -1.0F;
    } else {
      bsum = (bsum > 0.0F);
    }
  }
  for (ib = 0; ib < 9; ib++) {
    C[ib] = 0.0F;
  }
  C[0] = 1.0F;
  C[4] = 1.0F;
  C[8] = bsum;
  for (ib = 0; ib < 3; ib++) {
    float f1;
    bsum = V[ib];
    b_s = V[ib + 3];
    f = V[ib + 6];
    for (xpageoffset = 0; xpageoffset < 3; xpageoffset++) {
      b_V[ib + 3 * xpageoffset] =
          (bsum * C[3 * xpageoffset] + b_s * C[3 * xpageoffset + 1]) +
          f * C[3 * xpageoffset + 2];
    }
    bsum = 0.0F;
    b_s = b_V[ib];
    f = b_V[ib + 3];
    f1 = b_V[ib + 6];
    for (xpageoffset = 0; xpageoffset < 3; xpageoffset++) {
      float f2;
      f2 = (b_s * U[xpageoffset] + f * U[xpageoffset + 3]) +
           f1 * U[xpageoffset + 6];
      R[ib + 3 * xpageoffset] = f2;
      bsum += f2 * centroid1[xpageoffset];
    }
    t[ib] = static_cast<float>(centroid2[ib]) - bsum;
  }
}

} // namespace calibration
} // namespace internal
} // namespace vision
} // namespace coder

//
// File trailer for computeRigidTransform.cpp
//
// [EOF]
//
