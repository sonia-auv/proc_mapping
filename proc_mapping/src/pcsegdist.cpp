//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: pcsegdist.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 01-Aug-2022 08:26:09
//

// Include Files
#include "pcsegdist.h"
#include "pointCloud.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <string.h>

// Function Definitions
//
// Arguments    : const pointCloud *ptCloud
//                double minDistance
//                ::coder::array<unsigned int, 1U> &labels
//                double *numClusters
// Return Type  : void
//
namespace coder {
void pcsegdist(const pointCloud *ptCloud, double minDistance,
               ::coder::array<unsigned int, 1U> &labels, double *numClusters)
{
  d_pointCloud pc;
  d_pointCloud *b_pc;
  array<double, 1U> validIndices;
  array<unsigned int, 1U> L;
  array<int, 1U> idx;
  array<unsigned int, 1U> ind;
  array<int, 1U> iwork;
  array<int, 1U> r;
  array<int, 1U> r1;
  double newLabel;
  int b_i;
  int i;
  int i1;
  int j;
  int k;
  unsigned int labelsSize_idx_0;
  int n;
  int na;
  int nb;
  int qEnd;
  unsigned int u1;
  pc.matlabCodegenIsDeleted = true;
  labelsSize_idx_0 = static_cast<unsigned int>(ptCloud->Location.size(0));
  labels.set_size(static_cast<int>(labelsSize_idx_0));
  j = static_cast<int>(labelsSize_idx_0);
  for (i = 0; i < j; i++) {
    labels[i] = 0U;
  }
  ptCloud->removeInvalidPoints(&pc, &b_pc, validIndices);
  nb = pc.Location.size(0);
  L.set_size(nb);
  for (i = 0; i < nb; i++) {
    L[i] = 0U;
  }
  newLabel = 0.0;
  i = pc.Location.size(0) - 1;
  for (b_i = 0; b_i <= i; b_i++) {
    if (L[b_i] == 0U) {
      double c_pc[3];
      double b_newLabel;
      c_pc[0] = pc.Location[b_i];
      c_pc[1] = pc.Location[b_i + pc.Location.size(0)];
      c_pc[2] = pc.Location[b_i + pc.Location.size(0) * 2];
      pc.findNeighborsInRadius(c_pc, minDistance, ind);
      b_newLabel = newLabel;
      i1 = ind.size(0);
      for (k = 0; k < i1; k++) {
        unsigned int u;
        labelsSize_idx_0 = ind[k];
        u = L[static_cast<int>(ind[k]) - 1];
        if ((u > 0U) && (L[b_i] > 0U)) {
          if (u > L[b_i]) {
            j = L.size(0) - 1;
            nb = 0;
            for (int pEnd{0}; pEnd <= j; pEnd++) {
              if (L[pEnd] == L[static_cast<int>(labelsSize_idx_0) - 1]) {
                nb++;
              }
            }
            r1.set_size(nb);
            nb = 0;
            for (int pEnd{0}; pEnd <= j; pEnd++) {
              if (L[pEnd] == L[static_cast<int>(labelsSize_idx_0) - 1]) {
                r1[nb] = pEnd + 1;
                nb++;
              }
            }
            labelsSize_idx_0 = L[b_i];
            j = r1.size(0);
            for (nb = 0; nb < j; nb++) {
              L[r1[nb] - 1] = labelsSize_idx_0;
            }
          } else if (L[static_cast<int>(ind[k]) - 1] < L[b_i]) {
            j = L.size(0) - 1;
            nb = 0;
            for (int pEnd{0}; pEnd <= j; pEnd++) {
              if (L[pEnd] == L[b_i]) {
                nb++;
              }
            }
            idx.set_size(nb);
            nb = 0;
            for (int pEnd{0}; pEnd <= j; pEnd++) {
              if (L[pEnd] == L[b_i]) {
                idx[nb] = pEnd + 1;
                nb++;
              }
            }
            labelsSize_idx_0 = L[static_cast<int>(ind[k]) - 1];
            j = idx.size(0);
            for (nb = 0; nb < j; nb++) {
              L[idx[nb] - 1] = labelsSize_idx_0;
            }
          }
        } else if (u > 0U) {
          L[b_i] = L[static_cast<int>(ind[k]) - 1];
        } else if (L[b_i] > 0U) {
          L[static_cast<int>(ind[k]) - 1] = L[b_i];
        }
      }
      if (L[b_i] == 0U) {
        b_newLabel = newLabel + 1.0;
        r.set_size(ind.size(0));
        j = ind.size(0);
        for (i1 = 0; i1 < j; i1++) {
          r[i1] = static_cast<int>(ind[i1]);
        }
        j = r.size(0);
        if (j - 1 >= 0) {
          if (newLabel + 1.0 < 4.294967296E+9) {
            u1 = static_cast<unsigned int>(newLabel + 1.0);
          } else {
            u1 = MAX_uint32_T;
          }
        }
        for (i1 = 0; i1 < j; i1++) {
          L[r[i1] - 1] = u1;
        }
      }
      newLabel = b_newLabel;
    }
  }
  na = L.size(0);
  n = L.size(0) + 1;
  idx.set_size(L.size(0));
  j = L.size(0);
  for (i = 0; i < j; i++) {
    idx[i] = 0;
  }
  if (L.size(0) != 0) {
    iwork.set_size(L.size(0));
    i = L.size(0) - 1;
    for (k = 1; k <= i; k += 2) {
      if (L[k - 1] <= L[k]) {
        idx[k - 1] = k;
        idx[k] = k + 1;
      } else {
        idx[k - 1] = k + 1;
        idx[k] = k;
      }
    }
    if ((L.size(0) & 1) != 0) {
      idx[L.size(0) - 1] = L.size(0);
    }
    b_i = 2;
    while (b_i < n - 1) {
      nb = b_i << 1;
      j = 1;
      for (int pEnd{b_i + 1}; pEnd < n; pEnd = qEnd + b_i) {
        int kEnd;
        int p;
        int q;
        p = j;
        q = pEnd;
        qEnd = j + nb;
        if (qEnd > n) {
          qEnd = n;
        }
        k = 0;
        kEnd = qEnd - j;
        while (k + 1 <= kEnd) {
          i = idx[q - 1];
          i1 = idx[p - 1];
          if (L[i1 - 1] <= L[i - 1]) {
            iwork[k] = i1;
            p++;
            if (p == pEnd) {
              while (q < qEnd) {
                k++;
                iwork[k] = idx[q - 1];
                q++;
              }
            }
          } else {
            iwork[k] = i;
            q++;
            if (q == qEnd) {
              while (p < pEnd) {
                k++;
                iwork[k] = idx[p - 1];
                p++;
              }
            }
          }
          k++;
        }
        for (k = 0; k < kEnd; k++) {
          idx[(j + k) - 1] = iwork[k];
        }
        j = qEnd;
      }
      b_i = nb;
    }
  }
  ind.set_size(L.size(0));
  for (k = 0; k < na; k++) {
    ind[k] = L[idx[k] - 1];
  }
  nb = 0;
  k = 1;
  while (k <= na) {
    labelsSize_idx_0 = ind[k - 1];
    do {
      k++;
    } while (!((k > na) || (ind[k - 1] != labelsSize_idx_0)));
    nb++;
    ind[nb - 1] = labelsSize_idx_0;
  }
  if (nb < 1) {
    nb = 0;
  }
  ind.set_size(nb);
  for (k = 0; k < nb; k++) {
    j = L.size(0);
    for (b_i = 0; b_i < j; b_i++) {
      if (L[b_i] == ind[k]) {
        L[b_i] = static_cast<unsigned int>(k + 1);
      }
    }
  }
  j = L.size(0);
  for (i = 0; i < j; i++) {
    labels[static_cast<int>(validIndices[i]) - 1] = L[i];
  }
  pc.matlabCodegenDestructor();
  *numClusters = nb;
}

} // namespace coder

//
// File trailer for pcsegdist.cpp
//
// [EOF]
//
