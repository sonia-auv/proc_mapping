//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: schur.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 01-Aug-2022 08:26:09
//

// Include Files
#include "schur.h"
#include "proc_mapping_rtwutil.h"
#include "rt_nonfinite.h"
#include "xdhseqr.h"
#include "xnrm2.h"
#include "xzlarf.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <string.h>

// Function Definitions
//
// Arguments    : double A[16]
//                double V[16]
// Return Type  : void
//
namespace coder {
void schur(double A[16], double V[16])
{
  double work[4];
  double xnorm;
  bool p;
  p = true;
  for (int k{0}; k < 16; k++) {
    if (p) {
      xnorm = A[k];
      if (std::isinf(xnorm) || std::isnan(xnorm)) {
        p = false;
      }
    } else {
      p = false;
    }
  }
  if (!p) {
    int knt;
    for (int i{0}; i < 16; i++) {
      V[i] = rtNaN;
    }
    knt = 2;
    for (int k{0}; k < 3; k++) {
      if (knt <= 4) {
        std::memset(&V[(k * 4 + knt) + -1], 0, (-knt + 5) * sizeof(double));
      }
      knt++;
    }
    for (int i{0}; i < 16; i++) {
      A[i] = rtNaN;
    }
  } else {
    double tau[3];
    int i;
    int ia;
    int ix0;
    int knt;
    work[0] = 0.0;
    work[1] = 0.0;
    work[2] = 0.0;
    work[3] = 0.0;
    for (int b_i{0}; b_i < 3; b_i++) {
      double alpha1_tmp;
      int alpha1_tmp_tmp;
      int ic0;
      int im1n_tmp_tmp;
      int in;
      int lastc;
      int lastv;
      im1n_tmp_tmp = b_i << 2;
      in = (b_i + 1) << 2;
      alpha1_tmp_tmp = (b_i + im1n_tmp_tmp) + 1;
      alpha1_tmp = A[alpha1_tmp_tmp];
      if (b_i + 3 <= 4) {
        knt = b_i + 1;
      } else {
        knt = 2;
      }
      ix0 = (knt + im1n_tmp_tmp) + 2;
      tau[b_i] = 0.0;
      xnorm = internal::blas::b_xnrm2(2 - b_i, A, ix0);
      if (xnorm != 0.0) {
        double beta1;
        beta1 = rt_hypotd_snf(alpha1_tmp, xnorm);
        if (alpha1_tmp >= 0.0) {
          beta1 = -beta1;
        }
        if (std::abs(beta1) < 1.0020841800044864E-292) {
          knt = 0;
          i = (ix0 - b_i) + 1;
          do {
            knt++;
            for (int k{ix0}; k <= i; k++) {
              A[k - 1] *= 9.9792015476736E+291;
            }
            beta1 *= 9.9792015476736E+291;
            alpha1_tmp *= 9.9792015476736E+291;
          } while ((std::abs(beta1) < 1.0020841800044864E-292) && (knt < 20));
          beta1 = rt_hypotd_snf(alpha1_tmp,
                                internal::blas::b_xnrm2(2 - b_i, A, ix0));
          if (alpha1_tmp >= 0.0) {
            beta1 = -beta1;
          }
          tau[b_i] = (beta1 - alpha1_tmp) / beta1;
          xnorm = 1.0 / (alpha1_tmp - beta1);
          i = (ix0 - b_i) + 1;
          for (int k{ix0}; k <= i; k++) {
            A[k - 1] *= xnorm;
          }
          for (int k{0}; k < knt; k++) {
            beta1 *= 1.0020841800044864E-292;
          }
          alpha1_tmp = beta1;
        } else {
          xnorm = A[alpha1_tmp_tmp];
          tau[b_i] = (beta1 - xnorm) / beta1;
          xnorm = 1.0 / (xnorm - beta1);
          i = (ix0 - b_i) + 1;
          for (int k{ix0}; k <= i; k++) {
            A[k - 1] *= xnorm;
          }
          alpha1_tmp = beta1;
        }
      }
      A[alpha1_tmp_tmp] = 1.0;
      ic0 = in + 1;
      if (tau[b_i] != 0.0) {
        bool exitg2;
        lastv = 2 - b_i;
        knt = (alpha1_tmp_tmp - b_i) + 2;
        while ((lastv + 1 > 0) && (A[knt] == 0.0)) {
          lastv--;
          knt--;
        }
        lastc = 4;
        exitg2 = false;
        while ((!exitg2) && (lastc > 0)) {
          int exitg1;
          knt = in + lastc;
          ia = knt;
          do {
            exitg1 = 0;
            if (ia <= knt + (lastv << 2)) {
              if (A[ia - 1] != 0.0) {
                exitg1 = 1;
              } else {
                ia += 4;
              }
            } else {
              lastc--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);
          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        lastv = -1;
        lastc = 0;
      }
      if (lastv + 1 > 0) {
        int i1;
        if (lastc != 0) {
          std::memset(&work[0], 0, lastc * sizeof(double));
          knt = alpha1_tmp_tmp;
          i = (in + (lastv << 2)) + 1;
          for (int k{ic0}; k <= i; k += 4) {
            i1 = (k + lastc) - 1;
            for (ia = k; ia <= i1; ia++) {
              ix0 = ia - k;
              work[ix0] += A[ia - 1] * A[knt];
            }
            knt++;
          }
        }
        if (!(-tau[b_i] == 0.0)) {
          knt = in;
          for (int k{0}; k <= lastv; k++) {
            xnorm = A[alpha1_tmp_tmp + k];
            if (xnorm != 0.0) {
              xnorm *= -tau[b_i];
              i = knt + 1;
              i1 = lastc + knt;
              for (ix0 = i; ix0 <= i1; ix0++) {
                A[ix0 - 1] += work[(ix0 - knt) - 1] * xnorm;
              }
            }
            knt += 4;
          }
        }
      }
      internal::reflapack::xzlarf(3 - b_i, 3 - b_i, (b_i + im1n_tmp_tmp) + 2,
                                  tau[b_i], A, (b_i + in) + 2, work);
      A[alpha1_tmp_tmp] = alpha1_tmp;
    }
    std::copy(&A[0], &A[16], &V[0]);
    for (int k{2}; k >= 0; k--) {
      ia = (k + 1) << 2;
      for (int b_i{0}; b_i <= k; b_i++) {
        V[ia + b_i] = 0.0;
      }
      i = k + 3;
      for (int b_i{i}; b_i < 5; b_i++) {
        knt = ia + b_i;
        V[knt - 1] = V[knt - 5];
      }
    }
    V[1] = 0.0;
    V[2] = 0.0;
    V[3] = 0.0;
    V[0] = 1.0;
    work[0] = 0.0;
    work[1] = 0.0;
    work[2] = 0.0;
    work[3] = 0.0;
    for (int b_i{2}; b_i >= 0; b_i--) {
      knt = (b_i + (b_i << 2)) + 5;
      if (b_i + 1 < 3) {
        V[knt] = 1.0;
        internal::reflapack::xzlarf(3 - b_i, 2 - b_i, knt + 1, tau[b_i], V,
                                    knt + 5, work);
        ix0 = knt + 2;
        i = (knt - b_i) + 3;
        for (int k{ix0}; k <= i; k++) {
          V[k - 1] *= -tau[b_i];
        }
      }
      V[knt] = 1.0 - tau[b_i];
      for (int k{0}; k < b_i; k++) {
        V[(knt - k) - 1] = 0.0;
      }
    }
    internal::reflapack::eml_dlahqr(A, V);
    A[3] = 0.0;
  }
}

} // namespace coder

//
// File trailer for schur.cpp
//
// [EOF]
//
