//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: schur.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 31-Jul-2022 13:03:34
//

// Include Files
#include "schur.h"
#include "proc_mapping_rtwutil.h"
#include "rt_nonfinite.h"
#include "xhseqr.h"
#include "xnrm2.h"
#include "xzlarf.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <string.h>

// Function Definitions
//
// Arguments    : const double A[16]
//                double V[16]
//                double T[16]
// Return Type  : void
//
namespace coder {
void schur(const double A[16], double V[16], double T[16])
{
  double work[4];
  bool p;
  p = true;
  for (int k{0}; k < 16; k++) {
    if ((!p) || (std::isinf(A[k]) || std::isnan(A[k]))) {
      p = false;
    }
  }
  if (!p) {
    int knt;
    for (int i{0}; i < 16; i++) {
      V[i] = rtNaN;
    }
    knt = 2;
    for (int ix0{0}; ix0 < 3; ix0++) {
      if (knt <= 4) {
        std::memset(&V[(ix0 * 4 + knt) + -1], 0, (-knt + 5) * sizeof(double));
      }
      knt++;
    }
    for (int i{0}; i < 16; i++) {
      T[i] = rtNaN;
    }
  } else {
    double tau[3];
    int i;
    int ia;
    int ix0;
    int knt;
    std::copy(&A[0], &A[16], &T[0]);
    work[0] = 0.0;
    work[1] = 0.0;
    work[2] = 0.0;
    work[3] = 0.0;
    for (int b_i{0}; b_i < 3; b_i++) {
      double alpha1_tmp;
      double xnorm;
      int alpha1_tmp_tmp;
      int ic0;
      int im1n_tmp_tmp;
      int in;
      int lastc;
      int lastv;
      im1n_tmp_tmp = b_i << 2;
      in = (b_i + 1) << 2;
      alpha1_tmp_tmp = (b_i + im1n_tmp_tmp) + 1;
      alpha1_tmp = T[alpha1_tmp_tmp];
      if (b_i + 3 <= 4) {
        knt = b_i + 1;
      } else {
        knt = 2;
      }
      ix0 = (knt + im1n_tmp_tmp) + 2;
      tau[b_i] = 0.0;
      xnorm = internal::blas::b_xnrm2(2 - b_i, T, ix0);
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
              T[k - 1] *= 9.9792015476736E+291;
            }
            beta1 *= 9.9792015476736E+291;
            alpha1_tmp *= 9.9792015476736E+291;
          } while ((std::abs(beta1) < 1.0020841800044864E-292) && (knt < 20));
          beta1 = rt_hypotd_snf(alpha1_tmp,
                                internal::blas::b_xnrm2(2 - b_i, T, ix0));
          if (alpha1_tmp >= 0.0) {
            beta1 = -beta1;
          }
          tau[b_i] = (beta1 - alpha1_tmp) / beta1;
          xnorm = 1.0 / (alpha1_tmp - beta1);
          i = (ix0 - b_i) + 1;
          for (int k{ix0}; k <= i; k++) {
            T[k - 1] *= xnorm;
          }
          for (int k{0}; k < knt; k++) {
            beta1 *= 1.0020841800044864E-292;
          }
          alpha1_tmp = beta1;
        } else {
          tau[b_i] = (beta1 - alpha1_tmp) / beta1;
          xnorm = 1.0 / (alpha1_tmp - beta1);
          i = (ix0 - b_i) + 1;
          for (int k{ix0}; k <= i; k++) {
            T[k - 1] *= xnorm;
          }
          alpha1_tmp = beta1;
        }
      }
      T[alpha1_tmp_tmp] = 1.0;
      ic0 = in + 1;
      if (tau[b_i] != 0.0) {
        bool exitg2;
        lastv = 2 - b_i;
        knt = (alpha1_tmp_tmp - b_i) + 2;
        while ((lastv + 1 > 0) && (T[knt] == 0.0)) {
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
              if (T[ia - 1] != 0.0) {
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
              work[ix0] += T[ia - 1] * T[knt];
            }
            knt++;
          }
        }
        if (!(-tau[b_i] == 0.0)) {
          knt = in;
          for (ix0 = 0; ix0 <= lastv; ix0++) {
            xnorm = T[alpha1_tmp_tmp + ix0];
            if (xnorm != 0.0) {
              xnorm *= -tau[b_i];
              i = knt + 1;
              i1 = lastc + knt;
              for (int k{i}; k <= i1; k++) {
                T[k - 1] += work[(k - knt) - 1] * xnorm;
              }
            }
            knt += 4;
          }
        }
      }
      internal::reflapack::xzlarf(3 - b_i, 3 - b_i, (b_i + im1n_tmp_tmp) + 2,
                                  tau[b_i], T, (b_i + in) + 2, work);
      T[alpha1_tmp_tmp] = alpha1_tmp;
    }
    std::copy(&T[0], &T[16], &V[0]);
    for (ix0 = 2; ix0 >= 0; ix0--) {
      ia = (ix0 + 1) << 2;
      for (int b_i{0}; b_i <= ix0; b_i++) {
        V[ia + b_i] = 0.0;
      }
      i = ix0 + 3;
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
      for (ix0 = 0; ix0 < b_i; ix0++) {
        V[(knt - ix0) - 1] = 0.0;
      }
    }
    internal::lapack::xhseqr(T, V);
  }
}

} // namespace coder

//
// File trailer for schur.cpp
//
// [EOF]
//
