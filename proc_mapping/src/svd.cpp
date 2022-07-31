//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: svd.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 31-Jul-2022 13:03:34
//

// Include Files
#include "svd.h"
#include "rt_nonfinite.h"
#include "svd1.h"
#include "xnrm2.h"
#include "xrotg.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <string.h>

// Function Definitions
//
// Arguments    : const double A[9]
//                double U[9]
//                double S[9]
//                double V[9]
// Return Type  : void
//
namespace coder {
void svd(const double A[9], double U[9], double S[9], double V[9])
{
  double s[3];
  bool p;
  p = true;
  for (int k{0}; k < 9; k++) {
    if ((!p) || (std::isinf(A[k]) || std::isnan(A[k]))) {
      p = false;
    }
  }
  if (p) {
    internal::b_svd(A, U, s, V);
  } else {
    s[0] = rtNaN;
    s[1] = rtNaN;
    s[2] = rtNaN;
    for (int k{0}; k < 9; k++) {
      U[k] = rtNaN;
      V[k] = rtNaN;
    }
  }
  std::memset(&S[0], 0, 9U * sizeof(double));
  S[0] = s[0];
  S[4] = s[1];
  S[8] = s[2];
}

//
// Arguments    : const double A[9]
//                double U[3]
// Return Type  : void
//
void svd(const double A[9], double U[3])
{
  double b_A[9];
  double s[3];
  double nrm;
  double rt;
  double sm;
  double sqds;
  bool apply_transform;
  apply_transform = true;
  for (int k{0}; k < 9; k++) {
    if ((!apply_transform) || (std::isinf(A[k]) || std::isnan(A[k]))) {
      apply_transform = false;
    }
  }
  if (apply_transform) {
    double e[3];
    double work[3];
    double r;
    double snorm;
    int ii;
    int iter;
    int m;
    int qp1;
    int qq;
    int qq_tmp;
    int qs;
    std::copy(&A[0], &A[9], &b_A[0]);
    s[0] = 0.0;
    e[0] = 0.0;
    work[0] = 0.0;
    s[1] = 0.0;
    e[1] = 0.0;
    work[1] = 0.0;
    s[2] = 0.0;
    e[2] = 0.0;
    work[2] = 0.0;
    for (int q{0}; q < 2; q++) {
      qp1 = q + 2;
      qq_tmp = q + 3 * q;
      qq = qq_tmp + 1;
      apply_transform = false;
      nrm = internal::blas::xnrm2(3 - q, b_A, qq_tmp + 1);
      if (nrm > 0.0) {
        apply_transform = true;
        if (b_A[qq_tmp] < 0.0) {
          nrm = -nrm;
        }
        s[q] = nrm;
        if (std::abs(nrm) >= 1.0020841800044864E-292) {
          nrm = 1.0 / nrm;
          qs = (qq_tmp - q) + 3;
          for (int k{qq}; k <= qs; k++) {
            b_A[k - 1] *= nrm;
          }
        } else {
          qs = (qq_tmp - q) + 3;
          for (int k{qq}; k <= qs; k++) {
            b_A[k - 1] /= s[q];
          }
        }
        b_A[qq_tmp]++;
        s[q] = -s[q];
      } else {
        s[q] = 0.0;
      }
      for (m = qp1; m < 4; m++) {
        qq = q + 3 * (m - 1);
        if (apply_transform) {
          qs = 2 - q;
          nrm = 0.0;
          for (int k{0}; k <= qs; k++) {
            nrm += b_A[qq_tmp + k] * b_A[qq + k];
          }
          nrm = -(nrm / b_A[qq_tmp]);
          if (!(nrm == 0.0)) {
            for (int k{0}; k <= qs; k++) {
              iter = qq + k;
              b_A[iter] += nrm * b_A[qq_tmp + k];
            }
          }
        }
        e[m - 1] = b_A[qq];
      }
      if (q + 1 <= 1) {
        nrm = internal::blas::xnrm2(e);
        if (nrm == 0.0) {
          e[0] = 0.0;
        } else {
          if (e[1] < 0.0) {
            e[0] = -nrm;
          } else {
            e[0] = nrm;
          }
          nrm = e[0];
          if (std::abs(e[0]) >= 1.0020841800044864E-292) {
            nrm = 1.0 / e[0];
            for (int k{qp1}; k < 4; k++) {
              e[k - 1] *= nrm;
            }
          } else {
            for (int k{qp1}; k < 4; k++) {
              e[k - 1] /= nrm;
            }
          }
          e[1]++;
          e[0] = -e[0];
          for (ii = qp1; ii < 4; ii++) {
            work[ii - 1] = 0.0;
          }
          for (m = qp1; m < 4; m++) {
            nrm = e[m - 1];
            if (!(nrm == 0.0)) {
              qq = q + 3 * (m - 1);
              work[q + 1] += nrm * b_A[qq + 1];
              work[q + 2] += nrm * b_A[qq + 2];
            }
          }
          for (m = qp1; m < 4; m++) {
            nrm = -e[m - 1] / e[1];
            if (!(nrm == 0.0)) {
              qq = (q + 3 * (m - 1)) + 1;
              b_A[qq] += nrm * work[q + 1];
              b_A[qq + 1] += nrm * work[q + 2];
            }
          }
        }
      }
    }
    m = 1;
    s[2] = b_A[8];
    e[1] = b_A[7];
    e[2] = 0.0;
    iter = 0;
    nrm = s[0];
    if (s[0] != 0.0) {
      rt = std::abs(s[0]);
      r = s[0] / rt;
      nrm = rt;
      s[0] = rt;
      e[0] /= r;
    }
    if (e[0] != 0.0) {
      rt = std::abs(e[0]);
      r = rt / e[0];
      e[0] = rt;
      s[1] *= r;
    }
    snorm = std::fmax(std::abs(nrm), e[0]);
    nrm = s[1];
    if (s[1] != 0.0) {
      rt = std::abs(s[1]);
      r = s[1] / rt;
      nrm = rt;
      s[1] = rt;
      e[1] = b_A[7] / r;
    }
    if (e[1] != 0.0) {
      rt = std::abs(e[1]);
      r = rt / e[1];
      e[1] = rt;
      s[2] = b_A[8] * r;
    }
    snorm = std::fmax(snorm, std::fmax(std::abs(nrm), e[1]));
    nrm = s[2];
    if (s[2] != 0.0) {
      rt = std::abs(s[2]);
      nrm = rt;
      s[2] = rt;
    }
    snorm = std::fmax(snorm, std::fmax(std::abs(nrm), 0.0));
    while ((m + 2 > 0) && (iter < 75)) {
      bool exitg1;
      qq_tmp = m + 1;
      ii = m + 1;
      exitg1 = false;
      while (!(exitg1 || (ii == 0))) {
        nrm = std::abs(e[ii - 1]);
        if ((nrm <= 2.2204460492503131E-16 *
                        (std::abs(s[ii - 1]) + std::abs(s[ii]))) ||
            (nrm <= 1.0020841800044864E-292) ||
            ((iter > 20) && (nrm <= 2.2204460492503131E-16 * snorm))) {
          e[ii - 1] = 0.0;
          exitg1 = true;
        } else {
          ii--;
        }
      }
      if (ii == m + 1) {
        qq = 4;
      } else {
        qs = m + 2;
        qq = m + 2;
        exitg1 = false;
        while ((!exitg1) && (qq >= ii)) {
          qs = qq;
          if (qq == ii) {
            exitg1 = true;
          } else {
            nrm = 0.0;
            if (qq < m + 2) {
              nrm = std::abs(e[qq - 1]);
            }
            if (qq > ii + 1) {
              nrm += std::abs(e[qq - 2]);
            }
            rt = std::abs(s[qq - 1]);
            if ((rt <= 2.2204460492503131E-16 * nrm) ||
                (rt <= 1.0020841800044864E-292)) {
              s[qq - 1] = 0.0;
              exitg1 = true;
            } else {
              qq--;
            }
          }
        }
        if (qs == ii) {
          qq = 3;
        } else if (qs == m + 2) {
          qq = 1;
        } else {
          qq = 2;
          ii = qs;
        }
      }
      switch (qq) {
      case 1:
        rt = e[m];
        e[m] = 0.0;
        for (int k{qq_tmp}; k >= ii + 1; k--) {
          internal::blas::xrotg(&s[k - 1], &rt, &sm, &sqds);
          if (k > ii + 1) {
            rt = -sqds * e[0];
            e[0] *= sm;
          }
        }
        break;
      case 2:
        rt = e[ii - 1];
        e[ii - 1] = 0.0;
        for (int k{ii + 1}; k <= m + 2; k++) {
          internal::blas::xrotg(&s[k - 1], &rt, &sm, &sqds);
          r = e[k - 1];
          rt = -sqds * r;
          e[k - 1] = r * sm;
        }
        break;
      case 3: {
        double scale;
        nrm = s[m + 1];
        scale = std::fmax(
            std::fmax(std::fmax(std::fmax(std::abs(nrm), std::abs(s[m])),
                                std::abs(e[m])),
                      std::abs(s[ii])),
            std::abs(e[ii]));
        sm = nrm / scale;
        nrm = s[m] / scale;
        rt = e[m] / scale;
        sqds = s[ii] / scale;
        r = ((nrm + sm) * (nrm - sm) + rt * rt) / 2.0;
        nrm = sm * rt;
        nrm *= nrm;
        if ((r != 0.0) || (nrm != 0.0)) {
          rt = std::sqrt(r * r + nrm);
          if (r < 0.0) {
            rt = -rt;
          }
          rt = nrm / (r + rt);
        } else {
          rt = 0.0;
        }
        rt += (sqds + sm) * (sqds - sm);
        nrm = sqds * (e[ii] / scale);
        for (int k{ii + 1}; k <= qq_tmp; k++) {
          internal::blas::xrotg(&rt, &nrm, &sm, &sqds);
          if (k > ii + 1) {
            e[0] = rt;
          }
          nrm = e[k - 1];
          r = s[k - 1];
          e[k - 1] = sm * nrm - sqds * r;
          rt = sqds * s[k];
          s[k] *= sm;
          s[k - 1] = sm * r + sqds * nrm;
          internal::blas::xrotg(&s[k - 1], &rt, &sm, &sqds);
          rt = sm * e[k - 1] + sqds * s[k];
          s[k] = -sqds * e[k - 1] + sm * s[k];
          nrm = sqds * e[k];
          e[k] *= sm;
        }
        e[m] = rt;
        iter++;
      } break;
      default:
        if (s[ii] < 0.0) {
          s[ii] = -s[ii];
        }
        qp1 = ii + 1;
        while ((ii + 1 < 3) && (s[ii] < s[qp1])) {
          rt = s[ii];
          s[ii] = s[qp1];
          s[qp1] = rt;
          ii = qp1;
          qp1++;
        }
        iter = 0;
        m--;
        break;
      }
    }
    U[0] = s[0];
    U[1] = s[1];
    U[2] = s[2];
  } else {
    U[0] = rtNaN;
    U[1] = rtNaN;
    U[2] = rtNaN;
  }
}

} // namespace coder

//
// File trailer for svd.cpp
//
// [EOF]
//
