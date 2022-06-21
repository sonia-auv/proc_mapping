//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// svd.cpp
//
// Code generation for function 'svd'
//

// Include files
#include "svd.h"
#include "rt_nonfinite.h"
#include "xnrm2.h"
#include "xrotg.h"
#include "coder_array.h"
#include <cmath>
#include <string.h>

// Function Definitions
namespace coder {
namespace internal {
void svd(const ::coder::array<double, 2U> &A, ::coder::array<double, 1U> &U)
{
  array<double, 2U> b_A;
  array<double, 1U> s;
  array<double, 1U> work;
  double e[3];
  double nrm;
  double rt;
  double sm;
  double sqds;
  int i;
  int minnp;
  int n;
  int ns;
  b_A.set_size(A.size(0), 3);
  ns = A.size(0) * 3;
  for (i = 0; i < ns; i++) {
    b_A[i] = A[i];
  }
  n = A.size(0);
  ns = A.size(0) + 1;
  if (ns > 3) {
    ns = 3;
  }
  minnp = A.size(0);
  if (minnp > 3) {
    minnp = 3;
  }
  s.set_size(ns);
  for (i = 0; i < ns; i++) {
    s[i] = 0.0;
  }
  e[0] = 0.0;
  e[1] = 0.0;
  e[2] = 0.0;
  work.set_size(A.size(0));
  ns = A.size(0);
  for (i = 0; i < ns; i++) {
    work[i] = 0.0;
  }
  if (A.size(0) != 0) {
    double r;
    double snorm;
    int iter;
    int jj;
    int mm;
    int nct;
    int nmq;
    int qjj;
    int qp1;
    nct = A.size(0) - 1;
    if (nct > 3) {
      nct = 3;
    }
    if (nct >= 1) {
      i = nct;
    } else {
      i = 1;
    }
    for (int q{0}; q < i; q++) {
      bool apply_transform;
      qp1 = q + 2;
      ns = (q + n * q) + 1;
      nmq = (n - q) - 1;
      apply_transform = false;
      if (q + 1 <= nct) {
        nrm = blas::xnrm2(nmq + 1, b_A, ns);
        if (nrm > 0.0) {
          apply_transform = true;
          if (b_A[ns - 1] < 0.0) {
            nrm = -nrm;
          }
          s[q] = nrm;
          if (std::abs(nrm) >= 1.0020841800044864E-292) {
            nrm = 1.0 / nrm;
            mm = ns + nmq;
            for (int k{ns}; k <= mm; k++) {
              b_A[k - 1] = nrm * b_A[k - 1];
            }
          } else {
            mm = ns + nmq;
            for (int k{ns}; k <= mm; k++) {
              b_A[k - 1] = b_A[k - 1] / s[q];
            }
          }
          b_A[ns - 1] = b_A[ns - 1] + 1.0;
          s[q] = -s[q];
        } else {
          s[q] = 0.0;
        }
      }
      for (jj = qp1; jj < 4; jj++) {
        qjj = q + n * (jj - 1);
        if (apply_transform) {
          nrm = 0.0;
          if (nmq + 1 >= 1) {
            for (int k{0}; k <= nmq; k++) {
              nrm += b_A[(ns + k) - 1] * b_A[qjj + k];
            }
          }
          nrm = -(nrm / b_A[q + b_A.size(0) * q]);
          if ((nmq + 1 >= 1) && (!(nrm == 0.0))) {
            for (int k{0}; k <= nmq; k++) {
              mm = qjj + k;
              b_A[mm] = b_A[mm] + nrm * b_A[(ns + k) - 1];
            }
          }
        }
        e[jj - 1] = b_A[qjj];
      }
      if (q + 1 <= 1) {
        nrm = blas::xnrm2(e);
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
          if (n >= 2) {
            for (jj = qp1; jj <= n; jj++) {
              work[jj - 1] = 0.0;
            }
            for (jj = qp1; jj < 4; jj++) {
              nrm = e[jj - 1];
              ns = n * (jj - 1) + 2;
              if ((nmq >= 1) && (!(nrm == 0.0))) {
                mm = nmq - 1;
                for (int k{0}; k <= mm; k++) {
                  iter = k + 1;
                  work[iter] = work[iter] + nrm * b_A[(ns + k) - 1];
                }
              }
            }
            for (jj = qp1; jj < 4; jj++) {
              nrm = -e[jj - 1] / e[1];
              ns = n * (jj - 1) + 2;
              if ((nmq >= 1) && (!(nrm == 0.0))) {
                mm = nmq - 1;
                for (int k{0}; k <= mm; k++) {
                  iter = (ns + k) - 1;
                  b_A[iter] = b_A[iter] + nrm * work[k + 1];
                }
              }
            }
          }
        }
      }
    }
    if (A.size(0) + 1 >= 3) {
      nmq = 2;
    } else {
      nmq = A.size(0);
    }
    if (nct < 3) {
      s[nct] = b_A[nct + b_A.size(0) * nct];
    }
    if (A.size(0) < nmq + 1) {
      s[nmq] = 0.0;
    }
    if (nmq + 1 > 2) {
      e[1] = b_A[b_A.size(0) * nmq + 1];
    }
    e[nmq] = 0.0;
    mm = nmq;
    iter = 0;
    snorm = 0.0;
    for (int q{0}; q <= nmq; q++) {
      nrm = s[q];
      if (s[q] != 0.0) {
        rt = std::abs(s[q]);
        r = s[q] / rt;
        nrm = rt;
        s[q] = rt;
        if (q + 1 < nmq + 1) {
          e[q] /= r;
        }
      }
      if ((q + 1 < nmq + 1) && (e[q] != 0.0)) {
        rt = std::abs(e[q]);
        r = rt / e[q];
        e[q] = rt;
        s[q + 1] = s[q + 1] * r;
      }
      snorm = std::fmax(snorm, std::fmax(std::abs(nrm), std::abs(e[q])));
    }
    while ((nmq + 1 > 0) && (iter < 75)) {
      bool exitg1;
      jj = nmq;
      exitg1 = false;
      while (!(exitg1 || (jj == 0))) {
        nrm = std::abs(e[jj - 1]);
        if ((nrm <= 2.2204460492503131E-16 *
                        (std::abs(s[jj - 1]) + std::abs(s[jj]))) ||
            (nrm <= 1.0020841800044864E-292) ||
            ((iter > 20) && (nrm <= 2.2204460492503131E-16 * snorm))) {
          e[jj - 1] = 0.0;
          exitg1 = true;
        } else {
          jj--;
        }
      }
      if (jj == nmq) {
        ns = 4;
      } else {
        qjj = nmq + 1;
        ns = nmq + 1;
        exitg1 = false;
        while ((!exitg1) && (ns >= jj)) {
          qjj = ns;
          if (ns == jj) {
            exitg1 = true;
          } else {
            nrm = 0.0;
            if (ns < nmq + 1) {
              nrm = std::abs(e[ns - 1]);
            }
            if (ns > jj + 1) {
              nrm += std::abs(e[ns - 2]);
            }
            rt = std::abs(s[ns - 1]);
            if ((rt <= 2.2204460492503131E-16 * nrm) ||
                (rt <= 1.0020841800044864E-292)) {
              s[ns - 1] = 0.0;
              exitg1 = true;
            } else {
              ns--;
            }
          }
        }
        if (qjj == jj) {
          ns = 3;
        } else if (qjj == nmq + 1) {
          ns = 1;
        } else {
          ns = 2;
          jj = qjj;
        }
      }
      switch (ns) {
      case 1:
        rt = e[nmq - 1];
        e[nmq - 1] = 0.0;
        for (int k{nmq}; k >= jj + 1; k--) {
          blas::xrotg(&s[k - 1], &rt, &sqds, &sm);
          if (k > jj + 1) {
            rt = -sm * e[0];
            e[0] *= sqds;
          }
        }
        break;
      case 2:
        rt = e[jj - 1];
        e[jj - 1] = 0.0;
        for (int k{jj + 1}; k <= nmq + 1; k++) {
          blas::xrotg(&s[k - 1], &rt, &sqds, &sm);
          r = e[k - 1];
          rt = -sm * r;
          e[k - 1] = r * sqds;
        }
        break;
      case 3: {
        double scale;
        nrm = s[nmq - 1];
        rt = e[nmq - 1];
        scale = std::fmax(
            std::fmax(std::fmax(std::fmax(std::abs(s[nmq]), std::abs(nrm)),
                                std::abs(rt)),
                      std::abs(s[jj])),
            std::abs(e[jj]));
        sm = s[nmq] / scale;
        nrm /= scale;
        rt /= scale;
        sqds = s[jj] / scale;
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
        nrm = sqds * (e[jj] / scale);
        for (int k{jj + 1}; k <= nmq; k++) {
          blas::xrotg(&rt, &nrm, &sqds, &sm);
          if (k > jj + 1) {
            e[k - 2] = rt;
          }
          nrm = e[k - 1];
          r = s[k - 1];
          e[k - 1] = sqds * nrm - sm * r;
          rt = sm * s[k];
          s[k] = s[k] * sqds;
          s[k - 1] = sqds * r + sm * nrm;
          blas::xrotg(&s[k - 1], &rt, &sqds, &sm);
          rt = sqds * e[k - 1] + sm * s[k];
          s[k] = -sm * e[k - 1] + sqds * s[k];
          nrm = sm * e[k];
          e[k] *= sqds;
        }
        e[nmq - 1] = rt;
        iter++;
      } break;
      default:
        if (s[jj] < 0.0) {
          s[jj] = -s[jj];
        }
        qp1 = jj + 1;
        while ((jj + 1 < mm + 1) && (s[jj] < s[qp1])) {
          rt = s[jj];
          s[jj] = s[qp1];
          s[qp1] = rt;
          jj = qp1;
          qp1++;
        }
        iter = 0;
        nmq--;
        break;
      }
    }
  }
  U.set_size(minnp);
  for (int k{0}; k < minnp; k++) {
    U[k] = s[k];
  }
}

} // namespace internal
} // namespace coder

// End of code generation (svd.cpp)
