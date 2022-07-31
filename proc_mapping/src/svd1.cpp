//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: svd1.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 31-Jul-2022 13:03:34
//

// Include Files
#include "svd1.h"
#include "rt_nonfinite.h"
#include "xaxpy.h"
#include "xdotc.h"
#include "xnrm2.h"
#include "xrot.h"
#include "xrotg.h"
#include "xswap.h"
#include <cmath>
#include <string.h>

// Function Definitions
//
// Arguments    : const float A[9]
//                float U[3]
// Return Type  : void
//
namespace coder {
namespace internal {
void b_svd(const float A[9], float U[3])
{
  float b_A[9];
  float e[3];
  float s[3];
  float work[3];
  float nrm;
  float rt;
  float sm;
  float snorm;
  float sqds;
  float ztest;
  int ii;
  int iter;
  int m;
  int qjj;
  int qp1;
  int qq;
  int qq_tmp;
  for (qjj = 0; qjj < 9; qjj++) {
    b_A[qjj] = A[qjj];
  }
  s[0] = 0.0F;
  e[0] = 0.0F;
  work[0] = 0.0F;
  s[1] = 0.0F;
  e[1] = 0.0F;
  work[1] = 0.0F;
  s[2] = 0.0F;
  e[2] = 0.0F;
  work[2] = 0.0F;
  for (int q{0}; q < 2; q++) {
    bool apply_transform;
    qp1 = q + 2;
    qq_tmp = q + 3 * q;
    qq = qq_tmp + 1;
    apply_transform = false;
    nrm = blas::xnrm2(3 - q, b_A, qq_tmp + 1);
    if (nrm > 0.0F) {
      apply_transform = true;
      if (b_A[qq_tmp] < 0.0F) {
        ztest = -nrm;
      } else {
        ztest = nrm;
      }
      s[q] = ztest;
      if (std::abs(ztest) >= 9.86076132E-32F) {
        nrm = 1.0F / ztest;
        qjj = (qq_tmp - q) + 3;
        for (int k{qq}; k <= qjj; k++) {
          b_A[k - 1] *= nrm;
        }
      } else {
        qjj = (qq_tmp - q) + 3;
        for (int k{qq}; k <= qjj; k++) {
          b_A[k - 1] /= s[q];
        }
      }
      b_A[qq_tmp]++;
      s[q] = -s[q];
    } else {
      s[q] = 0.0F;
    }
    for (m = qp1; m < 4; m++) {
      qjj = q + 3 * (m - 1);
      if (apply_transform) {
        qq = 2 - q;
        nrm = 0.0F;
        for (int k{0}; k <= qq; k++) {
          nrm += b_A[qq_tmp + k] * b_A[qjj + k];
        }
        nrm = -(nrm / b_A[qq_tmp]);
        if (!(nrm == 0.0F)) {
          for (int k{0}; k <= qq; k++) {
            iter = qjj + k;
            b_A[iter] += nrm * b_A[qq_tmp + k];
          }
        }
      }
      e[m - 1] = b_A[qjj];
    }
    if (q + 1 <= 1) {
      nrm = blas::xnrm2(e);
      if (nrm == 0.0F) {
        e[0] = 0.0F;
      } else {
        if (e[1] < 0.0F) {
          e[0] = -nrm;
        } else {
          e[0] = nrm;
        }
        nrm = e[0];
        if (std::abs(e[0]) >= 9.86076132E-32F) {
          nrm = 1.0F / e[0];
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
          work[ii - 1] = 0.0F;
        }
        for (m = qp1; m < 4; m++) {
          ztest = e[m - 1];
          if (!(ztest == 0.0F)) {
            qjj = q + 3 * (m - 1);
            work[q + 1] += ztest * b_A[qjj + 1];
            work[q + 2] += ztest * b_A[qjj + 2];
          }
        }
        for (m = qp1; m < 4; m++) {
          nrm = -e[m - 1] / e[1];
          if (!(nrm == 0.0F)) {
            qjj = (q + 3 * (m - 1)) + 1;
            b_A[qjj] += nrm * work[q + 1];
            b_A[qjj + 1] += nrm * work[q + 2];
          }
        }
      }
    }
  }
  m = 1;
  s[2] = b_A[8];
  e[1] = b_A[7];
  e[2] = 0.0F;
  iter = 0;
  ztest = s[0];
  if (s[0] != 0.0F) {
    rt = std::abs(s[0]);
    nrm = s[0] / rt;
    ztest = rt;
    s[0] = rt;
    e[0] /= nrm;
  }
  if (e[0] != 0.0F) {
    rt = std::abs(e[0]);
    nrm = e[0];
    e[0] = rt;
    s[1] *= rt / nrm;
  }
  snorm = std::fmax(std::abs(ztest), e[0]);
  ztest = s[1];
  if (s[1] != 0.0F) {
    rt = std::abs(s[1]);
    nrm = s[1] / rt;
    ztest = rt;
    s[1] = rt;
    e[1] = b_A[7] / nrm;
  }
  if (e[1] != 0.0F) {
    rt = std::abs(e[1]);
    nrm = e[1];
    e[1] = rt;
    s[2] = b_A[8] * (rt / nrm);
  }
  snorm = std::fmax(snorm, std::fmax(std::abs(ztest), e[1]));
  ztest = s[2];
  if (s[2] != 0.0F) {
    rt = std::abs(s[2]);
    ztest = rt;
    s[2] = rt;
  }
  snorm = std::fmax(snorm, std::fmax(std::abs(ztest), 0.0F));
  while ((m + 2 > 0) && (iter < 75)) {
    bool exitg1;
    qq_tmp = m + 1;
    ii = m + 1;
    exitg1 = false;
    while (!(exitg1 || (ii == 0))) {
      nrm = std::abs(e[ii - 1]);
      if ((nrm <= 1.1920929E-7F * (std::abs(s[ii - 1]) + std::abs(s[ii]))) ||
          (nrm <= 9.86076132E-32F) ||
          ((iter > 20) && (nrm <= 1.1920929E-7F * snorm))) {
        e[ii - 1] = 0.0F;
        exitg1 = true;
      } else {
        ii--;
      }
    }
    if (ii == m + 1) {
      qjj = 4;
    } else {
      qq = m + 2;
      qjj = m + 2;
      exitg1 = false;
      while ((!exitg1) && (qjj >= ii)) {
        qq = qjj;
        if (qjj == ii) {
          exitg1 = true;
        } else {
          nrm = 0.0F;
          if (qjj < m + 2) {
            nrm = std::abs(e[qjj - 1]);
          }
          if (qjj > ii + 1) {
            nrm += std::abs(e[qjj - 2]);
          }
          ztest = std::abs(s[qjj - 1]);
          if ((ztest <= 1.1920929E-7F * nrm) || (ztest <= 9.86076132E-32F)) {
            s[qjj - 1] = 0.0F;
            exitg1 = true;
          } else {
            qjj--;
          }
        }
      }
      if (qq == ii) {
        qjj = 3;
      } else if (qq == m + 2) {
        qjj = 1;
      } else {
        qjj = 2;
        ii = qq;
      }
    }
    switch (qjj) {
    case 1:
      ztest = e[m];
      e[m] = 0.0F;
      for (int k{qq_tmp}; k >= ii + 1; k--) {
        blas::xrotg(&s[k - 1], &ztest, &sm, &sqds);
        if (k > ii + 1) {
          ztest = -sqds * e[0];
          e[0] *= sm;
        }
      }
      break;
    case 2:
      ztest = e[ii - 1];
      e[ii - 1] = 0.0F;
      for (int k{ii + 1}; k <= m + 2; k++) {
        blas::xrotg(&s[k - 1], &ztest, &sm, &sqds);
        rt = e[k - 1];
        ztest = -sqds * rt;
        e[k - 1] = rt * sm;
      }
      break;
    case 3: {
      float scale;
      nrm = s[m + 1];
      scale = std::fmax(
          std::fmax(std::fmax(std::fmax(std::abs(nrm), std::abs(s[m])),
                              std::abs(e[m])),
                    std::abs(s[ii])),
          std::abs(e[ii]));
      sm = nrm / scale;
      nrm = s[m] / scale;
      ztest = e[m] / scale;
      sqds = s[ii] / scale;
      rt = ((nrm + sm) * (nrm - sm) + ztest * ztest) / 2.0F;
      nrm = sm * ztest;
      nrm *= nrm;
      if ((rt != 0.0F) || (nrm != 0.0F)) {
        ztest = std::sqrt(rt * rt + nrm);
        if (rt < 0.0F) {
          ztest = -ztest;
        }
        ztest = nrm / (rt + ztest);
      } else {
        ztest = 0.0F;
      }
      ztest += (sqds + sm) * (sqds - sm);
      nrm = sqds * (e[ii] / scale);
      for (int k{ii + 1}; k <= qq_tmp; k++) {
        blas::xrotg(&ztest, &nrm, &sm, &sqds);
        if (k > ii + 1) {
          e[0] = ztest;
        }
        nrm = e[k - 1];
        rt = s[k - 1];
        e[k - 1] = sm * nrm - sqds * rt;
        ztest = sqds * s[k];
        s[k] *= sm;
        s[k - 1] = sm * rt + sqds * nrm;
        blas::xrotg(&s[k - 1], &ztest, &sm, &sqds);
        ztest = sm * e[k - 1] + sqds * s[k];
        s[k] = -sqds * e[k - 1] + sm * s[k];
        nrm = sqds * e[k];
        e[k] *= sm;
      }
      e[m] = ztest;
      iter++;
    } break;
    default:
      if (s[ii] < 0.0F) {
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
}

//
// Arguments    : const float A[9]
//                float U[9]
//                float s[3]
//                float V[9]
// Return Type  : void
//
void b_svd(const float A[9], float U[9], float s[3], float V[9])
{
  float b_A[9];
  float b_s[3];
  float e[3];
  float work[3];
  float nrm;
  float rt;
  float sm;
  float snorm;
  float sqds;
  int ii;
  int kase;
  int m;
  int qjj;
  int qp1;
  int qq;
  b_s[0] = 0.0F;
  e[0] = 0.0F;
  work[0] = 0.0F;
  b_s[1] = 0.0F;
  e[1] = 0.0F;
  work[1] = 0.0F;
  b_s[2] = 0.0F;
  e[2] = 0.0F;
  work[2] = 0.0F;
  for (kase = 0; kase < 9; kase++) {
    b_A[kase] = A[kase];
    U[kase] = 0.0F;
    V[kase] = 0.0F;
  }
  for (int q{0}; q < 2; q++) {
    bool apply_transform;
    qp1 = q + 2;
    qq = (q + 3 * q) + 1;
    apply_transform = false;
    nrm = blas::xnrm2(3 - q, b_A, qq);
    if (nrm > 0.0F) {
      apply_transform = true;
      if (b_A[qq - 1] < 0.0F) {
        nrm = -nrm;
      }
      b_s[q] = nrm;
      if (std::abs(nrm) >= 9.86076132E-32F) {
        nrm = 1.0F / nrm;
        kase = (qq - q) + 2;
        for (int k{qq}; k <= kase; k++) {
          b_A[k - 1] *= nrm;
        }
      } else {
        kase = (qq - q) + 2;
        for (int k{qq}; k <= kase; k++) {
          b_A[k - 1] /= b_s[q];
        }
      }
      b_A[qq - 1]++;
      b_s[q] = -b_s[q];
    } else {
      b_s[q] = 0.0F;
    }
    for (kase = qp1; kase < 4; kase++) {
      qjj = q + 3 * (kase - 1);
      if (apply_transform) {
        blas::xaxpy(
            3 - q,
            -(blas::xdotc(3 - q, b_A, qq, b_A, qjj + 1) / b_A[q + 3 * q]), qq,
            b_A, qjj + 1);
      }
      e[kase - 1] = b_A[qjj];
    }
    for (ii = q + 1; ii < 4; ii++) {
      kase = (ii + 3 * q) - 1;
      U[kase] = b_A[kase];
    }
    if (q + 1 <= 1) {
      nrm = blas::xnrm2(e);
      if (nrm == 0.0F) {
        e[0] = 0.0F;
      } else {
        if (e[1] < 0.0F) {
          e[0] = -nrm;
        } else {
          e[0] = nrm;
        }
        nrm = e[0];
        if (std::abs(e[0]) >= 9.86076132E-32F) {
          nrm = 1.0F / e[0];
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
          work[ii - 1] = 0.0F;
        }
        for (kase = qp1; kase < 4; kase++) {
          blas::xaxpy(e[kase - 1], b_A, 3 * (kase - 1) + 2, work);
        }
        for (kase = qp1; kase < 4; kase++) {
          blas::xaxpy(-e[kase - 1] / e[1], work, b_A, 3 * (kase - 1) + 2);
        }
      }
      for (ii = qp1; ii < 4; ii++) {
        V[ii - 1] = e[ii - 1];
      }
    }
  }
  m = 1;
  b_s[2] = b_A[8];
  e[1] = b_A[7];
  e[2] = 0.0F;
  U[6] = 0.0F;
  U[7] = 0.0F;
  U[8] = 1.0F;
  for (int q{1}; q >= 0; q--) {
    qp1 = q + 2;
    qq = q + 3 * q;
    if (b_s[q] != 0.0F) {
      for (kase = qp1; kase < 4; kase++) {
        qjj = (q + 3 * (kase - 1)) + 1;
        blas::xaxpy(3 - q, -(blas::xdotc(3 - q, U, qq + 1, U, qjj) / U[qq]),
                    qq + 1, U, qjj);
      }
      for (ii = q + 1; ii < 4; ii++) {
        kase = (ii + 3 * q) - 1;
        U[kase] = -U[kase];
      }
      U[qq]++;
      if (q - 1 >= 0) {
        U[3 * q] = 0.0F;
      }
    } else {
      U[3 * q] = 0.0F;
      U[3 * q + 1] = 0.0F;
      U[3 * q + 2] = 0.0F;
      U[qq] = 1.0F;
    }
  }
  for (int q{2}; q >= 0; q--) {
    if ((q + 1 <= 1) && (e[0] != 0.0F)) {
      blas::xaxpy(2, -(blas::xdotc(2, V, 2, V, 5) / V[1]), 2, V, 5);
      blas::xaxpy(2, -(blas::xdotc(2, V, 2, V, 8) / V[1]), 2, V, 8);
    }
    V[3 * q] = 0.0F;
    V[3 * q + 1] = 0.0F;
    V[3 * q + 2] = 0.0F;
    V[q + 3 * q] = 1.0F;
  }
  qq = 0;
  snorm = 0.0F;
  for (int q{0}; q < 3; q++) {
    nrm = b_s[q];
    if (nrm != 0.0F) {
      rt = std::abs(nrm);
      nrm /= rt;
      b_s[q] = rt;
      if (q + 1 < 3) {
        e[q] /= nrm;
      }
      qjj = 3 * q;
      kase = qjj + 3;
      for (int k{qjj + 1}; k <= kase; k++) {
        U[k - 1] *= nrm;
      }
    }
    if (q + 1 < 3) {
      nrm = e[q];
      if (nrm != 0.0F) {
        rt = std::abs(nrm);
        nrm = rt / nrm;
        e[q] = rt;
        b_s[q + 1] *= nrm;
        qjj = 3 * (q + 1);
        kase = qjj + 3;
        for (int k{qjj + 1}; k <= kase; k++) {
          V[k - 1] *= nrm;
        }
      }
    }
    snorm = std::fmax(snorm, std::fmax(std::abs(b_s[q]), std::abs(e[q])));
  }
  while ((m + 2 > 0) && (qq < 75)) {
    bool exitg1;
    qp1 = m + 1;
    ii = m + 1;
    exitg1 = false;
    while (!(exitg1 || (ii == 0))) {
      nrm = std::abs(e[ii - 1]);
      if ((nrm <=
           1.1920929E-7F * (std::abs(b_s[ii - 1]) + std::abs(b_s[ii]))) ||
          (nrm <= 9.86076132E-32F) ||
          ((qq > 20) && (nrm <= 1.1920929E-7F * snorm))) {
        e[ii - 1] = 0.0F;
        exitg1 = true;
      } else {
        ii--;
      }
    }
    if (ii == m + 1) {
      kase = 4;
    } else {
      qjj = m + 2;
      kase = m + 2;
      exitg1 = false;
      while ((!exitg1) && (kase >= ii)) {
        qjj = kase;
        if (kase == ii) {
          exitg1 = true;
        } else {
          nrm = 0.0F;
          if (kase < m + 2) {
            nrm = std::abs(e[kase - 1]);
          }
          if (kase > ii + 1) {
            nrm += std::abs(e[kase - 2]);
          }
          rt = std::abs(b_s[kase - 1]);
          if ((rt <= 1.1920929E-7F * nrm) || (rt <= 9.86076132E-32F)) {
            b_s[kase - 1] = 0.0F;
            exitg1 = true;
          } else {
            kase--;
          }
        }
      }
      if (qjj == ii) {
        kase = 3;
      } else if (qjj == m + 2) {
        kase = 1;
      } else {
        kase = 2;
        ii = qjj;
      }
    }
    switch (kase) {
    case 1:
      rt = e[m];
      e[m] = 0.0F;
      for (int k{qp1}; k >= ii + 1; k--) {
        blas::xrotg(&b_s[k - 1], &rt, &sm, &sqds);
        if (k > ii + 1) {
          rt = -sqds * e[0];
          e[0] *= sm;
        }
        blas::xrot(V, 3 * (k - 1) + 1, 3 * (m + 1) + 1, sm, sqds);
      }
      break;
    case 2: {
      rt = e[ii - 1];
      e[ii - 1] = 0.0F;
      for (int k{ii + 1}; k <= m + 2; k++) {
        float b;
        blas::xrotg(&b_s[k - 1], &rt, &sm, &sqds);
        b = e[k - 1];
        rt = -sqds * b;
        e[k - 1] = b * sm;
        blas::xrot(U, 3 * (k - 1) + 1, 3 * (ii - 1) + 1, sm, sqds);
      }
    } break;
    case 3: {
      float b;
      float scale;
      nrm = b_s[m + 1];
      scale = std::fmax(
          std::fmax(std::fmax(std::fmax(std::abs(nrm), std::abs(b_s[m])),
                              std::abs(e[m])),
                    std::abs(b_s[ii])),
          std::abs(e[ii]));
      sm = nrm / scale;
      nrm = b_s[m] / scale;
      rt = e[m] / scale;
      sqds = b_s[ii] / scale;
      b = ((nrm + sm) * (nrm - sm) + rt * rt) / 2.0F;
      nrm = sm * rt;
      nrm *= nrm;
      if ((b != 0.0F) || (nrm != 0.0F)) {
        rt = std::sqrt(b * b + nrm);
        if (b < 0.0F) {
          rt = -rt;
        }
        rt = nrm / (b + rt);
      } else {
        rt = 0.0F;
      }
      rt += (sqds + sm) * (sqds - sm);
      nrm = sqds * (e[ii] / scale);
      for (int k{ii + 1}; k <= qp1; k++) {
        blas::xrotg(&rt, &nrm, &sm, &sqds);
        if (k > ii + 1) {
          e[0] = rt;
        }
        nrm = e[k - 1];
        b = b_s[k - 1];
        e[k - 1] = sm * nrm - sqds * b;
        rt = sqds * b_s[k];
        b_s[k] *= sm;
        blas::xrot(V, 3 * (k - 1) + 1, 3 * k + 1, sm, sqds);
        b_s[k - 1] = sm * b + sqds * nrm;
        blas::xrotg(&b_s[k - 1], &rt, &sm, &sqds);
        rt = sm * e[k - 1] + sqds * b_s[k];
        b_s[k] = -sqds * e[k - 1] + sm * b_s[k];
        nrm = sqds * e[k];
        e[k] *= sm;
        blas::xrot(U, 3 * (k - 1) + 1, 3 * k + 1, sm, sqds);
      }
      e[m] = rt;
      qq++;
    } break;
    default:
      if (b_s[ii] < 0.0F) {
        b_s[ii] = -b_s[ii];
        qjj = 3 * ii;
        kase = qjj + 3;
        for (int k{qjj + 1}; k <= kase; k++) {
          V[k - 1] = -V[k - 1];
        }
      }
      qp1 = ii + 1;
      while ((ii + 1 < 3) && (b_s[ii] < b_s[qp1])) {
        rt = b_s[ii];
        b_s[ii] = b_s[qp1];
        b_s[qp1] = rt;
        blas::xswap(V, 3 * ii + 1, 3 * (ii + 1) + 1);
        blas::xswap(U, 3 * ii + 1, 3 * (ii + 1) + 1);
        ii = qp1;
        qp1++;
      }
      qq = 0;
      m--;
      break;
    }
  }
  s[0] = b_s[0];
  s[1] = b_s[1];
  s[2] = b_s[2];
}

//
// Arguments    : const double A[9]
//                double U[9]
//                double s[3]
//                double V[9]
// Return Type  : void
//
void b_svd(const double A[9], double U[9], double s[3], double V[9])
{
  double b_A[9];
  double b_s[3];
  double e[3];
  double work[3];
  double nrm;
  double rt;
  double sm;
  double snorm;
  double sqds;
  int ii;
  int kase;
  int m;
  int qjj;
  int qp1;
  int qq;
  b_s[0] = 0.0;
  e[0] = 0.0;
  work[0] = 0.0;
  b_s[1] = 0.0;
  e[1] = 0.0;
  work[1] = 0.0;
  b_s[2] = 0.0;
  e[2] = 0.0;
  work[2] = 0.0;
  for (kase = 0; kase < 9; kase++) {
    b_A[kase] = A[kase];
    U[kase] = 0.0;
    V[kase] = 0.0;
  }
  for (int q{0}; q < 2; q++) {
    bool apply_transform;
    qp1 = q + 2;
    qq = (q + 3 * q) + 1;
    apply_transform = false;
    nrm = blas::xnrm2(3 - q, b_A, qq);
    if (nrm > 0.0) {
      apply_transform = true;
      if (b_A[qq - 1] < 0.0) {
        nrm = -nrm;
      }
      b_s[q] = nrm;
      if (std::abs(nrm) >= 1.0020841800044864E-292) {
        nrm = 1.0 / nrm;
        kase = (qq - q) + 2;
        for (int k{qq}; k <= kase; k++) {
          b_A[k - 1] *= nrm;
        }
      } else {
        kase = (qq - q) + 2;
        for (int k{qq}; k <= kase; k++) {
          b_A[k - 1] /= b_s[q];
        }
      }
      b_A[qq - 1]++;
      b_s[q] = -b_s[q];
    } else {
      b_s[q] = 0.0;
    }
    for (kase = qp1; kase < 4; kase++) {
      qjj = q + 3 * (kase - 1);
      if (apply_transform) {
        blas::xaxpy(
            3 - q,
            -(blas::xdotc(3 - q, b_A, qq, b_A, qjj + 1) / b_A[q + 3 * q]), qq,
            b_A, qjj + 1);
      }
      e[kase - 1] = b_A[qjj];
    }
    for (ii = q + 1; ii < 4; ii++) {
      kase = (ii + 3 * q) - 1;
      U[kase] = b_A[kase];
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
        for (ii = qp1; ii < 4; ii++) {
          work[ii - 1] = 0.0;
        }
        for (kase = qp1; kase < 4; kase++) {
          blas::xaxpy(e[kase - 1], b_A, 3 * (kase - 1) + 2, work);
        }
        for (kase = qp1; kase < 4; kase++) {
          blas::xaxpy(-e[kase - 1] / e[1], work, b_A, 3 * (kase - 1) + 2);
        }
      }
      for (ii = qp1; ii < 4; ii++) {
        V[ii - 1] = e[ii - 1];
      }
    }
  }
  m = 1;
  b_s[2] = b_A[8];
  e[1] = b_A[7];
  e[2] = 0.0;
  U[6] = 0.0;
  U[7] = 0.0;
  U[8] = 1.0;
  for (int q{1}; q >= 0; q--) {
    qp1 = q + 2;
    qq = q + 3 * q;
    if (b_s[q] != 0.0) {
      for (kase = qp1; kase < 4; kase++) {
        qjj = (q + 3 * (kase - 1)) + 1;
        blas::xaxpy(3 - q, -(blas::xdotc(3 - q, U, qq + 1, U, qjj) / U[qq]),
                    qq + 1, U, qjj);
      }
      for (ii = q + 1; ii < 4; ii++) {
        kase = (ii + 3 * q) - 1;
        U[kase] = -U[kase];
      }
      U[qq]++;
      if (q - 1 >= 0) {
        U[3 * q] = 0.0;
      }
    } else {
      U[3 * q] = 0.0;
      U[3 * q + 1] = 0.0;
      U[3 * q + 2] = 0.0;
      U[qq] = 1.0;
    }
  }
  for (int q{2}; q >= 0; q--) {
    if ((q + 1 <= 1) && (e[0] != 0.0)) {
      blas::xaxpy(2, -(blas::xdotc(2, V, 2, V, 5) / V[1]), 2, V, 5);
      blas::xaxpy(2, -(blas::xdotc(2, V, 2, V, 8) / V[1]), 2, V, 8);
    }
    V[3 * q] = 0.0;
    V[3 * q + 1] = 0.0;
    V[3 * q + 2] = 0.0;
    V[q + 3 * q] = 1.0;
  }
  qq = 0;
  snorm = 0.0;
  for (int q{0}; q < 3; q++) {
    nrm = b_s[q];
    if (nrm != 0.0) {
      rt = std::abs(nrm);
      nrm /= rt;
      b_s[q] = rt;
      if (q + 1 < 3) {
        e[q] /= nrm;
      }
      qjj = 3 * q;
      kase = qjj + 3;
      for (int k{qjj + 1}; k <= kase; k++) {
        U[k - 1] *= nrm;
      }
    }
    if (q + 1 < 3) {
      nrm = e[q];
      if (nrm != 0.0) {
        rt = std::abs(nrm);
        nrm = rt / nrm;
        e[q] = rt;
        b_s[q + 1] *= nrm;
        qjj = 3 * (q + 1);
        kase = qjj + 3;
        for (int k{qjj + 1}; k <= kase; k++) {
          V[k - 1] *= nrm;
        }
      }
    }
    snorm = std::fmax(snorm, std::fmax(std::abs(b_s[q]), std::abs(e[q])));
  }
  while ((m + 2 > 0) && (qq < 75)) {
    bool exitg1;
    qp1 = m + 1;
    ii = m + 1;
    exitg1 = false;
    while (!(exitg1 || (ii == 0))) {
      nrm = std::abs(e[ii - 1]);
      if ((nrm <= 2.2204460492503131E-16 *
                      (std::abs(b_s[ii - 1]) + std::abs(b_s[ii]))) ||
          (nrm <= 1.0020841800044864E-292) ||
          ((qq > 20) && (nrm <= 2.2204460492503131E-16 * snorm))) {
        e[ii - 1] = 0.0;
        exitg1 = true;
      } else {
        ii--;
      }
    }
    if (ii == m + 1) {
      kase = 4;
    } else {
      qjj = m + 2;
      kase = m + 2;
      exitg1 = false;
      while ((!exitg1) && (kase >= ii)) {
        qjj = kase;
        if (kase == ii) {
          exitg1 = true;
        } else {
          nrm = 0.0;
          if (kase < m + 2) {
            nrm = std::abs(e[kase - 1]);
          }
          if (kase > ii + 1) {
            nrm += std::abs(e[kase - 2]);
          }
          rt = std::abs(b_s[kase - 1]);
          if ((rt <= 2.2204460492503131E-16 * nrm) ||
              (rt <= 1.0020841800044864E-292)) {
            b_s[kase - 1] = 0.0;
            exitg1 = true;
          } else {
            kase--;
          }
        }
      }
      if (qjj == ii) {
        kase = 3;
      } else if (qjj == m + 2) {
        kase = 1;
      } else {
        kase = 2;
        ii = qjj;
      }
    }
    switch (kase) {
    case 1:
      rt = e[m];
      e[m] = 0.0;
      for (int k{qp1}; k >= ii + 1; k--) {
        blas::xrotg(&b_s[k - 1], &rt, &sm, &sqds);
        if (k > ii + 1) {
          rt = -sqds * e[0];
          e[0] *= sm;
        }
        blas::xrot(V, 3 * (k - 1) + 1, 3 * (m + 1) + 1, sm, sqds);
      }
      break;
    case 2: {
      rt = e[ii - 1];
      e[ii - 1] = 0.0;
      for (int k{ii + 1}; k <= m + 2; k++) {
        double b;
        blas::xrotg(&b_s[k - 1], &rt, &sm, &sqds);
        b = e[k - 1];
        rt = -sqds * b;
        e[k - 1] = b * sm;
        blas::xrot(U, 3 * (k - 1) + 1, 3 * (ii - 1) + 1, sm, sqds);
      }
    } break;
    case 3: {
      double b;
      double scale;
      nrm = b_s[m + 1];
      scale = std::fmax(
          std::fmax(std::fmax(std::fmax(std::abs(nrm), std::abs(b_s[m])),
                              std::abs(e[m])),
                    std::abs(b_s[ii])),
          std::abs(e[ii]));
      sm = nrm / scale;
      nrm = b_s[m] / scale;
      rt = e[m] / scale;
      sqds = b_s[ii] / scale;
      b = ((nrm + sm) * (nrm - sm) + rt * rt) / 2.0;
      nrm = sm * rt;
      nrm *= nrm;
      if ((b != 0.0) || (nrm != 0.0)) {
        rt = std::sqrt(b * b + nrm);
        if (b < 0.0) {
          rt = -rt;
        }
        rt = nrm / (b + rt);
      } else {
        rt = 0.0;
      }
      rt += (sqds + sm) * (sqds - sm);
      nrm = sqds * (e[ii] / scale);
      for (int k{ii + 1}; k <= qp1; k++) {
        blas::xrotg(&rt, &nrm, &sm, &sqds);
        if (k > ii + 1) {
          e[0] = rt;
        }
        nrm = e[k - 1];
        b = b_s[k - 1];
        e[k - 1] = sm * nrm - sqds * b;
        rt = sqds * b_s[k];
        b_s[k] *= sm;
        blas::xrot(V, 3 * (k - 1) + 1, 3 * k + 1, sm, sqds);
        b_s[k - 1] = sm * b + sqds * nrm;
        blas::xrotg(&b_s[k - 1], &rt, &sm, &sqds);
        rt = sm * e[k - 1] + sqds * b_s[k];
        b_s[k] = -sqds * e[k - 1] + sm * b_s[k];
        nrm = sqds * e[k];
        e[k] *= sm;
        blas::xrot(U, 3 * (k - 1) + 1, 3 * k + 1, sm, sqds);
      }
      e[m] = rt;
      qq++;
    } break;
    default:
      if (b_s[ii] < 0.0) {
        b_s[ii] = -b_s[ii];
        qjj = 3 * ii;
        kase = qjj + 3;
        for (int k{qjj + 1}; k <= kase; k++) {
          V[k - 1] = -V[k - 1];
        }
      }
      qp1 = ii + 1;
      while ((ii + 1 < 3) && (b_s[ii] < b_s[qp1])) {
        rt = b_s[ii];
        b_s[ii] = b_s[qp1];
        b_s[qp1] = rt;
        blas::xswap(V, 3 * ii + 1, 3 * (ii + 1) + 1);
        blas::xswap(U, 3 * ii + 1, 3 * (ii + 1) + 1);
        ii = qp1;
        qp1++;
      }
      qq = 0;
      m--;
      break;
    }
  }
  s[0] = b_s[0];
  s[1] = b_s[1];
  s[2] = b_s[2];
}

} // namespace internal
} // namespace coder

//
// File trailer for svd1.cpp
//
// [EOF]
//
