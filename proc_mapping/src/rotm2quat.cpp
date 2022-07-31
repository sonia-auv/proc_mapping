//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: rotm2quat.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 31-Jul-2022 13:03:34
//

// Include Files
#include "rotm2quat.h"
#include "proc_mapping_data.h"
#include "rt_nonfinite.h"
#include "schur.h"
#include "xzggev.h"
#include <cmath>
#include <string.h>

// Function Definitions
//
// Arguments    : const double R[9]
//                double quat[4]
// Return Type  : void
//
namespace coder {
void rotm2quat(const double R[9], double quat[4])
{
  creal_T V[16];
  creal_T alpha1[4];
  double K[16];
  double D[4];
  double K12;
  double K13;
  double K14;
  double K23;
  double K24;
  double K34;
  int i;
  int k;
  int kend_tmp;
  int sgn;
  bool exitg2;
  bool p;
  K12 = R[1] + R[3];
  K13 = R[2] + R[6];
  K14 = R[5] - R[7];
  K23 = R[5] + R[7];
  K24 = R[6] - R[2];
  K34 = R[1] - R[3];
  K[0] = ((R[0] - R[4]) - R[8]) / 3.0;
  K[4] = K12 / 3.0;
  K[8] = K13 / 3.0;
  K[12] = K14 / 3.0;
  K[1] = K12 / 3.0;
  K[5] = ((R[4] - R[0]) - R[8]) / 3.0;
  K[9] = K23 / 3.0;
  K[13] = K24 / 3.0;
  K[2] = K13 / 3.0;
  K[6] = K23 / 3.0;
  K[10] = ((R[8] - R[0]) - R[4]) / 3.0;
  K[14] = K34 / 3.0;
  K[3] = K14 / 3.0;
  K[7] = K24 / 3.0;
  K[11] = K34 / 3.0;
  K[15] = ((R[0] + R[4]) + R[8]) / 3.0;
  p = true;
  for (k = 0; k < 16; k++) {
    if ((!p) || (std::isinf(K[k]) || std::isnan(K[k]))) {
      p = false;
    }
  }
  if (!p) {
    for (i = 0; i < 16; i++) {
      V[i].re = rtNaN;
      V[i].im = 0.0;
    }
    alpha1[0].re = rtNaN;
    alpha1[1].re = rtNaN;
    alpha1[2].re = rtNaN;
    alpha1[3].re = rtNaN;
  } else {
    int exitg1;
    p = true;
    k = 0;
    exitg2 = false;
    while ((!exitg2) && (k < 4)) {
      sgn = 0;
      do {
        exitg1 = 0;
        if (sgn <= k) {
          if (!(K[sgn + (k << 2)] == K[k + (sgn << 2)])) {
            p = false;
            exitg1 = 1;
          } else {
            sgn++;
          }
        } else {
          k++;
          exitg1 = 2;
        }
      } while (exitg1 == 0);
      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
    if (p) {
      double T[16];
      double U[16];
      schur(K, U, T);
      for (i = 0; i < 16; i++) {
        V[i].re = U[i];
        V[i].im = 0.0;
      }
      alpha1[0].re = T[0];
      alpha1[1].re = T[5];
      alpha1[2].re = T[10];
      alpha1[3].re = T[15];
    } else {
      p = true;
      k = 0;
      exitg2 = false;
      while ((!exitg2) && (k < 4)) {
        sgn = 0;
        do {
          exitg1 = 0;
          if (sgn <= k) {
            if (!(K[sgn + (k << 2)] == -K[k + (sgn << 2)])) {
              p = false;
              exitg1 = 1;
            } else {
              sgn++;
            }
          } else {
            k++;
            exitg1 = 2;
          }
        } while (exitg1 == 0);
        if (exitg1 == 1) {
          exitg2 = true;
        }
      }
      if (p) {
        double T[16];
        double U[16];
        schur(K, U, T);
        sgn = 1;
        do {
          exitg1 = 0;
          if (sgn <= 4) {
            bool guard1{false};
            guard1 = false;
            if (sgn != 4) {
              K12 = T[sgn + ((sgn - 1) << 2)];
              if (K12 != 0.0) {
                K12 = std::abs(K12);
                alpha1[sgn - 1].re = 0.0;
                alpha1[sgn - 1].im = K12;
                alpha1[sgn].re = 0.0;
                alpha1[sgn].im = -K12;
                sgn += 2;
              } else {
                guard1 = true;
              }
            } else {
              guard1 = true;
            }
            if (guard1) {
              alpha1[sgn - 1].re = 0.0;
              alpha1[sgn - 1].im = 0.0;
              sgn++;
            }
          } else {
            exitg1 = 1;
          }
        } while (exitg1 == 0);
        for (i = 0; i < 16; i++) {
          V[i].re = U[i];
          V[i].im = 0.0;
        }
        k = 1;
        do {
          exitg1 = 0;
          if (k <= 4) {
            if (k != 4) {
              i = (k - 1) << 2;
              if (T[k + i] != 0.0) {
                if (T[k + ((k - 1) << 2)] < 0.0) {
                  sgn = 1;
                } else {
                  sgn = -1;
                }
                K12 = V[i].re;
                kend_tmp = k << 2;
                K14 = static_cast<double>(sgn) * V[kend_tmp].re;
                if (K14 == 0.0) {
                  V[i].re = K12 / 1.4142135623730951;
                  V[i].im = 0.0;
                } else if (K12 == 0.0) {
                  V[i].re = 0.0;
                  V[i].im = K14 / 1.4142135623730951;
                } else {
                  V[i].re = K12 / 1.4142135623730951;
                  V[i].im = K14 / 1.4142135623730951;
                }
                V[kend_tmp].re = V[i].re;
                V[kend_tmp].im = -V[i].im;
                K12 = V[i + 1].re;
                K14 = static_cast<double>(sgn) * V[kend_tmp + 1].re;
                if (K14 == 0.0) {
                  V[i + 1].re = K12 / 1.4142135623730951;
                  V[i + 1].im = 0.0;
                } else if (K12 == 0.0) {
                  V[i + 1].re = 0.0;
                  V[i + 1].im = K14 / 1.4142135623730951;
                } else {
                  V[i + 1].re = K12 / 1.4142135623730951;
                  V[i + 1].im = K14 / 1.4142135623730951;
                }
                V[kend_tmp + 1].re = V[i + 1].re;
                V[kend_tmp + 1].im = -V[i + 1].im;
                K12 = V[i + 2].re;
                K14 = static_cast<double>(sgn) * V[kend_tmp + 2].re;
                if (K14 == 0.0) {
                  V[i + 2].re = K12 / 1.4142135623730951;
                  V[i + 2].im = 0.0;
                } else if (K12 == 0.0) {
                  V[i + 2].re = 0.0;
                  V[i + 2].im = K14 / 1.4142135623730951;
                } else {
                  V[i + 2].re = K12 / 1.4142135623730951;
                  V[i + 2].im = K14 / 1.4142135623730951;
                }
                V[kend_tmp + 2].re = V[i + 2].re;
                V[kend_tmp + 2].im = -V[i + 2].im;
                K12 = V[i + 3].re;
                K14 = static_cast<double>(sgn) * V[kend_tmp + 3].re;
                if (K14 == 0.0) {
                  V[i + 3].re = K12 / 1.4142135623730951;
                  V[i + 3].im = 0.0;
                } else if (K12 == 0.0) {
                  V[i + 3].re = 0.0;
                  V[i + 3].im = K14 / 1.4142135623730951;
                } else {
                  V[i + 3].re = K12 / 1.4142135623730951;
                  V[i + 3].im = K14 / 1.4142135623730951;
                }
                V[kend_tmp + 3].re = V[i + 3].re;
                V[kend_tmp + 3].im = -V[i + 3].im;
                k += 2;
              } else {
                k++;
              }
            } else {
              k++;
            }
          } else {
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      } else {
        creal_T At[16];
        creal_T beta1[4];
        for (i = 0; i < 16; i++) {
          At[i].re = K[i];
          At[i].im = 0.0;
        }
        internal::reflapack::xzggev(At, &sgn, alpha1, beta1, V);
        for (sgn = 0; sgn <= 12; sgn += 4) {
          K23 = 0.0;
          K12 = 3.3121686421112381E-170;
          kend_tmp = sgn + 4;
          for (k = sgn + 1; k <= kend_tmp; k++) {
            K13 = std::abs(V[k - 1].re);
            if (K13 > K12) {
              K14 = K12 / K13;
              K23 = K23 * K14 * K14 + 1.0;
              K12 = K13;
            } else {
              K14 = K13 / K12;
              K23 += K14 * K14;
            }
            K13 = std::abs(V[k - 1].im);
            if (K13 > K12) {
              K14 = K12 / K13;
              K23 = K23 * K14 * K14 + 1.0;
              K12 = K13;
            } else {
              K14 = K13 / K12;
              K23 += K14 * K14;
            }
          }
          K23 = K12 * std::sqrt(K23);
          for (k = sgn + 1; k <= kend_tmp; k++) {
            K12 = V[k - 1].re;
            K14 = V[k - 1].im;
            if (K14 == 0.0) {
              K13 = K12 / K23;
              K12 = 0.0;
            } else if (K12 == 0.0) {
              K13 = 0.0;
              K12 = K14 / K23;
            } else {
              K13 = K12 / K23;
              K12 = K14 / K23;
            }
            V[k - 1].re = K13;
            V[k - 1].im = K12;
          }
        }
        if (beta1[0].im == 0.0) {
          if (alpha1[0].im == 0.0) {
            K13 = alpha1[0].re / beta1[0].re;
          } else if (alpha1[0].re == 0.0) {
            K13 = 0.0;
          } else {
            K13 = alpha1[0].re / beta1[0].re;
          }
        } else if (beta1[0].re == 0.0) {
          if (alpha1[0].re == 0.0) {
            K13 = alpha1[0].im / beta1[0].im;
          } else if (alpha1[0].im == 0.0) {
            K13 = 0.0;
          } else {
            K13 = alpha1[0].im / beta1[0].im;
          }
        } else {
          K12 = std::abs(beta1[0].re);
          K13 = std::abs(beta1[0].im);
          if (K12 > K13) {
            K12 = beta1[0].im / beta1[0].re;
            K13 = (alpha1[0].re + K12 * alpha1[0].im) /
                  (beta1[0].re + K12 * beta1[0].im);
          } else if (K13 == K12) {
            if (beta1[0].re > 0.0) {
              K13 = 0.5;
            } else {
              K13 = -0.5;
            }
            if (beta1[0].im > 0.0) {
              K14 = 0.5;
            } else {
              K14 = -0.5;
            }
            K13 = (alpha1[0].re * K13 + alpha1[0].im * K14) / K12;
          } else {
            K12 = beta1[0].re / beta1[0].im;
            K13 = (K12 * alpha1[0].re + alpha1[0].im) /
                  (beta1[0].im + K12 * beta1[0].re);
          }
        }
        alpha1[0].re = K13;
        if (beta1[1].im == 0.0) {
          if (alpha1[1].im == 0.0) {
            K13 = alpha1[1].re / beta1[1].re;
          } else if (alpha1[1].re == 0.0) {
            K13 = 0.0;
          } else {
            K13 = alpha1[1].re / beta1[1].re;
          }
        } else if (beta1[1].re == 0.0) {
          if (alpha1[1].re == 0.0) {
            K13 = alpha1[1].im / beta1[1].im;
          } else if (alpha1[1].im == 0.0) {
            K13 = 0.0;
          } else {
            K13 = alpha1[1].im / beta1[1].im;
          }
        } else {
          K12 = std::abs(beta1[1].re);
          K13 = std::abs(beta1[1].im);
          if (K12 > K13) {
            K12 = beta1[1].im / beta1[1].re;
            K13 = (alpha1[1].re + K12 * alpha1[1].im) /
                  (beta1[1].re + K12 * beta1[1].im);
          } else if (K13 == K12) {
            if (beta1[1].re > 0.0) {
              K13 = 0.5;
            } else {
              K13 = -0.5;
            }
            if (beta1[1].im > 0.0) {
              K14 = 0.5;
            } else {
              K14 = -0.5;
            }
            K13 = (alpha1[1].re * K13 + alpha1[1].im * K14) / K12;
          } else {
            K12 = beta1[1].re / beta1[1].im;
            K13 = (K12 * alpha1[1].re + alpha1[1].im) /
                  (beta1[1].im + K12 * beta1[1].re);
          }
        }
        alpha1[1].re = K13;
        if (beta1[2].im == 0.0) {
          if (alpha1[2].im == 0.0) {
            K13 = alpha1[2].re / beta1[2].re;
          } else if (alpha1[2].re == 0.0) {
            K13 = 0.0;
          } else {
            K13 = alpha1[2].re / beta1[2].re;
          }
        } else if (beta1[2].re == 0.0) {
          if (alpha1[2].re == 0.0) {
            K13 = alpha1[2].im / beta1[2].im;
          } else if (alpha1[2].im == 0.0) {
            K13 = 0.0;
          } else {
            K13 = alpha1[2].im / beta1[2].im;
          }
        } else {
          K12 = std::abs(beta1[2].re);
          K13 = std::abs(beta1[2].im);
          if (K12 > K13) {
            K12 = beta1[2].im / beta1[2].re;
            K13 = (alpha1[2].re + K12 * alpha1[2].im) /
                  (beta1[2].re + K12 * beta1[2].im);
          } else if (K13 == K12) {
            if (beta1[2].re > 0.0) {
              K13 = 0.5;
            } else {
              K13 = -0.5;
            }
            if (beta1[2].im > 0.0) {
              K14 = 0.5;
            } else {
              K14 = -0.5;
            }
            K13 = (alpha1[2].re * K13 + alpha1[2].im * K14) / K12;
          } else {
            K12 = beta1[2].re / beta1[2].im;
            K13 = (K12 * alpha1[2].re + alpha1[2].im) /
                  (beta1[2].im + K12 * beta1[2].re);
          }
        }
        alpha1[2].re = K13;
        if (beta1[3].im == 0.0) {
          if (alpha1[3].im == 0.0) {
            K13 = alpha1[3].re / beta1[3].re;
          } else if (alpha1[3].re == 0.0) {
            K13 = 0.0;
          } else {
            K13 = alpha1[3].re / beta1[3].re;
          }
        } else if (beta1[3].re == 0.0) {
          if (alpha1[3].re == 0.0) {
            K13 = alpha1[3].im / beta1[3].im;
          } else if (alpha1[3].im == 0.0) {
            K13 = 0.0;
          } else {
            K13 = alpha1[3].im / beta1[3].im;
          }
        } else {
          K12 = std::abs(beta1[3].re);
          K13 = std::abs(beta1[3].im);
          if (K12 > K13) {
            K12 = beta1[3].im / beta1[3].re;
            K13 = (alpha1[3].re + K12 * alpha1[3].im) /
                  (beta1[3].re + K12 * beta1[3].im);
          } else if (K13 == K12) {
            if (beta1[3].re > 0.0) {
              K13 = 0.5;
            } else {
              K13 = -0.5;
            }
            if (beta1[3].im > 0.0) {
              K14 = 0.5;
            } else {
              K14 = -0.5;
            }
            K13 = (alpha1[3].re * K13 + alpha1[3].im * K14) / K12;
          } else {
            K12 = beta1[3].re / beta1[3].im;
            K13 = (K12 * alpha1[3].re + alpha1[3].im) /
                  (beta1[3].im + K12 * beta1[3].re);
          }
        }
        alpha1[3].re = K13;
      }
    }
  }
  D[0] = alpha1[0].re;
  D[1] = alpha1[1].re;
  D[2] = alpha1[2].re;
  D[3] = alpha1[3].re;
  if (!std::isnan(alpha1[0].re)) {
    sgn = 1;
  } else {
    sgn = 0;
    k = 2;
    exitg2 = false;
    while ((!exitg2) && (k < 5)) {
      if (!std::isnan(D[k - 1])) {
        sgn = k;
        exitg2 = true;
      } else {
        k++;
      }
    }
  }
  if (sgn == 0) {
    kend_tmp = 0;
  } else {
    K13 = D[sgn - 1];
    kend_tmp = sgn - 1;
    i = sgn + 1;
    for (k = i; k < 5; k++) {
      K12 = D[k - 1];
      if (K13 < K12) {
        K13 = K12;
        kend_tmp = k - 1;
      }
    }
  }
  sgn = kend_tmp << 2;
  quat[0] = V[sgn + 3].re;
  quat[1] = V[sgn].re;
  quat[2] = V[sgn + 1].re;
  quat[3] = V[sgn + 2].re;
  if (quat[0] < 0.0) {
    quat[0] = -quat[0];
    quat[1] = -quat[1];
    quat[2] = -quat[2];
    quat[3] = -quat[3];
  }
}

} // namespace coder

//
// File trailer for rotm2quat.cpp
//
// [EOF]
//
