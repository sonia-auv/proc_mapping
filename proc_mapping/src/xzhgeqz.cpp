//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xzhgeqz.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 01-Aug-2022 08:26:09
//

// Include Files
#include "xzhgeqz.h"
#include "proc_mapping_data.h"
#include "proc_mapping_rtwutil.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "xzlartg.h"
#include <cmath>
#include <string.h>

// Function Definitions
//
// Arguments    : creal_T A[16]
//                int ilo
//                int ihi
//                creal_T Z[16]
//                int *info
//                creal_T alpha1[4]
//                creal_T beta1[4]
// Return Type  : void
//
namespace coder {
namespace internal {
namespace reflapack {
void xzhgeqz(creal_T A[16], int ilo, int ihi, creal_T Z[16], int *info,
             creal_T alpha1[4], creal_T beta1[4])
{
  creal_T ctemp;
  creal_T stemp;
  creal_T y;
  double anorm;
  double ascale;
  double b_atol;
  double colscale;
  double colssq;
  double eshift_im;
  double eshift_re;
  double scale;
  double ssq;
  double t;
  int col;
  int i;
  int ilast;
  int j;
  int jm1;
  int nm1;
  int row;
  bool failed;
  bool guard1{false};
  bool guard2{false};
  *info = 0;
  alpha1[0].re = 0.0;
  alpha1[0].im = 0.0;
  beta1[0].re = 1.0;
  beta1[0].im = 0.0;
  alpha1[1].re = 0.0;
  alpha1[1].im = 0.0;
  beta1[1].re = 1.0;
  beta1[1].im = 0.0;
  alpha1[2].re = 0.0;
  alpha1[2].im = 0.0;
  beta1[2].re = 1.0;
  beta1[2].im = 0.0;
  alpha1[3].re = 0.0;
  alpha1[3].im = 0.0;
  beta1[3].re = 1.0;
  beta1[3].im = 0.0;
  eshift_re = 0.0;
  eshift_im = 0.0;
  ctemp.re = 0.0;
  ctemp.im = 0.0;
  anorm = 0.0;
  if (ilo <= ihi) {
    scale = 3.3121686421112381E-170;
    ssq = 0.0;
    nm1 = ihi - ilo;
    for (j = 0; j <= nm1; j++) {
      colscale = 3.3121686421112381E-170;
      colssq = 0.0;
      col = (ilo + j) - 1;
      jm1 = j + 1;
      if (jm1 > nm1) {
        jm1 = nm1;
      }
      i = ilo + jm1;
      for (row = ilo; row <= i; row++) {
        jm1 = (row + (col << 2)) - 1;
        anorm = std::abs(A[jm1].re);
        if (anorm > colscale) {
          t = colscale / anorm;
          colssq = colssq * t * t + 1.0;
          colscale = anorm;
        } else {
          t = anorm / colscale;
          colssq += t * t;
        }
        anorm = std::abs(A[jm1].im);
        if (anorm > colscale) {
          t = colscale / anorm;
          colssq = colssq * t * t + 1.0;
          colscale = anorm;
        } else {
          t = anorm / colscale;
          colssq += t * t;
        }
      }
      if (scale >= colscale) {
        t = colscale / scale;
        ssq += t * t * colssq;
      } else {
        t = scale / colscale;
        ssq = colssq + t * t * ssq;
        scale = colscale;
      }
    }
    anorm = scale * std::sqrt(ssq);
  }
  t = 2.2204460492503131E-16 * anorm;
  b_atol = 2.2250738585072014E-308;
  if (t > 2.2250738585072014E-308) {
    b_atol = t;
  }
  t = 2.2250738585072014E-308;
  if (anorm > 2.2250738585072014E-308) {
    t = anorm;
  }
  ascale = 1.0 / t;
  failed = true;
  i = ihi + 1;
  for (j = i; j < 5; j++) {
    alpha1[j - 1] = A[(j + ((j - 1) << 2)) - 1];
  }
  guard1 = false;
  guard2 = false;
  if (ihi >= ilo) {
    int ifirst;
    int iiter;
    int ilastm1;
    int istart;
    int jiter;
    bool goto60;
    bool goto70;
    bool goto90;
    ifirst = ilo;
    istart = ilo;
    ilast = ihi - 1;
    ilastm1 = ihi - 2;
    iiter = 0;
    goto60 = false;
    goto70 = false;
    goto90 = false;
    jiter = 0;
    int exitg1;
    do {
      exitg1 = 0;
      if (jiter <= 30 * ((ihi - ilo) + 1) - 1) {
        bool b_guard1{false};
        bool exitg2;
        b_guard1 = false;
        if (ilast + 1 == ilo) {
          goto60 = true;
          b_guard1 = true;
        } else {
          i = ilastm1 << 2;
          nm1 = ilast + i;
          jm1 = ilast + (ilast << 2);
          i += ilastm1;
          if (std::abs(A[nm1].re) + std::abs(A[nm1].im) <=
              std::fmax(2.2250738585072014E-308,
                        2.2204460492503131E-16 *
                            ((std::abs(A[jm1].re) + std::abs(A[jm1].im)) +
                             (std::abs(A[i].re) + std::abs(A[i].im))))) {
            A[nm1].re = 0.0;
            A[nm1].im = 0.0;
            goto60 = true;
            b_guard1 = true;
          } else {
            bool guard3{false};
            j = ilastm1;
            guard3 = false;
            exitg2 = false;
            while ((!exitg2) && (j + 1 >= ilo)) {
              if (j + 1 == ilo) {
                guard3 = true;
                exitg2 = true;
              } else {
                i = j + ((j - 1) << 2);
                nm1 = j + (j << 2);
                if (std::abs(A[i].re) + std::abs(A[i].im) <=
                    std::fmax(2.2250738585072014E-308,
                              2.2204460492503131E-16 *
                                  ((std::abs(A[nm1].re) + std::abs(A[nm1].im)) +
                                   (std::abs(A[i - 1].re) +
                                    std::abs(A[i - 1].im))))) {
                  A[i].re = 0.0;
                  A[i].im = 0.0;
                  guard3 = true;
                  exitg2 = true;
                } else {
                  j--;
                  guard3 = false;
                }
              }
            }
            if (guard3) {
              ifirst = j + 1;
              goto70 = true;
            }
            if (goto70) {
              b_guard1 = true;
            } else {
              alpha1[0].re = rtNaN;
              alpha1[0].im = 0.0;
              beta1[0].re = rtNaN;
              beta1[0].im = 0.0;
              alpha1[1].re = rtNaN;
              alpha1[1].im = 0.0;
              beta1[1].re = rtNaN;
              beta1[1].im = 0.0;
              alpha1[2].re = rtNaN;
              alpha1[2].im = 0.0;
              beta1[2].re = rtNaN;
              beta1[2].im = 0.0;
              alpha1[3].re = rtNaN;
              alpha1[3].im = 0.0;
              beta1[3].re = rtNaN;
              beta1[3].im = 0.0;
              for (i = 0; i < 16; i++) {
                Z[i].re = rtNaN;
                Z[i].im = 0.0;
              }
              *info = 1;
              exitg1 = 1;
            }
          }
        }
        if (b_guard1) {
          if (goto60) {
            goto60 = false;
            alpha1[ilast] = A[ilast + (ilast << 2)];
            ilast = ilastm1;
            ilastm1--;
            if (ilast + 1 < ilo) {
              failed = false;
              guard2 = true;
              exitg1 = 1;
            } else {
              iiter = 0;
              eshift_re = 0.0;
              eshift_im = 0.0;
              jiter++;
            }
          } else {
            if (goto70) {
              double ad22_im;
              double ad22_re;
              goto70 = false;
              iiter++;
              if (iiter - div_nzp_s32(iiter, 10) * 10 != 0) {
                jm1 = ilast + (ilast << 2);
                anorm = ascale * A[jm1].re;
                t = ascale * A[jm1].im;
                if (t == 0.0) {
                  ad22_re = anorm / 0.5;
                  ad22_im = 0.0;
                } else if (anorm == 0.0) {
                  ad22_re = 0.0;
                  ad22_im = t / 0.5;
                } else {
                  ad22_re = anorm / 0.5;
                  ad22_im = t / 0.5;
                }
                jm1 = ilastm1 + (ilast << 2);
                anorm = ascale * A[jm1].re;
                t = ascale * A[jm1].im;
                if (t == 0.0) {
                  stemp.re = anorm / 0.5;
                  stemp.im = 0.0;
                } else if (anorm == 0.0) {
                  stemp.re = 0.0;
                  stemp.im = t / 0.5;
                } else {
                  stemp.re = anorm / 0.5;
                  stemp.im = t / 0.5;
                }
                b_sqrt(&stemp);
                jm1 = ilast + (ilastm1 << 2);
                anorm = ascale * A[jm1].re;
                t = ascale * A[jm1].im;
                if (t == 0.0) {
                  y.re = anorm / 0.5;
                  y.im = 0.0;
                } else if (anorm == 0.0) {
                  y.re = 0.0;
                  y.im = t / 0.5;
                } else {
                  y.re = anorm / 0.5;
                  y.im = t / 0.5;
                }
                b_sqrt(&y);
                ctemp.re = stemp.re * y.re - stemp.im * y.im;
                ctemp.im = stemp.re * y.im + stemp.im * y.re;
                if ((ctemp.re != 0.0) || (ctemp.im != 0.0)) {
                  double x_im;
                  jm1 = ilastm1 + (ilastm1 << 2);
                  anorm = ascale * A[jm1].re;
                  t = ascale * A[jm1].im;
                  if (t == 0.0) {
                    anorm /= 0.5;
                    t = 0.0;
                  } else if (anorm == 0.0) {
                    anorm = 0.0;
                    t /= 0.5;
                  } else {
                    anorm /= 0.5;
                    t /= 0.5;
                  }
                  colssq = 0.5 * (anorm - ad22_re);
                  x_im = 0.5 * (t - ad22_im);
                  colscale = std::abs(colssq) + std::abs(x_im);
                  ssq = std::fmax(std::abs(ctemp.re) + std::abs(ctemp.im),
                                  colscale);
                  if (x_im == 0.0) {
                    stemp.re = colssq / ssq;
                    stemp.im = 0.0;
                  } else if (colssq == 0.0) {
                    stemp.re = 0.0;
                    stemp.im = x_im / ssq;
                  } else {
                    stemp.re = colssq / ssq;
                    stemp.im = x_im / ssq;
                  }
                  if (ctemp.im == 0.0) {
                    y.re = ctemp.re / ssq;
                    y.im = 0.0;
                  } else if (ctemp.re == 0.0) {
                    y.re = 0.0;
                    y.im = ctemp.im / ssq;
                  } else {
                    y.re = ctemp.re / ssq;
                    y.im = ctemp.im / ssq;
                  }
                  anorm = stemp.re * stemp.re - stemp.im * stemp.im;
                  t = stemp.re * stemp.im;
                  scale = y.re * y.im;
                  stemp.re = anorm + (y.re * y.re - y.im * y.im);
                  stemp.im = (t + t) + (scale + scale);
                  b_sqrt(&stemp);
                  y.re = ssq * stemp.re;
                  y.im = ssq * stemp.im;
                  if (colscale > 0.0) {
                    if (x_im == 0.0) {
                      t = colssq / colscale;
                      anorm = 0.0;
                    } else {
                      if (colssq == 0.0) {
                        t = 0.0;
                      } else {
                        t = colssq / colscale;
                      }
                      anorm = x_im / colscale;
                    }
                    if (t * y.re + anorm * y.im < 0.0) {
                      y.re = -y.re;
                      y.im = -y.im;
                    }
                  }
                  scale = colssq + y.re;
                  ssq = x_im + y.im;
                  if (ssq == 0.0) {
                    if (ctemp.im == 0.0) {
                      colssq = ctemp.re / scale;
                      anorm = 0.0;
                    } else if (ctemp.re == 0.0) {
                      colssq = 0.0;
                      anorm = ctemp.im / scale;
                    } else {
                      colssq = ctemp.re / scale;
                      anorm = ctemp.im / scale;
                    }
                  } else if (scale == 0.0) {
                    if (ctemp.re == 0.0) {
                      colssq = ctemp.im / ssq;
                      anorm = 0.0;
                    } else if (ctemp.im == 0.0) {
                      colssq = 0.0;
                      anorm = -(ctemp.re / ssq);
                    } else {
                      colssq = ctemp.im / ssq;
                      anorm = -(ctemp.re / ssq);
                    }
                  } else {
                    colscale = std::abs(scale);
                    anorm = std::abs(ssq);
                    if (colscale > anorm) {
                      t = ssq / scale;
                      anorm = scale + t * ssq;
                      colssq = (ctemp.re + t * ctemp.im) / anorm;
                      anorm = (ctemp.im - t * ctemp.re) / anorm;
                    } else if (anorm == colscale) {
                      if (scale > 0.0) {
                        t = 0.5;
                      } else {
                        t = -0.5;
                      }
                      if (ssq > 0.0) {
                        anorm = 0.5;
                      } else {
                        anorm = -0.5;
                      }
                      colssq = (ctemp.re * t + ctemp.im * anorm) / colscale;
                      anorm = (ctemp.im * t - ctemp.re * anorm) / colscale;
                    } else {
                      t = scale / ssq;
                      anorm = ssq + t * scale;
                      colssq = (t * ctemp.re + ctemp.im) / anorm;
                      anorm = (t * ctemp.im - ctemp.re) / anorm;
                    }
                  }
                  ad22_re -= ctemp.re * colssq - ctemp.im * anorm;
                  ad22_im -= ctemp.re * anorm + ctemp.im * colssq;
                }
              } else {
                if (iiter - div_nzp_s32(iiter, 20) * 20 == 0) {
                  jm1 = ilast + (ilast << 2);
                  anorm = ascale * A[jm1].re;
                  t = ascale * A[jm1].im;
                  if (t == 0.0) {
                    anorm /= 0.5;
                    t = 0.0;
                  } else if (anorm == 0.0) {
                    anorm = 0.0;
                    t /= 0.5;
                  } else {
                    anorm /= 0.5;
                    t /= 0.5;
                  }
                  eshift_re += anorm;
                  eshift_im += t;
                } else {
                  jm1 = ilast + (ilastm1 << 2);
                  anorm = ascale * A[jm1].re;
                  t = ascale * A[jm1].im;
                  if (t == 0.0) {
                    anorm /= 0.5;
                    t = 0.0;
                  } else if (anorm == 0.0) {
                    anorm = 0.0;
                    t /= 0.5;
                  } else {
                    anorm /= 0.5;
                    t /= 0.5;
                  }
                  eshift_re += anorm;
                  eshift_im += t;
                }
                ad22_re = eshift_re;
                ad22_im = eshift_im;
              }
              j = ilastm1;
              nm1 = ilastm1 + 1;
              exitg2 = false;
              while ((!exitg2) && (j + 1 > ifirst)) {
                istart = j + 1;
                col = j << 2;
                row = j + col;
                ctemp.re = ascale * A[row].re - ad22_re * 0.5;
                ctemp.im = ascale * A[row].im - ad22_im * 0.5;
                ssq = std::abs(ctemp.re) + std::abs(ctemp.im);
                jm1 = nm1 + col;
                colscale = ascale * (std::abs(A[jm1].re) + std::abs(A[jm1].im));
                anorm = ssq;
                if (colscale > ssq) {
                  anorm = colscale;
                }
                if ((anorm < 1.0) && (anorm != 0.0)) {
                  ssq /= anorm;
                  colscale /= anorm;
                }
                i = j + ((j - 1) << 2);
                if ((std::abs(A[i].re) + std::abs(A[i].im)) * colscale <=
                    ssq * b_atol) {
                  goto90 = true;
                  exitg2 = true;
                } else {
                  nm1 = j;
                  j--;
                }
              }
              if (!goto90) {
                istart = ifirst;
                row = (ifirst + ((ifirst - 1) << 2)) - 1;
                ctemp.re = ascale * A[row].re - ad22_re * 0.5;
                ctemp.im = ascale * A[row].im - ad22_im * 0.5;
              }
              goto90 = false;
              jm1 = istart + ((istart - 1) << 2);
              stemp.re = ascale * A[jm1].re;
              stemp.im = ascale * A[jm1].im;
              xzlartg(ctemp, stemp, &colscale, &y);
              j = istart;
              jm1 = istart - 2;
              while (j < ilast + 1) {
                if (j > istart) {
                  nm1 = j + (jm1 << 2);
                  xzlartg(A[nm1 - 1], A[nm1], &colscale, &y,
                          &A[(j + (jm1 << 2)) - 1]);
                  A[nm1].re = 0.0;
                  A[nm1].im = 0.0;
                }
                for (col = j; col < 5; col++) {
                  nm1 = j + ((col - 1) << 2);
                  anorm = A[nm1].im;
                  t = A[nm1].re;
                  scale = A[nm1 - 1].re;
                  stemp.re = colscale * scale + (y.re * t - y.im * anorm);
                  ssq = A[nm1 - 1].im;
                  stemp.im = colscale * ssq + (y.re * anorm + y.im * t);
                  A[nm1].re = colscale * t - (y.re * scale + y.im * ssq);
                  A[nm1].im =
                      colscale * A[nm1].im - (y.re * ssq - y.im * scale);
                  A[nm1 - 1] = stemp;
                }
                y.re = -y.re;
                y.im = -y.im;
                nm1 = j;
                if (ilast + 1 < j + 2) {
                  nm1 = ilast - 1;
                }
                for (jm1 = 1; jm1 <= nm1 + 2; jm1++) {
                  col = (jm1 + ((j - 1) << 2)) - 1;
                  anorm = A[col].im;
                  t = A[col].re;
                  row = (jm1 + (j << 2)) - 1;
                  scale = A[row].re;
                  stemp.re = colscale * scale + (y.re * t - y.im * anorm);
                  ssq = A[row].im;
                  stemp.im = colscale * ssq + (y.re * anorm + y.im * t);
                  A[col].re =
                      colscale * A[col].re - (y.re * scale + y.im * ssq);
                  A[col].im =
                      colscale * A[col].im - (y.re * ssq - y.im * scale);
                  A[row] = stemp;
                }
                nm1 = (j - 1) << 2;
                anorm = Z[nm1].im;
                t = Z[nm1].re;
                row = j << 2;
                scale = Z[row].re;
                stemp.re = colscale * scale + (y.re * t - y.im * anorm);
                ssq = Z[row].im;
                stemp.im = colscale * ssq + (y.re * anorm + y.im * t);
                Z[nm1].re = colscale * t - (y.re * scale + y.im * ssq);
                Z[nm1].im = colscale * Z[nm1].im - (y.re * ssq - y.im * scale);
                Z[row] = stemp;
                anorm = Z[nm1 + 1].im;
                t = Z[nm1 + 1].re;
                scale = Z[row + 1].re;
                stemp.re = colscale * scale + (y.re * t - y.im * anorm);
                ssq = Z[row + 1].im;
                stemp.im = colscale * ssq + (y.re * anorm + y.im * t);
                Z[nm1 + 1].re = colscale * t - (y.re * scale + y.im * ssq);
                Z[nm1 + 1].im =
                    colscale * Z[nm1 + 1].im - (y.re * ssq - y.im * scale);
                Z[row + 1] = stemp;
                anorm = Z[nm1 + 2].im;
                t = Z[nm1 + 2].re;
                scale = Z[row + 2].re;
                stemp.re = colscale * scale + (y.re * t - y.im * anorm);
                ssq = Z[row + 2].im;
                stemp.im = colscale * ssq + (y.re * anorm + y.im * t);
                Z[nm1 + 2].re = colscale * t - (y.re * scale + y.im * ssq);
                Z[nm1 + 2].im =
                    colscale * Z[nm1 + 2].im - (y.re * ssq - y.im * scale);
                Z[row + 2] = stemp;
                anorm = Z[nm1 + 3].im;
                t = Z[nm1 + 3].re;
                scale = Z[row + 3].re;
                stemp.re = colscale * scale + (y.re * t - y.im * anorm);
                ssq = Z[row + 3].im;
                stemp.im = colscale * ssq + (y.re * anorm + y.im * t);
                Z[nm1 + 3].re = colscale * t - (y.re * scale + y.im * ssq);
                Z[nm1 + 3].im =
                    colscale * Z[nm1 + 3].im - (y.re * ssq - y.im * scale);
                Z[row + 3] = stemp;
                jm1 = j - 1;
                j++;
              }
            }
            jiter++;
          }
        }
      } else {
        guard2 = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  } else {
    guard1 = true;
  }
  if (guard2) {
    if (failed) {
      *info = ilast + 1;
      for (jm1 = 0; jm1 <= ilast; jm1++) {
        alpha1[jm1].re = rtNaN;
        alpha1[jm1].im = 0.0;
        beta1[jm1].re = rtNaN;
        beta1[jm1].im = 0.0;
      }
      for (i = 0; i < 16; i++) {
        Z[i].re = rtNaN;
        Z[i].im = 0.0;
      }
    } else {
      guard1 = true;
    }
  }
  if (guard1) {
    for (j = 0; j <= ilo - 2; j++) {
      alpha1[j] = A[j + (j << 2)];
    }
  }
}

} // namespace reflapack
} // namespace internal
} // namespace coder

//
// File trailer for xzhgeqz.cpp
//
// [EOF]
//
