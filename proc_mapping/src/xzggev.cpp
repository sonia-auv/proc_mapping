//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xzggev.cpp
//
// Code generation for function 'xzggev'
//

// Include files
#include "xzggev.h"
#include "proc_mapping_rtwutil.h"
#include "rt_nonfinite.h"
#include "xzhgeqz.h"
#include "xzlartg.h"
#include "xztgevc.h"
#include <cmath>
#include <cstring>
#include <string.h>

// Function Definitions
namespace coder {
namespace internal {
namespace reflapack {
void xzggev(creal_T A[16], int *info, creal_T alpha1[4], creal_T beta1[4],
            creal_T V[16])
{
  creal_T atmp;
  double absxk;
  double anrm;
  double ctoc;
  int jcol;
  bool exitg1;
  *info = 0;
  anrm = 0.0;
  jcol = 0;
  exitg1 = false;
  while ((!exitg1) && (jcol < 16)) {
    absxk = rt_hypotd_snf(A[jcol].re, A[jcol].im);
    if (std::isnan(absxk)) {
      anrm = rtNaN;
      exitg1 = true;
    } else {
      if (absxk > anrm) {
        anrm = absxk;
      }
      jcol++;
    }
  }
  if (std::isinf(anrm) || std::isnan(anrm)) {
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
    for (int jcolp1{0}; jcolp1 < 16; jcolp1++) {
      V[jcolp1].re = rtNaN;
      V[jcolp1].im = 0.0;
    }
  } else {
    double a;
    double anrmto;
    double cto1;
    double vtemp;
    int rscale[4];
    int A_tmp;
    int exitg3;
    int i;
    int ihi;
    int ii;
    int ilo;
    int j;
    int jcolp1;
    int nzcount;
    bool exitg4;
    bool guard1{false};
    bool ilascl;
    bool notdone;
    ilascl = false;
    anrmto = anrm;
    guard1 = false;
    if ((anrm > 0.0) && (anrm < 6.7178761075670888E-139)) {
      anrmto = 6.7178761075670888E-139;
      ilascl = true;
      guard1 = true;
    } else if (anrm > 1.4885657073574029E+138) {
      anrmto = 1.4885657073574029E+138;
      ilascl = true;
      guard1 = true;
    }
    if (guard1) {
      absxk = anrm;
      ctoc = anrmto;
      notdone = true;
      while (notdone) {
        vtemp = absxk * 2.0041683600089728E-292;
        cto1 = ctoc / 4.9896007738368E+291;
        if ((vtemp > ctoc) && (ctoc != 0.0)) {
          a = 2.0041683600089728E-292;
          absxk = vtemp;
        } else if (cto1 > absxk) {
          a = 4.9896007738368E+291;
          ctoc = cto1;
        } else {
          a = ctoc / absxk;
          notdone = false;
        }
        for (jcolp1 = 0; jcolp1 < 16; jcolp1++) {
          A[jcolp1].re *= a;
          A[jcolp1].im *= a;
        }
      }
    }
    rscale[0] = 1;
    rscale[1] = 1;
    rscale[2] = 1;
    rscale[3] = 1;
    ilo = 1;
    ihi = 4;
    do {
      exitg3 = 0;
      i = 0;
      j = 0;
      notdone = false;
      ii = ihi;
      exitg1 = false;
      while ((!exitg1) && (ii > 0)) {
        nzcount = 0;
        i = ii;
        j = ihi;
        jcol = 0;
        exitg4 = false;
        while ((!exitg4) && (jcol <= ihi - 1)) {
          A_tmp = (ii + (jcol << 2)) - 1;
          if ((A[A_tmp].re != 0.0) || (A[A_tmp].im != 0.0) ||
              (ii == jcol + 1)) {
            if (nzcount == 0) {
              j = jcol + 1;
              nzcount = 1;
              jcol++;
            } else {
              nzcount = 2;
              exitg4 = true;
            }
          } else {
            jcol++;
          }
        }
        if (nzcount < 2) {
          notdone = true;
          exitg1 = true;
        } else {
          ii--;
        }
      }
      if (!notdone) {
        exitg3 = 2;
      } else {
        if (i != ihi) {
          atmp = A[i - 1];
          A[i - 1] = A[ihi - 1];
          A[ihi - 1] = atmp;
          atmp = A[i + 3];
          A[i + 3] = A[ihi + 3];
          A[ihi + 3] = atmp;
          atmp = A[i + 7];
          A[i + 7] = A[ihi + 7];
          A[ihi + 7] = atmp;
          atmp = A[i + 11];
          A[i + 11] = A[ihi + 11];
          A[ihi + 11] = atmp;
        }
        if (j != ihi) {
          for (jcol = 0; jcol < ihi; jcol++) {
            ii = jcol + ((j - 1) << 2);
            atmp = A[ii];
            jcolp1 = jcol + ((ihi - 1) << 2);
            A[ii] = A[jcolp1];
            A[jcolp1] = atmp;
          }
        }
        rscale[ihi - 1] = j;
        ihi--;
        if (ihi == 1) {
          rscale[0] = 1;
          exitg3 = 1;
        }
      }
    } while (exitg3 == 0);
    if (exitg3 != 1) {
      int exitg2;
      do {
        exitg2 = 0;
        i = 0;
        j = 0;
        notdone = false;
        jcol = ilo;
        exitg1 = false;
        while ((!exitg1) && (jcol <= ihi)) {
          nzcount = 0;
          i = ihi;
          j = jcol;
          ii = ilo;
          exitg4 = false;
          while ((!exitg4) && (ii <= ihi)) {
            A_tmp = (ii + ((jcol - 1) << 2)) - 1;
            if ((A[A_tmp].re != 0.0) || (A[A_tmp].im != 0.0) || (ii == jcol)) {
              if (nzcount == 0) {
                i = ii;
                nzcount = 1;
                ii++;
              } else {
                nzcount = 2;
                exitg4 = true;
              }
            } else {
              ii++;
            }
          }
          if (nzcount < 2) {
            notdone = true;
            exitg1 = true;
          } else {
            jcol++;
          }
        }
        if (!notdone) {
          exitg2 = 1;
        } else {
          if (i != ilo) {
            for (jcol = ilo; jcol < 5; jcol++) {
              ii = (jcol - 1) << 2;
              nzcount = (i + ii) - 1;
              atmp = A[nzcount];
              jcolp1 = (ilo + ii) - 1;
              A[nzcount] = A[jcolp1];
              A[jcolp1] = atmp;
            }
          }
          if (j != ilo) {
            for (jcol = 0; jcol < ihi; jcol++) {
              ii = jcol + ((j - 1) << 2);
              atmp = A[ii];
              jcolp1 = jcol + ((ilo - 1) << 2);
              A[ii] = A[jcolp1];
              A[jcolp1] = atmp;
            }
          }
          rscale[ilo - 1] = j;
          ilo++;
          if (ilo == ihi) {
            rscale[ilo - 1] = ilo;
            exitg2 = 1;
          }
        }
      } while (exitg2 == 0);
    }
    std::memset(&V[0], 0, 16U * sizeof(creal_T));
    V[0].re = 1.0;
    V[0].im = 0.0;
    V[5].re = 1.0;
    V[5].im = 0.0;
    V[10].re = 1.0;
    V[10].im = 0.0;
    V[15].re = 1.0;
    V[15].im = 0.0;
    if (ihi >= ilo + 2) {
      for (jcol = ilo - 1; jcol + 1 < ihi - 1; jcol++) {
        jcolp1 = jcol + 2;
        for (int jrow{ihi - 1}; jrow + 1 > jcol + 2; jrow--) {
          A_tmp = jrow + (jcol << 2);
          xzlartg(A[A_tmp - 1], A[A_tmp], &ctoc, &atmp,
                  &A[(jrow + (jcol << 2)) - 1]);
          A[A_tmp].re = 0.0;
          A[A_tmp].im = 0.0;
          for (j = jcolp1; j < 5; j++) {
            nzcount = jrow + ((j - 1) << 2);
            absxk = A[nzcount].im;
            vtemp = A[nzcount].re;
            cto1 = A[nzcount - 1].re;
            a = A[nzcount - 1].im;
            A[nzcount].re = ctoc * vtemp - (atmp.re * cto1 + atmp.im * a);
            A[nzcount].im =
                ctoc * A[nzcount].im - (atmp.re * a - atmp.im * cto1);
            A[nzcount - 1].re =
                ctoc * cto1 + (atmp.re * vtemp - atmp.im * absxk);
            A[nzcount - 1].im = ctoc * a + (atmp.re * absxk + atmp.im * vtemp);
          }
          atmp.re = -atmp.re;
          atmp.im = -atmp.im;
          for (i = 1; i <= ihi; i++) {
            nzcount = (i + ((jrow - 1) << 2)) - 1;
            absxk = A[nzcount].im;
            vtemp = A[nzcount].re;
            ii = (i + (jrow << 2)) - 1;
            cto1 = A[ii].re;
            a = A[ii].im;
            A[nzcount].re = ctoc * vtemp - (atmp.re * cto1 + atmp.im * a);
            A[nzcount].im =
                ctoc * A[nzcount].im - (atmp.re * a - atmp.im * cto1);
            A[ii].re = ctoc * cto1 + (atmp.re * vtemp - atmp.im * absxk);
            A[ii].im = ctoc * a + (atmp.re * absxk + atmp.im * vtemp);
          }
          nzcount = (jrow - 1) << 2;
          vtemp = V[nzcount].im;
          absxk = V[nzcount].re;
          ii = jrow << 2;
          cto1 = V[ii].re;
          a = V[ii].im;
          V[nzcount].re = ctoc * absxk - (atmp.re * cto1 + atmp.im * a);
          V[nzcount].im = ctoc * vtemp - (atmp.re * a - atmp.im * cto1);
          V[ii].re = ctoc * cto1 + (atmp.re * absxk - atmp.im * vtemp);
          V[ii].im = ctoc * a + (atmp.re * vtemp + atmp.im * absxk);
          vtemp = V[nzcount + 1].im;
          absxk = V[nzcount + 1].re;
          cto1 = V[ii + 1].re;
          a = V[ii + 1].im;
          V[nzcount + 1].re = ctoc * absxk - (atmp.re * cto1 + atmp.im * a);
          V[nzcount + 1].im = ctoc * vtemp - (atmp.re * a - atmp.im * cto1);
          V[ii + 1].re = ctoc * cto1 + (atmp.re * absxk - atmp.im * vtemp);
          V[ii + 1].im = ctoc * a + (atmp.re * vtemp + atmp.im * absxk);
          vtemp = V[nzcount + 2].im;
          absxk = V[nzcount + 2].re;
          cto1 = V[ii + 2].re;
          a = V[ii + 2].im;
          V[nzcount + 2].re = ctoc * absxk - (atmp.re * cto1 + atmp.im * a);
          V[nzcount + 2].im = ctoc * vtemp - (atmp.re * a - atmp.im * cto1);
          V[ii + 2].re = ctoc * cto1 + (atmp.re * absxk - atmp.im * vtemp);
          V[ii + 2].im = ctoc * a + (atmp.re * vtemp + atmp.im * absxk);
          vtemp = V[nzcount + 3].im;
          absxk = V[nzcount + 3].re;
          cto1 = V[ii + 3].re;
          a = V[ii + 3].im;
          V[nzcount + 3].re = ctoc * absxk - (atmp.re * cto1 + atmp.im * a);
          V[nzcount + 3].im = ctoc * vtemp - (atmp.re * a - atmp.im * cto1);
          V[ii + 3].re = ctoc * cto1 + (atmp.re * absxk - atmp.im * vtemp);
          V[ii + 3].im = ctoc * a + (atmp.re * vtemp + atmp.im * absxk);
        }
      }
    }
    xzhgeqz(A, ilo, ihi, V, info, alpha1, beta1);
    if (*info == 0) {
      xztgevc(A, V);
      if (ilo > 1) {
        for (i = ilo - 2; i + 1 >= 1; i--) {
          jcol = rscale[i] - 1;
          if (rscale[i] != i + 1) {
            atmp = V[i];
            V[i] = V[jcol];
            V[jcol] = atmp;
            atmp = V[i + 4];
            V[i + 4] = V[jcol + 4];
            V[jcol + 4] = atmp;
            atmp = V[i + 8];
            V[i + 8] = V[jcol + 8];
            V[jcol + 8] = atmp;
            atmp = V[i + 12];
            V[i + 12] = V[jcol + 12];
            V[jcol + 12] = atmp;
          }
        }
      }
      if (ihi < 4) {
        jcolp1 = ihi + 1;
        for (i = jcolp1; i < 5; i++) {
          ii = rscale[i - 1];
          if (ii != i) {
            atmp = V[i - 1];
            V[i - 1] = V[ii - 1];
            V[ii - 1] = atmp;
            atmp = V[i + 3];
            V[i + 3] = V[ii + 3];
            V[ii + 3] = atmp;
            atmp = V[i + 7];
            V[i + 7] = V[ii + 7];
            V[ii + 7] = atmp;
            atmp = V[i + 11];
            V[i + 11] = V[ii + 11];
            V[ii + 11] = atmp;
          }
        }
      }
      for (nzcount = 0; nzcount < 4; nzcount++) {
        double d;
        double d1;
        double d2;
        double d3;
        double y;
        ii = nzcount << 2;
        absxk = V[ii].re;
        ctoc = V[ii].im;
        vtemp = std::abs(absxk) + std::abs(ctoc);
        cto1 = V[ii + 1].re;
        a = V[ii + 1].im;
        y = std::abs(cto1) + std::abs(a);
        if (y > vtemp) {
          vtemp = y;
        }
        d = V[ii + 2].re;
        d1 = V[ii + 2].im;
        y = std::abs(d) + std::abs(d1);
        if (y > vtemp) {
          vtemp = y;
        }
        d2 = V[ii + 3].re;
        d3 = V[ii + 3].im;
        y = std::abs(d2) + std::abs(d3);
        if (y > vtemp) {
          vtemp = y;
        }
        if (vtemp >= 6.7178761075670888E-139) {
          vtemp = 1.0 / vtemp;
          V[ii].re = vtemp * absxk;
          V[ii].im = vtemp * ctoc;
          cto1 *= vtemp;
          V[ii + 1].re = cto1;
          a *= vtemp;
          V[ii + 1].im = a;
          d *= vtemp;
          V[ii + 2].re = d;
          d1 *= vtemp;
          V[ii + 2].im = d1;
          d2 *= vtemp;
          V[ii + 3].re = d2;
          d3 *= vtemp;
          V[ii + 3].im = d3;
        }
      }
      if (ilascl) {
        notdone = true;
        while (notdone) {
          vtemp = anrmto * 2.0041683600089728E-292;
          cto1 = anrm / 4.9896007738368E+291;
          if ((vtemp > anrm) && (anrm != 0.0)) {
            a = 2.0041683600089728E-292;
            anrmto = vtemp;
          } else if (cto1 > anrmto) {
            a = 4.9896007738368E+291;
            anrm = cto1;
          } else {
            a = anrm / anrmto;
            notdone = false;
          }
          alpha1[0].re *= a;
          alpha1[0].im *= a;
          alpha1[1].re *= a;
          alpha1[1].im *= a;
          alpha1[2].re *= a;
          alpha1[2].im *= a;
          alpha1[3].re *= a;
          alpha1[3].im *= a;
        }
      }
    }
  }
}

} // namespace reflapack
} // namespace internal
} // namespace coder

// End of code generation (xzggev.cpp)
