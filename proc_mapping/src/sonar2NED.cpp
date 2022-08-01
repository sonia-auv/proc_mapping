//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: sonar2NED.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 01-Aug-2022 08:26:09
//

// Include Files
#include "sonar2NED.h"
#include "rt_nonfinite.h"
#include <string.h>

// Function Definitions
//
// sonar2NED
//     OUT1 = sonar2NED(IN1,IN2,IN3,IN4)
//
// Arguments    : const double in1[3]
//                const double in2[4]
//                const double in3[3]
//                const double in4[3]
//                double out1[4]
// Return Type  : void
//
void sonar2NED(const double in1[3], const double in2[4], const double in3[3],
               const double in4[3], double out1[4])
{
  double t10;
  double t11;
  double t12;
  double t13;
  double t17;
  double t18;
  double t19;
  double t20;
  double t21;
  double t5;
  double t6;
  double t7;
  double t8;
  double t9;
  //     This function was generated by the Symbolic Math Toolbox version 9.1.
  //     11-May-2022 11:36:02
  t5 = in2[1] * in2[2] * 2.0;
  t6 = in2[1] * in2[3] * 2.0;
  t7 = in2[2] * in2[3] * 2.0;
  t8 = in2[0] * in2[1] * 2.0;
  t9 = in2[0] * in2[2] * 2.0;
  t10 = in2[0] * in2[3] * 2.0;
  t11 = in2[1] * in2[1] * 2.0;
  t12 = in2[2] * in2[2] * 2.0;
  t13 = in2[3] * in2[3] * 2.0;
  t17 = t5 + t10;
  t18 = t6 + t9;
  t19 = t7 + t8;
  t20 = t5 + -t10;
  t21 = t6 + -t9;
  t6 = t7 + -t8;
  t9 = (t11 + t12) - 1.0;
  t10 = (t11 + t13) - 1.0;
  t5 = (t12 + t13) - 1.0;
  out1[0] = (((((in1[0] - in3[0] * t5) + in3[1] * t20) + in3[2] * t18) -
              t5 * in4[0]) +
             t20 * in4[1]) +
            t18 * in4[2];
  out1[1] = (((((in1[1] + in3[0] * t17) - in3[1] * t10) + in3[2] * t6) +
              t17 * in4[0]) -
             t10 * in4[1]) +
            t6 * in4[2];
  out1[2] = (((((in1[2] + in3[0] * t21) + in3[1] * t19) - in3[2] * t9) +
              t21 * in4[0]) +
             t19 * in4[1]) -
            t9 * in4[2];
  out1[3] = 1.0;
}

//
// File trailer for sonar2NED.cpp
//
// [EOF]
//
