//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: eml_rand_shr3cong_stateful.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 31-Jul-2022 13:03:34
//

// Include Files
#include "eml_rand_shr3cong_stateful.h"
#include "proc_mapping_data.h"
#include "rt_nonfinite.h"
#include <string.h>

// Function Definitions
//
// Arguments    : void
// Return Type  : void
//
void eml_rand_shr3cong_stateful_init()
{
  b_state[0] = 362436069U;
  b_state[1] = 521288629U;
}

//
// File trailer for eml_rand_shr3cong_stateful.cpp
//
// [EOF]
//
