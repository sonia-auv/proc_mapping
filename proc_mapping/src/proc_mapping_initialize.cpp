//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: proc_mapping_initialize.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 01-Aug-2022 08:26:09
//

// Include Files
#include "proc_mapping_initialize.h"
#include "CoderTimeAPI.h"
#include "PointCloudBundler.h"
#include "SoundCloudBundler.h"
#include "eml_rand.h"
#include "eml_rand_mcg16807_stateful.h"
#include "eml_rand_mt19937ar_stateful.h"
#include "eml_rand_shr3cong_stateful.h"
#include "proc_mapping_data.h"
#include "rt_nonfinite.h"
#include <string.h>

// Function Definitions
//
// Arguments    : void
// Return Type  : void
//
void proc_mapping_initialize()
{
  freq_not_empty_init();
  PointCloudBundler::persistentDataStore_init();
  SoundCloudBundler::persistentDataStore_init();
  eml_rand_init();
  eml_rand_mcg16807_stateful_init();
  eml_rand_shr3cong_stateful_init();
  eml_rand_mt19937ar_stateful_init();
  isInitialized_proc_mapping = true;
}

//
// File trailer for proc_mapping_initialize.cpp
//
// [EOF]
//
