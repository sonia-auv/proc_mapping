//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: proc_mapping_data.cpp
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 01-Aug-2022 08:26:09
//

// Include Files
#include "proc_mapping_data.h"
#include "rt_nonfinite.h"
#include <string.h>

// Variable Definitions
double freq;

bool freq_not_empty;

bool newSonarMsg;

bool bundleStarted;

bool newClearBundleMsg;

bool newHydroMsg;

bool b_bundleStarted;

bool b_newClearBundleMsg;

unsigned int method;

unsigned int state;

unsigned int b_state[2];

unsigned int c_state[625];

bool isInitialized_proc_mapping{false};

//
// File trailer for proc_mapping_data.cpp
//
// [EOF]
//
