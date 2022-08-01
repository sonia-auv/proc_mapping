//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: proc_mapping_data.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 01-Aug-2022 08:26:09
//

#ifndef PROC_MAPPING_DATA_H
#define PROC_MAPPING_DATA_H

// Include Files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Variable Declarations
extern double freq;
extern bool freq_not_empty;
extern bool newSonarMsg;
extern bool bundleStarted;
extern bool newClearBundleMsg;
extern bool newHydroMsg;
extern bool b_bundleStarted;
extern bool b_newClearBundleMsg;
extern unsigned int method;
extern unsigned int state;
extern unsigned int b_state[2];
extern unsigned int c_state[625];
extern bool isInitialized_proc_mapping;

#endif
//
// File trailer for proc_mapping_data.h
//
// [EOF]
//
