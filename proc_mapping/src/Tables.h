//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: Tables.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 01-Aug-2022 08:26:09
//

#ifndef TABLES_H
#define TABLES_H

// Include Files
#include "proc_mapping_internal_types.h"
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace coder {
class pointCloud;

}
struct sonia_common_ObstacleInfoStruct_T;

// Type Definitions
class Tables {
public:
  void SegementByAtribute(sonia_common_ObstacleInfoStruct_T *feature) const;
  coder::pointCloud *filteredPT;
  i_struct_T param;
};

#endif
//
// File trailer for Tables.h
//
// [EOF]
//
