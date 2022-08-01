//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: rosReadXYZ.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 01-Aug-2022 08:26:09
//

#ifndef ROSREADXYZ_H
#define ROSREADXYZ_H

// Include Files
#include "proc_mapping_types.h"
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
void rosReadXYZ(
    unsigned int msg_Height, unsigned int msg_Width,
    const ::coder::array<sensor_msgs_PointFieldStruct_T, 1U> &msg_Fields,
    unsigned int msg_PointStep,
    const ::coder::array<unsigned char, 1U> &msg_Data,
    ::coder::array<float, 2U> &xyz);

}

#endif
//
// File trailer for rosReadXYZ.h
//
// [EOF]
//
