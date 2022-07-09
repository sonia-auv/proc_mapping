//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// startsWith.cpp
//
// Code generation for function 'startsWith'
//

// Include files
#include "startsWith.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <string.h>

// Function Definitions
namespace coder {
bool startsWith(const ::coder::array<char, 2U> &str)
{
  static const char b_cv[2]{'~', '/'};
  int endPos;
  int matchPos;
  if (str.size(1) <= 2) {
    endPos = str.size(1) - 2;
  } else {
    endPos = 0;
  }
  matchPos = 0;
  int exitg1;
  do {
    exitg1 = 0;
    if (matchPos <= endPos) {
      matchPos = 1;
      while ((matchPos <= 2) && (str[matchPos - 1] == b_cv[matchPos - 1])) {
        matchPos++;
      }
      if (matchPos > 2) {
        matchPos = 1;
        exitg1 = 1;
      } else {
        matchPos = 1;
      }
    } else {
      matchPos = 0;
      exitg1 = 1;
    }
  } while (exitg1 == 0);
  return matchPos == 1;
}

} // namespace coder

// End of code generation (startsWith.cpp)
