/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARKS_FMM_KERNEL_H
#define STAPL_BENCHMARKS_FMM_KERNEL_H

#include <cmath>
#include "types.h"

namespace kernel {

/// P2P kernel between cells Ci and Cj
void P2P(C_iter Ci, C_iter Cj, real_t eps2, vec3 Xperiodic, bool mutual);
/// P2P kernel for cell C
void P2P(C_iter C, real_t eps2);
/// P2M kernel for cell C
void P2M(C_iter C);
/// M2M kernel for one parent cell Ci
void M2M(C_iter Ci, C_iter C0);
/// M2L kernel between cells Ci and Cj
void M2L(C_iter Ci, C_iter Cj, vec3 Xperiodic, bool mutual);
/// L2L kernel for one child cell Ci
void L2L(C_iter Ci, C_iter C0);
/// L2P kernel for cell Ci
void L2P(C_iter Ci);

};

#include "../kernels/LaplaceCartesianCPU.cc"
#include "../kernels/LaplaceP2PCPU.cc"

#endif // STAPL_BENCHMARKS_FMM_KERNEL_H
