/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARKS_FMM_MACROS_H
#define STAPL_BENCHMARKS_FMM_MACROS_H

// Disable a few Intel compiler warnings
#ifdef __INTEL_COMPILER
#pragma warning(disable:161 193 383 444 981 1572 2259)
#endif

// Override assertion
#if ASSERT
#include <cassert>
#else
#define assert(x)
#endif

// Detect SIMD Byte length of architecture
#if __MIC__
/// SIMD byte length of MIC
const int SIMD_BYTES = 64;
#elif __AVX__ | __bgq__
/// SIMD byte length of AVX and BG/Q
const int SIMD_BYTES = 32;
#elif __SSE__ | __bgp__ | __sparc_v9__
/// SIMD byte length of SSE and BG/P
const int SIMD_BYTES = 16;
#else
#error no SIMD
#endif

// Bluegene/Q and K computer don't have single precision arithmetic
#if __bgp__ | __bgq__ | __sparc_v9__
#ifndef FP64
#error Please use FP64 for BG/P, BG/Q, and K computer
#endif
#endif

#endif // STAPL_BENCHMARKS_FMM_MACROS_H
