/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARKS_FMM_TYPES_H
#define STAPL_BENCHMARKS_FMM_TYPES_H

#include "align.h"
#include <complex>
#include "kahan.h"
#include "macros.h"
#include <stdint.h>
#include <vector>
#include "vec.h"
#include "stapl_entry.h"
// Basic type definitions
#if FP64
/// Floating point type is double precision
typedef double               real_t;
/// Double precision epsilon
const real_t EPS = 1e-16;
#else
/// Floating point type is single precision
typedef float                real_t;
/// Single precision epsilon
const real_t EPS = 1e-8;
#endif
/// Complex type
typedef std::complex<real_t> complex_t;
/// Vector of 3 real_t types
typedef vec<3,real_t>        vec3;

// SIMD vector types for MIC, AVX, and SSE
/// SIMD vector length (SIMD_BYTES defined in macros.h)
const int NSIMD = SIMD_BYTES / sizeof(real_t);
/// SIMD vector type
typedef vec<NSIMD,real_t> simdvec;

// Kahan summation types (Achieves quasi-double precision using single
// precision types)
#if KAHAN
/// Floating point type with Kahan summation
typedef kahan<real_t>  kreal_t;
/// Vector of 4 floats with Kahan summaiton
typedef vec<4,kreal_t> kvec4;
/// SIMD vector type with Kahan summation
typedef kahan<simdvec> ksimdvec;
#else
/// Floating point type
typedef real_t         kreal_t;
/// Vector of 4 floating point types
typedef vec<4,real_t>  kvec4;
/// SIMD vector type
typedef simdvec        ksimdvec;
#endif

// Multipole/local expansion coefficients
/// Order of expansions
const int P = EXPANSION;
#if Cartesian
/// Number of Cartesian mutlipole/local terms
const int NTERM = P*(P+1)*(P+2)/6;
/// Multipole/local coefficient type for Cartesian
typedef vec<NTERM,real_t> vecP;
#elif Spherical
/// Number of Spherical multipole/local terms
const int NTERM = P*(P+1)/2;
/// Multipole/local coefficient type for spherical
typedef vec<NTERM,complex_t> vecP;
#endif

/// Center and radius of bounding box
struct Box
{
  /// Box center
  vec3   X;
  /// Box radius
  real_t R;
};

/// Min & max bounds of bounding box
struct Bounds
{
  /// Minimum value of coordinates
  vec3 Xmin;
  /// Maximum value of coordinates
  vec3 Xmax;
  void define_type(stapl::typer& t)
  {
    t.member(Xmin);
    t.member(Xmax);
  }
};

/// Structure of aligned source for SIMD
struct Source
{
  /// Position
  vec3   X;
  /// Scalar source values
  real_t SRC;
  void define_type(stapl::typer& t)
  {
    t.member(X);
    t.member(SRC);
  }
};

/// Structure of bodies
struct Body : public Source
{
  /// Initial body numbering for sorting back
  int      IBODY;
  /// Initial rank numbering for partitioning back
  int      IRANK;
  /// Cell index
  uint64_t ICELL;
  /// Weight for partitioning
  real_t   WEIGHT;
  /// Scalar+vector3 target values
  kvec4    TRG;
  void define_type(stapl::typer& t)
  {
    t.base<Source>(*this);
    t.member(IBODY);
    t.member(IRANK);
    t.member(ICELL);
    t.member(WEIGHT);
    t.member(TRG);
  }
};

/// Body alignment allocator
typedef AlignedAllocator<Body,SIMD_BYTES> BodyAllocator;
/// Vector of bodies
typedef stapl::lightweight_vector<Body>     Bodies;

/// Iterator of body vector
typedef Bodies::iterator                  B_iter;

/// Structure of cells
//template <typename iterator = B_iter>
struct Cell
{
  /// Index of parent cell
  int       IPARENT;
  /// Index of first child cell
  int       ICHILD;
  /// Number of child cells
  int       NCHILD;
  /// Index of first body
  int       IBODY;
  /// Number of descendant bodies
  int       NBODY;
  /// Iterator of first body
  B_iter    BODY;
  /// Cell index
  uint64_t  ICELL;
  /// Weight for partitioning
  real_t    WEIGHT;
  /// Cell center
  vec3      X;
  /// Cell radius
  real_t    R;
  /// Multipole coefficients
  vecP      M;
  /// Local coefficients
  vecP      L;
  void define_type(stapl::typer& t)
  {
    t.member(IPARENT);
    t.member(ICHILD);
    t.member(NCHILD);
    t.member(IBODY);
    t.member(NBODY);
    t.member(BODY);
    t.member(ICELL);
    t.member(WEIGHT);
    t.member(X);
    t.member(R);
    t.member(M);
    t.member(L);
  }
};

//typedef CellT<> Cell;
/// Vector of cells
typedef stapl::lightweight_vector<Cell>     Cells;
// Other possibilities for cells type
// typedef std::vector<Cell> Cells
/// Iterator of cell vector
typedef Cells::iterator   C_iter;

#endif // STAPL_BENCHMARKS_FMM_TYPES_H
