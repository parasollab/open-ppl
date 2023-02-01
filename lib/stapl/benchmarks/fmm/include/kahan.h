/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARKS_FMM_KAHAN_H
#define STAPL_BENCHMARKS_FMM_KAHAN_H

#include <iostream>
#ifndef __CUDACC__
#define __host__
#define __device__
#define __forceinline__
#endif
//! Operator overloading for Kahan summation
template<typename T>
struct kahan
{
  T s;
  T c;
  __host__ __device__ __forceinline__
  // Default constructor
  kahan(){}
  __host__ __device__ __forceinline__
  // Copy constructor (scalar)
  kahan(T const& v)
  {
    s = v;
    c = 0;
  }
  __host__ __device__ __forceinline__
  // Copy constructor (structure)
  kahan(kahan const& v)
  {
    s = v.s;
    c = v.c;
  }
  __host__ __device__ __forceinline__
  // Destructor
  ~kahan(){}
  __host__ __device__ __forceinline__
  // Scalar assignment
  kahan const& operator=(const T v)
  {
    s = v;
    c = 0;
    return *this;
  }
  __host__ __device__ __forceinline__
  // Scalar compound assignment (add)
  kahan const& operator+=(const T v)
  {
    T y = v - c;
    T t = s + y;
    c = (t - s) - y;
    s = t;
    return *this;
  }
  __host__ __device__ __forceinline__
  // Scalar compound assignment (subtract)
  kahan const& operator-=(const T v)
  {
    T y = - v - c;
    T t = s + y;
    c = (t - s) - y;
    s = t;
    return *this;
  }
  __host__ __device__ __forceinline__
  // Scalar compound assignment (multiply)
  kahan const& operator*=(const T v)
  {
    c *= v;
    s *= v;
    return *this;
  }
  __host__ __device__ __forceinline__
  // Scalar compound assignment (divide)
  kahan const& operator/=(const T v)
  {
    c /= v;
    s /= v;
    return *this;
  }
  __host__ __device__ __forceinline__
  // Vector assignment
  kahan const& operator=(kahan const& v)
  {
    s = v.s;
    c = v.c;
    return *this;
  }
  __host__ __device__ __forceinline__
  // Vector compound assignment (add)
  kahan const& operator+=(kahan const& v)
  {
    T y = v.s - c;
    T t = s + y;
    c = (t - s) - y;
    s = t;
    y = v.c - c;
    t = s + y;
    c = (t - s) - y;
    s = t;
    return *this;
  }
  __host__ __device__ __forceinline__
  // Vector compound assignment (subtract)
  kahan const& operator-=(kahan const& v)
  {
    T y = - v.s - c;
    T t = s + y;
    c = (t - s) - y;
    s = t;
    y = - v.c - c;
    t = s + y;
    c = (t - s) - y;
    s = t;
    return *this;
  }
  __host__ __device__ __forceinline__
  // Vector compound assignment (multiply)
  kahan const& operator*=(kahan const& v)
  {
    c *= (v.c + v.s);
    s *= (v.c + v.s);
    return *this;
  }
  __host__ __device__ __forceinline__
  // Vector compound assignment (divide)
  kahan const& operator/=(kahan const& v)
  {
    c /= (v.c + v.s);
    s /= (v.c + v.s);
    return *this;
  }
  __host__ __device__ __forceinline__
  // Vector arithmetic (negation)
  kahan operator-() const
  {
    kahan temp;
    temp.s = -s;
    temp.c = -c;
    return temp;
  }
  __host__ __device__ __forceinline__
  // Type-casting (lvalue)
  operator       T ()       {return s+c;}
  __host__ __device__ __forceinline__
  // Type-casting (rvalue)
  operator const T () const {return s+c;}
  // Output stream
  friend std::ostream &operator<<(std::ostream & s, kahan const& v)
  {
    s << (v.s + v.c);
    return s;
  }
  // Input stream
  friend std::istream &operator>>(std::istream & s, kahan & v)
  {
    s >> v.s;
    v.c = 0;
    return s;
  }
};

#endif // STAPL_BENCHMARKS_FMM_KAHAN_H
