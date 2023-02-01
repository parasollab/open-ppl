/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARKS_FMM_UNROLL_H
#define STAPL_BENCHMARKS_FMM_UNROLL_H

namespace Ops {
  template<typename T>
  struct Assign
  {
    __host__ __device__ __forceinline__
    const T operator() (T & lhs, T const& rhs) const
    {
      return lhs = rhs;
    }
  };
  template<typename T>
  struct Add
  {
    __host__ __device__ __forceinline__
    const T operator() (T & lhs, T const& rhs) const
    {
      return lhs += rhs;
    }
  };
  template<typename T>
  struct Sub
  {
    __host__ __device__ __forceinline__
    const T operator() (T & lhs, T const& rhs) const
    {
      return lhs -= rhs;
    }
  };
  template<typename T>
  struct Mul
  {
    __host__ __device__ __forceinline__
    const T operator() (T & lhs, T const& rhs) const
    {
      return lhs *= rhs;
    }
  };
  template<typename T>
  struct Div
  {
    __host__ __device__ __forceinline__
    const T operator() (T & lhs, T const& rhs) const
    {
      return lhs /= rhs;
    }
  };
  template<typename T>
  struct Gt
  {
    __host__ __device__ __forceinline__
    bool operator() (T & lhs, T const& rhs) const
    {
      return lhs >= rhs;
    }
  };
  template<typename T>
  struct Lt
  {
    __host__ __device__ __forceinline__
    bool operator() (T & lhs, T const& rhs) const
    {
      return lhs <= rhs;
    }
  };
  template<typename T>
  struct And
  {
    __host__ __device__ __forceinline__
    int operator() (int & lhs, int const& rhs) const
    {
      return lhs &= rhs;
    }
  };
  template<typename T>
  struct Or
  {
    __host__ __device__ __forceinline__
    int operator() (int & lhs, int const& rhs) const
    {
      return lhs |= rhs;
    }
  };
  template<typename T>
  struct Negate
  {
    __host__ __device__ __forceinline__
    T operator() (T & lhs, T const& rhs) const
    {
      return lhs = -rhs;
    }
  };
  template<typename T>
  struct Min
  {
    __host__ __device__ __forceinline__
    T operator() (T & lhs, T const& rhs) const
    {
      return lhs < rhs ? lhs : rhs;
    }
  };
  template<typename T>
  struct Max
  {
    __host__ __device__ __forceinline__
    T operator() (T & lhs, T const& rhs) const
    {
      return lhs > rhs ? lhs : rhs;
    }
  };
  template<typename T>
  struct Abs
  {
    __host__ __device__ __forceinline__
    T operator() (T & lhs, T const& rhs) const
    {
      return lhs = fabsf(rhs);
    }
  };
  template<typename T>
  struct Rsqrt
  {
    __host__ __device__ __forceinline__
    T operator() (T & lhs, T const& rhs) const
    {
      return lhs = rsqrtf(rhs);
    }
  };
}

template<typename Op, typename T, int N>
struct Unroll
{
  __host__ __device__ __forceinline__
  static void loop(T * lhs, const T * rhs)
  {
    Op operation;
    Unroll<Op,T,N-1>::loop(lhs, rhs);
    operation(lhs[N-1], rhs[N-1]);
  }
  __host__ __device__ __forceinline__
  static void loop(T * lhs, const T rhs)
  {
    Op operation;
    Unroll<Op,T,N-1>::loop(lhs, rhs);
    operation(lhs[N-1], rhs);
  }
  __host__ __device__ __forceinline__
  static const T reduce(const T * val)
  {
    Op operation;
    return operation(const_cast<T*>(val)[N-1], Unroll<Op,T,N-1>::reduce(val));
  }
};

template<typename Op, typename T>
struct Unroll<Op,T,1>
{
  __host__ __device__ __forceinline__
  static void loop(T * lhs, const T * rhs)
  {
    Op operation;
    operation(lhs[0], rhs[0]);
  }
  __host__ __device__ __forceinline__
  static void loop(T * lhs, const T rhs)
  {
    Op operation;
    operation(lhs[0], rhs);
  }
  __host__ __device__ __forceinline__
  static const T reduce(const T * val)
  {
    return val[0];
  }
};

#endif // STAPL_BENCHMARKS_FMM_UNROLL_H
