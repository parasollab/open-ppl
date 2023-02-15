/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARKS_FMM_VEC_H
#define STAPL_BENCHMARKS_FMM_VEC_H

#include <cassert>
#include <ostream>
#include <stapl/runtime.hpp>
#define NEWTON 1

/// Custom vector type for small vectors 
template<int N, typename T>
class vec
{
private:
  T data[N];

public:
  // Default constructor
  vec()
  {
    for (int i=0; i<N; i++) data[i] = 0;
  }
  // Copy constructor (scalar)
  vec(T const&v)
  {
    for (int i=0; i<N; i++) data[i] = v;
  }
  // Copy constructor (vector)
  vec(vec const& v)
  {
    for (int i=0; i<N; i++) data[i] = v[i];
  }
  // Destructor
  ~vec()
  { }
  // Scalar assignment
  vec const& operator=(const T v)
  {
    for (int i=0; i<N; i++) data[i] = v;
    return *this;
  }
  // Scalar compound assignment (add)
  vec const& operator+=(const T v)
  {
    for (int i=0; i<N; i++) data[i] += v;
    return *this;
  }
  // Scalar compound assignment (subtract)
  vec const& operator-=(const T v)
  {
    for (int i=0; i<N; i++) data[i] -= v;
    return *this;
  }
  // Scalar compound assignment (multiply)
  vec const& operator*=(const T v)
  {
    for (int i=0; i<N; i++) data[i] *= v;
    return *this;
  }
  // Scalar compound assignment (divide)
  vec const& operator/=(const T v)
  {
    for (int i=0; i<N; i++) data[i] /= v;
    return *this;
  }
  // Scalar compound assignment (greater than)
  vec const& operator>=(const T v)
  {
    for (int i=0; i<N; i++) data[i] >= v;
    return *this;
  }
  // Scalar compound assignment (less than)
  vec const& operator<=(const T v)
  {
    for (int i=0; i<N; i++) data[i] <= v;
    return *this;
  }
  // Scalar compound assignment (bitwise and)
  vec const& operator&=(const T v)
  {
    for (int i=0; i<N; i++) data[i] &= v;
    return *this;
  }
  // Scalar compound assignment (bitwise or)
  vec const& operator|=(const T v)
  {
    for (int i=0; i<N; i++) data[i] |= v;
    return *this;
  }
  // Vector assignment
  vec const& operator=(vec const&  v)
  {
    for (int i=0; i<N; i++) data[i] = v[i];
    return *this;
  }
  // Vector compound assignment (add)
  vec const& operator+=(vec const&  v)
  {
    for (int i=0; i<N; i++) data[i] += v[i];
    return *this;
  }
  // Vector compound assignment (subtract)
  vec const& operator-=(vec const&  v)
  {
    for (int i=0; i<N; i++) data[i] -= v[i];
    return *this;
  }
  // Vector compound assignment (multiply)
  vec const& operator*=(vec const&  v)
  {
    for (int i=0; i<N; i++) data[i] *= v[i];
    return *this;
  }
  // Vector compound assignment (divide)
  vec const& operator/=(vec const&  v)
  {
    for (int i=0; i<N; i++) data[i] /= v[i];
    return *this;
  }
  // Vector compound assignment (greater than)
  vec const& operator>=(vec const&  v)
  {
    for (int i=0; i<N; i++) data[i] >= v[i];
    return *this;
  }
  // Vector compound assignment (less than)
  vec const& operator<=(vec const&  v)
  {
    for (int i=0; i<N; i++) data[i] <= v[i];
    return *this;
  }
  // Vector compound assignment (bitwise and)
  vec const& operator&=(vec const&  v)
  {
    for (int i=0; i<N; i++) data[i] &= v[i];
    return *this;
  }
  // Vector compound assignment (bitwise or)
  vec const& operator|=(vec const&  v)
  {
    for (int i=0; i<N; i++) data[i] |= v[i];
    return *this;
  }
  // Scalar arithmetic (add)
  vec operator+(const T v) const
  {
    return vec(*this) += v;
  }
  // Scalar arithmetic (subtract)
  vec operator-(const T v) const
  {
    return vec(*this) -= v;
  }
  // Scalar arithmetic (multiply)
  vec operator*(const T v) const
  {
    return vec(*this) *= v;
  }
  // Scalar arithmetic (divide)
  vec operator/(const T v) const
  {
    return vec(*this) /= v;
  }
  // Scalar arithmetic (greater than)
  vec operator>(const T v) const
  {
    return vec(*this) >= v;
  }
  // Scalar arithmetic (less than)
  vec operator<(const T v) const
  {
    return vec(*this) <= v;
  }
  // Scalar arithmetic (bitwise and)
  vec operator&(const T v) const
  {
    return vec(*this) &= v;
  }
  // Scalar arithmetic (bitwise or)
  vec operator|(const T v) const
  {
    return vec(*this) |= v;
  }
  // Vector arithmetic (add)
  vec operator+(vec const&  v) const
  {
    return vec(*this) += v;
  }
  // Vector arithmetic (subtract)
  vec operator-(vec const&  v) const
  {
    return vec(*this) -= v;
  }
  // Vector arithmetic (multiply)
  vec operator*(vec const&  v) const
  {
    return vec(*this) *= v;
  }
  // Vector arithmetic (divide)
  vec operator/(vec const&  v) const
  {
    return vec(*this) /= v;
  }
  // Vector arithmetic (greater than)
  vec operator>(vec const&  v) const
  {
    return vec(*this) >= v;
  }
  // Vector arithmetic (less than)
  vec operator<(vec const&  v) const
  {
    return vec(*this) <= v;
  }
  // Vector arithmetic (bitwise and)
  vec operator&(vec const&  v) const
  {
    return vec(*this) &= v;
  }
  // Vector arithmetic (bitwise or)
  vec operator|(vec const&  v) const
  {
    return vec(*this) |= v;
  }
  // Vector arithmetic (negation)
  vec operator-() const
  {
    vec temp;
    for (int i=0; i<N; i++) temp[i] = -data[i];
    return temp;
  }
  // Indexing (lvalue)
  T &operator[](int i)
  {
    return data[i];
  }
  // Indexing (rvalue)
  T const&operator[](int i) const
  {
    return data[i];
  }
  // Type-casting (lvalue)
  operator       T* ()       {return data;}
  // Type-casting (rvalue)
  operator const T* () const {return data;}

  friend std::ostream &operator<<(std::ostream & s, vec const&  v)
  {// Component-wise output stream
    for (int i=0; i<N; i++) s << v[i] << ' ';
    return s;
  }
  // Sum vector
  friend T sum(vec const&  v)
  {
    T temp = 0;
    for (int i=0; i<N; i++) temp += v[i];
    return temp;
  }
  // L2 norm squared
  friend T norm(vec const&  v)
  {
    T temp = 0;
    for (int i=0; i<N; i++) temp += v[i] * v[i];
    return temp;
  }
  // Element-wise minimum
  friend vec min(vec const&  v, vec const&  w)
  {
    vec temp;
    for (int i=0; i<N; i++) temp[i] = v[i] < w[i] ? v[i] : w[i];
    return temp;
  }
  // Element-wise maximum
  friend vec max(vec const&  v, vec const&  w)
  {
    vec temp;
    for (int i=0; i<N; i++) temp[i] = v[i] > w[i] ? v[i] : w[i];
    return temp;
  }
  // Reciprocal square root
  friend vec rsqrt(vec const&  v)
  {
    vec temp;
    for (int i=0; i<N; i++) temp[i] = 1. / std::sqrt(v[i]);
    return temp;
  }
  // Wrap around periodic boundary
  friend int wrap(vec & v, T const& w)
  {
    assert( N <= 16 );
    int iw = 0;
    for (int i=0; i<N; i++) {
      if (v[i] < -w / 2) {
        v[i] += w;
        iw |= 1 << i;
      }
      if (v[i] >  w / 2) {
        v[i] -= w;
        iw |= 1 << i;
      }
    }
    return iw;
  }
  // Undo wrap around periodic boundary
  friend void unwrap(vec & v, T const& w, int const& iw)
  {
    assert( N <=16 );
    for (int i=0; i<N; i++) {
      if ((iw >> i) & 1) v[i] += (v[i] > 0 ? -w : w);
    }
  }
  // STAPL accessor
  T& value_at(int i) {
    return data[i];
  }
  // STAPL accessor
  T const& value_at(int i) const
  {
    return data[i];
  }
  // STAPL serializer
  void define_type(stapl::typer& t)
  {
      t.member(data);
  }
};

#endif // STAPL_BENCHMARKS_FMM_VEC_H
