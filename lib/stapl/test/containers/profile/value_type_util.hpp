/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_PROFILING_VALUE_TYPE_HPP
#define STAPL_PROFILING_VALUE_TYPE_HPP

#include <iostream>
#include <stapl/algorithms/numeric.hpp>
#include <stapl/algorithms/identity_value.hpp>

////////////////////////////////////////////////////////////////////////////////
/// @brief Simple user defined type that houses a fixed-size array and provides
/// basic operations required by the profiling suite.
///
/// @tparam T type of the elements of the member array.
/// @tparam N Size of the member array.
////////////////////////////////////////////////////////////////////////////////
template<class T, size_t N>
class my_variable_type
{
  static_assert(N > 0, "Non-zero size for the member array of the user-defined "
    "class required.");

public:
  T a[N];

  //////////////////////////////////////////////////////////////////////////////
  /// @brief Initializes the elements of the member array to 0.
  //////////////////////////////////////////////////////////////////////////////
  my_variable_type(void)
  {
    for (size_t i=0; i<N; ++i)
      a[i] = 0;
  }

  //////////////////////////////////////////////////////////////////////////////
  /// @brief Initializes the elements of the member array to a value @p v.
  //////////////////////////////////////////////////////////////////////////////
  my_variable_type(size_t v)
  {
    for (size_t i=0; i<N; ++i)
      a[i] = v;
  }

  my_variable_type& operator=(size_t v)
  {
    for (size_t i=0; i<N; ++i)
      a[i] = v;
    return *this;
  }

  bool operator==(my_variable_type const& other) const
  {
    bool eql=true;
    for (size_t i=0; i<N; ++i)
    {
      if (a[i] != other.a[i])
      {
        eql = false;
        break;
      }
    }

    return eql;
  }

  void operator+=(size_t v)
  {
    for (size_t i=0; i<N; ++i)
      a[i] += v;
  }

  my_variable_type operator+(size_t v) const
  {
    my_variable_type temp;
    for (size_t i=0; i<N; ++i)
      temp.a[i] = this->a[i] + v;
    return temp;
  }

  void operator+=(my_variable_type const& v)
  {
    for (size_t i=0; i<N; ++i)
      a[i] += v.a[i];
  }

  void define_type(stapl::typer& t)
  {
    t.member(a);
  }
};

namespace stapl {

////////////////////////////////////////////////////////////////////////////////
/// @brief A proxy for the my_variable_type class.
////////////////////////////////////////////////////////////////////////////////
template <class T, size_t N, typename Accessor>
class proxy<my_variable_type<T, N>, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;
  typedef my_variable_type<T, N> target_t;

public:

  explicit proxy(Accessor const& acc)
  : Accessor(acc)
  { }

  operator target_t() const
  { return Accessor::read(); }

  proxy const& operator=(proxy const& rhs)
  { Accessor::write(rhs); return *this; }

  proxy const& operator=(target_t const& rhs)
  { Accessor::write(rhs); return *this;}

  void operator+=(size_t v)
  { Accessor::invoke(&target_t::operator+=, v); }

  void operator+=(target_t const& v)
  { Accessor::invoke(&target_t::operator+=, v); }

  target_t operator+(size_t v)
  { return Accessor::const_invoke(&target_t::operator+, v); }

}; // class proxy

} // namespace stapl

template<class T, size_t N>
std::ostream& operator<<(std::ostream& out, my_variable_type<T,N> const& mv)
{
  out << "a: array of size " << N;

  if (N < 16)
  {
    out << ", { " << mv.a[0];
    for (size_t i = 1; i < N; ++i)
      out << ", " << mv.a[i];
    out << "}";
  }

  return out << std::endl;
}

using MVT = my_variable_type<int,8>;

#endif // STAPL_PROFILING_VALUE_TYPE_HPP
