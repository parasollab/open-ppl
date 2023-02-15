/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_TEST_SERIALIZATION_UTILITY_H
#define STAPL_RUNTIME_TEST_SERIALIZATION_UTILITY_H

#define BOOST_TEST_MODULE STAPL_RUNTIME_TEST_MODULE

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-variable"
#endif

#ifdef BOOST_TEST_USE_INCLUDED
# include <boost/test/included/unit_test.hpp>
#else
# define BOOST_TEST_DYN_LINK
# include <boost/test/unit_test.hpp>
#endif

#ifdef __clang__
#pragma clang diagnostic pop
#endif

#include <cstdlib>
#include <iostream>
#include <algorithm>
#include <functional>
#include <valarray>

// Bring in required instances for Boost.Serialization
#include "../../../src/runtime/serialization.cc"


struct buffer
{
  char*       buf;
  std::size_t size;

  explicit buffer(std::size_t s)
  : buf(new char[3 * s]),
    size(s)
  {
    std::fill(buf,          buf + size,   0);
    std::fill(buf + size,   buf + 2*size, 'a');
    std::fill(buf + 2*size, buf + 3*size, 0);
  }

  ~buffer(void)
  {
    std::fill(buf, buf + 3*size, 0);
    delete[] buf;
  }

  bool overwritten(void) const noexcept
  {
    typedef std::not_equal_to<char> wf_type;
    if (std::find_if(buf, (buf + size),
                     [](char c) { return (c!=0); }) != (buf+size))
      return true;

    if ( std::find_if((buf + 2*size), (buf + 3*size),
                      [](char c) { return (c!=0); }) != (buf + 3*size) )
      return true;

    // not overwritten
    return false;
  }

  const void* address(void) const noexcept
  { return &buf[size]; }

  void* address(void) noexcept
  { return &buf[size]; }
};

namespace stapl {

namespace runtime {

void assert_fail(const char* assertion,
                 const char* file, unsigned int line, const char* function)
{
  std::cerr << assertion
            << " (file: "     << file
            << ", function: " << function
            << ", line: "     << line << ")\n";
  std::abort();
}

void assert_fail(const char* assertion, const char* function)
{
  std::cerr << assertion << " (function: " << function << ")\n";
  std::abort();
}

} // namespace runtime

} // namespace stapl


template<typename T>
void check_equal(T const& x, T const& y) noexcept
{
  BOOST_REQUIRE(x==y);
}


template<typename T1, typename T2>
void check_equal(std::pair<T1, T2> const& x,
                 std::pair<T1, T2> const& y) noexcept
{
  BOOST_CHECK_EQUAL(x.first,  y.first);
  BOOST_CHECK_EQUAL(x.second, y.second);
}

template<typename T>
void check_equal(std::valarray<T> const& x,
                 std::valarray<T> const& y) noexcept
{
  BOOST_CHECK_EQUAL(x.size(), y.size());
  for (std::size_t i = 0; i < x.size(); ++i)
    check_equal(x[i], y[i]);
}


template<typename V, typename T>
void check_equal_associative(T const& x, T const& y)
{
  BOOST_CHECK_EQUAL(x.size(), y.size());
  std::vector<V> v1{std::begin(x), std::end(x)};
  std::vector<V> v2{std::begin(y), std::end(y)};
  std::sort(v1.begin(), v1.end());
  std::sort(v2.begin(), v2.end());
  BOOST_CHECK_EQUAL(v1.size(), v2.size());
  for (typename std::vector<V>::size_type i = 0; i < v1.size(); ++i)
    check_equal(v1[i], v2[i]);
}


template<typename Key, typename T>
void check_equal(std::multimap<Key, T> const& x,
                 std::multimap<Key, T> const& y)
{ check_equal_associative<std::pair<Key, T>>(x, y); }


template<typename T>
void check_equal(std::reference_wrapper<T> const& x,
                 std::reference_wrapper<T> const& y) noexcept
{ check_equal(x.get(), y.get()); }

#endif
