/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


//////////////////////////////////////////////////////////////////////
/// @file
/// Unit test for @ref stapl::function marshaling.
//////////////////////////////////////////////////////////////////////

#define STAPL_RUNTIME_TEST_MODULE function
#include "utility.h"
#include <stapl/runtime/function.hpp>
#include <stapl/runtime/request/arg_storage.hpp>
#include <algorithm>
#include <numeric>
#include <functional>
#include <type_traits>
#include <vector>
#include <boost/optional.hpp>

using stapl::function;
using namespace stapl::runtime;

int foo(void)
{ return 42; }

int foo_1(int i)
{ return i; }

int foo_7(int i, int j, int k, int l, int m, int n, int o)
{ return (i+j+k+l+m+n+o); }

int vec_sum(std::vector<int> const& v)
{ return std::accumulate(v.begin(), v.end(), 0); }


BOOST_AUTO_TEST_CASE( empty_function )
{
  typedef function<void(void)>                        function_type;
  typedef arg_storage_t<function_type, function_type> arg_storage_type;

  function_type f;

  // find size
  const std::size_t static_size = sizeof(arg_storage_type);
  BOOST_CHECK_EQUAL( static_size, aligned_size(static_size) );
  const std::size_t sz = static_size + arg_storage_type::packed_size(f);
  BOOST_CHECK_EQUAL( sz, aligned_size(sz) );

  // make buffer
  buffer buf{sz};
  void* p = buf.address();

  // pack
  std::size_t size = static_size;
  arg_storage_type* const a = new(p) arg_storage_type{f, p, size};
  BOOST_CHECK( !buf.overwritten() );
  BOOST_CHECK_EQUAL( size, sz );
  BOOST_CHECK_EQUAL( static_size, sz );

  // unpack
  std::size_t size2 = static_size;
  BOOST_CHECK( !(a->get(p, size2)) );
  BOOST_CHECK( !buf.overwritten() );
  BOOST_CHECK_EQUAL( size2, sz );

  // destroy
  a->~arg_storage_type();
  BOOST_CHECK( !buf.overwritten() );
}


BOOST_AUTO_TEST_CASE( function_ptr )
{
  typedef function<int(void)>                         function_type;
  typedef arg_storage_t<function_type, function_type> arg_storage_type;

  function_type f{foo};
  boost::optional<function_type> o = f;

  // find size
  const std::size_t static_size = sizeof(arg_storage_type);
  BOOST_CHECK_EQUAL( static_size, aligned_size(static_size) );
  const std::size_t sz = static_size + arg_storage_type::packed_size(*o);
  BOOST_CHECK_EQUAL( sz, aligned_size(sz) );

  // make buffer
  buffer buf{sz};
  void* p = buf.address();

  // pack
  std::size_t size = static_size;
  arg_storage_type* const a = new(p) arg_storage_type{*o, p, size};
  BOOST_CHECK( !buf.overwritten() );
  BOOST_CHECK_EQUAL( size, sz );

  // clear o
  o = boost::none;

  // unpack
  std::size_t size2 = static_size;
  BOOST_CHECK( a->get(p, size2)()==f() );
  BOOST_CHECK( !buf.overwritten() );
  BOOST_CHECK_EQUAL( size2, sz );

  // destroy
  a->~arg_storage_type();
  BOOST_CHECK( !buf.overwritten() );
}


BOOST_AUTO_TEST_CASE( function_boost_bind )
{
  typedef function<int(void)>                         function_type;
  typedef arg_storage_t<function_type, function_type> arg_storage_type;

  function_type f{std::bind(foo_1, 42)};
  boost::optional<function_type> o = f;

  // find size
  const std::size_t static_size = sizeof(arg_storage_type);
  BOOST_CHECK_EQUAL( static_size, aligned_size(static_size) );
  const std::size_t sz = static_size + arg_storage_type::packed_size(*o);
  BOOST_CHECK_EQUAL( sz, aligned_size(sz) );

  // make buffer
  buffer buf{sz};
  void* p = buf.address();

  // pack
  std::size_t size = static_size;
  arg_storage_type* const a = new(p) arg_storage_type{*o, p, size};
  BOOST_CHECK( !buf.overwritten() );
  BOOST_CHECK_EQUAL( size, sz );

  // clear o
  o = boost::none;

  // unpack
  std::size_t size2 = static_size;
  BOOST_CHECK( a->get(p, size2)()==f() );
  BOOST_CHECK( !buf.overwritten() );
  BOOST_CHECK_EQUAL( size2, sz );

  // destroy
  a->~arg_storage_type();
  BOOST_CHECK( !buf.overwritten() );
}


BOOST_AUTO_TEST_CASE( function_std_bind )
{
  typedef function<int(void)>                         function_type;
  typedef arg_storage_t<function_type, function_type> arg_storage_type;

  function_type f{std::bind(foo_1, 42)};
  boost::optional<function_type> o = f;

  // find size
  const std::size_t static_size = sizeof(arg_storage_type);
  BOOST_CHECK_EQUAL( static_size, aligned_size(static_size) );
  const std::size_t sz = static_size + arg_storage_type::packed_size(*o);
  BOOST_CHECK_EQUAL( sz, aligned_size(sz) );

  // make buffer
  buffer buf{sz};
  void* p = buf.address();

  // pack
  std::size_t size = static_size;
  arg_storage_type* const a = new(p) arg_storage_type{*o, p, size};
  BOOST_CHECK( !buf.overwritten() );
  BOOST_CHECK_EQUAL( size, sz );

  // clear o
  o = boost::none;

  // unpack
  std::size_t size2 = static_size;
  BOOST_CHECK( a->get(p, size2)()==f() );
  BOOST_CHECK( !buf.overwritten() );
  BOOST_CHECK_EQUAL( size2, sz );

  // destroy
  a->~arg_storage_type();
  BOOST_CHECK( !buf.overwritten() );
}


BOOST_AUTO_TEST_CASE( function_std_bind_7 )
{
  typedef function<int(void)>                         function_type;
  typedef arg_storage_t<function_type, function_type> arg_storage_type;

  function_type f{std::bind(foo_7, 1, 2, 3, 4, 5, 6, 7)};
  boost::optional<function_type> o = f;

  // find size
  const std::size_t static_size = sizeof(arg_storage_type);
  BOOST_CHECK_EQUAL( static_size, aligned_size(static_size) );
  const std::size_t sz = static_size + arg_storage_type::packed_size(*o);
  BOOST_CHECK_EQUAL( sz, aligned_size(sz) );

  // make buffer
  buffer buf{sz};
  void* p = buf.address();

  // pack
  std::size_t size = static_size;
  arg_storage_type* const a = new(p) arg_storage_type{*o, p, size};
  BOOST_CHECK( !buf.overwritten() );
  BOOST_CHECK_EQUAL( size, sz );

  // clear o
  o = boost::none;

  // unpack
  std::size_t size2 = static_size;
  BOOST_CHECK( a->get(p, size2)()==f() );
  BOOST_CHECK( !buf.overwritten() );
  BOOST_CHECK_EQUAL( size2, sz );

  // destroy
  a->~arg_storage_type();
  BOOST_CHECK( !buf.overwritten() );
}


BOOST_AUTO_TEST_CASE( function_boost_bind_7 )
{
  typedef function<int(void)>                         function_type;
  typedef arg_storage_t<function_type, function_type> arg_storage_type;

  function_type f{std::bind(foo_7, 1, 2, 3, 4, 5, 6, 7)};
  boost::optional<function_type> o = f;

  // find size
  const std::size_t static_size = sizeof(arg_storage_type);
  BOOST_CHECK_EQUAL( static_size, aligned_size(static_size) );
  const std::size_t sz = static_size + arg_storage_type::packed_size(*o);
  BOOST_CHECK_EQUAL( sz, aligned_size(sz) );

  // make buffer
  buffer buf{sz};
  void* p = buf.address();

  // pack
  std::size_t size = static_size;
  arg_storage_type* const a = new(p) arg_storage_type{*o, p, size};
  BOOST_CHECK( !buf.overwritten() );
  BOOST_CHECK_EQUAL( size, sz );

  // clear o
  o = boost::none;

  // unpack
  std::size_t size2 = static_size;
  BOOST_CHECK( a->get(p, size2)()==f() );
  BOOST_CHECK( !buf.overwritten() );
  BOOST_CHECK_EQUAL( size2, sz );

  // destroy
  a->~arg_storage_type();
  BOOST_CHECK( !buf.overwritten() );
}


BOOST_AUTO_TEST_CASE( function_std_bind_vec )
{
  typedef function<int(void)>                         function_type;
  typedef arg_storage_t<function_type, function_type> arg_storage_type;

  std::vector<int> v(100, -2);
  int r = vec_sum(v);

  function_type f{std::bind(vec_sum, v)};
  v.clear();
  boost::optional<function_type> o = f;

  // find size
  const std::size_t static_size = sizeof(arg_storage_type);
  BOOST_CHECK_EQUAL( static_size, aligned_size(static_size) );
  const std::size_t sz = static_size + arg_storage_type::packed_size(*o);
  BOOST_CHECK_EQUAL( sz, aligned_size(sz) );

  // make buffer
  buffer buf{sz};
  void* p = buf.address();

  // pack
  std::size_t size = static_size;
  arg_storage_type* const a = new(p) arg_storage_type{*o, p, size};
  BOOST_CHECK( !buf.overwritten() );
  BOOST_CHECK_EQUAL( size, sz );

  // clear o
  o = boost::none;

  // unpack
  std::size_t size2 = static_size;
  BOOST_CHECK( a->get(p, size2)()==r );
  BOOST_CHECK( !buf.overwritten() );
  BOOST_CHECK_EQUAL( size2, sz );

  // destroy
  a->~arg_storage_type();
  BOOST_CHECK( !buf.overwritten() );
}


BOOST_AUTO_TEST_CASE( function_boost_bind_vec )
{
  typedef function<int(void)>                         function_type;
  typedef arg_storage_t<function_type, function_type> arg_storage_type;

  std::vector<int> v(100, -2);
  int r = vec_sum(v);

  function_type f{std::bind(vec_sum, v)};
  v.clear();
  boost::optional<function_type> o = f;

  // find size
  const std::size_t static_size = sizeof(arg_storage_type);
  BOOST_CHECK_EQUAL( static_size, aligned_size(static_size) );
  const std::size_t sz = static_size + arg_storage_type::packed_size(*o);
  BOOST_CHECK_EQUAL( sz, aligned_size(sz) );

  // make buffer
  buffer buf{sz};
  void* p = buf.address();

  // pack
  std::size_t size = static_size;
  arg_storage_type* const a = new(p) arg_storage_type{*o, p, size};
  BOOST_CHECK( !buf.overwritten() );
  BOOST_CHECK_EQUAL( size, sz );

  // clear o
  o = boost::none;

  // unpack
  std::size_t size2 = static_size;
  BOOST_CHECK( a->get(p, size2)()==r );
  BOOST_CHECK( !buf.overwritten() );
  BOOST_CHECK_EQUAL( size2, sz );

  // destroy
  a->~arg_storage_type();
  BOOST_CHECK( !buf.overwritten() );
}
